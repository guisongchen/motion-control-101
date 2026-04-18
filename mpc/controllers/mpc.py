"""
Linear MPC controller for discrete-time LTI systems.

Phase 2/3: Dense QP formulation with OSQP solver, supporting control
bounds, control rate limits, and state constraints.

Given a discrete-time linear system:
    x(k+1) = A x(k) + B u(k)

Solve at each timestep:
    min_U   sum_{k=0}^{N-1} ||x(k) - x_ref||_Q^2 + ||u(k)||_R^2 + ||x(N)||_P^2
    s.t.    u_min <= u(k) <= u_max          (optional control bounds)
            |u(k) - u(k-1)| <= du_max      (optional control rate limits)
            lb <= C x(k) <= ub  for k in steps  (optional state constraints)

where U = [u(0); u(1); ...; u(N-1)] is the stacked control sequence.
"""

import numpy as np
import scipy.sparse as sp
import osqp


class LinearMPC:
    """
    Linear MPC using dense QP formulation and OSQP.

    Parameters
    ----------
    Ad : np.ndarray, shape (nx, nx)
        Discrete-time state matrix.
    Bd : np.ndarray, shape (nx, nu)
        Discrete-time input matrix.
    N : int
        Prediction horizon (number of control steps).
    Q : np.ndarray, shape (nx, nx)
        Stage state cost matrix.
    R : np.ndarray, shape (nu, nu)
        Stage control cost matrix.
    P : np.ndarray, shape (nx, nx) | None
        Terminal state cost matrix. If None, defaults to Q.
    u_min : float | np.ndarray | None
        Lower bound on control(s). Scalar or shape (nu,).
    u_max : float | np.ndarray | None
        Upper bound on control(s). Scalar or shape (nu,).
    du_max : float | np.ndarray | None
        Max control rate (|u(k) - u(k-1)| <= du_max). Scalar or shape (nu,).
    state_constraints : list[dict] | None
        Each dict keys:
            - 'idx'   : int — state component index to constrain.
            - 'lb'    : float — lower bound on that component.
            - 'ub'    : float — upper bound on that component.
            - 'steps' : list[int] | 'all' — prediction steps (1-indexed).
                        Default 'all' means k = 1 .. N.
    """

    def __init__(
        self,
        Ad: np.ndarray,
        Bd: np.ndarray,
        N: int,
        Q: np.ndarray,
        R: np.ndarray,
        P: np.ndarray | None = None,
        u_min: float | np.ndarray | None = None,
        u_max: float | np.ndarray | None = None,
        du_max: float | np.ndarray | None = None,
        state_constraints: list[dict] | None = None,
    ):
        self.Ad = np.asarray(Ad, dtype=float)
        self.Bd = np.asarray(Bd, dtype=float)
        self.nx, self.nu = self.Bd.shape
        self.N = N

        self.Q = np.asarray(Q, dtype=float)
        self.R = np.asarray(R, dtype=float)
        self.P = np.asarray(P, dtype=float) if P is not None else self.Q.copy()

        # Control bounds
        if u_min is not None:
            self.u_min = np.atleast_1d(np.asarray(u_min, dtype=float))
            if self.u_min.shape == (1,):
                self.u_min = np.tile(self.u_min, self.nu)
        else:
            self.u_min = None

        if u_max is not None:
            self.u_max = np.atleast_1d(np.asarray(u_max, dtype=float))
            if self.u_max.shape == (1,):
                self.u_max = np.tile(self.u_max, self.nu)
        else:
            self.u_max = None

        # Control rate limits
        if du_max is not None:
            self.du_max = np.atleast_1d(np.asarray(du_max, dtype=float))
            if self.du_max.shape == (1,):
                self.du_max = np.tile(self.du_max, self.nu)
        else:
            self.du_max = None

        # State constraints
        self.state_constraints = state_constraints or []
        for sc in self.state_constraints:
            if "steps" not in sc or sc["steps"] == "all":
                sc["steps"] = list(range(1, N + 1))
            else:
                sc["steps"] = list(sc["steps"])

        # ---- Build prediction matrices ----
        self.A_pred, self.B_pred = self._build_prediction_matrices()

        # ---- Build cost matrices ----
        self.H, self._Q_bar, self._R_bar = self._build_cost_matrices()

        # ---- Setup OSQP ----
        self._setup_osqp()

        # Track previous applied control for rate constraints
        self._u_prev = np.zeros(self.nu)

    def _build_prediction_matrices(self) -> tuple[np.ndarray, np.ndarray]:
        """Build A_pred (N*nx, nx) and B_pred (N*nx, N*nu)."""
        nx, nu, N = self.nx, self.nu, self.N
        A_pred = np.zeros((N * nx, nx))
        B_pred = np.zeros((N * nx, N * nu))

        for i in range(N):
            A_power = np.linalg.matrix_power(self.Ad, i + 1)
            A_pred[i * nx : (i + 1) * nx, :] = A_power

            for j in range(i + 1):
                A_pwr = np.linalg.matrix_power(self.Ad, i - j)
                B_pred[
                    i * nx : (i + 1) * nx, j * nu : (j + 1) * nu
                ] = A_pwr @ self.Bd

        return A_pred, B_pred

    def _build_cost_matrices(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        nx, nu, N = self.nx, self.nu, self.N

        Q_bar = np.zeros((N * nx, N * nx))
        for i in range(N - 1):
            Q_bar[i * nx : (i + 1) * nx, i * nx : (i + 1) * nx] = self.Q
        Q_bar[(N - 1) * nx : N * nx, (N - 1) * nx : N * nx] = self.P

        R_bar = np.zeros((N * nu, N * nu))
        for i in range(N):
            R_bar[i * nu : (i + 1) * nu, i * nu : (i + 1) * nu] = self.R

        H = 2 * (self.B_pred.T @ Q_bar @ self.B_pred + R_bar)
        return H, Q_bar, R_bar

    def _setup_osqp(self):
        """Initialize OSQP solver with problem structure."""
        self.solver = osqp.OSQP()

        P_sparse = sp.csc_matrix(self.H)
        q = np.zeros(self.N * self.nu)

        n_vars = self.N * self.nu
        n_ctrl_con = n_vars if (self.u_min is not None or self.u_max is not None) else 0
        n_rate_con = (self.N - 1) * self.nu if self.du_max is not None else 0
        n_state_con = sum(len(sc["steps"]) for sc in self.state_constraints)
        n_con = n_ctrl_con + n_rate_con + n_state_con

        A_con = np.zeros((n_con, n_vars))
        l = np.full(n_con, -np.inf, dtype=float)
        u = np.full(n_con, np.inf, dtype=float)

        # Control bounds
        if n_ctrl_con > 0:
            A_con[:n_vars, :] = np.eye(n_vars)
            if self.u_min is not None:
                l[:n_vars] = np.tile(self.u_min, self.N)
            if self.u_max is not None:
                u[:n_vars] = np.tile(self.u_max, self.N)

        # Rate constraints: |u(k) - u(k-1)| <= du_max for k = 1..N-1
        # u(k) - u(k-1) <= du_max  and  u(k-1) - u(k) <= du_max
        self._rate_con_start = n_ctrl_con
        if n_rate_con > 0:
            for k in range(1, self.N):
                for j in range(self.nu):
                    row = n_ctrl_con + (k - 1) * self.nu + j
                    col_prev = (k - 1) * self.nu + j
                    col_curr = k * self.nu + j
                    # u(k) - u(k-1) <= du_max
                    A_con[row, col_curr] = 1.0
                    A_con[row, col_prev] = -1.0
                    u[row] = self.du_max[j]
                    l[row] = -self.du_max[j]

        # State constraints
        self._state_con_meta = []
        row = n_ctrl_con + n_rate_con
        for sc in self.state_constraints:
            idx = sc["idx"]
            lb, ub = sc["lb"], sc["ub"]
            for step in sc["steps"]:
                flat_row = (step - 1) * self.nx + idx
                A_con[row, :] = self.B_pred[flat_row, :]
                l[row] = lb
                u[row] = ub
                self._state_con_meta.append({
                    "A_row_idx": flat_row,
                    "con_row_idx": row,
                })
                row += 1

        if n_con > 0:
            A_sparse = sp.csc_matrix(A_con)
        else:
            A_sparse = sp.csc_matrix((0, n_vars))
            l = np.array([])
            u = np.array([])

        self.solver.setup(
            P=P_sparse,
            q=q,
            A=A_sparse,
            l=l,
            u=u,
            verbose=False,
            polish=False,
            warm_start=True,
        )
        self._A_con = A_sparse
        self._l = l
        self._u = u

    def solve(self, x0: np.ndarray, x_ref: np.ndarray | None = None) -> tuple[float, dict]:
        """
        Solve MPC QP for current state x0.

        Parameters
        ----------
        x0 : np.ndarray, shape (nx,)
            Current state.
        x_ref : np.ndarray, shape (nx,) | None
            Reference state. Defaults to origin.

        Returns
        -------
        u0 : float
            First control input to apply.
        info : dict
            Contains full solution U, predicted states X, solve time, status.
        """
        x0 = np.asarray(x0, dtype=float).reshape(self.nx)
        if x_ref is None:
            x_ref = np.zeros(self.nx)
        else:
            x_ref = np.asarray(x_ref, dtype=float).reshape(self.nx)

        # Gradient
        X_ref = np.tile(x_ref, self.N)
        g = 2 * self.B_pred.T @ self._Q_bar @ (self.A_pred @ x0 - X_ref)

        # Build bound updates
        l_new = self._l.copy()
        u_new = self._u.copy()

        # Rate constraint on u(0): |u(0) - u_prev| <= du_max
        # This tightens the control bounds on the first step
        if self.du_max is not None and self.N > 0:
            for j in range(self.nu):
                idx = j  # first control element
                if self.u_min is not None:
                    l_new[idx] = max(l_new[idx], self._u_prev[j] - self.du_max[j])
                else:
                    l_new[idx] = self._u_prev[j] - self.du_max[j]
                if self.u_max is not None:
                    u_new[idx] = min(u_new[idx], self._u_prev[j] + self.du_max[j])
                else:
                    u_new[idx] = self._u_prev[j] + self.du_max[j]

        # State constraints: update affine x0 term
        if self._state_con_meta:
            for meta in self._state_con_meta:
                offset = float(self.A_pred[meta["A_row_idx"], :] @ x0)
                l_new[meta["con_row_idx"]] -= offset
                u_new[meta["con_row_idx"]] -= offset

        self.solver.update(q=g, l=l_new, u=u_new)
        result = self.solver.solve()

        U_opt = result.x
        u0 = U_opt[: self.nu]

        # Update previous control for next timestep
        self._u_prev = np.array(u0)

        # Predicted states for analysis
        X_pred = self.A_pred @ x0 + self.B_pred @ U_opt

        info = {
            "U": U_opt,
            "X_pred": X_pred,
            "solve_time_ms": result.info.solve_time * 1000,
            "status": result.info.status,
            "status_val": result.info.status_val,
            "iter": result.info.iter,
        }

        return float(u0.item()) if self.nu == 1 else u0, info

    def reset(self):
        """Reset internal state (e.g., u_prev) for a fresh simulation."""
        self._u_prev = np.zeros(self.nu)

    def print_matrices(self):
        """Print MPC matrices for verification."""
        np.set_printoptions(precision=4, suppress=True)
        print("=== Linear MPC Matrices ===")
        print(f"nx={self.nx}, nu={self.nu}, N={self.N}")
        print(f"\nA_pred ({self.A_pred.shape}):\n{self.A_pred}")
        print(f"\nB_pred ({self.B_pred.shape}):\n{self.B_pred}")
        print(f"\nH ({self.H.shape}):\n{self.H}")
        print("=============================")


class CartPoleMPC(LinearMPC):
    """
    Convenience wrapper: LinearMPC pre-configured for cart-pole dimensions.

    Accepts the same parameters as CartPoleLQR for easy comparison.
    """

    def __init__(
        self,
        Ad: np.ndarray,
        Bd: np.ndarray,
        N: int = 20,
        Q: np.ndarray | None = None,
        R: np.ndarray | None = None,
        P: np.ndarray | None = None,
        u_min: float | None = None,
        u_max: float | None = None,
        du_max: float | None = None,
        state_constraints: list[dict] | None = None,
    ):
        if Q is None:
            Q = np.diag([1.0, 0.1, 10.0, 0.1])
        if R is None:
            R = np.array([[1.0]])
        else:
            R = np.atleast_2d(np.asarray(R, dtype=float))

        # Default terminal cost to discrete-time ARE solution for sensible baseline
        if P is None:
            from scipy.linalg import solve_discrete_are

            P = solve_discrete_are(Ad, Bd, Q, R)

        super().__init__(
            Ad=Ad,
            Bd=Bd,
            N=N,
            Q=Q,
            R=R,
            P=P,
            u_min=u_min,
            u_max=u_max,
            du_max=du_max,
            state_constraints=state_constraints,
        )


if __name__ == "__main__":
    # Quick sanity check with cart-pole matrices
    from mpc.controllers.lqr import CartPoleLQR

    lqr = CartPoleLQR()
    mpc = CartPoleMPC(Ad=lqr.Ad, Bd=lqr.Bd, N=20)
    mpc.print_matrices()

    x0 = np.array([0.0, 0.0, 0.1, 0.0])
    u0, info = mpc.solve(x0)
    print(f"\nMPC control at theta=0.1 rad: {u0:.4f} N")
    print(f"  LQR control at same state:  {lqr.control(x0):.4f} N")
    print(f"  OSQP status: {info['status']} (val={info['status_val']})")
    print(f"  Solve time: {info['solve_time_ms']:.3f} ms")
