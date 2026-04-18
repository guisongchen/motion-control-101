"""
Linear MPC controller for discrete-time LTI systems.

Phase 2: Dense QP formulation with OSQP solver.

Given a discrete-time linear system:
    x(k+1) = A x(k) + B u(k)

Solve at each timestep:
    min_U   sum_{k=0}^{N-1} ||x(k) - x_ref||_Q^2 + ||u(k)||_R^2 + ||x(N)||_P^2
    s.t.    u_min <= u(k) <= u_max    (optional control bounds)

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

        # ---- Build prediction matrices ----
        # X = A_pred @ x0 + B_pred @ U
        # X = [x(1); x(2); ...; x(N)]  shape (N*nx,)
        # U = [u(0); u(1); ...; u(N-1)]  shape (N*nu,)
        self.A_pred, self.B_pred = self._build_prediction_matrices()

        # ---- Build cost matrices ----
        self.H, self._Q_bar, self._R_bar = self._build_cost_matrices()

        # ---- Setup OSQP ----
        self._setup_osqp()

    def _build_prediction_matrices(self) -> tuple[np.ndarray, np.ndarray]:
        """Build A_pred (N*nx, nx) and B_pred (N*nx, N*nu)."""
        nx, nu, N = self.nx, self.nu, self.N
        A_pred = np.zeros((N * nx, nx))
        B_pred = np.zeros((N * nx, N * nu))

        # Fill row by row
        for i in range(N):
            # A^(i+1)
            A_power = np.linalg.matrix_power(self.Ad, i + 1)
            A_pred[i * nx : (i + 1) * nx, :] = A_power

            # B_pred block row i: [A^i B, A^(i-1) B, ..., B, 0, ...]
            for j in range(i + 1):
                A_pwr = np.linalg.matrix_power(self.Ad, i - j)
                B_pred[
                    i * nx : (i + 1) * nx, j * nu : (j + 1) * nu
                ] = A_pwr @ self.Bd

        return A_pred, B_pred

    def _build_cost_matrices(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Build H = 2*(B_pred^T Q_bar B_pred + R_bar).

        Returns H, Q_bar, R_bar.
        """
        nx, nu, N = self.nx, self.nu, self.N

        # Q_bar: block diag(Q, Q, ..., P)
        # First N-1 blocks are Q, last block is P
        Q_bar = np.zeros((N * nx, N * nx))
        for i in range(N - 1):
            Q_bar[i * nx : (i + 1) * nx, i * nx : (i + 1) * nx] = self.Q
        Q_bar[(N - 1) * nx : N * nx, (N - 1) * nx : N * nx] = self.P

        # R_bar: block diag(R, R, ..., R)
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

        # Constraints: control bounds on each element of U
        # OSQP form: l <= A_con z <= u
        n_vars = self.N * self.nu
        if self.u_min is not None or self.u_max is not None:
            A_con = sp.eye(n_vars, format="csc")
            l = np.full(n_vars, -np.inf, dtype=float)
            u = np.full(n_vars, np.inf, dtype=float)

            if self.u_min is not None:
                l[:] = np.tile(self.u_min, self.N)
            if self.u_max is not None:
                u[:] = np.tile(self.u_max, self.N)
        else:
            A_con = sp.csc_matrix((0, n_vars))
            l = np.array([])
            u = np.array([])

        self.solver.setup(
            P=P_sparse,
            q=q,
            A=A_con,
            l=l,
            u=u,
            verbose=False,
            polish=False,
            warm_start=True,
        )
        self._A_con = A_con
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

        # Build gradient: g = 2 * B_pred^T @ Q_bar @ (A_pred @ x0 - X_ref)
        X_ref = np.tile(x_ref, self.N)
        g = 2 * self.B_pred.T @ self._Q_bar @ (self.A_pred @ x0 - X_ref)

        # Update and solve
        self.solver.update(q=g)
        result = self.solver.solve()

        U_opt = result.x
        u0 = U_opt[: self.nu]

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
