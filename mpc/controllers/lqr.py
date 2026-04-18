"""
LQR controller for cart-pole balancing.

Phase 1: Linearize cart-pole dynamics around upright equilibrium,
discretize, and solve discrete-time Algebraic Riccati Equation (ARE).
"""

import numpy as np
from scipy.linalg import solve_discrete_are, expm
from scipy.signal import cont2discrete


class CartPoleLQR:
    """
    Infinite-horizon LQR for cart-pole around upright (theta = 0).

    Parameters
    ----------
    mc : float
        Cart mass [kg].
    mp : float
        Pole (point) mass [kg].
    l : float
        Distance from pivot to pole center of mass [m].
    g : float
        Gravity [m/s^2].
    dt : float
        Control / discretization period [s].
    Q : np.ndarray, shape (4, 4)
        State cost matrix.
    R : np.ndarray, shape (1, 1) or float
        Control cost scalar / matrix.
    """

    def __init__(
        self,
        mc: float = 1.0,
        mp: float = 0.1,
        l: float = 0.5,
        g: float = 9.81,
        dt: float = 0.01,
        Q: np.ndarray | None = None,
        R: np.ndarray | None = None,
    ):
        self.mc = mc
        self.mp = mp
        self.l = l
        self.g = g
        self.dt = dt

        # Default weights from task.md
        if Q is None:
            Q = np.diag([1.0, 0.1, 10.0, 0.1])
        if R is None:
            R = np.array([[1.0]])
        else:
            R = np.atleast_2d(np.asarray(R, dtype=float))

        self.Q = np.asarray(Q, dtype=float)
        self.R = R

        # 1. Continuous-time linearization around upright
        self.Ac, self.Bc = self._linearize_continuous()

        # 2. Zero-Order Hold (ZOH) discretization
        self.Ad, self.Bd = self._discretize_zoh()

        # 3. Solve discrete-time ARE: P = A^T P A - A^T P B (R + B^T P B)^{-1} B^T P A + Q
        self.P = solve_discrete_are(self.Ad, self.Bd, self.Q, self.R)

        # 4. LQR gain: K = (R + B^T P B)^{-1} B^T P A
        self.K = np.linalg.solve(
            self.R + self.Bd.T @ self.P @ self.Bd,
            self.Bd.T @ self.P @ self.Ad,
        )

    def _linearize_continuous(self) -> tuple[np.ndarray, np.ndarray]:
        """
        Derive continuous-time A_c and B_c for the cart-pole.

        State: x = [x, x_dot, theta, theta_dot]^T
        Control: u = F (force on cart, positive -> +x)

        Linearized dynamics (small angle approx., theta = 0 at upright):
            x_dot = A_c x + B_c u

        Derivation for point-mass pendulum (mass mp at distance l):
            (mc + mp) * x_ddot + mp * l * theta_ddot = F
            mp * l * x_ddot + mp * l^2 * theta_ddot = mp * g * l * theta

        Solving:
            x_ddot     =  F/mc - (mp*g/mc) * theta
            theta_ddot = -F/(mc*l) + ((mc+mp)*g/(mc*l)) * theta
        """
        mc, mp, l, g = self.mc, self.mp, self.l, self.g

        Ac = np.array(
            [
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, -mp * g / mc, 0.0],
                [0.0, 0.0, 0.0, 1.0],
                [0.0, 0.0, (mc + mp) * g / (mc * l), 0.0],
            ]
        )

        Bc = np.array(
            [
                [0.0],
                [1.0 / mc],
                [0.0],
                [-1.0 / (mc * l)],
            ]
        )

        return Ac, Bc

    def _discretize_zoh(self) -> tuple[np.ndarray, np.ndarray]:
        """Zero-Order Hold discretization using scipy."""
        # cont2discrete expects (A, B, C, D), returns (Ad, Bd, Cd, Dd, dt)
        n = self.Ac.shape[0]
        C = np.zeros((1, n))
        D = np.zeros((1, 1))
        sysd = cont2discrete((self.Ac, self.Bc, C, D), self.dt, method="zoh")
        Ad, Bd = sysd[0], sysd[1]
        return Ad, Bd

    def control(self, state: np.ndarray) -> float:
        """
        Compute LQR control: u = -K @ (state - state_ref).

        Parameters
        ----------
        state : np.ndarray, shape (4,)
            [x, x_dot, theta, theta_dot].
            theta = 0 is upright.

        Returns
        -------
        u : float
            Control force [N].
        """
        state = np.asarray(state).reshape(4)
        x_err = state  # reference is origin (upright)
        u = -self.K @ x_err
        return float(u.item())

    def stage_cost(self, state: np.ndarray, u: float) -> float:
        """Quadratic stage cost x^T Q x + u^T R u."""
        state = np.asarray(state).reshape(4)
        u_vec = np.atleast_1d(u)
        return float(state @ self.Q @ state + u_vec @ self.R @ u_vec)

    def print_matrices(self):
        """Print derived matrices for verification."""
        np.set_printoptions(precision=4, suppress=True)
        print("=== Cart-Pole LQR Matrices ===")
        print(f"Parameters: mc={self.mc}, mp={self.mp}, l={self.l}, g={self.g}, dt={self.dt}")
        print(f"\nA_c (continuous):\n{self.Ac}")
        print(f"\nB_c (continuous):\n{self.Bc}")
        print(f"\nA_d (discrete):\n{self.Ad}")
        print(f"\nB_d (discrete):\n{self.Bd}")
        print(f"\nQ:\n{self.Q}")
        print(f"\nR:\n{self.R}")
        print(f"\nP (ARE solution):\n{self.P}")
        print(f"\nK (LQR gain): {self.K.flatten()}")
        print("================================")


if __name__ == "__main__":
    # Quick sanity check
    ctrl = CartPoleLQR()
    ctrl.print_matrices()
    print("\nSanity: control at theta=0.1 rad:", ctrl.control(np.array([0.0, 0.0, 0.1, 0.0])))
