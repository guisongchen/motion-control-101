"""Unified Whole-Body Controller (WBC) with soft slip penalty + Jc_dot.

Cleaner version of the original wbc.py with:
- Soft slip penalty (NOT hard equality) to allow CoM mobility
- Jc_dot * v computed via finite difference for accurate slip tracking
- Unified contact interface
"""

from __future__ import annotations

from typing import Optional

import numpy as np
import osqp
from scipy import sparse

from config import GRAVITY, MU, Kp_c, Kd_c, Kp_L, Kd_L, W1, W2, W3, W4
from utils import build_friction_cone_matrix


SLIP_WEIGHT = 50000.0  # soft penalty weight for contact no-slip


class WholeBodyController:
    """Unified WBC with soft slip penalty."""

    def __init__(
        self,
        nv: int,
        n_dof: int,
        kp_c: float | None = None,
        kd_c: float | None = None,
        kp_L: float | None = None,
        kd_L: float | None = None,
    ):
        self.nv = nv
        self.n_dof = n_dof

        self.kp_c = kp_c if kp_c is not None else Kp_c
        self.kd_c = kd_c if kd_c is not None else Kd_c
        self.kp_L = kp_L if kp_L is not None else Kp_L
        self.kd_L = kd_L if kd_L is not None else Kd_L

        self.A_fcon, self.b_fcon = build_friction_cone_matrix(MU)

        self._prev_J_c: np.ndarray | None = None
        self._prev_dt: float | None = None
        self._last_n_contacts: int = -1
        self._solver: osqp.OSQP | None = None
        self.last_status: str = "not_run"
        self.last_status_val: int = -99
        self._solve_count: int = 0
        self._fail_count: int = 0

        self.MAX_ACCEL = 8.0
        self.MAX_MOMENTUM_RATE = 50.0

    def compute_desired_acceleration(
        self, c_ref, c_est, c_dot_ref, c_dot_est, c_ddot_ref,
    ) -> np.ndarray:
        c_ddot_des = c_ddot_ref + self.kp_c * (c_ref - c_est) + self.kd_c * (c_dot_ref - c_dot_est)
        return np.clip(c_ddot_des, -self.MAX_ACCEL, self.MAX_ACCEL)

    def compute_desired_momentum_rate(
        self, L_ref, L_est, L_dot_ref, L_dot_est,
    ) -> np.ndarray:
        L_dot_des = L_dot_ref + self.kp_L * (L_ref - L_est) + self.kd_L * (L_dot_ref - L_dot_est)
        return np.clip(L_dot_des, -self.MAX_MOMENTUM_RATE, self.MAX_MOMENTUM_RATE)

    def solve(
        self,
        M: np.ndarray,
        C: np.ndarray,
        J_com: np.ndarray,
        J_L: np.ndarray,
        c_ddot_des: np.ndarray,
        L_dot_des: np.ndarray,
        contact_Jc: np.ndarray,
        Jc_dot_v: np.ndarray | None,
        f_ref: np.ndarray,
        v: np.ndarray,
        tau_min: np.ndarray,
        tau_max: np.ndarray,
        dt: float,
        slip_weight: float = SLIP_WEIGHT,
    ) -> dict | None:
        """Solve WBC QP with soft slip penalty.

        Returns dict with tau, f, v_dot, solve_time or None on failure.
        """
        nv = self.nv
        n_dof = self.n_dof
        nf = len(f_ref)
        n_contacts = nf // 3
        J_c = contact_Jc

        # --- Jc_dot * v via finite difference ---
        if Jc_dot_v is not None:
            bias = np.asarray(Jc_dot_v, dtype=float)
        else:
            if self._prev_J_c is not None and self._prev_J_c.shape == J_c.shape and self._prev_dt is not None:
                Jc_dot_approx = (J_c - self._prev_J_c) / max(self._prev_dt, 1e-8)
                bias = Jc_dot_approx @ v
            else:
                bias = np.zeros(J_c.shape[0])
        self._prev_J_c = J_c.copy()
        self._prev_dt = dt

        # --- Cost: P_vv, P_ff ---
        P_vv = J_com.T @ W1 @ J_com + J_L.T @ W2 @ J_L + W4 * np.eye(nv)

        # Soft slip penalty: 0.5 * slip_weight * ||J_c * v_dot + Jc_dot * v||^2
        # = 0.5 * slip_weight * [v_dot^T J_c^T J_c v_dot + 2 v_dot^T J_c^T bias + bias^T bias]
        W_slip = slip_weight * np.eye(J_c.shape[0])
        P_vv += J_c.T @ W_slip @ J_c

        # Force regularization
        if W3.shape == (nf, nf):
            W3_mat = W3
        elif W3.shape == (3, 3):
            W3_mat = sparse.block_diag([W3] * n_contacts, format="csc").toarray()
        elif W3.shape == (4, 4):
            w3_sub = W3[:3, :3]
            W3_mat = sparse.block_diag([w3_sub] * n_contacts, format="csc").toarray()
        else:
            W3_mat = 0.5 * np.eye(nf)
        P_ff = W3_mat
        P = sparse.block_diag([P_vv, P_ff], format="csc")

        # --- Linear term q ---
        q_v = -(J_com.T @ W1 @ c_ddot_des + J_L.T @ W2 @ L_dot_des)
        q_v += J_c.T @ W_slip @ bias   # from the slip penalty cross term
        q_f = -W3_mat @ f_ref
        q = np.concatenate([q_v, q_f])

        # --- Constraints ---
        # Block 0: Friction cone  —  A_fcon * f_i <= 0  (4 rows per contact)
        # Block 1: Torque limits  —  tau_min - C_a <= M_a * v_dot - J_{c,a}^T * f <= tau_max - C_a
        n_fcon_ineq = 4 * n_contacts
        n_tau_ineq = n_dof
        n_constr = n_fcon_ineq + n_tau_ineq
        nz = nv + nf

        A_dok = sparse.dok_matrix((n_constr, nz))
        l = np.zeros(n_constr)
        u = np.zeros(n_constr)

        # Block 0: Friction cone
        for ci in range(n_contacts):
            r0 = ci * 4
            c0 = nv + ci * 3
            for i in range(4):
                for j in range(3):
                    A_dok[r0 + i, c0 + j] = self.A_fcon[i, j]
        l[:n_fcon_ineq] = -np.inf
        u[:n_fcon_ineq] = 0.0

        # Block 1: Torque limits
        row0_tau = n_fcon_ineq
        C_a = C[6:]
        M_a = M[6:, :]
        J_c_a_T = J_c[:, 6:].T

        for i in range(n_dof):
            for j in range(nv):
                A_dok[row0_tau + i, j] = M_a[i, j]
            for j in range(nf):
                A_dok[row0_tau + i, nv + j] = -J_c_a_T[i, j]
        for i in range(n_dof):
            l[row0_tau + i] = tau_min[i] - C_a[i]
            u[row0_tau + i] = tau_max[i] - C_a[i]

        # --- Setup solver ---
        A_csc = A_dok.tocsc()
        dims_changed = (self._last_n_contacts != n_contacts)
        if self._solver is None or dims_changed:
            self._solver = osqp.OSQP()
            self._last_n_contacts = n_contacts

        self._solver.setup(
            P=P, q=q, A=A_csc, l=l, u=u,
            verbose=False,
            eps_abs=1e-4,
            eps_rel=1e-4,
            max_iter=4000,
            warm_start=False,
        )

        result = self._solver.solve()
        self._solve_count += 1
        self.last_status = result.info.status
        self.last_status_val = result.info.status_val

        if self.last_status_val != 1:
            self._fail_count += 1
            return None

        z_opt = result.x
        v_dot = z_opt[:nv]
        f = z_opt[nv:]

        tau = (M @ v_dot + C - J_c.T @ f)[6:]
        return {
            "tau": tau,
            "f": f,
            "v_dot": v_dot,
            "solve_time": result.info.run_time,
        }
