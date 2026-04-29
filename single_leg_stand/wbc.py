"""WBC QP 构造与求解"""

from typing import Optional

import numpy as np
import osqp
from scipy import sparse

from config import GRAVITY, MU, TORSIONAL_FRICTION_GAMMA, Kp_c, Kd_c, Kp_L, Kd_L, W1, W2, W3, W4
from utils import build_friction_cone_matrix


class WholeBodyController:
    """全身控制器：将 MPC 参考转化为关节力矩。"""

    def __init__(self, nv: int, num_contacts: int = 1, contact_dim: int = 4,
                 kp_c: float = None, kd_c: float = None,
                 kp_L: float = None, kd_L: float = None):
        """
        Args:
            nv: 广义速度维度 (n+6)
            num_contacts: 接触点数量
            contact_dim: 每个接触点的接触力维度。3 = [fx, fy, fz], 4 = [fx, fy, fz, mz]
            kp_c, kd_c: CoM tracking PD gains. Default from config.
            kp_L, kd_L: Angular momentum PD gains. Default from config.
        """
        if contact_dim not in (3, 4):
            raise ValueError(f"contact_dim must be 3 or 4, got {contact_dim}")
        self.nv = nv
        self.num_contacts = num_contacts
        self.contact_dim = contact_dim
        self.nf = contact_dim * self.num_contacts
        self.nz = self.nv + self.nf   # 决策变量 z = [v_dot; f]
        self.n_dof = nv - 6

        self.solver = osqp.OSQP()
        if contact_dim == 4:
            self.A_fcon, self.b_fcon = build_friction_cone_matrix(MU, TORSIONAL_FRICTION_GAMMA)
        else:
            self.A_fcon, self.b_fcon = build_friction_cone_matrix(MU)
        self.kp_c = kp_c if kp_c is not None else Kp_c
        self.kd_c = kd_c if kd_c is not None else Kd_c
        self.kp_L = kp_L if kp_L is not None else Kp_L
        self.kd_L = kd_L if kd_L is not None else Kd_L

        self.last_status = "not_run"
        self.last_status_val = None

        self._build_qp_matrices()

    def _linear_contact_jacobian(self, J_c: np.ndarray) -> np.ndarray:
        """Extract stacked linear (+ angular-z for 4D) Jacobians for each contact point."""
        expected_rows = self.contact_dim * self.num_contacts
        expected_spatial_rows = 6 * self.num_contacts
        if J_c.shape == (expected_rows, self.nv):
            return J_c
        if J_c.shape == (expected_spatial_rows, self.nv):
            blocks = []
            for contact_idx in range(self.num_contacts):
                blocks.append(J_c[6 * contact_idx : 6 * contact_idx + 3, :])
                if self.contact_dim == 4:
                    blocks.append(J_c[6 * contact_idx + 5 : 6 * contact_idx + 6, :])
            return np.vstack(blocks)
        raise ValueError(
            f"Unexpected contact Jacobian shape {J_c.shape}; expected "
            f"({expected_rows}, {self.nv}) or ({expected_spatial_rows}, {self.nv})."
        )

    def _force_weight_matrix(self) -> np.ndarray:
        """Expand the configured force regularization to match the contact count."""
        if W3.shape == (self.nf, self.nf):
            return W3
        if W3.shape == (self.contact_dim, self.contact_dim):
            return sparse.block_diag([W3] * self.num_contacts, format="csc").toarray()
        # Fallback: if W3 is 4x4 but we need 3x3, extract the top-left submatrix
        if self.contact_dim == 3 and W3.shape == (4, 4):
            w3_sub = W3[:3, :3]
            return sparse.block_diag([w3_sub] * self.num_contacts, format="csc").toarray()
        raise ValueError(
            f"W3 must be shape ({self.contact_dim}, {self.contact_dim}) or "
            f"({self.nf}, {self.nf}) for {self.num_contacts} contacts; got {W3.shape}."
        )

    def _build_qp_matrices(self):
        """
        构造 WBC QP 的标准形式：
            min  0.5 * z^T P z + q^T z
            s.t. l <= A z <= u
        其中 z = [v_dot; f]
        """
        nv, nf, n_dof = self.nv, self.nf, self.n_dof
        nz = self.nz
        cd = self.contact_dim

        n_slip = cd * self.num_contacts
        n_fcon = (6 if cd == 4 else 4) * self.num_contacts
        n_tau = n_dof
        n_constr = n_slip + n_fcon + n_tau

        self._P = sparse.csc_matrix((nz, nz))
        self._q = np.zeros(nz)

        A_dok = sparse.dok_matrix((n_constr, nz))

        for i in range(n_slip):
            for j in range(nv):
                A_dok[i, j] = 1.0

        n_fcon_ineq = 6 if cd == 4 else 4
        for contact_idx in range(self.num_contacts):
            row_offset = n_slip + n_fcon_ineq * contact_idx
            col_offset = nv + cd * contact_idx
            for i in range(n_fcon_ineq):
                for j in range(cd):
                    A_dok[row_offset + i, col_offset + j] = self.A_fcon[i, j]

        for i in range(n_dof):
            for j in range(nv):
                A_dok[n_slip + n_fcon + i, j] = 1.0
            for j in range(nf):
                A_dok[n_slip + n_fcon + i, nv + j] = 1.0

        self._A = A_dok.tocsc()

        self._l = np.zeros(n_constr)
        self._u = np.zeros(n_constr)

        self._l[n_slip : n_slip + n_fcon] = -np.inf
        self._u[n_slip : n_slip + n_fcon] = 0.0

        self.solver.setup(
            P=self._P,
            q=self._q,
            A=self._A,
            l=self._l,
            u=self._u,
            verbose=False,
            eps_abs=1e-5,
            eps_rel=1e-5,
            max_iter=4000,
        )

    def compute_desired_acceleration(self,
                                     c_ref: np.ndarray, c_est: np.ndarray,
                                     c_dot_ref: np.ndarray, c_dot_est: np.ndarray,
                                     c_ddot_ref: np.ndarray) -> np.ndarray:
        c_ddot_des = (c_ddot_ref
                      + self.kp_c * (c_ref - c_est)
                      + self.kd_c * (c_dot_ref - c_dot_est))
        return c_ddot_des

    def compute_desired_momentum_rate(self,
                                      L_ref: np.ndarray, L_est: np.ndarray,
                                      L_dot_ref: np.ndarray, L_dot_est: np.ndarray) -> np.ndarray:
        L_dot_des = (L_dot_ref
                     + self.kp_L * (L_ref - L_est)
                     + self.kd_L * (L_dot_ref - L_dot_est))
        return L_dot_des

    def solve(self,
              M: np.ndarray, C: np.ndarray,
              J_c: np.ndarray, Jc_dot: np.ndarray,
              J_com: np.ndarray, J_L: np.ndarray,
              c_ddot_des: np.ndarray, L_dot_des: np.ndarray,
              f_ref: np.ndarray,
              v: np.ndarray,
              tau_min: np.ndarray, tau_max: np.ndarray,
              force_task_matrix: Optional[np.ndarray] = None,
              force_task_ref: Optional[np.ndarray] = None,
              force_task_weight: Optional[np.ndarray] = None,
              slip_weight: float = 5000.0) -> Optional[dict]:
        nv, nf, n_dof = self.nv, self.nf, self.n_dof
        nz = self.nz
        cd = self.contact_dim
        n_slip = cd * self.num_contacts
        n_fcon = (6 if cd == 4 else 4) * self.num_contacts

        J_c_lin = self._linear_contact_jacobian(J_c)
        Jc_dot_lin = self._linear_contact_jacobian(Jc_dot)

        W4_mat = W4 * np.eye(nv)
        P_vv = J_com.T @ W1 @ J_com + J_L.T @ W2 @ J_L + W4_mat
        # Soft slip penalty: 0.5 * ||J_c * v_dot + Jc_dot * v||^2
        W_slip = slip_weight * np.eye(J_c_lin.shape[0])
        P_vv += J_c_lin.T @ W_slip @ J_c_lin
        W3_mat = self._force_weight_matrix()
        P_ff = W3_mat.copy()
        P = sparse.block_diag([P_vv, P_ff], format="csc")

        q_v = -(J_com.T @ W1 @ c_ddot_des + J_L.T @ W2 @ L_dot_des)
        q_v += J_c_lin.T @ W_slip @ (Jc_dot_lin @ v)
        q_f = -W3_mat @ f_ref
        if force_task_matrix is not None:
            if force_task_ref is None or force_task_weight is None:
                raise ValueError(
                    "force_task_ref and force_task_weight must be provided with force_task_matrix."
                )
            task_matrix = np.asarray(force_task_matrix, dtype=float)
            task_ref = np.asarray(force_task_ref, dtype=float)
            task_weight = np.asarray(force_task_weight, dtype=float)
            if task_matrix.shape[1] != self.nf:
                raise ValueError(
                    f"Force task matrix must have {self.nf} columns, got {task_matrix.shape[1]}."
                )
            if task_matrix.shape[0] != task_ref.shape[0]:
                raise ValueError(
                    f"Force task reference length {task_ref.shape[0]} does not match task rows "
                    f"{task_matrix.shape[0]}."
                )
            if task_weight.ndim == 1:
                weight_matrix = np.diag(task_weight)
            else:
                weight_matrix = task_weight
            if weight_matrix.shape != (task_matrix.shape[0], task_matrix.shape[0]):
                raise ValueError(
                    "Force task weight must be a vector or square matrix matching the task rows."
                )
            P_ff += task_matrix.T @ weight_matrix @ task_matrix
            q_f += -(task_matrix.T @ weight_matrix @ task_ref)
            P = sparse.block_diag([P_vv, P_ff], format="csc")
        q = np.concatenate([q_v, q_f])

        A_dok = self._A.todok()

        # Clear old slip constraint rows (these are now soft penalties)
        for i in range(n_slip):
            for j in range(nv):
                A_dok[i, j] = 0.0

        row_base = n_slip + n_fcon
        JcT = J_c_lin.T
        for i in range(n_dof):
            for j in range(nv):
                A_dok[row_base + i, j] = M[6 + i, j]
            for j in range(nf):
                A_dok[row_base + i, nv + j] = -JcT[6 + i, j]

        A_new = A_dok.tocsc()

        l = self._l.copy()
        u = self._u.copy()

        # Slip constraints are now soft — set bounds open
        l[:n_slip] = -np.inf
        u[:n_slip] = np.inf

        SC = C[6:]
        l[row_base : row_base + n_dof] = tau_min - SC
        u[row_base : row_base + n_dof] = tau_max - SC

        self.solver.setup(
            P=P,
            q=q,
            A=A_new,
            l=l,
            u=u,
            verbose=False,
            eps_abs=1e-5,
            eps_rel=1e-5,
            max_iter=4000,
        )
        result = self.solver.solve()
        self.last_status = result.info.status
        self.last_status_val = result.info.status_val

        if result.info.status_val != 1:
            return None

        z_opt = result.x
        v_dot = z_opt[:nv]
        f = z_opt[nv:]

        tau = (M @ v_dot + C - JcT @ f)[6:]

        return {
            "tau": tau,
            "f": f,
            "v_dot": v_dot,
            "solve_time": result.info.run_time,
            "contact_dim": self.contact_dim,
        }
