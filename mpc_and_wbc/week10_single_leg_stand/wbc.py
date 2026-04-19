"""WBC QP 构造与求解"""

import numpy as np
import osqp
from scipy import sparse
from typing import Optional
from config import GRAVITY, MU, Kp_c, Kd_c, Kp_L, Kd_L, W1, W2, W3, W4
from utils import build_friction_cone_matrix


class WholeBodyController:
    """全身控制器：将 MPC 参考转化为关节力矩。"""

    def __init__(self, nv: int):
        """
        Args:
            nv: 广义速度维度 (n+6)
        """
        self.nv = nv
        self.nf = 3               # 单支撑只有 1 个接触点，3 维线力
        self.nz = self.nv + self.nf   # 决策变量 z = [v_dot; f]
        self.n_dof = nv - 6

        self.solver = osqp.OSQP()
        self.A_fcon, self.b_fcon = build_friction_cone_matrix(MU)
        self.last_status = "not_run"
        self.last_status_val = None

        self._build_qp_matrices()

    def _build_qp_matrices(self):
        """
        构造 WBC QP 的标准形式：
            min  0.5 * z^T P z + q^T z
            s.t. l <= A z <= u
        其中 z = [v_dot; f]
        """
        nv, nf, n_dof = self.nv, self.nf, self.n_dof
        nz = self.nz

        # 约束数
        n_slip = 3      # 无滑动：接触点线加速度为 0
        n_fcon = 4      # 摩擦锥
        n_tau = n_dof   # 力矩限幅（双边）
        n_constr = n_slip + n_fcon + n_tau

        # P, q 在 solve 中更新
        self._P = sparse.csc_matrix((nz, nz))
        self._q = np.zeros(nz)

        # A 用 dok 构造稀疏结构——必须预分配所有 solve 中会用到的非零位置，
        # 因为 OSQP update(Ax=...) 不允许超过 setup 时的 nnz 数量。
        A_dok = sparse.dok_matrix((n_constr, nz))

        # 1) 无滑动约束占位 (行 0:3, 列 0:nv)
        for i in range(3):
            for j in range(nv):
                A_dok[i, j] = 1.0

        # 2) 摩擦锥约束 (行 3:7, 列 nv:nv+nf) — 固定
        for i in range(4):
            for j in range(nf):
                A_dok[n_slip + i, nv + j] = self.A_fcon[i, j]

        # 3) 力矩限幅占位 (行 7:7+n_dof, 列 0:nv 和 nv:nz)
        for i in range(n_dof):
            for j in range(nv):
                A_dok[n_slip + n_fcon + i, j] = 1.0
            for j in range(nf):
                A_dok[n_slip + n_fcon + i, nv + j] = 1.0

        self._A = A_dok.tocsc()

        # l, u
        self._l = np.zeros(n_constr)
        self._u = np.zeros(n_constr)

        # 摩擦锥: l = -inf, u = 0
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
        """
        计算期望 CoM 加速度（PD 反馈 + 前馈）。
        """
        c_ddot_des = (c_ddot_ref
                      + Kp_c * (c_ref - c_est)
                      + Kd_c * (c_dot_ref - c_dot_est))
        return c_ddot_des

    def compute_desired_momentum_rate(self,
                                      L_ref: np.ndarray, L_est: np.ndarray,
                                      L_dot_ref: np.ndarray, L_dot_est: np.ndarray) -> np.ndarray:
        """
        计算期望角动量变化率（PD 反馈 + 前馈）。
        """
        L_dot_des = (L_dot_ref
                     + Kp_L * (L_ref - L_est)
                     + Kd_L * (L_dot_ref - L_dot_est))
        return L_dot_des

    def solve(self,
              M: np.ndarray, C: np.ndarray,
              J_c: np.ndarray, Jc_dot: np.ndarray,
              J_com: np.ndarray, J_L: np.ndarray,
              c_ddot_des: np.ndarray, L_dot_des: np.ndarray,
              f_ref: np.ndarray,
              v: np.ndarray,
              tau_min: np.ndarray, tau_max: np.ndarray) -> Optional[dict]:
        """
        求解 WBC QP，返回关节力矩。

        Returns:
            result: dict，包含
                - tau: (n_dof,) 关节力矩
                - f: (3,) 接触力
                - v_dot: (nv,) 广义加速度
                - solve_time: 求解耗时 [s]
        """
        nv, nf, n_dof = self.nv, self.nf, self.n_dof
        nz = self.nz
        n_slip = 3
        n_fcon = 4

        J_c_lin = J_c[:3, :]  # (3, nv) 线速度 Jacobian

        # -----------------------------------------------------------------
        # 1. 更新 P, q
        # -----------------------------------------------------------------
        W4_mat = W4 * np.eye(nv)
        P_vv = J_com.T @ W1 @ J_com + J_L.T @ W2 @ J_L + W4_mat
        P_ff = W3
        P = sparse.block_diag([P_vv, P_ff], format="csc")

        q_v = -(J_com.T @ W1 @ c_ddot_des + J_L.T @ W2 @ L_dot_des)
        q_f = -W3 @ f_ref
        q = np.concatenate([q_v, q_f])

        # -----------------------------------------------------------------
        # 2. 更新 A 矩阵
        # -----------------------------------------------------------------
        A_dok = self._A.todok()

        # 无滑动: J_c_lin v_dot = 0
        for i in range(3):
            for j in range(nv):
                A_dok[i, j] = J_c_lin[i, j]

        # 力矩限幅: S(M v_dot + C - J_c_lin.T @ f) in [tau_min, tau_max]
        # 等价于: (M v_dot + C - J_c_lin.T @ f)[6:] in [tau_min, tau_max]
        # A_tau = [M[6:, :], -J_c_lin.T[6:, :]]
        row_base = n_slip + n_fcon
        JcT = J_c_lin.T
        for i in range(n_dof):
            for j in range(nv):
                A_dok[row_base + i, j] = M[6 + i, j]
            for j in range(nf):
                A_dok[row_base + i, nv + j] = -JcT[6 + i, j]

        A_new = A_dok.tocsc()

        # -----------------------------------------------------------------
        # 3. 更新 l, u
        # -----------------------------------------------------------------
        l = self._l.copy()
        u = self._u.copy()

        # 无滑动等式: l = u = 0
        l[:n_slip] = 0.0
        u[:n_slip] = 0.0

        # 力矩限幅
        SC = C[6:]
        tau_ext = (JcT @ f_ref)[6:]  # 近似：用后验项中的 JcT f 的关节部分
        # 实际上约束是 M v_dot + C - J_c^T f，所以边界是 tau_min - C + J_c^T f 中的已知部分
        # 但在 OSQP 的 l <= A z <= u 形式中：
        # A_tau z = M[6:,:] v_dot - J_c^T[6:,:] f
        # 要求 tau_min <= M v_dot + C - J_c^T f <= tau_max
        # 即 tau_min - C <= M v_dot - J_c^T f <= tau_max - C
        l[row_base : row_base + n_dof] = tau_min - SC
        u[row_base : row_base + n_dof] = tau_max - SC

        # -----------------------------------------------------------------
        # 4. 求解（使用 setup 避免稀疏模式变化导致的 update 错误）
        # -----------------------------------------------------------------
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

        # 后验计算关节力矩
        # tau = S (M v_dot + C - J_c_lin.T @ f)
        tau = (M @ v_dot + C - JcT @ f)[6:]

        return {
            "tau": tau,
            "f": f,
            "v_dot": v_dot,
            "solve_time": result.info.run_time,
        }
