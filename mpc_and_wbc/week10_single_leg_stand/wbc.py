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
        self.nf = 3               # 单支撑只有 1 个接触点，3 维力
        self.nz = self.nv + self.nf   # 决策变量 z = [v_dot; f]

        self.solver = osqp.OSQP()
        self.A_fcon, self.b_fcon = build_friction_cone_matrix(MU)

        # TODO: 初始化 QP
        self._build_qp_matrices()

    def _build_qp_matrices(self):
        """
        构造 WBC QP 的标准形式：
            min  0.5 * z^T P z + q^T z
            s.t. l <= A z <= u
        其中 z = [v_dot; f]
        """
        # TODO:
        #   1. 构造目标函数权重 P, q
        #      - CoM 跟踪: ||J_c v_dot + Jc_dot v - c_ddot_des||_{W1}
        #      - 角动量跟踪: ||J_L v_dot - L_dot_des||_{W2}
        #      - 力参考: ||f - f_ref||_{W3}
        #      - 最小化加速度: ||v_dot||_{W4}
        #   2. 构造约束 A, l, u
        #      - 动力学: M v_dot + C = S^T tau + J_c^T f
        #      - 无滑动: J_c v_dot + Jc_dot v = 0
        #      - 摩擦锥: f in K
        #      - 力矩限幅: tau_min <= S(M v_dot + C - J_c^T f) <= tau_max
        pass

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
                - tau: (n,) 关节力矩
                - f: (3,) 接触力
                - v_dot: (nv,) 广义加速度
                - solve_time: 求解耗时 [s]
        """
        # TODO:
        #   1. 根据当前动力学量更新 QP 矩阵
        #   2. 调用 solver.update()
        #   3. 后验计算 tau = S (M v_dot + C - J_c^T f)
        #   4. 返回结果
        pass
