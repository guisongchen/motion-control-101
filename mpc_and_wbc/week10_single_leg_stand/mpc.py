"""Centroidal MPC QP 构造与求解"""

import numpy as np
import osqp
from scipy import sparse
from typing import Optional
from config import NX, NU, N_HORIZON, T_S, Q, R, QN, GRAVITY, MU
from utils import build_friction_cone_matrix


class CentroidalMPC:
    """基于 centroidal dynamics 的线性 MPC。"""

    def __init__(self):
        self.nx = NX
        self.nu = NU
        self.N = N_HORIZON
        self.dt = T_S

        # 线性化动力学矩阵（固定在参考点计算）
        self.A_d = np.zeros((self.nx, self.nx))
        self.B_d = np.zeros((self.nx, self.nu))
        self.d_d = np.zeros(self.nx)

        self.solver = osqp.OSQP()

        # 摩擦锥
        self.A_fcon, self.b_fcon = build_friction_cone_matrix(MU)

        # 参考轨迹
        self.x_ref = np.zeros(self.nx)
        self.u_ref = np.zeros(self.nu)

        # TODO: 初始化 QP 问题
        self._build_qp_matrices()

    def _build_qp_matrices(self):
        """
        构造 MPC QP 的标准形式：
            min  0.5 * z^T P z + q^T z
            s.t. l <= A z <= u
        其中 z = [x_0, x_1, ..., x_N, u_0, u_1, ..., u_{N-1}]
        """
        # TODO:
        #   1. 堆叠 P（块对角 Q, QN, R）
        #   2. 堆叠 q（参考轨迹偏差）
        #   3. 堆叠等式约束（初始状态 + 动力学）
        #   4. 堆叠不等式约束（摩擦锥）
        #   5. 调用 self.solver.setup(P=P, q=q, A=A, l=l, u=u)
        pass

    def set_reference(self, x_ref: np.ndarray, u_ref: np.ndarray):
        """设置参考轨迹。"""
        self.x_ref[:] = x_ref
        self.u_ref[:] = u_ref

    def set_dynamics(self, A_d: np.ndarray, B_d: np.ndarray, d_d: np.ndarray):
        """更新离散化动力学矩阵。"""
        self.A_d[:] = A_d
        self.B_d[:] = B_d
        self.d_d[:] = d_d

    def solve(self, x0: np.ndarray) -> Optional[dict]:
        """
        求解 MPC，返回最优轨迹。

        Returns:
            result: dict，包含
                - x_traj: (N+1, nx) 状态轨迹
                - u0: (nu,) 当前步控制
                - solve_time: 求解耗时 [s]
        """
        # TODO:
        #   1. 更新初始状态约束 x0
        #   2. 更新参考轨迹项 q
        #   3. 调用 self.solver.update()
        #   4. 提取结果并返回
        pass
