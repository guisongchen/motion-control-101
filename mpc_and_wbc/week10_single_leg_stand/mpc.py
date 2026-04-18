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

        # 线性化动力学矩阵（由外部调用 set_dynamics 更新）
        self.A_d = np.zeros((self.nx, self.nx))
        self.B_d = np.zeros((self.nx, self.nu))
        self.d_d = np.zeros(self.nx)

        self.solver = osqp.OSQP()

        # 摩擦锥
        self.A_fcon, self.b_fcon = build_friction_cone_matrix(MU)

        # 参考轨迹
        self.x_ref = np.zeros(self.nx)
        self.u_ref = np.zeros(self.nu)

        # QP 矩阵（在 _build_qp_matrices 中构建）
        self._P = None
        self._A = None
        self._l = None
        self._u = None
        self._nz = None
        self._n_constr = None

        self._build_qp_matrices()

    def _build_qp_matrices(self):
        """
        构造 MPC QP 的标准形式：
            min  0.5 * z^T P z + q^T z
            s.t. l <= A z <= u
        其中 z = [x_0, x_1, ..., x_N, u_0, u_1, ..., u_{N-1}]

        约束结构：
            - 初始状态: x_0 = x0_hat            (nx 个等式)
            - 动力学: x_{k+1} = A_d x_k + B_d u_k + d_d   (N*nx 个等式)
            - 摩擦锥: A_fcon @ u_k <= 0        (N*4 个不等式)
        """
        nx, nu, N = self.nx, self.nu, self.N
        self._nz = (N + 1) * nx + N * nu
        n_init = nx
        n_dyn = N * nx
        n_fcon = N * 4
        self._n_constr = n_init + n_dyn + n_fcon

        # -----------------------------------------------------------------
        # P: 块对角 (Q, Q, ..., QN, R, R, ..., R)
        # -----------------------------------------------------------------
        P_blocks = [Q] * N + [QN] + [R] * N
        self._P = sparse.block_diag(P_blocks, format="csc")

        # -----------------------------------------------------------------
        # A: 用 dok_matrix 构造稀疏结构，动力学部分用 0 占位
        # -----------------------------------------------------------------
        A_dok = sparse.dok_matrix((self._n_constr, self._nz))

        # 1) 初始状态约束: x_0 = x0  →  行 0:nx, 列 0:nx = I
        for i in range(nx):
            A_dok[i, i] = 1.0

        # 2) 动力学约束: x_{k+1} - A_d x_k - B_d u_k = d_d
        #    初始 A_d = 0, B_d = 0，仅保留稀疏结构占位
        for k in range(N):
            row_base = n_init + k * nx
            col_xk = k * nx
            col_xnext = (k + 1) * nx
            col_uk = (N + 1) * nx + k * nu

            for i in range(nx):
                # x_{k+1} 系数: +I
                A_dok[row_base + i, col_xnext + i] = 1.0
                # x_k 系数: -A_d[i,j]（占位 0）
                for j in range(nx):
                    A_dok[row_base + i, col_xk + j] = 0.0
                # u_k 系数: -B_d[i,j]（占位 0）
                for j in range(nu):
                    A_dok[row_base + i, col_uk + j] = 0.0

        # 3) 摩擦锥约束: A_fcon @ u_k <= 0
        for k in range(N):
            row_base = n_init + n_dyn + k * 4
            col_uk = (N + 1) * nx + k * nu
            for i in range(4):
                for j in range(nu):
                    A_dok[row_base + i, col_uk + j] = self.A_fcon[i, j]

        self._A = A_dok.tocsc()

        # -----------------------------------------------------------------
        # l, u
        # -----------------------------------------------------------------
        self._l = np.zeros(self._n_constr)
        self._u = np.zeros(self._n_constr)

        # 摩擦锥: l = -inf, u = 0
        self._l[n_init + n_dyn :] = -np.inf
        self._u[n_init + n_dyn :] = 0.0

        # 初始状态和动力学部分: l=u（等式约束），在 solve 中填入具体值

        # -----------------------------------------------------------------
        # 求解器设置
        # -----------------------------------------------------------------
        self.solver.setup(
            P=self._P,
            q=np.zeros(self._nz),
            A=self._A,
            l=self._l,
            u=self._u,
            verbose=False,
            eps_abs=1e-5,
            eps_rel=1e-5,
            max_iter=4000,
        )

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

        Args:
            x0: 当前状态估计 (nx,)

        Returns:
            result: dict，包含
                - x_traj: (N+1, nx) 状态轨迹
                - u0: (nu,) 当前步控制
                - solve_time: 求解耗时 [s]
        """
        nx, nu, N = self.nx, self.nu, self.N
        nz = self._nz
        n_init = nx
        n_dyn = N * nx

        # -----------------------------------------------------------------
        # 1. 更新一次项 q（参考轨迹偏差）
        # -----------------------------------------------------------------
        q = np.zeros(nz)
        for k in range(N):
            q[k * nx : (k + 1) * nx] = -Q @ self.x_ref
        q[N * nx : (N + 1) * nx] = -QN @ self.x_ref
        for k in range(N):
            q[(N + 1) * nx + k * nu : (N + 1) * nx + (k + 1) * nu] = -R @ self.u_ref

        # -----------------------------------------------------------------
        # 2. 更新 A 矩阵中动力学部分的数据
        # -----------------------------------------------------------------
        A_dok = self._A.todok()
        for k in range(N):
            row_base = n_init + k * nx
            col_xk = k * nx
            col_uk = (N + 1) * nx + k * nu
            for i in range(nx):
                for j in range(nx):
                    A_dok[row_base + i, col_xk + j] = -self.A_d[i, j]
                for j in range(nu):
                    A_dok[row_base + i, col_uk + j] = -self.B_d[i, j]
        A_new = A_dok.tocsc()

        # -----------------------------------------------------------------
        # 3. 更新约束边界 l, u
        # -----------------------------------------------------------------
        l = self._l.copy()
        u = self._u.copy()

        # 初始状态
        l[:nx] = x0
        u[:nx] = x0

        # 动力学右端
        for k in range(N):
            l[n_init + k * nx : n_init + (k + 1) * nx] = self.d_d
            u[n_init + k * nx : n_init + (k + 1) * nx] = self.d_d

        # -----------------------------------------------------------------
        # 4. 调用 osqp update 并求解
        # -----------------------------------------------------------------
        self.solver.update(q=q, Ax=A_new.data, l=l, u=u)
        result = self.solver.solve()

        if result.info.status_val != 1:  # 1 = OSQP_SOLVED
            return None

        z_opt = result.x
        x_traj = z_opt[: (N + 1) * nx].reshape(N + 1, nx)
        u_traj = z_opt[(N + 1) * nx :].reshape(N, nu)

        return {
            "x_traj": x_traj,
            "u0": u_traj[0],
            "solve_time": result.info.run_time,
        }
