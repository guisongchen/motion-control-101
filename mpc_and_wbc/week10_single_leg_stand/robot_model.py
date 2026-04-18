"""URDF 加载，Jacobians，动力学量计算"""

import numpy as np
import pybullet as p
from typing import Optional


def _skew(v: np.ndarray) -> np.ndarray:
    """向量叉乘矩阵。"""
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ])


class RobotModel:
    """封装 PyBullet 机器人模型的运动学与动力学接口。"""

    def __init__(self, urdf_path: str, base_position: Optional[np.ndarray] = None,
                 base_orientation: Optional[np.ndarray] = None,
                 use_fixed_base: bool = False):
        """
        加载 URDF 并初始化模型。

        Args:
            urdf_path: URDF 文件路径
            base_position: 初始基座位置 [x, y, z]
            base_orientation: 初始基座姿态四元数 [x, y, z, w]
            use_fixed_base: 是否固定基座（仅用于测试，正常浮动基应为 False）
        """
        self.urdf_path = urdf_path
        if base_position is None:
            base_position = [0.0, 0.0, 1.0]
        if base_orientation is None:
            base_orientation = [0.0, 0.0, 0.0, 1.0]

        self.robot_id = p.loadURDF(
            urdf_path,
            basePosition=base_position,
            baseOrientation=base_orientation,
            useFixedBase=use_fixed_base,
        )
        self.num_joints = p.getNumJoints(self.robot_id)

        # 解析关节信息，区分固定关节与自由度关节
        self.joint_info = [p.getJointInfo(self.robot_id, i) for i in range(self.num_joints)]
        self.dof_joints = [
            i for i, info in enumerate(self.joint_info)
            if info[2] != p.JOINT_FIXED
        ]
        # link 名 -> link 索引映射（基座为 -1）
        self.link_name_to_index = {
            info[12].decode('utf-8'): i
            for i, info in enumerate(self.joint_info)
        }
        self.link_name_to_index['base'] = -1
        self.nv = 6 + len(self.dof_joints)          # 广义速度维度
        self.nq = 7 + self.num_joints               # 广义位置维度（含所有关节）

        # 计算并缓存总质量
        self._total_mass = sum(
            p.getDynamicsInfo(self.robot_id, i)[0]
            for i in range(-1, self.num_joints)
        )
        if self._total_mass <= 0.0:
            raise ValueError(
                f"Robot total mass must be positive, got {self._total_mass}. "
                "Check URDF inertial definitions."
            )

    # -----------------------------------------------------------------------
    # 属性
    # -----------------------------------------------------------------------
    @property
    def total_mass(self) -> float:
        """机器人总质量 [kg]。"""
        return self._total_mass

    # -----------------------------------------------------------------------
    # 状态读取
    # -----------------------------------------------------------------------
    def get_state(self) -> tuple:
        """
        返回当前状态 (q, v)。

        q: 广义位置 [base_pos(3), base_quat(4), joint_pos(num_joints)]
        v: 广义速度 [base_vel(3), base_omega(3), joint_vel(num_dof_joints)]
        """
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        lin_vel, ang_vel = p.getBaseVelocity(self.robot_id)
        joint_states = p.getJointStates(self.robot_id, range(self.num_joints))

        q = np.zeros(self.nq)
        q[0:3] = pos
        q[3:7] = orn
        for i in range(self.num_joints):
            q[7 + i] = joint_states[i][0]

        v = np.zeros(self.nv)
        v[0:3] = lin_vel
        v[3:6] = ang_vel
        for idx, joint_idx in enumerate(self.dof_joints):
            v[6 + idx] = joint_states[joint_idx][1]

        return q, v

    # -----------------------------------------------------------------------
    # 动力学计算
    # -----------------------------------------------------------------------
    def compute_mass_matrix(self, q: np.ndarray) -> np.ndarray:
        """计算质量矩阵 M(q)，维度 (nv, nv)。"""
        joint_positions = q[7:].tolist()
        M, _ = p.calculateMassMatrix(self.robot_id, joint_positions)
        return np.array(M)

    def compute_coriolis_gravity(self, q: np.ndarray, v: np.ndarray) -> np.ndarray:
        """
        计算 C(q, v)（科氏力 + 重力），维度 (nv,)。

        通过将加速度设为零调用 calculateInverseDynamics 得到。
        """
        joint_positions = q[7:].tolist()

        # PyBullet 需要完整长度的关节速度（含固定关节）
        v_full = np.zeros(self.num_joints)
        for idx, joint_idx in enumerate(self.dof_joints):
            v_full[joint_idx] = v[6 + idx]
        obj_velocities = v_full.tolist()
        obj_accelerations = [0.0] * self.num_joints

        C = p.calculateInverseDynamics(
            self.robot_id, joint_positions, obj_velocities, obj_accelerations
        )
        return np.array(C)

    def compute_com_position(self, q: np.ndarray = None) -> np.ndarray:
        """计算质心 CoM 位置 (3,)。"""
        com = np.zeros(3)

        # 基座
        mass = p.getDynamicsInfo(self.robot_id, -1)[0]
        pos, _ = p.getBasePositionAndOrientation(self.robot_id)
        com += mass * np.array(pos)

        # 连杆
        for i in range(self.num_joints):
            mass = p.getDynamicsInfo(self.robot_id, i)[0]
            if mass <= 0.0:
                continue
            link_state = p.getLinkState(self.robot_id, i, computeForwardKinematics=1)
            pos = np.array(link_state[0])   # linkWorldPosition = CoM
            com += mass * pos

        com /= self.total_mass
        return com

    def compute_com_velocity(self, q: np.ndarray, v: np.ndarray) -> np.ndarray:
        """计算质心 CoM 速度 (3,)。"""
        J_com = self.get_com_jacobian(q)
        return J_com @ v

    def compute_centroidal_momentum(self, q: np.ndarray, v: np.ndarray) -> np.ndarray:
        """计算质心角动量 L (3,)。"""
        J_L = self.get_angular_momentum_jacobian(q)
        return J_L @ v

    # -----------------------------------------------------------------------
    # Jacobian 计算
    # -----------------------------------------------------------------------
    def get_foot_jacobian(self, foot_link: int, q: np.ndarray,
                          local_position: Optional[np.ndarray] = None) -> np.ndarray:
        """
        计算支撑足的接触 Jacobian J_c，维度 (6, nv)。

        Args:
            foot_link: 支撑足连杆索引
            q: 广义位置
            local_position: 接触点在连杆局部坐标系中的位置，默认 [0, 0, 0]
        """
        if local_position is None:
            local_position = [0.0, 0.0, 0.0]
        joint_positions = q[7:].tolist()
        lin_jac, ang_jac = p.calculateJacobian(
            self.robot_id, foot_link, local_position,
            joint_positions,
            [0.0] * self.num_joints,
            [0.0] * self.num_joints,
        )
        J_c = np.vstack([np.array(lin_jac), np.array(ang_jac)])
        return J_c

    def get_com_jacobian(self, q: np.ndarray) -> np.ndarray:
        """
        计算 CoM Jacobian J_com，维度 (3, nv)。

        J_com = (1 / M) * sum_i(m_i * J_{v_i})
        """
        joint_positions = q[7:].tolist()
        J_com = np.zeros((3, self.nv))

        # 基座
        mass, _, _, local_inertial_pos, _ = p.getDynamicsInfo(self.robot_id, -1)[:5]
        if mass > 0.0:
            lin_jac, _ = p.calculateJacobian(
                self.robot_id, -1, local_inertial_pos,
                joint_positions,
                [0.0] * self.num_joints,
                [0.0] * self.num_joints,
            )
            J_com += mass * np.array(lin_jac)

        # 连杆
        for i in range(self.num_joints):
            mass, _, _, local_inertial_pos, _ = p.getDynamicsInfo(self.robot_id, i)[:5]
            if mass <= 0.0:
                continue
            lin_jac, _ = p.calculateJacobian(
                self.robot_id, i, local_inertial_pos,
                joint_positions,
                [0.0] * self.num_joints,
                [0.0] * self.num_joints,
            )
            J_com += mass * np.array(lin_jac)

        J_com /= self.total_mass
        return J_com

    def get_angular_momentum_jacobian(self, q: np.ndarray) -> np.ndarray:
        """
        计算角动量 Jacobian J_L，维度 (3, nv)。

        J_L = sum_i( I_i_world * J_{w_i} + m_i * [r_i - c]x * J_{v_i} )
        """
        c = self.compute_com_position()
        joint_positions = q[7:].tolist()
        J_L = np.zeros((3, self.nv))

        # 基座
        mass, _, local_inertia_diag, local_inertial_pos, _ = p.getDynamicsInfo(self.robot_id, -1)[:5]
        if mass > 0.0:
            lin_jac, ang_jac = p.calculateJacobian(
                self.robot_id, -1, local_inertial_pos,
                joint_positions,
                [0.0] * self.num_joints,
                [0.0] * self.num_joints,
            )
            J_v = np.array(lin_jac)
            J_w = np.array(ang_jac)

            base_pos, base_orn = p.getBasePositionAndOrientation(self.robot_id)
            R = np.array(p.getMatrixFromQuaternion(base_orn)).reshape(3, 3)
            I_local = np.diag(local_inertia_diag)
            I_world = R @ I_local @ R.T

            r = np.array(base_pos) - c
            J_L += I_world @ J_w + mass * _skew(r) @ J_v

        # 连杆
        for i in range(self.num_joints):
            mass, _, local_inertia_diag, local_inertial_pos, _ = p.getDynamicsInfo(self.robot_id, i)[:5]
            if mass <= 0.0:
                continue

            lin_jac, ang_jac = p.calculateJacobian(
                self.robot_id, i, local_inertial_pos,
                joint_positions,
                [0.0] * self.num_joints,
                [0.0] * self.num_joints,
            )
            J_v = np.array(lin_jac)
            J_w = np.array(ang_jac)

            link_state = p.getLinkState(self.robot_id, i, computeForwardKinematics=1)
            link_pos = np.array(link_state[0])
            link_orn = np.array(link_state[1])
            R = np.array(p.getMatrixFromQuaternion(link_orn)).reshape(3, 3)
            I_local = np.diag(local_inertia_diag)
            I_world = R @ I_local @ R.T

            r = link_pos - c
            J_L += I_world @ J_w + mass * _skew(r) @ J_v

        return J_L

    # -----------------------------------------------------------------------
    # 连杆位姿查询
    # -----------------------------------------------------------------------
    def get_link_com_position(self, link_index: int) -> np.ndarray:
        """
        获取指定连杆的质心位置（世界坐标）。

        Args:
            link_index: 连杆索引（-1 表示基座）

        Returns:
            质心位置 (3,)
        """
        if link_index == -1:
            pos, _ = p.getBasePositionAndOrientation(self.robot_id)
            return np.array(pos)
        link_state = p.getLinkState(self.robot_id, link_index, computeForwardKinematics=1)
        return np.array(link_state[0])

    def get_link_velocity(self, link_index: int) -> tuple:
        """
        获取指定连杆质心的线速度与角速度（世界坐标）。

        Args:
            link_index: 连杆索引（-1 表示基座）

        Returns:
            (lin_vel, ang_vel) 均为 (3,)
        """
        if link_index == -1:
            lin_vel, ang_vel = p.getBaseVelocity(self.robot_id)
            return np.array(lin_vel), np.array(ang_vel)
        link_state = p.getLinkState(
            self.robot_id, link_index,
            computeLinkVelocity=1, computeForwardKinematics=1
        )
        return np.array(link_state[6]), np.array(link_state[7])

    # -----------------------------------------------------------------------
    # 辅助
    # -----------------------------------------------------------------------
    def reset_joint_positions(self, joint_positions: np.ndarray):
        """重置关节位置。"""
        n = min(len(joint_positions), self.num_joints)
        for i in range(n):
            p.resetJointState(self.robot_id, i, float(joint_positions[i]))

    def reset_base_pose(self, position: np.ndarray, orientation: np.ndarray):
        """重置基座位姿。"""
        p.resetBasePositionAndOrientation(
            self.robot_id, position.tolist(), orientation.tolist()
        )
