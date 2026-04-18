"""URDF 加载，Jacobians，动力学量计算"""

import numpy as np
import pybullet as p
from typing import Optional


class RobotModel:
    """封装 PyBullet 机器人模型的运动学与动力学接口。"""

    def __init__(self, urdf_path: str, base_position: Optional[np.ndarray] = None):
        """
        加载 URDF 并初始化模型。
        """
        self.urdf_path = urdf_path
        if base_position is None:
            base_position = [0.0, 0.0, 1.0]
        self.robot_id = p.loadURDF(urdf_path, basePosition=base_position)
        self.num_joints = p.getNumJoints(self.robot_id)
        # TODO: 记录浮动基、支撑足、摆动足的 link/joint index
        pass

    # -----------------------------------------------------------------------
    # 状态读取
    # -----------------------------------------------------------------------
    def get_state(self) -> tuple:
        """
        返回当前状态 (q, v)。
        q: 广义位置 [base_pos(3), base_quat(4), joint_pos(n)]
        v: 广义速度 [base_vel(3), base_omega(3), joint_vel(n)]
        """
        # TODO: 从 PyBullet 读取 q, v
        pass

    # -----------------------------------------------------------------------
    # 动力学计算
    # -----------------------------------------------------------------------
    def compute_mass_matrix(self, q: np.ndarray) -> np.ndarray:
        """计算质量矩阵 M(q)。"""
        # TODO: 调用 p.calculateMassMatrix
        pass

    def compute_coriolis_gravity(self, q: np.ndarray, v: np.ndarray) -> np.ndarray:
        """计算 C(q, v)（科氏力 + 重力）。"""
        # TODO: 调用 p.calculateInverseDynamics 零加速度得到 C
        pass

    def compute_com_position(self, q: np.ndarray) -> np.ndarray:
        """计算质心 CoM 位置。"""
        # TODO: 加权平均各 link 质心
        pass

    def compute_com_velocity(self, q: np.ndarray, v: np.ndarray) -> np.ndarray:
        """计算质心 CoM 速度。"""
        # TODO: 通过 CoM Jacobian 映射
        pass

    def compute_centroidal_momentum(self, q: np.ndarray, v: np.ndarray) -> np.ndarray:
        """计算质心动量 L。"""
        # TODO: 计算角动量
        pass

    # -----------------------------------------------------------------------
    # Jacobian 计算
    # -----------------------------------------------------------------------
    def get_foot_jacobian(self, foot_link: int, q: np.ndarray) -> np.ndarray:
        """计算支撑足的接触 Jacobian J_c。"""
        # TODO: 调用 p.calculateJacobian
        pass

    def get_com_jacobian(self, q: np.ndarray) -> np.ndarray:
        """计算 CoM Jacobian J_com。"""
        # TODO: 构造 CoM Jacobian
        pass

    def get_angular_momentum_jacobian(self, q: np.ndarray) -> np.ndarray:
        """计算角动量 Jacobian J_L。"""
        # TODO: 构造 J_L
        pass

    # -----------------------------------------------------------------------
    # 辅助
    # -----------------------------------------------------------------------
    def reset_joint_positions(self, joint_positions: np.ndarray):
        """重置关节位置。"""
        # TODO: 设置初始姿态
        pass
