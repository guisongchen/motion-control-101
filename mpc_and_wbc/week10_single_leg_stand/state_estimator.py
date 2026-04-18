"""从仿真读取状态，计算 centroidal quantities"""

import numpy as np
from robot_model import RobotModel


class StateEstimator:
    """状态估计器：从 PyBullet 读取原始状态并计算 MPC/WBC 所需量。"""

    def __init__(self, robot: RobotModel, support_foot_link: int):
        """
        Args:
            robot: RobotModel 实例
            support_foot_link: 支撑足 link 索引（PyBullet link index）
        """
        self.robot = robot
        self.support_foot_link = support_foot_link

    def update(self) -> dict:
        """
        读取当前仿真状态并计算所有 MPC/WBC 输入量。

        Returns:
            state: dict，包含
                - c: CoM 位置 (3,)
                - c_dot: CoM 速度 (3,)
                - L: 质心角动量 (3,)
                - q: 广义位置
                - v: 广义速度
                - p_foot: 支撑足质心位置 (3,)
        """
        q, v = self.robot.get_state()

        c = self.robot.compute_com_position()
        c_dot = self.robot.compute_com_velocity(q, v)
        L = self.robot.compute_centroidal_momentum(q, v)
        p_foot = self.robot.get_link_com_position(self.support_foot_link)

        return {
            "c": c,
            "c_dot": c_dot,
            "L": L,
            "q": q,
            "v": v,
            "p_foot": p_foot,
        }
