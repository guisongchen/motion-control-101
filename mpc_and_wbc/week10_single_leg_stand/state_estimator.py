"""从仿真读取状态，计算 centroidal quantities"""

import numpy as np
from robot_model import RobotModel


class StateEstimator:
    """状态估计器：从 PyBullet 读取原始状态并计算 MPC/WBC 所需量。"""

    def __init__(self, robot: RobotModel):
        self.robot = robot

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
                - p_foot: 支撑足位置 (3,)
        """
        q, v = self.robot.get_state()

        c = self.robot.compute_com_position(q)
        c_dot = self.robot.compute_com_velocity(q, v)
        L = self.robot.compute_centroidal_momentum(q, v)

        # TODO: 读取支撑足位置
        p_foot = np.zeros(3)

        return {
            "c": c,
            "c_dot": c_dot,
            "L": L,
            "q": q,
            "v": v,
            "p_foot": p_foot,
        }
