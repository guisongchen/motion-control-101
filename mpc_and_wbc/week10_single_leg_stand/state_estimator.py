"""从仿真读取状态，计算 centroidal quantities"""

import numpy as np
from typing import List
from robot_model import RobotModel


class StateEstimator:
    """状态估计器：从 PyBullet 读取原始状态并计算 MPC/WBC 所需量。"""

    def __init__(self, robot: RobotModel, candidate_foot_links: List[int]):
        """
        Args:
            robot: RobotModel 实例
            candidate_foot_links: 候选足端 link 索引列表，动态检测哪个在接触
        """
        self.robot = robot
        self.candidate_foot_links = candidate_foot_links

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
                - support_foot_link: 当前检测到的支撑足 link 索引
                - p_foot: 支撑足质心位置 (3,)
                - foot_contacts: 各候选足端的接触力信息 List[dict]
        """
        q, v = self.robot.get_state()

        c = self.robot.compute_com_position()
        c_dot = self.robot.compute_com_velocity(q, v)
        L = self.robot.compute_centroidal_momentum(q, v)

        # 动态检测各候选足端的接触状态
        foot_contacts = []
        support_foot_link = None
        max_force = 0.0
        for link in self.candidate_foot_links:
            is_contact, normal_force = self.robot.check_contact(link)
            foot_contacts.append({
                "link": link,
                "is_contact": is_contact,
                "normal_force": normal_force,
                "position": self.robot.get_link_com_position(link),
            })
            # 选法向力最大的作为支撑足
            if normal_force > max_force:
                max_force = normal_force
                support_foot_link = link

        # 回退：若都没有接触，取第一个候选足端（避免 None 导致后续崩溃）
        if support_foot_link is None and self.candidate_foot_links:
            support_foot_link = self.candidate_foot_links[0]

        p_foot = self.robot.get_link_com_position(support_foot_link)

        return {
            "c": c,
            "c_dot": c_dot,
            "L": L,
            "q": q,
            "v": v,
            "support_foot_link": support_foot_link,
            "p_foot": p_foot,
            "foot_contacts": foot_contacts,
        }
