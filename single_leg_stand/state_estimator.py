"""从仿真读取状态，计算 centroidal quantities"""

import numpy as np
from typing import List
from robot_model import RobotModel
from phase_metrics import compute_load_shift_metrics, LoadShiftMetrics


class StateEstimator:
    """状态估计器：从 MuJoCo 读取原始状态并计算 MPC/WBC 所需量。"""

    def __init__(self, robot: RobotModel):
        """
        Args:
            robot: RobotModel 实例（必须已构建 foot_link_ids）
        """
        self.robot = robot
        self.candidate_foot_links = robot.foot_link_ids
        self._preferred_support_foot_link = robot.foot_name_to_link[
            robot.config.support_foot_name
        ]
        self._swing_foot_link = next(
            link for link in self.candidate_foot_links
            if link != self._preferred_support_foot_link
        )
        self._initial_foot_pos: dict[int, np.ndarray] | None = None

    def update(
        self,
        preferred_support_foot_link: int | None = None,
        lock_support: bool = False,
    ) -> dict:
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
                - load_shift_metrics: 负载转移指标 LoadShiftMetrics
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
            contact_metrics = self.robot.get_contact_metrics(link)
            is_contact = contact_metrics["is_contact"]
            normal_force = contact_metrics["normal_force"]
            foot_contacts.append({
                "link": link,
                "is_contact": is_contact,
                "normal_force": normal_force,
                "position": np.array(contact_metrics["position"], copy=True),
            })
            # 选法向力最大的作为支撑足
            if normal_force > max_force:
                max_force = normal_force
                support_foot_link = link

        if lock_support:
            support_foot_link = (
                preferred_support_foot_link
                if preferred_support_foot_link is not None
                else self._preferred_support_foot_link
            )

        # 回退：若都没有接触，取第一个候选足端（避免 None 导致后续崩溃）
        if support_foot_link is None and self.candidate_foot_links:
            support_foot_link = self.candidate_foot_links[0]

        p_foot = self.robot.get_link_com_position(support_foot_link)

        # 首次更新时捕获初始足端位置（机器人已处于起始姿态）
        if self._initial_foot_pos is None:
            self._initial_foot_pos = {
                link: np.array(self.robot.get_contact_metrics(link)["position"], copy=True)
                for link in self.candidate_foot_links
            }

        load_shift_metrics = compute_load_shift_metrics(
            c,
            c_dot,
            foot_contacts,
            self._preferred_support_foot_link,
            self._swing_foot_link,
            self._initial_foot_pos,
        )

        return {
            "c": c,
            "c_dot": c_dot,
            "L": L,
            "q": q,
            "v": v,
            "support_foot_link": support_foot_link,
            "p_foot": p_foot,
            "foot_contacts": foot_contacts,
            "load_shift_metrics": load_shift_metrics,
        }
