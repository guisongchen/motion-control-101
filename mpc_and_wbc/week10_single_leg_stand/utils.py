"""绘图、日志、摩擦锥辅助函数"""

import numpy as np
import matplotlib.pyplot as plt
from typing import List


def build_friction_cone_matrix(mu: float) -> tuple:
    """
    构造 4 面线性摩擦锥的约束矩阵 A_fcon 和边界 b_fcon。
    约束形式: A_fcon @ f <= b_fcon
    """
    # |f_x| <= mu * f_z  ->  f_x - mu * f_z <= 0, -f_x - mu * f_z <= 0
    # |f_y| <= mu * f_z  ->  f_y - mu * f_z <= 0, -f_y - mu * f_z <= 0
    A_fcon = np.array([
        [1.0,  0.0, -mu],
        [-1.0, 0.0, -mu],
        [0.0,  1.0, -mu],
        [0.0, -1.0, -mu],
    ])
    b_fcon = np.zeros(4)
    return A_fcon, b_fcon


def add_friction_cone_to_qp(A: np.ndarray, l: np.ndarray, u: np.ndarray,
                            A_fcon: np.ndarray, b_fcon: np.ndarray,
                            u_slice: slice) -> tuple:
    """
    将摩擦锥不等式约束叠加到现有 QP 上。
    假设 u 变量中的 u_slice 对应接触力 f。
    """
    # TODO: 将 A_fcon @ f <= b_fcon 映射到完整决策变量
    return A, l, u


def setup_logger():
    """初始化实验日志记录器。"""
    # TODO: 配置 logging
    pass


def plot_com_tracking(time_log: List[float],
                      com_log: List[np.ndarray],
                      com_ref_log: List[np.ndarray]):
    """绘制 CoM 跟踪曲线。"""
    # TODO: 绘制 x, y, z 三轴跟踪对比
    pass


def plot_contact_force(time_log: List[float],
                       force_log: List[np.ndarray]):
    """绘制接触力曲线。"""
    # TODO: 绘制 f_x, f_y, f_z 时序
    pass


def plot_torques(time_log: List[float],
                 tau_log: List[np.ndarray]):
    """绘制关节力矩曲线。"""
    # TODO: 绘制各关节力矩时序
    pass


def compute_rmse(actual: List[np.ndarray], reference: List[np.ndarray]) -> float:
    """计算 RMSE。"""
    actual_arr = np.stack(actual)
    ref_arr = np.stack(reference)
    return float(np.sqrt(np.mean(np.sum((actual_arr - ref_arr) ** 2, axis=1))))
