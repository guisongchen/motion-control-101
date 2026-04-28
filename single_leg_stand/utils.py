"""绘图、日志、摩擦锥辅助函数"""

import logging
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from typing import List


OUTPUT_DIR = Path(__file__).resolve().parent / "output"


def _save_figure(fig: plt.Figure, filename: str):
    """将图像保存到实验 output 目录。"""
    OUTPUT_DIR.mkdir(exist_ok=True)
    fig.savefig(OUTPUT_DIR / filename, dpi=150, bbox_inches="tight")
    plt.close(fig)


def build_friction_cone_matrix(mu: float, gamma: float = 0.0) -> tuple:
    """
    构造线性摩擦锥的约束矩阵 A_fcon 和边界 b_fcon。

    当 gamma == 0 时，返回 3D 版本 (f = [fx, fy, fz])，4 个不等式，
    兼容 MPC 等使用 3D 接触力的代码。

    当 gamma > 0 时，返回 4D 版本 (f = [fx, fy, fz, mz])，6 个不等式：
      |f_x| <= mu * f_z     -> 2 个
      |f_y| <= mu * f_z     -> 2 个
      |m_z| <= gamma * f_z  -> 2 个
    """
    if gamma <= 0.0:
        # 3D 摩擦锥 (兼容 MPC)
        A_fcon = np.array([
            [1.0,  0.0, -mu],
            [-1.0, 0.0, -mu],
            [0.0,  1.0, -mu],
            [0.0, -1.0, -mu],
        ])
        b_fcon = np.zeros(4)
        return A_fcon, b_fcon

    # 4D 摩擦锥 (WBC 用)
    A_fcon = np.array([
        [1.0,  0.0, -mu,    0.0],
        [-1.0, 0.0, -mu,    0.0],
        [0.0,  1.0, -mu,    0.0],
        [0.0, -1.0, -mu,    0.0],
        [0.0,  0.0, -gamma, 1.0],
        [0.0,  0.0, -gamma, -1.0],
    ])
    b_fcon = np.zeros(6)
    return A_fcon, b_fcon


def add_friction_cone_to_qp(A: np.ndarray, l: np.ndarray, u: np.ndarray,
                            A_fcon: np.ndarray, b_fcon: np.ndarray,
                            u_slice: slice) -> tuple:
    """
    将摩擦锥不等式约束叠加到现有 QP 上。
    假设 u 变量中的 u_slice 对应接触力 f。

    新增约束: A_fcon @ f <= b_fcon，映射到完整决策变量 z。
    """
    n_cone = A_fcon.shape[0]
    nz = A.shape[1]

    # 构造摩擦锥在完整变量上的约束矩阵
    A_cone = np.zeros((n_cone, nz))
    A_cone[:, u_slice] = A_fcon

    # 叠加到现有约束
    A_new = np.vstack([A, A_cone])
    l_new = np.concatenate([l, np.full(n_cone, -np.inf)])
    u_new = np.concatenate([u, b_fcon])

    return A_new, l_new, u_new


def setup_logger(name: str = "motion_control", level: int = logging.INFO) -> logging.Logger:
    """初始化实验日志记录器。"""
    logger = logging.getLogger(name)
    logger.setLevel(level)

    if not logger.handlers:
        handler = logging.StreamHandler(sys.stdout)
        handler.setLevel(level)
        formatter = logging.Formatter(
            "[%(asctime)s] [%(levelname)s] %(message)s",
            datefmt="%H:%M:%S",
        )
        handler.setFormatter(formatter)
        logger.addHandler(handler)

    return logger


def plot_contact_force(time_log: List[float],
                       force_log: List[np.ndarray]):
    """绘制接触力曲线。"""
    time_arr = np.array(time_log)
    force_arr = np.stack(force_log)

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    labels = ["f_x", "f_y", "f_z"]
    colors = ["C0", "C1", "C2"]

    for i, (ax, label, color) in enumerate(zip(axes, labels, colors)):
        ax.plot(time_arr, force_arr[:, i], color=color, label=label)
        ax.set_ylabel(f"{label} [N]")
        ax.legend(loc="upper right")
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Time [s]")
    fig.suptitle("Contact Force")
    plt.tight_layout()
    _save_figure(fig, "contact_force.png")


def plot_torques(time_log: List[float],
                 tau_log: List[np.ndarray]):
    """绘制关节力矩曲线。"""
    time_arr = np.array(time_log)
    tau_arr = np.stack(tau_log)
    n_dof = tau_arr.shape[1]

    fig, ax = plt.subplots(figsize=(10, 5))
    for i in range(n_dof):
        ax.plot(time_arr, tau_arr[:, i], label=f"joint {i}")

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Torque [Nm]")
    ax.set_title("Joint Torques")
    ax.legend(loc="upper right", ncol=max(1, n_dof // 6))
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    _save_figure(fig, "joint_torques.png")


def compute_rmse(actual: List[np.ndarray], reference: List[np.ndarray]) -> float:
    """计算 RMSE。"""
    actual_arr = np.stack(actual)
    ref_arr = np.stack(reference)
    return float(np.sqrt(np.mean(np.sum((actual_arr - ref_arr) ** 2, axis=1))))


def compute_pd_torque(
    q_target: np.ndarray,
    q_current: np.ndarray,
    qd_current: np.ndarray,
    kp: float,
    kd: float,
    tau_limit: np.ndarray,
) -> np.ndarray:
    """Simple joint-space PD torque for MuJoCo motor actuators."""
    tau = kp * (q_target - q_current) - kd * qd_current
    return np.clip(tau, -tau_limit, tau_limit)


