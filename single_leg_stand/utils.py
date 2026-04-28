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


def plot_diagnostics(
    log, link_to_foot_name, support_foot_link, swing_foot_link,
    rmse_thresh=0.02, slip_thresh=0.005,
    ds_min_force=80.0, load_shift_roll_delta=0.07, ds_max_L_norm=0.5,
):
    """Multi-panel diagnostic plot with threshold reference lines."""
    time_arr = np.array(log.t)
    com_arr = np.stack(log.com)
    com_ref_arr = np.stack(log.com_ref)
    phase_arr = np.array(log.phase)
    L_arr = np.stack(log.L)
    n_phases = max(phase_arr)

    phase_times = []
    for p in range(1, n_phases + 1):
        idx = np.where(phase_arr >= p)[0]
        if len(idx) > 0:
            phase_times.append(time_arr[idx[0]])

    def vlines(ax):
        for pt in phase_times:
            ax.axvline(pt, color="gray", linestyle="--", alpha=0.4)

    fig, axes = plt.subplots(6, 1, figsize=(12, 16), sharex=True)

    ax = axes[0]
    for i, label in enumerate(["x", "y", "z"]):
        ax.plot(time_arr, com_arr[:, i] - com_ref_arr[:, i], label=label)
    ax.axhline(0, color="k", linewidth=0.5)
    ax.axhline(rmse_thresh, color="red", linestyle="--", alpha=0.5, label=f"±{rmse_thresh*1000:.0f} mm")
    ax.axhline(-rmse_thresh, color="red", linestyle="--", alpha=0.5)
    ax.set_ylabel("CoM error [m]")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    vlines(ax)

    ax = axes[1]
    ax.plot(time_arr, log.support_force, label=f"support force ({link_to_foot_name.get(support_foot_link, '')})", color="C0")
    ax.plot(time_arr, log.swing_force, label=f"swing force ({link_to_foot_name.get(swing_foot_link, '')})", color="C1")
    ax.axhline(0, color="gray", linestyle=":", alpha=0.3)
    ax.axhline(ds_min_force, color="red", linestyle="--", alpha=0.6, label=f"min force ({ds_min_force} N)")
    ax.set_ylabel("Foot force [N]")
    ax.legend(loc="upper left")
    ax.grid(True, alpha=0.3)
    vlines(ax)

    ax = axes[2]
    slip_thresh_mm = slip_thresh * 1000.0
    for link_id in log.foot_links:
        slip = np.array(log.foot_slip[link_id]) * 1000.0
        name = link_to_foot_name.get(link_id, f"link_{link_id}")
        ax.plot(time_arr, slip, label=name)
    ax.axhline(slip_thresh_mm, color="red", linestyle="--", alpha=0.6, label=f"slip limit ({slip_thresh_mm:.1f} mm)")
    ax.set_ylabel("Foot slip [mm]")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    vlines(ax)

    ax = axes[3]
    ax.plot(time_arr, log.roll_delta, label="roll delta")
    ax.axhline(0, color="gray", linestyle=":", alpha=0.3)
    ax.axhline(load_shift_roll_delta, color="red", linestyle="--", alpha=0.5, label=f"±{load_shift_roll_delta:.3f} rad")
    ax.axhline(-load_shift_roll_delta, color="red", linestyle="--", alpha=0.5)
    ax.set_ylabel("Roll delta [rad]")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    vlines(ax)

    ax = axes[4]
    ax.plot(time_arr, log.cop_y, label="CoP y")
    ax.axhline(0, color="gray", linestyle=":", alpha=0.3)
    ax.set_ylabel("CoP y [m]")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    vlines(ax)

    ax = axes[5]
    for i, label in enumerate(["Lx", "Ly", "Lz"]):
        ax.plot(time_arr, L_arr[:, i], label=label)
    ax.axhline(0, color="k", linewidth=0.5)
    ax.axhline(ds_max_L_norm, color="red", linestyle="--", alpha=0.5, label=f"±{ds_max_L_norm:.2f}")
    ax.axhline(-ds_max_L_norm, color="red", linestyle="--", alpha=0.5)
    ax.set_ylabel("Angular momentum")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    vlines(ax)

    axes[-1].set_xlabel("Time [s]")
    fig.suptitle("Diagnostics")
    plt.tight_layout()
    _save_figure(fig, "diagnostics.png")


