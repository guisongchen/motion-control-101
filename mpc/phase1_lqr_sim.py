"""
Phase 1: LQR baseline controller for cart-pole in MuJoCo.

Simulation: 5 seconds, 100 Hz control, theta_0 = 0.1 rad.
Records state / control trajectories, cumulative cost, and convergence metrics.
"""

import os
import sys
import time
from pathlib import Path

import matplotlib.pyplot as plt
import mujoco
import numpy as np

# Add repo root to path so that `mpc` package is importable
_repo_root = Path(__file__).parent.parent
sys.path.insert(0, str(_repo_root))

from mpc.controllers.lqr import CartPoleLQR


def run_lqr_simulation(
    xml_path: str | Path,
    duration: float = 5.0,
    theta0: float = 0.1,
    render: bool = False,
) -> dict:
    """
    Run closed-loop LQR simulation in MuJoCo.

    Returns
    -------
    data : dict
        Contains time, state, control, cost arrays and metrics.
    """
    # Load model
    xml_path = Path(xml_path)
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)

    # Ensure timestep matches control period (0.01 s)
    dt = model.opt.timestep
    assert np.isclose(dt, 0.01), f"Model timestep {dt} != 0.01 s"

    # Controller
    ctrl = CartPoleLQR(dt=dt)

    # Initial state: theta = theta0, others = 0
    # qpos: [slider_pos, hinge_angle]
    # qvel: [slider_vel, hinge_vel]
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    data.qpos[1] = theta0  # hinge angle
    mujoco.mj_forward(model, data)

    n_steps = int(duration / dt)
    t_hist = np.zeros(n_steps)
    x_hist = np.zeros((n_steps, 4))  # [x, x_dot, theta, theta_dot]
    u_hist = np.zeros(n_steps)
    J_hist = np.zeros(n_steps)  # cumulative cost

    # Optional viewer
    if render:
        from mujoco import viewer

        viewer_ctx = viewer.launch_passive(model, data)
    else:
        viewer_ctx = None

    start_wall = time.perf_counter()

    J_cum = 0.0
    for k in range(n_steps):
        # Read current state
        x = data.qpos[0]
        theta = data.qpos[1]
        x_dot = data.qvel[0]
        theta_dot = data.qvel[1]
        state = np.array([x, x_dot, theta, theta_dot])

        # LQR control
        u = ctrl.control(state)
        data.ctrl[0] = u

        # Record
        t_hist[k] = data.time
        x_hist[k] = state
        u_hist[k] = u
        J_cum += ctrl.stage_cost(state, u)
        J_hist[k] = J_cum

        # Step simulation
        mujoco.mj_step(model, data)

        if viewer_ctx is not None and viewer_ctx.is_running():
            viewer_ctx.sync()
            time.sleep(dt * 0.5)  # slow down for visualization

    wall_time = time.perf_counter() - start_wall
    print(f"Simulated {duration}s in {wall_time:.3f}s wall time")

    if viewer_ctx is not None:
        viewer_ctx.close()

    # Metrics
    theta = x_hist[:, 2]
    settling_time = None
    for k in range(n_steps):
        if np.all(np.abs(theta[k:]) < 0.01):
            settling_time = t_hist[k]
            break

    # Overshoot: how far the pole swings past upright on the opposite side
    if theta0 > 0:
        overshoot = abs(float(np.min(theta)))
    else:
        overshoot = abs(float(np.max(theta)))

    metrics = {
        "time": t_hist,
        "state": x_hist,
        "control": u_hist,
        "cost": J_hist,
        "settling_time": settling_time,
        "overshoot_rad": overshoot,
        "max_force": np.max(np.abs(u_hist)),
        "final_theta": theta[-1],
        "final_x": x_hist[-1, 0],
    }

    return metrics


def plot_results(metrics: dict, save_path: str | Path | None = None):
    """Plot state trajectory, control trajectory, and cumulative cost."""
    t = metrics["time"]
    x = metrics["state"]
    u = metrics["control"]
    J = metrics["cost"]

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

    # State trajectory
    ax = axes[0]
    ax.plot(t, x[:, 0], label="cart x [m]", lw=1.5)
    ax.plot(t, x[:, 2], label="theta [rad]", lw=1.5)
    ax.axhline(0.01, color="gray", ls="--", lw=0.8)
    ax.axhline(-0.01, color="gray", ls="--", lw=0.8)
    ax.set_ylabel("state")
    ax.legend(loc="upper right")
    ax.set_title("Phase 1: LQR Closed-Loop Trajectory")
    ax.grid(True, alpha=0.3)

    # Control trajectory
    ax = axes[1]
    ax.plot(t, u, label="force [N]", color="C2", lw=1.5)
    ax.set_ylabel("control")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    # Cumulative cost
    ax = axes[2]
    ax.plot(t, J, label="cumulative cost J", color="C3", lw=1.5)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("cost")
    ax.legend(loc="lower right")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()

    if save_path is not None:
        plt.savefig(save_path, dpi=150)
        print(f"Plot saved to {save_path}")
    else:
        plt.show()


def print_metrics(metrics: dict):
    print("\n=== Phase 1 LQR Metrics ===")
    print(f"Settling time (|theta| < 0.01 rad): {metrics['settling_time']}")
    print(f"Overshoot: {metrics['overshoot_rad']:.4f} rad")
    print(f"Max |force|: {metrics['max_force']:.3f} N")
    print(f"Final theta: {metrics['final_theta']:.6f} rad")
    print(f"Final x: {metrics['final_x']:.6f} m")
    print(f"Final cumulative cost J: {metrics['cost'][-1]:.3f}")
    print("============================\n")


if __name__ == "__main__":
    xml_path = _repo_root / "mpc" / "models" / "cartpole.xml"

    print(f"Running LQR simulation with model: {xml_path}")
    metrics = run_lqr_simulation(xml_path, duration=5.0, theta0=0.1, render=False)
    print_metrics(metrics)

    # Save plot
    out_dir = _repo_root / "mpc" / "outputs"
    out_dir.mkdir(exist_ok=True)
    plot_results(metrics, save_path=out_dir / "phase1_lqr_results.png")
