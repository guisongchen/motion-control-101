"""
Phase 2: Linear MPC closed-loop simulation and comparison with LQR baseline.

Runs both controllers on the same MuJoCo cart-pole model and initial condition,
records trajectories / costs / solve times, and produces overlay plots.
"""

import os
import sys
import time
from pathlib import Path

import matplotlib.pyplot as plt
import mujoco
import numpy as np

_repo_root = Path(__file__).parent.parent
sys.path.insert(0, str(_repo_root))

from mpc.controllers.lqr import CartPoleLQR
from mpc.controllers.mpc import CartPoleMPC


def run_mpc_simulation(
    xml_path: str | Path,
    duration: float = 5.0,
    theta0: float = 0.1,
    N: int = 20,
    u_min: float | None = None,
    u_max: float | None = None,
    render: bool = False,
) -> dict:
    """
    Run closed-loop MPC simulation in MuJoCo.

    Returns
    -------
    data : dict
        Contains time, state, control, cost, solve_time arrays and metrics.
    """
    xml_path = Path(xml_path)
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)

    dt = model.opt.timestep
    assert np.isclose(dt, 0.01), f"Model timestep {dt} != 0.01 s"

    # LQR for shared dynamics / terminal cost
    lqr = CartPoleLQR(dt=dt)
    mpc = CartPoleMPC(Ad=lqr.Ad, Bd=lqr.Bd, N=N, P=lqr.P, u_min=u_min, u_max=u_max)

    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    data.qpos[1] = theta0
    mujoco.mj_forward(model, data)

    n_steps = int(duration / dt)
    t_hist = np.zeros(n_steps)
    x_hist = np.zeros((n_steps, 4))
    u_hist = np.zeros(n_steps)
    J_hist = np.zeros(n_steps)
    solve_time_hist = np.zeros(n_steps)
    iter_hist = np.zeros(n_steps, dtype=int)

    if render:
        from mujoco import viewer

        viewer_ctx = viewer.launch_passive(model, data)
    else:
        viewer_ctx = None

    start_wall = time.perf_counter()

    J_cum = 0.0
    for k in range(n_steps):
        x = data.qpos[0]
        theta = data.qpos[1]
        x_dot = data.qvel[0]
        theta_dot = data.qvel[1]
        state = np.array([x, x_dot, theta, theta_dot])

        # MPC: solve QP and apply first control
        u, info = mpc.solve(state)
        data.ctrl[0] = u

        t_hist[k] = data.time
        x_hist[k] = state
        u_hist[k] = u
        J_cum += lqr.stage_cost(state, u)
        J_hist[k] = J_cum
        solve_time_hist[k] = info["solve_time_ms"]
        iter_hist[k] = info["iter"]

        mujoco.mj_step(model, data)

        if viewer_ctx is not None and viewer_ctx.is_running():
            viewer_ctx.sync()
            time.sleep(dt * 0.5)

    wall_time = time.perf_counter() - start_wall
    print(f"[MPC] Simulated {duration}s in {wall_time:.3f}s wall time")

    if viewer_ctx is not None:
        viewer_ctx.close()

    theta = x_hist[:, 2]
    settling_time = None
    for k in range(n_steps):
        if np.all(np.abs(theta[k:]) < 0.01):
            settling_time = t_hist[k]
            break

    if theta0 > 0:
        overshoot = abs(float(np.min(theta)))
    else:
        overshoot = abs(float(np.max(theta)))

    metrics = {
        "time": t_hist,
        "state": x_hist,
        "control": u_hist,
        "cost": J_hist,
        "solve_time_ms": solve_time_hist,
        "iter": iter_hist,
        "settling_time": settling_time,
        "overshoot_rad": overshoot,
        "max_force": np.max(np.abs(u_hist)),
        "final_theta": theta[-1],
        "final_x": x_hist[-1, 0],
        "mean_solve_time_ms": np.mean(solve_time_ms := solve_time_hist),
        "max_solve_time_ms": np.max(solve_time_ms),
    }
    return metrics


def run_lqr_simulation(
    xml_path: str | Path,
    duration: float = 5.0,
    theta0: float = 0.1,
) -> dict:
    """Thin wrapper around Phase 1 LQR sim for side-by-side comparison."""
    from mpc.phase1_lqr_sim import run_lqr_simulation as _run_lqr

    return _run_lqr(xml_path, duration=duration, theta0=theta0, render=False)


def plot_comparison(lqr_metrics: dict, mpc_metrics: dict, save_path: str | Path | None = None):
    """Overlay LQR vs MPC trajectories."""
    t = lqr_metrics["time"]

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

    # State trajectory
    ax = axes[0]
    ax.plot(t, lqr_metrics["state"][:, 0], label="LQR cart x", lw=1.5, color="C0", ls="-")
    ax.plot(t, mpc_metrics["state"][:, 0], label="MPC cart x", lw=1.5, color="C0", ls="--")
    ax.plot(t, lqr_metrics["state"][:, 2], label="LQR theta", lw=1.5, color="C1", ls="-")
    ax.plot(t, mpc_metrics["state"][:, 2], label="MPC theta", lw=1.5, color="C1", ls="--")
    ax.axhline(0.01, color="gray", ls="--", lw=0.8)
    ax.axhline(-0.01, color="gray", ls="--", lw=0.8)
    ax.set_ylabel("state")
    ax.legend(loc="upper right", ncol=2)
    ax.set_title("Phase 2: LQR vs MPC Closed-Loop Trajectory")
    ax.grid(True, alpha=0.3)

    # Control trajectory
    ax = axes[1]
    ax.plot(t, lqr_metrics["control"], label="LQR force", lw=1.5, color="C2", ls="-")
    ax.plot(t, mpc_metrics["control"], label="MPC force", lw=1.5, color="C2", ls="--")
    ax.set_ylabel("control [N]")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    # Cumulative cost
    ax = axes[2]
    ax.plot(t, lqr_metrics["cost"], label="LQR J", lw=1.5, color="C3", ls="-")
    ax.plot(t, mpc_metrics["cost"], label="MPC J", lw=1.5, color="C3", ls="--")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("cumulative cost")
    ax.legend(loc="lower right")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()

    if save_path is not None:
        plt.savefig(save_path, dpi=150)
        print(f"Comparison plot saved to {save_path}")
    else:
        plt.show()


def plot_solve_time(mpc_metrics: dict, save_path: str | Path | None = None):
    """Plot MPC solve time histogram and per-frame trace."""
    t = mpc_metrics["time"]
    st = mpc_metrics["solve_time_ms"]

    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=False)

    ax = axes[0]
    ax.plot(t, st, lw=1.0, color="C4")
    ax.axhline(np.mean(st), color="red", ls="--", lw=1.0, label=f"mean={np.mean(st):.3f} ms")
    ax.axhline(10.0, color="gray", ls=":", lw=0.8, label="control period=10 ms")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("QP solve time [ms]")
    ax.set_title("MPC OSQP Solve Time per Frame")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    ax.hist(st, bins=50, color="C4", edgecolor="white", alpha=0.8)
    ax.axvline(np.mean(st), color="red", ls="--", lw=1.0, label=f"mean={np.mean(st):.3f} ms")
    ax.set_xlabel("solve time [ms]")
    ax.set_ylabel("count")
    ax.set_title("Solve Time Distribution")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()

    if save_path is not None:
        plt.savefig(save_path, dpi=150)
        print(f"Solve-time plot saved to {save_path}")
    else:
        plt.show()


def print_comparison(lqr_metrics: dict, mpc_metrics: dict):
    print("\n=== Phase 2 Comparison: LQR vs MPC ===")
    print(f"{'Metric':<30} {'LQR':>12} {'MPC':>12}")
    print("-" * 54)
    print(
        f"{'Settling time (s)':<30} {str(lqr_metrics['settling_time']):>12} {str(mpc_metrics['settling_time']):>12}"
    )
    print(f"{'Overshoot (rad)':<30} {lqr_metrics['overshoot_rad']:>12.4f} {mpc_metrics['overshoot_rad']:>12.4f}")
    print(f"{'Max |force| (N)':<30} {lqr_metrics['max_force']:>12.3f} {mpc_metrics['max_force']:>12.3f}")
    print(f"{'Final theta (rad)':<30} {lqr_metrics['final_theta']:>12.6f} {mpc_metrics['final_theta']:>12.6f}")
    print(f"{'Final x (m)':<30} {lqr_metrics['final_x']:>12.6f} {mpc_metrics['final_x']:>12.6f}")
    print(f"{'Final cost J':<30} {lqr_metrics['cost'][-1]:>12.3f} {mpc_metrics['cost'][-1]:>12.3f}")
    print(f"{'Mean solve time (ms)':<30} {'—':>12} {mpc_metrics['mean_solve_time_ms']:>12.3f}")
    print(f"{'Max solve time (ms)':<30} {'—':>12} {mpc_metrics['max_solve_time_ms']:>12.3f}")
    print("========================================\n")


if __name__ == "__main__":
    xml_path = _repo_root / "mpc" / "models" / "cartpole.xml"
    out_dir = _repo_root / "mpc" / "outputs"
    out_dir.mkdir(exist_ok=True)

    duration = 5.0
    theta0 = 0.1
    N = 20

    print(f"Running LQR simulation (theta0={theta0} rad)...")
    lqr_metrics = run_lqr_simulation(xml_path, duration=duration, theta0=theta0)

    print(f"Running MPC simulation (theta0={theta0} rad, N={N})...")
    mpc_metrics = run_mpc_simulation(xml_path, duration=duration, theta0=theta0, N=N)

    print_comparison(lqr_metrics, mpc_metrics)

    plot_comparison(lqr_metrics, mpc_metrics, save_path=out_dir / "phase2_mpc_vs_lqr.png")
    plot_solve_time(mpc_metrics, save_path=out_dir / "phase2_mpc_solve_time.png")
