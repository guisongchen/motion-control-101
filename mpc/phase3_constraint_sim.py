"""
Phase 3: Constraint comparison experiments — LQR (saturated) vs MPC (constrained).

Initial condition: theta0 = 0.3 rad (larger disturbance).
Control bounds: u_min = -5 N, u_max = +5 N.

LQR: standard LQR gain with naive clipping (saturation).
MPC: explicit box constraints inside OSQP optimization.
"""

import sys
import time
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import mujoco
import numpy as np

_repo_root = Path(__file__).parent.parent
sys.path.insert(0, str(_repo_root))

from mpc.controllers.lqr import CartPoleLQR
from mpc.controllers.mpc import CartPoleMPC


def _clip(u: float, u_min: float, u_max: float) -> float:
    return max(u_min, min(u_max, u))


def _detect_saturation_steps(u_hist: np.ndarray, bound: float, tol: float = 1e-6) -> int:
    """Count steps where |u| is at or beyond the saturation bound."""
    return int(np.sum(np.abs(u_hist) >= bound - tol))


def _is_stable(theta_hist: np.ndarray, t_hist: np.ndarray, threshold: float = 0.8) -> bool:
    """Return False if pole swings beyond threshold (indicating likely failure)."""
    return bool(np.all(np.abs(theta_hist) < threshold))


def run_lqr_saturated(
    xml_path: str | Path,
    duration: float = 5.0,
    theta0: float = 0.3,
    u_min: float = -5.0,
    u_max: float = 5.0,
    render: bool = False,
) -> dict:
    """LQR with control saturation (clipping)."""
    xml_path = Path(xml_path)
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)

    dt = model.opt.timestep
    assert np.isclose(dt, 0.01)

    ctrl = CartPoleLQR(dt=dt)

    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    data.qpos[1] = theta0
    mujoco.mj_forward(model, data)

    n_steps = int(duration / dt)
    t_hist = np.zeros(n_steps)
    x_hist = np.zeros((n_steps, 4))
    u_hist = np.zeros(n_steps)
    u_raw_hist = np.zeros(n_steps)
    J_hist = np.zeros(n_steps)

    if render:
        from mujoco import viewer
        viewer_ctx = viewer.launch_passive(model, data)
    else:
        viewer_ctx = None

    J_cum = 0.0
    for k in range(n_steps):
        x = data.qpos[0]
        theta = data.qpos[1]
        x_dot = data.qvel[0]
        theta_dot = data.qvel[1]
        state = np.array([x, x_dot, theta, theta_dot])

        u_raw = ctrl.control(state)
        u = _clip(u_raw, u_min, u_max)
        data.ctrl[0] = u

        t_hist[k] = data.time
        x_hist[k] = state
        u_hist[k] = u
        u_raw_hist[k] = u_raw
        J_cum += ctrl.stage_cost(state, u)
        J_hist[k] = J_cum

        mujoco.mj_step(model, data)

        if viewer_ctx is not None and viewer_ctx.is_running():
            viewer_ctx.sync()
            time.sleep(dt * 0.5)

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

    bound = min(abs(u_min), abs(u_max))
    sat_steps = _detect_saturation_steps(u_hist, bound)

    return {
        "time": t_hist,
        "state": x_hist,
        "control": u_hist,
        "control_raw": u_raw_hist,
        "cost": J_hist,
        "settling_time": settling_time,
        "overshoot_rad": overshoot,
        "max_force": np.max(np.abs(u_hist)),
        "final_theta": theta[-1],
        "final_x": x_hist[-1, 0],
        "saturation_steps": sat_steps,
        "saturation_duration": sat_steps * dt,
        "saturation_pct": 100.0 * sat_steps / n_steps,
        "is_stable": _is_stable(theta, t_hist),
    }


def run_mpc_constrained(
    xml_path: str | Path,
    duration: float = 5.0,
    theta0: float = 0.3,
    N: int = 20,
    u_min: float = -5.0,
    u_max: float = 5.0,
    render: bool = False,
) -> dict:
    """MPC with explicit control constraints."""
    xml_path = Path(xml_path)
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)

    dt = model.opt.timestep
    assert np.isclose(dt, 0.01)

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

    J_cum = 0.0
    for k in range(n_steps):
        x = data.qpos[0]
        theta = data.qpos[1]
        x_dot = data.qvel[0]
        theta_dot = data.qvel[1]
        state = np.array([x, x_dot, theta, theta_dot])

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

    bound = min(abs(u_min), abs(u_max))
    sat_steps = _detect_saturation_steps(u_hist, bound)

    return {
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
        "saturation_steps": sat_steps,
        "saturation_duration": sat_steps * dt,
        "saturation_pct": 100.0 * sat_steps / n_steps,
        "is_stable": _is_stable(theta, t_hist),
        "mean_solve_time_ms": np.mean(solve_time_hist),
        "max_solve_time_ms": np.max(solve_time_hist),
    }


def plot_phase3_comparison(
    lqr_metrics: dict,
    mpc_metrics: dict,
    u_min: float = -5.0,
    u_max: float = 5.0,
    save_path: str | Path | None = None,
):
    """Overlay LQR (saturated) vs MPC (constrained) with saturation bands."""
    t = lqr_metrics["time"]

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

    # --- State ---
    ax = axes[0]
    ax.plot(t, lqr_metrics["state"][:, 0], label="LQR cart x", lw=1.5, color="C0", ls="-")
    ax.plot(t, mpc_metrics["state"][:, 0], label="MPC cart x", lw=1.5, color="C0", ls="--")
    ax.plot(t, lqr_metrics["state"][:, 2], label="LQR theta", lw=1.5, color="C1", ls="-")
    ax.plot(t, mpc_metrics["state"][:, 2], label="MPC theta", lw=1.5, color="C1", ls="--")
    ax.axhline(0.01, color="gray", ls="--", lw=0.8)
    ax.axhline(-0.01, color="gray", ls="--", lw=0.8)
    ax.set_ylabel("state")
    ax.legend(loc="upper right", ncol=2)
    ax.set_title("Phase 3: LQR (saturated) vs MPC (constrained) — theta0=0.3 rad")
    ax.grid(True, alpha=0.3)

    # --- Control with saturation bands ---
    ax = axes[1]
    ax.plot(t, lqr_metrics["control"], label="LQR force", lw=1.5, color="C2", ls="-")
    ax.plot(t, mpc_metrics["control"], label="MPC force", lw=1.5, color="C2", ls="--")
    ax.axhline(u_max, color="red", ls=":", lw=1.0, alpha=0.7, label=f"saturation ±{abs(u_max)} N")
    ax.axhline(u_min, color="red", ls=":", lw=1.0, alpha=0.7)

    # Highlight saturation regions
    bound = min(abs(u_min), abs(u_max))
    lqr_sat = np.abs(lqr_metrics["control"]) >= bound - 1e-6
    mpc_sat = np.abs(mpc_metrics["control"]) >= bound - 1e-6
    ax.fill_between(t, u_min, u_max, where=lqr_sat, color="C2", alpha=0.15, label="LQR sat")
    ax.fill_between(t, u_min, u_max, where=mpc_sat, color="C2", alpha=0.15, label="MPC sat")

    ax.set_ylabel("control [N]")
    ax.legend(loc="upper right", ncol=2)
    ax.grid(True, alpha=0.3)

    # --- Cumulative cost ---
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
        print(f"Phase 3 comparison plot saved to {save_path}")
    else:
        plt.show()


def print_phase3_comparison(lqr_metrics: dict, mpc_metrics: dict):
    print("\n=== Phase 3 Comparison: LQR (saturated) vs MPC (constrained) ===")
    print(f"{'Metric':<35} {'LQR':>14} {'MPC':>14}")
    print("-" * 63)
    print(f"{'Settling time (s)':<35} {str(lqr_metrics['settling_time']):>14} {str(mpc_metrics['settling_time']):>14}")
    print(f"{'Overshoot (rad)':<35} {lqr_metrics['overshoot_rad']:>14.4f} {mpc_metrics['overshoot_rad']:>14.4f}")
    print(f"{'Max |force| (N)':<35} {lqr_metrics['max_force']:>14.3f} {mpc_metrics['max_force']:>14.3f}")
    print(f"{'Final theta (rad)':<35} {lqr_metrics['final_theta']:>14.6f} {mpc_metrics['final_theta']:>14.6f}")
    print(f"{'Final x (m)':<35} {lqr_metrics['final_x']:>14.6f} {mpc_metrics['final_x']:>14.6f}")
    print(f"{'Final cost J':<35} {lqr_metrics['cost'][-1]:>14.3f} {mpc_metrics['cost'][-1]:>14.3f}")
    print(f"{'Saturation steps':<35} {lqr_metrics['saturation_steps']:>14} {mpc_metrics['saturation_steps']:>14}")
    print(f"{'Saturation duration (s)':<35} {lqr_metrics['saturation_duration']:>14.3f} {mpc_metrics['saturation_duration']:>14.3f}")
    print(f"{'Saturation %':<35} {lqr_metrics['saturation_pct']:>14.1f} {mpc_metrics['saturation_pct']:>14.1f}")
    print(f"{'Stable?':<35} {str(lqr_metrics['is_stable']):>14} {str(mpc_metrics['is_stable']):>14}")
    print(f"{'Mean solve time (ms)':<35} {'—':>14} {mpc_metrics['mean_solve_time_ms']:>14.3f}")
    print(f"{'Max solve time (ms)':<35} {'—':>14} {mpc_metrics['max_solve_time_ms']:>14.3f}")
    print("=================================================================\n")


if __name__ == "__main__":
    xml_path = _repo_root / "mpc" / "models" / "cartpole.xml"
    out_dir = _repo_root / "mpc" / "outputs"
    out_dir.mkdir(exist_ok=True)

    duration = 5.0
    theta0 = 0.3
    N = 20
    u_min, u_max = -5.0, 5.0

    print(f"Running LQR with saturation (theta0={theta0} rad, bounds=[{u_min}, {u_max}])...")
    lqr_metrics = run_lqr_saturated(xml_path, duration=duration, theta0=theta0, u_min=u_min, u_max=u_max)

    print(f"Running MPC with constraints (theta0={theta0} rad, N={N}, bounds=[{u_min}, {u_max}])...")
    mpc_metrics = run_mpc_constrained(xml_path, duration=duration, theta0=theta0, N=N, u_min=u_min, u_max=u_max)

    print_phase3_comparison(lqr_metrics, mpc_metrics)

    plot_phase3_comparison(
        lqr_metrics,
        mpc_metrics,
        u_min=u_min,
        u_max=u_max,
        save_path=out_dir / "phase3_constraint_comparison.png",
    )
