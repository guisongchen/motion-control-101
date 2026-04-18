"""
Phase 3.2: State constraint experiment — MPC with |theta| <= 0.5 rad.

Compares:
1. MPC with control bounds only (±5 N)
2. MPC with control bounds + state constraint on theta (±0.5 rad at all prediction steps)

Initial condition: theta0 = 0.4 rad (large enough to trigger state constraint in prediction).
"""

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


def _is_stable(theta_hist: np.ndarray, threshold: float = 0.8) -> bool:
    return bool(np.all(np.abs(theta_hist) < threshold))


def run_mpc(
    xml_path: str | Path,
    duration: float = 5.0,
    theta0: float = 0.4,
    N: int = 20,
    u_min: float = -5.0,
    u_max: float = 5.0,
    state_constraints: list[dict] | None = None,
    label: str = "MPC",
    render: bool = False,
) -> dict:
    """Generic MPC closed-loop runner."""
    xml_path = Path(xml_path)
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)

    dt = model.opt.timestep
    assert np.isclose(dt, 0.01)

    lqr = CartPoleLQR(dt=dt)
    mpc = CartPoleMPC(
        Ad=lqr.Ad,
        Bd=lqr.Bd,
        N=N,
        P=lqr.P,
        u_min=u_min,
        u_max=u_max,
        state_constraints=state_constraints,
    )

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
    # Track predicted theta max at each timestep
    pred_theta_max_hist = np.zeros(n_steps)
    pred_theta_min_hist = np.zeros(n_steps)

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

        # Record predicted theta range from open-loop prediction
        X_pred = info["X_pred"]
        thetas_pred = X_pred[2::4]  # theta component of each predicted state
        pred_theta_max_hist[k] = np.max(thetas_pred)
        pred_theta_min_hist[k] = np.min(thetas_pred)

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
    sat_steps = int(np.sum(np.abs(u_hist) >= bound - 1e-6))

    return {
        "label": label,
        "time": t_hist,
        "state": x_hist,
        "control": u_hist,
        "cost": J_hist,
        "solve_time_ms": solve_time_hist,
        "iter": iter_hist,
        "pred_theta_max": pred_theta_max_hist,
        "pred_theta_min": pred_theta_min_hist,
        "settling_time": settling_time,
        "overshoot_rad": overshoot,
        "max_force": np.max(np.abs(u_hist)),
        "final_theta": theta[-1],
        "final_x": x_hist[-1, 0],
        "saturation_steps": sat_steps,
        "saturation_duration": sat_steps * dt,
        "saturation_pct": 100.0 * sat_steps / n_steps,
        "is_stable": _is_stable(theta),
        "mean_solve_time_ms": np.mean(solve_time_hist),
        "max_solve_time_ms": np.max(solve_time_hist),
    }


def plot_comparison(
    baseline: dict,
    constrained: dict,
    theta_bound: float = 0.5,
    save_path: str | Path | None = None,
):
    """Overlay MPC without vs with state constraints."""
    t = baseline["time"]

    fig, axes = plt.subplots(4, 1, figsize=(10, 12), sharex=True)

    # State trajectory
    ax = axes[0]
    ax.plot(t, baseline["state"][:, 0], label=f"{baseline['label']} cart x", lw=1.5, color="C0", ls="-")
    ax.plot(t, constrained["state"][:, 0], label=f"{constrained['label']} cart x", lw=1.5, color="C0", ls="--")
    ax.plot(t, baseline["state"][:, 2], label=f"{baseline['label']} theta", lw=1.5, color="C1", ls="-")
    ax.plot(t, constrained["state"][:, 2], label=f"{constrained['label']} theta", lw=1.5, color="C1", ls="--")
    ax.axhline(0.01, color="gray", ls="--", lw=0.8)
    ax.axhline(-0.01, color="gray", ls="--", lw=0.8)
    ax.set_ylabel("state")
    ax.legend(loc="upper right", ncol=2)
    ax.set_title(f"Phase 3.2: MPC Control-only vs MPC with State Constraint |theta| <= {theta_bound} rad")
    ax.grid(True, alpha=0.3)

    # Control trajectory with saturation bands
    ax = axes[1]
    ax.plot(t, baseline["control"], label=f"{baseline['label']} force", lw=1.5, color="C2", ls="-")
    ax.plot(t, constrained["control"], label=f"{constrained['label']} force", lw=1.5, color="C2", ls="--")
    ax.axhline(5.0, color="red", ls=":", lw=1.0, alpha=0.7, label="saturation ±5 N")
    ax.axhline(-5.0, color="red", ls=":", lw=1.0, alpha=0.7)
    ax.set_ylabel("control [N]")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    # Predicted theta range (open-loop at each timestep)
    ax = axes[2]
    ax.fill_between(t, baseline["pred_theta_min"], baseline["pred_theta_max"], color="C1", alpha=0.2, label="predicted theta range (control-only)")
    ax.fill_between(t, constrained["pred_theta_min"], constrained["pred_theta_max"], color="C4", alpha=0.2, label="predicted theta range (state-constrained)")
    ax.plot(t, baseline["state"][:, 2], color="C1", lw=1.0, ls="-", label="actual theta (control-only)")
    ax.plot(t, constrained["state"][:, 2], color="C4", lw=1.0, ls="--", label="actual theta (state-constrained)")
    ax.axhline(theta_bound, color="red", ls=":", lw=1.0, alpha=0.7, label=f"state bound ±{theta_bound} rad")
    ax.axhline(-theta_bound, color="red", ls=":", lw=1.0, alpha=0.7)
    ax.set_ylabel("theta [rad]")
    ax.legend(loc="upper right", ncol=2)
    ax.grid(True, alpha=0.3)

    # Cumulative cost
    ax = axes[3]
    ax.plot(t, baseline["cost"], label=f"{baseline['label']} J", lw=1.5, color="C3", ls="-")
    ax.plot(t, constrained["cost"], label=f"{constrained['label']} J", lw=1.5, color="C3", ls="--")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("cumulative cost")
    ax.legend(loc="lower right")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()

    if save_path is not None:
        plt.savefig(save_path, dpi=150)
        print(f"Phase 3.2 comparison plot saved to {save_path}")
    else:
        plt.show()


def print_comparison(baseline: dict, constrained: dict):
    print("\n=== Phase 3.2 Comparison: Control-only MPC vs State-constrained MPC ===")
    print(f"{'Metric':<35} {baseline['label']:>14} {constrained['label']:>14}")
    print("-" * 63)
    print(f"{'Settling time (s)':<35} {str(baseline['settling_time']):>14} {str(constrained['settling_time']):>14}")
    print(f"{'Overshoot (rad)':<35} {baseline['overshoot_rad']:>14.4f} {constrained['overshoot_rad']:>14.4f}")
    print(f"{'Max |force| (N)':<35} {baseline['max_force']:>14.3f} {constrained['max_force']:>14.3f}")
    print(f"{'Final theta (rad)':<35} {baseline['final_theta']:>14.6f} {constrained['final_theta']:>14.6f}")
    print(f"{'Final x (m)':<35} {baseline['final_x']:>14.6f} {constrained['final_x']:>14.6f}")
    print(f"{'Final cost J':<35} {baseline['cost'][-1]:>14.3f} {constrained['cost'][-1]:>14.3f}")
    print(f"{'Saturation steps':<35} {baseline['saturation_steps']:>14} {constrained['saturation_steps']:>14}")
    print(f"{'Saturation duration (s)':<35} {baseline['saturation_duration']:>14.3f} {constrained['saturation_duration']:>14.3f}")
    print(f"{'Stable?':<35} {str(baseline['is_stable']):>14} {str(constrained['is_stable']):>14}")
    print(f"{'Mean solve time (ms)':<35} {baseline['mean_solve_time_ms']:>14.3f} {constrained['mean_solve_time_ms']:>14.3f}")
    print(f"{'Max solve time (ms)':<35} {baseline['max_solve_time_ms']:>14.3f} {constrained['max_solve_time_ms']:>14.3f}")
    print("=====================================================================\n")


if __name__ == "__main__":
    xml_path = _repo_root / "mpc" / "models" / "cartpole.xml"
    out_dir = _repo_root / "mpc" / "outputs"
    out_dir.mkdir(exist_ok=True)

    duration = 5.0
    theta0 = 0.4
    N = 20
    u_min, u_max = -5.0, 5.0
    theta_bound = 0.5

    print(f"Running MPC control-only (theta0={theta0} rad, N={N})...")
    baseline = run_mpc(
        xml_path,
        duration=duration,
        theta0=theta0,
        N=N,
        u_min=u_min,
        u_max=u_max,
        state_constraints=None,
        label="MPC-control",
    )

    print(f"Running MPC with state constraint |theta|<={theta_bound} (theta0={theta0} rad, N={N})...")
    state_constraints = [
        {"idx": 2, "lb": -theta_bound, "ub": theta_bound, "steps": "all"},
    ]
    constrained = run_mpc(
        xml_path,
        duration=duration,
        theta0=theta0,
        N=N,
        u_min=u_min,
        u_max=u_max,
        state_constraints=state_constraints,
        label="MPC-state-con",
    )

    print_comparison(baseline, constrained)

    plot_comparison(
        baseline,
        constrained,
        theta_bound=theta_bound,
        save_path=out_dir / "phase3_2_state_constraint_comparison.png",
    )
