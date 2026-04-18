"""
Phase 4: Prediction horizon N trade-off analysis.

Sweep N = [5, 10, 20, 40, 80] on MPC with control bounds (±5 N).
Initial condition: theta0 = 0.1 rad.

Records per-N metrics:
- Average / max QP solve time
- Settling time
- Cumulative cost J
- Control smoothness (sum of squared du)

Generates:
1. State/control trajectory overlay for all N values
2. Dual-axis sweep plot: N vs (avg solve time, cumulative cost)
3. QP solve time histograms (per N and combined)
"""

import sys
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import mujoco

_repo_root = Path(__file__).parent.parent
sys.path.insert(0, str(_repo_root))

from mpc.controllers.lqr import CartPoleLQR
from mpc.controllers.mpc import CartPoleMPC


def run_mpc(
    xml_path: str | Path,
    duration: float = 5.0,
    theta0: float = 0.1,
    N: int = 20,
    u_min: float = -5.0,
    u_max: float = 5.0,
    label: str = "MPC",
) -> dict:
    """Run single MPC closed-loop simulation and return metrics."""
    xml_path = Path(xml_path)
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)

    dt = model.opt.timestep
    assert np.isclose(dt, 0.01)

    lqr = CartPoleLQR(dt=dt)
    mpc = CartPoleMPC(
        Ad=lqr.Ad, Bd=lqr.Bd, N=N, P=lqr.P, u_min=u_min, u_max=u_max
    )
    mpc.reset()

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

    J_cum = 0.0
    for k in range(n_steps):
        state = np.array([
            data.qpos[0],   # x
            data.qvel[0],   # x_dot
            data.qpos[1],   # theta
            data.qvel[1],   # theta_dot
        ])

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

    du = np.diff(u_hist)
    smoothness = float(np.sum(du ** 2))

    bound = min(abs(u_min), abs(u_max))
    sat_steps = int(np.sum(np.abs(u_hist) >= bound - 1e-6))

    return {
        "label": label,
        "N": N,
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
        "saturation_pct": 100.0 * sat_steps / n_steps,
        "mean_solve_time_ms": np.mean(solve_time_hist),
        "max_solve_time_ms": np.max(solve_time_hist),
        "std_solve_time_ms": np.std(solve_time_hist),
        "smoothness": smoothness,
    }


def plot_trajectories(results: list[dict], save_path: str | Path | None = None):
    """Overlay state/control trajectories for each N."""
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

    # Theta trajectory
    ax = axes[0]
    for r in results:
        ax.plot(r["time"], r["state"][:, 2], label=f"N={r['N']}", lw=1.5)
    ax.axhline(0.01, color="gray", ls="--", lw=0.8)
    ax.axhline(-0.01, color="gray", ls="--", lw=0.8)
    ax.set_ylabel("pole angle theta [rad]")
    ax.set_title("Phase 4: Effect of Prediction Horizon N on Trajectory")
    ax.legend(loc="upper right", ncol=3, fontsize=8)
    ax.grid(True, alpha=0.3)

    # Cart position
    ax = axes[1]
    for r in results:
        ax.plot(r["time"], r["state"][:, 0], label=f"N={r['N']}", lw=1.5)
    ax.set_ylabel("cart position x [m]")
    ax.legend(loc="upper right", ncol=3, fontsize=8)
    ax.grid(True, alpha=0.3)

    # Control
    ax = axes[2]
    for r in results:
        ax.plot(r["time"], r["control"], label=f"N={r['N']}", lw=1.5)
    ax.axhline(5.0, color="red", ls=":", lw=1.0, alpha=0.7)
    ax.axhline(-5.0, color="red", ls=":", lw=1.0, alpha=0.7)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("control [N]")
    ax.legend(loc="upper right", ncol=3, fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_path is not None:
        plt.savefig(save_path, dpi=150)
        print(f"Trajectory overlay saved to {save_path}")
    else:
        plt.show()


def plot_n_sweep(results: list[dict], lqr_cost: float, save_path: str | Path | None = None):
    """Dual-axis plot: N vs (avg solve time, cumulative cost)."""
    Ns = [r["N"] for r in results]
    avg_times = [r["mean_solve_time_ms"] for r in results]
    max_times = [r["max_solve_time_ms"] for r in results]
    costs = [r["cost"][-1] for r in results]

    fig, ax1 = plt.subplots(figsize=(8, 5))

    color_time = "C0"
    ax1.set_xlabel("Prediction Horizon N")
    ax1.set_ylabel("Avg QP Solve Time [ms]", color=color_time)
    ax1.plot(Ns, avg_times, color=color_time, marker="o", lw=2, label="Avg solve time")
    ax1.plot(Ns, max_times, color=color_time, marker="s", ls="--", lw=1.5, label="Max solve time")
    ax1.axhline(10.0, color="gray", ls=":", lw=1.0, label="Control period T=10 ms")
    ax1.tick_params(axis="y", labelcolor=color_time)
    ax1.set_xscale("log", base=2)
    ax1.set_xticks(Ns)
    ax1.set_xticklabels([str(n) for n in Ns])
    ax1.legend(loc="upper left")
    ax1.grid(True, alpha=0.3)

    ax2 = ax1.twinx()
    color_cost = "C1"
    ax2.set_ylabel("Cumulative Cost J", color=color_cost)
    ax2.plot(Ns, costs, color=color_cost, marker="D", lw=2, label="MPC cost")
    ax2.axhline(lqr_cost, color=color_cost, ls="--", lw=1.5, label="LQR cost (unconstrained)")
    ax2.tick_params(axis="y", labelcolor=color_cost)
    ax2.legend(loc="upper right")

    plt.title("Phase 4: N Trade-off — Solve Time vs Cumulative Cost")
    plt.tight_layout()

    if save_path is not None:
        plt.savefig(save_path, dpi=150)
        print(f"N sweep plot saved to {save_path}")
    else:
        plt.show()


def plot_solve_time_histograms(results: list[dict], save_path: str | Path | None = None):
    """Histogram of solve times for each N (subplots)."""
    n = len(results)
    cols = 3
    rows = (n + cols - 1) // cols

    fig, axes = plt.subplots(rows, cols, figsize=(12, 4 * rows))
    axes = np.array(axes).flatten()

    for idx, r in enumerate(results):
        ax = axes[idx]
        st = r["solve_time_ms"]
        ax.hist(st, bins=50, color=f"C{idx}", edgecolor="white", alpha=0.8)
        ax.axvline(np.mean(st), color="red", ls="--", lw=1.5, label=f"mean={np.mean(st):.3f} ms")
        ax.axvline(np.max(st), color="black", ls=":", lw=1.5, label=f"max={np.max(st):.3f} ms")
        ax.set_title(f"N={r['N']} (solve time dist)")
        ax.set_xlabel("solve time [ms]")
        ax.set_ylabel("count")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)

    # Hide unused subplots
    for idx in range(n, len(axes)):
        axes[idx].set_visible(False)

    plt.suptitle("Phase 4: QP Solve Time Distribution per N", y=1.02)
    plt.tight_layout()

    if save_path is not None:
        plt.savefig(save_path, dpi=150)
        print(f"Solve-time histograms saved to {save_path}")
    else:
        plt.show()


def print_table(results: list[dict], lqr_cost: float):
    print("\n=== Phase 4: N Trade-off Results ===")
    header = f"{'N':>4} {'Settle(s)':>10} {'Overshoot':>10} {'Max|u|':>8} {'J_final':>10} {'Sat%':>6} {'Smooth':>10} {'Avg_ms':>8} {'Max_ms':>8} {'Std_ms':>8}"
    print(header)
    print("-" * len(header))
    for r in results:
        print(
            f"{r['N']:>4} "
            f"{str(r['settling_time']):>10} "
            f"{r['overshoot_rad']:>10.4f} "
            f"{r['max_force']:>8.3f} "
            f"{r['cost'][-1]:>10.3f} "
            f"{r['saturation_pct']:>6.1f} "
            f"{r['smoothness']:>10.1f} "
            f"{r['mean_solve_time_ms']:>8.3f} "
            f"{r['max_solve_time_ms']:>8.3f} "
            f"{r['std_solve_time_ms']:>8.3f}"
        )
    print(f"\nLQR (unconstrained) cost J = {lqr_cost:.3f}")
    print("=====================================\n")


if __name__ == "__main__":
    xml_path = _repo_root / "mpc" / "models" / "cartpole.xml"
    out_dir = _repo_root / "mpc" / "outputs"
    out_dir.mkdir(exist_ok=True)

    duration = 5.0
    theta0 = 0.1
    N_values = [5, 10, 20, 40, 80]
    u_min, u_max = -5.0, 5.0

    # Run LQR baseline for cost comparison
    print("Running LQR baseline (unconstrained)...")
    from mpc.phase1_lqr_sim import run_lqr_simulation
    lqr_res = run_lqr_simulation(xml_path, duration=duration, theta0=theta0, render=False)
    lqr_cost = float(lqr_res["cost"][-1])

    results = []
    for N in N_values:
        print(f"Running MPC with N={N} (theta0={theta0} rad, bounds=[{u_min}, {u_max}])...")
        res = run_mpc(
            xml_path,
            duration=duration,
            theta0=theta0,
            N=N,
            u_min=u_min,
            u_max=u_max,
            label=f"MPC-N={N}",
        )
        results.append(res)

    print_table(results, lqr_cost)

    plot_trajectories(results, save_path=out_dir / "phase4_trajectories_overlay.png")
    plot_n_sweep(results, lqr_cost, save_path=out_dir / "phase4_n_sweep.png")
    plot_solve_time_histograms(results, save_path=out_dir / "phase4_solve_time_histograms.png")
