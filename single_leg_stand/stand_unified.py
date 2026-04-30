"""Unified single-leg stand experiment — clean redesign.

Phases:
  SETTLE  — hold double-support posture (PD settles physics)
  SHIFT   — smooth CoM transition from midpoint to above support foot
  HOLD    — maintain single-support balance

Uses a unified WBC (wbc_v2) with hard contact no-slip constraints
and Jc_dot finite-difference, eliminating slip.
"""

from __future__ import annotations

import numpy as np
import mujoco

from config import DT_SIM, GRAVITY, SLIP_NET_THRESH, SLIP_PEAK_THRESH, RMSE_THRESH
from robot_model import RobotModel
from robots.unitree_g1 import g1_config
from state_estimator import StateEstimator
from wbc import WholeBodyController
from utils import compute_rmse, compute_pd_torque, plot_diagnostics

from enum import IntEnum


# ---------------------------------------------------------------------------
# Configuration (self-contained, no PHASE_CONFIG dict)
# ---------------------------------------------------------------------------

SETTLE_TIME = 1.0
SHIFT_TIME = 5.0
SIM_DURATION = 20.0

KP_POSTURE = 220.0
KD_POSTURE = 36.0
KP_SWING = 180.0
KD_SWING = 15.0
MIN_CONTACT_FORCE = 15.0
MAX_ROLL_DELTA = 0.08  # modest lean angle

# Swing-lift joint targets (radians)
SWING_HIP_PITCH_TARGET = 0.40
SWING_KNEE_TARGET = 1.10
SWING_ANKLE_PITCH_TARGET = -0.15


# ---------------------------------------------------------------------------
# Helper: smooth quintic
# ---------------------------------------------------------------------------

def _quintic(s: float) -> float:
    s = float(np.clip(s, 0.0, 1.0))
    return 10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5


def _quintic_deriv(s: float, T: float) -> tuple[float, float, float]:
    """Derivative helpers for quintic trajectory: velocity, acceleration factors."""
    s = float(np.clip(s, 0.0, 1.0))
    ds = (30.0 * s**2 - 60.0 * s**3 + 30.0 * s**4) / T
    dds = (60.0 * s - 180.0 * s**2 + 120.0 * s**3) / T**2
    return ds, dds


# ---------------------------------------------------------------------------
# Phase enum
# ---------------------------------------------------------------------------

class Phase(IntEnum):
    SETTLE = 0
    SHIFT = 1
    HOLD = 2


# ---------------------------------------------------------------------------
# Foot geometry helpers
# ---------------------------------------------------------------------------

def _get_foot_corner_local_positions(robot: RobotModel, link: int) -> list[np.ndarray]:
    """Return the 4 foot-sole corners in the body-local frame, sorted."""
    local_positions: list[np.ndarray] = []
    has_box = False
    box_local = None
    box_size = None
    for geom_id in range(robot.model.ngeom):
        if int(robot.model.geom_bodyid[geom_id]) != link:
            continue
        gt = int(robot.model.geom_type[geom_id])
        if gt == mujoco.mjtGeom.mjGEOM_SPHERE:
            local_positions.append(np.array(robot.model.geom_pos[geom_id], copy=True))
        elif gt == mujoco.mjtGeom.mjGEOM_BOX:
            has_box = True
            box_local = np.array(robot.model.geom_pos[geom_id], copy=True)
            box_size = np.array(robot.model.geom_size[geom_id], copy=True)
    if has_box and box_local is not None and box_size is not None:
        hx, hy, hz = box_size
        cx, cy, cz = box_local
        return sorted(
            [
                np.array([cx - hx, cy - hy, cz - hz]),
                np.array([cx - hx, cy + hy, cz - hz]),
                np.array([cx + hx, cy - hy, cz - hz]),
                np.array([cx + hx, cy + hy, cz - hz]),
            ],
            key=lambda p: (p[0], p[1]),
        )
    return sorted(local_positions, key=lambda p: (p[0], p[1]))


def _compute_corner_force_ref(
    corners_local: list[np.ndarray],
    foot_rotation: np.ndarray,
    cop_local: np.ndarray,
    total_fz: float,
) -> np.ndarray:
    """Distribute total_fz across 4 corners to realize the desired CoP."""
    if len(corners_local) != 4:
        n = max(len(corners_local), 1)
        return np.tile(np.array([0.0, 0.0, total_fz / n]), n)

    xs = np.array([p[0] for p in corners_local])
    ys = np.array([p[1] for p in corners_local])
    mx = 0.01  # margin from edge
    x_alpha = np.clip((cop_local[0] - (xs.min() + mx)) / max(xs.max() - xs.min() - 2 * mx, 1e-6), 0.0, 1.0)
    y_alpha = np.clip((cop_local[1] - (ys.min() + mx)) / max(ys.max() - ys.min() - 2 * mx, 1e-6), 0.0, 1.0)
    w = np.array([
        (1 - x_alpha) * (1 - y_alpha),
        (1 - x_alpha) * y_alpha,
        x_alpha * (1 - y_alpha),
        x_alpha * y_alpha,
    ])
    w /= w.sum()
    f_ref = np.zeros(12)
    for i in range(4):
        f_ref[3 * i + 2] = w[i] * total_fz
    return f_ref


# ---------------------------------------------------------------------------
# SimLog (simplified)
# ---------------------------------------------------------------------------

class SimLog:
    def __init__(self, foot_links):
        self.foot_links = foot_links
        self.t: list[float] = []
        self.com: list[np.ndarray] = []
        self.com_ref: list[np.ndarray] = []
        self.phase: list[int] = []
        self.foot_force: dict[int, list[float]] = {l: [] for l in foot_links}
        self.foot_slip: dict[int, list[float]] = {l: [] for l in foot_links}
        self.support_ratio: list[float] = []
        self.L: list[np.ndarray] = []
        self.cop_y: list[float] = []
        self.roll_delta: list[float] = []
        self.swing_force: list[float] = []
        self.support_force: list[float] = []


# ---------------------------------------------------------------------------
# Results printer
# ---------------------------------------------------------------------------

def _print_results(log: SimLog, robot: RobotModel, initial_foot_pos: dict):
    print(f"\n{'='*60}")
    print("===== RESULTS =====")
    rmse = compute_rmse(log.com, log.com_ref)
    com_arr = np.stack(log.com)
    com_ref_arr = np.stack(log.com_ref)
    max_dev = float(np.max(np.linalg.norm(com_arr - com_ref_arr, axis=1)))
    print(f"CoM RMSE: {rmse:.4f} m (target < {RMSE_THRESH} m)")
    print(f"CoM max deviation: {max_dev:.4f} m")

    max_slip = 0.0
    net_slip = 0.0
    for link in log.foot_links:
        slip_arr = np.array(log.foot_slip[link])
        non_nan = slip_arr[~np.isnan(slip_arr)]
        if len(non_nan) > 0:
            max_slip = max(max_slip, float(np.max(non_nan)))
        try:
            final_pos = robot.get_contact_metrics(link)["position"]
            net = np.linalg.norm(final_pos[:2] - initial_foot_pos[link][:2])
            net_slip = max(net_slip, net)
        except Exception:
            pass
    print(f"Max peak slip: {max_slip * 1000:.2f} mm (target < {SLIP_PEAK_THRESH * 1000:.1f} mm)")
    print(f"Max net displacement: {net_slip * 1000:.2f} mm (target < {SLIP_NET_THRESH * 1000:.1f} mm)")

    for link in log.foot_links:
        forces = np.array(log.foot_force[link])
        avg = np.mean(forces) if len(forces) > 0 else 0.0
        name = robot.link_to_foot_name.get(link, str(link))
        print(f"  {name} avg force: {avg:.1f} N")

    max_phase = max(log.phase)
    print(f"\nHighest phase reached: {Phase(max_phase).name}")

    reached_hold = max_phase >= Phase.HOLD
    passed = reached_hold and rmse < RMSE_THRESH and max_slip < SLIP_PEAK_THRESH and net_slip < SLIP_NET_THRESH
    print(f"Verdict: {'PASS' if passed else 'FAIL'}")
    if not passed:
        reasons = []
        if not reached_hold:
            reasons.append("did not reach HOLD phase")
        if rmse >= RMSE_THRESH:
            reasons.append(f"RMSE {rmse:.4f} >= {RMSE_THRESH}")
        if max_slip >= SLIP_PEAK_THRESH:
            reasons.append(f"peak slip {max_slip * 1000:.1f}mm >= {SLIP_PEAK_THRESH * 1000:.0f}mm")
        if net_slip >= SLIP_NET_THRESH:
            reasons.append(f"net slip {net_slip * 1000:.1f}mm >= {SLIP_NET_THRESH * 1000:.0f}mm")
        print(f"  Failures: {'; '.join(reasons)}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    # --- Robot setup ---
    robot = RobotModel(g1_config)
    initial_dof_angles = robot.set_standing_angles(g1_config.standing_joint_angles)

    support_leg = "right"
    swing_leg = "left"
    support_link = robot.foot_name_to_link[g1_config.support_foot_name]
    swing_link = robot.foot_name_to_link[g1_config.swing_foot_name]
    foot_links = robot.foot_link_ids

    tau_max_limits = robot.tau_limits.copy()
    tau_min_limits = -tau_max_limits

    joint_name_to_dof_idx = robot.dof_joint_name_to_index
    swing_dof_indices = np.array([
        joint_name_to_dof_idx[n]
        for n in g1_config.leg_joint_names[swing_leg]
        if n in joint_name_to_dof_idx
    ], dtype=int)

    nv_base = robot.model.jnt_dofadr[1] if robot.model.njnt > 1 else robot.model.nv
    nq_base = robot.model.jnt_qposadr[1] if robot.model.njnt > 1 else robot.model.nq

    # Initial foot positions (for slip measurement)
    initial_foot_pos = {
        link: np.array(robot.get_contact_metrics(link)["position"], copy=True)
        for link in foot_links
    }

    # Geometry
    measured_com = robot.compute_com_position()
    standing_com_z = float(measured_com[2])
    support_xy = initial_foot_pos[support_link][:2]
    swing_xy = initial_foot_pos[swing_link][:2]
    stance_midpoint = 0.5 * (support_xy + swing_xy)
    support_offset = support_xy - stance_midpoint
    support_dir = support_offset / max(np.linalg.norm(support_offset), 1e-6)
    support_half_stance = float(np.linalg.norm(support_offset))

    # Foot corner geometries (body-local)
    support_corners_local = _get_foot_corner_local_positions(robot, support_link)
    swing_corners_local = _get_foot_corner_local_positions(robot, swing_link)

    # Total mass (for force reference)
    total_mass = robot.total_mass
    total_weight = -GRAVITY[2] * total_mass

    # --- Controller ---
    wbc_ss = WholeBodyController(robot.nv, num_contacts=4, contact_dim=3,
                                   kp_c=300.0, kd_c=60.0)
    estimator = StateEstimator(robot)

    # --- Phase state ---
    phase = Phase.SETTLE
    phase_start_time = 0.0

    # --- Trajectory ---
    c_ref = np.zeros(3)
    c_ref[:2] = stance_midpoint.copy()
    c_ref[2] = standing_com_z

    # --- Simulation ---
    total_steps = int(SIM_DURATION / DT_SIM)
    log = SimLog(foot_links)

    print(f"\n===== Unified Stand Experiment (duration={SIM_DURATION:.1f}s) =====")
    print(f"Support: {support_leg}, Swing: {swing_leg}")
    print(f"Phases: SETTLE -> SHIFT -> HOLD")
    print("=" * 60)

    for step in range(total_steps):
        t = step * DT_SIM

        # --- State estimation ---
        state = estimator.update(lock_support=(phase >= Phase.SHIFT))
        c = state["c"]
        c_dot = state["c_dot"]
        L = state["L"]
        q = state["q"]
        v = state["v"]
        joint_positions = q[nq_base:]
        joint_velocities = v[nv_base:]
        foot_contacts = state["foot_contacts"]

        elapsed = t - phase_start_time

        # --- Phase transitions ---
        if phase == Phase.SETTLE and elapsed >= SETTLE_TIME:
            phase = Phase.SHIFT
            phase_start_time = t
            print(f"[PHASE] t={t:.2f}s -> SHIFT")
        elif phase == Phase.SHIFT and elapsed >= SHIFT_TIME:
            phase = Phase.HOLD
            phase_start_time = t
            print(f"[PHASE] t={t:.2f}s -> HOLD")

        # --- Contact detection ---
        contact_forces: dict[int, float] = {}
        contact_positions: dict[int, np.ndarray] = {}
        for fc in foot_contacts:
            contact_forces[fc["link"]] = fc["normal_force"]
            contact_positions[fc["link"]] = fc["position"]

        support_in_contact = contact_forces.get(support_link, 0.0) > MIN_CONTACT_FORCE
        swing_in_contact = contact_forces.get(swing_link, 0.0) > MIN_CONTACT_FORCE

        # --- Compute c_ref ---
        if phase == Phase.SETTLE:
            c_ref[:2] = stance_midpoint
        elif phase == Phase.SHIFT:
            s = elapsed / SHIFT_TIME
            # Shift com_shift_ratio from 0 (midpoint) to 0.85 (over support foot with lean)
            target_ratio = _quintic(s) * 0.85
            c_ref[:2] = stance_midpoint + target_ratio * support_dir * support_half_stance
        else:  # HOLD
            c_ref[:2] = stance_midpoint + 0.85 * support_dir * support_half_stance

        c_ref[2] = standing_com_z

        # --- Compute roll_delta (lean) ---
        if phase >= Phase.SHIFT:
            if phase == Phase.SHIFT:
                s = elapsed / max(SHIFT_TIME, 1e-6)
                roll_delta = _quintic(s) * MAX_ROLL_DELTA
            else:
                roll_delta = MAX_ROLL_DELTA
        else:
            roll_delta = 0.0

        # --- Safe PD posture torque (with roll lean) ---
        C_safe = robot.compute_coriolis_gravity(q, v)
        safe_targets = initial_dof_angles.copy()
        if roll_delta != 0.0:
            for leg in ("left", "right"):
                sign = 1.0 if leg == support_leg else -1.0
                for suffix in ("hip_roll_joint", "ankle_roll_joint"):
                    name = f"{leg}_{suffix}"
                    if name in joint_name_to_dof_idx:
                        idx = joint_name_to_dof_idx[name]
                        safe_targets[idx] += sign * roll_delta

        safe_tau = C_safe[nv_base:] + compute_pd_torque(
            safe_targets, joint_positions, joint_velocities,
            KP_POSTURE, KD_POSTURE, tau_max_limits,
        )

        # --- Build torque command ---
        if phase in (Phase.SETTLE, Phase.SHIFT):
            # Pure PD posture — reliable and stable for quasi-static weight shift
            tau_cmd = safe_tau.copy()
            result = None
        else:
            # HOLD: use WBC for dynamic single-support balancing, blended with posture
            c_ddot_des = wbc_ss.compute_desired_acceleration(c_ref, c, np.zeros(3), c_dot, np.zeros(3))
            L_dot_des = wbc_ss.compute_desired_momentum_rate(np.zeros(3), L, np.zeros(3), np.zeros(3))

            # Build support-foot contact Jacobian (4 corner points)
            Jc_blocks = []
            f_ref_blocks = []
            sup_rot = np.array(robot.data.xmat[support_link]).reshape(3, 3)
            cop_local = sup_rot.T @ (state["p_foot"] - np.array(robot.data.xpos[support_link]))
            sup_f_ref = _compute_corner_force_ref(support_corners_local, sup_rot, cop_local, total_weight)
            for lp in support_corners_local:
                Jc_blocks.append(robot.get_foot_jacobian(support_link, q, local_position=lp)[:3])
            f_ref_blocks.extend(sup_f_ref.tolist())

            J_c_full = np.vstack(Jc_blocks)
            f_ref = np.array(f_ref_blocks, dtype=float)

            M = robot.compute_mass_matrix(q)
            C_vec = robot.compute_coriolis_gravity(q, v)
            J_com = robot.get_com_jacobian(q)
            J_L = robot.get_angular_momentum_jacobian(q)

            result = wbc_ss.solve(
                M=M, C=C_vec, J_c=J_c_full, Jc_dot=np.zeros_like(J_c_full),
                J_com=J_com, J_L=J_L,
                c_ddot_des=c_ddot_des, L_dot_des=L_dot_des,
                f_ref=f_ref, v=v,
                tau_min=tau_min_limits, tau_max=tau_max_limits,
                slip_weight=50000.0,
            )

            if result is not None:
                wbc_tau = np.clip(result["tau"], tau_min_limits, tau_max_limits)
                blend = 0.30  # 70% WBC, 30% posture
                tau_cmd = (1.0 - blend) * wbc_tau + blend * safe_tau
            else:
                tau_cmd = safe_tau.copy()

        # --- Swing leg PD override (only during HOLD) ---
        if phase == Phase.HOLD and len(swing_dof_indices) > 0:
            swing_targets = initial_dof_angles[swing_dof_indices].copy()
            progress = _quintic(min(1.0, elapsed / 1.5))

            hip_name = f"{swing_leg}_hip_pitch_joint"
            knee_name = f"{swing_leg}_knee_joint"
            ank_name = f"{swing_leg}_ankle_pitch_joint"
            if hip_name in joint_name_to_dof_idx:
                sw_idx = joint_name_to_dof_idx[hip_name] - nv_base
                swing_targets[sw_idx] += progress * (SWING_HIP_PITCH_TARGET - initial_dof_angles[joint_name_to_dof_idx[hip_name]])
            if knee_name in joint_name_to_dof_idx:
                sw_idx = joint_name_to_dof_idx[knee_name] - nv_base
                swing_targets[sw_idx] += progress * (SWING_KNEE_TARGET - initial_dof_angles[joint_name_to_dof_idx[knee_name]])
            if ank_name in joint_name_to_dof_idx:
                sw_idx = joint_name_to_dof_idx[ank_name] - nv_base
                swing_targets[sw_idx] += progress * (SWING_ANKLE_PITCH_TARGET - initial_dof_angles[joint_name_to_dof_idx[ank_name]])

            swing_tau = compute_pd_torque(
                swing_targets, joint_positions[swing_dof_indices],
                joint_velocities[swing_dof_indices],
                KP_SWING, KD_SWING, tau_max_limits[swing_dof_indices],
            )
            tau_cmd[swing_dof_indices] = swing_tau

        tau_cmd = np.clip(tau_cmd, tau_min_limits, tau_max_limits)

        # --- Apply and step ---
        robot.set_joint_torques(tau_cmd)
        robot.step()

        # --- Logging ---
        log.t.append(t)
        log.com.append(c.copy())
        log.com_ref.append(c_ref.copy())
        log.phase.append(int(phase))
        log.L.append(L.copy())
        log.roll_delta.append(roll_delta)
        log.cop_y.append(0.0)
        for fc in foot_contacts:
            log.foot_force[fc["link"]].append(fc["normal_force"])
        cf = {fc["link"]: fc["normal_force"] for fc in foot_contacts}
        log.support_force.append(cf.get(support_link, 0.0))
        log.swing_force.append(cf.get(swing_link, 0.0))
        for link in foot_links:
            if cf.get(link, 0.0) > 10.0:
                cur = robot.get_contact_metrics(link)["position"]
                log.foot_slip[link].append(float(np.linalg.norm(cur[:2] - initial_foot_pos[link][:2])))
            else:
                log.foot_slip[link].append(np.nan)
        sf = cf.get(support_link, 0.0)
        wf = cf.get(swing_link, 0.0)
        r = sf / max(sf + wf, 1e-6)
        log.support_ratio.append(r)

        # --- Periodic diagnostics ---
        if step % 500 == 0:
            wbc_ok = "OK" if (result is not None) else ("PD" if phase != Phase.HOLD else "FAIL")
            print(f"[t={t:.2f}s] {Phase(phase).name:8s} "
                  f"sf={cf.get(support_link,0):.0f}N wf={cf.get(swing_link,0):.0f}N "
                  f"ratio={r:.3f} c_ref_y={c_ref[1]:.4f} c_y={c[1]:.4f} "
                  f"wbc={wbc_ok} rd={roll_delta:.3f}")

    # --- Results ---
    _print_results(log, robot, initial_foot_pos)

    plot_diagnostics(
        log, robot.link_to_foot_name, support_link, swing_link,
        rmse_thresh=RMSE_THRESH, slip_net_thresh=SLIP_NET_THRESH, slip_peak_thresh=SLIP_PEAK_THRESH,
        ds_min_force=MIN_CONTACT_FORCE,
        load_shift_roll_delta=0.12,
        ds_max_L_norm=0.5,
    )


if __name__ == "__main__":
    main()
