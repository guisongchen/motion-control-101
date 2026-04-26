"""Phase-by-phase single-leg stand experiment.

Phase 1: INIT_SETTLE + DOUBLE_SUPPORT_HOLD (stable stance only)
Phase 2: LOAD_SHIFT (weight transfer with corner-patch DS WBC)
Phase 3: PRE_LIFTOFF (continue transfer + unload swing foot)
Phase 4: SINGLE_SUPPORT (MPC + WBC balancing)

Each phase is unlocked only after the previous one is verified stable.
"""

from __future__ import annotations

import numpy as np
import mujoco

from config import (
    DT_SIM,
    GRAVITY,
    BASE_DOF_DAMPING,
    JOINT_DOF_DAMPING,
    MIN_SUPPORT_FORCE,
    RMSE_THRESH,
    SLIP_THRESH,
    NX,
    NU,
)
from robot_model import RobotModel
from robots.unitree_g1 import g1_config
from state_estimator import StateEstimator
from phase_metrics import compute_load_shift_metrics
from phase_core import (
    ControlPhase,
    StabilityGate,
    get_contact_entry,
    support_name,
    skew,
)
from mpc import CentroidalMPC
from wbc import WholeBodyController
from direct_single_support import (
    build_direct_pose,
    resolve_support_contact_local_positions,
    yaw_from_rotation,
    DIRECT_SINGLE_SUPPORT_CONFIG as direct_cfg,
)
from direct_single_support.primitives import (
    compute_corner_patch_force_reference,
    build_corner_patch_wrench_task,
    apply_measured_cop_feedback,
)
from utils import compute_rmse, compute_pd_torque, plot_com_tracking, _save_figure

import matplotlib
matplotlib.use("Agg")  # double-check backend
import matplotlib.pyplot as plt

import wbc as wbc_module

from enum import IntEnum


class PhaseId(IntEnum):
    INIT_SETTLE = 0
    DOUBLE_SUPPORT_HOLD = 1
    LOAD_SHIFT = 2
    PRE_LIFTOFF = 3
    SINGLE_SUPPORT = 4


# ---------------------------------------------------------------------------
# Phase-specific config (tuned incrementally)
# ---------------------------------------------------------------------------
PHASE_CONFIG = {
    "init_settle_time": 0.20,
    "ds_ready_time": 0.80,
    "ds_min_force": 80.0,
    "ds_max_com_vel": 0.08,
    "ds_max_L_norm": 0.50,
    "ds_force_ratio_min": 0.30,
    "ds_com_margin": 0.02,
    "load_shift_time": 1.50,
    "load_shift_target_ratio": 0.72,
    "load_shift_ratio_min": 0.55,
    "load_shift_com_ratio": 0.35,
    "load_shift_roll_delta": 0.07,
    "load_shift_swing_force_min": 25.0,
    "com_vel_ready_thresh": 0.35,
    "pre_liftoff_time": 1.00,
    "pre_liftoff_ratio_min": 0.60,
    "pre_liftoff_com_ratio": 0.55,
    "pre_liftoff_extra_roll_delta": 0.04,
    "pre_liftoff_swing_force_max": 150.0,
    "single_support_entry_time": 0.60,
    "single_support_hold_force": 15.0,
    "single_support_support_ratio": 0.90,
    "single_support_com_ratio": 0.70,
    "transition_blend_time": 0.15,
    "single_support_max_tau_blend": 0.85,
    "single_support_swing_ramp_time": 0.8,
    "single_support_swing_progress_max": 1.0,
    "swing_hip_pitch_target": 0.10,
    "swing_knee_target": 1.05,
    "swing_ankle_pitch_target": -0.55,
    "ds_force_weight_xy": 100.0,
    "ds_force_weight_z": 100.0,
    "ds_load_dist_weight": 3000.0,
    "ds_posture_blend": 0.50,
    "kp_c": 150.0,
    "kd_c": 30.0,
    "kp_L": 15.0,
    "kd_L": 3.0,
    "posture_kp": 120.0,
    "posture_kd": 16.0,
    "lift_leg_kp": 120.0,
    "lift_leg_kd": 8.0,
    "sim_duration": 15.0,
}


def smoothstep(t: float) -> float:
    t = float(np.clip(t, 0.0, 1.0))
    return t * t * (3.0 - 2.0 * t)


def compute_foot_corners_world_xy(robot: RobotModel, link: int) -> list[np.ndarray]:
    body_origin = np.array(robot.data.xpos[link], copy=True)
    body_rotation = np.array(robot.data.xmat[link]).reshape(3, 3)
    corners: list[np.ndarray] = []
    has_box = False
    box_local_pos = None
    box_size = None
    for geom_id in range(robot.model.ngeom):
        if int(robot.model.geom_bodyid[geom_id]) != link:
            continue
        geom_type = int(robot.model.geom_type[geom_id])
        if geom_type == mujoco.mjtGeom.mjGEOM_SPHERE:
            local_pos = np.array(robot.model.geom_pos[geom_id], copy=True)
            world_pos = body_origin + body_rotation @ local_pos
            corners.append(world_pos[:2])
        elif geom_type == mujoco.mjtGeom.mjGEOM_BOX:
            has_box = True
            box_local_pos = np.array(robot.model.geom_pos[geom_id], copy=True)
            box_size = np.array(robot.model.geom_size[geom_id], copy=True)
    if has_box and box_local_pos is not None and box_size is not None:
        hx, hy, _hz = box_size
        cx, cy, cz = box_local_pos
        local_corners = [
            np.array([cx - hx, cy - hy, cz - _hz]),
            np.array([cx - hx, cy + hy, cz - _hz]),
            np.array([cx + hx, cy - hy, cz - _hz]),
            np.array([cx + hx, cy + hy, cz - _hz]),
        ]
        return [(body_rotation @ lc)[:2] for lc in local_corners]
    return corners


def resolve_corner_local_positions(robot: RobotModel, link: int) -> list[np.ndarray]:
    local_positions: list[np.ndarray] = []
    has_box = False
    box_local_pos = None
    box_size = None
    for geom_id in range(robot.model.ngeom):
        if int(robot.model.geom_bodyid[geom_id]) != link:
            continue
        geom_type = int(robot.model.geom_type[geom_id])
        if geom_type == mujoco.mjtGeom.mjGEOM_SPHERE:
            local_positions.append(np.array(robot.model.geom_pos[geom_id], copy=True))
        elif geom_type == mujoco.mjtGeom.mjGEOM_BOX:
            has_box = True
            box_local_pos = np.array(robot.model.geom_pos[geom_id], copy=True)
            box_size = np.array(robot.model.geom_size[geom_id], copy=True)
    if has_box and box_local_pos is not None and box_size is not None:
        hx, hy, hz = box_size
        cx, cy, cz = box_local_pos
        corners_local = [
            np.array([cx - hx, cy - hy, cz - hz]),
            np.array([cx - hx, cy + hy, cz - hz]),
            np.array([cx + hx, cy - hy, cz - hz]),
            np.array([cx + hx, cy + hy, cz - hz]),
        ]
        return sorted(corners_local, key=lambda pos: (pos[0], pos[1]))
    return sorted(local_positions, key=lambda pos: (pos[0], pos[1]))


def apply_roll_shift_offset(targets, joint_name_to_dof_idx, support_leg, delta):
    support_sign = 1.0 if support_leg == "right" else -1.0
    roll_delta = support_sign * delta
    for leg in ("left", "right"):
        name = f"{leg}_hip_roll_joint"
        if name in joint_name_to_dof_idx:
            targets[joint_name_to_dof_idx[name]] += roll_delta
        name = f"{leg}_ankle_roll_joint"
        if name in joint_name_to_dof_idx:
            targets[joint_name_to_dof_idx[name]] -= roll_delta


def build_safe_targets(
    initial_dof_angles, joint_name_to_dof_idx, phase_id, phase_start_time, t,
    support_leg, load_shift_metrics,
):
    targets = initial_dof_angles.copy()
    cfg = PHASE_CONFIG
    swing_leg = "left" if support_leg == "right" else "right"
    roll_delta = 0.0

    if phase_id >= PhaseId.LOAD_SHIFT:
        ratio_error = max(0.0, cfg["load_shift_target_ratio"] - load_shift_metrics.support_ratio)
        com_error = max(0.0, cfg["load_shift_com_ratio"] - load_shift_metrics.com_shift_ratio)
        roll_feedback = cfg["load_shift_roll_delta"] * min(
            1.0,
            8.0 * (0.7 * ratio_error / max(cfg["load_shift_target_ratio"], 1e-6)
                    + 0.3 * com_error / max(cfg["load_shift_com_ratio"], 1e-6)),
        )
        slip_scale = max(0.0, 1.0 - load_shift_metrics.swing_slip / max(SLIP_THRESH, 1e-6))
        roll_delta = roll_feedback * slip_scale

        if phase_id == PhaseId.LOAD_SHIFT:
            progress = smoothstep((t - phase_start_time) / max(cfg["load_shift_time"], 1e-6))
            roll_delta = min(progress * cfg["load_shift_roll_delta"], roll_delta)
        elif phase_id == PhaseId.PRE_LIFTOFF:
            elapsed = t - phase_start_time
            time_progress = min(1.0, elapsed / max(cfg["pre_liftoff_time"], 1e-6))
            swing_force_error = max(0.0, load_shift_metrics.swing_force - cfg["pre_liftoff_swing_force_max"])
            roll_delta += cfg["pre_liftoff_extra_roll_delta"] * min(
                1.0, swing_force_error / max(cfg["pre_liftoff_swing_force_max"], 1e-6)
            )
            roll_delta *= min(1.0, time_progress)

        apply_roll_shift_offset(targets, joint_name_to_dof_idx, support_leg, roll_delta)

    return targets, roll_delta


def plot_diagnostics(
    time_log, com_log, com_ref_log, phase_log, support_ratio_log,
    foot_force_log, foot_slip_log, cop_y_log, L_log, roll_delta_log,
    swing_force_log, candidate_foot_links, link_to_foot_name,
):
    """Multi-panel diagnostic plot."""
    time_arr = np.array(time_log)
    com_arr = np.stack(com_log)
    com_ref_arr = np.stack(com_ref_log)
    phase_arr = np.array(phase_log)
    L_arr = np.stack(L_log)
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

    # 1. CoM tracking error
    ax = axes[0]
    for i, label in enumerate(["x", "y", "z"]):
        ax.plot(time_arr, com_arr[:, i] - com_ref_arr[:, i], label=label)
    ax.set_ylabel("CoM error [m]")
    ax.axhline(0, color="k", linewidth=0.5)
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    vlines(ax)

    # 2. Support ratio
    ax = axes[1]
    ax.plot(time_arr, support_ratio_log, label="support ratio")
    ax.axhline(0.5, color="gray", linestyle=":", alpha=0.5)
    ax.set_ylabel("Support ratio")
    ax.set_ylim(0.4, 1.05)
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    vlines(ax)

    # 3. Foot slip
    ax = axes[2]
    for link_id in candidate_foot_links:
        slip = np.array(foot_slip_log[link_id]) * 1000.0
        name = link_to_foot_name.get(link_id, f"link_{link_id}")
        ax.plot(time_arr, slip, label=name)
    ax.set_ylabel("Foot slip [mm]")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    vlines(ax)

    # 4. Roll delta
    ax = axes[3]
    ax.plot(time_arr, roll_delta_log)
    ax.set_ylabel("Roll delta [rad]")
    ax.grid(True, alpha=0.3)
    vlines(ax)

    # 5. CoP y-position
    ax = axes[4]
    ax.plot(time_arr, cop_y_log, label="CoP y")
    ax.set_ylabel("CoP y [m]")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    vlines(ax)

    # 6. Angular momentum
    ax = axes[5]
    for i, label in enumerate(["Lx", "Ly", "Lz"]):
        ax.plot(time_arr, L_arr[:, i], label=label)
    ax.set_ylabel("Angular momentum")
    ax.axhline(0, color="k", linewidth=0.5)
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    vlines(ax)

    axes[-1].set_xlabel("Time [s]")
    fig.suptitle("Diagnostics")
    plt.tight_layout()
    _save_figure(fig, "diagnostics.png")


def main() -> None:
    cfg = PHASE_CONFIG
    wbc_module.Kp_c = cfg["kp_c"]
    wbc_module.Kd_c = cfg["kd_c"]
    wbc_module.Kp_L = cfg["kp_L"]
    wbc_module.Kd_L = cfg["kd_L"]

    robot = RobotModel(g1_config)
    robot.model.opt.gravity[:] = GRAVITY
    robot.model.opt.timestep = DT_SIM
    robot.model.dof_damping[:6] = BASE_DOF_DAMPING
    robot.model.dof_damping[6:] = JOINT_DOF_DAMPING
    robot.reset_base_pose(g1_config.base_initial_pos, g1_config.base_initial_orn)

    swing_leg = g1_config.lift_leg
    support_leg = "right" if swing_leg == "left" else "left"
    preferred_support_foot_link = robot.foot_name_to_link[g1_config.support_foot_name]
    swing_foot_link = robot.foot_name_to_link[g1_config.swing_foot_name]
    candidate_foot_links = robot.foot_link_ids
    estimator = StateEstimator(robot)

    initial_dof_angles = np.zeros(len(robot.dof_joints))
    joint_name_to_dof_idx = robot.dof_joint_name_to_index.copy()
    for idx, joint_name in enumerate(robot.dof_joint_names):
        if joint_name in g1_config.standing_joint_angles:
            initial_dof_angles[idx] = g1_config.standing_joint_angles[joint_name]
    robot.reset_joint_positions(initial_dof_angles)
    measured_com_z = float(robot.compute_com_position()[2])

    tau_max_limits = robot.tau_limits.copy()
    tau_min_limits = -tau_max_limits
    swing_leg_dof_indices = [
        joint_name_to_dof_idx[name]
        for name in g1_config.leg_joint_names[swing_leg]
        if name in joint_name_to_dof_idx
    ]

    nq_base = robot.model.jnt_qposadr[1] if robot.model.njnt > 1 else robot.model.nq
    nv_base = robot.model.jnt_dofadr[1] if robot.model.njnt > 1 else robot.model.nv

    initial_foot_pos = {
        link: np.array(robot.get_contact_metrics(link)["position"], copy=True)
        for link in candidate_foot_links
    }
    nominal_c_ref = np.zeros(3)
    nominal_c_ref[2] = measured_com_z
    nominal_c_ref[:2] = np.mean(
        [initial_foot_pos[link][:2] for link in candidate_foot_links], axis=0
    )

    support_contact_locals = resolve_corner_local_positions(robot, preferred_support_foot_link)
    swing_contact_locals = resolve_corner_local_positions(robot, swing_foot_link)
    initial_support_metrics = robot.get_contact_metrics(preferred_support_foot_link)
    initial_support_contact = initial_support_metrics["position"].copy()
    initial_support_yaw = yaw_from_rotation(
        np.array(robot.data.xmat[preferred_support_foot_link]).reshape(3, 3)
    )

    # Single-support MPC and WBC
    mpc = CentroidalMPC()
    wbc_ss = WholeBodyController(robot.nv, num_contacts=4, contact_dim=3)
    x_ref = np.zeros(NX)
    x_ref[2] = measured_com_z
    u_ref = np.zeros(NU)
    u_ref[2] = -GRAVITY[2] * robot.total_mass
    mpc.set_reference(x_ref, u_ref)

    # Direct-pose for single-support reference
    direct_pose = build_direct_pose(robot.dof_joint_names, support_leg)
    support_contact_local_positions = resolve_support_contact_local_positions(
        robot, preferred_support_foot_link, initial_support_contact
    )

    mpc_force_target = u_ref.copy()
    mpc_result = None
    wbc_result_ss = None
    last_valid_support_tau = None
    ss_established = False
    ss_com_ref = None
    ss_joint_ref = None
    filtered_cop_world = np.array(initial_support_metrics["cop_position"], copy=True)
    filtered_support_pos = np.array(initial_support_metrics["position"], copy=True)

    total_steps = int(cfg["sim_duration"] / DT_SIM)

    time_log = []
    com_log = []
    com_ref_log = []
    support_ratio_log = []
    phase_log = []
    foot_force_log = {link: [] for link in candidate_foot_links}
    foot_slip_log = {link: [] for link in candidate_foot_links}
    cop_y_log = []
    L_log = []
    roll_delta_log = []
    swing_force_log = []

    phase = PhaseId.INIT_SETTLE
    phase_start_time = 0.0
    ds_gate = StabilityGate()
    ls_gate = StabilityGate()
    pl_gate = StabilityGate()
    c_ref = nominal_c_ref.copy()
    standing_com_z = measured_com_z
    phase_entry_com_xy = nominal_c_ref[:2].copy()

    print(f"\n===== Phase Experiment (duration={cfg['sim_duration']:.1f}s) =====")
    print(f"Support: {support_leg}, Swing: {swing_leg}")
    print(f"Phases: INIT_SETTLE -> DS_HOLD -> LOAD_SHIFT -> PRE_LIFTOFF -> SINGLE_SUPPORT")
    print("=" * 60)

    for step in range(total_steps):
        t = step * DT_SIM
        state = estimator.update(lock_support=(phase >= PhaseId.LOAD_SHIFT))
        c = state["c"]
        c_dot = state["c_dot"]
        q = state["q"]
        v = state["v"]
        joint_positions = q[nq_base:]
        joint_velocities = v[nv_base:]
        support_foot_link = state["support_foot_link"]
        foot_contacts = state["foot_contacts"]
        load_shift_metrics = state["load_shift_metrics"]
        elapsed = t - phase_start_time

        # --- Phase transitions ---
        next_phase = None

        if phase == PhaseId.INIT_SETTLE:
            if elapsed >= cfg["init_settle_time"]:
                next_phase = PhaseId.DOUBLE_SUPPORT_HOLD

        elif phase == PhaseId.DOUBLE_SUPPORT_HOLD:
            double_support_forces = [
                get_contact_entry(foot_contacts, link)["normal_force"]
                if get_contact_entry(foot_contacts, link) is not None else 0.0
                for link in candidate_foot_links
            ]
            max_slip = max(
                np.linalg.norm(
                    get_contact_entry(foot_contacts, link)["position"][:2]
                    - initial_foot_pos[link][:2]
                )
                for link in candidate_foot_links
            )
            com_vel_ok = float(np.linalg.norm(c_dot)) <= cfg["ds_max_com_vel"]
            L = state["L"]
            momentum_ok = float(np.linalg.norm(L)) <= cfg["ds_max_L_norm"]
            total_force = sum(double_support_forces)
            balanced = False
            if total_force > 1e-6:
                balanced = (min(double_support_forces) / total_force) >= cfg["ds_force_ratio_min"]
            corners_xy: list[np.ndarray] = []
            for link in candidate_foot_links:
                corners_xy.extend(compute_foot_corners_world_xy(robot, link))
            com_inside = False
            if corners_xy:
                xs = [float(p[0]) for p in corners_xy]
                ys = [float(p[1]) for p in corners_xy]
                com_inside = (
                    float(c[0]) >= min(xs) + cfg["ds_com_margin"]
                    and float(c[0]) <= max(xs) - cfg["ds_com_margin"]
                    and float(c[1]) >= min(ys) + cfg["ds_com_margin"]
                    and float(c[1]) <= max(ys) - cfg["ds_com_margin"]
                )
            all_forces_ok = all(f >= cfg["ds_min_force"] for f in double_support_forces)
            stable = all_forces_ok and max_slip <= SLIP_THRESH and com_vel_ok and momentum_ok and balanced and com_inside
            if ds_gate.check(stable, t, cfg["ds_ready_time"]):
                next_phase = PhaseId.LOAD_SHIFT

        elif phase == PhaseId.LOAD_SHIFT:
            checks = {
                "time": elapsed >= cfg["load_shift_time"],
                "support_force": load_shift_metrics.support_force >= cfg["ds_min_force"],
                "support_ratio": load_shift_metrics.support_ratio >= cfg["load_shift_ratio_min"],
                "com_speed": load_shift_metrics.com_speed <= cfg["com_vel_ready_thresh"],
            }
            if ls_gate.check(all(checks.values()), t, 0.10):
                next_phase = PhaseId.PRE_LIFTOFF

        elif phase == PhaseId.PRE_LIFTOFF:
            elapsed_ok = elapsed >= cfg["pre_liftoff_time"]
            ratio_ok = load_shift_metrics.support_ratio >= cfg["pre_liftoff_ratio_min"]
            speed_ok = load_shift_metrics.com_speed <= cfg["com_vel_ready_thresh"]
            force_ok = load_shift_metrics.swing_force < cfg["pre_liftoff_swing_force_max"]
            # Also allow transition after extended time if ratio is decent
            time_fallback = elapsed >= cfg["pre_liftoff_time"] * 2 and load_shift_metrics.support_ratio >= 0.55
            checks = elapsed_ok and (ratio_ok or time_fallback) and speed_ok and force_ok
            if pl_gate.check(checks, t, 0.10):
                next_phase = PhaseId.SINGLE_SUPPORT

        if next_phase is not None:
            phase = next_phase
            phase_start_time = t
            phase_entry_com_xy = c[:2].copy()
            print(f"[PHASE] t={t:.3f}s -> {PhaseId(phase).name}")
            if phase == PhaseId.DOUBLE_SUPPORT_HOLD:
                standing_com_z = float(c[2])
            elif phase == PhaseId.LOAD_SHIFT:
                standing_com_z = float(c[2])
            elif phase == PhaseId.SINGLE_SUPPORT:
                standing_com_z = float(c[2])

        # --- Compute c_ref (adaptive: smoothstep from phase-entry CoM toward target) ---
        c_dot_ref = np.zeros(3)
        c_ddot_ref = np.zeros(3)
        support_xy = initial_foot_pos[preferred_support_foot_link][:2]
        swing_xy = initial_foot_pos[swing_foot_link][:2]
        stance_midpoint = 0.5 * (support_xy + swing_xy)
        support_direction = support_xy - stance_midpoint

        if phase <= PhaseId.DOUBLE_SUPPORT_HOLD:
            c_ref = nominal_c_ref.copy()
        elif phase == PhaseId.LOAD_SHIFT:
            progress = smoothstep(elapsed / max(cfg["load_shift_time"], 1e-6))
            target_xy = stance_midpoint + cfg["load_shift_target_ratio"] * support_direction
            c_ref = nominal_c_ref.copy()
            c_ref[:2] = (1.0 - progress) * phase_entry_com_xy + progress * target_xy
        elif phase == PhaseId.PRE_LIFTOFF:
            progress = smoothstep(elapsed / max(cfg["pre_liftoff_time"], 1e-6))
            target_xy = stance_midpoint + cfg["single_support_support_ratio"] * support_direction
            c_ref = nominal_c_ref.copy()
            c_ref[:2] = (1.0 - progress) * phase_entry_com_xy + progress * target_xy
        else:
            # SINGLE_SUPPORT: hold CoM at phase-entry position (adaptive)
            c_ref = nominal_c_ref.copy()
            c_ref[:2] = phase_entry_com_xy

        if standing_com_z is not None:
            c_ref[2] = standing_com_z

        # --- Compute safe_tau (PD posture + C_bias) ---
        C_safe = robot.compute_coriolis_gravity(q, v)
        safe_targets, roll_delta = build_safe_targets(
            initial_dof_angles, joint_name_to_dof_idx, phase, phase_start_time, t,
            support_leg, load_shift_metrics,
        )
        safe_tau = C_safe[6:] + compute_pd_torque(
            safe_targets, joint_positions, joint_velocities,
            cfg["posture_kp"], cfg["posture_kd"], tau_max_limits,
        )
        if phase >= PhaseId.PRE_LIFTOFF and swing_leg_dof_indices:
            swing_kp = cfg["lift_leg_kp"] * 0.8
            swing_kd = cfg["lift_leg_kd"] * 0.9
            swing_idx = np.array(swing_leg_dof_indices, dtype=int)
            swing_targets = initial_dof_angles[swing_idx].copy()
            hip_name = f"{swing_leg}_hip_pitch_joint"
            knee_name = f"{swing_leg}_knee_joint"
            ankle_pitch_name = f"{swing_leg}_ankle_pitch_joint"

            if phase == PhaseId.PRE_LIFTOFF:
                progress = min(1.0, elapsed / max(cfg["pre_liftoff_time"], 1e-6))
                if hip_name in joint_name_to_dof_idx:
                    swing_targets[joint_name_to_dof_idx[hip_name] - nv_base] += 0.15 * progress
                if knee_name in joint_name_to_dof_idx:
                    swing_targets[joint_name_to_dof_idx[knee_name] - nv_base] += -0.15 * progress
            elif phase == PhaseId.SINGLE_SUPPORT:
                # Blend from pre-liftoff pose toward the direct single-support pose
                swing_ramp = min(1.0, (elapsed - 0.0) / max(cfg.get("single_support_swing_ramp_time", 1.5), 1e-6))
                swing_ramp = smoothstep(swing_ramp)
                max_swing_progress = cfg.get("single_support_swing_progress_max", 0.45)
                swing_progress = max_swing_progress * swing_ramp

                if ss_joint_ref is not None:
                    # Blend support-side joints toward the direct pose
                    support_blend = min(1.0, elapsed / max(cfg.get("single_support_pose_blend_time", 0.6), 1e-6))
                    support_blend = smoothstep(support_blend)
                    for name in joint_name_to_dof_idx:
                        idx = joint_name_to_dof_idx[name] - nv_base
                        if name in [f"{support_leg}_hip_pitch_joint", f"{support_leg}_knee_joint",
                                     f"{support_leg}_ankle_pitch_joint", f"{support_leg}_hip_roll_joint",
                                     f"{support_leg}_ankle_roll_joint"]:
                            swing_targets_local_idx = None
                            pass  # handled below via direct_pose blend on safe_targets

                # Swing leg: knee bend + hip flexion + ankle dorsiflexion
                if hip_name in joint_name_to_dof_idx:
                    swing_targets[joint_name_to_dof_idx[hip_name] - nv_base] = (
                        (1.0 - swing_progress) * initial_dof_angles[joint_name_to_dof_idx[hip_name]]
                        + swing_progress * cfg.get("swing_hip_pitch_target", 0.0)
                    )
                if knee_name in joint_name_to_dof_idx:
                    swing_targets[joint_name_to_dof_idx[knee_name] - nv_base] = (
                        (1.0 - swing_progress) * initial_dof_angles[joint_name_to_dof_idx[knee_name]]
                        + swing_progress * cfg.get("swing_knee_target", 0.80)
                    )
                if ankle_pitch_name in joint_name_to_dof_idx:
                    swing_targets[joint_name_to_dof_idx[ankle_pitch_name] - nv_base] = (
                        (1.0 - swing_progress) * initial_dof_angles[joint_name_to_dof_idx[ankle_pitch_name]]
                        + swing_progress * cfg.get("swing_ankle_pitch_target", -0.35)
                    )

            safe_tau[swing_idx] = (
                C_safe[6:][swing_idx]
                + compute_pd_torque(
                    swing_targets, joint_positions[swing_idx], joint_velocities[swing_idx],
                    swing_kp, swing_kd, tau_max_limits[swing_idx],
                )
            )
        safe_tau = np.clip(safe_tau, tau_min_limits, tau_max_limits)

        # --- Compute control torque based on phase ---
        applied_tau = safe_tau.copy()

        if step % 200 == 0 and phase == PhaseId.PRE_LIFTOFF:
            elapsed_ok = elapsed >= cfg["pre_liftoff_time"]
            ratio_ok = load_shift_metrics.support_ratio >= cfg["pre_liftoff_ratio_min"]
            speed_ok = load_shift_metrics.com_speed <= cfg["com_vel_ready_thresh"]
            force_ok = load_shift_metrics.swing_force < cfg["pre_liftoff_swing_force_max"]
            time_fallback = elapsed >= cfg["pre_liftoff_time"] * 2 and load_shift_metrics.support_ratio >= 0.55
            print(f"  [PL-GATE] elapsed_ok={elapsed_ok} ratio={ratio_ok}({load_shift_metrics.support_ratio:.3f}) speed={speed_ok}({load_shift_metrics.com_speed:.3f}) force={force_ok}({load_shift_metrics.swing_force:.1f}) fb={time_fallback}")

        if phase in (PhaseId.DOUBLE_SUPPORT_HOLD, PhaseId.LOAD_SHIFT, PhaseId.PRE_LIFTOFF):
            # DS WBC with corner-patch contacts per foot
            # Progressive: drop swing foot from WBC if its normal force is too low
            swing_force_thresh = 80.0
            include_swing = swing_force >= swing_force_thresh
            active_locals = support_contact_locals + (swing_contact_locals if include_swing else [])
            active_links = [preferred_support_foot_link] * len(support_contact_locals) + ([swing_foot_link] * len(swing_contact_locals) if include_swing else [])
            n_contacts = len(active_locals)
            wbc_ds = WholeBodyController(robot.nv, num_contacts=n_contacts, contact_dim=3)

            M = robot.compute_mass_matrix(q)
            C = robot.compute_coriolis_gravity(q, v)

            all_locals = active_locals
            all_links = active_links

            J_blocks = []
            for local_pos, link in zip(all_locals, all_links):
                J = robot.get_foot_jacobian(link, q, local_position=local_pos)
                J_blocks.append(J[:3])
            J_c = np.vstack(J_blocks)
            Jc_dot = np.zeros_like(J_c)
            J_com = robot.get_com_jacobian(q)
            J_L = robot.get_angular_momentum_jacobian(q)

            c_ddot_des = wbc_ds.compute_desired_acceleration(c_ref, c, c_dot_ref, c_dot, c_ddot_ref)
            L_ref = np.zeros(3)
            L_dot_ref = np.zeros(3)
            L = state["L"]
            L_dot_des = wbc_ds.compute_desired_momentum_rate(L_ref, L, L_dot_ref, np.zeros(3))

            support_force = 0.0
            swing_force = 0.0
            for fc in foot_contacts:
                if fc["link"] == preferred_support_foot_link:
                    support_force = fc["normal_force"]
                elif fc["link"] == swing_foot_link:
                    swing_force = fc["normal_force"]
            total_force = support_force + swing_force
            if total_force < 10.0:
                total_force = -GRAVITY[2] * robot.total_mass
                support_force = total_force * 0.5
                swing_force = total_force * 0.5

            if phase == PhaseId.DOUBLE_SUPPORT_HOLD:
                target_ratio = 0.5
            elif phase == PhaseId.LOAD_SHIFT:
                progress = smoothstep(elapsed / max(cfg["load_shift_time"], 1e-6))
                target_ratio = 0.5 + progress * (cfg["load_shift_target_ratio"] - 0.5)
            else:
                progress = smoothstep(elapsed / max(cfg["pre_liftoff_time"], 1e-6))
                target_ratio = cfg["load_shift_target_ratio"] + progress * (cfg["single_support_support_ratio"] - cfg["load_shift_target_ratio"])

            f_ref = np.zeros(3 * n_contacts)
            total_fz = max(-GRAVITY[2] * robot.total_mass, 1e-6)
            n_support = len(support_contact_locals)
            n_swing_active = len(swing_contact_locals) if include_swing else 0
            for i in range(n_support):
                f_ref[3 * i + 2] = target_ratio * total_fz / n_support
            for i in range(n_swing_active):
                f_ref[3 * (n_support + i) + 2] = (1.0 - target_ratio) * total_fz / n_swing_active

            wbc_result = wbc_ds.solve(
                M, C, J_c, Jc_dot,
                J_com, J_L,
                c_ddot_des, L_dot_des,
                f_ref, v,
                tau_min_limits, tau_max_limits,
            )
            if wbc_result is not None:
                blend = cfg["ds_posture_blend"]
                applied_tau = (1.0 - blend) * wbc_result["tau"] + blend * safe_tau
            else:
                applied_tau = safe_tau.copy()

        elif phase == PhaseId.SINGLE_SUPPORT:
            ss_elapsed = t - phase_start_time
            # Update support foot tracking
            support_contact = get_contact_entry(foot_contacts, preferred_support_foot_link)
            measured_support_point = (
                support_contact["position"].copy()
                if support_contact is not None
                else robot.get_link_com_position(preferred_support_foot_link)
            )
            alpha = 0.1
            filtered_support_pos = (1.0 - alpha) * filtered_support_pos + alpha * measured_support_point
            p_foot = filtered_support_pos.copy()

            if not ss_established and ss_elapsed > cfg["single_support_entry_time"]:
                support_metrics_now = robot.get_contact_metrics(preferred_support_foot_link)
                if support_metrics_now["normal_force"] >= cfg["single_support_hold_force"]:
                    ss_established = True
                    ss_com_ref = c.copy()
                    ss_com_ref[2] = standing_com_z if standing_com_z is not None else c_ref[2]
                    ss_joint_ref = joint_positions.copy()
                    filtered_cop_world = np.array(support_metrics_now["cop_position"], copy=True)

            # Set up reference
            c_ref_ss = c_ref.copy()
            if ss_com_ref is not None:
                c_ref_ss[:2] = ss_com_ref[:2]

            # MPC (at reduced frequency)
            mpc_period = max(1, round(1.0 / DT_SIM / 20))
            if step % mpc_period == 0 and ss_established:
                x_ref_ss = np.zeros(NX)
                x_ref_ss[:3] = c_ref_ss
                x_ref_ss[3:6] = c_dot_ref
                x_ref_ss[6:9] = np.zeros(3)
                mpc.set_reference(x_ref_ss, u_ref)
                from phase_core import compute_centroidal_dynamics
                A_d, B_d, d_d = compute_centroidal_dynamics(robot.total_mass, c, p_foot, DT_SIM)
                mpc.set_dynamics(A_d, B_d, d_d)
                x0 = np.zeros(NX)
                x0[:3] = c
                x0[3:6] = c_dot
                x0[6:9] = state["L"]
                mpc_result = mpc.solve(x0)
                if mpc_result is not None:
                    mpc_force_target = mpc_result["u0"]

            # Single-support WBC
            if ss_established:
                M = robot.compute_mass_matrix(q)
                C = robot.compute_coriolis_gravity(q, v)

                J_blocks_ss = []
                for local_pos in support_contact_local_positions:
                    J = robot.get_foot_jacobian(preferred_support_foot_link, q, local_position=local_pos)
                    J_blocks_ss.append(J[:3])
                J_c_ss = np.vstack(J_blocks_ss)
                Jc_dot_ss = np.zeros_like(J_c_ss)
                J_com = robot.get_com_jacobian(q)
                J_L = robot.get_angular_momentum_jacobian(q)

                c_ddot_des = wbc_ss.compute_desired_acceleration(c_ref_ss, c, c_dot_ref, c_dot, c_ddot_ref)
                L_ref = np.zeros(3)
                L_dot_ref = np.zeros(3)
                L_dot_des = wbc_ss.compute_desired_momentum_rate(L_ref, state["L"], L_dot_ref, np.zeros(3))

                total_fz = max(-GRAVITY[2] * robot.total_mass, 1e-6)
                f_ref_ss = compute_corner_patch_force_reference(
                    support_contact_local_positions,
                    np.array(robot.data.xpos[preferred_support_foot_link], copy=True),
                    np.array(robot.data.xmat[preferred_support_foot_link]).reshape(3, 3),
                    filtered_cop_world,
                    total_fz,
                )

                wbc_result_ss = wbc_ss.solve(
                    M, C, J_c_ss, Jc_dot_ss,
                    J_com, J_L,
                    c_ddot_des, L_dot_des,
                    f_ref_ss, v,
                    tau_min_limits, tau_max_limits,
                )

            # Blend WBC with safe_tau
            support_mask = np.ones(robot.num_joints, dtype=bool)
            if swing_leg_dof_indices:
                support_mask[np.array(swing_leg_dof_indices, dtype=int)] = False

            if wbc_result_ss is not None:
                tau_wbc = np.clip(wbc_result_ss["tau"], tau_min_limits, tau_max_limits)
                last_valid_support_tau = tau_wbc[support_mask].copy()
            elif last_valid_support_tau is None:
                last_valid_support_tau = safe_tau[support_mask].copy()

            support_force_val = support_contact["normal_force"] if support_contact is not None else 0.0
            transition_alpha = min(1.0, ss_elapsed / max(cfg["transition_blend_time"], cfg["single_support_entry_time"], 1e-6))
            support_alpha = min(1.0, support_force_val / max(cfg["single_support_hold_force"], 1e-6))
            effective_alpha = min(cfg["single_support_max_tau_blend"], transition_alpha * support_alpha)

            tau_cmd = safe_tau.copy()
            if last_valid_support_tau is not None:
                tau_cmd[support_mask] = (1.0 - effective_alpha) * safe_tau[support_mask] + effective_alpha * last_valid_support_tau

            applied_tau = np.clip(tau_cmd, tau_min_limits, tau_max_limits)

        robot.set_joint_torques(applied_tau)
        robot.step()

        # --- Logging ---
        time_log.append(t)
        com_log.append(c.copy())
        com_ref_log.append(c_ref.copy())
        phase_log.append(int(phase))
        support_ratio_log.append(load_shift_metrics.support_ratio)
        for fc in foot_contacts:
            link = fc["link"]
            foot_force_log[link].append(fc["normal_force"])

        link_force = {fc["link"]: fc["normal_force"] for fc in foot_contacts}
        for link in candidate_foot_links:
            if link_force.get(link, 0.0) > 10.0:
                current_pos = robot.get_contact_metrics(link)["position"]
                foot_slip_log[link].append(
                    float(np.linalg.norm(current_pos[:2] - initial_foot_pos[link][:2]))
                )
            else:
                foot_slip_log[link].append(np.nan)

        support_cop = robot.get_contact_metrics(preferred_support_foot_link)
        cop_y_log.append(float(support_cop["cop_position"][1]))

        L_log.append(state["L"].copy())
        roll_delta_log.append(roll_delta)
        swing_force_log.append(load_shift_metrics.swing_force)

        # --- Periodic diagnostics ---
        if step % 500 == 0:
            phase_name = PhaseId(phase).name
            sf = load_shift_metrics.support_force
            wf = load_shift_metrics.swing_force
            sr = load_shift_metrics.support_ratio
            cs = load_shift_metrics.com_shift_ratio
            print(f"[t={t:.2f}s] {phase_name:20s} sf={sf:.0f}N wf={wf:.0f}N ratio={sr:.3f} com_shift={cs:.3f} c_ref_y={c_ref[1]:.4f} c_y={c[1]:.4f}")

    # --- Results ---
    print(f"\n{'='*60}")
    print("===== RESULTS =====")
    rmse = compute_rmse(com_log, com_ref_log)
    print(f"CoM RMSE: {rmse:.4f} m (target < {RMSE_THRESH} m)")
    max_slip = 0.0
    for link in candidate_foot_links:
        positions = np.array([fp for fp in foot_force_log[link]])
        if len(positions) > 0:
            pass
    max_slip = 0.0
    final_foot_pos = {
        link: robot.get_contact_metrics(link)["position"].copy()
        for link in candidate_foot_links
    }
    for link in candidate_foot_links:
        slip = np.linalg.norm(final_foot_pos[link][:2] - initial_foot_pos[link][:2])
        max_slip = max(max_slip, slip)
    print(f"Max foot slip: {max_slip*1000:.2f} mm (target < {SLIP_THRESH*1000:.1f} mm)")
    for link in candidate_foot_links:
        forces = np.array(foot_force_log[link])
        avg_force = np.mean(forces) if len(forces) > 0 else 0.0
        name = support_name(robot.link_to_foot_name, link)
        print(f"  {name} avg force: {avg_force:.1f} N")

    max_phase = max(phase_log)
    print(f"\nHighest phase reached: {PhaseId(max_phase).name}")
    if max_phase >= PhaseId.LOAD_SHIFT:
        ls_start = None
        ls_ratios = []
        for i, p in enumerate(phase_log):
            if p == PhaseId.LOAD_SHIFT and ls_start is None:
                ls_start = time_log[i]
            if p >= PhaseId.LOAD_SHIFT and ls_start is not None:
                ls_ratios.append(support_ratio_log[i])
        if ls_ratios:
            print(f"  LOAD_SHIFT max ratio: {max(ls_ratios):.3f}, mean: {np.mean(ls_ratios):.3f}")
    if max_phase >= PhaseId.PRE_LIFTOFF:
        pl_start = None
        pl_ratios = []
        for i, p in enumerate(phase_log):
            if p == PhaseId.PRE_LIFTOFF and pl_start is None:
                pl_start = time_log[i]
            if p >= PhaseId.PRE_LIFTOFF and pl_start is not None:
                pl_ratios.append(support_ratio_log[i])
        if pl_ratios:
            print(f"  PRE_LIFTOFF max ratio: {max(pl_ratios):.3f}, mean: {np.mean(pl_ratios):.3f}")

    plot_com_tracking(time_log, com_log, com_ref_log)

    plot_diagnostics(
        time_log, com_log, com_ref_log, phase_log,
        support_ratio_log, foot_force_log, foot_slip_log,
        cop_y_log, L_log, roll_delta_log, swing_force_log,
        candidate_foot_links, robot.link_to_foot_name,
    )


if __name__ == "__main__":
    main()