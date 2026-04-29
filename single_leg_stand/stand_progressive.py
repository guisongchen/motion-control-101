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
    MIN_SUPPORT_FORCE,
    RMSE_THRESH,
    SLIP_THRESH,
    NX,
    NU,
    WBC_FREQ,
)
from robot_model import RobotModel
from robots.unitree_g1 import g1_config
from state_estimator import StateEstimator
from phase_core import (
    StabilityGate,
    get_contacts,
    is_com_inside_support_polygon,
    support_name,
)
from mpc import CentroidalMPC
from wbc import WholeBodyController
from direct_single_support import (
    resolve_support_contact_local_positions,
)
from direct_single_support.primitives import (
    compute_corner_patch_force_reference,
)
from utils import compute_rmse, compute_pd_torque, plot_diagnostics

from enum import IntEnum


class SimLog:
    def __init__(self, foot_links):
        self.foot_links: list[int] = foot_links
        self.t: list[float] = []
        self.com: list[np.ndarray] = []
        self.com_ref: list[np.ndarray] = []
        self.support_ratio: list[float] = []
        self.phase: list[int] = []
        self.foot_force: dict[int, list[float]] = {link: [] for link in foot_links}
        self.foot_slip: dict[int, list[float]] = {link: [] for link in foot_links}
        self.cop_y: list[float] = []
        self.L: list[np.ndarray] = []
        self.roll_delta: list[float] = []
        self.swing_force: list[float] = []
        self.support_force: list[float] = []


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
    "load_shift_ratio_min": 0.53,
    "load_shift_com_ratio": 0.35,
    "load_shift_roll_delta": 0.10,
    "load_shift_swing_force_min": 25.0,
    "com_vel_ready_thresh": 0.35,
    "pre_liftoff_time": 1.00,
    "pre_liftoff_ratio_min": 0.55,
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
    "ds_shift_posture_blend": 0.40,
    "adaptive_ratio_span": 0.10,
    "ds_force_ratio_cap": 0.85,
    "ds_force_target_margin": 0.05,
    "ds_force_desired_factor": 0.0,
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


def compute_swing_progress(elapsed, phase, cfg):
    if phase != PhaseId.SINGLE_SUPPORT:
        return 0.0
    swing_ramp = min(1.0, elapsed / max(cfg.get("single_support_swing_ramp_time", 1.5), 1e-6))
    swing_ramp = smoothstep(swing_ramp)
    max_progress = cfg.get("single_support_swing_progress_max", 0.45)
    return max_progress * swing_ramp


def compute_safe_tau(
    C_safe, initial_dof_angles, joint_positions, joint_velocities,
    joint_name_to_dof_idx, nv_base,
    tau_min_limits, tau_max_limits,
    phase, elapsed, support_leg, swing_leg, load_shift_metrics,
    swing_leg_dof_indices, roll_delta, cfg,
) -> np.ndarray:
    safe_targets = initial_dof_angles.copy()
    if roll_delta != 0.0:
        apply_roll_shift_offset(safe_targets, joint_name_to_dof_idx, support_leg, roll_delta)
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
            swing_progress = compute_swing_progress(elapsed, phase, cfg)
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

    return np.clip(safe_tau, tau_min_limits, tau_max_limits)


def compute_roll_delta(
    phase_id, phase_start_time, t, support_leg, load_shift_metrics,
) -> float:
    cfg = PHASE_CONFIG
    if phase_id < PhaseId.LOAD_SHIFT:
        return 0.0

    elapsed = t - phase_start_time
    desired = desired_support_ratio(phase_id, elapsed, cfg)
    ratio_error = max(0.0, desired - load_shift_metrics.support_ratio)
    com_error = max(0.0, cfg["load_shift_com_ratio"] - load_shift_metrics.com_shift_ratio)
    roll_feedback = cfg["load_shift_roll_delta"] * min(
        1.0,
        8.0 * (0.7 * ratio_error / max(desired, 1e-6)
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

    return roll_delta


def desired_support_ratio(phase, elapsed, cfg):
    if phase <= PhaseId.DOUBLE_SUPPORT_HOLD:
        return 0.5
    if phase == PhaseId.LOAD_SHIFT:
        progress = np.clip(elapsed / max(cfg["load_shift_time"], 1e-6), 0.0, 1.0)
        return 0.5 + progress * cfg.get("adaptive_ratio_span", 0.45)
    if phase == PhaseId.PRE_LIFTOFF:
        progress = np.clip(elapsed / max(cfg["pre_liftoff_time"], 1e-6), 0.0, 1.0)
        start = 0.5 + cfg.get("adaptive_ratio_span", 0.45)
        return start + progress * (0.98 - start)
    return 0.98


def compute_c_ref(phase, elapsed, phase_entry_com_xy, nominal_c_ref,
                  stance_midpoint, support_direction, standing_com_z, cfg):
    c_ref = nominal_c_ref.copy()
    if phase <= PhaseId.DOUBLE_SUPPORT_HOLD:
        pass
    elif phase in (PhaseId.LOAD_SHIFT, PhaseId.PRE_LIFTOFF):
        if phase == PhaseId.LOAD_SHIFT:
            t_total = max(cfg["load_shift_time"], 1e-6)
        else:
            t_total = max(cfg["pre_liftoff_time"], 1e-6)
        progress = smoothstep(elapsed / t_total)
        desired = desired_support_ratio(phase, elapsed, cfg)
        target_shift_ratio = 2.0 * max(0.0, desired - 0.5)
        target_xy = stance_midpoint + target_shift_ratio * support_direction
        c_ref[:2] = (1.0 - progress) * phase_entry_com_xy + progress * target_xy
    else:
        c_ref[:2] = phase_entry_com_xy
    if standing_com_z is not None:
        c_ref[2] = standing_com_z
    return c_ref


def print_results(log: SimLog, robot: RobotModel, initial_foot_pos: dict):
    print(f"\n{'='*60}")
    print("===== RESULTS =====")
    rmse = compute_rmse(log.com, log.com_ref)
    print(f"CoM RMSE: {rmse:.4f} m (target < {RMSE_THRESH} m)")
    max_slip = 0.0
    final_foot_pos = {
        link: robot.get_contact_metrics(link)["position"].copy()
        for link in log.foot_links
    }
    for link in log.foot_links:
        slip = np.linalg.norm(final_foot_pos[link][:2] - initial_foot_pos[link][:2])
        max_slip = max(max_slip, slip)
    print(f"Max foot slip: {max_slip*1000:.2f} mm (target < {SLIP_THRESH*1000:.1f} mm)")
    for link in log.foot_links:
        forces = np.array(log.foot_force[link])
        avg_force = np.mean(forces) if len(forces) > 0 else 0.0
        name = support_name(robot.link_to_foot_name, link)
        print(f"  {name} avg force: {avg_force:.1f} N")

    max_phase = max(log.phase)
    print(f"\nHighest phase reached: {PhaseId(max_phase).name}")
    if max_phase >= PhaseId.LOAD_SHIFT:
        ls_ratios = []
        ls_start = None
        for i, p in enumerate(log.phase):
            if p == PhaseId.LOAD_SHIFT and ls_start is None:
                ls_start = log.t[i]
            if p >= PhaseId.LOAD_SHIFT and ls_start is not None:
                ls_ratios.append(log.support_ratio[i])
        if ls_ratios:
            print(f"  LOAD_SHIFT max ratio: {max(ls_ratios):.3f}, mean: {np.mean(ls_ratios):.3f}")
    if max_phase >= PhaseId.PRE_LIFTOFF:
        pl_ratios = []
        pl_start = None
        for i, p in enumerate(log.phase):
            if p == PhaseId.PRE_LIFTOFF and pl_start is None:
                pl_start = log.t[i]
            if p >= PhaseId.PRE_LIFTOFF and pl_start is not None:
                pl_ratios.append(log.support_ratio[i])
        if pl_ratios:
            print(f"  PRE_LIFTOFF max ratio: {max(pl_ratios):.3f}, mean: {np.mean(pl_ratios):.3f}")


def compute_ds_applied_tau(
    step, wbc_period, last_ds_wbc_tau, foot_contacts,
    support_leg_link, swing_foot_link, robot,
    support_leg_corner_locals, swing_leg_corner_locals,
    q, v, C_safe, wbc_ds_4, wbc_ds_8,
    c_ref, c, c_dot_ref, c_dot, c_ddot_ref, L,
    phase, elapsed, cfg, com_shift_ratio,
    tau_min_limits, tau_max_limits, safe_tau,
):
    if step % wbc_period != 0 and last_ds_wbc_tau is not None:
        blend = cfg["ds_posture_blend"] if phase == PhaseId.DOUBLE_SUPPORT_HOLD else cfg.get("ds_shift_posture_blend", 0.25)
        return (1.0 - blend) * last_ds_wbc_tau + blend * safe_tau, last_ds_wbc_tau

    support_force = 0.0
    swing_force = 0.0
    for fc in foot_contacts:
        if fc["link"] == support_leg_link:
            support_force = fc["normal_force"]
        elif fc["link"] == swing_foot_link:
            swing_force = fc["normal_force"]
    total_force = support_force + swing_force
    if total_force < 10.0:
        total_force = -GRAVITY[2] * robot.total_mass
        support_force = total_force * 0.5
        swing_force = total_force * 0.5
    swing_force_thresh = 80.0
    include_swing = swing_force >= swing_force_thresh
    active_locals = support_leg_corner_locals + (swing_leg_corner_locals if include_swing else [])
    active_links = [support_leg_link] * len(support_leg_corner_locals) + ([swing_foot_link] * len(swing_leg_corner_locals) if include_swing else [])
    n_contacts = len(active_locals)
    wbc_ds = wbc_ds_4 if n_contacts <= 4 else wbc_ds_8

    M = robot.compute_mass_matrix(q)
    Ci = C_safe

    J_blocks = []
    for local_pos, link in zip(active_locals, active_links):
        J = robot.get_foot_jacobian(link, q, local_position=local_pos)
        J_blocks.append(J[:3])
    J_c = np.vstack(J_blocks)
    Jc_dot = np.zeros_like(J_c)
    J_com = robot.get_com_jacobian(q)
    J_L = robot.get_angular_momentum_jacobian(q)

    c_ddot_des = wbc_ds.compute_desired_acceleration(c_ref, c, c_dot_ref, c_dot, c_ddot_ref)
    L_dot_des = wbc_ds.compute_desired_momentum_rate(np.zeros(3), L, np.zeros(3), np.zeros(3))

    if phase == PhaseId.DOUBLE_SUPPORT_HOLD:
        target_ratio = 0.5
    else:
        com_shift_clipped = max(0.0, com_shift_ratio)
        physical_target = 0.5 + 0.5 * com_shift_clipped + cfg.get("ds_force_target_margin", 0.05)
        desired = desired_support_ratio(phase, elapsed, cfg)
        desired_push = 0.5 + cfg.get("ds_force_desired_factor", 0.3) * (desired - 0.5)
        target_ratio = max(physical_target, desired_push)
        target_ratio = min(target_ratio, cfg.get("ds_force_ratio_cap", 0.95))

    f_ref = np.zeros(3 * n_contacts)
    total_fz = max(-GRAVITY[2] * robot.total_mass, 1e-6)
    n_support = len(support_leg_corner_locals)
    n_swing_active = len(swing_leg_corner_locals) if include_swing else 0
    for i in range(n_support):
        f_ref[3 * i + 2] = target_ratio * total_fz / n_support
    for i in range(n_swing_active):
        f_ref[3 * (n_support + i) + 2] = (1.0 - target_ratio) * total_fz / n_swing_active

    wbc_result = wbc_ds.solve(
        M, Ci, J_c, Jc_dot,
        J_com, J_L,
        c_ddot_des, L_dot_des,
        f_ref, v,
        tau_min_limits, tau_max_limits,
    )
    if wbc_result is not None:
        last_ds_wbc_tau = wbc_result["tau"]

    blend = cfg["ds_posture_blend"] if phase == PhaseId.DOUBLE_SUPPORT_HOLD else cfg.get("ds_shift_posture_blend", 0.25)
    return (1.0 - blend) * last_ds_wbc_tau + blend * safe_tau, last_ds_wbc_tau


class SingleSupportController:
    def __init__(self, robot, mpc, wbc_ss, support_contact_local_positions,
                 init_cop, init_support_pos, u_ref):
        self.ss_established = False
        self.ss_com_ref = None
        self.filtered_cop_world = init_cop.copy()
        self.filtered_support_pos = init_support_pos.copy()
        self.mpc_force_target = u_ref.copy()
        self.u_ref = u_ref.copy()
        self.last_ss_wbc_tau = None
        self.last_valid_support_tau = None
        self.robot = robot
        self.mpc = mpc
        self.wbc_ss = wbc_ss
        self.support_contact_local_positions = support_contact_local_positions

    def compute(
        self, step, q, v, c, c_dot, L, joint_positions,
        c_ref, c_dot_ref, c_ddot_ref, t, phase_start_time, standing_com_z,
        support_leg_link, foot_contacts, safe_tau, tau_min, tau_max,
        swing_leg_dof_indices, cfg,
    ):
        ss_elapsed = t - phase_start_time
        contacts = get_contacts(foot_contacts)
        support_contact = contacts.get(support_leg_link)
        measured_support_point = (
            support_contact["position"].copy()
            if support_contact is not None
            else self.robot.get_link_com_position(support_leg_link)
        )
        alpha = 0.1
        self.filtered_support_pos = (1.0 - alpha) * self.filtered_support_pos + alpha * measured_support_point
        p_foot = self.filtered_support_pos.copy()

        if not self.ss_established and ss_elapsed > cfg["single_support_entry_time"]:
            support_metrics_now = self.robot.get_contact_metrics(support_leg_link)
            if support_metrics_now["normal_force"] >= cfg["single_support_hold_force"]:
                self.ss_established = True
                self.ss_com_ref = c_ref.copy()
                self.ss_com_ref[2] = standing_com_z if standing_com_z is not None else c_ref[2]
                self.filtered_cop_world = np.array(support_metrics_now["cop_position"], copy=True)

        c_ref_ss = c_ref.copy()
        if self.ss_com_ref is not None:
            c_ref_ss[:2] = self.ss_com_ref[:2]

        mpc_period = max(1, round(1.0 / DT_SIM / 20))
        if step % mpc_period == 0 and self.ss_established:
            x_ref_ss = np.zeros(NX)
            x_ref_ss[:3] = c_ref_ss
            x_ref_ss[3:6] = c_dot_ref
            x_ref_ss[6:9] = np.zeros(3)
            self.mpc.set_reference(x_ref_ss, self.u_ref)
            from phase_core import compute_centroidal_dynamics
            A_d, B_d, d_d = compute_centroidal_dynamics(self.robot.total_mass, c, p_foot, DT_SIM)
            self.mpc.set_dynamics(A_d, B_d, d_d)
            x0 = np.zeros(NX)
            x0[:3] = c
            x0[3:6] = c_dot
            x0[6:9] = L
            mpc_result = self.mpc.solve(x0)
            if mpc_result is not None:
                self.mpc_force_target = mpc_result["u0"]

        wbc_period = max(1, round(1.0 / (WBC_FREQ * DT_SIM)))
        if self.ss_established:
            if step % wbc_period == 0 or self.last_ss_wbc_tau is None:
                M = self.robot.compute_mass_matrix(q)
                Ci = self.robot.compute_coriolis_gravity(q, v)

                J_blocks_ss = []
                for local_pos in self.support_contact_local_positions:
                    J = self.robot.get_foot_jacobian(support_leg_link, q, local_position=local_pos)
                    J_blocks_ss.append(J[:3])
                J_c_ss = np.vstack(J_blocks_ss)
                Jc_dot_ss = np.zeros_like(J_c_ss)
                J_com = self.robot.get_com_jacobian(q)
                J_L = self.robot.get_angular_momentum_jacobian(q)

                c_ddot_des = self.wbc_ss.compute_desired_acceleration(c_ref_ss, c, c_dot_ref, c_dot, c_ddot_ref)
                L_dot_des = self.wbc_ss.compute_desired_momentum_rate(np.zeros(3), L, np.zeros(3), np.zeros(3))

                total_fz_ss = max(-GRAVITY[2] * self.robot.total_mass, 1e-6)
                f_ref_ss = compute_corner_patch_force_reference(
                    self.support_contact_local_positions,
                    np.array(self.robot.data.xpos[support_leg_link], copy=True),
                    np.array(self.robot.data.xmat[support_leg_link]).reshape(3, 3),
                    self.filtered_cop_world,
                    total_fz_ss,
                )

                wbc_result_ss = self.wbc_ss.solve(
                    M, Ci, J_c_ss, Jc_dot_ss,
                    J_com, J_L,
                    c_ddot_des, L_dot_des,
                    f_ref_ss, v,
                    tau_min, tau_max,
                )
                if wbc_result_ss is not None:
                    self.last_ss_wbc_tau = np.clip(wbc_result_ss["tau"], tau_min, tau_max)
            elif self.last_ss_wbc_tau is None:
                self.last_ss_wbc_tau = safe_tau.copy()

        support_mask = np.ones(self.robot.num_joints, dtype=bool)
        if swing_leg_dof_indices:
            support_mask[np.array(swing_leg_dof_indices, dtype=int)] = False

        if self.last_ss_wbc_tau is not None:
            self.last_valid_support_tau = self.last_ss_wbc_tau[support_mask].copy()
        elif self.last_valid_support_tau is None:
            self.last_valid_support_tau = safe_tau[support_mask].copy()

        support_force_val = support_contact["normal_force"] if support_contact is not None else 0.0
        transition_alpha = min(1.0, ss_elapsed / max(cfg["transition_blend_time"], cfg["single_support_entry_time"], 1e-6))
        support_alpha = min(1.0, support_force_val / max(cfg["single_support_hold_force"], 1e-6))
        effective_alpha = min(cfg["single_support_max_tau_blend"], transition_alpha * support_alpha)

        tau_cmd = safe_tau.copy()
        if self.last_valid_support_tau is not None:
            tau_cmd[support_mask] = (1.0 - effective_alpha) * safe_tau[support_mask] + effective_alpha * self.last_valid_support_tau

        return np.clip(tau_cmd, tau_min, tau_max)


def main() -> None:
    cfg = PHASE_CONFIG

    robot = RobotModel(g1_config)

    swing_leg = g1_config.lift_leg
    support_leg = "right" if swing_leg == "left" else "left"
    support_leg_link = robot.foot_name_to_link[g1_config.support_foot_name]
    swing_foot_link = robot.foot_name_to_link[g1_config.swing_foot_name]
    foot_links = robot.foot_link_ids
    estimator = StateEstimator(robot)

    initial_dof_angles = robot.set_standing_angles(g1_config.standing_joint_angles)
    joint_name_to_dof_idx = robot.dof_joint_name_to_index
    measured_com_z = float(robot.compute_com_position()[2])

    tau_max_limits = robot.tau_limits.copy()
    tau_min_limits = -tau_max_limits
    swing_leg_dof_indices = [
        joint_name_to_dof_idx[name]
        for name in g1_config.leg_joint_names[swing_leg]
        if name in joint_name_to_dof_idx
    ]

    # qpos starts with floating base (7: 3 pos + 4 quat), then actuated joints.
    # qvel starts with 6 base DOFs, then actuated joints.
    # These offsets let us slice q[nq_base:] and v[nv_base:] to get joint state only.
    nq_base = robot.model.jnt_qposadr[1] if robot.model.njnt > 1 else robot.model.nq
    nv_base = robot.model.jnt_dofadr[1] if robot.model.njnt > 1 else robot.model.nv

    initial_foot_pos = {
        link: np.array(robot.get_contact_metrics(link)["position"], copy=True)
        for link in foot_links
    }
    nominal_c_ref = np.zeros(3)
    nominal_c_ref[2] = measured_com_z
    nominal_c_ref[:2] = np.mean(
        [initial_foot_pos[link][:2] for link in foot_links], axis=0
    )

    support_xy = initial_foot_pos[support_leg_link][:2]
    swing_xy = initial_foot_pos[swing_foot_link][:2]
    stance_midpoint = 0.5 * (support_xy + swing_xy)
    support_direction = support_xy - stance_midpoint

    support_leg_corner_locals = resolve_corner_local_positions(robot, support_leg_link)
    swing_leg_corner_locals = resolve_corner_local_positions(robot, swing_foot_link)
    support_leg_init_metrics = robot.get_contact_metrics(support_leg_link)
    support_leg_init_contact = support_leg_init_metrics["position"].copy()

    # Single-support MPC and WBC
    mpc = CentroidalMPC()
    wbc_ss = WholeBodyController(robot.nv, num_contacts=4, contact_dim=3)
    wbc_ds_4 = WholeBodyController(robot.nv, num_contacts=4, contact_dim=3)
    wbc_ds_8 = WholeBodyController(robot.nv, num_contacts=8, contact_dim=3)
    wbc_period = max(1, round(1.0 / (WBC_FREQ * DT_SIM)))
    last_ds_wbc_tau = None
    x_ref = np.zeros(NX)
    x_ref[2] = measured_com_z
    u_ref = np.zeros(NU)
    u_ref[2] = -GRAVITY[2] * robot.total_mass
    mpc.set_reference(x_ref, u_ref)

    support_contact_local_positions = resolve_support_contact_local_positions(
        robot, support_leg_link, support_leg_init_contact
    )

    total_steps = int(cfg["sim_duration"] / DT_SIM)

    log = SimLog(foot_links)

    ss_ctrl = SingleSupportController(
        robot, mpc, wbc_ss, support_contact_local_positions,
        support_leg_init_metrics["cop_position"],
        support_leg_init_metrics["position"],
        u_ref,
    )

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
        foot_contacts = state["foot_contacts"]
        load_shift_metrics = state["load_shift_metrics"]
        elapsed = t - phase_start_time

        # --- Phase transitions ---
        next_phase = None

        if phase == PhaseId.INIT_SETTLE:
            if elapsed >= cfg["init_settle_time"]:
                next_phase = PhaseId.DOUBLE_SUPPORT_HOLD

        elif phase == PhaseId.DOUBLE_SUPPORT_HOLD:
            contacts = get_contacts(foot_contacts)
            forces = [contacts[link]["normal_force"] for link in foot_links]
            total_force = sum(forces)

            corners_xy: list[np.ndarray] = []
            for link in foot_links:
                corners_xy.extend(compute_foot_corners_world_xy(robot, link))

            checks = {
                "forces_ok": all(f >= cfg["ds_min_force"] for f in forces),
                "balanced": total_force > 1e-6 and (min(forces) / total_force) >= cfg["ds_force_ratio_min"],
                "slip_ok": max(
                    np.linalg.norm(contacts[link]["position"][:2] - initial_foot_pos[link][:2])
                    for link in foot_links
                ) <= SLIP_THRESH,
                "com_vel_ok": float(np.linalg.norm(c_dot)) <= cfg["ds_max_com_vel"],
                "momentum_ok": float(np.linalg.norm(state["L"])) <= cfg["ds_max_L_norm"],
                "com_inside": is_com_inside_support_polygon(c, corners_xy, cfg["ds_com_margin"]),
            }
            if ds_gate.check(all(checks.values()), t, cfg["ds_ready_time"]):
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
            ratio_met = load_shift_metrics.support_ratio >= cfg["pre_liftoff_ratio_min"]
            ratio_fallback = (
                elapsed >= cfg["pre_liftoff_time"] * 2
                and load_shift_metrics.support_ratio >= 0.55
            )
            checks = {
                "time_ok": elapsed >= cfg["pre_liftoff_time"],
                "ratio_ok": ratio_met or ratio_fallback,
                "speed_ok": load_shift_metrics.com_speed <= cfg["com_vel_ready_thresh"],
                "force_ok": load_shift_metrics.swing_force < cfg["pre_liftoff_swing_force_max"],
            }
            if pl_gate.check(all(checks.values()), t, 0.10):
                next_phase = PhaseId.SINGLE_SUPPORT

        if next_phase is not None:
            phase = next_phase
            phase_start_time = t
            if next_phase == PhaseId.SINGLE_SUPPORT:
                phase_entry_com_xy = stance_midpoint + 0.95 * support_direction
            else:
                phase_entry_com_xy = c[:2].copy()
            standing_com_z = float(c[2])
            print(f"[PHASE] t={t:.3f}s -> {PhaseId(phase).name}")

        # --- Compute c_ref
        c_dot_ref = np.zeros(3)
        c_ddot_ref = np.zeros(3)

        c_ref = compute_c_ref(
            phase, elapsed, phase_entry_com_xy, nominal_c_ref,
            stance_midpoint, support_direction, standing_com_z, cfg,
        )

        # --- Compute safe_tau (PD posture + C_bias) ---
        C_safe = robot.compute_coriolis_gravity(q, v)
        roll_delta = compute_roll_delta(phase, phase_start_time, t, support_leg, load_shift_metrics)
        safe_tau = compute_safe_tau(
            C_safe, initial_dof_angles, joint_positions, joint_velocities,
            joint_name_to_dof_idx, nv_base,
            tau_min_limits, tau_max_limits,
            phase, elapsed, support_leg, swing_leg, load_shift_metrics,
            swing_leg_dof_indices, roll_delta, cfg,
        )

        # --- Compute control torque based on phase ---
        applied_tau = safe_tau.copy()

        if phase in (PhaseId.DOUBLE_SUPPORT_HOLD, PhaseId.LOAD_SHIFT, PhaseId.PRE_LIFTOFF):
            applied_tau, last_ds_wbc_tau = compute_ds_applied_tau(
                step, wbc_period, last_ds_wbc_tau, foot_contacts,
                support_leg_link, swing_foot_link, robot,
                support_leg_corner_locals, swing_leg_corner_locals,
                q, v, C_safe, wbc_ds_4, wbc_ds_8,
                c_ref, c, c_dot_ref, c_dot, c_ddot_ref, state["L"],
                phase, elapsed, cfg,
                load_shift_metrics.com_shift_ratio,
                tau_min_limits, tau_max_limits, safe_tau,
            )

        elif phase == PhaseId.SINGLE_SUPPORT:
            applied_tau = ss_ctrl.compute(
                step, q, v, c, c_dot, state["L"], joint_positions,
                c_ref, c_dot_ref, c_ddot_ref, t, phase_start_time, standing_com_z,
                support_leg_link, foot_contacts, safe_tau, tau_min_limits, tau_max_limits,
                swing_leg_dof_indices, cfg,
            )

        robot.set_joint_torques(applied_tau)
        robot.step()

        # --- Logging ---
        log.t.append(t)
        log.com.append(c.copy())
        log.com_ref.append(c_ref.copy())
        log.phase.append(int(phase))
        log.support_ratio.append(load_shift_metrics.support_ratio)
        for fc in foot_contacts:
            log.foot_force[fc["link"]].append(fc["normal_force"])
        link_force = {fc["link"]: fc["normal_force"] for fc in foot_contacts}
        for link in foot_links:
            if link_force.get(link, 0.0) > 10.0:
                current_pos = robot.get_contact_metrics(link)["position"]
                log.foot_slip[link].append(
                    float(np.linalg.norm(current_pos[:2] - initial_foot_pos[link][:2]))
                )
            else:
                log.foot_slip[link].append(np.nan)
        log.cop_y.append(float(robot.get_contact_metrics(support_leg_link)["cop_position"][1]))
        log.L.append(state["L"].copy())
        log.roll_delta.append(roll_delta)
        log.swing_force.append(load_shift_metrics.swing_force)
        log.support_force.append(load_shift_metrics.support_force)

        # --- Periodic diagnostics ---
        if step % 500 == 0:
            phase_name = PhaseId(phase).name
            sf = load_shift_metrics.support_force
            wf = load_shift_metrics.swing_force
            sr = load_shift_metrics.support_ratio
            cs = load_shift_metrics.com_shift_ratio
            print(f"[t={t:.2f}s] {phase_name:20s} sf={sf:.0f}N wf={wf:.0f}N ratio={sr:.3f} com_shift={cs:.3f} c_ref_y={c_ref[1]:.4f} c_y={c[1]:.4f}")

    print_results(log, robot, initial_foot_pos)

    plot_diagnostics(
        log, robot.link_to_foot_name, support_leg_link, swing_foot_link,
        rmse_thresh=RMSE_THRESH, slip_thresh=SLIP_THRESH,
        ds_min_force=cfg["ds_min_force"],
        load_shift_roll_delta=cfg["load_shift_roll_delta"],
        ds_max_L_norm=cfg["ds_max_L_norm"],
    )


if __name__ == "__main__":
    main()