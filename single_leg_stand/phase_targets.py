"""Joint-target generation, swing progress, and safe-tau computation."""

from __future__ import annotations

import numpy as np

from config import (
    POSTURE_KP,
    POSTURE_KD,
    LIFT_LEG_KP,
    LIFT_LEG_KD,
    SINGLE_SUPPORT_SWING_RAMP_TIME,
    SINGLE_SUPPORT_SWING_PROGRESS_MAX,
    SWING_HIP_PITCH_TARGET,
    SWING_KNEE_TARGET,
    PRE_LIFTOFF_SWING_PROGRESS,
    PRE_LIFTOFF_TIME,
    PRE_LIFTOFF_EXTRA_SWING_PROGRESS,
    PRE_LIFTOFF_SWING_KP_SCALE,
    PRE_LIFTOFF_SWING_KD_SCALE,
    SINGLE_SUPPORT_POSE_BLEND_TIME,
    SINGLE_SUPPORT_SWING_HIP_PITCH_TARGET,
    SINGLE_SUPPORT_SWING_KNEE_TARGET,
    SINGLE_SUPPORT_SWING_ANKLE_PITCH_TARGET,
    LOAD_SHIFT_TIME,
    LOAD_SHIFT_SUPPORT_RATIO,
    LOAD_SHIFT_COM_RATIO,
    LOAD_SHIFT_ROLL_DELTA,
    PRE_LIFTOFF_SUPPORT_RATIO,
    PRE_LIFTOFF_COM_RATIO,
    PRE_LIFTOFF_SWING_FORCE_MAX,
    PRE_LIFTOFF_EXTRA_ROLL_DELTA,
    SINGLE_SUPPORT_SUPPORT_RATIO,
    SINGLE_SUPPORT_COM_RATIO,
    SINGLE_SUPPORT_EXTRA_ROLL_DELTA,
    SINGLE_SUPPORT_SWING_FORCE_TARGET,
    SLIP_THRESH,
    SINGLE_SUPPORT_ENTRY_TIME,
    SINGLE_SUPPORT_FORWARD_ERROR_CLIP,
    SINGLE_SUPPORT_SUPPORT_HIP_PITCH_DELTA,
)
from phase_core import ControlPhase
from phase_metrics import LoadShiftMetrics
from utils import compute_pd_torque


def offset_joint_target(
    targets: np.ndarray,
    joint_name_to_dof_idx: dict[str, int],
    joint_name: str,
    delta: float,
) -> None:
    """Add an offset to one joint target when the joint exists in the model."""
    if joint_name in joint_name_to_dof_idx:
        targets[joint_name_to_dof_idx[joint_name]] += delta


def apply_roll_shift_offset(
    targets: np.ndarray,
    joint_name_to_dof_idx: dict[str, int],
    support_leg: str,
    delta: float,
) -> None:
    """Lean the body toward the support foot while keeping both feet roughly flat."""
    support_sign = 1.0 if support_leg == "right" else -1.0
    roll_delta = support_sign * delta
    for leg in ("left", "right"):
        offset_joint_target(targets, joint_name_to_dof_idx, f"{leg}_hip_roll_joint", roll_delta)
        offset_joint_target(targets, joint_name_to_dof_idx, f"{leg}_ankle_roll_joint", -roll_delta)


def apply_pitch_balance_offset(
    targets: np.ndarray,
    joint_name_to_dof_idx: dict[str, int],
    support_leg: str,
    delta: float,
) -> None:
    """Bias the support leg pitch joints to pull the torso back over the support foot."""
    offset_joint_target(targets, joint_name_to_dof_idx, f"{support_leg}_hip_pitch_joint", delta)
    offset_joint_target(targets, joint_name_to_dof_idx, f"{support_leg}_ankle_pitch_joint", -delta)


def blend_joint_targets_toward_pose(
    targets: np.ndarray,
    joint_name_to_dof_idx: dict[str, int],
    optimized_joint_angles: np.ndarray | None,
    joint_names: list[str],
    blend: float,
) -> None:
    """Blend a selected set of joints toward the optimized single-support pose."""
    if optimized_joint_angles is None or blend <= 0.0:
        return
    for joint_name in joint_names:
        if joint_name not in joint_name_to_dof_idx:
            continue
        joint_idx = joint_name_to_dof_idx[joint_name]
        targets[joint_idx] = (
            (1.0 - blend) * targets[joint_idx]
            + blend * optimized_joint_angles[joint_idx]
        )


def compute_load_shift_progress(phase: ControlPhase, phase_start_time: float, t: float) -> float:
    """Compute normalized load-transfer progress for posture shaping."""
    if phase in (ControlPhase.INIT_SETTLE, ControlPhase.DOUBLE_SUPPORT_HOLD):
        return 0.0
    if phase == ControlPhase.LOAD_SHIFT:
        return min(1.0, (t - phase_start_time) / max(LOAD_SHIFT_TIME, 1e-6))
    return 1.0


def compute_preliftoff_unload(
    phase: ControlPhase,
    phase_start_time: float,
    t: float,
    load_shift_metrics: LoadShiftMetrics,
) -> float:
    """Compute how strongly PRE_LIFTOFF should unload the swing foot."""
    if phase != ControlPhase.PRE_LIFTOFF:
        return 0.0
    elapsed = t - phase_start_time
    time_progress = min(1.0, elapsed / max(PRE_LIFTOFF_TIME, 1e-6))
    force_ratio = min(
        1.0, load_shift_metrics.swing_force / max(2.0 * PRE_LIFTOFF_SWING_FORCE_MAX, 1e-6)
    )
    slip_guard = max(
        0.0, 1.0 - load_shift_metrics.swing_slip / max(SLIP_THRESH, 1e-6)
    )
    return time_progress * force_ratio * slip_guard


def compute_single_support_entry_progress(
    phase: ControlPhase, phase_start_time: float, t: float
) -> float:
    """Normalized progress through the guarded single-support entry window."""
    if phase != ControlPhase.SINGLE_SUPPORT:
        return 1.0
    return min(1.0, (t - phase_start_time) / max(SINGLE_SUPPORT_ENTRY_TIME, 1e-6))


def compute_single_support_unload(
    phase: ControlPhase,
    phase_start_time: float,
    load_shift_metrics: LoadShiftMetrics,
) -> float:
    """Keep unloading the swing foot if it still carries too much contact in single support."""
    if phase != ControlPhase.SINGLE_SUPPORT:
        return 0.0
    force_error = max(
        0.0,
        load_shift_metrics.swing_force - SINGLE_SUPPORT_SWING_FORCE_TARGET,
    )
    if force_error <= 0.0:
        return 0.0
    slip_guard = max(
        0.0, 1.0 - load_shift_metrics.swing_slip / max(2.0 * SLIP_THRESH, 1e-6)
    )
    return min(
        0.5,
        0.5 * force_error / max(SINGLE_SUPPORT_SWING_FORCE_TARGET, 1e-6),
    ) * slip_guard


def compute_swing_unload_factor(
    phase: ControlPhase,
    phase_start_time: float,
    t: float,
    load_shift_metrics: LoadShiftMetrics,
) -> float:
    """How much swing-leg relaxation to apply around the pre-liftoff handoff."""
    if phase == ControlPhase.PRE_LIFTOFF:
        return compute_preliftoff_unload(phase, phase_start_time, t, load_shift_metrics)
    if phase == ControlPhase.SINGLE_SUPPORT:
        entry_tail = max(0.0, 1.0 - compute_single_support_entry_progress(phase, phase_start_time, t))
        contact_tail = compute_single_support_unload(phase, phase_start_time, load_shift_metrics)
        return max(entry_tail, contact_tail)
    return 0.0


def compute_swing_progress(
    phase: ControlPhase,
    phase_start_time: float,
    t: float,
    load_shift_metrics: LoadShiftMetrics,
) -> float:
    """Compute normalized swing progress based on the active phase."""
    elapsed = t - phase_start_time
    if phase in (ControlPhase.INIT_SETTLE, ControlPhase.DOUBLE_SUPPORT_HOLD, ControlPhase.LOAD_SHIFT):
        return 0.0
    if phase == ControlPhase.PRE_LIFTOFF:
        base_progress = PRE_LIFTOFF_SWING_PROGRESS * min(1.0, elapsed / max(PRE_LIFTOFF_TIME, 1e-6))
        unload_progress = compute_preliftoff_unload(phase, phase_start_time, t, load_shift_metrics)
        return min(0.6, base_progress + PRE_LIFTOFF_EXTRA_SWING_PROGRESS * unload_progress)
    single_support_elapsed = max(0.0, elapsed - SINGLE_SUPPORT_ENTRY_TIME)
    single_support_progress = min(
        1.0,
        single_support_elapsed / max(SINGLE_SUPPORT_SWING_RAMP_TIME, 1e-6),
    )
    return min(
        SINGLE_SUPPORT_SWING_PROGRESS_MAX,
        PRE_LIFTOFF_SWING_PROGRESS
        + (SINGLE_SUPPORT_SWING_PROGRESS_MAX - PRE_LIFTOFF_SWING_PROGRESS)
        * single_support_progress,
    )


def _apply_load_shift_targets(
    targets: np.ndarray,
    joint_name_to_dof_idx: dict[str, int],
    phase: ControlPhase,
    phase_start_time: float,
    t: float,
    load_shift_metrics: LoadShiftMetrics,
    support_leg: str,
) -> None:
    """LOAD_SHIFT phase: roll toward support foot based on force/com error."""
    target_ratio = LOAD_SHIFT_SUPPORT_RATIO
    target_com_ratio = LOAD_SHIFT_COM_RATIO
    ratio_error = max(0.0, target_ratio - load_shift_metrics.support_ratio)
    com_error = max(0.0, target_com_ratio - load_shift_metrics.com_shift_ratio)
    roll_feedback = LOAD_SHIFT_ROLL_DELTA * min(
        1.0,
        8.0
        * (
            0.7 * ratio_error / max(target_ratio, 1e-6)
            + 0.3 * com_error / max(target_com_ratio, 1e-6)
        ),
    )
    load_shift_progress = compute_load_shift_progress(phase, phase_start_time, t)
    roll_delta = min(load_shift_progress * LOAD_SHIFT_ROLL_DELTA, roll_feedback)
    slip_scale = max(0.0, 1.0 - load_shift_metrics.swing_slip / max(SLIP_THRESH, 1e-6))
    roll_delta *= slip_scale
    apply_roll_shift_offset(targets, joint_name_to_dof_idx, support_leg, roll_delta)


def _apply_pre_liftoff_targets(
    targets: np.ndarray,
    joint_name_to_dof_idx: dict[str, int],
    phase: ControlPhase,
    phase_start_time: float,
    t: float,
    load_shift_metrics: LoadShiftMetrics,
    c: np.ndarray,
    c_dot: np.ndarray,
    c_ref: np.ndarray,
    optimized_joint_angles: np.ndarray | None,
    preliftoff_pose_joint_names: list[str],
    support_leg: str,
) -> None:
    """PRE_LIFTOFF phase: roll, pose blend, and pitch balance."""
    target_ratio = PRE_LIFTOFF_SUPPORT_RATIO
    target_com_ratio = PRE_LIFTOFF_COM_RATIO
    ratio_error = max(0.0, target_ratio - load_shift_metrics.support_ratio)
    com_error = max(0.0, target_com_ratio - load_shift_metrics.com_shift_ratio)
    roll_feedback = LOAD_SHIFT_ROLL_DELTA * min(
        1.0,
        8.0
        * (
            0.7 * ratio_error / max(target_ratio, 1e-6)
            + 0.3 * com_error / max(target_com_ratio, 1e-6)
        ),
    )
    load_shift_progress = compute_load_shift_progress(phase, phase_start_time, t)
    roll_delta = min(load_shift_progress * LOAD_SHIFT_ROLL_DELTA, roll_feedback)

    swing_force_error = max(
        0.0, load_shift_metrics.swing_force - PRE_LIFTOFF_SWING_FORCE_MAX
    )
    roll_delta += PRE_LIFTOFF_EXTRA_ROLL_DELTA * min(
        1.0, swing_force_error / max(PRE_LIFTOFF_SWING_FORCE_MAX, 1e-6)
    )

    slip_scale = max(0.0, 1.0 - load_shift_metrics.swing_slip / max(SLIP_THRESH, 1e-6))
    roll_delta *= slip_scale
    apply_roll_shift_offset(targets, joint_name_to_dof_idx, support_leg, roll_delta)

    time_progress = min(
        1.0,
        (t - phase_start_time) / max(PRE_LIFTOFF_TIME, 1e-6),
    )
    pose_blend = 0.75 * time_progress * slip_scale
    blend_joint_targets_toward_pose(
        targets,
        joint_name_to_dof_idx,
        optimized_joint_angles,
        preliftoff_pose_joint_names,
        pose_blend,
    )
    pos_clip = 0.08
    vel_clip = 0.20
    forward_error = np.clip(c[0] - c_ref[0], -pos_clip, pos_clip)
    forward_velocity = np.clip(c_dot[0], -vel_clip, vel_clip)
    pitch_delta = (
        -0.10 * (forward_error / max(pos_clip, 1e-6))
        -0.06 * (forward_velocity / max(vel_clip, 1e-6))
    )
    pitch_delta = float(np.clip(pitch_delta, -0.12, 0.12))
    apply_pitch_balance_offset(targets, joint_name_to_dof_idx, support_leg, pitch_delta)


def _apply_single_support_targets(
    targets: np.ndarray,
    joint_name_to_dof_idx: dict[str, int],
    phase: ControlPhase,
    phase_start_time: float,
    t: float,
    c: np.ndarray,
    c_ref: np.ndarray,
    load_shift_metrics: LoadShiftMetrics,
    support_leg: str,
) -> None:
    """SINGLE_SUPPORT phase: roll and pitch balance."""
    target_ratio = SINGLE_SUPPORT_SUPPORT_RATIO
    target_com_ratio = SINGLE_SUPPORT_COM_RATIO
    ratio_error = max(0.0, target_ratio - load_shift_metrics.support_ratio)
    com_error = max(0.0, target_com_ratio - load_shift_metrics.com_shift_ratio)
    roll_feedback = LOAD_SHIFT_ROLL_DELTA * min(
        1.0,
        8.0
        * (
            0.7 * ratio_error / max(target_ratio, 1e-6)
            + 0.3 * com_error / max(target_com_ratio, 1e-6)
        ),
    )
    load_shift_progress = compute_load_shift_progress(phase, phase_start_time, t)
    roll_delta = min(load_shift_progress * LOAD_SHIFT_ROLL_DELTA, roll_feedback)

    swing_force_error = max(
        0.0, load_shift_metrics.swing_force - SINGLE_SUPPORT_SWING_FORCE_TARGET
    )
    roll_delta += SINGLE_SUPPORT_EXTRA_ROLL_DELTA * min(
        1.0, swing_force_error / max(SINGLE_SUPPORT_SWING_FORCE_TARGET, 1e-6)
    )

    slip_scale = max(0.0, 1.0 - load_shift_metrics.swing_slip / max(SLIP_THRESH, 1e-6))
    roll_delta *= slip_scale
    apply_roll_shift_offset(targets, joint_name_to_dof_idx, support_leg, roll_delta)

    forward_error = np.clip(
        c[0] - c_ref[0],
        -SINGLE_SUPPORT_FORWARD_ERROR_CLIP,
        SINGLE_SUPPORT_FORWARD_ERROR_CLIP,
    )
    pitch_delta = -SINGLE_SUPPORT_SUPPORT_HIP_PITCH_DELTA * (
        forward_error / max(SINGLE_SUPPORT_FORWARD_ERROR_CLIP, 1e-6)
    )
    apply_pitch_balance_offset(targets, joint_name_to_dof_idx, support_leg, pitch_delta)


def build_safe_targets(
    initial_dof_angles: np.ndarray,
    joint_name_to_dof_idx: dict[str, int],
    phase: ControlPhase,
    phase_start_time: float,
    single_support_joint_ref: np.ndarray | None,
    t: float,
    swing_leg: str,
    support_leg: str,
    c: np.ndarray,
    c_dot: np.ndarray,
    c_ref: np.ndarray,
    load_shift_metrics: LoadShiftMetrics,
    optimized_joint_angles: np.ndarray | None = None,
) -> np.ndarray:
    """Build posture targets for the current phase."""
    if (
        phase == ControlPhase.SINGLE_SUPPORT
        and single_support_joint_ref is not None
    ):
        targets = single_support_joint_ref.copy()
    else:
        targets = initial_dof_angles.copy()

    swing_progress = compute_swing_progress(phase, phase_start_time, t, load_shift_metrics)
    load_shift_progress = compute_load_shift_progress(phase, phase_start_time, t)

    hip_name = f"{swing_leg}_hip_pitch_joint"
    knee_name = f"{swing_leg}_knee_joint"
    ankle_pitch_name = f"{swing_leg}_ankle_pitch_joint"

    preliftoff_pose_joint_names = [
        f"{support_leg}_hip_pitch_joint",
        f"{support_leg}_knee_joint",
        f"{support_leg}_ankle_pitch_joint",
        f"{support_leg}_hip_roll_joint",
        f"{support_leg}_ankle_roll_joint",
        f"{swing_leg}_hip_roll_joint",
        f"{swing_leg}_ankle_roll_joint",
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_elbow_joint",
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_elbow_joint",
    ]

    if load_shift_progress > 0.0:
        if phase == ControlPhase.LOAD_SHIFT:
            _apply_load_shift_targets(
                targets, joint_name_to_dof_idx, phase, phase_start_time, t, load_shift_metrics, support_leg
            )
        elif phase == ControlPhase.PRE_LIFTOFF:
            _apply_pre_liftoff_targets(
                targets,
                joint_name_to_dof_idx,
                phase,
                phase_start_time,
                t,
                load_shift_metrics,
                c,
                c_dot,
                c_ref,
                optimized_joint_angles,
                preliftoff_pose_joint_names,
                support_leg,
            )
        elif phase == ControlPhase.SINGLE_SUPPORT:
            _apply_single_support_targets(
                targets,
                joint_name_to_dof_idx,
                phase,
                phase_start_time,
                t,
                c,
                c_ref,
                load_shift_metrics,
                support_leg,
            )

    # Swing leg trajectory
    if phase == ControlPhase.SINGLE_SUPPORT and single_support_joint_ref is not None:
        pose_progress = min(
            1.0,
            (t - phase_start_time) / max(SINGLE_SUPPORT_POSE_BLEND_TIME, 1e-6),
        )
        opt_hip = (
            optimized_joint_angles[joint_name_to_dof_idx[hip_name]]
            if optimized_joint_angles is not None and hip_name in joint_name_to_dof_idx
            else SINGLE_SUPPORT_SWING_HIP_PITCH_TARGET
        )
        opt_knee = (
            optimized_joint_angles[joint_name_to_dof_idx[knee_name]]
            if optimized_joint_angles is not None and knee_name in joint_name_to_dof_idx
            else SINGLE_SUPPORT_SWING_KNEE_TARGET
        )
        opt_ankle = (
            optimized_joint_angles[joint_name_to_dof_idx[ankle_pitch_name]]
            if optimized_joint_angles is not None and ankle_pitch_name in joint_name_to_dof_idx
            else SINGLE_SUPPORT_SWING_ANKLE_PITCH_TARGET
        )
        if hip_name in joint_name_to_dof_idx:
            hip_idx = joint_name_to_dof_idx[hip_name]
            targets[hip_idx] = (
                (1.0 - pose_progress) * single_support_joint_ref[hip_idx]
                + pose_progress * opt_hip
            )
        if knee_name in joint_name_to_dof_idx:
            knee_idx = joint_name_to_dof_idx[knee_name]
            targets[knee_idx] = (
                (1.0 - pose_progress) * single_support_joint_ref[knee_idx]
                + pose_progress * opt_knee
            )
        if ankle_pitch_name in joint_name_to_dof_idx:
            ankle_pitch_idx = joint_name_to_dof_idx[ankle_pitch_name]
            targets[ankle_pitch_idx] = (
                (1.0 - pose_progress) * single_support_joint_ref[ankle_pitch_idx]
                + pose_progress * opt_ankle
            )
    elif hip_name in joint_name_to_dof_idx:
        hip_idx = joint_name_to_dof_idx[hip_name]
        targets[hip_idx] = (
            (1.0 - swing_progress) * initial_dof_angles[hip_idx]
            + swing_progress * SWING_HIP_PITCH_TARGET
        )

    if (
        phase != ControlPhase.SINGLE_SUPPORT
        or single_support_joint_ref is None
    ) and knee_name in joint_name_to_dof_idx:
        knee_idx = joint_name_to_dof_idx[knee_name]
        targets[knee_idx] = (
            (1.0 - swing_progress) * initial_dof_angles[knee_idx]
            + swing_progress * SWING_KNEE_TARGET
        )

    return targets


def compute_safe_tau(
    initial_dof_angles: np.ndarray,
    safe_targets: np.ndarray,
    joint_positions: np.ndarray,
    joint_velocities: np.ndarray,
    C_safe: np.ndarray,
    tau_min_limits: np.ndarray,
    tau_max_limits: np.ndarray,
    swing_leg_dof_indices: list[int],
    phase: ControlPhase,
    phase_start_time: float,
    t: float,
    load_shift_metrics: LoadShiftMetrics,
) -> np.ndarray:
    """Build posture-hold torques with bias-force compensation."""
    safe_tau = C_safe[6:] + compute_pd_torque(
        safe_targets,
        joint_positions,
        joint_velocities,
        POSTURE_KP,
        POSTURE_KD,
        tau_max_limits,
    )

    if swing_leg_dof_indices:
        swing_idx = np.array(swing_leg_dof_indices, dtype=int)
        unload_progress = compute_swing_unload_factor(phase, phase_start_time, t, load_shift_metrics)
        swing_kp_scale = 1.0 - unload_progress * (1.0 - PRE_LIFTOFF_SWING_KP_SCALE)
        swing_kd_scale = 1.0 - unload_progress * (1.0 - PRE_LIFTOFF_SWING_KD_SCALE)
        safe_tau[swing_idx] = (
            C_safe[6:][swing_idx]
            + compute_pd_torque(
                safe_targets[swing_idx],
                joint_positions[swing_idx],
                joint_velocities[swing_idx],
                LIFT_LEG_KP * swing_kp_scale,
                LIFT_LEG_KD * swing_kd_scale,
                tau_max_limits[swing_idx],
            )
        )

    return np.clip(safe_tau, tau_min_limits, tau_max_limits)
