"""Main loop: MPC + WBC + MuJoCo with explicit control phases."""

import json
from dataclasses import dataclass
from enum import Enum, auto
from pathlib import Path
from typing import Optional

import numpy as np

from config import (
    DT_SIM, WBC_FREQ, MPC_FREQ, SIM_DURATION,
    MODEL_PATH, LIFT_LEG, H_COM,
    FOOT_LINK_NAMES, STANDING_JOINT_ANGLES,
    SUPPORT_FOOT_NAME,
    BASE_INITIAL_POS, BASE_INITIAL_ORN,
    NX, NU, T_S, GRAVITY,
    RMSE_THRESH, SLIP_THRESH, MPC_TIME_THRESH, WBC_TIME_THRESH,
    POSTURE_KP, POSTURE_KD, LIFT_LEG_KP, LIFT_LEG_KD,
    TRANSITION_BLEND_TIME, SWING_RAMP_TIME, SINGLE_SUPPORT_SWING_RAMP_TIME,
    SINGLE_SUPPORT_SWING_PROGRESS_MAX,
    SWING_HIP_PITCH_TARGET, SWING_KNEE_TARGET, SUPPORT_POINT_FILTER,
    MIN_SUPPORT_FORCE, BASE_DOF_DAMPING, JOINT_DOF_DAMPING,
    INIT_SETTLE_TIME, DOUBLE_SUPPORT_READY_TIME, LOAD_SHIFT_TIME,
    PRE_LIFTOFF_TIME, DOUBLE_SUPPORT_MIN_FORCE, PRE_LIFTOFF_SWING_PROGRESS,
    LOAD_SHIFT_ROLL_DELTA, PRE_LIFTOFF_EXTRA_ROLL_DELTA,
    PRE_LIFTOFF_EXTRA_SWING_PROGRESS,
    PRE_LIFTOFF_SWING_KP_SCALE, PRE_LIFTOFF_SWING_KD_SCALE,
    SINGLE_SUPPORT_EXTRA_ROLL_DELTA,
    SINGLE_SUPPORT_ENTRY_TIME, SINGLE_SUPPORT_MPC_DELAY,
    SINGLE_SUPPORT_ESTABLISH_TIME, SINGLE_SUPPORT_ESTABLISH_SUPPORT_RATIO,
    SINGLE_SUPPORT_ESTABLISH_SWING_FORCE_MAX, SINGLE_SUPPORT_ESTABLISH_COM_SPEED,
    SINGLE_SUPPORT_POSE_BLEND_TIME,
    SINGLE_SUPPORT_FORCE_BLEND_TIME, SINGLE_SUPPORT_MIN_FORCE_RATIO, SINGLE_SUPPORT_HOLD_FORCE,
    SINGLE_SUPPORT_MAX_TAU_BLEND, SINGLE_SUPPORT_MAX_HORIZONTAL_FORCE,
    SINGLE_SUPPORT_SUPPORT_HIP_PITCH_DELTA, SINGLE_SUPPORT_FORWARD_ERROR_CLIP,
    SINGLE_SUPPORT_SWING_HIP_PITCH_TARGET, SINGLE_SUPPORT_SWING_KNEE_TARGET,
    SINGLE_SUPPORT_SWING_ANKLE_PITCH_TARGET,
    LOAD_SHIFT_SUPPORT_RATIO, PRE_LIFTOFF_SUPPORT_RATIO, SINGLE_SUPPORT_SUPPORT_RATIO,
    LOAD_SHIFT_COM_RATIO, PRE_LIFTOFF_COM_RATIO, SINGLE_SUPPORT_COM_RATIO,
    PRE_LIFTOFF_SWING_FORCE_MAX, SINGLE_SUPPORT_SWING_FORCE_TARGET, COM_VEL_READY_THRESH,
    PRE_LIFTOFF_FORWARD_ERROR_THRESH, PRE_LIFTOFF_FORWARD_VEL_THRESH,
    LOAD_SHIFT_READY_TIME, PRE_LIFTOFF_READY_TIME,
)
from robot_model import RobotModel
from state_estimator import StateEstimator
from mpc import CentroidalMPC
from wbc import WholeBodyController
from utils import (
    compute_rmse, plot_com_tracking, plot_contact_force, plot_torques,
    compute_pd_torque, get_leg_joint_names,
)

# Re-use proven single-support control primitives from direct_single_support benchmark
from direct_single_support import (
    resolve_support_contact_local_positions,
    build_corner_patch_wrench_task,
    compute_corner_patch_force_reference,
    compute_corner_patch_wrench_force_reference,
    apply_measured_cop_feedback,
    clip_corner_patch_cop_target,
    wrap_to_pi,
    yaw_from_rotation,
)
from direct_single_support_config import DIRECT_SINGLE_SUPPORT_CONFIG as direct_cfg


class ControlPhase(Enum):
    """High-level task phases for single-support standing."""

    INIT_SETTLE = auto()
    DOUBLE_SUPPORT_HOLD = auto()
    LOAD_SHIFT = auto()
    PRE_LIFTOFF = auto()
    SINGLE_SUPPORT = auto()


@dataclass
class PhaseState:
    """Track phase machine progress and persistent control context."""

    phase: ControlPhase
    phase_start_time: float
    ready_since: Optional[float]
    locked_support_foot_link: int
    filtered_support_point: np.ndarray
    last_valid_support_tau: Optional[np.ndarray]
    last_wbc_warn_step: int
    single_support_com_ref: Optional[np.ndarray]
    single_support_joint_ref: Optional[np.ndarray]
    single_support_ready_since: Optional[float]
    single_support_established: bool


@dataclass
class LoadShiftMetrics:
    """Physical readiness indicators for moving from double to single support."""

    support_force: float
    swing_force: float
    support_ratio: float
    com_shift_ratio: float
    com_speed: float
    support_slip: float
    swing_slip: float


def skew(v: np.ndarray) -> np.ndarray:
    """向量叉乘矩阵。"""
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ])


def compute_centroidal_dynamics(
    m: float, c: np.ndarray, p_foot: np.ndarray, dt: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Linearize centroidal dynamics around the current support point."""
    A_d = np.eye(9)
    A_d[0:3, 3:6] = dt * np.eye(3)

    B_d = np.zeros((9, 3))
    B_d[3:6, :] = dt * np.eye(3) / m
    r = p_foot - c
    B_d[6:9, :] = dt * skew(r)

    d_d = np.zeros(9)
    d_d[3:6] = dt * (-GRAVITY)

    return A_d, B_d, d_d


def format_vector(vec: np.ndarray) -> str:
    """Compact vector formatter for runtime diagnostics."""
    return "[" + ", ".join(f"{x:.2f}" for x in vec) + "]"


def get_contact_entry(foot_contacts: list[dict], link: int) -> Optional[dict]:
    """Return the contact record for one foot link."""
    return next((fc for fc in foot_contacts if fc["link"] == link), None)


def support_name(candidate_foot_links: list[int], link: int) -> str:
    """Map support link index back to the configured foot name."""
    return FOOT_LINK_NAMES[candidate_foot_links.index(link)]


def transition_phase(phase_state: PhaseState, new_phase: ControlPhase, t: float) -> None:
    """Switch phases and reset per-phase timers."""
    phase_state.phase = new_phase
    phase_state.phase_start_time = t
    phase_state.ready_since = None
    if new_phase != ControlPhase.SINGLE_SUPPORT:
        phase_state.single_support_com_ref = None
        phase_state.single_support_joint_ref = None
        phase_state.single_support_ready_since = None
        phase_state.single_support_established = False


def build_single_support_com_ref(
    c: np.ndarray,
    initial_foot_pos: dict[int, np.ndarray],
    support_foot_link: int,
    swing_foot_link: int,
) -> np.ndarray:
    """Anchor the single-support CoM target to the stable handoff state."""
    c_ref = c.copy()
    support_xy = initial_foot_pos[support_foot_link][:2]
    swing_xy = initial_foot_pos[swing_foot_link][:2]
    stance_midpoint = 0.5 * (support_xy + swing_xy)
    target_xy = stance_midpoint + SINGLE_SUPPORT_COM_RATIO * (support_xy - stance_midpoint)

    support_axis = support_xy - swing_xy
    support_axis_norm = np.linalg.norm(support_axis)
    if support_axis_norm > 1e-6:
        support_axis /= support_axis_norm
        current_projection = np.dot(c[:2] - stance_midpoint, support_axis)
        target_projection = np.dot(target_xy - stance_midpoint, support_axis)
        if current_projection < target_projection:
            c_ref[:2] += (target_projection - current_projection) * support_axis
        # Damp forward/backward drift toward stance midpoint
        perp_axis = np.array([-support_axis[1], support_axis[0]])
        perp_error = np.dot(c[:2] - stance_midpoint, perp_axis)
        c_ref[:2] -= 0.5 * perp_error * perp_axis
    c_ref[2] = c[2]
    return c_ref


def choose_support_foot(
    foot_contacts: list[dict],
    preferred_support_foot_link: int,
) -> int:
    """Choose the support foot deterministically, preferring the configured side."""
    preferred_contact = get_contact_entry(foot_contacts, preferred_support_foot_link)
    if (
        preferred_contact is not None
        and preferred_contact["normal_force"] >= MIN_SUPPORT_FORCE
    ):
        return preferred_support_foot_link
    return max(foot_contacts, key=lambda fc: fc["normal_force"])["link"]


def phase_elapsed(phase_state: PhaseState, t: float) -> float:
    """Elapsed wall-clock simulation time inside the current phase."""
    return t - phase_state.phase_start_time


def compute_preliftoff_unload(
    phase_state: PhaseState,
    t: float,
    load_shift_metrics: LoadShiftMetrics,
) -> float:
    """Compute how strongly PRE_LIFTOFF should unload the swing foot."""
    if phase_state.phase != ControlPhase.PRE_LIFTOFF:
        return 0.0
    elapsed = phase_elapsed(phase_state, t)
    time_progress = min(1.0, elapsed / max(PRE_LIFTOFF_TIME, 1e-6))
    force_ratio = min(
        1.0, load_shift_metrics.swing_force / max(2.0 * PRE_LIFTOFF_SWING_FORCE_MAX, 1e-6)
    )
    slip_guard = max(
        0.0, 1.0 - load_shift_metrics.swing_slip / max(SLIP_THRESH, 1e-6)
    )
    return time_progress * force_ratio * slip_guard


def compute_single_support_entry_progress(phase_state: PhaseState, t: float) -> float:
    """Normalized progress through the guarded single-support entry window."""
    if phase_state.phase != ControlPhase.SINGLE_SUPPORT:
        return 1.0
    return min(1.0, phase_elapsed(phase_state, t) / max(SINGLE_SUPPORT_ENTRY_TIME, 1e-6))


def compute_single_support_unload(
    phase_state: PhaseState,
    load_shift_metrics: LoadShiftMetrics,
) -> float:
    """Keep unloading the swing foot if it still carries too much contact in single support."""
    if phase_state.phase != ControlPhase.SINGLE_SUPPORT:
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


def world_point_to_local_body_point(
    robot: RobotModel,
    link: int,
    world_point: np.ndarray,
) -> np.ndarray:
    """Express a world-frame contact point in the queried body frame."""
    foot_origin = np.array(robot.data.xpos[link], copy=True)
    foot_rotation = np.array(robot.data.xmat[link]).reshape(3, 3)
    return foot_rotation.T @ (np.asarray(world_point, dtype=float) - foot_origin)


def should_model_swing_contact(
    phase_state: PhaseState,
    load_shift_metrics: LoadShiftMetrics,
) -> bool:
    """Keep the swing foot in the WBC model while it still carries meaningful load."""
    return (
        phase_state.phase == ControlPhase.SINGLE_SUPPORT
        and load_shift_metrics.swing_force > SINGLE_SUPPORT_HOLD_FORCE
    )


def compute_swing_unload_factor(
    phase_state: PhaseState,
    t: float,
    load_shift_metrics: LoadShiftMetrics,
) -> float:
    """How much swing-leg relaxation to apply around the pre-liftoff handoff."""
    if phase_state.phase == ControlPhase.PRE_LIFTOFF:
        return compute_preliftoff_unload(phase_state, t, load_shift_metrics)
    if phase_state.phase == ControlPhase.SINGLE_SUPPORT:
        entry_tail = max(0.0, 1.0 - compute_single_support_entry_progress(phase_state, t))
        contact_tail = compute_single_support_unload(phase_state, load_shift_metrics)
        return max(entry_tail, contact_tail)
    return 0.0


def compute_swing_progress(
    phase_state: PhaseState,
    t: float,
    load_shift_metrics: LoadShiftMetrics,
) -> float:
    """Compute normalized swing progress based on the active phase."""
    elapsed = phase_elapsed(phase_state, t)
    if phase_state.phase in (ControlPhase.INIT_SETTLE, ControlPhase.DOUBLE_SUPPORT_HOLD, ControlPhase.LOAD_SHIFT):
        return 0.0
    if phase_state.phase == ControlPhase.PRE_LIFTOFF:
        base_progress = PRE_LIFTOFF_SWING_PROGRESS * min(1.0, elapsed / max(PRE_LIFTOFF_TIME, 1e-6))
        unload_progress = compute_preliftoff_unload(phase_state, t, load_shift_metrics)
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


def compute_load_shift_progress(phase_state: PhaseState, t: float) -> float:
    """Compute normalized load-transfer progress for posture shaping."""
    if phase_state.phase in (ControlPhase.INIT_SETTLE, ControlPhase.DOUBLE_SUPPORT_HOLD):
        return 0.0
    if phase_state.phase == ControlPhase.LOAD_SHIFT:
        return min(1.0, phase_elapsed(phase_state, t) / max(LOAD_SHIFT_TIME, 1e-6))
    return 1.0


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


def update_single_support_establishment(
    phase_state: PhaseState,
    t: float,
    load_shift_metrics: LoadShiftMetrics,
) -> None:
    """Only enable optimization after one-leg support looks physically established."""
    if phase_state.phase != ControlPhase.SINGLE_SUPPORT:
        phase_state.single_support_ready_since = None
        phase_state.single_support_established = False
        return
    if phase_state.single_support_established:
        return

    established_ready = (
        load_shift_metrics.support_force >= MIN_SUPPORT_FORCE
        and load_shift_metrics.support_ratio >= SINGLE_SUPPORT_ESTABLISH_SUPPORT_RATIO
        and load_shift_metrics.swing_force <= SINGLE_SUPPORT_ESTABLISH_SWING_FORCE_MAX
        and load_shift_metrics.com_speed <= SINGLE_SUPPORT_ESTABLISH_COM_SPEED
        and load_shift_metrics.support_slip <= SLIP_THRESH
        and load_shift_metrics.swing_slip <= SLIP_THRESH
    )
    if not established_ready:
        phase_state.single_support_ready_since = None
        return
    if phase_state.single_support_ready_since is None:
        phase_state.single_support_ready_since = t
        return
    if t - phase_state.single_support_ready_since >= SINGLE_SUPPORT_ESTABLISH_TIME:
        phase_state.single_support_established = True


def compute_load_shift_metrics(
    c: np.ndarray,
    c_dot: np.ndarray,
    foot_contacts: list[dict],
    support_foot_link: int,
    swing_foot_link: int,
    initial_foot_pos: dict[int, np.ndarray],
) -> LoadShiftMetrics:
    """Measure whether weight has actually moved onto the intended support foot."""
    support_contact = get_contact_entry(foot_contacts, support_foot_link)
    swing_contact = get_contact_entry(foot_contacts, swing_foot_link)
    support_force = support_contact["normal_force"] if support_contact is not None else 0.0
    swing_force = swing_contact["normal_force"] if swing_contact is not None else 0.0
    total_force = max(support_force + swing_force, 1e-6)
    support_ratio = support_force / total_force
    support_pos = (
        support_contact["position"] if support_contact is not None else initial_foot_pos[support_foot_link]
    )
    swing_pos = (
        swing_contact["position"] if swing_contact is not None else initial_foot_pos[swing_foot_link]
    )

    support_xy = initial_foot_pos[support_foot_link][:2]
    swing_xy = initial_foot_pos[swing_foot_link][:2]
    stance_vec = support_xy - swing_xy
    stance_half_width = max(0.5 * np.linalg.norm(stance_vec), 1e-6)
    stance_axis = stance_vec / (2.0 * stance_half_width)
    stance_midpoint = 0.5 * (support_xy + swing_xy)
    com_shift_ratio = float(np.dot(c[:2] - stance_midpoint, stance_axis) / stance_half_width)

    return LoadShiftMetrics(
        support_force=support_force,
        swing_force=swing_force,
        support_ratio=support_ratio,
        com_shift_ratio=com_shift_ratio,
        com_speed=float(np.linalg.norm(c_dot[:2])),
        support_slip=float(np.linalg.norm(support_pos[:2] - initial_foot_pos[support_foot_link][:2])),
        swing_slip=float(np.linalg.norm(swing_pos[:2] - initial_foot_pos[swing_foot_link][:2])),
    )


def compute_phase_com_target(
    nominal_c_ref: np.ndarray,
    phase_state: PhaseState,
    initial_foot_pos: dict[int, np.ndarray],
    support_foot_link: int,
    swing_foot_link: int,
) -> np.ndarray:
    """Shift the CoM reference toward the support foot before liftoff."""
    c_ref = nominal_c_ref.copy()
    if phase_state.phase in (ControlPhase.INIT_SETTLE, ControlPhase.DOUBLE_SUPPORT_HOLD):
        return c_ref
    if (
        phase_state.phase == ControlPhase.SINGLE_SUPPORT
        and phase_state.single_support_com_ref is not None
    ):
        return phase_state.single_support_com_ref.copy()

    support_xy = initial_foot_pos[support_foot_link][:2]
    swing_xy = initial_foot_pos[swing_foot_link][:2]
    stance_midpoint = 0.5 * (support_xy + swing_xy)
    if phase_state.phase == ControlPhase.LOAD_SHIFT:
        target_ratio = LOAD_SHIFT_COM_RATIO
    elif phase_state.phase == ControlPhase.PRE_LIFTOFF:
        target_ratio = PRE_LIFTOFF_COM_RATIO
    else:
        target_ratio = SINGLE_SUPPORT_COM_RATIO
    target_xy = stance_midpoint + target_ratio * (support_xy - stance_midpoint)
    c_ref[:2] = target_xy
    return c_ref


def compute_forward_handoff_metrics(
    c: np.ndarray,
    c_dot: np.ndarray,
    c_ref: np.ndarray,
) -> tuple[float, float]:
    """Return forward CoM position error and x velocity for handoff gating."""
    return float(c[0] - c_ref[0]), float(c_dot[0])


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


def build_safe_targets(
    initial_dof_angles: np.ndarray,
    joint_name_to_dof_idx: dict[str, int],
    phase_state: PhaseState,
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
        phase_state.phase == ControlPhase.SINGLE_SUPPORT
        and phase_state.single_support_joint_ref is not None
    ):
        targets = phase_state.single_support_joint_ref.copy()
    else:
        targets = initial_dof_angles.copy()
    swing_progress = compute_swing_progress(phase_state, t, load_shift_metrics)
    load_shift_progress = compute_load_shift_progress(phase_state, t)
    hip_name = f"{swing_leg}_hip_pitch_joint"
    knee_name = f"{swing_leg}_knee_joint"
    ankle_pitch_name = f"{swing_leg}_ankle_pitch_joint"
    preliftoff_pose_joint_names = [
        f"{support_leg}_hip_pitch_joint",
        f"{support_leg}_knee_joint",
        f"{support_leg}_ankle_pitch_joint",
        f"{support_leg}_hip_roll_joint",
        f"{support_leg}_ankle_roll_joint",
        f"{swing_leg}_hip_pitch_joint",
        f"{swing_leg}_knee_joint",
        f"{swing_leg}_ankle_pitch_joint",
        f"{swing_leg}_hip_roll_joint",
        f"{swing_leg}_ankle_roll_joint",
        f"{support_leg}_shoulder_pitch_joint",
        f"{support_leg}_shoulder_roll_joint",
        f"{support_leg}_elbow_joint",
    ]

    if load_shift_progress > 0.0:
        if phase_state.phase == ControlPhase.LOAD_SHIFT:
            target_ratio = LOAD_SHIFT_SUPPORT_RATIO
            target_com_ratio = LOAD_SHIFT_COM_RATIO
        elif phase_state.phase == ControlPhase.PRE_LIFTOFF:
            target_ratio = PRE_LIFTOFF_SUPPORT_RATIO
            target_com_ratio = PRE_LIFTOFF_COM_RATIO
        else:
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
        roll_delta = min(load_shift_progress * LOAD_SHIFT_ROLL_DELTA, roll_feedback)
        if phase_state.phase == ControlPhase.PRE_LIFTOFF:
            swing_force_error = max(
                0.0, load_shift_metrics.swing_force - PRE_LIFTOFF_SWING_FORCE_MAX
            )
            roll_delta += PRE_LIFTOFF_EXTRA_ROLL_DELTA * min(
                1.0, swing_force_error / max(PRE_LIFTOFF_SWING_FORCE_MAX, 1e-6)
            )
        elif phase_state.phase == ControlPhase.SINGLE_SUPPORT:
            swing_force_error = max(
                0.0, load_shift_metrics.swing_force - SINGLE_SUPPORT_SWING_FORCE_TARGET
            )
            roll_delta += SINGLE_SUPPORT_EXTRA_ROLL_DELTA * min(
                1.0, swing_force_error / max(SINGLE_SUPPORT_SWING_FORCE_TARGET, 1e-6)
            )
        slip_scale = max(0.0, 1.0 - load_shift_metrics.swing_slip / max(SLIP_THRESH, 1e-6))
        roll_delta *= slip_scale
        apply_roll_shift_offset(targets, joint_name_to_dof_idx, support_leg, roll_delta)
        if phase_state.phase == ControlPhase.PRE_LIFTOFF:
            time_progress = min(
                1.0,
                phase_elapsed(phase_state, t) / max(PRE_LIFTOFF_TIME, 1e-6),
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
        elif phase_state.phase == ControlPhase.SINGLE_SUPPORT:
            forward_error = np.clip(
                c[0] - c_ref[0],
                -SINGLE_SUPPORT_FORWARD_ERROR_CLIP,
                SINGLE_SUPPORT_FORWARD_ERROR_CLIP,
            )
            pitch_delta = -SINGLE_SUPPORT_SUPPORT_HIP_PITCH_DELTA * (
                forward_error / max(SINGLE_SUPPORT_FORWARD_ERROR_CLIP, 1e-6)
            )
            apply_pitch_balance_offset(targets, joint_name_to_dof_idx, support_leg, pitch_delta)

    if phase_state.phase == ControlPhase.SINGLE_SUPPORT and phase_state.single_support_joint_ref is not None:
        pose_progress = min(
            1.0,
            phase_elapsed(phase_state, t) / max(SINGLE_SUPPORT_POSE_BLEND_TIME, 1e-6),
        )
        # Use optimized-pose swing-leg targets when available
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
                (1.0 - pose_progress) * phase_state.single_support_joint_ref[hip_idx]
                + pose_progress * opt_hip
            )
        if knee_name in joint_name_to_dof_idx:
            knee_idx = joint_name_to_dof_idx[knee_name]
            targets[knee_idx] = (
                (1.0 - pose_progress) * phase_state.single_support_joint_ref[knee_idx]
                + pose_progress * opt_knee
            )
        if ankle_pitch_name in joint_name_to_dof_idx:
            ankle_pitch_idx = joint_name_to_dof_idx[ankle_pitch_name]
            targets[ankle_pitch_idx] = (
                (1.0 - pose_progress) * phase_state.single_support_joint_ref[ankle_pitch_idx]
                + pose_progress * opt_ankle
            )
    elif hip_name in joint_name_to_dof_idx:
        hip_idx = joint_name_to_dof_idx[hip_name]
        targets[hip_idx] = (
            (1.0 - swing_progress) * initial_dof_angles[hip_idx]
            + swing_progress * SWING_HIP_PITCH_TARGET
        )
    if (
        phase_state.phase != ControlPhase.SINGLE_SUPPORT
        or phase_state.single_support_joint_ref is None
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
    phase_state: PhaseState,
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
        unload_progress = compute_swing_unload_factor(phase_state, t, load_shift_metrics)
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


def update_phase_machine(
    phase_state: PhaseState,
    t: float,
    c: np.ndarray,
    c_dot: np.ndarray,
    foot_contacts: list[dict],
    preferred_support_foot_link: int,
    swing_foot_link: int,
    initial_foot_pos: dict[int, np.ndarray],
    candidate_foot_links: list[int],
    joint_positions: np.ndarray,
) -> Optional[str]:
    """Advance the high-level control phase when entry/exit conditions are met."""
    elapsed = phase_elapsed(phase_state, t)
    double_support_forces = [
        get_contact_entry(foot_contacts, link)["normal_force"]
        if get_contact_entry(foot_contacts, link) is not None
        else 0.0
        for link in candidate_foot_links
    ]
    max_double_support_slip = max(
        np.linalg.norm(
            get_contact_entry(foot_contacts, link)["position"][:2] - initial_foot_pos[link][:2]
        )
        for link in candidate_foot_links
    )
    support_contact = get_contact_entry(foot_contacts, phase_state.locked_support_foot_link)
    support_force = support_contact["normal_force"] if support_contact is not None else 0.0

    if phase_state.phase == ControlPhase.INIT_SETTLE:
        if elapsed >= INIT_SETTLE_TIME:
            transition_phase(phase_state, ControlPhase.DOUBLE_SUPPORT_HOLD, t)
            return "double-support validation"
        return None

    if phase_state.phase == ControlPhase.DOUBLE_SUPPORT_HOLD:
        both_feet_ready = all(force >= DOUBLE_SUPPORT_MIN_FORCE for force in double_support_forces)
        stable_slip = max_double_support_slip <= SLIP_THRESH
        if both_feet_ready and stable_slip:
            if phase_state.ready_since is None:
                phase_state.ready_since = t
            if t - phase_state.ready_since >= DOUBLE_SUPPORT_READY_TIME:
                phase_state.locked_support_foot_link = choose_support_foot(
                    foot_contacts, preferred_support_foot_link
                )
                phase_state.filtered_support_point = initial_foot_pos[
                    phase_state.locked_support_foot_link
                ].copy()
                transition_phase(phase_state, ControlPhase.LOAD_SHIFT, t)
                return (
                    "load shift "
                    f"(support={support_name(candidate_foot_links, phase_state.locked_support_foot_link)})"
                )
        else:
            phase_state.ready_since = None
        return None

    load_shift_metrics = compute_load_shift_metrics(
        c,
        c_dot,
        foot_contacts,
        phase_state.locked_support_foot_link,
        swing_foot_link,
        initial_foot_pos,
    )
    phase_com_ref = compute_phase_com_target(
        c,
        phase_state,
        initial_foot_pos,
        phase_state.locked_support_foot_link,
        swing_foot_link,
    )
    forward_error, forward_velocity = compute_forward_handoff_metrics(c, c_dot, phase_com_ref)

    if phase_state.phase == ControlPhase.LOAD_SHIFT:
        load_shift_ready = (
            elapsed >= LOAD_SHIFT_TIME
            and load_shift_metrics.support_force >= DOUBLE_SUPPORT_MIN_FORCE
            and load_shift_metrics.support_ratio >= LOAD_SHIFT_SUPPORT_RATIO
            and load_shift_metrics.com_shift_ratio >= LOAD_SHIFT_COM_RATIO
            and load_shift_metrics.com_speed <= COM_VEL_READY_THRESH
            and load_shift_metrics.support_slip <= SLIP_THRESH
            and load_shift_metrics.swing_slip <= SLIP_THRESH
        )
        if load_shift_ready:
            if phase_state.ready_since is None:
                phase_state.ready_since = t
            if t - phase_state.ready_since >= LOAD_SHIFT_READY_TIME:
                transition_phase(phase_state, ControlPhase.PRE_LIFTOFF, t)
                return (
                    "pre-liftoff "
                    f"(support_ratio={load_shift_metrics.support_ratio:.2f}, "
                    f"com_shift={load_shift_metrics.com_shift_ratio:.2f})"
                )
        else:
            phase_state.ready_since = None
        return None

    if phase_state.phase == ControlPhase.PRE_LIFTOFF:
        pre_liftoff_ready = (
            elapsed >= PRE_LIFTOFF_TIME
            and load_shift_metrics.support_force >= MIN_SUPPORT_FORCE
            and load_shift_metrics.support_ratio >= PRE_LIFTOFF_SUPPORT_RATIO
            and load_shift_metrics.swing_force <= PRE_LIFTOFF_SWING_FORCE_MAX
            and load_shift_metrics.com_shift_ratio >= PRE_LIFTOFF_COM_RATIO
            and load_shift_metrics.com_speed <= COM_VEL_READY_THRESH
            and abs(forward_error) <= PRE_LIFTOFF_FORWARD_ERROR_THRESH
            and abs(forward_velocity) <= PRE_LIFTOFF_FORWARD_VEL_THRESH
            and load_shift_metrics.support_slip <= SLIP_THRESH
            and load_shift_metrics.swing_slip <= SLIP_THRESH
        )
        if pre_liftoff_ready:
            if phase_state.ready_since is None:
                phase_state.ready_since = t
            if t - phase_state.ready_since >= PRE_LIFTOFF_READY_TIME:
                phase_state.single_support_com_ref = build_single_support_com_ref(
                    c,
                    initial_foot_pos,
                    phase_state.locked_support_foot_link,
                    swing_foot_link,
                )
                phase_state.single_support_joint_ref = joint_positions.copy()
                phase_state.single_support_ready_since = None
                phase_state.single_support_established = False
                transition_phase(phase_state, ControlPhase.SINGLE_SUPPORT, t)
                return (
                    "single-support control "
                    f"(support={support_name(candidate_foot_links, phase_state.locked_support_foot_link)}, "
                    f"support_ratio={load_shift_metrics.support_ratio:.2f}, "
                    f"swing_force={load_shift_metrics.swing_force:.1f}N, "
                    f"forward_error={forward_error:.3f}m, "
                    f"forward_vel={forward_velocity:.3f}m/s)"
                )
        else:
            phase_state.ready_since = None
        return None

    return None


def main():
    robot = RobotModel(MODEL_PATH)
    robot.model.opt.gravity[:] = GRAVITY
    robot.model.opt.timestep = DT_SIM
    robot.model.dof_damping[:6] = BASE_DOF_DAMPING
    robot.model.dof_damping[6:] = JOINT_DOF_DAMPING
    robot.reset_base_pose(BASE_INITIAL_POS, BASE_INITIAL_ORN)

    candidate_foot_links = [robot.link_name_to_index[name] for name in FOOT_LINK_NAMES]
    foot_name_to_link = {name: robot.link_name_to_index[name] for name in FOOT_LINK_NAMES}
    preferred_support_foot_link = foot_name_to_link[SUPPORT_FOOT_NAME]
    swing_leg = LIFT_LEG
    support_leg = "right" if swing_leg == "left" else "left"
    swing_foot_link = foot_name_to_link[f"{swing_leg}_ankle_roll_link"]
    estimator = StateEstimator(robot, candidate_foot_links)

    initial_dof_angles = np.zeros(len(robot.dof_joints))
    joint_name_to_dof_idx = robot.dof_joint_name_to_index.copy()
    for idx, joint_name in enumerate(robot.dof_joint_names):
        if joint_name in STANDING_JOINT_ANGLES:
            initial_dof_angles[idx] = STANDING_JOINT_ANGLES[joint_name]
    robot.reset_joint_positions(initial_dof_angles)

    tau_max_limits = robot.tau_limits.copy()
    tau_min_limits = -tau_max_limits
    swing_leg_dof_indices = [
        joint_name_to_dof_idx[name]
        for name in get_leg_joint_names(swing_leg)
        if name in joint_name_to_dof_idx
    ]
    support_leg_dof_indices = [
        joint_name_to_dof_idx[name]
        for name in get_leg_joint_names(support_leg)
        if name in joint_name_to_dof_idx
    ]

    mpc = CentroidalMPC()
    wbc = WholeBodyController(robot.nv)
    wbc_ss = WholeBodyController(robot.nv, num_contacts=4)
    wbc_ss_with_swing = WholeBodyController(robot.nv, num_contacts=5)

    # -----------------------------------------------------------------
    # Load optimized single-support pose for SINGLE_SUPPORT target
    # -----------------------------------------------------------------
    optimized_pose_path = Path(__file__).parent / "results" / "optimized_pose_simulation_3s.json"
    with open(optimized_pose_path, "r") as f:
        opt_data = json.load(f)
    optimized_joint_angles = np.array(opt_data["joint_angles"], dtype=float)
    optimized_base_pos = np.array(opt_data["base_position"], dtype=float)
    optimized_base_orn = np.array(opt_data["base_orientation"], dtype=float)

    # Pre-parse corner-patch contact points for the preferred support foot
    initial_support_metrics = robot.get_contact_metrics(preferred_support_foot_link)
    initial_support_contact = initial_support_metrics["position"].copy()
    initial_support_body = robot.get_link_com_position(preferred_support_foot_link).copy()
    support_contact_local_positions = resolve_support_contact_local_positions(
        robot, preferred_support_foot_link, initial_support_contact
    )

    # Single-support WBC gain override (same as direct_single_support.py)
    import wbc as wbc_module
    original_gains = (
        wbc_module.Kp_c,
        wbc_module.Kd_c,
        wbc_module.Kp_L,
        wbc_module.Kd_L,
    )

    # CoP / support position filter state (activated in SINGLE_SUPPORT)
    filtered_cop_world = np.array(initial_support_metrics["cop_position"], copy=True)
    prev_filtered_cop_world = filtered_cop_world.copy()
    filtered_support_position_world = np.array(initial_support_metrics["position"], copy=True)
    prev_filtered_support_position_world = filtered_support_position_world.copy()
    initial_support_yaw = yaw_from_rotation(
        np.array(robot.data.xmat[preferred_support_foot_link]).reshape(3, 3)
    )

    x_ref = np.zeros(NX)
    x_ref[2] = H_COM
    u_ref = np.zeros(NU)
    u_ref[2] = -GRAVITY[2] * robot.total_mass
    mpc.set_reference(x_ref, u_ref)

    nominal_c_ref = x_ref[:3].copy()
    c_ref = nominal_c_ref.copy()
    c_dot_ref = x_ref[3:6]
    L_ref = x_ref[6:9]
    c_ddot_ref = np.zeros(3)
    L_dot_ref = np.zeros(3)

    wbc_period = max(1, int(1.0 / (WBC_FREQ * DT_SIM)))
    mpc_period = max(1, int(1.0 / (MPC_FREQ * DT_SIM)))
    total_steps = int(SIM_DURATION / DT_SIM)

    time_log = []
    com_log = []
    com_ref_log = []
    foot_pos_log = {link: [] for link in candidate_foot_links}
    foot_force_log = {link: [] for link in candidate_foot_links}
    support_foot_log = []
    mpc_time_log = []
    wbc_time_log = []
    tau_log = []
    mpc_f_ref_log = []
    wbc_f_log = []

    initial_foot_pos = {link: robot.get_link_com_position(link) for link in candidate_foot_links}
    phase_state = PhaseState(
        phase=ControlPhase.INIT_SETTLE,
        phase_start_time=0.0,
        ready_since=None,
        locked_support_foot_link=preferred_support_foot_link,
        filtered_support_point=initial_foot_pos[preferred_support_foot_link].copy(),
        last_valid_support_tau=None,
        last_wbc_warn_step=-10_000,
        single_support_com_ref=None,
        single_support_joint_ref=None,
        single_support_ready_since=None,
        single_support_established=False,
    )

    mpc_result = None
    wbc_result = None
    f_ref = u_ref.copy()
    mpc_force_target = u_ref.copy()

    print("\n===== 开始仿真（MuJoCo 单足站立测试模式）=====")
    print(f"总质量: {robot.total_mass:.2f} kg")
    print(f"仿真时长: {SIM_DURATION:.1f} s")
    print(f"MPC 周期: {mpc_period} 步 ({mpc_period * DT_SIM * 1000:.1f} ms)")
    print(f"WBC 周期: {wbc_period} 步 ({wbc_period * DT_SIM * 1000:.3f} ms)")
    print("=" * 50)

    try:
        for step in range(total_steps):
            t = step * DT_SIM
            use_wbc = phase_state.phase == ControlPhase.SINGLE_SUPPORT
            lock_support = phase_state.phase in (
                ControlPhase.LOAD_SHIFT,
                ControlPhase.PRE_LIFTOFF,
                ControlPhase.SINGLE_SUPPORT,
            )
            state = estimator.update(
                preferred_support_foot_link=phase_state.locked_support_foot_link if lock_support else None,
                lock_support=lock_support,
            )
            c = state["c"]
            c_dot = state["c_dot"]
            L = state["L"]
            q = state["q"]
            v = state["v"]
            support_foot_link = state["support_foot_link"]
            foot_contacts = state["foot_contacts"]
    
            transition_msg = update_phase_machine(
                phase_state,
                t,
                c,
                c_dot,
                foot_contacts,
                preferred_support_foot_link,
                swing_foot_link,
                initial_foot_pos,
                candidate_foot_links,
                q[7:],
            )
            if transition_msg is not None:
                print(f"[INFO] t={t:.3f}s 进入阶段: {transition_msg}")
                if phase_state.phase == ControlPhase.SINGLE_SUPPORT:
                    # Override WBC gains to proven direct-single-support values
                    wbc_module.Kp_c = direct_cfg.control.com_kp
                    wbc_module.Kd_c = direct_cfg.control.com_kd
                    wbc_module.Kp_L = direct_cfg.control.momentum_kp
                    wbc_module.Kd_L = direct_cfg.control.momentum_kd
    
                    # Re-initialize corner-patch contact state from current pose
                    support_metrics_now = robot.get_contact_metrics(support_foot_link)
                    initial_support_contact = support_metrics_now["position"].copy()
                    initial_support_body = robot.get_link_com_position(support_foot_link).copy()
                    filtered_cop_world = np.array(support_metrics_now["cop_position"], copy=True)
                    prev_filtered_cop_world = filtered_cop_world.copy()
                    filtered_support_position_world = np.array(support_metrics_now["position"], copy=True)
                    prev_filtered_support_position_world = filtered_support_position_world.copy()
                    initial_support_yaw = yaw_from_rotation(
                        np.array(robot.data.xmat[support_foot_link]).reshape(3, 3)
                    )
    
                    # Resolve corner-patch local positions for the locked support foot
                    support_contact_local_positions = resolve_support_contact_local_positions(
                        robot, support_foot_link, initial_support_contact
                    )
    
            lock_support = phase_state.phase in (
                ControlPhase.LOAD_SHIFT,
                ControlPhase.PRE_LIFTOFF,
                ControlPhase.SINGLE_SUPPORT,
            )
            use_wbc = phase_state.phase == ControlPhase.SINGLE_SUPPORT
            if lock_support:
                support_foot_link = phase_state.locked_support_foot_link
                support_contact = get_contact_entry(foot_contacts, support_foot_link)
                measured_support_point = (
                    support_contact["position"].copy()
                    if support_contact is not None
                    else robot.get_link_com_position(support_foot_link)
                )
                phase_state.filtered_support_point = (
                    (1.0 - SUPPORT_POINT_FILTER) * phase_state.filtered_support_point
                    + SUPPORT_POINT_FILTER * measured_support_point
                )
                p_foot = phase_state.filtered_support_point.copy()
            else:
                p_foot = state["p_foot"]
    
            load_shift_metrics = compute_load_shift_metrics(
                c,
                c_dot,
                foot_contacts,
                phase_state.locked_support_foot_link,
                swing_foot_link,
                initial_foot_pos,
            )
            update_single_support_establishment(phase_state, t, load_shift_metrics)
            c_ref = compute_phase_com_target(
                nominal_c_ref,
                phase_state,
                initial_foot_pos,
                phase_state.locked_support_foot_link,
                swing_foot_link,
            )
            x_ref[:3] = c_ref
            mpc.set_reference(x_ref, u_ref)
    
            x0 = np.concatenate([c, c_dot, L])
    
            M = None
            C = None
            J_c = None
            entry_progress = compute_single_support_entry_progress(phase_state, t)
            if use_wbc:
                single_support_elapsed = phase_elapsed(phase_state, t)
                if (
                    single_support_elapsed >= SINGLE_SUPPORT_MPC_DELAY
                    and step % mpc_period == 0
                ):
                    A_d, B_d, d_d = compute_centroidal_dynamics(robot.total_mass, c, p_foot, T_S)
                    mpc.set_dynamics(A_d, B_d, d_d)
                    mpc_result = mpc.solve(x0)
                    if mpc_result is not None:
                        mpc_force_target = mpc_result["u0"]
                        mpc_time_log.append(mpc_result["solve_time"])
                    else:
                        print(f"[WARN] t={t:.3f}s MPC 求解失败")
                if single_support_elapsed >= SINGLE_SUPPORT_MPC_DELAY:
                    blend_progress = max(
                        0.0,
                        single_support_elapsed - SINGLE_SUPPORT_MPC_DELAY,
                    ) / max(SINGLE_SUPPORT_FORCE_BLEND_TIME, 1e-6)
                    mpc_blend = min(1.0, blend_progress)
                    f_ref = (1.0 - mpc_blend) * u_ref + mpc_blend * mpc_force_target
                else:
                    f_ref = u_ref.copy()
                f_ref[2] = max(f_ref[2], SINGLE_SUPPORT_MIN_FORCE_RATIO * u_ref[2])
                f_ref[:2] = np.clip(
                    f_ref[:2],
                    -SINGLE_SUPPORT_MAX_HORIZONTAL_FORCE,
                    SINGLE_SUPPORT_MAX_HORIZONTAL_FORCE,
                )
            else:
                f_ref = u_ref.copy()
                mpc_force_target = u_ref.copy()
    
            active_wbc_solver = wbc_ss
            if use_wbc and step % wbc_period == 0:
                M = robot.compute_mass_matrix(q)
                C = robot.compute_coriolis_gravity(q, v)
                modeled_swing_contact = should_model_swing_contact(
                    phase_state, load_shift_metrics
                )
                swing_metrics_pre = (
                    robot.get_contact_metrics(swing_foot_link)
                    if modeled_swing_contact
                    else None
                )
                contact_jacobians = [
                    robot.get_foot_jacobian(support_foot_link, q, local_position=lp)
                    for lp in support_contact_local_positions
                ]
                residual_swing_force = 0.0
                if (
                    swing_metrics_pre is not None
                    and swing_metrics_pre["normal_force"] > SINGLE_SUPPORT_HOLD_FORCE
                ):
                    swing_contact_local_position = world_point_to_local_body_point(
                        robot,
                        swing_foot_link,
                        swing_metrics_pre["cop_position"],
                    )
                    contact_jacobians.append(
                        robot.get_foot_jacobian(
                            swing_foot_link,
                            q,
                            local_position=swing_contact_local_position,
                        )
                    )
                    residual_swing_force = min(
                        float(swing_metrics_pre["normal_force"]),
                        max(float(f_ref[2]) - SINGLE_SUPPORT_HOLD_FORCE, 0.0),
                    )
                modeled_swing_contact = len(contact_jacobians) == 5
                active_wbc_ss = wbc_ss_with_swing if modeled_swing_contact else wbc_ss
                active_wbc_solver = active_wbc_ss
                J_c = np.vstack(contact_jacobians)
                J_com = robot.get_com_jacobian(q)
                J_L = robot.get_angular_momentum_jacobian(q)
                Jc_dot = np.zeros_like(J_c)
    
                c_ddot_des = wbc_ss.compute_desired_acceleration(
                    c_ref, c, c_dot_ref, c_dot, c_ddot_ref
                )
                L_dot_des = wbc_ss.compute_desired_momentum_rate(
                    L_ref, L, L_dot_ref, np.zeros(3)
                )
    
                # ---- Corner-patch CoP / slip / yaw control ----
                foot_origin = np.array(robot.data.xpos[support_foot_link], copy=True)
                foot_rotation = np.array(robot.data.xmat[support_foot_link]).reshape(3, 3)
    
                support_metrics_pre = robot.get_contact_metrics(support_foot_link)
                if support_metrics_pre["normal_force"] > 1e-6:
                    measured_cop_world = np.array(support_metrics_pre["cop_position"], copy=True)
                    filtered_cop_world = (
                        direct_cfg.cop.filter_alpha * measured_cop_world
                        + (1.0 - direct_cfg.cop.filter_alpha) * filtered_cop_world
                    )
                    measured_support_position_world = np.array(support_metrics_pre["position"], copy=True)
                    filtered_support_position_world = (
                        direct_cfg.wrench.state_filter_alpha * measured_support_position_world
                        + (1.0 - direct_cfg.wrench.state_filter_alpha) * filtered_support_position_world
                    )
    
                measured_cop_velocity_world = (
                    filtered_cop_world - prev_filtered_cop_world
                ) / DT_SIM
                prev_filtered_cop_world = filtered_cop_world.copy()
                measured_support_slip_velocity_world = (
                    filtered_support_position_world - prev_filtered_support_position_world
                ) / DT_SIM
                prev_filtered_support_position_world = filtered_support_position_world.copy()
    
                nominal_cop_target_world = initial_support_contact
                cop_target_world = nominal_cop_target_world
                if direct_cfg.cop.enabled:
                    cop_target_world, _ = apply_measured_cop_feedback(
                        support_contact_local_positions,
                        foot_origin,
                        foot_rotation,
                        nominal_cop_target_world,
                        filtered_cop_world,
                        measured_cop_velocity_world,
                    )
    
                support_slip_world = filtered_support_position_world - initial_support_contact
                desired_force_xy_world = -(
                    direct_cfg.wrench.slip_force_kp * support_slip_world[:2]
                    + direct_cfg.wrench.slip_force_kd * measured_support_slip_velocity_world[:2]
                )
                desired_force_xy_world = np.clip(
                    desired_force_xy_world,
                    -direct_cfg.wrench.slip_force_max,
                    direct_cfg.wrench.slip_force_max,
                )
                # Blend MPC horizontal force into the restoring force
                desired_force_xy_world += f_ref[:2]
    
                _, support_ang_vel = robot.get_link_velocity(support_foot_link)
                support_yaw_error = wrap_to_pi(yaw_from_rotation(foot_rotation) - initial_support_yaw)
                desired_yaw_moment = -(
                    direct_cfg.wrench.yaw_moment_kp * support_yaw_error
                    + direct_cfg.wrench.yaw_moment_kd * support_ang_vel[2]
                )
                desired_yaw_moment = float(np.clip(
                    desired_yaw_moment,
                    -direct_cfg.wrench.yaw_moment_max,
                    direct_cfg.wrench.yaw_moment_max,
                ))
    
                total_normal_force = max(f_ref[2] - residual_swing_force, SINGLE_SUPPORT_HOLD_FORCE)
                force_task_matrix, force_task_ref, force_task_weight = build_corner_patch_wrench_task(
                    support_contact_local_positions,
                    foot_origin,
                    foot_rotation,
                    cop_target_world,
                    total_normal_force,
                    desired_force_xy_world=desired_force_xy_world,
                    desired_yaw_moment=desired_yaw_moment,
                )
                f_ref_12d = compute_corner_patch_wrench_force_reference(
                    force_task_matrix, force_task_ref
                )
                if modeled_swing_contact:
                    swing_force_ref = np.array([0.0, 0.0, residual_swing_force], dtype=float)
                    f_ref_ss = np.concatenate([f_ref_12d, swing_force_ref])
                    force_task_matrix = np.hstack(
                        [force_task_matrix, np.zeros((force_task_matrix.shape[0], 3))]
                    )
                else:
                    f_ref_ss = f_ref_12d
    
                wbc_result = active_wbc_ss.solve(
                    M, C, J_c, np.zeros_like(J_c),
                    J_com, J_L,
                    c_ddot_des, L_dot_des,
                    f_ref_ss, v,
                    tau_min_limits, tau_max_limits,
                    force_task_matrix=force_task_matrix,
                    force_task_ref=force_task_ref,
                    force_task_weight=force_task_weight,
                )
                if wbc_result is None:
                    print(f"[WARN] t={t:.3f}s WBC 求解失败")
    
            # Gradually blend single-support joint reference toward optimized pose
            if (
                phase_state.phase == ControlPhase.SINGLE_SUPPORT
                and phase_state.single_support_joint_ref is not None
            ):
                pose_progress = min(
                    1.0,
                    phase_elapsed(phase_state, t) / max(SINGLE_SUPPORT_POSE_BLEND_TIME, 1e-6),
                )
                phase_state.single_support_joint_ref = (
                    (1.0 - pose_progress) * phase_state.single_support_joint_ref
                    + pose_progress * optimized_joint_angles
                )
    
            joint_positions = q[7:]
            joint_velocities = v[6:]
            C_safe = robot.compute_coriolis_gravity(q, v)
            safe_targets = build_safe_targets(
                initial_dof_angles,
                joint_name_to_dof_idx,
                phase_state,
                t,
                swing_leg,
                support_leg,
                c,
                c_dot,
                c_ref,
                load_shift_metrics,
                optimized_joint_angles=optimized_joint_angles,
            )
            safe_tau = compute_safe_tau(
                initial_dof_angles,
                safe_targets,
                joint_positions,
                joint_velocities,
                C_safe,
                tau_min_limits,
                tau_max_limits,
                swing_leg_dof_indices,
                phase_state,
                t,
                load_shift_metrics,
            )
    
            if not use_wbc:
                applied_tau = safe_tau.copy()
                robot.set_joint_torques(applied_tau)
            else:
                support_mask = np.ones(robot.num_joints, dtype=bool)
                support_mask[swing_leg_dof_indices] = False
                tau_cmd = safe_tau.copy()
                support_contact = get_contact_entry(foot_contacts, support_foot_link)
                support_force = support_contact["normal_force"] if support_contact is not None else 0.0
                hold_force_threshold = (
                    SINGLE_SUPPORT_HOLD_FORCE if phase_state.phase == ControlPhase.SINGLE_SUPPORT else MIN_SUPPORT_FORCE
                )
                support_contact_ready = support_force >= hold_force_threshold
    
                transition_alpha = min(
                    1.0,
                    max(0.0, phase_elapsed(phase_state, t) / max(TRANSITION_BLEND_TIME, SINGLE_SUPPORT_ENTRY_TIME)),
                )
                if wbc_result is not None:
                    phase_state.last_valid_support_tau = np.clip(
                        wbc_result["tau"][support_mask],
                        tau_min_limits[support_mask],
                        tau_max_limits[support_mask],
                    )
                elif phase_state.last_valid_support_tau is None:
                    phase_state.last_valid_support_tau = safe_tau[support_mask].copy()
    
                support_alpha = min(1.0, max(0.0, support_force / max(MIN_SUPPORT_FORCE, 1e-6)))
                max_tau_blend = (
                    SINGLE_SUPPORT_MAX_TAU_BLEND
                    if phase_state.phase == ControlPhase.SINGLE_SUPPORT
                    else 1.0
                )
                effective_alpha = min(
                    max_tau_blend,
                    transition_alpha * support_alpha,
                )
                tau_cmd[support_mask] = (
                    (1.0 - effective_alpha) * safe_tau[support_mask]
                    + effective_alpha * phase_state.last_valid_support_tau
                )
    
                if (wbc_result is None or support_force < hold_force_threshold) and step - phase_state.last_wbc_warn_step >= 60:
                    if J_c is not None and J_c.shape[0] % 6 == 0:
                        J_c_lin = np.vstack(
                            [J_c[6 * i : 6 * i + 3, :] for i in range(J_c.shape[0] // 6)]
                        )
                    else:
                        J_c_lin = J_c[:3, :] if J_c is not None else np.zeros((3, robot.nv))
                    J_c_gram = J_c_lin @ J_c_lin.T
                    J_c_cond = np.linalg.cond(J_c_gram + 1e-6 * np.eye(J_c_gram.shape[0]))
                    fallback_reason = (
                        f"support force below hold threshold ({support_force:.1f}N)"
                        if support_force < hold_force_threshold
                        else f"solver status={active_wbc_solver.last_status}"
                    )
                    print(
                        f"[WARN] t={t:.3f}s using fallback support torque: {fallback_reason}, "
                        f"phase={phase_state.phase.name}, "
                        f"support={support_name(candidate_foot_links, support_foot_link)}, "
                        f"force={support_force:.1f}N, alpha={effective_alpha:.2f}, "
                        f"rank(Jc)={np.linalg.matrix_rank(J_c_lin)}, "
                        f"cond(JcJc^T)={J_c_cond:.2e}, f_ref={format_vector(f_ref)}"
                    )
                    phase_state.last_wbc_warn_step = step
    
                applied_tau = tau_cmd.copy()
                robot.set_joint_torques(applied_tau)
    
            robot.step()
    
            time_log.append(t)
            com_log.append(c.copy())
            com_ref_log.append(x_ref[:3].copy())
            support_foot_log.append(support_foot_link)
    
            for fc in foot_contacts:
                link = fc["link"]
                foot_pos_log[link].append(fc["position"].copy())
                foot_force_log[link].append(fc["normal_force"])
    
            tau_log.append(applied_tau.copy())
            if wbc_result is not None:
                # Aggregate all modeled 3D contact forces for logging
                f_opt = wbc_result["f"]
                if f_opt.shape[0] % 3 == 0:
                    f_aggregate = np.zeros(3)
                    for i in range(f_opt.shape[0] // 3):
                        f_aggregate += f_opt[3 * i : 3 * i + 3]
                    wbc_f_log.append(f_aggregate)
                else:
                    wbc_f_log.append(f_opt.copy())
                wbc_time_log.append(wbc_result["solve_time"])
            else:
                wbc_f_log.append(np.zeros(3))
    
            if mpc_result is not None:
                mpc_f_ref_log.append(f_ref.copy())
            else:
                mpc_f_ref_log.append(np.zeros(3))
    
            if (step + 1) % 240 == 0:
                print(f"\n--- t={t:.3f}s [{phase_state.phase.name}] ---")
                print(
                    f"  CoM: [{c[0]:.3f}, {c[1]:.3f}, {c[2]:.3f}]  "
                    f"(ref [{c_ref[0]:.3f}, {c_ref[1]:.3f}, {c_ref[2]:.2f}])"
                )
                mpc_t = mpc_time_log[-1] * 1000 if mpc_time_log else 0.0
                wbc_t = wbc_time_log[-1] * 1000 if wbc_time_log else 0.0
                print(f"  MPC solve: {mpc_t:.2f} ms | WBC solve: {wbc_t:.2f} ms")
                print(
                    f"  Load shift: support_ratio={load_shift_metrics.support_ratio:.2f}  "
                    f"com_shift={load_shift_metrics.com_shift_ratio:.2f}  "
                    f"swing_force={load_shift_metrics.swing_force:.1f}N  "
                    f"com_speed={load_shift_metrics.com_speed:.3f}m/s  "
                    f"forward_error={c[0] - c_ref[0]:.3f}m  "
                    f"forward_vel={c_dot[0]:.3f}m/s  "
                    f"swing_slip={load_shift_metrics.swing_slip*1000:.2f}mm  "
                    f"unload={compute_swing_unload_factor(phase_state, t, load_shift_metrics):.2f}  "
                    f"entry={entry_progress:.2f}  "
                    f"established={int(phase_state.single_support_established)}"
                )
                for fc in foot_contacts:
                    link_name = support_name(candidate_foot_links, fc["link"])
                    slip = np.linalg.norm(fc["position"][:2] - initial_foot_pos[fc["link"]][:2])
                    print(f"  {link_name}: force={fc['normal_force']:.1f}N  slip={slip*1000:.2f}mm")
    
    finally:
        (
            wbc_module.Kp_c,
            wbc_module.Kd_c,
            wbc_module.Kp_L,
            wbc_module.Kd_L,
        ) = original_gains

    rmse = compute_rmse(com_log, com_ref_log)
    print(f"\n{'='*50}")
    print("===== 实验结果 =====")
    print(f"CoM 位置 RMSE: {rmse:.4f} m (目标 < {RMSE_THRESH} m)")

    max_slip = 0.0
    for link in candidate_foot_links:
        positions = np.array(foot_pos_log[link])
        if len(positions) > 0:
            slips = np.linalg.norm(positions[:, :2] - initial_foot_pos[link][:2], axis=1)
            max_slip = max(max_slip, np.max(slips))
    print(f"最大足端滑移: {max_slip*1000:.2f} mm (目标 < {SLIP_THRESH*1000:.1f} mm)")

    for link in candidate_foot_links:
        forces = np.array(foot_force_log[link])
        link_name = support_name(candidate_foot_links, link)
        avg_force = np.mean(forces) if len(forces) > 0 else 0.0
        print(f"{link_name} 平均接触力: {avg_force:.1f} N")

    if mpc_time_log:
        avg_mpc_time = np.mean(mpc_time_log) * 1000
        max_mpc_time = np.max(mpc_time_log) * 1000
        print(f"MPC 平均求解时间: {avg_mpc_time:.2f} ms (目标 < {MPC_TIME_THRESH*1000:.1f} ms)")
        print(f"MPC 最大求解时间: {max_mpc_time:.2f} ms")
    if wbc_time_log:
        avg_wbc_time = np.mean(wbc_time_log) * 1000
        max_wbc_time = np.max(wbc_time_log) * 1000
        print(f"WBC 平均求解时间: {avg_wbc_time:.3f} ms (目标 < {WBC_TIME_THRESH*1000:.1f} ms)")
        print(f"WBC 最大求解时间: {max_wbc_time:.3f} ms")

    plot_com_tracking(time_log, com_log, com_ref_log)
    if len(wbc_f_log) > 0:
        plot_contact_force(time_log, wbc_f_log)
    if len(tau_log) > 0:
        plot_torques(time_log, tau_log)


if __name__ == "__main__":
    main()
