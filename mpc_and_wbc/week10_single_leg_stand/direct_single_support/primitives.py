"""Direct single-support control primitives: corner-patch contact, CoP feedback, math utilities."""

from __future__ import annotations

import math

import mujoco
import numpy as np

from direct_single_support.config import DIRECT_SINGLE_SUPPORT_CONFIG as cfg
from robot_model import RobotModel


def quat_from_roll(roll: float) -> np.ndarray:
    """Build an xyzw quaternion from a pure roll angle."""
    return np.array([math.sin(roll / 2.0), 0.0, 0.0, math.cos(roll / 2.0)], dtype=float)


def wrap_to_pi(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_rotation(rotation: np.ndarray) -> float:
    """Extract world yaw from a rotation matrix."""
    return math.atan2(rotation[1, 0], rotation[0, 0])


def build_direct_pose(
    joint_names: list[str],
    support_leg: str,
    pose_cfg=None,
) -> np.ndarray:
    """Construct the tuned direct single-support pose in DOF order."""
    if pose_cfg is None:
        pose_cfg = cfg.pose
    swing_leg = "left" if support_leg == "right" else "right"
    support_sign = 1.0 if support_leg == "right" else -1.0
    pose = dict(cfg.env.standing_joint_angles)

    pose[f"{support_leg}_hip_pitch_joint"] = -0.28 - pose_cfg.support_pitch_delta
    pose[f"{support_leg}_knee_joint"] = 0.62 + 0.4 * pose_cfg.support_pitch_delta
    pose[f"{support_leg}_ankle_pitch_joint"] = -0.34 + 0.2 * pose_cfg.support_pitch_delta

    pose[f"{support_leg}_hip_roll_joint"] = support_sign * pose_cfg.support_roll_delta
    pose[f"{support_leg}_ankle_roll_joint"] = -support_sign * pose_cfg.support_roll_delta

    pose[f"{swing_leg}_hip_pitch_joint"] = pose_cfg.swing_hip_pitch
    pose[f"{swing_leg}_knee_joint"] = pose_cfg.swing_knee
    pose[f"{swing_leg}_ankle_pitch_joint"] = pose_cfg.swing_ankle_pitch
    pose[f"{swing_leg}_hip_roll_joint"] = support_sign * pose_cfg.swing_roll_delta
    pose[f"{swing_leg}_ankle_roll_joint"] = -support_sign * pose_cfg.swing_roll_delta

    pose[f"{support_leg}_shoulder_pitch_joint"] = pose_cfg.support_arm_shoulder_pitch
    pose[f"{support_leg}_shoulder_roll_joint"] = (
        support_sign * pose_cfg.support_arm_shoulder_roll
    )
    pose[f"{support_leg}_elbow_joint"] = support_sign * pose_cfg.support_arm_elbow

    q_target = np.zeros(len(joint_names))
    for idx, name in enumerate(joint_names):
        q_target[idx] = pose.get(name, 0.0)
    return q_target


def resolve_support_contact_local_positions(
    robot: RobotModel,
    support_link: int,
    initial_support_contact: np.ndarray,
) -> list[np.ndarray | None]:
    """Pick the support-foot points whose accelerations WBC constrains to zero."""
    mode = cfg.contact.wbc_contact_point
    if mode == "body_origin":
        return [None]
    if mode == "initial_contact":
        body_origin = np.array(robot.data.xpos[support_link], copy=True)
        body_rotation = np.array(robot.data.xmat[support_link]).reshape(3, 3)
        return [body_rotation.T @ (initial_support_contact - body_origin)]
    if mode == "corner_patch":
        local_positions: list[np.ndarray] = []
        for geom_id in range(robot.model.ngeom):
            if int(robot.model.geom_bodyid[geom_id]) != support_link:
                continue
            if int(robot.model.geom_type[geom_id]) != mujoco.mjtGeom.mjGEOM_SPHERE:
                continue
            local_positions.append(np.array(robot.model.geom_pos[geom_id], copy=True))
        if not local_positions:
            raise ValueError(
                f"No corner-patch sphere geoms found on support body {support_link}."
            )
        return sorted(local_positions, key=lambda pos: (pos[0], pos[1]))
    raise ValueError(f"Unsupported direct single-support contact point mode: {mode}")


def compute_corner_patch_force_reference(
    local_positions: list[np.ndarray | None],
    foot_origin: np.ndarray,
    foot_rotation: np.ndarray,
    cop_world: np.ndarray,
    total_normal_force: float,
) -> np.ndarray:
    """Distribute normal load across the four sole corners to realize a desired CoP."""
    if len(local_positions) != 4 or any(pos is None for pos in local_positions):
        return np.tile(
            np.array([0.0, 0.0, total_normal_force / len(local_positions)], dtype=float),
            len(local_positions),
        )

    corner_positions = [np.asarray(pos, dtype=float) for pos in local_positions]
    cop_local = foot_rotation.T @ (cop_world - foot_origin)
    x_coords = np.array([pos[0] for pos in corner_positions])
    y_coords = np.array([pos[1] for pos in corner_positions])
    x_min = float(np.min(x_coords)) + cfg.contact.cop_margin
    x_max = float(np.max(x_coords)) - cfg.contact.cop_margin
    y_min = float(np.min(y_coords)) + cfg.contact.cop_margin
    y_max = float(np.max(y_coords)) - cfg.contact.cop_margin

    target_x = float(np.clip(cop_local[0], x_min, x_max))
    target_y = float(np.clip(cop_local[1], y_min, y_max))
    x_alpha = (target_x - x_min) / max(x_max - x_min, 1e-6)
    y_alpha = (target_y - y_min) / max(y_max - y_min, 1e-6)
    weights = np.array(
        [
            (1.0 - x_alpha) * (1.0 - y_alpha),
            (1.0 - x_alpha) * y_alpha,
            x_alpha * (1.0 - y_alpha),
            x_alpha * y_alpha,
        ],
        dtype=float,
    )
    weights /= np.sum(weights)

    f_ref = np.zeros(3 * len(corner_positions), dtype=float)
    for contact_idx, weight in enumerate(weights):
        f_ref[3 * contact_idx + 2] = weight * total_normal_force
    return f_ref


def clip_corner_patch_cop_target(
    local_positions: list[np.ndarray | None],
    foot_origin: np.ndarray,
    foot_rotation: np.ndarray,
    cop_world: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Project a desired CoP into the interior of the corner patch."""
    if len(local_positions) != 4 or any(pos is None for pos in local_positions):
        return np.asarray(cop_world, dtype=float), np.zeros(3)

    corner_positions = [np.asarray(pos, dtype=float) for pos in local_positions]
    cop_local = foot_rotation.T @ (cop_world - foot_origin)
    x_coords = np.array([pos[0] for pos in corner_positions])
    y_coords = np.array([pos[1] for pos in corner_positions])
    x_min = float(np.min(x_coords)) + cfg.contact.cop_margin
    x_max = float(np.max(x_coords)) - cfg.contact.cop_margin
    y_min = float(np.min(y_coords)) + cfg.contact.cop_margin
    y_max = float(np.max(y_coords)) - cfg.contact.cop_margin

    clipped_local = cop_local.copy()
    clipped_local[0] = np.clip(clipped_local[0], x_min, x_max)
    clipped_local[1] = np.clip(clipped_local[1], y_min, y_max)
    clipped_world = foot_origin + foot_rotation @ clipped_local
    return clipped_world, clipped_local


def apply_measured_cop_feedback(
    local_positions: list[np.ndarray | None],
    foot_origin: np.ndarray,
    foot_rotation: np.ndarray,
    nominal_cop_world: np.ndarray,
    measured_cop_world: np.ndarray,
    measured_cop_velocity_world: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Shift the desired CoP using measured CoP error while staying inside the patch."""
    clipped_nominal_world, nominal_local = clip_corner_patch_cop_target(
        local_positions,
        foot_origin,
        foot_rotation,
        nominal_cop_world,
    )
    if len(local_positions) != 4 or any(pos is None for pos in local_positions):
        return clipped_nominal_world, nominal_local

    _, measured_local = clip_corner_patch_cop_target(
        local_positions,
        foot_origin,
        foot_rotation,
        measured_cop_world,
    )
    measured_velocity_local = foot_rotation.T @ measured_cop_velocity_world
    local_correction = np.zeros(3, dtype=float)
    local_correction[:2] = (
        cfg.cop.kp * (nominal_local[:2] - measured_local[:2])
        - cfg.cop.kd * measured_velocity_local[:2]
    )
    local_correction[:2] = np.clip(
        local_correction[:2],
        -cfg.cop.max_offset,
        cfg.cop.max_offset,
    )
    corrected_local = nominal_local + local_correction
    corrected_world, corrected_local = clip_corner_patch_cop_target(
        local_positions,
        foot_origin,
        foot_rotation,
        foot_origin + foot_rotation @ corrected_local,
    )
    return corrected_world, corrected_local


def build_corner_patch_wrench_task(
    local_positions: list[np.ndarray | None],
    foot_origin: np.ndarray,
    foot_rotation: np.ndarray,
    cop_world: np.ndarray,
    total_normal_force: float,
    desired_force_xy_world: np.ndarray | None = None,
    desired_yaw_moment: float = 0.0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Build a support-wrench task for the 4-corner foot patch."""
    if len(local_positions) != 4 or any(pos is None for pos in local_positions):
        raise ValueError("Support wrench task requires four explicit corner contact points.")
    if desired_force_xy_world is None:
        desired_force_xy_world = np.zeros(2, dtype=float)

    clipped_world, _ = clip_corner_patch_cop_target(
        local_positions,
        foot_origin,
        foot_rotation,
        cop_world,
    )
    task_matrix = np.zeros((6, 3 * len(local_positions)), dtype=float)
    for contact_idx, local_position in enumerate(local_positions):
        world_position = foot_origin + foot_rotation @ np.asarray(local_position, dtype=float)
        r = world_position - foot_origin
        col = 3 * contact_idx
        task_matrix[0:3, col : col + 3] = np.eye(3)
        task_matrix[3, col : col + 3] = np.array([0.0, -r[2], r[1]])
        task_matrix[4, col : col + 3] = np.array([r[2], 0.0, -r[0]])
        task_matrix[5, col : col + 3] = np.array([-r[1], r[0], 0.0])

    cop_offset = clipped_world - foot_origin
    task_ref = np.array(
        [
            desired_force_xy_world[0],
            desired_force_xy_world[1],
            total_normal_force,
            cop_offset[1] * total_normal_force,
            -cop_offset[0] * total_normal_force,
            desired_yaw_moment,
        ],
        dtype=float,
    )
    task_weight = np.diag(
        [
            cfg.wrench.force_xy_weight,
            cfg.wrench.force_xy_weight,
            cfg.wrench.force_z_weight,
            cfg.wrench.moment_weight,
            cfg.wrench.moment_weight,
            cfg.wrench.yaw_moment_weight,
        ]
    )
    return task_matrix, task_ref, task_weight


def compute_corner_patch_wrench_force_reference(
    task_matrix: np.ndarray,
    task_ref: np.ndarray,
) -> np.ndarray:
    """Project a desired aggregate wrench into a minimum-norm corner-force reference."""
    return np.linalg.pinv(task_matrix) @ task_ref


def resolve_support_cop_target_world(
    state: dict,
    c_ref: np.ndarray,
    initial_support_contact: np.ndarray,
) -> np.ndarray:
    """Choose the world-frame CoP target used to bias the patch force reference."""
    mode = cfg.contact.cop_target
    if mode == "initial_contact":
        return initial_support_contact
    if mode == "support_com_ref":
        return c_ref
    if mode == "current_com":
        return state["c"]
    raise ValueError(f"Unsupported direct single-support CoP target mode: {mode}")
