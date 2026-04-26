"""Smoothstep CoM trajectory from double support to single support."""

from __future__ import annotations

import numpy as np

def smoothstep(t: float) -> float:
    """C1-continuous smoothstep: 3t^2 - 2t^3."""
    t = float(np.clip(t, 0.0, 1.0))
    return t * t * (3.0 - 2.0 * t)


def smoothstep_d1(t: float) -> float:
    """First derivative of smoothstep: 6t(1 - t)."""
    t = float(np.clip(t, 0.0, 1.0))
    return 6.0 * t * (1.0 - t)


def smoothstep_d2(t: float) -> float:
    """Second derivative of smoothstep: 6(1 - 2t)."""
    t = float(np.clip(t, 0.0, 1.0))
    return 6.0 * (1.0 - 2.0 * t)


def compute_transition_com_trajectory(
    progress: float,
    start_xy: np.ndarray,
    end_xy: np.ndarray,
    duration: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Compute smoothstep CoM reference, velocity, and acceleration.

    Args:
        progress: normalized time progress [0, 1].
        start_xy: initial CoM xy (e.g., stance midpoint).
        end_xy: target CoM xy (e.g., support foot centroid).
        duration: total transition duration in seconds.

    Returns:
        c_ref, c_dot_ref, c_ddot_ref
    """
    s = smoothstep(progress)
    ds = smoothstep_d1(progress)
    d2s = smoothstep_d2(progress)

    inv_T = 1.0 / max(duration, 1e-6)
    inv_T2 = inv_T * inv_T

    c_ref = np.zeros(3)
    c_ref[:2] = (1.0 - s) * np.asarray(start_xy, dtype=float) + s * np.asarray(end_xy, dtype=float)

    delta_xy = np.asarray(end_xy, dtype=float) - np.asarray(start_xy, dtype=float)

    c_dot_ref = np.zeros(3)
    c_dot_ref[:2] = ds * inv_T * delta_xy

    c_ddot_ref = np.zeros(3)
    c_ddot_ref[:2] = d2s * inv_T2 * delta_xy

    return c_ref, c_dot_ref, c_ddot_ref


def build_transition_com_trajectory(
    start_xy: np.ndarray,
    end_xy: np.ndarray,
    duration: float,
    dt: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Precompute the full smoothstep CoM trajectory on the simulation grid."""
    num_samples = max(2, int(np.ceil(max(duration, 0.0) / max(dt, 1e-6))) + 1)
    progress_samples = np.linspace(0.0, 1.0, num_samples)

    c_refs = np.zeros((num_samples, 3))
    c_dot_refs = np.zeros((num_samples, 3))
    c_ddot_refs = np.zeros((num_samples, 3))

    for idx, progress in enumerate(progress_samples):
        c_ref, c_dot_ref, c_ddot_ref = compute_transition_com_trajectory(
            progress,
            start_xy,
            end_xy,
            duration,
        )
        c_refs[idx] = c_ref
        c_dot_refs[idx] = c_dot_ref
        c_ddot_refs[idx] = c_ddot_ref

    return c_refs, c_dot_refs, c_ddot_refs


def compute_foot_centroid_xy(robot, foot_link: int) -> np.ndarray:
    """World-frame xy centroid of the foot's contact geoms (spheres or box)."""
    import mujoco

    body_origin = np.array(robot.data.xpos[foot_link], copy=True)
    body_rotation = np.array(robot.data.xmat[foot_link]).reshape(3, 3)
    corners: list[np.ndarray] = []
    has_box = False
    box_local_pos = None
    box_size = None
    for geom_id in range(robot.model.ngeom):
        if int(robot.model.geom_bodyid[geom_id]) != foot_link:
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
        # Box centroid in world xy
        world_pos = body_origin + body_rotation @ box_local_pos
        return world_pos[:2]
    if corners:
        return np.mean(corners, axis=0)
    return body_origin[:2]
