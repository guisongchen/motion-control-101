"""Direct single-support control primitives: corner-patch contact and force reference."""

from __future__ import annotations

import mujoco
import numpy as np

from direct_single_support.config import DIRECT_SINGLE_SUPPORT_CONFIG as cfg
from robot_model import RobotModel


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
        has_box = False
        box_local_pos = None
        box_size = None
        for geom_id in range(robot.model.ngeom):
            if int(robot.model.geom_bodyid[geom_id]) != support_link:
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
        if not local_positions:
            raise ValueError(
                f"No corner-patch contact geoms found on support body {support_link}."
            )
        return sorted(local_positions, key=lambda pos: (pos[0], pos[1]))
    raise ValueError(f"Unsupported contact point mode: {mode}")


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
