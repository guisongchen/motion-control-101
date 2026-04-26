"""DOUBLE_SUPPORT_HOLD phase: validate stability and choose support foot."""

from __future__ import annotations

from typing import Optional

import mujoco
import numpy as np

from config import (
    DOUBLE_SUPPORT_READY_TIME,
    DOUBLE_SUPPORT_MIN_FORCE,
    DOUBLE_SUPPORT_MAX_COM_VEL,
    DOUBLE_SUPPORT_MAX_L_NORM,
    DOUBLE_SUPPORT_FORCE_RATIO_MIN,
    DOUBLE_SUPPORT_COM_MARGIN,
    SLIP_THRESH,
)
from phase_core import ControlPhase, StabilityGate, get_contact_entry
from robot_model import RobotModel

_double_support_gate = StabilityGate()


def _foot_corners_world_xy(robot: RobotModel, link: int) -> list[np.ndarray]:
    """World-frame xy positions of the foot contact-patch corners (spheres or box)."""
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
        # Bottom-face corners in local frame
        local_corners = [
            np.array([cx - hx, cy - hy, cz - _hz]),
            np.array([cx - hx, cy + hy, cz - _hz]),
            np.array([cx + hx, cy - hy, cz - _hz]),
            np.array([cx + hx, cy + hy, cz - _hz]),
        ]
        return [(body_rotation @ lc)[:2] for lc in local_corners]
    return corners


def check_double_support_transition(
    t: float,
    state: dict,
    candidate_foot_links: list[int],
    initial_foot_pos: dict[int, np.ndarray],
    robot: RobotModel,
) -> Optional[ControlPhase]:
    """Transition to LOAD_SHIFT when both feet are stable and loaded."""
    c = state["c"]
    c_dot = state["c_dot"]
    L = state["L"]
    foot_contacts = state["foot_contacts"]

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

    both_feet_ready = all(force >= DOUBLE_SUPPORT_MIN_FORCE for force in double_support_forces)
    stable_slip = max_double_support_slip <= SLIP_THRESH

    com_vel_ok = float(np.linalg.norm(c_dot)) <= DOUBLE_SUPPORT_MAX_COM_VEL
    momentum_ok = float(np.linalg.norm(L)) <= DOUBLE_SUPPORT_MAX_L_NORM

    total_force = sum(double_support_forces)
    balanced = False
    if total_force > 1e-6:
        balanced = (min(double_support_forces) / total_force) >= DOUBLE_SUPPORT_FORCE_RATIO_MIN

    corners_xy: list[np.ndarray] = []
    for link in candidate_foot_links:
        corners_xy.extend(_foot_corners_world_xy(robot, link))
    com_inside_polygon = False
    if corners_xy:
        xs = [float(p[0]) for p in corners_xy]
        ys = [float(p[1]) for p in corners_xy]
        com_xy = c[:2]
        com_inside_polygon = (
            float(com_xy[0]) >= min(xs) + DOUBLE_SUPPORT_COM_MARGIN
            and float(com_xy[0]) <= max(xs) - DOUBLE_SUPPORT_COM_MARGIN
            and float(com_xy[1]) >= min(ys) + DOUBLE_SUPPORT_COM_MARGIN
            and float(com_xy[1]) <= max(ys) - DOUBLE_SUPPORT_COM_MARGIN
        )

    if _double_support_gate.check(
        both_feet_ready
        and stable_slip
        and com_vel_ok
        and momentum_ok
        and balanced
        and com_inside_polygon,
        t,
        DOUBLE_SUPPORT_READY_TIME,
    ):
        return ControlPhase.LOAD_SHIFT
    return None
