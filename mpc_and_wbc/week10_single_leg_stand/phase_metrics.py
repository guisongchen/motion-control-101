"""Physical readiness metrics for weight transfer and single-support validation."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from config import MIN_SUPPORT_FORCE, SLIP_THRESH
from phase_core import get_contact_entry


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
