"""DOUBLE_SUPPORT_HOLD phase: validate stability and choose support foot."""

from __future__ import annotations

from typing import Optional

import numpy as np

from config import (
    DOUBLE_SUPPORT_READY_TIME,
    DOUBLE_SUPPORT_MIN_FORCE,
    DOUBLE_SUPPORT_MAX_COM_VEL,
    DOUBLE_SUPPORT_MAX_L_NORM,
    DOUBLE_SUPPORT_FORCE_RATIO_MIN,
    SLIP_THRESH,
    MIN_SUPPORT_FORCE,
)
from phase_core import (
    ControlPhase,
    PhaseState,
    phase_elapsed,
    get_contact_entry,
    transition_phase,
)


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


def check_double_support_transition(
    phase_state: PhaseState,
    t: float,
    foot_contacts: list[dict],
    candidate_foot_links: list[int],
    initial_foot_pos: dict[int, np.ndarray],
    preferred_support_foot_link: int,
    c_dot: np.ndarray,
    L: np.ndarray,
    foot_name_map: dict[int, str],
) -> Optional[str]:
    """Transition to LOAD_SHIFT when both feet are stable and loaded."""
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

    if both_feet_ready and stable_slip and com_vel_ok and momentum_ok and balanced:
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
            from phase_core import support_name
            return (
                "load shift "
                f"(support={support_name(foot_name_map, phase_state.locked_support_foot_link)})"
            )
    else:
        phase_state.ready_since = None
    return None
