"""PRE_LIFTOFF phase: unload swing foot and prepare for single support."""

from __future__ import annotations

from typing import Optional

import numpy as np

from config import (
    PRE_LIFTOFF_TIME,
    MIN_SUPPORT_FORCE,
    COM_VEL_READY_THRESH,
)
from phase_core import (
    ControlPhase,
    SingleSupportState,
)
from robot_model import RobotModel


def check_pre_liftoff_transition(
    elapsed: float,
    ss_state: SingleSupportState,
    state: dict,
    c_ref: np.ndarray,
    swing_foot_link: int,
    joint_positions: np.ndarray,
    initial_foot_pos: dict[int, np.ndarray],
    locked_support_foot_link: int,
    robot: RobotModel,
) -> Optional[ControlPhase]:
    """Transition to SINGLE_SUPPORT when pre-liftoff readiness is achieved."""
    load_shift_metrics = state["load_shift_metrics"]

    pre_liftoff_ready = (
        elapsed >= PRE_LIFTOFF_TIME
        and load_shift_metrics.support_force >= MIN_SUPPORT_FORCE
        and load_shift_metrics.support_ratio >= 0.50
        and load_shift_metrics.com_speed <= COM_VEL_READY_THRESH
    )
    if pre_liftoff_ready:
        ss_state.com_ref = np.array(state["c"], copy=True)
        ss_state.joint_ref = joint_positions.copy()
        ss_state.ready_since = None
        ss_state.established = False
        return ControlPhase.SINGLE_SUPPORT
    return None
