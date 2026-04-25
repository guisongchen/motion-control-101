"""INIT_SETTLE phase: initial gravity compensation and settling."""

from __future__ import annotations

from typing import Optional

from config import INIT_SETTLE_TIME
from phase_core import ControlPhase


def check_init_settle_transition(
    phase: ControlPhase,
    phase_start_time: float,
    t: float,
) -> Optional[tuple[ControlPhase, str]]:
    """Transition from INIT_SETTLE to DOUBLE_SUPPORT_HOLD after settle time."""
    if t - phase_start_time >= INIT_SETTLE_TIME:
        return ControlPhase.DOUBLE_SUPPORT_HOLD, "double-support validation"
    return None
