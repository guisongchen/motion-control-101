"""INIT_SETTLE phase: initial gravity compensation and settling."""

from __future__ import annotations

from typing import Optional

from config import INIT_SETTLE_TIME
from phase_core import ControlPhase, PhaseTiming, phase_elapsed, transition_phase


def check_init_settle_transition(
    phase_timing: PhaseTiming,
    t: float,
) -> Optional[str]:
    """Transition from INIT_SETTLE to DOUBLE_SUPPORT_HOLD after settle time."""
    elapsed = phase_elapsed(phase_timing, t)
    if elapsed >= INIT_SETTLE_TIME:
        transition_phase(phase_timing, ControlPhase.DOUBLE_SUPPORT_HOLD, t)
        return "double-support validation"
    return None
