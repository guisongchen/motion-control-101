"""LOAD_SHIFT phase: gradually transfer weight to the support foot."""

from __future__ import annotations

from typing import Optional

from config import (
    LOAD_SHIFT_TIME,
    DOUBLE_SUPPORT_MIN_FORCE,
    LOAD_SHIFT_SUPPORT_RATIO,
    LOAD_SHIFT_COM_RATIO,
    COM_VEL_READY_THRESH,
    SLIP_THRESH,
    LOAD_SHIFT_READY_TIME,
)
from phase_core import ControlPhase, StabilityGate

_load_shift_gate = StabilityGate()


def check_load_shift_transition(
    phase_start_time: float,
    t: float,
    state: dict,
) -> Optional[ControlPhase]:
    """Transition to PRE_LIFTOFF when weight transfer metrics are satisfied."""
    load_shift_metrics = state["load_shift_metrics"]
    elapsed = t - phase_start_time
    load_shift_ready = (
        elapsed >= LOAD_SHIFT_TIME
        and load_shift_metrics.support_force >= DOUBLE_SUPPORT_MIN_FORCE
        and load_shift_metrics.support_ratio >= LOAD_SHIFT_SUPPORT_RATIO
        and load_shift_metrics.com_shift_ratio >= LOAD_SHIFT_COM_RATIO
        and load_shift_metrics.com_speed <= COM_VEL_READY_THRESH
        and load_shift_metrics.support_slip <= SLIP_THRESH
        and load_shift_metrics.swing_slip <= SLIP_THRESH
    )
    if _load_shift_gate.check(load_shift_ready, t, LOAD_SHIFT_READY_TIME):
        return ControlPhase.PRE_LIFTOFF
    return None
