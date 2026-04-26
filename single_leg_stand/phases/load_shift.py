"""LOAD_SHIFT phase: gradually transfer weight to the support foot."""

from __future__ import annotations

from typing import Optional

from config import (
    LOAD_SHIFT_TIME,
    DOUBLE_SUPPORT_MIN_FORCE,
    LOAD_SHIFT_SUPPORT_RATIO,
    LOAD_SHIFT_SWING_FORCE_MIN,
    COM_VEL_READY_THRESH,
    LOAD_SHIFT_READY_TIME,
)
from phase_core import ControlPhase, StabilityGate
from phase_metrics import LoadShiftMetrics

_load_shift_gate = StabilityGate()


def evaluate_load_shift_readiness(
    phase_start_time: float,
    t: float,
    load_shift_metrics: LoadShiftMetrics,
) -> tuple[bool, dict[str, bool]]:
    """Return overall readiness and the individual gate results for LOAD_SHIFT."""
    elapsed = t - phase_start_time
    support_ratio_gate = max(0.45, LOAD_SHIFT_SUPPORT_RATIO - 0.03)
    checks = {
        "time": elapsed >= LOAD_SHIFT_TIME,
        "support_force": load_shift_metrics.support_force >= DOUBLE_SUPPORT_MIN_FORCE,
        "swing_force": load_shift_metrics.swing_force >= LOAD_SHIFT_SWING_FORCE_MIN,
        "support_ratio": load_shift_metrics.support_ratio >= support_ratio_gate,
        "com_speed": load_shift_metrics.com_speed <= COM_VEL_READY_THRESH,
    }
    return all(checks.values()), checks


def check_load_shift_transition(
    phase_start_time: float,
    t: float,
    state: dict,
) -> Optional[ControlPhase]:
    """Transition to PRE_LIFTOFF when weight transfer metrics are satisfied."""
    load_shift_metrics = state["load_shift_metrics"]
    load_shift_ready, _checks = evaluate_load_shift_readiness(
        phase_start_time,
        t,
        load_shift_metrics,
    )
    if _load_shift_gate.check(load_shift_ready, t, LOAD_SHIFT_READY_TIME):
        return ControlPhase.PRE_LIFTOFF
    return None
