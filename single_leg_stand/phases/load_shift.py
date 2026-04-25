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
from phase_core import ControlPhase, PhaseTiming, phase_elapsed, transition_phase
from phase_metrics import LoadShiftMetrics


def check_load_shift_transition(
    phase_timing: PhaseTiming,
    t: float,
    load_shift_metrics: LoadShiftMetrics,
) -> Optional[str]:
    """Transition to PRE_LIFTOFF when weight transfer metrics are satisfied."""
    elapsed = phase_elapsed(phase_timing, t)
    load_shift_ready = (
        elapsed >= LOAD_SHIFT_TIME
        and load_shift_metrics.support_force >= DOUBLE_SUPPORT_MIN_FORCE
        and load_shift_metrics.support_ratio >= LOAD_SHIFT_SUPPORT_RATIO
        and load_shift_metrics.com_shift_ratio >= LOAD_SHIFT_COM_RATIO
        and load_shift_metrics.com_speed <= COM_VEL_READY_THRESH
        and load_shift_metrics.support_slip <= SLIP_THRESH
        and load_shift_metrics.swing_slip <= SLIP_THRESH
    )
    if load_shift_ready:
        if phase_timing.ready_since is None:
            phase_timing.ready_since = t
        if t - phase_timing.ready_since >= LOAD_SHIFT_READY_TIME:
            transition_phase(phase_timing, ControlPhase.PRE_LIFTOFF, t)
            return (
                "pre-liftoff "
                f"(support_ratio={load_shift_metrics.support_ratio:.2f}, "
                f"com_shift={load_shift_metrics.com_shift_ratio:.2f})"
            )
    else:
        phase_timing.ready_since = None
    return None
