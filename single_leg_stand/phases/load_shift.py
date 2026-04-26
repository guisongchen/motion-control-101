"""LOAD_SHIFT phase: confirm support-foot takeover before pre-liftoff."""

from __future__ import annotations

from config import (
    LOAD_SHIFT_TIME,
    DOUBLE_SUPPORT_MIN_FORCE,
    LOAD_SHIFT_SUPPORT_RATIO_MIN,
    LOAD_SHIFT_SWING_FORCE_MIN,
    COM_VEL_READY_THRESH,
)
from phase_metrics import LoadShiftMetrics


def evaluate_load_shift_readiness(
    elapsed: float,
    load_shift_metrics: LoadShiftMetrics,
) -> tuple[bool, dict[str, bool]]:
    """Return overall readiness and individual takeover checks for LOAD_SHIFT."""
    checks = {
        "time": elapsed >= LOAD_SHIFT_TIME,
        "support_force": load_shift_metrics.support_force >= DOUBLE_SUPPORT_MIN_FORCE,
        "swing_force": load_shift_metrics.swing_force >= LOAD_SHIFT_SWING_FORCE_MIN,
        "support_ratio": load_shift_metrics.support_ratio >= LOAD_SHIFT_SUPPORT_RATIO_MIN,
        "com_speed": load_shift_metrics.com_speed <= COM_VEL_READY_THRESH,
    }
    return all(checks.values()), checks
