"""PRE_LIFTOFF phase: unload swing foot and prepare for single support."""

from __future__ import annotations

from typing import Optional

from config import (
    PRE_LIFTOFF_TIME,
    MIN_SUPPORT_FORCE,
    PRE_LIFTOFF_SUPPORT_RATIO,
    PRE_LIFTOFF_SWING_FORCE_MAX,
    PRE_LIFTOFF_COM_RATIO,
    COM_VEL_READY_THRESH,
    PRE_LIFTOFF_FORWARD_ERROR_THRESH,
    PRE_LIFTOFF_FORWARD_VEL_THRESH,
    SLIP_THRESH,
    PRE_LIFTOFF_READY_TIME,
)
from phase_core import (
    ControlPhase,
    StabilityGate,
    SingleSupportState,
    compute_forward_handoff_metrics,
)

_pre_liftoff_gate = StabilityGate()


def check_pre_liftoff_transition(
    phase_start_time: float,
    ss_state: SingleSupportState,
    t: float,
    state: dict,
    c_ref: np.ndarray,
    swing_foot_link: int,
    joint_positions: np.ndarray,
    initial_foot_pos: dict[int, np.ndarray],
    locked_support_foot_link: int,
) -> Optional[ControlPhase]:
    """Transition to SINGLE_SUPPORT when pre-liftoff readiness is achieved."""
    c = state["c"]
    c_dot = state["c_dot"]
    load_shift_metrics = state["load_shift_metrics"]
    elapsed = t - phase_start_time
    forward_error, forward_velocity = compute_forward_handoff_metrics(c, c_dot, c_ref)

    pre_liftoff_ready = (
        elapsed >= PRE_LIFTOFF_TIME
        and load_shift_metrics.support_force >= MIN_SUPPORT_FORCE
        and load_shift_metrics.support_ratio >= PRE_LIFTOFF_SUPPORT_RATIO
        and load_shift_metrics.swing_force <= PRE_LIFTOFF_SWING_FORCE_MAX
        and load_shift_metrics.com_shift_ratio >= PRE_LIFTOFF_COM_RATIO
        and load_shift_metrics.com_speed <= COM_VEL_READY_THRESH
        and abs(forward_error) <= PRE_LIFTOFF_FORWARD_ERROR_THRESH
        and abs(forward_velocity) <= PRE_LIFTOFF_FORWARD_VEL_THRESH
        and load_shift_metrics.support_slip <= SLIP_THRESH
        and load_shift_metrics.swing_slip <= SLIP_THRESH
    )
    if _pre_liftoff_gate.check(pre_liftoff_ready, t, PRE_LIFTOFF_READY_TIME):
        from phases.single_support import build_single_support_com_ref
        ss_state.com_ref = build_single_support_com_ref(
            c,
            initial_foot_pos,
            locked_support_foot_link,
            swing_foot_link,
        )
        ss_state.joint_ref = joint_positions.copy()
        ss_state.ready_since = None
        ss_state.established = False
        return ControlPhase.SINGLE_SUPPORT
    return None
