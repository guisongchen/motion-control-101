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
    PhaseState,
    phase_elapsed,
    transition_phase,
    compute_forward_handoff_metrics,
)
from phase_metrics import LoadShiftMetrics


def check_pre_liftoff_transition(
    phase_state: PhaseState,
    t: float,
    load_shift_metrics: LoadShiftMetrics,
    c: np.ndarray,
    c_dot: np.ndarray,
    c_ref: np.ndarray,
    swing_foot_link: int,
    candidate_foot_links: list[int],
    joint_positions: np.ndarray,
    initial_foot_pos: dict[int, np.ndarray],
) -> Optional[str]:
    """Transition to SINGLE_SUPPORT when pre-liftoff readiness is achieved."""
    elapsed = phase_elapsed(phase_state, t)
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
    if pre_liftoff_ready:
        if phase_state.ready_since is None:
            phase_state.ready_since = t
        if t - phase_state.ready_since >= PRE_LIFTOFF_READY_TIME:
            from phases.single_support import build_single_support_com_ref
            phase_state.single_support_com_ref = build_single_support_com_ref(
                c,
                initial_foot_pos,
                phase_state.locked_support_foot_link,
                swing_foot_link,
            )
            phase_state.single_support_joint_ref = joint_positions.copy()
            phase_state.single_support_ready_since = None
            phase_state.single_support_established = False
            transition_phase(phase_state, ControlPhase.SINGLE_SUPPORT, t)
            from phase_core import support_name
            return (
                "single-support control "
                f"(support={support_name(candidate_foot_links, phase_state.locked_support_foot_link)}, "
                f"support_ratio={load_shift_metrics.support_ratio:.2f}, "
                f"swing_force={load_shift_metrics.swing_force:.1f}N, "
                f"forward_error={forward_error:.3f}m, "
                f"forward_vel={forward_velocity:.3f}m/s)"
            )
    else:
        phase_state.ready_since = None
    return None
