"""SINGLE_SUPPORT phase: CoM reference, establishment checks, and WBC/MPC control."""

from __future__ import annotations

import numpy as np

from config import (
    DT_SIM,
    MIN_SUPPORT_FORCE,
    SINGLE_SUPPORT_ESTABLISH_SUPPORT_RATIO,
    SINGLE_SUPPORT_ESTABLISH_SWING_FORCE_MAX,
    SINGLE_SUPPORT_ESTABLISH_COM_SPEED,
    SLIP_THRESH,
    SINGLE_SUPPORT_COM_RATIO,
    SINGLE_SUPPORT_HOLD_FORCE,
    SINGLE_SUPPORT_MPC_DELAY,
    SINGLE_SUPPORT_FORCE_BLEND_TIME,
    SINGLE_SUPPORT_MIN_FORCE_RATIO,
    SINGLE_SUPPORT_MAX_HORIZONTAL_FORCE,
    SINGLE_SUPPORT_ENTRY_TIME,
    SINGLE_SUPPORT_ESTABLISH_TIME,
    T_S,
    GRAVITY,
)
from phase_core import (
    ControlPhase,
    PhaseState,
    phase_elapsed,
    compute_centroidal_dynamics,
    world_point_to_local_body_point,
)
from phase_metrics import LoadShiftMetrics
from phase_targets import compute_single_support_entry_progress


def build_single_support_com_ref(
    c: np.ndarray,
    initial_foot_pos: dict[int, np.ndarray],
    support_foot_link: int,
    swing_foot_link: int,
) -> np.ndarray:
    """Anchor the single-support CoM target to the stable handoff state."""
    c_ref = c.copy()
    support_xy = initial_foot_pos[support_foot_link][:2]
    swing_xy = initial_foot_pos[swing_foot_link][:2]
    stance_midpoint = 0.5 * (support_xy + swing_xy)
    target_xy = stance_midpoint + SINGLE_SUPPORT_COM_RATIO * (support_xy - stance_midpoint)

    support_axis = support_xy - swing_xy
    support_axis_norm = np.linalg.norm(support_axis)
    if support_axis_norm > 1e-6:
        support_axis /= support_axis_norm
        current_projection = np.dot(c[:2] - stance_midpoint, support_axis)
        target_projection = np.dot(target_xy - stance_midpoint, support_axis)
        if current_projection < target_projection:
            c_ref[:2] += (target_projection - current_projection) * support_axis
        perp_axis = np.array([-support_axis[1], support_axis[0]])
        perp_error = np.dot(c[:2] - stance_midpoint, perp_axis)
        c_ref[:2] -= 0.5 * perp_error * perp_axis
    c_ref[2] = c[2]
    return c_ref


def update_single_support_establishment(
    phase_state: PhaseState,
    t: float,
    load_shift_metrics: LoadShiftMetrics,
) -> None:
    """Only enable optimization after one-leg support looks physically established."""
    if phase_state.phase != ControlPhase.SINGLE_SUPPORT:
        phase_state.single_support_ready_since = None
        phase_state.single_support_established = False
        return
    if phase_state.single_support_established:
        return

    established_ready = (
        load_shift_metrics.support_force >= MIN_SUPPORT_FORCE
        and load_shift_metrics.support_ratio >= SINGLE_SUPPORT_ESTABLISH_SUPPORT_RATIO
        and load_shift_metrics.swing_force <= SINGLE_SUPPORT_ESTABLISH_SWING_FORCE_MAX
        and load_shift_metrics.com_speed <= SINGLE_SUPPORT_ESTABLISH_COM_SPEED
        and load_shift_metrics.support_slip <= SLIP_THRESH
        and load_shift_metrics.swing_slip <= SLIP_THRESH
    )
    if not established_ready:
        phase_state.single_support_ready_since = None
        return
    if phase_state.single_support_ready_since is None:
        phase_state.single_support_ready_since = t
        return
    if t - phase_state.single_support_ready_since >= SINGLE_SUPPORT_ESTABLISH_TIME:
        phase_state.single_support_established = True


def should_model_swing_contact(
    phase_state: PhaseState,
    load_shift_metrics: LoadShiftMetrics,
) -> bool:
    """Keep the swing foot in the WBC model while it still carries meaningful load."""
    return (
        phase_state.phase == ControlPhase.SINGLE_SUPPORT
        and load_shift_metrics.swing_force > SINGLE_SUPPORT_HOLD_FORCE
    )


def run_single_support_control(
    robot,
    phase_state: PhaseState,
    t: float,
    step: int,
    q: np.ndarray,
    v: np.ndarray,
    c: np.ndarray,
    c_dot: np.ndarray,
    L: np.ndarray,
    c_ref: np.ndarray,
    c_dot_ref: np.ndarray,
    c_ddot_ref: np.ndarray,
    L_ref: np.ndarray,
    L_dot_ref: np.ndarray,
    p_foot: np.ndarray,
    support_foot_link: int,
    swing_foot_link: int,
    foot_contacts: list[dict],
    load_shift_metrics: LoadShiftMetrics,
    mpc,
    mpc_period: int,
    wbc_ss,
    wbc_ss_with_swing,
    tau_min_limits: np.ndarray,
    tau_max_limits: np.ndarray,
    support_contact_local_positions: list[np.ndarray],
    filtered_cop_world: np.ndarray,
    prev_filtered_cop_world: np.ndarray,
    filtered_support_position_world: np.ndarray,
    prev_filtered_support_position_world: np.ndarray,
    initial_support_contact: np.ndarray,
    initial_support_yaw: float,
    mpc_force_target: np.ndarray,
    u_ref: np.ndarray,
    mpc_result,
    wbc_result,
    wbc_period: int,
) -> tuple:
    """Execute MPC and WBC for single support; return updated control values."""
    from direct_single_support import (
        apply_measured_cop_feedback,
        build_corner_patch_wrench_task,
        compute_corner_patch_force_reference,
        compute_corner_patch_wrench_force_reference,
        wrap_to_pi,
        yaw_from_rotation,
        DIRECT_SINGLE_SUPPORT_CONFIG as direct_cfg,
    )

    single_support_elapsed = phase_elapsed(phase_state, t)

    if (
        single_support_elapsed >= SINGLE_SUPPORT_MPC_DELAY
        and step % mpc_period == 0
    ):
        A_d, B_d, d_d = compute_centroidal_dynamics(robot.total_mass, c, p_foot, T_S)
        mpc.set_dynamics(A_d, B_d, d_d)
        x0 = np.concatenate([c, c_dot, L])
        mpc_result = mpc.solve(x0)
        if mpc_result is not None:
            mpc_force_target = mpc_result["u0"]
        else:
            print(f"[WARN] t={t:.3f}s MPC 求解失败")

    if single_support_elapsed >= SINGLE_SUPPORT_MPC_DELAY:
        blend_progress = max(
            0.0,
            single_support_elapsed - SINGLE_SUPPORT_MPC_DELAY,
        ) / max(SINGLE_SUPPORT_FORCE_BLEND_TIME, 1e-6)
        mpc_blend = min(1.0, blend_progress)
        f_ref = (1.0 - mpc_blend) * u_ref + mpc_blend * mpc_force_target
    else:
        f_ref = u_ref.copy()
    f_ref[2] = max(f_ref[2], SINGLE_SUPPORT_MIN_FORCE_RATIO * u_ref[2])
    f_ref[:2] = np.clip(
        f_ref[:2],
        -SINGLE_SUPPORT_MAX_HORIZONTAL_FORCE,
        SINGLE_SUPPORT_MAX_HORIZONTAL_FORCE,
    )

    # WBC solve
    if step % wbc_period == 0:
        M = robot.compute_mass_matrix(q)
        C = robot.compute_coriolis_gravity(q, v)
        modeled_swing_contact = should_model_swing_contact(phase_state, load_shift_metrics)
        swing_metrics_pre = (
            robot.get_contact_metrics(swing_foot_link)
            if modeled_swing_contact
            else None
        )
        contact_jacobians = [
            robot.get_foot_jacobian(support_foot_link, q, local_position=lp)
            for lp in support_contact_local_positions
        ]
        residual_swing_force = 0.0
        if (
            swing_metrics_pre is not None
            and swing_metrics_pre["normal_force"] > SINGLE_SUPPORT_HOLD_FORCE
        ):
            swing_contact_local_position = world_point_to_local_body_point(
                robot,
                swing_foot_link,
                swing_metrics_pre["cop_position"],
            )
            contact_jacobians.append(
                robot.get_foot_jacobian(
                    swing_foot_link,
                    q,
                    local_position=swing_contact_local_position,
                )
            )
            residual_swing_force = min(
                float(swing_metrics_pre["normal_force"]),
                max(float(f_ref[2]) - SINGLE_SUPPORT_HOLD_FORCE, 0.0),
            )
        modeled_swing_contact = len(contact_jacobians) == 5
        active_wbc_ss = wbc_ss_with_swing if modeled_swing_contact else wbc_ss
        J_c = np.vstack(contact_jacobians)
        J_com = robot.get_com_jacobian(q)
        J_L = robot.get_angular_momentum_jacobian(q)
        Jc_dot = np.zeros_like(J_c)

        c_ddot_des = wbc_ss.compute_desired_acceleration(
            c_ref, c, c_dot_ref, c_dot, c_ddot_ref
        )
        L_dot_des = wbc_ss.compute_desired_momentum_rate(
            L_ref, L, L_dot_ref, np.zeros(3)
        )

        # Corner-patch CoP / slip / yaw control
        foot_origin = np.array(robot.data.xpos[support_foot_link], copy=True)
        foot_rotation = np.array(robot.data.xmat[support_foot_link]).reshape(3, 3)

        support_metrics_pre = robot.get_contact_metrics(support_foot_link)
        if support_metrics_pre["normal_force"] > 1e-6:
            measured_cop_world = np.array(support_metrics_pre["cop_position"], copy=True)
            filtered_cop_world[:] = (
                direct_cfg.cop.filter_alpha * measured_cop_world
                + (1.0 - direct_cfg.cop.filter_alpha) * filtered_cop_world
            )
            measured_support_position_world = np.array(support_metrics_pre["position"], copy=True)
            filtered_support_position_world[:] = (
                direct_cfg.wrench.state_filter_alpha * measured_support_position_world
                + (1.0 - direct_cfg.wrench.state_filter_alpha) * filtered_support_position_world
            )

        measured_cop_velocity_world = (
            filtered_cop_world - prev_filtered_cop_world
        ) / DT_SIM
        prev_filtered_cop_world[:] = filtered_cop_world
        measured_support_slip_velocity_world = (
            filtered_support_position_world - prev_filtered_support_position_world
        ) / DT_SIM
        prev_filtered_support_position_world[:] = filtered_support_position_world

        nominal_cop_target_world = initial_support_contact
        cop_target_world = nominal_cop_target_world
        if direct_cfg.cop.enabled:
            cop_target_world, _ = apply_measured_cop_feedback(
                support_contact_local_positions,
                foot_origin,
                foot_rotation,
                nominal_cop_target_world,
                filtered_cop_world,
                measured_cop_velocity_world,
            )

        support_slip_world = filtered_support_position_world - initial_support_contact
        desired_force_xy_world = -(
            direct_cfg.wrench.slip_force_kp * support_slip_world[:2]
            + direct_cfg.wrench.slip_force_kd * measured_support_slip_velocity_world[:2]
        )
        desired_force_xy_world = np.clip(
            desired_force_xy_world,
            -direct_cfg.wrench.slip_force_max,
            direct_cfg.wrench.slip_force_max,
        )
        desired_force_xy_world += f_ref[:2]

        _, support_ang_vel = robot.get_link_velocity(support_foot_link)
        support_yaw_error = wrap_to_pi(yaw_from_rotation(foot_rotation) - initial_support_yaw)
        desired_yaw_moment = -(
            direct_cfg.wrench.yaw_moment_kp * support_yaw_error
            + direct_cfg.wrench.yaw_moment_kd * support_ang_vel[2]
        )
        desired_yaw_moment = float(np.clip(
            desired_yaw_moment,
            -direct_cfg.wrench.yaw_moment_max,
            direct_cfg.wrench.yaw_moment_max,
        ))

        total_normal_force = max(f_ref[2] - residual_swing_force, SINGLE_SUPPORT_HOLD_FORCE)
        force_task_matrix, force_task_ref, force_task_weight = build_corner_patch_wrench_task(
            support_contact_local_positions,
            foot_origin,
            foot_rotation,
            cop_target_world,
            total_normal_force,
            desired_force_xy_world=desired_force_xy_world,
            desired_yaw_moment=desired_yaw_moment,
        )
        f_ref_12d = compute_corner_patch_wrench_force_reference(
            force_task_matrix, force_task_ref
        )
        if modeled_swing_contact:
            swing_force_ref = np.array([0.0, 0.0, residual_swing_force], dtype=float)
            f_ref_ss = np.concatenate([f_ref_12d, swing_force_ref])
            force_task_matrix = np.hstack(
                [force_task_matrix, np.zeros((force_task_matrix.shape[0], 3))]
            )
        else:
            f_ref_ss = f_ref_12d

        wbc_result = active_wbc_ss.solve(
            M, C, J_c, np.zeros_like(J_c),
            J_com, J_L,
            c_ddot_des, L_dot_des,
            f_ref_ss, v,
            tau_min_limits, tau_max_limits,
            force_task_matrix=force_task_matrix,
            force_task_ref=force_task_ref,
            force_task_weight=force_task_weight,
        )
        if wbc_result is None:
            print(f"[WARN] t={t:.3f}s WBC 求解失败")
    else:
        active_wbc_ss = wbc_ss
        J_c = None

    return f_ref, mpc_force_target, wbc_result, active_wbc_ss, J_c, mpc_result
