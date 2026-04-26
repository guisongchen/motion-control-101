"""SINGLE_SUPPORT phase: CoM reference, establishment checks, and WBC/MPC control."""

from __future__ import annotations

import mujoco
import numpy as np

from config import (
    DT_SIM,
    MIN_SUPPORT_FORCE,
    SINGLE_SUPPORT_ESTABLISH_SUPPORT_RATIO,
    SINGLE_SUPPORT_ESTABLISH_SWING_FORCE_MAX,
    SINGLE_SUPPORT_ESTABLISH_COM_SPEED,
    SLIP_THRESH,
    SINGLE_SUPPORT_HOLD_FORCE,
    SINGLE_SUPPORT_MPC_DELAY,
    SINGLE_SUPPORT_FORCE_BLEND_TIME,
    SINGLE_SUPPORT_MIN_FORCE_RATIO,
    SINGLE_SUPPORT_MAX_HORIZONTAL_FORCE,
    SINGLE_SUPPORT_ESTABLISH_TIME,
    T_S,
)
from phase_core import (
    ControlPhase,
    SingleSupportState,
    compute_centroidal_dynamics,
    world_point_to_local_body_point,
    CentroidalState,
    TaskReference,
    SupportContext,
    SolverConfig,
    ControlMemory,
)
from phase_metrics import LoadShiftMetrics
from robot_model import RobotModel


def build_single_support_com_ref(
    robot: RobotModel,
    support_foot_link: int,
) -> np.ndarray:
    """Anchor the single-support CoM target to the contact-patch centroid."""
    body_origin = np.array(robot.data.xpos[support_foot_link], copy=True)
    body_rotation = np.array(robot.data.xmat[support_foot_link]).reshape(3, 3)

    corners: list[np.ndarray] = []
    has_box = False
    box_local_pos = None
    for geom_id in range(robot.model.ngeom):
        if int(robot.model.geom_bodyid[geom_id]) != support_foot_link:
            continue
        geom_type = int(robot.model.geom_type[geom_id])
        if geom_type == mujoco.mjtGeom.mjGEOM_SPHERE:
            local_pos = np.array(robot.model.geom_pos[geom_id], copy=True)
            world_pos = body_origin + body_rotation @ local_pos
            corners.append(world_pos)
        elif geom_type == mujoco.mjtGeom.mjGEOM_BOX:
            has_box = True
            box_local_pos = np.array(robot.model.geom_pos[geom_id], copy=True)

    c_ref = np.zeros(3)
    if has_box and box_local_pos is not None:
        centroid_world = body_origin + body_rotation @ box_local_pos
        c_ref[:2] = centroid_world[:2]
    elif corners:
        centroid = np.mean(corners, axis=0)
        c_ref[:2] = centroid[:2]
    else:
        c_ref[:2] = body_origin[:2]
    return c_ref


def update_single_support_establishment(
    ss_state: SingleSupportState,
    phase: ControlPhase,
    t: float,
    load_shift_metrics: LoadShiftMetrics,
) -> None:
    """Only enable optimization after one-leg support looks physically established."""
    if phase != ControlPhase.SINGLE_SUPPORT:
        ss_state.ready_since = None
        ss_state.established = False
        return
    if ss_state.established:
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
        ss_state.ready_since = None
        return
    if ss_state.ready_since is None:
        ss_state.ready_since = t
        return
    if t - ss_state.ready_since >= SINGLE_SUPPORT_ESTABLISH_TIME:
        ss_state.established = True


def should_model_swing_contact(
    phase: ControlPhase,
    load_shift_metrics: LoadShiftMetrics,
) -> bool:
    """Keep the swing foot in the WBC model while it still carries meaningful load."""
    return (
        phase == ControlPhase.SINGLE_SUPPORT
        and load_shift_metrics.swing_force > SINGLE_SUPPORT_HOLD_FORCE
    )


def run_single_support_control(
    robot,
    phase: ControlPhase,
    phase_start_time: float,
    ss_state: SingleSupportState,
    t: float,
    step: int,
    state: CentroidalState,
    refs: TaskReference,
    support: SupportContext,
    load_shift_metrics: LoadShiftMetrics,
    solvers: SolverConfig,
    tau_limits: tuple[np.ndarray, np.ndarray],
    u_ref: np.ndarray,
    memory: ControlMemory,
) -> tuple:
    """Execute MPC and WBC for single support; return updated control values."""
    from direct_single_support import (
        apply_measured_cop_feedback,
        build_corner_patch_wrench_task,
        compute_corner_patch_wrench_force_reference,
        wrap_to_pi,
        yaw_from_rotation,
        DIRECT_SINGLE_SUPPORT_CONFIG as direct_cfg,
    )

    single_support_elapsed = t - phase_start_time

    # Unpack grouped arguments for local convenience
    c = state.c
    c_dot = state.c_dot
    L = state.L
    mpc_force_target = memory.mpc_force_target
    mpc_result = memory.mpc_result
    wbc_result = memory.wbc_result
    tau_min_limits, tau_max_limits = tau_limits

    if (
        single_support_elapsed >= SINGLE_SUPPORT_MPC_DELAY
        and step % solvers.mpc_period == 0
    ):
        A_d, B_d, d_d = compute_centroidal_dynamics(robot.total_mass, c, support.p_foot, T_S)
        solvers.mpc.set_dynamics(A_d, B_d, d_d)
        x0 = np.concatenate([c, c_dot, L])
        mpc_result = solvers.mpc.solve(x0)
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
    if step % solvers.wbc_period == 0:
        M = robot.compute_mass_matrix(state.q)
        C = robot.compute_coriolis_gravity(state.q, state.v)
        modeled_swing_contact = should_model_swing_contact(phase, load_shift_metrics)
        swing_metrics_pre = (
            robot.get_contact_metrics(support.swing_foot_link)
            if modeled_swing_contact
            else None
        )
        contact_jacobians = [
            robot.get_foot_jacobian(support.support_foot_link, state.q, local_position=lp)
            for lp in support.contact_local_positions
        ]
        residual_swing_force = 0.0
        if (
            swing_metrics_pre is not None
            and swing_metrics_pre["normal_force"] > SINGLE_SUPPORT_HOLD_FORCE
        ):
            swing_contact_local_position = world_point_to_local_body_point(
                robot,
                support.swing_foot_link,
                swing_metrics_pre["cop_position"],
            )
            contact_jacobians.append(
                robot.get_foot_jacobian(
                    support.swing_foot_link,
                    state.q,
                    local_position=swing_contact_local_position,
                )
            )
            residual_swing_force = min(
                float(swing_metrics_pre["normal_force"]),
                max(float(f_ref[2]) - SINGLE_SUPPORT_HOLD_FORCE, 0.0),
            )
        modeled_swing_contact = len(contact_jacobians) == 5
        active_wbc_ss = solvers.wbc_ss_with_swing if modeled_swing_contact else solvers.wbc_ss
        J_c = np.vstack(contact_jacobians)
        J_com = robot.get_com_jacobian(state.q)
        J_L = robot.get_angular_momentum_jacobian(state.q)
        Jc_dot = np.zeros_like(J_c)

        c_ddot_des = solvers.wbc_ss.compute_desired_acceleration(
            refs.c, c, refs.c_dot, c_dot, refs.c_ddot
        )
        L_dot_des = solvers.wbc_ss.compute_desired_momentum_rate(
            refs.L, L, refs.L_dot, np.zeros(3)
        )

        # Corner-patch CoP / slip / yaw control
        foot_origin = np.array(robot.data.xpos[support.support_foot_link], copy=True)
        foot_rotation = np.array(robot.data.xmat[support.support_foot_link]).reshape(3, 3)

        support_metrics_pre = robot.get_contact_metrics(support.support_foot_link)
        if support_metrics_pre["normal_force"] > 1e-6:
            measured_cop_world = np.array(support_metrics_pre["cop_position"], copy=True)
            ss_state.filtered_cop_world[:] = (
                direct_cfg.cop.filter_alpha * measured_cop_world
                + (1.0 - direct_cfg.cop.filter_alpha) * ss_state.filtered_cop_world
            )
            measured_support_position_world = np.array(support_metrics_pre["position"], copy=True)
            ss_state.filtered_support_position_world[:] = (
                direct_cfg.wrench.state_filter_alpha * measured_support_position_world
                + (1.0 - direct_cfg.wrench.state_filter_alpha) * ss_state.filtered_support_position_world
            )

        dt_wbc = solvers.wbc_period * DT_SIM
        measured_cop_velocity_world = (
            ss_state.filtered_cop_world - ss_state.prev_filtered_cop_world
        ) / dt_wbc
        ss_state.prev_filtered_cop_world[:] = ss_state.filtered_cop_world
        measured_support_slip_velocity_world = (
            ss_state.filtered_support_position_world - ss_state.prev_filtered_support_position_world
        ) / dt_wbc
        ss_state.prev_filtered_support_position_world[:] = ss_state.filtered_support_position_world

        nominal_cop_target_world = support.initial_contact
        cop_target_world = nominal_cop_target_world
        if direct_cfg.cop.enabled:
            cop_target_world, _ = apply_measured_cop_feedback(
                support.contact_local_positions,
                foot_origin,
                foot_rotation,
                nominal_cop_target_world,
                ss_state.filtered_cop_world,
                measured_cop_velocity_world,
            )

        support_slip_world = ss_state.filtered_support_position_world - support.initial_contact
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

        _, support_ang_vel = robot.get_link_velocity(support.support_foot_link)
        support_yaw_error = wrap_to_pi(yaw_from_rotation(foot_rotation) - support.initial_yaw)
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
            support.contact_local_positions,
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
            f_ref_ss, state.v,
            tau_min_limits, tau_max_limits,
            force_task_matrix=force_task_matrix,
            force_task_ref=force_task_ref,
            force_task_weight=force_task_weight,
        )
        if wbc_result is None:
            print(f"[WARN] t={t:.3f}s WBC 求解失败")
    else:
        active_wbc_ss = solvers.wbc_ss
        J_c = None

    return f_ref, mpc_force_target, wbc_result, active_wbc_ss, J_c, mpc_result
