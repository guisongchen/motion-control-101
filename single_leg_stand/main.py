"""Main loop: MPC + WBC + MuJoCo with explicit control phases."""

from __future__ import annotations

import numpy as np

from config import (
    DT_SIM,
    WBC_FREQ,
    MPC_FREQ,
    SIM_DURATION,
    H_COM,
    NX,
    NU,
    GRAVITY,
    RMSE_THRESH,
    SLIP_THRESH,
    MPC_TIME_THRESH,
    WBC_TIME_THRESH,
    BASE_DOF_DAMPING,
    JOINT_DOF_DAMPING,
    MIN_SUPPORT_FORCE,
    SINGLE_SUPPORT_HOLD_FORCE,
    TRANSITION_BLEND_TIME,
    SINGLE_SUPPORT_ENTRY_TIME,
    SINGLE_SUPPORT_MAX_TAU_BLEND,
    SUPPORT_POINT_FILTER,
    INIT_SETTLE_TIME,
    LOAD_SHIFT_TIME,
    PRE_LIFTOFF_TIME,
    LOAD_SHIFT_SUPPORT_RATIO,
    SINGLE_SUPPORT_SUPPORT_RATIO,
    DS_TOTAL_FORCE_XY_WEIGHT,
    DS_TOTAL_FORCE_Z_WEIGHT,
    DS_LOAD_DISTRIBUTION_WEIGHT,
)
from robot_model import RobotModel
from robots.unitree_g1 import g1_config
from state_estimator import StateEstimator
from mpc import CentroidalMPC
from wbc import WholeBodyController
from trajectory import compute_transition_com_trajectory, compute_foot_centroid_xy, smoothstep
from utils import (
    compute_rmse,
    plot_com_tracking,
    plot_contact_force,
    plot_torques,
)

from phase_core import (
    ControlPhase,
    SingleSupportState,
    CentroidalState,
    TaskReference,
    SupportContext,
    SolverConfig,
    ControlMemory,
    get_contact_entry,
    support_name,
    compute_phase_com_target,
)
from phase_targets import (
    build_safe_targets,
    compute_safe_tau,
    compute_single_support_entry_progress,
    compute_swing_unload_factor,
)
from phases.double_support import check_double_support_transition
from phases.load_shift import evaluate_load_shift_readiness
from phases.pre_liftoff import check_pre_liftoff_transition
from phases.single_support import (
    update_single_support_establishment,
    run_single_support_control,
)

# Re-use proven single-support control primitives from direct_single_support benchmark
from direct_single_support import (
    build_direct_pose,
    resolve_support_contact_local_positions,
    yaw_from_rotation,
    DIRECT_SINGLE_SUPPORT_CONFIG as direct_cfg,
)

import wbc as wbc_module


def phase_locks_support(phase: ControlPhase) -> bool:
    """Return whether the current phase should keep the support foot locked."""
    return phase in (
        ControlPhase.LOAD_SHIFT,
        ControlPhase.PRE_LIFTOFF,
        ControlPhase.SINGLE_SUPPORT,
    )


def uses_transfer_com_trajectory(
    phase: ControlPhase,
    ss_state: SingleSupportState,
) -> bool:
    """Return whether the DS->SS CoM handoff trajectory should remain active."""
    return phase in (ControlPhase.LOAD_SHIFT, ControlPhase.PRE_LIFTOFF) or (
        phase == ControlPhase.SINGLE_SUPPORT and not ss_state.established
    )


def determine_next_phase(
    phase: ControlPhase,
    elapsed: float,
    t: float,
    state: dict,
    load_shift_metrics,
    ss_state: SingleSupportState,
    c_ref: np.ndarray,
    swing_foot_link: int,
    joint_positions: np.ndarray,
    candidate_foot_links: list[int],
    initial_foot_pos: dict[int, np.ndarray],
    locked_support_foot_link: int,
    robot: RobotModel,
) -> ControlPhase | None:
    """Return the next phase or None to hold the current phase."""
    if phase == ControlPhase.INIT_SETTLE:
        if elapsed >= INIT_SETTLE_TIME:
            return ControlPhase.DOUBLE_SUPPORT_HOLD
        return None

    if phase == ControlPhase.DOUBLE_SUPPORT_HOLD:
        return check_double_support_transition(
            t,
            state,
            candidate_foot_links,
            initial_foot_pos,
            robot,
        )

    if phase == ControlPhase.LOAD_SHIFT:
        load_shift_ready, _load_shift_checks = evaluate_load_shift_readiness(
            elapsed,
            load_shift_metrics,
        )
        if load_shift_ready:
            return ControlPhase.PRE_LIFTOFF
        return None

    if phase == ControlPhase.PRE_LIFTOFF:
        return check_pre_liftoff_transition(
            elapsed,
            ss_state,
            state,
            c_ref,
            swing_foot_link,
            joint_positions,
            initial_foot_pos,
            locked_support_foot_link,
            robot,
        )

    return None


def handle_phase_entry(
    phase: ControlPhase,
    t: float,
    c: np.ndarray,
    foot_contacts: list[dict],
    nominal_c_ref: np.ndarray,
    robot: RobotModel,
    locked_support_foot_link: int,
    support_foot_link: int,
    ss_state: SingleSupportState,
    direct_pose: np.ndarray,
    standing_com_z: float | None,
    initial_support_contact: np.ndarray,
    support_contact_local_positions: list[np.ndarray],
    initial_support_yaw: float,
) -> tuple[
    float | None,
    float | None,
    np.ndarray | None,
    np.ndarray | None,
    np.ndarray,
    list[np.ndarray],
    float,
]:
    """Apply phase-entry side effects and return updated phase state."""
    ds_to_ss_transition_start_time = None
    ds_to_ss_start_xy = None
    ds_to_ss_end_xy = None

    print(f"[INFO] t={t:.3f}s 进入阶段: {phase.name}")

    if phase == ControlPhase.DOUBLE_SUPPORT_HOLD:
        standing_com_z = float(c[2])
    elif phase == ControlPhase.LOAD_SHIFT:
        ds_to_ss_transition_start_time = t
        foot_positions = [fc["position"] for fc in foot_contacts]
        if foot_positions:
            ds_to_ss_start_xy = np.mean([p[:2] for p in foot_positions], axis=0)
        else:
            ds_to_ss_start_xy = nominal_c_ref[:2].copy()
        ds_to_ss_end_xy = compute_foot_centroid_xy(robot, locked_support_foot_link)
    elif phase == ControlPhase.SINGLE_SUPPORT:
        wbc_module.Kp_c = direct_cfg.control.com_kp
        wbc_module.Kd_c = direct_cfg.control.com_kd
        wbc_module.Kp_L = direct_cfg.control.momentum_kp
        wbc_module.Kd_L = direct_cfg.control.momentum_kd

        support_metrics_now = robot.get_contact_metrics(support_foot_link)
        initial_support_contact = support_metrics_now["position"].copy()
        ss_state.filtered_cop_world = np.array(
            support_metrics_now["cop_position"],
            copy=True,
        )
        ss_state.prev_filtered_cop_world = ss_state.filtered_cop_world.copy()
        ss_state.filtered_support_position_world = np.array(
            support_metrics_now["position"],
            copy=True,
        )
        ss_state.prev_filtered_support_position_world = (
            ss_state.filtered_support_position_world.copy()
        )
        initial_support_yaw = yaw_from_rotation(
            np.array(robot.data.xmat[support_foot_link]).reshape(3, 3)
        )

        if standing_com_z is not None and ss_state.com_ref is not None:
            ss_state.com_ref[2] = standing_com_z

        if ss_state.joint_ref is None:
            ss_state.joint_ref = direct_pose.copy()
        ss_state.ready_since = None
        ss_state.established = False

        support_contact_local_positions = resolve_support_contact_local_positions(
            robot,
            support_foot_link,
            initial_support_contact,
        )

    return (
        standing_com_z,
        ds_to_ss_transition_start_time,
        ds_to_ss_start_xy,
        ds_to_ss_end_xy,
        initial_support_contact,
        support_contact_local_positions,
        initial_support_yaw,
    )


def main() -> None:
    robot = RobotModel(g1_config)
    robot.model.opt.gravity[:] = GRAVITY
    robot.model.opt.timestep = DT_SIM
    robot.model.dof_damping[:6] = BASE_DOF_DAMPING
    robot.model.dof_damping[6:] = JOINT_DOF_DAMPING
    robot.reset_base_pose(g1_config.base_initial_pos, g1_config.base_initial_orn)

    candidate_foot_links = robot.foot_link_ids
    preferred_support_foot_link = robot.foot_name_to_link[g1_config.support_foot_name]
    swing_foot_link = robot.foot_name_to_link[g1_config.swing_foot_name]
    swing_leg = g1_config.lift_leg
    support_leg = "right" if swing_leg == "left" else "left"
    estimator = StateEstimator(robot)

    initial_dof_angles = np.zeros(len(robot.dof_joints))
    joint_name_to_dof_idx = robot.dof_joint_name_to_index.copy()
    for idx, joint_name in enumerate(robot.dof_joint_names):
        if joint_name in g1_config.standing_joint_angles:
            initial_dof_angles[idx] = g1_config.standing_joint_angles[joint_name]
    robot.reset_joint_positions(initial_dof_angles)

    tau_max_limits = robot.tau_limits.copy()
    tau_min_limits = -tau_max_limits
    swing_leg_dof_indices = [
        joint_name_to_dof_idx[name]
        for name in g1_config.leg_joint_names[swing_leg]
        if name in joint_name_to_dof_idx
    ]

    mpc = CentroidalMPC()
    wbc_ss = WholeBodyController(robot.nv, num_contacts=4, contact_dim=3)
    wbc_ss_with_swing = WholeBodyController(robot.nv, num_contacts=5, contact_dim=3)
    # Pre-parse corner-patch contact points for both feet (double-support WBC)
    ds_support_contact_locals = resolve_support_contact_local_positions(
        robot, preferred_support_foot_link, np.zeros(3)
    )
    ds_swing_contact_locals = resolve_support_contact_local_positions(
        robot, swing_foot_link, np.zeros(3)
    )
    # Use a single centroid contact per foot for double-support WBC.
    # Four corner constraints lock foot orientation and prevent ankle roll,
    # which makes CoM weight transfer impossible. A centroid contact allows
    # roll/pitch while still constraining translation at the correct ground point.
    ds_support_centroid_local = np.mean(ds_support_contact_locals, axis=0)
    ds_swing_centroid_local = np.mean(ds_swing_contact_locals, axis=0)
    wbc_ds = WholeBodyController(robot.nv, num_contacts=2, contact_dim=4)

    # Pre-parse corner-patch contact points for the preferred support foot
    initial_support_metrics = robot.get_contact_metrics(preferred_support_foot_link)
    initial_support_contact = initial_support_metrics["position"].copy()
    support_contact_local_positions = resolve_support_contact_local_positions(
        robot, preferred_support_foot_link, initial_support_contact
    )

    # Override default WBC gains to proven direct-single-support values
    # (the config defaults of Kp_c=100 are too aggressive and cause instability)
    wbc_module.Kp_c = direct_cfg.control.com_kp
    wbc_module.Kd_c = direct_cfg.control.com_kd
    wbc_module.Kp_L = direct_cfg.control.momentum_kp
    wbc_module.Kd_L = direct_cfg.control.momentum_kd

    # Proven single-support pose from the direct benchmark (used as reference
    # during PRE_LIFTOFF blending and SINGLE_SUPPORT initial posture)
    direct_pose = build_direct_pose(robot.dof_joint_names, support_leg)

    # Single-support geometry (activated in SINGLE_SUPPORT)
    initial_support_yaw = yaw_from_rotation(
        np.array(robot.data.xmat[preferred_support_foot_link]).reshape(3, 3)
    )

    x_ref = np.zeros(NX)
    x_ref[2] = H_COM
    u_ref = np.zeros(NU)
    u_ref[2] = -GRAVITY[2] * robot.total_mass
    mpc.set_reference(x_ref, u_ref)

    nominal_c_ref = x_ref[:3].copy()
    c_ref = nominal_c_ref.copy()
    c_dot_ref = x_ref[3:6]
    L_ref = x_ref[6:9]
    c_ddot_ref = np.zeros(3)
    L_dot_ref = np.zeros(3)

    physics_freq = 1.0 / DT_SIM
    if WBC_FREQ > physics_freq:
        raise ValueError(
            f"WBC_FREQ ({WBC_FREQ} Hz) exceeds physics rate ({physics_freq:.0f} Hz). "
            f"Lower WBC_FREQ or reduce DT_SIM."
        )
    if MPC_FREQ > physics_freq:
        raise ValueError(
            f"MPC_FREQ ({MPC_FREQ} Hz) exceeds physics rate ({physics_freq:.0f} Hz). "
            f"Lower MPC_FREQ or reduce DT_SIM."
        )
    wbc_period = max(1, round(physics_freq / WBC_FREQ))
    mpc_period = max(1, round(physics_freq / MPC_FREQ))
    total_steps = int(SIM_DURATION / DT_SIM)

    time_log = []
    com_log = []
    com_ref_log = []
    foot_pos_log = {link: [] for link in candidate_foot_links}
    foot_force_log = {link: [] for link in candidate_foot_links}
    support_foot_log = []
    mpc_time_log = []
    wbc_time_log = []
    tau_log = []
    mpc_f_ref_log = []
    wbc_f_log = []

    initial_foot_pos = {
        link: np.array(robot.get_contact_metrics(link)["position"], copy=True)
        for link in candidate_foot_links
    }
    locked_support_foot_link = preferred_support_foot_link
    filtered_support_point = initial_foot_pos[preferred_support_foot_link].copy()
    phase = ControlPhase.INIT_SETTLE
    phase_start_time = 0.0
    ss_state = SingleSupportState(
        filtered_cop_world=np.array(initial_support_metrics["cop_position"], copy=True),
        prev_filtered_cop_world=np.array(initial_support_metrics["cop_position"], copy=True),
        filtered_support_position_world=np.array(initial_support_metrics["position"], copy=True),
        prev_filtered_support_position_world=np.array(initial_support_metrics["position"], copy=True),
    )
    last_wbc_warn_time = -1e9
    last_valid_support_tau = None

    mpc_result = None
    wbc_result = None
    f_ref = u_ref.copy()
    mpc_force_target = u_ref.copy()

    # Double-support to single-support trajectory state
    ds_to_ss_start_xy: np.ndarray | None = None
    ds_to_ss_end_xy: np.ndarray | None = None
    ds_to_ss_transition_start_time: float | None = None
    ds_to_ss_duration = LOAD_SHIFT_TIME + PRE_LIFTOFF_TIME
    standing_com_z: float | None = None

    print("\n===== 开始仿真（MuJoCo 单足站立测试模式）=====")
    print(f"总质量: {robot.total_mass:.2f} kg")
    print(f"仿真时长: {SIM_DURATION:.1f} s")
    print(f"MPC 周期: {mpc_period} 步 ({mpc_period * DT_SIM * 1000:.1f} ms)")
    print(f"WBC 周期: {wbc_period} 步 ({wbc_period * DT_SIM * 1000:.3f} ms)")
    print("=" * 50)

    # Base DOF offsets (free joint: 7 qpos, 6 vel for standard humanoids)
    nq_base = robot.model.jnt_qposadr[1] if robot.model.njnt > 1 else robot.model.nq
    nv_base = robot.model.jnt_dofadr[1] if robot.model.njnt > 1 else robot.model.nv

    for step in range(total_steps):
        t = step * DT_SIM
        lock_support = phase_locks_support(phase)
        state = estimator.update(lock_support=lock_support)
        c = state["c"]
        c_dot = state["c_dot"]
        L = state["L"]
        q = state["q"]
        v = state["v"]
        joint_positions = q[nq_base:]
        support_foot_link = state["support_foot_link"]
        foot_contacts = state["foot_contacts"]
        load_shift_metrics = state["load_shift_metrics"]
        elapsed = t - phase_start_time

        next_phase = determine_next_phase(
            phase,
            elapsed,
            t,
            state,
            load_shift_metrics,
            ss_state,
            c_ref,
            swing_foot_link,
            joint_positions,
            candidate_foot_links,
            initial_foot_pos,
            locked_support_foot_link,
            robot,
        )

        if next_phase is not None:
            phase = next_phase
            phase_start_time = t
            (
                standing_com_z,
                entry_transition_start_time,
                entry_start_xy,
                entry_end_xy,
                initial_support_contact,
                support_contact_local_positions,
                initial_support_yaw,
            ) = handle_phase_entry(
                phase,
                t,
                c,
                foot_contacts,
                nominal_c_ref,
                robot,
                locked_support_foot_link,
                support_foot_link,
                ss_state,
                direct_pose,
                standing_com_z,
                initial_support_contact,
                support_contact_local_positions,
                initial_support_yaw,
            )
            if entry_transition_start_time is not None:
                ds_to_ss_transition_start_time = entry_transition_start_time
            if entry_start_xy is not None:
                ds_to_ss_start_xy = entry_start_xy
            if entry_end_xy is not None:
                ds_to_ss_end_xy = entry_end_xy

        lock_support = phase_locks_support(phase)
        if lock_support:
            support_foot_link = locked_support_foot_link
            support_contact = get_contact_entry(foot_contacts, support_foot_link)
            measured_support_point = (
                support_contact["position"].copy()
                if support_contact is not None
                else robot.get_link_com_position(support_foot_link)
            )
            filtered_support_point = (
                (1.0 - SUPPORT_POINT_FILTER) * filtered_support_point
                + SUPPORT_POINT_FILTER * measured_support_point
            )
            p_foot = filtered_support_point.copy()
        else:
            p_foot = state["p_foot"]

        if phase in (ControlPhase.INIT_SETTLE, ControlPhase.DOUBLE_SUPPORT_HOLD):
            foot_positions = [fc["position"] for fc in foot_contacts]
            if foot_positions:
                nominal_c_ref[:2] = np.mean([p[:2] for p in foot_positions], axis=0)

        update_single_support_establishment(ss_state, phase, t, load_shift_metrics)

        c_dot_ref = np.zeros(3)
        c_ddot_ref = np.zeros(3)

        c_ref = compute_phase_com_target(
            nominal_c_ref,
            phase,
            ss_state.com_ref,
            initial_foot_pos,
            locked_support_foot_link,
            swing_foot_link,
        )
        if uses_transfer_com_trajectory(phase, ss_state):
            if ds_to_ss_transition_start_time is not None:
                progress = (t - ds_to_ss_transition_start_time) / ds_to_ss_duration
                c_ref, c_dot_ref, c_ddot_ref = compute_transition_com_trajectory(
                    progress,
                    ds_to_ss_start_xy,
                    ds_to_ss_end_xy,
                    ds_to_ss_duration,
                )

        # Use actual standing CoM height instead of hardcoded H_COM=0.8
        if standing_com_z is not None:
            c_ref[2] = standing_com_z

        x_ref[:3] = c_ref
        x_ref[3:6] = c_dot_ref
        mpc.set_reference(x_ref, u_ref)

        J_c = None
        active_wbc_solver = wbc_ss
        entry_progress = compute_single_support_entry_progress(phase, phase_start_time, t)
        if phase == ControlPhase.SINGLE_SUPPORT and ss_state.established:
            f_ref, mpc_force_target, wbc_result, active_wbc_solver, J_c, mpc_result = (
                run_single_support_control(
                    robot,
                    phase,
                    phase_start_time,
                    ss_state,
                    t,
                    step,
                    CentroidalState(q=q, v=v, c=c, c_dot=c_dot, L=L),
                    TaskReference(
                        c=c_ref,
                        c_dot=c_dot_ref,
                        c_ddot=c_ddot_ref,
                        L=L_ref,
                        L_dot=L_dot_ref,
                    ),
                    SupportContext(
                        p_foot=p_foot,
                        support_foot_link=support_foot_link,
                        swing_foot_link=swing_foot_link,
                        foot_contacts=foot_contacts,
                        contact_local_positions=support_contact_local_positions,
                        initial_contact=initial_support_contact,
                        initial_yaw=initial_support_yaw,
                    ),
                    load_shift_metrics,
                    SolverConfig(
                        mpc=mpc,
                        wbc_ss=wbc_ss,
                        wbc_ss_with_swing=wbc_ss_with_swing,
                        mpc_period=mpc_period,
                        wbc_period=wbc_period,
                    ),
                    (tau_min_limits, tau_max_limits),
                    u_ref,
                    ControlMemory(
                        mpc_force_target=mpc_force_target,
                        mpc_result=mpc_result,
                        wbc_result=wbc_result,
                    ),
                )
            )
        else:
            f_ref = u_ref.copy()
            mpc_force_target = u_ref.copy()

        joint_velocities = v[nv_base:]
        C_safe = robot.compute_coriolis_gravity(q, v)
        posture_phase = (
            ControlPhase.PRE_LIFTOFF
            if phase == ControlPhase.SINGLE_SUPPORT and not ss_state.established
            else phase
        )
        safe_targets = build_safe_targets(
            initial_dof_angles,
            joint_name_to_dof_idx,
            posture_phase,
            phase_start_time,
            ss_state.joint_ref,
            t,
            swing_leg,
            support_leg,
            c,
            c_dot,
            c_ref,
            load_shift_metrics,
            optimized_joint_angles=direct_pose,
        )
        safe_tau = compute_safe_tau(
            initial_dof_angles,
            safe_targets,
            joint_positions,
            joint_velocities,
            C_safe,
            tau_min_limits,
            tau_max_limits,
            swing_leg_dof_indices,
            posture_phase,
            phase_start_time,
            t,
            load_shift_metrics,
        )

        if phase in (ControlPhase.LOAD_SHIFT, ControlPhase.PRE_LIFTOFF):
            # Double-support WBC with 4D contacts (3D force + yaw moment)
            # at the box centroid of each foot. This matches MuJoCo's box-plane
            # contact physics better than the old 3D point-contact model.
            M = robot.compute_mass_matrix(q)
            C = robot.compute_coriolis_gravity(q, v)
            J_c_support = robot.get_foot_jacobian(
                preferred_support_foot_link, q, local_position=ds_support_centroid_local
            )
            J_c_swing = robot.get_foot_jacobian(
                swing_foot_link, q, local_position=ds_swing_centroid_local
            )
            J_c = np.vstack([J_c_support, J_c_swing])
            Jc_dot = np.zeros_like(J_c)
            J_com = robot.get_com_jacobian(q)
            J_L = robot.get_angular_momentum_jacobian(q)

            c_ddot_des = wbc_ds.compute_desired_acceleration(
                c_ref, c, c_dot_ref, c_dot, c_ddot_ref
            )
            L_dot_des = wbc_ds.compute_desired_momentum_rate(
                L_ref, L, L_dot_ref, np.zeros(3)
            )

            # Build 4D force reference for 2 contacts: [fx, fy, fz, mz] per foot
            f_ref_ds = np.zeros(8)
            support_force = 0.0
            swing_force = 0.0
            for fc in foot_contacts:
                if fc["link"] == preferred_support_foot_link:
                    support_force = fc["normal_force"]
                elif fc["link"] == swing_foot_link:
                    swing_force = fc["normal_force"]
            total_force = support_force + swing_force
            if total_force < 10.0:
                total_force = -GRAVITY[2] * robot.total_mass
                support_force = total_force * 0.5
                swing_force = total_force * 0.5
            if phase == ControlPhase.LOAD_SHIFT:
                phase_progress = smoothstep(
                    (t - phase_start_time) / max(LOAD_SHIFT_TIME, 1e-6)
                )
                target_support_ratio = (
                    (1.0 - phase_progress) * 0.5
                    + phase_progress * LOAD_SHIFT_SUPPORT_RATIO
                )
            elif phase == ControlPhase.PRE_LIFTOFF:
                phase_progress = smoothstep(
                    (t - phase_start_time) / max(PRE_LIFTOFF_TIME, 1e-6)
                )
                target_support_ratio = (
                    (1.0 - phase_progress) * LOAD_SHIFT_SUPPORT_RATIO
                    + phase_progress * SINGLE_SUPPORT_SUPPORT_RATIO
                )

            # Force task: total contact force should equal m*(c_ddot_des - g)
            # This prevents the optimizer from using internal v_dot/f cancellation
            # and instead drives the CoM through realizable contact forces.
            force_task_matrix = np.zeros((4, 8))
            force_task_matrix[0, 0] = 1.0
            force_task_matrix[0, 4] = 1.0
            force_task_matrix[1, 1] = 1.0
            force_task_matrix[1, 5] = 1.0
            force_task_matrix[2, 2] = 1.0
            force_task_matrix[2, 6] = 1.0
            force_task_matrix[3, 2] = 1.0
            force_task_matrix[3, 6] = -1.0
            force_task_ref = robot.total_mass * (c_ddot_des - GRAVITY)
            target_total_vertical_force = max(force_task_ref[2], 1e-6)
            target_support_force = target_support_ratio * target_total_vertical_force
            target_swing_force = (1.0 - target_support_ratio) * target_total_vertical_force
            f_ref_ds[2] = target_support_force
            f_ref_ds[6] = target_swing_force
            force_task_ref = np.concatenate([
                force_task_ref,
                np.array([target_support_force - target_swing_force]),
            ])
            force_task_weight = np.array([
                DS_TOTAL_FORCE_XY_WEIGHT,
                DS_TOTAL_FORCE_XY_WEIGHT,
                DS_TOTAL_FORCE_Z_WEIGHT,
                DS_LOAD_DISTRIBUTION_WEIGHT,
            ])

            wbc_ds_result = wbc_ds.solve(
                M, C, J_c, Jc_dot,
                J_com, J_L,
                c_ddot_des, L_dot_des,
                f_ref_ds, v,
                tau_min_limits, tau_max_limits,
                force_task_matrix=force_task_matrix,
                force_task_ref=force_task_ref,
                force_task_weight=force_task_weight,
            )
            if wbc_ds_result is not None:
                applied_tau = wbc_ds_result["tau"].copy()
                wbc_result = wbc_ds_result
            else:
                applied_tau = safe_tau.copy()
                if t - last_wbc_warn_time >= 0.25:
                    print(
                        f"[WARN] t={t:.3f}s double-support WBC failed, using safe_tau fallback"
                    )
                    last_wbc_warn_time = t
            robot.set_joint_torques(applied_tau)
        elif phase != ControlPhase.SINGLE_SUPPORT:
            applied_tau = safe_tau.copy()
            robot.set_joint_torques(applied_tau)
        else:
            support_mask = np.ones(robot.num_joints, dtype=bool)
            support_mask[swing_leg_dof_indices] = False
            tau_cmd = safe_tau.copy()
            support_contact = get_contact_entry(foot_contacts, support_foot_link)
            support_force = support_contact["normal_force"] if support_contact is not None else 0.0
            hold_force_threshold = (
                SINGLE_SUPPORT_HOLD_FORCE
                if phase == ControlPhase.SINGLE_SUPPORT
                else MIN_SUPPORT_FORCE
            )

            transition_alpha = min(
                1.0,
                max(
                    0.0,
                    (t - phase_start_time) / max(TRANSITION_BLEND_TIME, SINGLE_SUPPORT_ENTRY_TIME),
                ),
            )
            if wbc_result is not None:
                last_valid_support_tau = np.clip(
                    wbc_result["tau"][support_mask],
                    tau_min_limits[support_mask],
                    tau_max_limits[support_mask],
                )
            elif last_valid_support_tau is None:
                last_valid_support_tau = safe_tau[support_mask].copy()

            support_alpha = min(1.0, max(0.0, support_force / max(MIN_SUPPORT_FORCE, 1e-6)))
            max_tau_blend = (
                SINGLE_SUPPORT_MAX_TAU_BLEND
                if phase == ControlPhase.SINGLE_SUPPORT
                else 1.0
            )
            effective_alpha = min(
                max_tau_blend,
                transition_alpha * support_alpha,
            )
            tau_cmd[support_mask] = (
                (1.0 - effective_alpha) * safe_tau[support_mask]
                + effective_alpha * last_valid_support_tau
            )

            if (
                wbc_result is None or support_force < hold_force_threshold
            ) and t - last_wbc_warn_time >= 0.25:
                fallback_reason = (
                    f"support force below hold threshold ({support_force:.1f}N)"
                    if support_force < hold_force_threshold
                    else f"solver status={active_wbc_solver.last_status}"
                )
                print(
                    f"[WARN] t={t:.3f}s using fallback support torque: {fallback_reason}, "
                    f"phase={phase.name}, "
                    f"support={support_name(robot.link_to_foot_name, support_foot_link)}"
                )
                last_wbc_warn_time = t

            applied_tau = tau_cmd.copy()
            robot.set_joint_torques(applied_tau)

        robot.step()

        time_log.append(t)
        com_log.append(c.copy())
        com_ref_log.append(x_ref[:3].copy())
        support_foot_log.append(support_foot_link)

        for fc in foot_contacts:
            link = fc["link"]
            foot_pos_log[link].append(fc["position"].copy())
            foot_force_log[link].append(fc["normal_force"])

        tau_log.append(applied_tau.copy())
        if wbc_result is not None:
            f_opt = wbc_result["f"]
            cd = wbc_result.get("contact_dim", 3)
            f_aggregate = np.zeros(3)
            if cd == 4 and f_opt.shape[0] % 4 == 0:
                for i in range(f_opt.shape[0] // 4):
                    f_aggregate += f_opt[4 * i : 4 * i + 3]
            elif f_opt.shape[0] % 3 == 0:
                for i in range(f_opt.shape[0] // 3):
                    f_aggregate += f_opt[3 * i : 3 * i + 3]
            else:
                f_aggregate = f_opt[:3].copy()
            wbc_f_log.append(f_aggregate)
            wbc_time_log.append(wbc_result["solve_time"])
        else:
            wbc_f_log.append(np.zeros(3))
            wbc_time_log.append(0.0)

        if mpc_result is not None:
            mpc_f_ref_log.append(f_ref.copy())
            mpc_time_log.append(mpc_result.get("solve_time", 0.0))
        else:
            mpc_f_ref_log.append(np.zeros(3))
            mpc_time_log.append(0.0)

    rmse = compute_rmse(com_log, com_ref_log)
    print(f"\n{'='*50}")
    print("===== 实验结果 =====")
    print(f"CoM 位置 RMSE: {rmse:.4f} m (目标 < {RMSE_THRESH} m)")

    max_slip = 0.0
    for link in candidate_foot_links:
        positions = np.array(foot_pos_log[link])
        if len(positions) > 0:
            slips = np.linalg.norm(positions[:, :2] - initial_foot_pos[link][:2], axis=1)
            max_slip = max(max_slip, np.max(slips))
    print(f"最大足端滑移: {max_slip*1000:.2f} mm (目标 < {SLIP_THRESH*1000:.1f} mm)")

    for link in candidate_foot_links:
        forces = np.array(foot_force_log[link])
        link_name = support_name(robot.link_to_foot_name, link)
        avg_force = np.mean(forces) if len(forces) > 0 else 0.0
        print(f"{link_name} 平均接触力: {avg_force:.1f} N")

    if mpc_time_log:
        avg_mpc_time = np.mean(mpc_time_log) * 1000
        max_mpc_time = np.max(mpc_time_log) * 1000
        print(f"MPC 平均求解时间: {avg_mpc_time:.2f} ms (目标 < {MPC_TIME_THRESH*1000:.1f} ms)")
        print(f"MPC 最大求解时间: {max_mpc_time:.2f} ms")
    if wbc_time_log:
        avg_wbc_time = np.mean(wbc_time_log) * 1000
        max_wbc_time = np.max(wbc_time_log) * 1000
        print(f"WBC 平均求解时间: {avg_wbc_time:.3f} ms (目标 < {WBC_TIME_THRESH*1000:.1f} ms)")
        print(f"WBC 最大求解时间: {max_wbc_time:.3f} ms")

    plot_com_tracking(time_log, com_log, com_ref_log)
    if len(wbc_f_log) > 0:
        plot_contact_force(time_log, wbc_f_log)
    if len(tau_log) > 0:
        plot_torques(time_log, tau_log)


if __name__ == "__main__":
    main()
