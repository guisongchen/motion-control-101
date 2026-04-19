"""Main loop: MPC + WBC + MuJoCo with explicit control phases."""

from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional

import numpy as np

from config import (
    DT_SIM, WBC_FREQ, MPC_FREQ, SIM_DURATION,
    MODEL_PATH, LIFT_LEG, H_COM,
    FOOT_LINK_NAMES, STANDING_JOINT_ANGLES,
    SUPPORT_FOOT_NAME,
    BASE_INITIAL_POS, BASE_INITIAL_ORN,
    NX, NU, T_S, GRAVITY,
    RMSE_THRESH, SLIP_THRESH, MPC_TIME_THRESH, WBC_TIME_THRESH,
    POSTURE_KP, POSTURE_KD, LIFT_LEG_KP, LIFT_LEG_KD,
    TRANSITION_BLEND_TIME, SWING_RAMP_TIME,
    SWING_HIP_PITCH_TARGET, SWING_KNEE_TARGET, SUPPORT_POINT_FILTER,
    MIN_SUPPORT_FORCE, BASE_DOF_DAMPING, JOINT_DOF_DAMPING,
    INIT_SETTLE_TIME, DOUBLE_SUPPORT_READY_TIME, LOAD_SHIFT_TIME,
    PRE_LIFTOFF_TIME, DOUBLE_SUPPORT_MIN_FORCE, PRE_LIFTOFF_SWING_PROGRESS,
)
from robot_model import RobotModel
from state_estimator import StateEstimator
from mpc import CentroidalMPC
from wbc import WholeBodyController
from utils import compute_rmse, plot_com_tracking, plot_contact_force, plot_torques


class ControlPhase(Enum):
    """High-level task phases for single-support standing."""

    INIT_SETTLE = auto()
    DOUBLE_SUPPORT_HOLD = auto()
    LOAD_SHIFT = auto()
    PRE_LIFTOFF = auto()
    SINGLE_SUPPORT = auto()


@dataclass
class PhaseState:
    """Track phase machine progress and persistent control context."""

    phase: ControlPhase
    phase_start_time: float
    ready_since: Optional[float]
    locked_support_foot_link: int
    filtered_support_point: np.ndarray
    last_valid_support_tau: Optional[np.ndarray]
    last_wbc_warn_step: int


def skew(v: np.ndarray) -> np.ndarray:
    """向量叉乘矩阵。"""
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ])


def compute_centroidal_dynamics(
    m: float, c: np.ndarray, p_foot: np.ndarray, dt: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Linearize centroidal dynamics around the current support point."""
    A_d = np.eye(9)
    A_d[0:3, 3:6] = dt * np.eye(3)

    B_d = np.zeros((9, 3))
    B_d[3:6, :] = dt * np.eye(3) / m
    r = p_foot - c
    B_d[6:9, :] = dt * skew(r)

    d_d = np.zeros(9)
    d_d[3:6] = dt * (-GRAVITY)

    return A_d, B_d, d_d


def compute_pd_torque(
    q_target: np.ndarray,
    q_current: np.ndarray,
    qd_current: np.ndarray,
    kp: float,
    kd: float,
    tau_limit: np.ndarray,
) -> np.ndarray:
    """Simple joint-space PD torque for MuJoCo motor actuators."""
    tau = kp * (q_target - q_current) - kd * qd_current
    return np.clip(tau, -tau_limit, tau_limit)


def format_vector(vec: np.ndarray) -> str:
    """Compact vector formatter for runtime diagnostics."""
    return "[" + ", ".join(f"{x:.2f}" for x in vec) + "]"


def get_leg_joint_names(side: str) -> list[str]:
    """Return the 6 actuated joints of one leg."""
    return [
        f"{side}_hip_pitch_joint",
        f"{side}_hip_roll_joint",
        f"{side}_hip_yaw_joint",
        f"{side}_knee_joint",
        f"{side}_ankle_pitch_joint",
        f"{side}_ankle_roll_joint",
    ]


def get_contact_entry(foot_contacts: list[dict], link: int) -> Optional[dict]:
    """Return the contact record for one foot link."""
    return next((fc for fc in foot_contacts if fc["link"] == link), None)


def support_name(candidate_foot_links: list[int], link: int) -> str:
    """Map support link index back to the configured foot name."""
    return FOOT_LINK_NAMES[candidate_foot_links.index(link)]


def transition_phase(phase_state: PhaseState, new_phase: ControlPhase, t: float) -> None:
    """Switch phases and reset per-phase timers."""
    phase_state.phase = new_phase
    phase_state.phase_start_time = t
    phase_state.ready_since = None


def choose_support_foot(
    foot_contacts: list[dict],
    preferred_support_foot_link: int,
) -> int:
    """Choose the support foot deterministically, preferring the configured side."""
    preferred_contact = get_contact_entry(foot_contacts, preferred_support_foot_link)
    if (
        preferred_contact is not None
        and preferred_contact["normal_force"] >= MIN_SUPPORT_FORCE
    ):
        return preferred_support_foot_link
    return max(foot_contacts, key=lambda fc: fc["normal_force"])["link"]


def phase_elapsed(phase_state: PhaseState, t: float) -> float:
    """Elapsed wall-clock simulation time inside the current phase."""
    return t - phase_state.phase_start_time


def compute_swing_progress(phase_state: PhaseState, t: float) -> float:
    """Compute normalized swing progress based on the active phase."""
    elapsed = phase_elapsed(phase_state, t)
    if phase_state.phase in (ControlPhase.INIT_SETTLE, ControlPhase.DOUBLE_SUPPORT_HOLD, ControlPhase.LOAD_SHIFT):
        return 0.0
    if phase_state.phase == ControlPhase.PRE_LIFTOFF:
        return PRE_LIFTOFF_SWING_PROGRESS * min(1.0, elapsed / PRE_LIFTOFF_TIME)
    single_support_progress = elapsed / SWING_RAMP_TIME
    return min(1.0, PRE_LIFTOFF_SWING_PROGRESS + (1.0 - PRE_LIFTOFF_SWING_PROGRESS) * single_support_progress)


def build_safe_targets(
    initial_dof_angles: np.ndarray,
    joint_name_to_dof_idx: dict[str, int],
    phase_state: PhaseState,
    t: float,
    swing_leg: str,
) -> np.ndarray:
    """Build posture targets for the current phase."""
    targets = initial_dof_angles.copy()
    swing_progress = compute_swing_progress(phase_state, t)
    hip_name = f"{swing_leg}_hip_pitch_joint"
    knee_name = f"{swing_leg}_knee_joint"

    if hip_name in joint_name_to_dof_idx:
        hip_idx = joint_name_to_dof_idx[hip_name]
        targets[hip_idx] = (
            (1.0 - swing_progress) * initial_dof_angles[hip_idx]
            + swing_progress * SWING_HIP_PITCH_TARGET
        )
    if knee_name in joint_name_to_dof_idx:
        knee_idx = joint_name_to_dof_idx[knee_name]
        targets[knee_idx] = (
            (1.0 - swing_progress) * initial_dof_angles[knee_idx]
            + swing_progress * SWING_KNEE_TARGET
        )
    return targets


def compute_safe_tau(
    initial_dof_angles: np.ndarray,
    safe_targets: np.ndarray,
    joint_positions: np.ndarray,
    joint_velocities: np.ndarray,
    C_safe: np.ndarray,
    tau_min_limits: np.ndarray,
    tau_max_limits: np.ndarray,
    swing_leg_dof_indices: list[int],
) -> np.ndarray:
    """Build posture-hold torques with bias-force compensation."""
    safe_tau = C_safe[6:] + compute_pd_torque(
        initial_dof_angles,
        joint_positions,
        joint_velocities,
        POSTURE_KP,
        POSTURE_KD,
        tau_max_limits,
    )

    if swing_leg_dof_indices:
        swing_idx = np.array(swing_leg_dof_indices, dtype=int)
        safe_tau[swing_idx] = (
            C_safe[6:][swing_idx]
            + compute_pd_torque(
                safe_targets[swing_idx],
                joint_positions[swing_idx],
                joint_velocities[swing_idx],
                LIFT_LEG_KP,
                LIFT_LEG_KD,
                tau_max_limits[swing_idx],
            )
        )

    return np.clip(safe_tau, tau_min_limits, tau_max_limits)


def update_phase_machine(
    phase_state: PhaseState,
    t: float,
    foot_contacts: list[dict],
    preferred_support_foot_link: int,
    swing_foot_link: int,
    initial_foot_pos: dict[int, np.ndarray],
    candidate_foot_links: list[int],
) -> Optional[str]:
    """Advance the high-level control phase when entry/exit conditions are met."""
    elapsed = phase_elapsed(phase_state, t)
    double_support_forces = [
        get_contact_entry(foot_contacts, link)["normal_force"]
        if get_contact_entry(foot_contacts, link) is not None
        else 0.0
        for link in candidate_foot_links
    ]
    max_double_support_slip = max(
        np.linalg.norm(
            get_contact_entry(foot_contacts, link)["position"][:2] - initial_foot_pos[link][:2]
        )
        for link in candidate_foot_links
    )
    support_contact = get_contact_entry(foot_contacts, phase_state.locked_support_foot_link)
    support_force = support_contact["normal_force"] if support_contact is not None else 0.0
    swing_contact = get_contact_entry(foot_contacts, swing_foot_link)
    swing_force = swing_contact["normal_force"] if swing_contact is not None else 0.0

    if phase_state.phase == ControlPhase.INIT_SETTLE:
        if elapsed >= INIT_SETTLE_TIME:
            transition_phase(phase_state, ControlPhase.DOUBLE_SUPPORT_HOLD, t)
            return "double-support validation"
        return None

    if phase_state.phase == ControlPhase.DOUBLE_SUPPORT_HOLD:
        both_feet_ready = all(force >= DOUBLE_SUPPORT_MIN_FORCE for force in double_support_forces)
        stable_slip = max_double_support_slip <= SLIP_THRESH
        if both_feet_ready and stable_slip:
            if phase_state.ready_since is None:
                phase_state.ready_since = t
            if t - phase_state.ready_since >= DOUBLE_SUPPORT_READY_TIME:
                phase_state.locked_support_foot_link = choose_support_foot(
                    foot_contacts, preferred_support_foot_link
                )
                phase_state.filtered_support_point = initial_foot_pos[
                    phase_state.locked_support_foot_link
                ].copy()
                transition_phase(phase_state, ControlPhase.LOAD_SHIFT, t)
                return (
                    "load shift "
                    f"(support={support_name(candidate_foot_links, phase_state.locked_support_foot_link)})"
                )
        else:
            phase_state.ready_since = None
        return None

    if phase_state.phase == ControlPhase.LOAD_SHIFT:
        if elapsed >= LOAD_SHIFT_TIME:
            transition_phase(phase_state, ControlPhase.PRE_LIFTOFF, t)
            return "pre-liftoff"
        return None

    if phase_state.phase == ControlPhase.PRE_LIFTOFF:
        if elapsed >= PRE_LIFTOFF_TIME:
            transition_phase(phase_state, ControlPhase.SINGLE_SUPPORT, t)
            return (
                "single-support control "
                f"(support={support_name(candidate_foot_links, phase_state.locked_support_foot_link)})"
            )
        return None

    return None


def main():
    robot = RobotModel(MODEL_PATH)
    robot.model.opt.gravity[:] = GRAVITY
    robot.model.opt.timestep = DT_SIM
    robot.model.dof_damping[:6] = BASE_DOF_DAMPING
    robot.model.dof_damping[6:] = JOINT_DOF_DAMPING
    robot.reset_base_pose(BASE_INITIAL_POS, BASE_INITIAL_ORN)

    candidate_foot_links = [robot.link_name_to_index[name] for name in FOOT_LINK_NAMES]
    foot_name_to_link = {name: robot.link_name_to_index[name] for name in FOOT_LINK_NAMES}
    preferred_support_foot_link = foot_name_to_link[SUPPORT_FOOT_NAME]
    swing_leg = LIFT_LEG
    support_leg = "right" if swing_leg == "left" else "left"
    swing_foot_link = foot_name_to_link[f"{swing_leg}_ankle_roll_link"]
    estimator = StateEstimator(robot, candidate_foot_links)

    initial_dof_angles = np.zeros(len(robot.dof_joints))
    joint_name_to_dof_idx = robot.dof_joint_name_to_index.copy()
    for idx, joint_name in enumerate(robot.dof_joint_names):
        if joint_name in STANDING_JOINT_ANGLES:
            initial_dof_angles[idx] = STANDING_JOINT_ANGLES[joint_name]
    robot.reset_joint_positions(initial_dof_angles)

    tau_max_limits = robot.tau_limits.copy()
    tau_min_limits = -tau_max_limits
    swing_leg_dof_indices = [
        joint_name_to_dof_idx[name]
        for name in get_leg_joint_names(swing_leg)
        if name in joint_name_to_dof_idx
    ]
    support_leg_dof_indices = [
        joint_name_to_dof_idx[name]
        for name in get_leg_joint_names(support_leg)
        if name in joint_name_to_dof_idx
    ]

    mpc = CentroidalMPC()
    wbc = WholeBodyController(robot.nv)

    x_ref = np.zeros(NX)
    x_ref[2] = H_COM
    u_ref = np.zeros(NU)
    u_ref[2] = -GRAVITY[2] * robot.total_mass
    mpc.set_reference(x_ref, u_ref)

    c_ref = x_ref[:3]
    c_dot_ref = x_ref[3:6]
    L_ref = x_ref[6:9]
    c_ddot_ref = np.zeros(3)
    L_dot_ref = np.zeros(3)

    wbc_period = max(1, int(1.0 / (WBC_FREQ * DT_SIM)))
    mpc_period = max(1, int(1.0 / (MPC_FREQ * DT_SIM)))
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

    initial_foot_pos = {link: robot.get_link_com_position(link) for link in candidate_foot_links}
    phase_state = PhaseState(
        phase=ControlPhase.INIT_SETTLE,
        phase_start_time=0.0,
        ready_since=None,
        locked_support_foot_link=preferred_support_foot_link,
        filtered_support_point=initial_foot_pos[preferred_support_foot_link].copy(),
        last_valid_support_tau=None,
        last_wbc_warn_step=-10_000,
    )

    mpc_result = None
    wbc_result = None
    f_ref = u_ref.copy()

    print("\n===== 开始仿真（MuJoCo 单足站立测试模式）=====")
    print(f"总质量: {robot.total_mass:.2f} kg")
    print(f"仿真时长: {SIM_DURATION:.1f} s")
    print(f"MPC 周期: {mpc_period} 步 ({mpc_period * DT_SIM * 1000:.1f} ms)")
    print(f"WBC 周期: {wbc_period} 步 ({wbc_period * DT_SIM * 1000:.3f} ms)")
    print("=" * 50)

    for step in range(total_steps):
        t = step * DT_SIM
        use_wbc = phase_state.phase == ControlPhase.SINGLE_SUPPORT
        lock_support = phase_state.phase in (
            ControlPhase.LOAD_SHIFT,
            ControlPhase.PRE_LIFTOFF,
            ControlPhase.SINGLE_SUPPORT,
        )
        state = estimator.update(
            preferred_support_foot_link=phase_state.locked_support_foot_link if lock_support else None,
            lock_support=lock_support,
        )
        c = state["c"]
        c_dot = state["c_dot"]
        L = state["L"]
        q = state["q"]
        v = state["v"]
        support_foot_link = state["support_foot_link"]
        foot_contacts = state["foot_contacts"]

        transition_msg = update_phase_machine(
            phase_state,
            t,
            foot_contacts,
            preferred_support_foot_link,
            swing_foot_link,
            initial_foot_pos,
            candidate_foot_links,
        )
        if transition_msg is not None:
            print(f"[INFO] t={t:.3f}s 进入阶段: {transition_msg}")

        lock_support = phase_state.phase in (
            ControlPhase.LOAD_SHIFT,
            ControlPhase.PRE_LIFTOFF,
            ControlPhase.SINGLE_SUPPORT,
        )
        use_wbc = phase_state.phase == ControlPhase.SINGLE_SUPPORT
        if lock_support:
            support_foot_link = phase_state.locked_support_foot_link
            support_contact = get_contact_entry(foot_contacts, support_foot_link)
            measured_support_point = (
                support_contact["position"].copy()
                if support_contact is not None
                else robot.get_link_com_position(support_foot_link)
            )
            phase_state.filtered_support_point = (
                (1.0 - SUPPORT_POINT_FILTER) * phase_state.filtered_support_point
                + SUPPORT_POINT_FILTER * measured_support_point
            )
            p_foot = phase_state.filtered_support_point.copy()
        else:
            p_foot = state["p_foot"]

        x0 = np.concatenate([c, c_dot, L])

        M = None
        C = None
        J_c = None
        if use_wbc and step % mpc_period == 0:
            A_d, B_d, d_d = compute_centroidal_dynamics(robot.total_mass, c, p_foot, T_S)
            mpc.set_dynamics(A_d, B_d, d_d)
            mpc_result = mpc.solve(x0)
            if mpc_result is not None:
                f_ref = mpc_result["u0"]
                mpc_time_log.append(mpc_result["solve_time"])
            else:
                print(f"[WARN] t={t:.3f}s MPC 求解失败")

        if use_wbc and step % wbc_period == 0:
            M = robot.compute_mass_matrix(q)
            C = robot.compute_coriolis_gravity(q, v)
            J_c = robot.get_foot_jacobian(support_foot_link, q)
            J_com = robot.get_com_jacobian(q)
            J_L = robot.get_angular_momentum_jacobian(q)
            Jc_dot = np.zeros((6, robot.nv))

            c_ddot_des = wbc.compute_desired_acceleration(
                c_ref, c, c_dot_ref, c_dot, c_ddot_ref
            )
            L_dot_des = wbc.compute_desired_momentum_rate(
                L_ref, L, L_dot_ref, np.zeros(3)
            )

            wbc_result = wbc.solve(
                M, C, J_c, Jc_dot,
                J_com, J_L,
                c_ddot_des, L_dot_des,
                f_ref, v,
                tau_min_limits, tau_max_limits,
            )
            if wbc_result is None:
                print(f"[WARN] t={t:.3f}s WBC 求解失败")

        joint_positions = q[7:]
        joint_velocities = v[6:]
        C_safe = robot.compute_coriolis_gravity(q, v)
        safe_targets = build_safe_targets(
            initial_dof_angles,
            joint_name_to_dof_idx,
            phase_state,
            t,
            swing_leg,
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
        )

        if not use_wbc:
            applied_tau = safe_tau.copy()
            robot.set_joint_torques(applied_tau)
        else:
            support_mask = np.ones(robot.num_joints, dtype=bool)
            support_mask[swing_leg_dof_indices] = False
            tau_cmd = safe_tau.copy()
            support_contact = get_contact_entry(foot_contacts, support_foot_link)
            support_force = support_contact["normal_force"] if support_contact is not None else 0.0
            support_contact_ready = support_force >= MIN_SUPPORT_FORCE

            transition_alpha = min(
                1.0, max(0.0, phase_elapsed(phase_state, t) / TRANSITION_BLEND_TIME)
            )
            if wbc_result is not None and support_contact_ready:
                phase_state.last_valid_support_tau = np.clip(
                    wbc_result["tau"][support_mask],
                    tau_min_limits[support_mask],
                    tau_max_limits[support_mask],
                )
            elif phase_state.last_valid_support_tau is None:
                phase_state.last_valid_support_tau = safe_tau[support_mask].copy()

            effective_alpha = transition_alpha if support_contact_ready else 0.0
            tau_cmd[support_mask] = (
                (1.0 - effective_alpha) * safe_tau[support_mask]
                + effective_alpha * phase_state.last_valid_support_tau
            )

            if (wbc_result is None or not support_contact_ready) and step - phase_state.last_wbc_warn_step >= 60:
                J_c_lin = J_c[:3, :] if J_c is not None else np.zeros((3, robot.nv))
                J_c_gram = J_c_lin @ J_c_lin.T
                J_c_cond = np.linalg.cond(J_c_gram + 1e-6 * np.eye(3))
                fallback_reason = (
                    f"support force below threshold ({support_force:.1f}N)"
                    if not support_contact_ready
                    else f"solver status={wbc.last_status}"
                )
                print(
                    f"[WARN] t={t:.3f}s using fallback support torque: {fallback_reason}, "
                    f"phase={phase_state.phase.name}, "
                    f"support={support_name(candidate_foot_links, support_foot_link)}, "
                    f"force={support_force:.1f}N, alpha={effective_alpha:.2f}, "
                    f"rank(Jc)={np.linalg.matrix_rank(J_c_lin)}, "
                    f"cond(JcJc^T)={J_c_cond:.2e}, f_ref={format_vector(f_ref)}"
                )
                phase_state.last_wbc_warn_step = step

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
            wbc_f_log.append(wbc_result["f"].copy())
            wbc_time_log.append(wbc_result["solve_time"])
        else:
            wbc_f_log.append(np.zeros(3))

        if mpc_result is not None:
            mpc_f_ref_log.append(f_ref.copy())
        else:
            mpc_f_ref_log.append(np.zeros(3))

        if (step + 1) % 240 == 0:
            print(f"\n--- t={t:.3f}s [{phase_state.phase.name}] ---")
            print(f"  CoM: [{c[0]:.3f}, {c[1]:.3f}, {c[2]:.3f}]  (ref z={H_COM:.2f})")
            mpc_t = mpc_time_log[-1] * 1000 if mpc_time_log else 0.0
            wbc_t = wbc_time_log[-1] * 1000 if wbc_time_log else 0.0
            print(f"  MPC solve: {mpc_t:.2f} ms | WBC solve: {wbc_t:.2f} ms")
            for fc in foot_contacts:
                link_name = support_name(candidate_foot_links, fc["link"])
                slip = np.linalg.norm(fc["position"][:2] - initial_foot_pos[fc["link"]][:2])
                print(f"  {link_name}: force={fc['normal_force']:.1f}N  slip={slip*1000:.2f}mm")

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
        link_name = support_name(candidate_foot_links, link)
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
