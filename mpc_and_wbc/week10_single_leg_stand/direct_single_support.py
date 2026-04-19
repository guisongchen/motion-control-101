"""Direct-spawn single-support WBC benchmark focused on support-foot slip."""

from dataclasses import dataclass
import math

import numpy as np

import config
import wbc as wbc_module
from main import compute_pd_torque, get_leg_joint_names
from robot_model import RobotModel
from state_estimator import StateEstimator
from wbc import WholeBodyController


@dataclass
class DirectSingleSupportResult:
    survived_s: float
    final_base_z: float
    max_contact_slip_mm: float
    max_body_slip_mm: float
    max_swing_force: float
    max_friction_ratio: float
    max_tangential_force: float
    wbc_failures: int
    success: bool


def quat_from_roll(roll: float) -> np.ndarray:
    """Build an xyzw quaternion from a pure roll angle."""
    return np.array([math.sin(roll / 2.0), 0.0, 0.0, math.cos(roll / 2.0)], dtype=float)


def build_direct_pose(
    joint_names: list[str],
    support_leg: str,
) -> np.ndarray:
    """Construct the tuned direct single-support pose in DOF order."""
    swing_leg = "left" if support_leg == "right" else "right"
    support_sign = 1.0 if support_leg == "right" else -1.0
    pose = dict(config.STANDING_JOINT_ANGLES)

    pose[f"{support_leg}_hip_pitch_joint"] = -0.28 - config.DIRECT_SINGLE_SUPPORT_SUPPORT_PITCH_DELTA
    pose[f"{support_leg}_knee_joint"] = 0.62 + 0.4 * config.DIRECT_SINGLE_SUPPORT_SUPPORT_PITCH_DELTA
    pose[f"{support_leg}_ankle_pitch_joint"] = -0.34 + 0.2 * config.DIRECT_SINGLE_SUPPORT_SUPPORT_PITCH_DELTA

    pose[f"{support_leg}_hip_roll_joint"] = support_sign * config.DIRECT_SINGLE_SUPPORT_SUPPORT_ROLL_DELTA
    pose[f"{support_leg}_ankle_roll_joint"] = -support_sign * config.DIRECT_SINGLE_SUPPORT_SUPPORT_ROLL_DELTA

    pose[f"{swing_leg}_hip_pitch_joint"] = config.DIRECT_SINGLE_SUPPORT_SWING_HIP_PITCH
    pose[f"{swing_leg}_knee_joint"] = config.DIRECT_SINGLE_SUPPORT_SWING_KNEE
    pose[f"{swing_leg}_ankle_pitch_joint"] = config.DIRECT_SINGLE_SUPPORT_SWING_ANKLE_PITCH
    pose[f"{swing_leg}_hip_roll_joint"] = support_sign * config.DIRECT_SINGLE_SUPPORT_SWING_ROLL_DELTA
    pose[f"{swing_leg}_ankle_roll_joint"] = -support_sign * config.DIRECT_SINGLE_SUPPORT_SWING_ROLL_DELTA

    pose[f"{support_leg}_shoulder_pitch_joint"] = config.DIRECT_SINGLE_SUPPORT_SUPPORT_ARM_SHOULDER_PITCH
    pose[f"{support_leg}_shoulder_roll_joint"] = (
        support_sign * config.DIRECT_SINGLE_SUPPORT_SUPPORT_ARM_SHOULDER_ROLL
    )
    pose[f"{support_leg}_elbow_joint"] = support_sign * config.DIRECT_SINGLE_SUPPORT_SUPPORT_ARM_ELBOW

    q_target = np.zeros(len(joint_names))
    for idx, name in enumerate(joint_names):
        q_target[idx] = pose.get(name, 0.0)
    return q_target


def resolve_support_contact_local_position(
    robot: RobotModel,
    support_link: int,
    initial_support_contact: np.ndarray,
) -> np.ndarray | None:
    """Pick the foot point whose acceleration WBC constrains to zero."""
    mode = config.DIRECT_SINGLE_SUPPORT_WBC_CONTACT_POINT
    if mode == "body_origin":
        return None
    if mode == "initial_contact":
        body_origin = np.array(robot.data.xpos[support_link], copy=True)
        body_rotation = np.array(robot.data.xmat[support_link]).reshape(3, 3)
        return body_rotation.T @ (initial_support_contact - body_origin)
    raise ValueError(f"Unsupported direct single-support contact point mode: {mode}")


def run_direct_single_support() -> DirectSingleSupportResult:
    """Run the direct single-support WBC benchmark with tuned current-best settings."""
    support_leg = "right" if config.DIRECT_SINGLE_SUPPORT_SUPPORT_FOOT_NAME.startswith("right") else "left"
    swing_leg = "left" if support_leg == "right" else "right"
    support_sign = 1.0 if support_leg == "right" else -1.0

    robot = RobotModel(config.MODEL_PATH)
    robot.model.opt.gravity[:] = config.GRAVITY
    robot.model.opt.timestep = config.DT_SIM
    robot.model.dof_damping[:6] = config.BASE_DOF_DAMPING * config.DIRECT_SINGLE_SUPPORT_DAMPING_SCALE
    robot.model.dof_damping[6:] = config.JOINT_DOF_DAMPING * config.DIRECT_SINGLE_SUPPORT_DAMPING_SCALE

    base_pos = np.array(
        [
            0.0,
            -support_sign * config.DIRECT_SINGLE_SUPPORT_BASE_LATERAL_SHIFT,
            config.DIRECT_SINGLE_SUPPORT_BASE_HEIGHT,
        ],
        dtype=float,
    )
    base_orn = quat_from_roll(-support_sign * config.DIRECT_SINGLE_SUPPORT_BASE_ROLL)
    robot.reset_base_pose(base_pos, base_orn)

    q_target = build_direct_pose(robot.dof_joint_names, support_leg)
    robot.reset_joint_positions(q_target)

    candidate_foot_links = [robot.link_name_to_index[name] for name in config.FOOT_LINK_NAMES]
    support_link = robot.link_name_to_index[config.DIRECT_SINGLE_SUPPORT_SUPPORT_FOOT_NAME]
    swing_link = next(link for link in candidate_foot_links if link != support_link)
    estimator = StateEstimator(robot, candidate_foot_links)
    controller = WholeBodyController(robot.nv)

    swing_leg_dof_indices = [
        robot.dof_joint_name_to_index[name]
        for name in get_leg_joint_names(swing_leg)
        if name in robot.dof_joint_name_to_index
    ]

    original_gains = (
        wbc_module.Kp_c,
        wbc_module.Kd_c,
        wbc_module.Kp_L,
        wbc_module.Kd_L,
    )
    wbc_module.Kp_c = config.DIRECT_SINGLE_SUPPORT_COM_KP
    wbc_module.Kd_c = config.DIRECT_SINGLE_SUPPORT_COM_KD
    wbc_module.Kp_L = config.DIRECT_SINGLE_SUPPORT_MOMENTUM_KP
    wbc_module.Kd_L = config.DIRECT_SINGLE_SUPPORT_MOMENTUM_KD

    initial_support_contact = robot.get_contact_metrics(support_link)["position"].copy()
    initial_support_body = robot.get_link_com_position(support_link).copy()
    state0 = estimator.update(preferred_support_foot_link=support_link, lock_support=True)
    c_ref = state0["c"].copy()
    f_ref = np.array([0.0, 0.0, -config.GRAVITY[2] * robot.total_mass], dtype=float)

    total_steps = int(config.DIRECT_SINGLE_SUPPORT_DURATION / config.DT_SIM)
    wbc_failures = 0
    max_contact_slip = 0.0
    max_body_slip = 0.0
    max_swing_force = 0.0
    max_friction_ratio = 0.0
    max_tangential_force = 0.0
    final_t = 0.0
    final_base_z = float(robot.data.qpos[2])

    print("\n===== 直接单足支撑 WBC 基准测试 =====")
    print(f"support foot: {config.DIRECT_SINGLE_SUPPORT_SUPPORT_FOOT_NAME}")
    print(f"duration: {config.DIRECT_SINGLE_SUPPORT_DURATION:.1f} s")
    print(f"damping scale: {config.DIRECT_SINGLE_SUPPORT_DAMPING_SCALE:.2f}")
    print(f"support blend: {config.DIRECT_SINGLE_SUPPORT_SUPPORT_BLEND:.3f}")
    print(f"contact point: {config.DIRECT_SINGLE_SUPPORT_WBC_CONTACT_POINT}")
    print("=" * 50)

    try:
        support_contact_local_position = resolve_support_contact_local_position(
            robot,
            support_link,
            initial_support_contact,
        )
        for step in range(total_steps):
            state = estimator.update(preferred_support_foot_link=support_link, lock_support=True)
            q = state["q"]
            v = state["v"]

            C = robot.compute_coriolis_gravity(q, v)
            safe_tau = C[6:] + compute_pd_torque(
                q_target,
                q[7:],
                v[6:],
                config.DIRECT_SINGLE_SUPPORT_POSTURE_KP,
                config.DIRECT_SINGLE_SUPPORT_POSTURE_KD,
                robot.tau_limits,
            )

            M = robot.compute_mass_matrix(q)
            J_c = robot.get_foot_jacobian(
                support_link,
                q,
                local_position=support_contact_local_position,
            )
            J_com = robot.get_com_jacobian(q)
            J_L = robot.get_angular_momentum_jacobian(q)
            c_ddot_des = controller.compute_desired_acceleration(
                c_ref,
                state["c"],
                np.zeros(3),
                state["c_dot"],
                np.zeros(3),
            )
            L_dot_des = controller.compute_desired_momentum_rate(
                np.zeros(3),
                state["L"],
                np.zeros(3),
                np.zeros(3),
            )
            wbc_result = controller.solve(
                M,
                C,
                J_c,
                np.zeros((6, robot.nv)),
                J_com,
                J_L,
                c_ddot_des,
                L_dot_des,
                f_ref,
                v,
                -robot.tau_limits,
                robot.tau_limits,
            )
            if wbc_result is None:
                wbc_failures += 1

            tau_cmd = safe_tau.copy()
            if wbc_result is not None:
                support_mask = np.ones(robot.num_joints, dtype=bool)
                support_mask[swing_leg_dof_indices] = False
                tau_cmd[support_mask] = (
                    (1.0 - config.DIRECT_SINGLE_SUPPORT_SUPPORT_BLEND) * safe_tau[support_mask]
                    + config.DIRECT_SINGLE_SUPPORT_SUPPORT_BLEND
                    * np.clip(
                        wbc_result["tau"][support_mask],
                        -robot.tau_limits[support_mask],
                        robot.tau_limits[support_mask],
                    )
                )

            robot.set_joint_torques(np.clip(tau_cmd, -robot.tau_limits, robot.tau_limits))
            robot.step()

            support_metrics = robot.get_contact_metrics(support_link)
            swing_metrics = robot.get_contact_metrics(swing_link)
            support_body = robot.get_link_com_position(support_link)
            max_contact_slip = max(
                max_contact_slip,
                float(np.linalg.norm(support_metrics["position"][:2] - initial_support_contact[:2])),
            )
            max_body_slip = max(
                max_body_slip,
                float(np.linalg.norm(support_body[:2] - initial_support_body[:2])),
            )
            max_swing_force = max(max_swing_force, float(swing_metrics["normal_force"]))
            max_friction_ratio = max(max_friction_ratio, float(support_metrics["friction_ratio"]))
            max_tangential_force = max(max_tangential_force, float(support_metrics["tangential_force"]))

            final_t = (step + 1) * config.DT_SIM
            final_base_z = float(robot.data.qpos[2])

            if (step + 1) % 240 == 0:
                print(
                    f"t={final_t:.3f}s  "
                    f"base_z={final_base_z:.3f}  "
                    f"contact_slip={1000.0 * max_contact_slip:.2f}mm  "
                    f"swing_force={swing_metrics['normal_force']:.1f}N  "
                    f"friction_ratio={support_metrics['friction_ratio']:.3f}"
                )

            if not np.isfinite(robot.data.qpos).all() or final_base_z < 0.45:
                break
    finally:
        (
            wbc_module.Kp_c,
            wbc_module.Kd_c,
            wbc_module.Kp_L,
            wbc_module.Kd_L,
        ) = original_gains

    success = (
        final_t >= config.DIRECT_SINGLE_SUPPORT_DURATION
        and max_contact_slip <= config.DIRECT_SINGLE_SUPPORT_MAX_CONTACT_SLIP
        and max_swing_force <= config.DIRECT_SINGLE_SUPPORT_MAX_SWING_FORCE
        and max_friction_ratio <= config.DIRECT_SINGLE_SUPPORT_MAX_FRICTION_RATIO
    )
    return DirectSingleSupportResult(
        survived_s=final_t,
        final_base_z=final_base_z,
        max_contact_slip_mm=1000.0 * max_contact_slip,
        max_body_slip_mm=1000.0 * max_body_slip,
        max_swing_force=max_swing_force,
        max_friction_ratio=max_friction_ratio,
        max_tangential_force=max_tangential_force,
        wbc_failures=wbc_failures,
        success=success,
    )


def main() -> None:
    result = run_direct_single_support()
    print("\n===== 结果汇总 =====")
    print(f"survived: {result.survived_s:.3f} s")
    print(f"final base z: {result.final_base_z:.3f} m")
    print(f"max contact slip: {result.max_contact_slip_mm:.2f} mm")
    print(f"max body slip: {result.max_body_slip_mm:.2f} mm")
    print(f"max swing force: {result.max_swing_force:.2f} N")
    print(f"max tangential force: {result.max_tangential_force:.2f} N")
    print(f"max friction ratio: {result.max_friction_ratio:.3f}")
    print(f"WBC failures: {result.wbc_failures}")
    print(f"success: {result.success}")


if __name__ == "__main__":
    main()
