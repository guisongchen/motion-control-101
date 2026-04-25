"""Direct single-support WBC benchmark runner."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np

from direct_single_support.config import DIRECT_SINGLE_SUPPORT_CONFIG as cfg
from direct_single_support.primitives import (
    build_corner_patch_wrench_task,
    compute_corner_patch_force_reference,
    compute_corner_patch_wrench_force_reference,
    resolve_support_contact_local_positions,
    resolve_support_cop_target_world,
    apply_measured_cop_feedback,
    wrap_to_pi,
    yaw_from_rotation,
)
import wbc as wbc_module
from utils import compute_pd_torque
from robot_model import RobotModel
from robots.unitree_g1 import g1_config
from state_estimator import StateEstimator
from wbc import WholeBodyController


@dataclass
class DirectSingleSupportResult:
    survived_s: float
    final_base_z: float
    max_contact_slip_mm: float
    max_body_slip_mm: float
    max_cop_error_mm: float
    max_swing_force: float
    max_friction_ratio: float
    max_tangential_force: float
    wbc_failures: int
    success: bool


def _load_optimized_pose() -> dict:
    """Load the simulation-optimized initial pose from results JSON."""
    result_path = Path(__file__).parent.parent / "results" / "optimized_pose_simulation_3s.json"
    with result_path.open("r") as f:
        data = json.load(f)
    return {
        "base_position": np.array(data["base_position"], dtype=float),
        "base_orientation": np.array(data["base_orientation"], dtype=float),
        "joint_angles": np.array(data["joint_angles"], dtype=float),
    }


def run_direct_single_support(
    pose_override: dict | None = None,
    quiet: bool = False,
    duration_override: float | None = None,
) -> DirectSingleSupportResult:
    """Run the direct single-support WBC benchmark with tuned current-best settings."""
    env_cfg = cfg.env
    pose_cfg = cfg.pose
    control_cfg = cfg.control
    contact_cfg = cfg.contact
    cop_cfg = cfg.cop
    wrench_cfg = cfg.wrench
    success_cfg = cfg.success

    support_leg = "right" if env_cfg.support_foot_name.startswith("right") else "left"
    swing_leg = "left" if support_leg == "right" else "right"
    support_sign = 1.0 if support_leg == "right" else -1.0

    robot = RobotModel(env_cfg.model_path)
    robot.model.opt.gravity[:] = env_cfg.gravity
    robot.model.opt.timestep = env_cfg.dt
    robot.model.dof_damping[:6] = env_cfg.base_damping * pose_cfg.damping_scale
    robot.model.dof_damping[6:] = env_cfg.joint_damping * pose_cfg.damping_scale

    if pose_override is None:
        pose_override = _load_optimized_pose()
    base_pos = np.asarray(pose_override["base_position"], dtype=float)
    base_orn = np.asarray(pose_override["base_orientation"], dtype=float)
    robot.reset_base_pose(base_pos, base_orn)
    q_target = np.asarray(pose_override["joint_angles"], dtype=float)
    robot.reset_joint_positions(q_target)

    candidate_foot_links = [robot.link_name_to_index[name] for name in env_cfg.foot_link_names]
    support_link = robot.link_name_to_index[env_cfg.support_foot_name]
    swing_link = next(link for link in candidate_foot_links if link != support_link)
    estimator = StateEstimator(robot)
    original_gains = (
        wbc_module.Kp_c,
        wbc_module.Kd_c,
        wbc_module.Kp_L,
        wbc_module.Kd_L,
    )
    wbc_module.Kp_c = control_cfg.com_kp
    wbc_module.Kd_c = control_cfg.com_kd
    wbc_module.Kp_L = control_cfg.momentum_kp
    wbc_module.Kd_L = control_cfg.momentum_kd

    initial_support_contact = robot.get_contact_metrics(support_link)["position"].copy()
    initial_support_body = robot.get_link_com_position(support_link).copy()
    support_contact_local_positions = resolve_support_contact_local_positions(
        robot,
        support_link,
        initial_support_contact,
    )
    controller = WholeBodyController(robot.nv, num_contacts=len(support_contact_local_positions))

    swing_leg_dof_indices = [
        robot.dof_joint_name_to_index[name]
        for name in g1_config.leg_joint_names[swing_leg]
        if name in robot.dof_joint_name_to_index
    ]

    state0 = estimator.update(preferred_support_foot_link=support_link, lock_support=True)
    c_ref = state0["c"].copy()
    support_total_normal_force = -env_cfg.gravity[2] * robot.total_mass
    initial_support_metrics = robot.get_contact_metrics(support_link)
    filtered_cop_world = np.array(initial_support_metrics["cop_position"], copy=True)
    prev_filtered_cop_world = filtered_cop_world.copy()
    filtered_support_position_world = np.array(initial_support_metrics["position"], copy=True)
    prev_filtered_support_position_world = filtered_support_position_world.copy()
    initial_support_yaw = yaw_from_rotation(np.array(robot.data.xmat[support_link]).reshape(3, 3))

    duration = duration_override if duration_override is not None else control_cfg.duration
    total_steps = int(duration / env_cfg.dt)
    wbc_failures = 0

    max_contact_slip = 0.0
    max_body_slip = 0.0
    max_cop_error = 0.0
    max_swing_force = 0.0
    max_friction_ratio = 0.0
    max_tangential_force = 0.0
    final_t = 0.0
    final_base_z = float(robot.data.qpos[2])

    if not quiet:
        print("\n===== 直接单足支撑 WBC 基准测试 =====")
        print(f"support foot: {env_cfg.support_foot_name}")
        print(f"duration: {control_cfg.duration:.1f} s")
        print(f"damping scale: {pose_cfg.damping_scale:.2f}")
        print(f"support blend: {control_cfg.support_blend:.3f}")
        print(f"contact point: {contact_cfg.wbc_contact_point}")
        print(f"CoP target: {contact_cfg.cop_target}")
        print(f"CoP feedback: {cop_cfg.enabled}")
        if cop_cfg.enabled:
            print(
                "CoP feedback gains: "
                f"alpha={cop_cfg.filter_alpha:.3f}, "
                f"kp={cop_cfg.kp:.3f}, "
                f"kd={cop_cfg.kd:.3f}, "
                f"max_offset={1000.0 * cop_cfg.max_offset:.1f}mm"
            )
        print(f"wrench objective: {control_cfg.use_wrench_objective}")
        print(
            "wrench regulation: "
            f"force_xy_w={wrench_cfg.force_xy_weight:.3f}, "
            f"moment_w={wrench_cfg.moment_weight:.3f}, "
            f"yaw_w={wrench_cfg.yaw_moment_weight:.3f}"
        )
        print("=" * 50)

    try:
        for step in range(total_steps):
            state = estimator.update(preferred_support_foot_link=support_link, lock_support=True)
            q = state["q"]
            v = state["v"]

            C = robot.compute_coriolis_gravity(q, v)
            safe_tau = C[6:] + compute_pd_torque(
                q_target,
                q[7:],
                v[6:],
                control_cfg.posture_kp,
                control_cfg.posture_kd,
                robot.tau_limits,
            )

            M = robot.compute_mass_matrix(q)
            J_c = np.vstack(
                [
                    robot.get_foot_jacobian(
                        support_link,
                        q,
                        local_position=local_position,
                    )
                    for local_position in support_contact_local_positions
                ]
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
            foot_origin = np.array(robot.data.xpos[support_link], copy=True)
            foot_rotation = np.array(robot.data.xmat[support_link]).reshape(3, 3)
            support_metrics_pre = robot.get_contact_metrics(support_link)
            if support_metrics_pre["normal_force"] > 1e-6:
                measured_cop_world = np.array(support_metrics_pre["cop_position"], copy=True)
                filtered_cop_world = (
                    cop_cfg.filter_alpha * measured_cop_world
                    + (1.0 - cop_cfg.filter_alpha) * filtered_cop_world
                )
                measured_support_position_world = np.array(support_metrics_pre["position"], copy=True)
                filtered_support_position_world = (
                    wrench_cfg.state_filter_alpha * measured_support_position_world
                    + (1.0 - wrench_cfg.state_filter_alpha) * filtered_support_position_world
                )
            measured_cop_velocity_world = (
                filtered_cop_world - prev_filtered_cop_world
            ) / env_cfg.dt
            prev_filtered_cop_world = filtered_cop_world.copy()
            measured_support_slip_velocity_world = (
                filtered_support_position_world - prev_filtered_support_position_world
            ) / env_cfg.dt
            prev_filtered_support_position_world = filtered_support_position_world.copy()
            nominal_cop_target_world = resolve_support_cop_target_world(
                state,
                c_ref,
                initial_support_contact,
            )
            cop_target_world = nominal_cop_target_world
            if cop_cfg.enabled:
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
                wrench_cfg.slip_force_kp * support_slip_world[:2]
                + wrench_cfg.slip_force_kd * measured_support_slip_velocity_world[:2]
            )
            desired_force_xy_world = np.clip(
                desired_force_xy_world,
                -wrench_cfg.slip_force_max,
                wrench_cfg.slip_force_max,
            )
            _, support_ang_vel = robot.get_link_velocity(support_link)
            support_yaw_error = wrap_to_pi(yaw_from_rotation(foot_rotation) - initial_support_yaw)
            desired_yaw_moment = -(
                wrench_cfg.yaw_moment_kp * support_yaw_error
                + wrench_cfg.yaw_moment_kd * support_ang_vel[2]
            )
            desired_yaw_moment = float(
                np.clip(
                    desired_yaw_moment,
                    -wrench_cfg.yaw_moment_max,
                    wrench_cfg.yaw_moment_max,
                )
            )
            if control_cfg.use_wrench_objective:
                force_task_matrix, force_task_ref, force_task_weight = build_corner_patch_wrench_task(
                    support_contact_local_positions,
                    foot_origin,
                    foot_rotation,
                    cop_target_world,
                    support_total_normal_force,
                    desired_force_xy_world=desired_force_xy_world,
                    desired_yaw_moment=desired_yaw_moment,
                )
                f_ref = compute_corner_patch_wrench_force_reference(
                    force_task_matrix,
                    force_task_ref,
                )
            else:
                f_ref = compute_corner_patch_force_reference(
                    support_contact_local_positions,
                    foot_origin,
                    foot_rotation,
                    cop_target_world,
                    support_total_normal_force,
                )
                force_task_matrix = None
                force_task_ref = None
                force_task_weight = None
            wbc_result = controller.solve(
                M,
                C,
                J_c,
                np.zeros_like(J_c),
                J_com,
                J_L,
                c_ddot_des,
                L_dot_des,
                f_ref,
                v,
                -robot.tau_limits,
                robot.tau_limits,
                force_task_matrix=force_task_matrix,
                force_task_ref=force_task_ref,
                force_task_weight=force_task_weight,
            )
            if wbc_result is None:
                wbc_failures += 1

            tau_cmd = safe_tau.copy()
            if wbc_result is not None:
                support_mask = np.ones(robot.num_joints, dtype=bool)
                support_mask[swing_leg_dof_indices] = False
                tau_cmd[support_mask] = (
                    (1.0 - control_cfg.support_blend) * safe_tau[support_mask]
                    + control_cfg.support_blend
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
            max_cop_error = max(
                max_cop_error,
                float(np.linalg.norm(support_metrics["cop_position"][:2] - cop_target_world[:2])),
            )
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

            final_t = (step + 1) * env_cfg.dt
            final_base_z = float(robot.data.qpos[2])

            if not quiet and (step + 1) % 240 == 0:
                print(
                    f"t={final_t:.3f}s  "
                    f"base_z={final_base_z:.3f}  "
                    f"contact_slip={1000.0 * max_contact_slip:.2f}mm  "
                    f"cop_error={1000.0 * max_cop_error:.2f}mm  "
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
        final_t >= duration - 1e-6
        and max_contact_slip <= success_cfg.max_contact_slip
        and max_swing_force <= success_cfg.max_swing_force
        and max_friction_ratio <= success_cfg.max_friction_ratio
    )
    return DirectSingleSupportResult(
        survived_s=final_t,
        final_base_z=final_base_z,
        max_contact_slip_mm=1000.0 * max_contact_slip,
        max_body_slip_mm=1000.0 * max_body_slip,
        max_cop_error_mm=1000.0 * max_cop_error,
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
    print(f"max CoP error: {result.max_cop_error_mm:.2f} mm")
    print(f"max swing force: {result.max_swing_force:.2f} N")
    print(f"max tangential force: {result.max_tangential_force:.2f} N")
    print(f"max friction ratio: {result.max_friction_ratio:.3f}")
    print(f"WBC failures: {result.wbc_failures}")
    print(f"success: {result.success}")


if __name__ == "__main__":
    main()
