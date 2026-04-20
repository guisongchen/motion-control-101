"""Direct-spawn single-support WBC benchmark focused on support-foot slip."""

from dataclasses import dataclass
import math

import mujoco
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
    max_cop_error_mm: float
    max_swing_force: float
    max_friction_ratio: float
    max_tangential_force: float
    wbc_failures: int
    success: bool


def quat_from_roll(roll: float) -> np.ndarray:
    """Build an xyzw quaternion from a pure roll angle."""
    return np.array([math.sin(roll / 2.0), 0.0, 0.0, math.cos(roll / 2.0)], dtype=float)


def wrap_to_pi(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_rotation(rotation: np.ndarray) -> float:
    """Extract world yaw from a rotation matrix."""
    return math.atan2(rotation[1, 0], rotation[0, 0])


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


def resolve_support_contact_local_positions(
    robot: RobotModel,
    support_link: int,
    initial_support_contact: np.ndarray,
) -> list[np.ndarray | None]:
    """Pick the support-foot points whose accelerations WBC constrains to zero."""
    mode = config.DIRECT_SINGLE_SUPPORT_WBC_CONTACT_POINT
    if mode == "body_origin":
        return [None]
    if mode == "initial_contact":
        body_origin = np.array(robot.data.xpos[support_link], copy=True)
        body_rotation = np.array(robot.data.xmat[support_link]).reshape(3, 3)
        return [body_rotation.T @ (initial_support_contact - body_origin)]
    if mode == "corner_patch":
        local_positions: list[np.ndarray] = []
        for geom_id in range(robot.model.ngeom):
            if int(robot.model.geom_bodyid[geom_id]) != support_link:
                continue
            if int(robot.model.geom_type[geom_id]) != mujoco.mjtGeom.mjGEOM_SPHERE:
                continue
            local_positions.append(np.array(robot.model.geom_pos[geom_id], copy=True))
        if not local_positions:
            raise ValueError(
                f"No corner-patch sphere geoms found on support body {support_link}."
            )
        return sorted(local_positions, key=lambda pos: (pos[0], pos[1]))
    raise ValueError(f"Unsupported direct single-support contact point mode: {mode}")


def compute_corner_patch_force_reference(
    local_positions: list[np.ndarray | None],
    foot_origin: np.ndarray,
    foot_rotation: np.ndarray,
    cop_world: np.ndarray,
    total_normal_force: float,
) -> np.ndarray:
    """Distribute normal load across the four sole corners to realize a desired CoP."""
    if len(local_positions) != 4 or any(pos is None for pos in local_positions):
        return np.tile(
            np.array([0.0, 0.0, total_normal_force / len(local_positions)], dtype=float),
            len(local_positions),
        )

    corner_positions = [np.asarray(pos, dtype=float) for pos in local_positions]
    cop_local = foot_rotation.T @ (cop_world - foot_origin)
    x_coords = np.array([pos[0] for pos in corner_positions])
    y_coords = np.array([pos[1] for pos in corner_positions])
    x_min = float(np.min(x_coords)) + config.DIRECT_SINGLE_SUPPORT_COP_MARGIN
    x_max = float(np.max(x_coords)) - config.DIRECT_SINGLE_SUPPORT_COP_MARGIN
    y_min = float(np.min(y_coords)) + config.DIRECT_SINGLE_SUPPORT_COP_MARGIN
    y_max = float(np.max(y_coords)) - config.DIRECT_SINGLE_SUPPORT_COP_MARGIN

    target_x = float(np.clip(cop_local[0], x_min, x_max))
    target_y = float(np.clip(cop_local[1], y_min, y_max))
    x_alpha = (target_x - x_min) / max(x_max - x_min, 1e-6)
    y_alpha = (target_y - y_min) / max(y_max - y_min, 1e-6)
    weights = np.array(
        [
            (1.0 - x_alpha) * (1.0 - y_alpha),
            (1.0 - x_alpha) * y_alpha,
            x_alpha * (1.0 - y_alpha),
            x_alpha * y_alpha,
        ],
        dtype=float,
    )
    weights /= np.sum(weights)

    f_ref = np.zeros(3 * len(corner_positions), dtype=float)
    for contact_idx, weight in enumerate(weights):
        f_ref[3 * contact_idx + 2] = weight * total_normal_force
    return f_ref


def clip_corner_patch_cop_target(
    local_positions: list[np.ndarray | None],
    foot_origin: np.ndarray,
    foot_rotation: np.ndarray,
    cop_world: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Project a desired CoP into the interior of the corner patch."""
    if len(local_positions) != 4 or any(pos is None for pos in local_positions):
        return np.asarray(cop_world, dtype=float), np.zeros(3)

    corner_positions = [np.asarray(pos, dtype=float) for pos in local_positions]
    cop_local = foot_rotation.T @ (cop_world - foot_origin)
    x_coords = np.array([pos[0] for pos in corner_positions])
    y_coords = np.array([pos[1] for pos in corner_positions])
    x_min = float(np.min(x_coords)) + config.DIRECT_SINGLE_SUPPORT_COP_MARGIN
    x_max = float(np.max(x_coords)) - config.DIRECT_SINGLE_SUPPORT_COP_MARGIN
    y_min = float(np.min(y_coords)) + config.DIRECT_SINGLE_SUPPORT_COP_MARGIN
    y_max = float(np.max(y_coords)) - config.DIRECT_SINGLE_SUPPORT_COP_MARGIN

    clipped_local = cop_local.copy()
    clipped_local[0] = np.clip(clipped_local[0], x_min, x_max)
    clipped_local[1] = np.clip(clipped_local[1], y_min, y_max)
    clipped_world = foot_origin + foot_rotation @ clipped_local
    return clipped_world, clipped_local


def apply_measured_cop_feedback(
    local_positions: list[np.ndarray | None],
    foot_origin: np.ndarray,
    foot_rotation: np.ndarray,
    nominal_cop_world: np.ndarray,
    measured_cop_world: np.ndarray,
    measured_cop_velocity_world: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Shift the desired CoP using measured CoP error while staying inside the patch."""
    clipped_nominal_world, nominal_local = clip_corner_patch_cop_target(
        local_positions,
        foot_origin,
        foot_rotation,
        nominal_cop_world,
    )
    if len(local_positions) != 4 or any(pos is None for pos in local_positions):
        return clipped_nominal_world, nominal_local

    _, measured_local = clip_corner_patch_cop_target(
        local_positions,
        foot_origin,
        foot_rotation,
        measured_cop_world,
    )
    measured_velocity_local = foot_rotation.T @ measured_cop_velocity_world
    local_correction = np.zeros(3, dtype=float)
    local_correction[:2] = (
        config.DIRECT_SINGLE_SUPPORT_COP_FEEDBACK_KP
        * (nominal_local[:2] - measured_local[:2])
        - config.DIRECT_SINGLE_SUPPORT_COP_FEEDBACK_KD * measured_velocity_local[:2]
    )
    local_correction[:2] = np.clip(
        local_correction[:2],
        -config.DIRECT_SINGLE_SUPPORT_COP_FEEDBACK_MAX_OFFSET,
        config.DIRECT_SINGLE_SUPPORT_COP_FEEDBACK_MAX_OFFSET,
    )
    corrected_local = nominal_local + local_correction
    corrected_world, corrected_local = clip_corner_patch_cop_target(
        local_positions,
        foot_origin,
        foot_rotation,
        foot_origin + foot_rotation @ corrected_local,
    )
    return corrected_world, corrected_local


def build_corner_patch_wrench_task(
    local_positions: list[np.ndarray | None],
    foot_origin: np.ndarray,
    foot_rotation: np.ndarray,
    cop_world: np.ndarray,
    total_normal_force: float,
    desired_force_xy_world: np.ndarray | None = None,
    desired_yaw_moment: float = 0.0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Build a support-wrench task for the 4-corner foot patch."""
    if len(local_positions) != 4 or any(pos is None for pos in local_positions):
        raise ValueError("Support wrench task requires four explicit corner contact points.")
    if desired_force_xy_world is None:
        desired_force_xy_world = np.zeros(2, dtype=float)

    clipped_world, _ = clip_corner_patch_cop_target(
        local_positions,
        foot_origin,
        foot_rotation,
        cop_world,
    )
    task_matrix = np.zeros((6, 3 * len(local_positions)), dtype=float)
    for contact_idx, local_position in enumerate(local_positions):
        world_position = foot_origin + foot_rotation @ np.asarray(local_position, dtype=float)
        r = world_position - foot_origin
        col = 3 * contact_idx
        task_matrix[0:3, col : col + 3] = np.eye(3)
        task_matrix[3, col : col + 3] = np.array([0.0, -r[2], r[1]])
        task_matrix[4, col : col + 3] = np.array([r[2], 0.0, -r[0]])
        task_matrix[5, col : col + 3] = np.array([-r[1], r[0], 0.0])

    cop_offset = clipped_world - foot_origin
    task_ref = np.array(
        [
            desired_force_xy_world[0],
            desired_force_xy_world[1],
            total_normal_force,
            cop_offset[1] * total_normal_force,
            -cop_offset[0] * total_normal_force,
            desired_yaw_moment,
        ],
        dtype=float,
    )
    task_weight = np.diag(
        [
            config.DIRECT_SINGLE_SUPPORT_WRENCH_FORCE_XY_WEIGHT,
            config.DIRECT_SINGLE_SUPPORT_WRENCH_FORCE_XY_WEIGHT,
            config.DIRECT_SINGLE_SUPPORT_WRENCH_FORCE_Z_WEIGHT,
            config.DIRECT_SINGLE_SUPPORT_WRENCH_MOMENT_WEIGHT,
            config.DIRECT_SINGLE_SUPPORT_WRENCH_MOMENT_WEIGHT,
            config.DIRECT_SINGLE_SUPPORT_WRENCH_YAW_MOMENT_WEIGHT,
        ]
    )
    return task_matrix, task_ref, task_weight


def compute_corner_patch_wrench_force_reference(
    task_matrix: np.ndarray,
    task_ref: np.ndarray,
) -> np.ndarray:
    """Project a desired aggregate wrench into a minimum-norm corner-force reference."""
    return np.linalg.pinv(task_matrix) @ task_ref


def resolve_support_cop_target_world(
    state: dict,
    c_ref: np.ndarray,
    initial_support_contact: np.ndarray,
) -> np.ndarray:
    """Choose the world-frame CoP target used to bias the patch force reference."""
    mode = config.DIRECT_SINGLE_SUPPORT_COP_TARGET
    if mode == "initial_contact":
        return initial_support_contact
    if mode == "support_com_ref":
        return c_ref
    if mode == "current_com":
        return state["c"]
    raise ValueError(f"Unsupported direct single-support CoP target mode: {mode}")


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
    support_contact_local_positions = resolve_support_contact_local_positions(
        robot,
        support_link,
        initial_support_contact,
    )
    controller = WholeBodyController(robot.nv, num_contacts=len(support_contact_local_positions))

    swing_leg_dof_indices = [
        robot.dof_joint_name_to_index[name]
        for name in get_leg_joint_names(swing_leg)
        if name in robot.dof_joint_name_to_index
    ]

    state0 = estimator.update(preferred_support_foot_link=support_link, lock_support=True)
    c_ref = state0["c"].copy()
    support_total_normal_force = -config.GRAVITY[2] * robot.total_mass
    initial_support_metrics = robot.get_contact_metrics(support_link)
    filtered_cop_world = np.array(initial_support_metrics["cop_position"], copy=True)
    prev_filtered_cop_world = filtered_cop_world.copy()
    filtered_support_position_world = np.array(initial_support_metrics["position"], copy=True)
    prev_filtered_support_position_world = filtered_support_position_world.copy()
    initial_support_yaw = yaw_from_rotation(np.array(robot.data.xmat[support_link]).reshape(3, 3))

    total_steps = int(config.DIRECT_SINGLE_SUPPORT_DURATION / config.DT_SIM)
    wbc_failures = 0
    max_contact_slip = 0.0
    max_body_slip = 0.0
    max_cop_error = 0.0
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
    print(f"CoP target: {config.DIRECT_SINGLE_SUPPORT_COP_TARGET}")
    print(f"CoP feedback: {config.DIRECT_SINGLE_SUPPORT_COP_FEEDBACK_ENABLED}")
    if config.DIRECT_SINGLE_SUPPORT_COP_FEEDBACK_ENABLED:
        print(
            "CoP feedback gains: "
            f"alpha={config.DIRECT_SINGLE_SUPPORT_COP_FEEDBACK_FILTER_ALPHA:.3f}, "
            f"kp={config.DIRECT_SINGLE_SUPPORT_COP_FEEDBACK_KP:.3f}, "
            f"kd={config.DIRECT_SINGLE_SUPPORT_COP_FEEDBACK_KD:.3f}, "
            f"max_offset={1000.0 * config.DIRECT_SINGLE_SUPPORT_COP_FEEDBACK_MAX_OFFSET:.1f}mm"
        )
    print(f"wrench objective: {config.DIRECT_SINGLE_SUPPORT_USE_WRENCH_OBJECTIVE}")
    print(
        "wrench regulation: "
        f"force_xy_w={config.DIRECT_SINGLE_SUPPORT_WRENCH_FORCE_XY_WEIGHT:.3f}, "
        f"moment_w={config.DIRECT_SINGLE_SUPPORT_WRENCH_MOMENT_WEIGHT:.3f}, "
        f"yaw_w={config.DIRECT_SINGLE_SUPPORT_WRENCH_YAW_MOMENT_WEIGHT:.3f}"
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
                config.DIRECT_SINGLE_SUPPORT_POSTURE_KP,
                config.DIRECT_SINGLE_SUPPORT_POSTURE_KD,
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
                    config.DIRECT_SINGLE_SUPPORT_COP_FEEDBACK_FILTER_ALPHA * measured_cop_world
                    + (1.0 - config.DIRECT_SINGLE_SUPPORT_COP_FEEDBACK_FILTER_ALPHA) * filtered_cop_world
                )
                measured_support_position_world = np.array(support_metrics_pre["position"], copy=True)
                filtered_support_position_world = (
                    config.DIRECT_SINGLE_SUPPORT_WRENCH_STATE_FILTER_ALPHA
                    * measured_support_position_world
                    + (1.0 - config.DIRECT_SINGLE_SUPPORT_WRENCH_STATE_FILTER_ALPHA)
                    * filtered_support_position_world
                )
            measured_cop_velocity_world = (
                filtered_cop_world - prev_filtered_cop_world
            ) / config.DT_SIM
            prev_filtered_cop_world = filtered_cop_world.copy()
            measured_support_slip_velocity_world = (
                filtered_support_position_world - prev_filtered_support_position_world
            ) / config.DT_SIM
            prev_filtered_support_position_world = filtered_support_position_world.copy()
            nominal_cop_target_world = resolve_support_cop_target_world(
                state,
                c_ref,
                initial_support_contact,
            )
            cop_target_world = nominal_cop_target_world
            if config.DIRECT_SINGLE_SUPPORT_COP_FEEDBACK_ENABLED:
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
                config.DIRECT_SINGLE_SUPPORT_WRENCH_SLIP_FORCE_KP * support_slip_world[:2]
                + config.DIRECT_SINGLE_SUPPORT_WRENCH_SLIP_FORCE_KD
                * measured_support_slip_velocity_world[:2]
            )
            desired_force_xy_world = np.clip(
                desired_force_xy_world,
                -config.DIRECT_SINGLE_SUPPORT_WRENCH_SLIP_FORCE_MAX,
                config.DIRECT_SINGLE_SUPPORT_WRENCH_SLIP_FORCE_MAX,
            )
            _, support_ang_vel = robot.get_link_velocity(support_link)
            support_yaw_error = wrap_to_pi(yaw_from_rotation(foot_rotation) - initial_support_yaw)
            desired_yaw_moment = -(
                config.DIRECT_SINGLE_SUPPORT_WRENCH_YAW_MOMENT_KP * support_yaw_error
                + config.DIRECT_SINGLE_SUPPORT_WRENCH_YAW_MOMENT_KD * support_ang_vel[2]
            )
            desired_yaw_moment = float(
                np.clip(
                    desired_yaw_moment,
                    -config.DIRECT_SINGLE_SUPPORT_WRENCH_YAW_MOMENT_MAX,
                    config.DIRECT_SINGLE_SUPPORT_WRENCH_YAW_MOMENT_MAX,
                )
            )
            if config.DIRECT_SINGLE_SUPPORT_USE_WRENCH_OBJECTIVE:
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

            final_t = (step + 1) * config.DT_SIM
            final_base_z = float(robot.data.qpos[2])

            if (step + 1) % 240 == 0:
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
