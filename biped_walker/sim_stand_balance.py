"""
Stand-up and balance simulation for the simplified biped URDF.

The robot starts from a crouched double-support pose, stands up through a
smooth joint-space trajectory, and then balances in place with simple posture
feedback around the upright stance.
"""

import argparse
import os
import matplotlib.pyplot as plt
import numpy as np

try:
    import pybullet as p
    import pybullet_data
except ImportError as e:
    raise ImportError("PyBullet not installed. Run: uv add pybullet") from e

from biped_urdf import create_biped_urdf


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.join(SCRIPT_DIR, "biped.urdf")
OUTPUT_PNG = os.path.join(SCRIPT_DIR, "standup_balance_results.png")

SIM_DT = 1.0 / 240.0
ANKLE_Z_OFFSET = 0.02
HIP_Y_OFFSET = 0.10
FOOT_LENGTH = 0.20
FOOT_WIDTH = 0.10
FOOT_COG_X = 0.03

SETTLE_TIME = 0.8
STANDUP_TIME = 2.5
BALANCE_TIME = 4.0

PELVIS_CROUCH = np.array([0.01, 0.0, 0.64])
PELVIS_STAND = np.array([0.03, 0.0, 0.78])
LEFT_FOOT_WORLD = np.array([0.0, HIP_Y_OFFSET, ANKLE_Z_OFFSET])
RIGHT_FOOT_WORLD = np.array([0.0, -HIP_Y_OFFSET, ANKLE_Z_OFFSET])

MOTOR_FORCE = 300.0
MOTOR_POSITION_GAIN = 0.4
MOTOR_VELOCITY_GAIN = 1.0

def smoothstep(alpha: float) -> float:
    alpha = np.clip(alpha, 0.0, 1.0)
    return alpha * alpha * (3.0 - 2.0 * alpha)


def ensure_biped_urdf() -> str:
    if os.path.exists(URDF_PATH):
        return URDF_PATH
    return create_biped_urdf(URDF_PATH)


def find_joint_index(client_id: int, robot_id: int, joint_name: str) -> int:
    for joint_index in range(p.getNumJoints(robot_id, physicsClientId=client_id)):
        name = p.getJointInfo(robot_id, joint_index, physicsClientId=client_id)[1].decode("utf-8")
        if name == joint_name:
            return joint_index
    raise ValueError(f"Joint not found: {joint_name}")


def get_joint_limits(client_id: int, robot_id: int):
    lower_limits = {}
    upper_limits = {}
    for joint_index in range(p.getNumJoints(robot_id, physicsClientId=client_id)):
        info = p.getJointInfo(robot_id, joint_index, physicsClientId=client_id)
        lower_limits[joint_index] = info[8]
        upper_limits[joint_index] = info[9]
    return lower_limits, upper_limits


def compute_zmp_from_contacts(client_id: int, robot_id: int, ground_id: int):
    contacts = p.getContactPoints(
        bodyA=robot_id,
        bodyB=ground_id,
        physicsClientId=client_id,
    )
    zmp_x = 0.0
    zmp_y = 0.0
    fz_total = 0.0

    for contact in contacts:
        px, py, _ = contact[5]
        normal_force = contact[9]
        zmp_x += px * normal_force
        zmp_y += py * normal_force
        fz_total += normal_force

    if fz_total <= 1e-6:
        return np.nan, np.nan, 0.0

    return zmp_x / fz_total, zmp_y / fz_total, fz_total


def desired_pelvis_position(sim_time: float) -> np.ndarray:
    if sim_time <= SETTLE_TIME:
        return PELVIS_CROUCH.copy()
    if sim_time <= SETTLE_TIME + STANDUP_TIME:
        alpha = (sim_time - SETTLE_TIME) / STANDUP_TIME
        return PELVIS_CROUCH + smoothstep(alpha) * (PELVIS_STAND - PELVIS_CROUCH)
    return PELVIS_STAND.copy()


def apply_joint_targets(client_id: int, robot_id: int, joint_targets: dict[int, float]):
    for joint_index, target in joint_targets.items():
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=joint_index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=float(target),
            force=MOTOR_FORCE,
            positionGain=MOTOR_POSITION_GAIN,
            velocityGain=MOTOR_VELOCITY_GAIN,
            physicsClientId=client_id,
        )


def solve_pitch_leg_angles(pelvis_target: np.ndarray):
    thigh_len = 0.37
    shin_len = 0.37
    hip_z_offset = 0.05

    dx = LEFT_FOOT_WORLD[0] - pelvis_target[0]
    dz = LEFT_FOOT_WORLD[2] - (pelvis_target[2] - hip_z_offset)

    x = -dx
    z = -dz
    reach = np.hypot(x, z)
    min_reach = 0.08
    max_reach = thigh_len + shin_len - 1e-6
    reach = np.clip(reach, min_reach, max_reach)

    if np.hypot(x, z) > 1e-9:
        scale = reach / np.hypot(x, z)
        x *= scale
        z *= scale

    cos_knee = (x * x + z * z - thigh_len**2 - shin_len**2) / (2.0 * thigh_len * shin_len)
    cos_knee = np.clip(cos_knee, -1.0, 1.0)
    knee = -np.arccos(cos_knee)
    hip = np.arctan2(x, z) - np.arctan2(shin_len * np.sin(knee), thigh_len + shin_len * np.cos(knee))
    ankle = -(hip + knee)
    return float(hip), float(knee), float(ankle)


def build_joint_targets(
    pelvis_target: np.ndarray,
    lower_limits: dict[int, float],
    upper_limits: dict[int, float],
):
    hip_pitch, knee, ankle = solve_pitch_leg_angles(pelvis_target)
    hip_roll = 0.0

    joint_targets = {
        0: hip_roll,
        1: hip_pitch,
        2: knee,
        3: ankle,
        4: -hip_roll,
        5: hip_pitch,
        6: knee,
        7: ankle,
    }

    for joint_index, target in joint_targets.items():
        joint_targets[joint_index] = float(np.clip(target, lower_limits[joint_index], upper_limits[joint_index]))

    return joint_targets


def plot_results(
    t_axis: np.ndarray,
    desired_pelvis_log: np.ndarray,
    actual_pelvis_log: np.ndarray,
    zmp_log: np.ndarray,
    force_log: np.ndarray,
    tilt_log: np.ndarray,
):
    fig, axes = plt.subplots(2, 2, figsize=(12, 9))

    ax = axes[0, 0]
    ax.plot(t_axis, desired_pelvis_log[:, 2], label="Desired pelvis Z")
    ax.plot(t_axis, actual_pelvis_log[:, 2], "--", label="Actual pelvis Z")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Height [m]")
    ax.set_title("Stand-up Height")
    ax.grid(True)
    ax.legend()

    ax = axes[0, 1]
    ax.plot(desired_pelvis_log[:, 0], desired_pelvis_log[:, 1], label="Desired pelvis")
    ax.plot(actual_pelvis_log[:, 0], actual_pelvis_log[:, 1], "--", label="Actual pelvis")
    ax.plot(zmp_log[:, 0], zmp_log[:, 1], ":", label="ZMP")

    for foot_pos, color, label in (
        (LEFT_FOOT_WORLD, "tab:green", "Left foot"),
        (RIGHT_FOOT_WORLD, "tab:red", "Right foot"),
    ):
        rect_x = [
            foot_pos[0] + FOOT_COG_X - FOOT_LENGTH / 2.0,
            foot_pos[0] + FOOT_COG_X + FOOT_LENGTH / 2.0,
            foot_pos[0] + FOOT_COG_X + FOOT_LENGTH / 2.0,
            foot_pos[0] + FOOT_COG_X - FOOT_LENGTH / 2.0,
            foot_pos[0] + FOOT_COG_X - FOOT_LENGTH / 2.0,
        ]
        rect_y = [
            foot_pos[1] - FOOT_WIDTH / 2.0,
            foot_pos[1] - FOOT_WIDTH / 2.0,
            foot_pos[1] + FOOT_WIDTH / 2.0,
            foot_pos[1] + FOOT_WIDTH / 2.0,
            foot_pos[1] - FOOT_WIDTH / 2.0,
        ]
        ax.plot(rect_x, rect_y, color=color, alpha=0.6, label=label)

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title("Balance in Support Area")
    ax.grid(True)
    ax.legend()
    ax.set_aspect("equal")

    ax = axes[1, 0]
    ax.plot(t_axis, np.rad2deg(tilt_log[:, 0]), label="Roll")
    ax.plot(t_axis, np.rad2deg(tilt_log[:, 1]), label="Pitch")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Angle [deg]")
    ax.set_title("Base Tilt")
    ax.grid(True)
    ax.legend()

    ax = axes[1, 1]
    ax.plot(t_axis, force_log, label="Vertical contact force")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Force [N]")
    ax.set_title("Ground Reaction Force")
    ax.grid(True)
    ax.legend()

    plt.tight_layout()
    plt.savefig(OUTPUT_PNG, dpi=150)
    print(f"[INFO] Plot saved to: {OUTPUT_PNG}")


def main():
    parser = argparse.ArgumentParser(description="Simulate stand-up and balance for the biped.")
    parser.add_argument("--gui", action="store_true", help="Show the PyBullet GUI.")
    args = parser.parse_args()

    urdf_path = ensure_biped_urdf()
    print(f"[INFO] Using URDF: {urdf_path}")

    sim_client = p.connect(p.GUI if args.gui else p.DIRECT)

    try:
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=sim_client)

        p.setGravity(0, 0, -9.81, physicsClientId=sim_client)
        p.setPhysicsEngineParameter(
            fixedTimeStep=SIM_DT,
            numSolverIterations=100,
            physicsClientId=sim_client,
        )

        ground_id = p.loadURDF("plane.urdf", physicsClientId=sim_client)
        robot_id = p.loadURDF(
            urdf_path,
            basePosition=PELVIS_CROUCH.tolist(),
            baseOrientation=p.getQuaternionFromEuler([0.0, 0.0, 0.0]),
            useFixedBase=False,
            physicsClientId=sim_client,
        )
        left_foot_link = find_joint_index(sim_client, robot_id, "left_ankle")
        right_foot_link = find_joint_index(sim_client, robot_id, "right_ankle")
        lower_limits, upper_limits = get_joint_limits(sim_client, robot_id)

        p.changeDynamics(ground_id, -1, lateralFriction=1.0, physicsClientId=sim_client)
        p.changeDynamics(robot_id, left_foot_link, lateralFriction=1.2, physicsClientId=sim_client)
        p.changeDynamics(robot_id, right_foot_link, lateralFriction=1.2, physicsClientId=sim_client)

        initial_targets = build_joint_targets(
            PELVIS_CROUCH,
            lower_limits=lower_limits,
            upper_limits=upper_limits,
        )

        for joint_index, target in initial_targets.items():
            p.resetJointState(robot_id, joint_index, targetValue=float(target), physicsClientId=sim_client)

        total_time = SETTLE_TIME + STANDUP_TIME + BALANCE_TIME
        n_steps = int(total_time / SIM_DT) + 1
        t_axis = np.arange(n_steps) * SIM_DT
        desired_pelvis_log = np.zeros((n_steps, 3))
        actual_pelvis_log = np.zeros((n_steps, 3))
        zmp_log = np.full((n_steps, 2), np.nan)
        force_log = np.zeros(n_steps)
        tilt_log = np.zeros((n_steps, 2))

        print("[INFO] Running stand-up and balance simulation...")
        for step, sim_time in enumerate(t_axis):
            pelvis_target = desired_pelvis_position(sim_time)

            joint_targets = build_joint_targets(
                pelvis_target,
                lower_limits=lower_limits,
                upper_limits=upper_limits,
            )
            apply_joint_targets(sim_client, robot_id, joint_targets)
            p.stepSimulation(physicsClientId=sim_client)

            base_position, base_orientation = p.getBasePositionAndOrientation(
                robot_id, physicsClientId=sim_client
            )
            roll, pitch, _ = p.getEulerFromQuaternion(base_orientation)
            zmp_x, zmp_y, vertical_force = compute_zmp_from_contacts(sim_client, robot_id, ground_id)

            desired_pelvis_log[step] = pelvis_target
            actual_pelvis_log[step] = np.asarray(base_position)
            zmp_log[step] = [zmp_x, zmp_y]
            force_log[step] = vertical_force
            tilt_log[step] = [roll, pitch]

            if base_position[2] < 0.35:
                print(f"[WARN] Robot lost balance at t={sim_time:.2f}s")
                desired_pelvis_log = desired_pelvis_log[: step + 1]
                actual_pelvis_log = actual_pelvis_log[: step + 1]
                zmp_log = zmp_log[: step + 1]
                force_log = force_log[: step + 1]
                tilt_log = tilt_log[: step + 1]
                t_axis = t_axis[: step + 1]
                break

        plot_results(t_axis, desired_pelvis_log, actual_pelvis_log, zmp_log, force_log, tilt_log)

        valid_zmp = ~np.isnan(zmp_log[:, 0])
        support_center = np.array([FOOT_COG_X, 0.0])
        print(
            f"[SUMMARY] Final pelvis height: {actual_pelvis_log[-1, 2]:.3f} m | "
            f"max |roll|={np.max(np.abs(np.rad2deg(tilt_log[:, 0]))):.2f} deg | "
            f"max |pitch|={np.max(np.abs(np.rad2deg(tilt_log[:, 1]))):.2f} deg"
        )
        if np.any(valid_zmp):
            zmp_rms = np.sqrt(np.mean(np.sum((zmp_log[valid_zmp] - support_center) ** 2, axis=1)))
            print(f"[SUMMARY] ZMP RMS distance to mid-foot center: {zmp_rms * 1000:.1f} mm")
    finally:
        p.disconnect(sim_client)


if __name__ == "__main__":
    main()
