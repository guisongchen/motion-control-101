"""
Minimal PyBullet walking demo for simplified 3D biped.
Generates a URDF, builds a walking pattern with Preview Control,
and drives the robot kinematically in PyBullet while recording ZMP.
"""
import os
import time
import numpy as np
import matplotlib.pyplot as plt

# PyBullet imports (make sure it's installed)
try:
    import pybullet as p
    import pybullet_data
except ImportError as e:
    raise ImportError("PyBullet not installed. Run: pip install pybullet numpy matplotlib scipy") from e

from biped_urdf import create_biped_urdf
from pattern_generator import build_full_gait

# ======================= CONFIGURATION =======================
USE_GUI = True              # Set to True to open PyBullet GUI
NUM_STEPS = 6                # Walking steps to simulate
STEP_LENGTH = 0.25           # meters
STEP_WIDTH = 0.18            # meters
Z_C = 0.85                   # CoM height (matches URDF roughly)
T_SS = 0.6                   # single support duration [s]
T_DS = 0.2                   # double support duration [s]
SWING_H = 0.06               # max foot clearance [m]
SIM_DT = 1.0 / 240.0         # PyBullet simulation step
Q_WEIGHT = 1.0
R_WEIGHT = 1e-4
N_PREVIEW = 160

# Ankle is at top of foot box; foot_h/2 in URDF is 0.02 m.
ANKLE_Z_OFFSET = 0.02
# =============================================================


def compute_zmp_from_contacts(robot_id, ground_id):
    """
    Compute Zero Moment Point from all contact points between robot and ground.
    Returns (zmp_x, zmp_y) or (nan, nan) if no contacts.
    """
    contacts = p.getContactPoints(bodyA=robot_id, bodyB=ground_id)
    zmp_x = 0.0
    zmp_y = 0.0
    fz_total = 0.0
    for cp in contacts:
        # cp layout from PyBullet Quickstart Guide:
        # [5,6,7] = positionOnA (x,y,z) – contact point on robot link
        # [15]    = normalForce
        px, py = cp[5], cp[6]
        fz = cp[15]
        zmp_x += px * fz
        zmp_y += py * fz
        fz_total += fz
    if fz_total > 1e-6:
        zmp_x /= fz_total
        zmp_y /= fz_total
    else:
        zmp_x = np.nan
        zmp_y = np.nan
    return zmp_x, zmp_y, fz_total


def main():
    # 1) Generate URDF on the fly
    urdf_path = create_biped_urdf("biped.urdf")
    print(f"[INFO] Generated URDF: {urdf_path}")

    # 2) Build walking pattern
    print("[INFO] Building walking pattern...")
    traj = build_full_gait(
        num_steps=NUM_STEPS,
        step_length=STEP_LENGTH,
        step_width=STEP_WIDTH,
        zc=Z_C,
        T_ss=T_SS,
        T_ds=T_DS,
        H=SWING_H,
        dt=SIM_DT,
        Q=Q_WEIGHT,
        R=R_WEIGHT,
        N_preview=N_PREVIEW,
    )
    t_axis = traj["t"]
    com_ref = traj["com"]
    zmp_ref = traj["zmp"]
    left_foot_ref = traj["left_foot"].copy()
    right_foot_ref = traj["right_foot"].copy()
    # Shift foot reference from ground-contact level to ankle-joint level
    left_foot_ref[:, 2] += ANKLE_Z_OFFSET
    right_foot_ref[:, 2] += ANKLE_Z_OFFSET

    n_samples = len(t_axis)
    print(f"[INFO] Trajectory samples: {n_samples}, duration: {t_axis[-1]:.2f} s")

    # 3) Start PyBullet
    mode = p.GUI if USE_GUI else p.DIRECT
    physics_client = p.connect(mode)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    ground_id = p.loadURDF("plane.urdf")
    start_pos = [0.0, 0.0, 1.0]
    start_orn = p.getQuaternionFromEuler([0.0, 0.0, 0.0])
    robot_id = p.loadURDF(urdf_path, start_pos, start_orn)

    n_joints = p.getNumJoints(robot_id)
    print(f"[INFO] Robot loaded. Joints: {n_joints}")

    # Locate foot link indices
    left_foot_link = None
    right_foot_link = None
    for i in range(n_joints):
        name = p.getJointInfo(robot_id, i)[1].decode("utf-8")
        if name == "left_ankle":
            left_foot_link = i
        elif name == "right_ankle":
            right_foot_link = i
    assert left_foot_link is not None and right_foot_link is not None, "Foot links not found"

    # Set up IK limits
    lower_limits = []
    upper_limits = []
    joint_ranges = []
    rest_poses = []
    for i in range(n_joints):
        info = p.getJointInfo(robot_id, i)
        lower_limits.append(info[8])
        upper_limits.append(info[9])
        joint_ranges.append(info[9] - info[8])
        rest_poses.append(0.0)
    damping = [0.1] * n_joints

    # Log arrays
    com_actual = np.zeros((n_samples, 3))
    zmp_actual = np.full((n_samples, 2), np.nan)
    fz_log = np.zeros(n_samples)

    # 4) Simulation loop (kinematic driving)
    print("[INFO] Running simulation...")
    prev_joint_angles = tuple(rest_poses)
    for k in range(n_samples):
        # Desired pelvis pose = CoM reference, upright orientation
        pelvis_pos = com_ref[k].tolist()
        pelvis_orn = [0.0, 0.0, 0.0, 1.0]
        p.resetBasePositionAndOrientation(robot_id, pelvis_pos, pelvis_orn)

        # Desired foot positions (ankle joints)
        target_positions = [left_foot_ref[k].tolist(), right_foot_ref[k].tolist()]
        # Flat foot orientation (identity quaternion = foot horizontal)
        flat_foot = [0.0, 0.0, 0.0, 1.0]
        target_orientations = [flat_foot, flat_foot]

        # Compute IK for both feet simultaneously
        try:
            joint_angles = p.calculateInverseKinematics2(
                bodyUniqueId=robot_id,
                endEffectorLinkIndices=[left_foot_link, right_foot_link],
                targetPositions=target_positions,
                lowerLimits=lower_limits,
                upperLimits=upper_limits,
                jointRanges=joint_ranges,
                restPoses=list(prev_joint_angles),
                jointDamping=damping,
                maxNumIterations=100,
                residualThreshold=1e-5,
            )
            prev_joint_angles = joint_angles
        except Exception as e:
            print(f"[WARN] IK failed at step {k}: {e}")
            joint_angles = prev_joint_angles

        # Apply joint angles
        for i in range(n_joints):
            p.resetJointState(robot_id, i, joint_angles[i])

        p.stepSimulation()
        time.sleep(0.01)  # Slow down to real time if using GUI

        # Record actual CoM (base position)
        pos, _ = p.getBasePositionAndOrientation(robot_id)
        com_actual[k] = pos

        # Record ZMP from contact forces
        zx, zy, fz = compute_zmp_from_contacts(robot_id, ground_id)
        zmp_actual[k] = [zx, zy]
        fz_log[k] = fz

        if k % 240 == 0:
            print(f"  t={t_axis[k]:.2f}s | base_z={pos[2]:.3f} | fz={fz:.1f}N")

    p.disconnect()
    print("[INFO] Simulation finished.")

    # 5) Plot and save results
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))

    # Sagittal X
    ax = axes[0, 0]
    ax.plot(t_axis, com_ref[:, 0], label="CoM ref X")
    ax.plot(t_axis, com_actual[:, 0], ls="--", label="CoM actual X")
    ax.plot(t_axis, zmp_ref[:, 0], label="ZMP ref X")
    ax.plot(t_axis, zmp_actual[:, 0], ls="--", label="ZMP actual X")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("X [m]")
    ax.legend()
    ax.set_title("Sagittal Trajectory")
    ax.grid(True)

    # Lateral Y
    ax = axes[0, 1]
    ax.plot(t_axis, com_ref[:, 1], label="CoM ref Y")
    ax.plot(t_axis, com_actual[:, 1], ls="--", label="CoM actual Y")
    ax.plot(t_axis, zmp_ref[:, 1], label="ZMP ref Y")
    ax.plot(t_axis, zmp_actual[:, 1], ls="--", label="ZMP actual Y")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Y [m]")
    ax.legend()
    ax.set_title("Lateral Trajectory")
    ax.grid(True)

    # Side view (X-Z)
    ax = axes[1, 0]
    ax.plot(com_ref[:, 0], com_ref[:, 2], label="CoM ref")
    ax.plot(com_actual[:, 0], com_actual[:, 2], ls="--", label="CoM actual")
    ax.plot(left_foot_ref[:, 0], left_foot_ref[:, 2], label="Left foot")
    ax.plot(right_foot_ref[:, 0], right_foot_ref[:, 2], label="Right foot")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Z [m]")
    ax.legend()
    ax.set_title("Side View")
    ax.grid(True)

    # Top view (X-Y) with support footprints
    ax = axes[1, 1]
    ax.plot(com_ref[:, 0], com_ref[:, 1], label="CoM ref")
    ax.plot(zmp_ref[:, 0], zmp_ref[:, 1], label="ZMP ref")
    # Draw support-foot rectangles
    half_l = 0.05
    half_w = 0.04
    for seg in traj["foot_refs"]:
        if seg["support"] == "left":
            fx, fy = seg["left"]
            rect_x = [fx - half_l, fx + half_l, fx + half_l, fx - half_l, fx - half_l]
            rect_y = [fy - half_w, fy - half_w, fy + half_w, fy + half_w, fy - half_w]
            ax.plot(rect_x, rect_y, "g-", lw=0.8, alpha=0.5)
        elif seg["support"] == "right":
            fx, fy = seg["right"]
            rect_x = [fx - half_l, fx + half_l, fx + half_l, fx - half_l, fx - half_l]
            rect_y = [fy - half_w, fy - half_w, fy + half_w, fy + half_w, fy - half_w]
            ax.plot(rect_x, rect_y, "r-", lw=0.8, alpha=0.5)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.legend()
    ax.set_title("Top View with Support Feet")
    ax.grid(True)
    ax.set_aspect("equal")

    plt.tight_layout()
    out_png = "walking_results.png"
    plt.savefig(out_png, dpi=150)
    print(f"[INFO] Plot saved to: {os.path.abspath(out_png)}")

    # Quick console summary
    valid = ~np.isnan(zmp_actual[:, 0])
    if np.any(valid):
        zmp_err_x = zmp_actual[valid, 0] - zmp_ref[valid, 0]
        zmp_err_y = zmp_actual[valid, 1] - zmp_ref[valid, 1]
        print(f"[SUMMARY] ZMP RMS error: X={np.sqrt(np.mean(zmp_err_x**2)) * 1000:.2f} mm, "
              f"Y={np.sqrt(np.mean(zmp_err_y**2)) * 1000:.2f} mm")
    print(f"[SUMMARY] CoM tracking RMS error: "
          f"X={np.sqrt(np.mean((com_actual[:, 0] - com_ref[:, 0])**2)) * 1000:.2f} mm, "
          f"Y={np.sqrt(np.mean((com_actual[:, 1] - com_ref[:, 1])**2)) * 1000:.2f} mm, "
          f"Z={np.sqrt(np.mean((com_actual[:, 2] - com_ref[:, 2])**2)) * 1000:.2f} mm")


if __name__ == "__main__":
    main()
