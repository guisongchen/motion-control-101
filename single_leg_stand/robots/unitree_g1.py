"""Unitree G1 robot configuration."""

import numpy as np

from robot_model import RobotConfig

g1_config = RobotConfig(
    model_path="/home/ccc/projects/unitree_ros/robots/g1_description/g1_23dof.xml",
    root_body_name="pelvis",
    foot_link_names=[
        "left_ankle_roll_link",
        "right_ankle_roll_link",
    ],
    standing_joint_angles={
        "left_hip_pitch_joint": -0.2,
        "left_hip_roll_joint": 0.0,
        "left_hip_yaw_joint": 0.0,
        "left_knee_joint": 0.4,
        "left_ankle_pitch_joint": -0.2,
        "left_ankle_roll_joint": 0.0,
        "right_hip_pitch_joint": -0.2,
        "right_hip_roll_joint": 0.0,
        "right_hip_yaw_joint": 0.0,
        "right_knee_joint": 0.4,
        "right_ankle_pitch_joint": -0.2,
        "right_ankle_roll_joint": 0.0,
        "waist_yaw_joint": 0.0,
        "left_shoulder_pitch_joint": 0.0,
        "left_shoulder_roll_joint": 0.0,
        "left_shoulder_yaw_joint": 0.0,
        "left_elbow_joint": 0.0,
        "left_wrist_roll_joint": 0.0,
        "right_shoulder_pitch_joint": 0.0,
        "right_shoulder_roll_joint": 0.0,
        "right_shoulder_yaw_joint": 0.0,
        "right_elbow_joint": 0.0,
        "right_wrist_roll_joint": 0.0,
    },
    base_initial_pos=np.array([0.0, 0.0, 0.76]),
    base_initial_orn=np.array([0.0, 0.0, 0.0, 1.0]),
    lift_leg="left",
    leg_joint_names={
        "left": [
            "left_hip_pitch_joint",
            "left_hip_roll_joint",
            "left_hip_yaw_joint",
            "left_knee_joint",
            "left_ankle_pitch_joint",
            "left_ankle_roll_joint",
        ],
        "right": [
            "right_hip_pitch_joint",
            "right_hip_roll_joint",
            "right_hip_yaw_joint",
            "right_knee_joint",
            "right_ankle_pitch_joint",
            "right_ankle_roll_joint",
        ],
    },
)
