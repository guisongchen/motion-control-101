"""Grouped configuration for the direct single-support benchmark."""

from dataclasses import dataclass, field

import numpy as np

import config
from robots.unitree_g1 import g1_config


@dataclass(frozen=True)
class DirectSingleSupportEnvConfig:
    model_path: str = g1_config.model_path
    gravity: np.ndarray = field(default_factory=lambda: np.array(config.GRAVITY, copy=True))
    dt: float = config.DT_SIM
    base_damping: float = config.BASE_DOF_DAMPING
    joint_damping: float = config.JOINT_DOF_DAMPING
    foot_link_names: tuple[str, ...] = field(default_factory=lambda: tuple(g1_config.foot_link_names))
    support_foot_name: str = "right_ankle_roll_link" if g1_config.lift_leg == "left" else "left_ankle_roll_link"
    standing_joint_angles: dict[str, float] = field(
        default_factory=lambda: dict(g1_config.standing_joint_angles)
    )


@dataclass(frozen=True)
class DirectSingleSupportPoseConfig:
    base_height: float = 0.72
    base_lateral_shift: float = 0.04
    base_roll: float = 0.02
    support_roll_delta: float = 0.04
    swing_roll_delta: float = 0.16
    support_pitch_delta: float = 0.02
    swing_hip_pitch: float = 0.10
    swing_knee: float = 1.05
    swing_ankle_pitch: float = -0.55
    support_arm_shoulder_pitch: float = -1.2
    support_arm_shoulder_roll: float = 0.3
    support_arm_elbow: float = 0.8
    damping_scale: float = 2.0


@dataclass(frozen=True)
class DirectSingleSupportControlConfig:
    duration: float = 3.0
    posture_kp: float = 120.0
    posture_kd: float = 18.0
    com_kp: float = 20.0
    com_kd: float = 5.0
    momentum_kp: float = 4.0
    momentum_kd: float = 1.0
    support_blend: float = 0.705
    use_wrench_objective: bool = True


@dataclass(frozen=True)
class DirectSingleSupportContactConfig:
    wbc_contact_point: str = "corner_patch"
    cop_target: str = "initial_contact"
    cop_margin: float = 0.005


@dataclass(frozen=True)
class DirectSingleSupportCopFeedbackConfig:
    enabled: bool = True
    filter_alpha: float = 0.1
    kp: float = 0.035
    kd: float = 0.0
    max_offset: float = 0.003


@dataclass(frozen=True)
class DirectSingleSupportWrenchConfig:
    state_filter_alpha: float = 0.1
    slip_force_kp: float = 600.0
    slip_force_kd: float = 20.0
    slip_force_max: float = 20.0
    yaw_moment_kp: float = 8.0
    yaw_moment_kd: float = 1.0
    yaw_moment_max: float = 4.0
    force_xy_weight: float = 0.03
    force_z_weight: float = 0.0
    moment_weight: float = 5.0
    yaw_moment_weight: float = 1.0


@dataclass(frozen=True)
class DirectSingleSupportSuccessConfig:
    max_contact_slip: float = 0.02
    max_swing_force: float = 20.0
    max_friction_ratio: float = 0.75


@dataclass(frozen=True)
class DirectSingleSupportConfig:
    env: DirectSingleSupportEnvConfig = field(default_factory=DirectSingleSupportEnvConfig)
    pose: DirectSingleSupportPoseConfig = field(default_factory=DirectSingleSupportPoseConfig)
    control: DirectSingleSupportControlConfig = field(default_factory=DirectSingleSupportControlConfig)
    contact: DirectSingleSupportContactConfig = field(default_factory=DirectSingleSupportContactConfig)
    cop: DirectSingleSupportCopFeedbackConfig = field(default_factory=DirectSingleSupportCopFeedbackConfig)
    wrench: DirectSingleSupportWrenchConfig = field(default_factory=DirectSingleSupportWrenchConfig)
    success: DirectSingleSupportSuccessConfig = field(default_factory=DirectSingleSupportSuccessConfig)


DIRECT_SINGLE_SUPPORT_CONFIG = DirectSingleSupportConfig()
