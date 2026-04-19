"""Week 10 experiment config: MuJoCo single-leg stand (MPC-WBC closed loop)."""

import numpy as np

# ---------------------------------------------------------------------------
# 仿真环境参数
# ---------------------------------------------------------------------------
DT_SIM = 1.0 / 240.0          # Simulation step [s]
WBC_FREQ = 1000               # WBC 控制频率 [Hz]
MPC_FREQ = 20                 # MPC 重求解频率 [Hz] (= 50 ms 周期)
SIM_DURATION = 3.0            # 总仿真时长 [s]
GRAVITY = np.array([0.0, 0.0, -9.81])
MU = 0.8                      # 地面摩擦系数

# ---------------------------------------------------------------------------
# 机器人模型参数
# ---------------------------------------------------------------------------
MODEL_PATH = "/home/ccc/projects/unitree_ros/robots/g1_description/g1_23dof.xml"
INITIAL_POSE = "standing"     # 初始姿势：双脚站立
LIFT_LEG = "left"             # t=0.5s 抬起的腿
LIFT_TIME = 0.5               # 抬腿时刻 [s]
H_COM = 0.8                   # 期望 CoM 高度 [m]（相对于支撑足）

# 候选足端 link 名称（动态检测哪个在接触）
FOOT_LINK_NAMES = [
    "left_ankle_roll_link",
    "right_ankle_roll_link",
]
SUPPORT_FOOT_NAME = "right_ankle_roll_link" if LIFT_LEG == "left" else "left_ankle_roll_link"

# 初始站立姿态（弧度）
# MuJoCo 下零位姿态会让双足站立非常脆弱；使用轻微屈膝姿态作为默认站立姿态。
STANDING_JOINT_ANGLES = {
    # 左腿
    "left_hip_pitch_joint": -0.2,
    "left_hip_roll_joint": 0.0,
    "left_hip_yaw_joint": 0.0,
    "left_knee_joint": 0.4,
    "left_ankle_pitch_joint": -0.2,
    "left_ankle_roll_joint": 0.0,
    # 右腿
    "right_hip_pitch_joint": -0.2,
    "right_hip_roll_joint": 0.0,
    "right_hip_yaw_joint": 0.0,
    "right_knee_joint": 0.4,
    "right_ankle_pitch_joint": -0.2,
    "right_ankle_roll_joint": 0.0,
    # 躯干
    "waist_yaw_joint": 0.0,
    # 手臂（自然下垂）
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
}

# 基座初始位姿
BASE_INITIAL_POS = np.array([0.0, 0.0, 0.76])       # 轻微屈膝站姿下的稳定双脚落地高度
BASE_INITIAL_ORN = np.array([0.0, 0.0, 0.0, 1.0])   # [qx, qy, qz, qw]

# ---------------------------------------------------------------------------
# Actuator layer
# ---------------------------------------------------------------------------
POSTURE_KP = 80.0
POSTURE_KD = 12.0
LIFT_LEG_KP = 120.0
LIFT_LEG_KD = 8.0
TRANSITION_BLEND_TIME = 0.15
SWING_RAMP_TIME = 0.8
SINGLE_SUPPORT_SWING_RAMP_TIME = 1.5
SINGLE_SUPPORT_SWING_PROGRESS_MAX = 0.45
SWING_HIP_PITCH_TARGET = 0.15
SWING_KNEE_TARGET = -0.3
SUPPORT_POINT_FILTER = 0.1
MIN_SUPPORT_FORCE = 50.0
BASE_DOF_DAMPING = 5.0
JOINT_DOF_DAMPING = 10.0
LOAD_SHIFT_ROLL_DELTA = 0.025
PRE_LIFTOFF_EXTRA_ROLL_DELTA = 0.001
PRE_LIFTOFF_EXTRA_SWING_PROGRESS = 0.05
PRE_LIFTOFF_SWING_KP_SCALE = 0.75
PRE_LIFTOFF_SWING_KD_SCALE = 0.85
SINGLE_SUPPORT_EXTRA_ROLL_DELTA = 0.0
SINGLE_SUPPORT_ENTRY_TIME = 0.60
SINGLE_SUPPORT_MPC_DELAY = 0.30
SINGLE_SUPPORT_FORCE_BLEND_TIME = 0.50
SINGLE_SUPPORT_ESTABLISH_TIME = 0.12
SINGLE_SUPPORT_ESTABLISH_SUPPORT_RATIO = 0.62
SINGLE_SUPPORT_ESTABLISH_SWING_FORCE_MAX = 105.0
SINGLE_SUPPORT_ESTABLISH_COM_SPEED = 0.35
SINGLE_SUPPORT_POSE_BLEND_TIME = 0.60
SINGLE_SUPPORT_MIN_FORCE_RATIO = 0.90
SINGLE_SUPPORT_HOLD_FORCE = 15.0
SINGLE_SUPPORT_PRE_ESTABLISH_TAU_BLEND = 0.20
SINGLE_SUPPORT_MAX_TAU_BLEND = 0.45
SINGLE_SUPPORT_MAX_HORIZONTAL_FORCE = 20.0
SINGLE_SUPPORT_SUPPORT_HIP_PITCH_DELTA = 0.0
SINGLE_SUPPORT_FORWARD_ERROR_CLIP = 0.08
SINGLE_SUPPORT_SWING_HIP_PITCH_TARGET = 0.00
SINGLE_SUPPORT_SWING_KNEE_TARGET = 0.80
SINGLE_SUPPORT_SWING_ANKLE_PITCH_TARGET = -0.35
LOAD_SHIFT_SUPPORT_RATIO = 0.52
PRE_LIFTOFF_SUPPORT_RATIO = 0.56
SINGLE_SUPPORT_SUPPORT_RATIO = 0.65
LOAD_SHIFT_COM_RATIO = 0.16
PRE_LIFTOFF_COM_RATIO = 0.20
SINGLE_SUPPORT_COM_RATIO = 0.28
PRE_LIFTOFF_SWING_FORCE_MAX = 110.0
SINGLE_SUPPORT_SWING_FORCE_TARGET = 100.0
COM_VEL_READY_THRESH = 0.20

# ---------------------------------------------------------------------------
# Phase machine
# ---------------------------------------------------------------------------
INIT_SETTLE_TIME = 0.20
DOUBLE_SUPPORT_READY_TIME = 0.60
LOAD_SHIFT_TIME = 0.20
PRE_LIFTOFF_TIME = 0.50
DOUBLE_SUPPORT_MIN_FORCE = 120.0
PRE_LIFTOFF_SWING_PROGRESS = 0.25
LOAD_SHIFT_READY_TIME = 0.08
PRE_LIFTOFF_READY_TIME = 0.10

# ---------------------------------------------------------------------------
# MPC 参数
# ---------------------------------------------------------------------------
N_HORIZON = 10                # 预测时域
T_S = 0.05                    # MPC 离散时间 [s]

# 状态维度: [c(3), c_dot(3), L(3)] -> 9
# 控制维度: 单支撑只有 1 个接触点 -> f(3)
NX = 9
NU = 3

Q = np.diag([100.0, 100.0, 100.0,
             1.0, 1.0, 1.0,
             1.0, 1.0, 1.0])
R = np.diag([0.001, 0.001, 0.001])
QN = Q.copy()

# 摩擦锥线性近似（4 面）
# |f_x| <= mu * f_z, |f_y| <= mu * f_z, f_z >= 0

# ---------------------------------------------------------------------------
# WBC 参数
# ---------------------------------------------------------------------------
# PD 增益
Kp_c = 100.0
Kd_c = 20.0
Kp_L = 10.0
Kd_L = 2.0

# 权重矩阵
W1 = 100.0 * np.eye(3)        # CoM 跟踪
W2 = 10.0 * np.eye(3)         # 角动量跟踪
W3 = 0.1 * np.eye(3)          # 接触力软参考
W4 = 0.01                     # 最小化加速度（在代码中根据维度扩展）

# ---------------------------------------------------------------------------
# 评估指标阈值
# ---------------------------------------------------------------------------
RMSE_THRESH = 0.02            # CoM 位置 RMSE < 2 cm
SLIP_THRESH = 0.005           # 支撑足滑移 < 5 mm
MPC_TIME_THRESH = 0.020       # MPC 求解时间 < 20 ms
WBC_TIME_THRESH = 0.0005      # WBC 求解时间 < 0.5 ms

# ---------------------------------------------------------------------------
# Direct single-support WBC benchmark
# ---------------------------------------------------------------------------
DIRECT_SINGLE_SUPPORT_DURATION = 3.0
DIRECT_SINGLE_SUPPORT_SUPPORT_FOOT_NAME = SUPPORT_FOOT_NAME
DIRECT_SINGLE_SUPPORT_BASE_HEIGHT = 0.72
DIRECT_SINGLE_SUPPORT_BASE_LATERAL_SHIFT = 0.04
DIRECT_SINGLE_SUPPORT_BASE_ROLL = 0.02
DIRECT_SINGLE_SUPPORT_SUPPORT_ROLL_DELTA = 0.04
DIRECT_SINGLE_SUPPORT_SWING_ROLL_DELTA = 0.16
DIRECT_SINGLE_SUPPORT_SUPPORT_PITCH_DELTA = 0.02
DIRECT_SINGLE_SUPPORT_SWING_HIP_PITCH = 0.10
DIRECT_SINGLE_SUPPORT_SWING_KNEE = 1.05
DIRECT_SINGLE_SUPPORT_SWING_ANKLE_PITCH = -0.55
DIRECT_SINGLE_SUPPORT_POSTURE_KP = 120.0
DIRECT_SINGLE_SUPPORT_POSTURE_KD = 18.0
DIRECT_SINGLE_SUPPORT_COM_KP = 20.0
DIRECT_SINGLE_SUPPORT_COM_KD = 5.0
DIRECT_SINGLE_SUPPORT_MOMENTUM_KP = 4.0
DIRECT_SINGLE_SUPPORT_MOMENTUM_KD = 1.0
DIRECT_SINGLE_SUPPORT_DAMPING_SCALE = 2.0
DIRECT_SINGLE_SUPPORT_SUPPORT_BLEND = 0.01
DIRECT_SINGLE_SUPPORT_MAX_CONTACT_SLIP = 0.02
DIRECT_SINGLE_SUPPORT_MAX_SWING_FORCE = 20.0
DIRECT_SINGLE_SUPPORT_MAX_FRICTION_RATIO = 0.75
