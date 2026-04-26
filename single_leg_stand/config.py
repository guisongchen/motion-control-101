"""Week 10 experiment config: MuJoCo single-leg stand (MPC-WBC closed loop)."""

import numpy as np

# ---------------------------------------------------------------------------
# 仿真环境参数
# ---------------------------------------------------------------------------
DT_SIM = 0.001                # Simulation step [s]
WBC_FREQ = 250                # WBC 控制频率 [Hz] (effective rate with DT_SIM=0.001)
MPC_FREQ = 20                 # MPC 重求解频率 [Hz] (= 50 ms 周期)
SIM_DURATION = 6.0            # 总仿真时长 [s]
GRAVITY = np.array([0.0, 0.0, -9.81])
MU = 0.8                      # 地面摩擦系数
TORSIONAL_FRICTION_GAMMA = 0.05  # 扭转摩擦边界系数: |mz| <= gamma * fz

# ---------------------------------------------------------------------------
# 机器人模型参数
# ---------------------------------------------------------------------------
H_COM = 0.8                   # 期望 CoM 高度 [m]（相对于支撑足）

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
SINGLE_SUPPORT_MAX_TAU_BLEND = 0.85
SINGLE_SUPPORT_MAX_HORIZONTAL_FORCE = 20.0
SINGLE_SUPPORT_SUPPORT_HIP_PITCH_DELTA = 0.0
SINGLE_SUPPORT_FORWARD_ERROR_CLIP = 0.08
SINGLE_SUPPORT_SWING_HIP_PITCH_TARGET = 0.00
SINGLE_SUPPORT_SWING_KNEE_TARGET = 0.80
SINGLE_SUPPORT_SWING_ANKLE_PITCH_TARGET = -0.35
LOAD_SHIFT_SUPPORT_RATIO = 0.48
PRE_LIFTOFF_SUPPORT_RATIO = 0.56
SINGLE_SUPPORT_SUPPORT_RATIO = 0.65
LOAD_SHIFT_COM_RATIO = 0.16
PRE_LIFTOFF_COM_RATIO = 0.20
SINGLE_SUPPORT_COM_RATIO = 0.28
LOAD_SHIFT_SWING_FORCE_MIN = MIN_SUPPORT_FORCE
PRE_LIFTOFF_SWING_FORCE_MAX = 150.0
SINGLE_SUPPORT_SWING_FORCE_TARGET = 100.0
COM_VEL_READY_THRESH = 0.35
PRE_LIFTOFF_FORWARD_ERROR_THRESH = 0.05
PRE_LIFTOFF_FORWARD_VEL_THRESH = 0.12

# ---------------------------------------------------------------------------
# Phase machine
# ---------------------------------------------------------------------------
INIT_SETTLE_TIME = 0.20
DOUBLE_SUPPORT_READY_TIME = 1.00
LOAD_SHIFT_TIME = 0.20
PRE_LIFTOFF_TIME = 0.50
DOUBLE_SUPPORT_MIN_FORCE = 120.0
DOUBLE_SUPPORT_MAX_COM_VEL = 0.05       # CoM 速度上限 [m/s]
DOUBLE_SUPPORT_MAX_L_NORM = 0.50        # 角动量上限 [kg·m²/s]
DOUBLE_SUPPORT_FORCE_RATIO_MIN = 0.35   # 双足力比 min/total
DOUBLE_SUPPORT_COM_MARGIN = 0.02        # CoM 投影到支撑多边形内边界的安全余量 [m]
PRE_LIFTOFF_SWING_PROGRESS = 0.25
LOAD_SHIFT_READY_TIME = 0.0
PRE_LIFTOFF_READY_TIME = 0.0

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
W3 = 0.1 * np.eye(4)          # 接触力软参考 (4D: fx, fy, fz, mz)
W4 = 0.01                     # 最小化加速度（在代码中根据维度扩展）
DS_TOTAL_FORCE_XY_WEIGHT = 100.0
DS_TOTAL_FORCE_Z_WEIGHT = 10.0
DS_LOAD_DISTRIBUTION_WEIGHT = 1000.0

# ---------------------------------------------------------------------------
# 评估指标阈值
# ---------------------------------------------------------------------------
RMSE_THRESH = 0.02            # CoM 位置 RMSE < 2 cm
SLIP_THRESH = 0.005           # 支撑足滑移 < 5 mm
MPC_TIME_THRESH = 0.020       # MPC 求解时间 < 20 ms
WBC_TIME_THRESH = 0.0005      # WBC 求解时间 < 0.5 ms
