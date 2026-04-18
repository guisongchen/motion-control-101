"""Week 10 实验配置：PyBullet 单腿站立（MPC-WBC 闭环验证）"""

import numpy as np

# ---------------------------------------------------------------------------
# 仿真环境参数
# ---------------------------------------------------------------------------
DT_SIM = 1.0 / 240.0          # PyBullet 默认物理步长 [s]
WBC_FREQ = 1000               # WBC 控制频率 [Hz]
MPC_FREQ = 20                 # MPC 重求解频率 [Hz] (= 50 ms 周期)
SIM_DURATION = 3.0            # 总仿真时长 [s]
GRAVITY = np.array([0.0, 0.0, -9.81])
MU = 0.8                      # 地面摩擦系数

# ---------------------------------------------------------------------------
# 机器人模型参数
# ---------------------------------------------------------------------------
URDF_PATH = "/home/ccc/projects/unitree_ros/robots/g1_description/g1_23dof.urdf" 
INITIAL_POSE = "standing"     # 初始姿势：双脚站立
LIFT_LEG = "left"             # t=0.5s 抬起的腿
LIFT_TIME = 0.5               # 抬腿时刻 [s]
H_COM = 0.8                   # 期望 CoM 高度 [m]（相对于支撑足）

# 候选足端 link 名称（动态检测哪个在接触）
FOOT_LINK_NAMES = [
    "left_ankle_roll_link",
    "right_ankle_roll_link",
]

# 初始站立姿态（弧度）
# 先测试零位姿态，观察稳定性后再微调
STANDING_JOINT_ANGLES = {
    # 左腿
    "left_hip_pitch_joint": 0.0,
    "left_hip_roll_joint": 0.0,
    "left_hip_yaw_joint": 0.0,
    "left_knee_joint": 0.0,
    "left_ankle_pitch_joint": 0.0,
    "left_ankle_roll_joint": 0.0,
    # 右腿
    "right_hip_pitch_joint": 0.0,
    "right_hip_roll_joint": 0.0,
    "right_hip_yaw_joint": 0.0,
    "right_knee_joint": 0.0,
    "right_ankle_pitch_joint": 0.0,
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
BASE_INITIAL_POS = np.array([0.0, 0.0, 0.697])      # 零位下 foot-to-base ≈ 0.697 m
BASE_INITIAL_ORN = np.array([0.0, 0.0, 0.0, 1.0])   # [qx, qy, qz, qw]

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
