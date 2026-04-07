"""
双足机器人 - 单关节 PD 控制测试
===============================
目标: 验证 Cart-Pole 的 PD 控制逻辑如何应用到双足

控制策略:
- 只控制脚踝力矩 (模仿 Cart-Pole 的小车力)
- 髋和膝固定角度 (简化问题)
- 目标: 减缓倾倒速度，验证控制方向正确

状态映射:
Cart-Pole          双足机器人
----------------------------------------
摆杆角度 θ    →    躯干 pitch 角度
摆杆角速度 ω  →    躯干 pitch 角速度
小车力 F      →    脚踝力矩 τ
"""

import pybullet as p
import pybullet_data
import numpy as np
import time


# ==================== 配置参数 ====================

# PD 控制器增益 (从 Cart-Pole 调整而来)
Kp = 20.0   # 比例增益 (比 Cart-Pole 小，因为力矩直接作用于躯干)
Kd = 5.0    # 微分增益

# 仿真参数
MAX_STEPS = 1000
TIME_STEP = 1./240.

# 初始姿态 (轻微弯曲，降低质心)
INITIAL_POSE = {
    "r_hip": -0.1,    # 髋: 轻微后摆
    "r_knee": 0.2,    # 膝: 轻微弯曲
    "r_ankle": 0.0,   # 踝: 初始直立
    "l_hip": -0.1,
    "l_knee": 0.2,
    "l_ankle": 0.0,
}

# 控制参数
HIP_TARGET = -0.1     # 髋固定目标
KNEE_TARGET = 0.2     # 膝固定目标

# 关节索引 (PyBullet humanoid.urdf)
JOINT_INDICES = {
    "r_hip": 8,
    "r_knee": 9,
    "r_ankle": 10,
    "l_hip": 14,
    "l_knee": 15,
    "l_ankle": 16,
}


# ==================== 工具函数 ====================

def setup_simulation():
    """初始化仿真环境"""
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(TIME_STEP)

    # 加载地面
    plane_id = p.loadURDF("plane.urdf")

    # 加载人形机器人 (使用完整路径)
    humanoid_path = pybullet_data.getDataPath() + "/humanoid/humanoid.urdf"
    robot = p.loadURDF(humanoid_path, [0, 0, 1.2])

    return robot


def set_initial_pose(robot):
    """设置初始姿态"""
    for joint_name, angle in INITIAL_POSE.items():
        idx = JOINT_INDICES[joint_name]
        p.resetJointState(robot, idx, angle)


def set_fixed_joints(robot):
    """设置髋和膝为固定位置控制"""
    # 右腿
    p.setJointMotorControl2(robot, JOINT_INDICES["r_hip"],
                           p.POSITION_CONTROL, targetPosition=HIP_TARGET,
                           positionGain=50, maxVelocity=5)
    p.setJointMotorControl2(robot, JOINT_INDICES["r_knee"],
                           p.POSITION_CONTROL, targetPosition=KNEE_TARGET,
                           positionGain=50, maxVelocity=5)

    # 左腿
    p.setJointMotorControl2(robot, JOINT_INDICES["l_hip"],
                           p.POSITION_CONTROL, targetPosition=HIP_TARGET,
                           positionGain=50, maxVelocity=5)
    p.setJointMotorControl2(robot, JOINT_INDICES["l_knee"],
                           p.POSITION_CONTROL, targetPosition=KNEE_TARGET,
                           positionGain=50, maxVelocity=5)


def pd_control_ankle(pitch, pitch_vel):
    """
    PD 控制计算脚踝力矩

    公式 (从 Cart-Pole 迁移):
        Cart-Pole:  F = Kp * angle + Kd * angular_velocity
        双足:        τ = -Kp * pitch - Kd * pitch_vel

    负号原因:
        - Cart-Pole 推小车带动摆杆 (间接)
        - 双足脚踝力矩直接作用于躯干 (直接)
        - 方向相反

    参数:
        pitch: 躯干 pitch 角度 (rad), 正值 = 前倾
        pitch_vel: pitch 角速度 (rad/s)

    返回:
        ankle_torque: 脚踝力矩 (Nm)
    """
    # PD 公式
    torque = -Kp * pitch - Kd * pitch_vel

    # 限制最大力矩 (防止数值问题)
    torque = np.clip(torque, -50, 50)

    return torque


def get_torso_state(robot):
    """获取躯干状态"""
    pos, orn = p.getBasePositionAndOrientation(robot)
    vel, ang_vel = p.getBaseVelocity(robot)

    # 四元数转欧拉角 (roll, pitch, yaw)
    euler = p.getEulerFromQuaternion(orn)
    pitch = euler[1]  # 绕 y 轴旋转 = 前后倾斜

    # pitch 角速度
    pitch_vel = ang_vel[1]

    return pos, pitch, pitch_vel


def apply_ankle_control(robot, torque):
    """施加脚踝力矩控制"""
    # 双腿同时施加相同力矩
    p.setJointMotorControl2(robot, JOINT_INDICES["r_ankle"],
                           p.TORQUE_CONTROL, force=torque)
    p.setJointMotorControl2(robot, JOINT_INDICES["l_ankle"],
                           p.TORQUE_CONTROL, force=torque)


def print_header():
    """打印表头"""
    print("=" * 80)
    print("双足单关节 PD 控制测试 - 仅脚踝")
    print("=" * 80)
    print(f"PD参数: Kp={Kp}, Kd={Kd}")
    print(f"髋/膝: 固定位置 | 踝: PD力矩控制")
    print("-" * 80)
    print(f"{'Step':>5} | {'Height':>7} | {'Pitch':>8} | {'Pitch ω':>8} | {'Torque':>8} | Status")
    print("-" * 80)


def print_state(step, pos, pitch, pitch_vel, torque):
    """打印当前状态"""
    pitch_deg = np.degrees(pitch)
    status = "OK" if abs(pitch_deg) < 30 else "倾斜"

    if step % 50 == 0:
        print(f"{step:>5} | {pos[2]:>7.3f} | {pitch_deg:>+7.1f}° | "
              f"{pitch_vel:>+8.3f} | {torque:>+8.1f} | {status}")


# ==================== 主程序 ====================

def main():
    """主仿真循环"""

    # 1. 初始化
    robot = setup_simulation()

    # 2. 设置初始姿态
    set_initial_pose(robot)

    # 让机器人稳定落地 (不控制，自由落体)
    print("落地稳定中...")
    for _ in range(100):
        p.stepSimulation()
        time.sleep(TIME_STEP)

    # 获取落地后的状态
    pos, pitch, pitch_vel = get_torso_state(robot)
    print(f"落地后: 高度={pos[2]:.3f}m, 倾斜={np.degrees(pitch):.1f}°")

    # 3. 启动控制
    # 切换到力矩控制模式需要先禁用电机
    p.setJointMotorControl2(robot, JOINT_INDICES["r_ankle"],
                           p.VELOCITY_CONTROL, force=0)
    p.setJointMotorControl2(robot, JOINT_INDICES["l_ankle"],
                           p.VELOCITY_CONTROL, force=0)

    print_header()

    # 4. 控制循环
    fall_step = None

    for step in range(MAX_STEPS):
        # 获取状态
        pos, pitch, pitch_vel = get_torso_state(robot)

        # 计算脚踝力矩 (PD控制)
        ankle_torque = pd_control_ankle(pitch, pitch_vel)

        # 施加控制
        set_fixed_joints(robot)  # 髋膝固定
        apply_ankle_control(robot, ankle_torque)  # 脚踝 PD

        # 打印状态
        print_state(step, pos, pitch, pitch_vel, ankle_torque)

        # 检查是否摔倒
        pitch_deg = abs(np.degrees(pitch))
        if pitch_deg > 60 and fall_step is None:
            fall_step = step
            print(f"\n>>> 机器人在第 {step} 步摔倒 (倾斜 {pitch_deg:.1f}°)")

        # 步进仿真
        p.stepSimulation()
        time.sleep(TIME_STEP)

    # 5. 结果总结
    print("-" * 80)
    print("仿真结束")

    if fall_step:
        print(f"坚持步数: {fall_step} / {MAX_STEPS}")
        print("分析: 脚踝力矩可能不足以维持平衡，或需要调节 PD 参数")
    else:
        print(f"✓ 成功! 机器人保持平衡 {MAX_STEPS} 步")

    print("\n参数调节建议:")
    print("- 如果倾倒太快: 增大 Kp")
    print("- 如果振荡不收敛: 增大 Kd")
    print("- 如果脚踝控制不够: 考虑添加髋/膝协调控制")

    input("\n按 Enter 退出...")
    p.disconnect()


# ==================== 参数调节指南 ====================
"""
调试步骤:

1. 先运行脚本，观察机器人是否比不控制时倒得慢
   - 如果完全没区别 → 检查力矩方向是否正确

2. 调节 Kp:
   - 从 10 开始，逐渐增大到 50
   - 观察是否能减缓倾倒

3. 调节 Kd:
   - 固定 Kp，从 0 开始增大
   - 观察是否有阻尼效果

4. 如果单脚踝无法平衡，这是正常的!
   - 下一步: 添加膝盖控制 (高度+姿态协调)
"""

if __name__ == "__main__":
    main()
