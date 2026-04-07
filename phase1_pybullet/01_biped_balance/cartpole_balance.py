"""
Cart-Pole 平衡控制示例
======================
目标: 理解倒立摆的 PD 控制原理，为双足控制打基础

核心概念:
- 状态: [小车位置x, 小车速度v, 摆杆角度θ, 摆杆角速度ω]
- 控制: 施加在小车上的水平力 F
- 目标: 保持摆杆直立 (θ ≈ 0)
"""

import pybullet as p
import pybullet_data
import time
import numpy as np

# ==================== 配置参数 ====================

# PD 控制器增益 (这是你要调节的核心参数)
Kp = 150.0   # 比例增益: 角度误差越大，纠正力越大 (增大以提高响应)
Kd = 30.0    # 微分增益: 角速度越大，阻尼力越大 (增大以减少振荡)

# 仿真参数
SIMULATION_STEPS = 5000   # 仿真步数
MAX_FORCE = 100           # 最大施加力
TARGET_FPS = 60

# 初始扰动 (测试鲁棒性)
INITIAL_PUSH = 5.0        # 初始推一下摆杆 (N)

# ==================== 工具函数 ====================

def pd_controller(angle, angular_velocity, cart_position=0, cart_velocity=0):
    """
    PD 控制器 - 核心控制逻辑 (带可选位置控制)

    输入:
        angle: 摆杆角度 (rad), 0表示直立向上
        angular_velocity: 摆杆角速度 (rad/s)
        cart_position: 小车位置 (可选)
        cart_velocity: 小车速度 (可选)

    输出:
        force: 施加在小车上的水平力 (N)
    """
    # 角度控制项
    error_angle = -angle
    error_angle_dot = -angular_velocity
    force_angle = Kp * error_angle + Kd * error_angle_dot

    # 可选: 位置控制 (防止小车跑太远)
    Kp_pos = 0.5   # 位置增益
    Kd_pos = 0.2   # 速度增益
    force_position = Kp_pos * (-cart_position) + Kd_pos * (-cart_velocity)

    # 总控制力
    force = force_angle + force_position

    # 限制最大力 (防止数值不稳定)
    force = np.clip(force, -MAX_FORCE, MAX_FORCE)

    return force


def get_cartpole_state(cartpole):
    """获取 Cart-Pole 的当前状态"""
    # 关节 0: 滑动关节，控制 cart 水平移动
    # 关节 1: 旋转关节，控制 pole 旋转

    # 获取小车位置 (滑动关节的位置)
    cart_joint_state = p.getJointState(cartpole, 0)
    cart_position = cart_joint_state[0]
    cart_velocity = cart_joint_state[1]

    # 获取摆杆角度 (旋转关节的位置)
    pole_joint_state = p.getJointState(cartpole, 1)
    angle = pole_joint_state[0]
    angular_velocity = pole_joint_state[1]

    return cart_position, angle, angular_velocity, cart_velocity


def debug_urdf_structure(cartpole):
    """打印 URDF 结构信息用于调试"""
    print("\n[DEBUG] URDF 结构分析:")
    print("-" * 60)

    # Base 信息
    base_pos = p.getBasePositionAndOrientation(cartpole)
    print(f"Base 位置: {base_pos[0]}")

    # 关节信息
    num_joints = p.getNumJoints(cartpole)
    print(f"关节数量: {num_joints}")

    for i in range(num_joints):
        joint_info = p.getJointInfo(cartpole, i)
        link_name = joint_info[12].decode('utf-8')
        joint_name = joint_info[1].decode('utf-8')
        joint_type = joint_info[2]
        print(f"  关节 {i}: Link='{link_name}', Joint='{joint_name}', Type={joint_type}")

    # Link 信息
    print("\nLink 状态:")
    for i in range(num_joints):
        link_state = p.getLinkState(cartpole, i)
        print(f"  Link {i}: Pos={link_state[0]}")

    # 关节状态
    print("\n关节状态:")
    for i in range(num_joints):
        joint_state = p.getJointState(cartpole, i)
        print(f"  关节 {i}: Pos={joint_state[0]:.4f}, Vel={joint_state[1]:.4f}")

    print("-" * 60)


def print_state(step, pos, angle, ang_vel, force):
    """打印当前状态 (角度转为度便于阅读)"""
    angle_deg = np.degrees(angle)
    if step % 100 == 0:  # 每100步打印一次
        print(f"Step {step:4d} | Pos: {pos:+.3f}m | "
              f"Angle: {angle_deg:+7.2f}° | "
              f"ω: {ang_vel:+.3f}rad/s | "
              f"Force: {force:+6.1f}N")

def print_detailed_debug(step, pos, angle, ang_vel, force, cart_vel, prev_pos=None):
    """详细调试信息，每10步打印一次"""
    if step % 10 == 0:
        angle_deg = np.degrees(angle)
        pos_change = pos - prev_pos if prev_pos is not None else 0
        print(f"[DEBUG] Step {step:4d} | Pos: {pos:+.4f}m | ΔPos: {pos_change:+.4f}m | "
              f"Angle: {angle_deg:+7.2f}° | ω: {ang_vel:+.4f}rad/s | Cart_v: {cart_vel:+.4f}m/s | Force: {force:+6.1f}N")


# ==================== 主程序 ====================

def main():
    """主仿真循环"""

    # ---- 1. 初始化 PyBullet ----
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # 加载地面
    plane_id = p.loadURDF("plane.urdf")

    # 加载 Cart-Pole
    # 注意: PyBullet 内置的 cartpole.urdf
    cartpole = p.loadURDF("cartpole.urdf", [0, 0, 0.1])

    # 设置仿真步长
    p.setTimeStep(1./240.)

    # ---- 关键: 释放关节控制 ----
    # 滑动关节默认有位置保持，需要先禁用才能自由移动
    p.setJointMotorControl2(cartpole, 0, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
    # 摆杆关节也设为自由模式
    p.setJointMotorControl2(cartpole, 1, p.VELOCITY_CONTROL, targetVelocity=0, force=0)

    # ---- 调试: 打印 URDF 结构 ----
    debug_urdf_structure(cartpole)

    # ---- 2. 初始扰动 (测试稳定性) ----
    print("=" * 60)
    print("Cart-Pole 平衡控制")
    print("=" * 60)
    print(f"PD参数: Kp={Kp}, Kd={Kd}")
    print(f"初始扰动: 施加 {INITIAL_PUSH}N 的力")
    print("-" * 60)

    # 给摆杆一个初始角度扰动 (通过设置关节位置)
    # 或者给一个初始速度
    p.resetJointState(cartpole, 1, targetValue=0.1, targetVelocity=0)  # 给摆杆初始角度 ~5.7度
    print(f"初始扰动: 设置摆杆初始角度 0.1 rad (~5.7°)")

    # ---- 3. 仿真循环 ----
    stable_steps = 0
    max_angle_deg = 0
    prev_pos = None

    print("\n[DEBUG] 开始仿真，监控小车位置变化...")
    print("=" * 80)

    for step in range(SIMULATION_STEPS):
        # 获取当前状态
        pos, angle, ang_vel, cart_vel = get_cartpole_state(cartpole)
        angle_deg = abs(np.degrees(angle))
        max_angle_deg = max(max_angle_deg, angle_deg)

        # 判断是否成功平衡 (角度小于15度认为是稳定)
        if angle_deg < 15:
            stable_steps += 1

        # 计算控制输出 (核心!)
        force = pd_controller(angle, ang_vel, pos, cart_vel)

        # 通过关节电机控制小车运动 (滑动关节索引 0)
        # 使用 TORQUE_CONTROL 或 VELOCITY_CONTROL
        p.setJointMotorControl2(cartpole, 0, p.TORQUE_CONTROL, force=force)

        # 详细调试输出
        print_detailed_debug(step, pos, angle, ang_vel, force, cart_vel, prev_pos)
        prev_pos = pos

        # 打印状态
        print_state(step, pos, angle, ang_vel, force)

        # 步进仿真
        p.stepSimulation()
        time.sleep(1./TARGET_FPS)

    # ---- 4. 结果统计 ----
    print("-" * 60)
    print("仿真结束")
    print(f"最大摆角: {max_angle_deg:.2f}°")
    print(f"稳定步数: {stable_steps}/{SIMULATION_STEPS} "
          f"({100*stable_steps/SIMULATION_STEPS:.1f}%)")

    if stable_steps > SIMULATION_STEPS * 0.9:
        print("✓ 成功! 控制参数有效")
    else:
        print("✗ 失败! 尝试调节 Kp 和 Kd")

    input("按 Enter 退出...")
    p.disconnect()


# ==================== 参数调节指南 ====================
"""
如何调节 PD 参数:

1. 先调 Kp (比例增益):
   - 从 Kp=10 开始，Kd=0
   - 如果摆杆纠正太慢 → 增大 Kp
   - 如果摆杆来回振荡 → 减小 Kp

2. 再调 Kd (微分增益):
   - 固定 Kp，从 Kd=1 开始
   - 如果振荡不收敛 → 增大 Kd
   - 如果反应迟钝 → 减小 Kd

3. 典型问题诊断:

   现象              原因              解决方法
   ─────────────────────────────────────────────
   摆杆直接倒下      Kp 太小          增大 Kp
   剧烈振荡          Kp 太大/Kd 太小  减小 Kp 或增大 Kd
   收敛慢            Kd 太大          减小 Kd
   小车跑太远        没有位置控制     添加位置反馈 (见下方)

4. 添加位置控制 (防止小车跑掉):
   修改 pd_controller 为:

   Kp_pos = 1.0    # 位置增益
   Kd_pos = 0.5    # 速度增益

   force_angle = Kp * (-angle) + Kd * (-ang_vel)
   force_position = Kp_pos * (-pos) + Kd_pos * (-cart_velocity)
   force = force_angle + force_position
"""

if __name__ == "__main__":
    main()
