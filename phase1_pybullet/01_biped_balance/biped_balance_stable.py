"""
项目1.1: 双足站立 - 稳定版
使用 Torque Control 避免 Position Control 的数值问题
"""

import pybullet as p
import pybullet_data
import numpy as np
import time


def pd_control(target, current, vel, kp, kd):
    """PD控制器"""
    return kp * (target - current) - kd * vel


def main():
    # 初始化
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # 设置更稳定的物理参数
    p.setGravity(0, 0, -9.81)
    p.setPhysicsEngineParameter(
        numSolverIterations=50,
        numSubSteps=4
    )

    # 加载
    plane = p.loadURDF("plane.urdf")
    robot = p.loadURDF("humanoid/humanoid.urdf", [0, 0, 1.05])

    # 关节映射
    r_hip, r_knee, r_ankle = 9, 10, 11
    l_hip, l_knee, l_ankle = 12, 13, 14
    leg_joints = [r_hip, r_knee, r_ankle, l_hip, l_knee, l_ankle]

    # 设置初始站立姿态（关键！）
    print("设置初始站立姿态...")
    standing_pose = {
        r_hip: -0.1, r_knee: 0.2, r_ankle: -0.1,
        l_hip: -0.1, l_knee: 0.2, l_ankle: -0.1,
    }
    for j, pos in standing_pose.items():
        p.resetJointState(robot, j, pos)

    print("稳定站立控制 - Torque Control 模式")
    print("="*50)

    # 等待落地并稍微稳定
    print("等待落地...")
    for _ in range(200):
        p.stepSimulation()

    pos, _ = p.getBasePositionAndOrientation(robot)
    print(f"落地后高度: {pos[2]:.3f}m")

    # 先禁用默认电机控制
    print("启动扭矩控制...")
    for j in range(p.getNumJoints(robot)):
        p.setJointMotorControl2(robot, j, p.VELOCITY_CONTROL, force=0)

    # 仿真循环
    dt = 1/240  # 使用更小的时间步
    target_height = 0.85

    for step in range(2400):  # 10秒
        # 获取 torso 状态
        pos, orn = p.getBasePositionAndOrientation(robot)
        vel, ang_vel = p.getBaseVelocity(robot)
        euler = p.getEulerFromQuaternion(orn)

        height = pos[2]
        pitch = euler[1]

        if height < 0.5:
            print(f"\n摔倒 at t={step*dt:.2f}s")
            break

        # 获取关节状态
        j_states = p.getJointStates(robot, leg_joints)
        j_pos = [s[0] for s in j_states]
        j_vel = [s[1] for s in j_states]

        # 目标关节角度 - 增强控制
        # 高度控制 -> 膝盖弯曲
        h_error = target_height - height
        knee_target = np.clip(h_error * 2.0, 0, 0.6)

        # 姿态控制 -> 脚踝调整 (增强以快速纠正倾斜)
        ankle_target = -pitch * 1.0 - ang_vel[1] * 0.3
        ankle_target = np.clip(ankle_target, -0.4, 0.4)

        # 髋关节补偿
        hip_target = -knee_target * 0.5 - pitch * 0.2

        targets = [hip_target, knee_target, ankle_target,
                   hip_target, knee_target, ankle_target]

        # 计算力矩 (PD控制) - 增大增益
        max_torque = 200
        for i, j in enumerate(leg_joints):
            torque = pd_control(targets[i], j_pos[i], j_vel[i], 80, 8)
            torque = np.clip(torque, -max_torque, max_torque)
            p.setJointMotorControl2(robot, j, p.TORQUE_CONTROL, force=torque)

        p.stepSimulation()

        if step % 240 == 0:  # 每秒打印
            print(f"t={step*dt:4.1f}s | h={height:.3f}m | pitch={np.degrees(pitch):5.1f}°")

        time.sleep(dt)

    else:
        print(f"\n✓ 成功站立 10 秒!")

    p.disconnect()


if __name__ == "__main__":
    main()
