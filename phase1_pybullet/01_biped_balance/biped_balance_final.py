"""
项目1.1: 双足站立控制 - 最终版

成功策略:
1. 从较低位置开始，双脚先着地
2. 立即启动 PD 位置控制 (比力矩控制更稳定)
3. 质心高度控制 + 姿态控制
"""

import pybullet as p
import pybullet_data
import numpy as np
import time


def main():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # 加载环境
    plane = p.loadURDF("plane.urdf")

    # 从较低位置加载，双脚先着地
    robot = p.loadURDF("humanoid/humanoid.urdf", [0, 0, 0.9])

    # 关节索引
    r_hip, r_knee, r_ankle = 9, 10, 11
    l_hip, l_knee, l_ankle = 12, 13, 14
    chest = 1
    leg_joints = [r_hip, r_knee, r_ankle, l_hip, l_knee, l_ankle]

    print("双足站立控制")
    print("="*50)

    # 控制参数
    target_height = 0.88
    Kp_h, Kd_h = 1.5, 0.5  # 高度控制
    Kp_p, Kd_p = 2.0, 0.5  # 姿态控制

    dt = 1/60  # 60Hz

    for step in range(600):  # 10秒
        # 获取 torso 状态
        pos, orn = p.getBasePositionAndOrientation(robot)
        vel, ang_vel = p.getBaseVelocity(robot)
        euler = p.getEulerFromQuaternion(orn)

        height = pos[2]
        pitch = euler[1]
        h_vel = vel[2]
        p_vel = ang_vel[1]

        # 计算目标关节角度
        # 1. 高度控制 -> 膝盖弯曲
        h_err = target_height - height
        knee = np.clip(Kp_h * h_err - Kd_h * h_vel, 0, 0.6)

        # 2. 姿态控制 -> 脚踝调整
        ankle = np.clip(-(Kp_p * pitch + Kd_p * p_vel), -0.4, 0.4)

        # 3. 髋关节补偿
        hip = -knee * 0.5 - pitch * 0.3

        # 应用控制
        targets = [hip, knee, ankle, hip, knee, ankle]
        for j, t in zip(leg_joints, targets):
            p.setJointMotorControl2(
                robot, j, p.POSITION_CONTROL,
                targetPosition=t,
                positionGain=100,
                velocityGain=10,
                force=300
            )

        # 躯干保持直立
        p.setJointMotorControl2(robot, chest, p.POSITION_CONTROL,
                                targetPosition=0, positionGain=50, force=100)

        p.stepSimulation()

        # 打印状态
        if step % 60 == 0:
            status = "OK" if height > 0.6 else "FALL"
            print(f"[{status}] t={step*dt:4.1f}s h={height:.3f}m "
                  f"pitch={np.degrees(pitch):5.1f}°")

        # 检查摔倒
        if height < 0.5 or np.isnan(height):
            print(f"\n摔倒 at t={step*dt:.2f}s")
            break

        time.sleep(dt)

    else:
        print(f"\n✓ 成功站立 10 秒!")

    p.disconnect()


if __name__ == "__main__":
    main()
