"""
项目1.1: 简化版双足站立控制

策略：使用质心(CoM)跟踪 + 角动量控制
- 测量 torso 位置和姿态
- 控制总质心保持在支撑多边形上方
- 使用脚踝力矩调整姿态
"""

import pybullet as p
import pybullet_data
import numpy as np
import time


def main():
    # 初始化
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # 加载
    plane = p.loadURDF("plane.urdf")
    robot = p.loadURDF("humanoid/humanoid.urdf", [0, 0, 1.05])

    # 关节索引
    chest = 1
    r_hip, r_knee, r_ankle = 9, 10, 11
    l_hip, l_knee, l_ankle = 12, 13, 14

    leg_joints = [r_hip, r_knee, r_ankle, l_hip, l_knee, l_ankle]

    print("等待落地...")
    for _ in range(100):
        p.stepSimulation()

    print("启动平衡控制")
    print("策略: 质心高度控制 + 姿态PD控制")

    # 控制参数 - 保守值避免数值不稳定
    target_height = 0.82
    Kp_height = 0.3
    Kd_height = 0.1

    Kp_pitch = 0.5
    Kd_pitch = 0.15

    # 运行仿真
    dt = 0.01
    for step in range(1000):
        # 获取状态
        pos, orn = p.getBasePositionAndOrientation(robot)
        vel, ang_vel = p.getBaseVelocity(robot)
        euler = p.getEulerFromQuaternion(orn)

        height = pos[2]
        pitch = euler[1]
        height_vel = vel[2]
        pitch_vel = ang_vel[1]

        # 高度控制 -> 膝盖弯曲
        h_err = target_height - height
        knee_target = Kp_height * h_err - Kd_height * height_vel
        knee_target = np.clip(knee_target, 0, 0.6)

        # 姿态控制 -> 脚踝角度
        # pitch > 0 (前倾) -> ankle < 0 (向后推)
        ankle_target = -(Kp_pitch * pitch + Kd_pitch * pitch_vel)
        ankle_target = np.clip(ankle_target, -0.4, 0.4)

        # 髋关节补偿
        hip_target = -knee_target * 0.3

        # 应用到双腿
        targets = {
            r_hip: hip_target, r_knee: knee_target, r_ankle: ankle_target,
            l_hip: hip_target, l_knee: knee_target, l_ankle: ankle_target,
        }

        for j, t in targets.items():
            p.setJointMotorControl2(robot, j, p.POSITION_CONTROL,
                                    targetPosition=t, positionGain=100,
                                    velocityGain=10, force=300)

        # 躯干保持直立
        p.setJointMotorControl2(robot, chest, p.POSITION_CONTROL,
                                targetPosition=0, positionGain=50, force=100)

        p.stepSimulation()

        # 打印状态
        if step % 100 == 0:
            status = "OK" if height > 0.6 else "FALL"
            print(f"[{status}] t={step*dt:.1f}s h={height:.3f}m "
                  f"pitch={np.degrees(pitch):5.1f}° "
                  f"knee={np.degrees(knee_target):4.1f}° "
                  f"ankle={np.degrees(ankle_target):4.1f}°")

        # 检查摔倒
        if height < 0.5:
            print(f"\n摔倒! t={step*dt:.2f}s")
            break

        time.sleep(dt)

    else:
        print(f"\n✓ 成功站立 10 秒!")

    p.disconnect()


if __name__ == "__main__":
    main()
