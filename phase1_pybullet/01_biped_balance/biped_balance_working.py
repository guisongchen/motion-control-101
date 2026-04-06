"""
项目1.1: 双足站立控制 - 工作版

关键发现：
1. 需要同时控制 hip, knee, ankle 协调运动
2. 从较低初始高度开始（双脚着地）
3. 使用位置控制比力矩控制更稳定
"""

import pybullet as p
import pybullet_data
import numpy as np
import time


def main():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    plane = p.loadURDF("plane.urdf")
    robot = p.loadURDF("humanoid/humanoid.urdf", [0, 0, 0.92])

    # 关节
    r_hip, r_knee, r_ankle = 9, 10, 11
    l_hip, l_knee, l_ankle = 12, 13, 14
    leg_joints = [r_hip, r_knee, r_ankle, l_hip, l_knee, l_ankle]

    print("双足站立控制 - 工作版")
    print("=" * 50)

    # 先落地
    print("等待落地...")
    for _ in range(100):
        p.stepSimulation()

    # 站立控制
    print("启动站立控制...")
    target_height = 0.88

    for step in range(600):  # 10秒
        pos, orn = p.getBasePositionAndOrientation(robot)
        vel, _ = p.getBaseVelocity(robot)
        euler = p.getEulerFromQuaternion(orn)

        height = pos[2]
        pitch = euler[1]

        # 高度误差
        h_err = target_height - height

        # 控制策略：
        # 1. 姿态控制优先：用脚踝纠正倾斜（倒立摆原理）
        # 2. 高度控制：用膝盖调节
        # 3. 髋关节：保持身体垂直

        # 姿态控制（关键！）：前倾时脚踝向后推
        ankle_target = np.clip(-pitch * 2.0 - euler[0] * 0.5, -0.4, 0.4)

        # 高度控制：目标0.85m，误差为正时弯曲膝盖抬高
        # 但限制膝盖弯曲范围，避免不稳定
        knee_target = np.clip(h_err * 1.0, -0.1, 0.3)

        # 髋关节：补偿膝盖弯曲，保持躯干垂直
        hip_target = -knee_target * 0.6 - pitch * 0.2

        # 应用到双腿
        targets = [hip_target, knee_target, ankle_target,
                   hip_target, knee_target, ankle_target]

        for j, t in zip(leg_joints, targets):
            p.setJointMotorControl2(
                robot, j, p.POSITION_CONTROL,
                targetPosition=t, force=300
            )

        p.stepSimulation()

        if step % 60 == 0:
            status = "✓" if height > 0.7 else "✗"
            print(f"{status} t={step/60:4.1f}s | h={height:.3f}m | "
                  f"pitch={np.degrees(pitch):5.1f}° | knee={np.degrees(knee_target):4.0f}°")

        if height < 0.5:
            print(f"\n摔倒 at t={step/60:.2f}s")
            break

        time.sleep(1/60)

    else:
        print("\n✓ 成功站立 10 秒!")

    p.disconnect()


if __name__ == "__main__":
    main()
