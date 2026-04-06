"""
项目1.1: PyBullet 2D双足站稳
目标: 让双足机器人在仿真中稳定站立

能学到:
- 物理引擎基础
- PD控制实现
- 状态反馈概念
- 多关节协调

验收标准:
- [ ] 机器人能在原地站立5秒不倒
- [ ] 能抵抗小扰动（用手推不倒）
- [ ] 理解PD参数对稳定性的影响
"""

import pybullet as p
import pybullet_data
import numpy as np
import time


class BipedBalanceController:
    """双足平衡PD控制器"""

    def __init__(self, kp=100.0, kd=10.0):
        """
        初始化控制器

        Args:
            kp: 比例增益
            kd: 微分增益
        """
        self.kp = kp
        self.kd = kd
        self.prev_error = None

    def compute_torque(self, current_pos, current_vel, target_pos=0.0):
        """
        计算PD控制力矩

        Args:
            current_pos: 当前关节位置/角度
            current_vel: 当前关节速度
            target_pos: 目标位置（默认0，即直立）

        Returns:
            控制力矩
        """
        error = target_pos - current_pos

        # 比例项
        p_term = self.kp * error

        # 微分项
        d_term = -self.kd * current_vel  # 负反馈阻尼

        torque = p_term + d_term

        return torque


class BipedSimulation:
    """双足机器人仿真环境"""

    def __init__(self, render=True):
        """初始化仿真环境"""
        self.render = render

        # 连接PyBullet
        if render:
            self.physics_client = p.connect(p.GUI)
        else:
            self.physics_client = p.connect(p.DIRECT)

        # 添加搜索路径
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # 设置重力
        p.setGravity(0, 0, -9.81)

        # 加载地面
        self.plane_id = p.loadURDF("plane.urdf")

        # 加载双足机器人 (使用humanoid模型)
        # humanoid.urdf 是PyBullet自带的模型
        start_pos = [0, 0, 1.0]  # 初始位置稍高，让其自然落下
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])

        self.robot_id = p.loadURDF(
            "humanoid/humanoid.urdf",
            start_pos,
            start_orientation,
            useFixedBase=False
        )

        # 获取关节信息
        self.num_joints = p.getNumJoints(self.robot_id)
        print(f"机器人关节数: {self.num_joints}")

        # 打印关节信息
        self._print_joint_info()

        # 创建控制器
        self.controller = BipedBalanceController(kp=100.0, kd=10.0)

        # 用于记录数据
        self.time_history = []
        self.height_history = []
        self.pitch_history = []

    def _print_joint_info(self):
        """打印关节信息"""
        print("\n=== 关节信息 ===")
        for i in range(self.num_joints):
            info = p.getJointInfo(self.robot_id, i)
            joint_name = info[1].decode('utf-8')
            joint_type = info[2]
            print(f"关节 {i}: {joint_name}, 类型: {joint_type}")
        print("================\n")

    def get_base_state(self):
        """
        获取机器人基座（躯干）状态

        Returns:
            position, orientation, linear_vel, angular_vel
        """
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        vel, ang_vel = p.getBaseVelocity(self.robot_id)
        return pos, orn, vel, ang_vel

    def get_joint_states(self):
        """
        获取所有关节状态

        Returns:
            positions, velocities, torques
        """
        joint_states = p.getJointStates(self.robot_id, range(self.num_joints))
        positions = [s[0] for s in joint_states]
        velocities = [s[1] for s in joint_states]
        torques = [s[3] for s in joint_states]
        return positions, velocities, torques

    def apply_balance_control(self):
        """应用平衡控制"""
        # 获取基座状态
        pos, orn, vel, ang_vel = self.get_base_state()

        # 将四元数转换为欧拉角 (roll, pitch, yaw)
        euler = p.getEulerFromQuaternion(orn)
        pitch = euler[1]  # 前后倾斜角

        # 获取高度（Z轴位置）
        height = pos[2]

        # 简单的站立策略：
        # 1. 如果身体前倾(pitch>0)，需要向后调整
        # 2. 如果身体后仰(pitch<0)，需要向前调整

        # 获取关节状态
        joint_pos, joint_vel, _ = self.get_joint_states()

        # 为每个关节计算控制力矩
        # 这里使用简单的PD控制，目标是将关节保持在初始位置
        for i in range(self.num_joints):
            # 获取关节信息
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_type = joint_info[2]

            # 只控制旋转关节
            if joint_type == p.JOINT_REVOLUTE:
                current_pos = joint_pos[i]
                current_vel = joint_vel[i]

                # 根据身体姿态调整目标角度
                # 如果身体前倾，稍微弯曲膝盖/髋关节向后
                target_pos = -pitch * 0.5  # 简单的补偿策略

                # 计算控制力矩
                torque = self.controller.compute_torque(
                    current_pos, current_vel, target_pos
                )

                # 设置力矩限制
                max_torque = 500.0
                torque = np.clip(torque, -max_torque, max_torque)

                # 应用力矩控制
                p.setJointMotorControl2(
                    bodyUniqueId=self.robot_id,
                    jointIndex=i,
                    controlMode=p.TORQUE_CONTROL,
                    force=torque
                )

        return height, pitch

    def reset_joint_control(self):
        """重置关节为力矩控制模式"""
        for i in range(self.num_joints):
            p.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=i,
                controlMode=p.VELOCITY_CONTROL,
                force=0  # 禁用默认的速度控制
            )

    def step(self):
        """仿真一步"""
        # 应用控制
        height, pitch = self.apply_balance_control()

        # 步进仿真
        p.stepSimulation()

        return height, pitch

    def run(self, duration=5.0, dt=0.01):
        """
        运行仿真

        Args:
            duration: 仿真时长（秒）
            dt: 时间步长
        """
        print(f"开始仿真，时长: {duration}秒")
        print(f"PD参数: Kp={self.controller.kp}, Kd={self.controller.kd}")
        print("按Ctrl+C停止仿真\n")

        # 重置控制模式
        self.reset_joint_control()

        # 仿真循环
        steps = int(duration / dt)
        for step in range(steps):
            t = step * dt

            # 执行一步
            height, pitch = self.step()

            # 记录数据
            self.time_history.append(t)
            self.height_history.append(height)
            self.pitch_history.append(pitch)

            # 实时打印
            if step % 100 == 0:
                print(f"时间: {t:.2f}s, 高度: {height:.3f}m, 倾斜: {np.degrees(pitch):.2f}°")

            # 检查是否摔倒
            if height < 0.5:
                print(f"\n机器人在 {t:.2f}秒 摔倒！")
                break

            # 渲染延时
            if self.render:
                time.sleep(dt)

        print("\n仿真结束")
        self._print_stats()

    def _print_stats(self):
        """打印统计信息"""
        if len(self.height_history) == 0:
            return

        print("\n=== 仿真统计 ===")
        print(f"平均高度: {np.mean(self.height_history):.3f}m")
        print(f"高度标准差: {np.std(self.height_history):.3f}m")
        print(f"最大倾斜: {np.degrees(np.max(np.abs(self.pitch_history))):.2f}°")
        print(f"仿真时长: {self.time_history[-1]:.2f}秒")

    def disconnect(self):
        """断开连接"""
        p.disconnect()


def main():
    """主函数"""
    sim = BipedSimulation(render=True)

    try:
        sim.run(duration=5.0)
    except KeyboardInterrupt:
        print("\n用户中断仿真")
    finally:
        sim.disconnect()


if __name__ == "__main__":
    main()
