"""
项目1.1: PyBullet 2D双足站稳 - 稳定版

控制策略：
1. 使用 PyBullet 内置 POSITION_CONTROL (更稳定)
2. 质心高度控制 → 调整膝盖角度
3. 姿态控制 → 调整踝关节角度 (类似倒立摆)
4. 髋关节补偿 → 保持身体直立
"""

import pybullet as p
import pybullet_data
import numpy as np
import time


class StableBipedController:
    """稳定双足控制器 - 使用内置PD控制"""

    # 关节索引映射
    JOINTS = {
        'right_hip': 9,
        'right_knee': 10,
        'right_ankle': 11,
        'left_hip': 12,
        'left_knee': 13,
        'left_ankle': 14,
    }

    def __init__(self, robot_id):
        self.robot_id = robot_id

        # 目标参数
        self.target_height = 0.85
        self.stand_pitch = 0.0

        # PD增益 (保守值保证稳定)
        self.pitch_kp = 0.3  # 姿态控制增益
        self.pitch_kd = 0.1

        self.height_kp = 0.5  # 高度控制增益
        self.height_kd = 0.2

        # 关节力矩限制
        self.max_force = 200

    def get_state(self):
        """获取机器人状态"""
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        vel, ang_vel = p.getBaseVelocity(self.robot_id)
        euler = p.getEulerFromQuaternion(orn)

        return {
            'height': pos[2],
            'height_vel': vel[2],
            'pitch': euler[1],
            'pitch_vel': ang_vel[1],
        }

    def compute_targets(self, state):
        """
        计算关节目标角度

        核心策略:
        1. 身体前倾(pitch>0) -> 踝关节向后(负)推地面
        2. 身体太低 -> 膝盖弯曲
        3. 髋关节补偿保持平衡
        """
        pitch = state['pitch']
        pitch_vel = state['pitch_vel']
        height = state['height']
        height_vel = state['height_vel']

        # 姿态控制: 踝关节调整身体倾斜
        # pitch > 0 (前倾) -> ankle < 0 (向后推)
        ankle_offset = -self.pitch_kp * pitch - self.pitch_kd * pitch_vel
        ankle_offset = np.clip(ankle_offset, -0.3, 0.3)

        # 高度控制: 膝盖弯曲程度
        height_error = self.target_height - height
        knee_bend = self.height_kp * height_error - self.height_kd * height_vel
        knee_bend = np.clip(knee_bend, 0, 0.5)

        # 髋关节: 补偿膝盖弯曲，保持躯干垂直
        hip_offset = -knee_bend * 0.5

        targets = {
            'right_hip': hip_offset,
            'right_knee': knee_bend,
            'right_ankle': ankle_offset,
            'left_hip': hip_offset,
            'left_knee': knee_bend,
            'left_ankle': ankle_offset,
        }

        return targets

    def apply(self, targets):
        """应用控制目标"""
        for name, target in targets.items():
            idx = self.JOINTS[name]
            p.setJointMotorControl2(
                self.robot_id, idx,
                p.POSITION_CONTROL,
                targetPosition=target,
                positionGain=100,
                velocityGain=10,
                force=self.max_force
            )


class BipedSimulation:
    """双足仿真环境"""

    def __init__(self, render=True):
        self.render = render

        # 初始化PyBullet
        if render:
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        # 加载环境
        self.plane = p.loadURDF("plane.urdf")

        # 加载机器人 - 从稍高的位置落下，让双脚先接触地面
        start_pos = [0, 0, 1.05]
        start_orn = p.getQuaternionFromEuler([0, 0, 0])
        self.robot = p.loadURDF("humanoid/humanoid.urdf", start_pos, start_orn)

        # 等待机器人落地稳定
        for _ in range(100):
            p.stepSimulation()

        # 等待落地后创建控制器
        time.sleep(0.5)  # 给GUI时间渲染

        self.controller = StableBipedController(self.robot)

        # 数据记录
        self.history = {'t': [], 'h': [], 'p': []}

    def step(self):
        """仿真一步"""
        state = self.controller.get_state()
        targets = self.controller.compute_targets(state)
        self.controller.apply(targets)
        p.stepSimulation()
        return state['height'], state['pitch']

    def run(self, duration=5.0, dt=0.01):
        """运行仿真"""
        print(f"\n双足平衡控制 - 稳定版")
        print(f"目标高度: {self.controller.target_height}m")
        print(f"按Ctrl+C停止\n")

        steps = int(duration / dt)
        fallen = False

        try:
            for i in range(steps):
                t = i * dt
                h, p = self.step()

                # 记录
                self.history['t'].append(t)
                self.history['h'].append(h)
                self.history['p'].append(p)

                # 打印
                if i % 100 == 0:
                    status = "OK" if h > 0.6 else "FALL"
                    print(f"[{status}] t={t:.1f}s h={h:.3f}m pitch={np.degrees(p):5.1f}°")

                # 检查摔倒
                if h < 0.5 or np.isnan(h):
                    print(f"\n摔倒! t={t:.2f}s")
                    fallen = True
                    break

                if self.render:
                    time.sleep(dt)

        except KeyboardInterrupt:
            print("\n中断")
            return None

        if not fallen:
            print(f"\n✓ 成功站立 {duration}秒")
            self._print_stats()
            return True
        return False

    def _print_stats(self):
        """打印统计"""
        h = self.history['h']
        p = self.history['p']
        print(f"\n高度: mean={np.mean(h):.3f}m std={np.std(h):.3f}m")
        print(f"倾斜: max={np.degrees(np.max(np.abs(p))):.1f}°")

    def disconnect(self):
        p.disconnect()


def main():
    sim = BipedSimulation(render=True)
    try:
        sim.run(duration=5.0)
    finally:
        sim.disconnect()


if __name__ == "__main__":
    main()
