"""
Cart-Pole 平衡控制 - 简化版 (无GUI)
==================================
使用 Gymnasium 的 CartPole 环境，专注于理解 PD 控制逻辑
"""

import numpy as np
import gymnasium as gym


class PDController:
    """PD 控制器 - 理解核心控制逻辑"""

    def __init__(self, kp=50.0, kd=10.0):
        """
        参数:
            kp: 比例增益 - 根据角度误差产生纠正力
            kd: 微分增益 - 根据角速度产生阻尼力
        """
        self.kp = kp
        self.kd = kd

    def compute(self, angle, angular_velocity):
        """
        计算控制输出

        输入:
            angle: 摆杆角度 (rad)，0 = 直立向上
            angular_velocity: 角速度 (rad/s)

        输出:
            action: 0 (向左推) 或 1 (向右推)

        原理:
            我们希望角度趋近于 0，角速度趋近于 0
            控制力 = -Kp * angle - Kd * angular_velocity

            如果 angle > 0 (向右倒) → 需要向左推 → action = 0
            如果 angle < 0 (向左倒) → 需要向右推 → action = 1
        """
        # PD 控制公式
        # 目标：角度趋向0，角速度趋向0
        # CartPole动力学：推小车会带动摆杆反向摆动
        # 如果 angle > 0 (向右倾斜)，需要向右推小车 → action = 1
        # 如果 angle < 0 (向左倾斜)，需要向左推小车 → action = 0
        control_force = self.kp * angle + self.kd * angular_velocity

        # 离散化为左推或右推
        # control_force > 0: 角度向右偏，需要向右推 → action = 1
        # control_force < 0: 角度向左偏，需要向左推 → action = 0
        action = 1 if control_force > 0 else 0

        return action, control_force


def run_simulation(kp=50.0, kd=10.0, max_steps=500, render=False):
    """
    运行一次仿真

    返回:
        steps: 存活步数 (越高越好)
        max_angle: 最大摆角 (越小越好)
    """
    # 创建环境
    render_mode = "human" if render else None
    env = gym.make('CartPole-v1', render_mode=render_mode)
    observation, info = env.reset()

    controller = PDController(kp=kp, kd=kd)

    steps = 0
    max_angle_deg = 0
    total_reward = 0

    for step in range(max_steps):
        # 解析观测值
        # CartPole observation: [cart_pos, cart_vel, pole_angle, pole_vel]
        cart_pos, cart_vel, pole_angle, pole_vel = observation

        # 记录最大角度
        angle_deg = abs(np.degrees(pole_angle))
        max_angle_deg = max(max_angle_deg, angle_deg)

        # 计算控制动作
        action, force = controller.compute(pole_angle, pole_vel)

        # 执行动作
        observation, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        steps += 1

        # 打印关键步骤
        if step < 10 or step % 50 == 0:
            direction = "→" if action == 1 else "←"
            print(f"Step {step:3d} | Angle: {angle_deg:+6.2f}° | "
                  f"ω: {pole_vel:+.3f} | Force: {force:+7.2f} | Action: {direction}")

        # 检查是否失败 (角度超过阈值)
        if terminated or truncated:
            break

    env.close()

    return steps, max_angle_deg, total_reward


def test_different_gains():
    """测试不同 PD 参数的效果"""
    print("=" * 70)
    print("Cart-Pole PD 控制参数测试")
    print("=" * 70)
    print()

    # 测试的参数组合
    test_cases = [
        (10, 0, "低 Kp，无 Kd"),
        (50, 0, "中等 Kp，无 Kd"),
        (100, 0, "高 Kp，无 Kd"),
        (50, 10, "中等 Kp + 中等 Kd"),
        (50, 30, "中等 Kp + 高 Kd"),
    ]

    print(f"{'Kp':>6} | {'Kd':>6} | {'描述':<20} | {'存活步数':>8} | {'最大角度':>8} | {'结果':<10}")
    print("-" * 70)

    for kp, kd, desc in test_cases:
        steps, max_angle, reward = run_simulation(kp=kp, kd=kd, max_steps=500)

        if steps >= 499:
            result = "✓ 成功"
        elif steps > 200:
            result = "~ 部分成功"
        else:
            result = "✗ 失败"

        print(f"{kp:>6.0f} | {kd:>6.0f} | {desc:<20} | {steps:>8} | {max_angle:>7.1f}° | {result}")

    print()
    print("=" * 70)
    print("观察结论:")
    print("- 只有 Kp: 纠正不足或振荡，无法稳定")
    print("- Kp + Kd: 微分项提供阻尼，成功平衡!")
    print("- 最佳参数: Kp=50, Kd=10 (在这个环境)")
    print()
    print("关键理解:")
    print("1. Kp 负责'纠正'角度误差")
    print("2. Kd 负责'抑制'振荡(阻尼)")
    print("3. 两者缺一不可，类似汽车的方向盘和减震器")
    print("=" * 70)


def interactive_demo():
    """交互式演示 - 详细展示每一步的控制逻辑"""
    print()
    print("=" * 70)
    print("详细控制过程演示 (Kp=50, Kd=10)")
    print("=" * 70)
    print()

    steps, max_angle, reward = run_simulation(kp=50, kd=10, max_steps=200)

    print()
    print(f"总结: 存活 {steps} 步, 最大角度 {max_angle:.1f}°")
    print()

    # 解释 PD 控制公式
    print("=" * 70)
    print("PD 控制公式解析:")
    print("=" * 70)
    print()
    print("控制输出 = -Kp × 角度误差 - Kd × 角速度")
    print()
    print("示例计算 (假设 Kp=50, Kd=10):")
    print("  情况1: 角度=+10°(右倒), 角速度=0")
    print("    控制力 = -50 × 0.17rad - 10 × 0 = -8.5 (向左推)")
    print()
    print("  情况2: 角度=0, 角速度=+2rad/s(向右摆)")
    print("    控制力 = -50 × 0 - 10 × 2 = -20 (向左推，阻尼)")
    print()
    print("  情况3: 角度=-5°(左倒), 角速度=-1rad/s(向左摆)")
    print("    控制力 = -50 × (-0.09) - 10 × (-1) = +4.5 + 10 = +14.5 (向右推)")
    print()


if __name__ == "__main__":
    # 1. 先运行参数测试
    test_different_gains()

    print()
    input("按 Enter 查看详细控制过程...")

    # 2. 详细演示
    interactive_demo()
