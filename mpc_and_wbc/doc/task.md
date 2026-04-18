Week 10 实验配置：PyBullet 单腿站立（MPC-WBC 闭环验证）

---
1. 实验目标

验证 MPC-WBC 分层控制架构的核心接口：
- MPC（50 Hz）：基于 centroidal dynamics 优化 CoM 轨迹与接触力
- WBC（1 kHz）：将 MPC 参考转化为关节力矩，实现单腿站立稳定控制
- 成功标准：机器人单腿支撑站立 3 秒，CoM 位置误差 < 2 cm，无滑移、无倾倒

---
2. 仿真环境

| 参数 | 设置 |
|------|------|
| 仿真器 | PyBullet |
| 时间步长 | dt_sim = 1/240 s（PyBullet 默认） |
| 控制频率 | WBC: 1 kHz（每步仿真 4 个物理步长） |
| MPC 重求解周期 | 50 ms（每 50 个 WBC 周期触发一次 MPC） |
| 重力 | [0, 0, -9.81] |
| 地面 | 水平平面，摩擦系数 $\mu = 0.8$ |

---
3. 机器人模型

- 模型：使用任意人形 URDF（如 humanoid.urdf 或 Atlas 简化版）
- 自由度：浮基（6）+ 关节（$\geq 12$）
- 关键假设（简化实验）：
  - 初始状态：双脚站立，质心在支撑多边形中心
  - t = 0.5 s 时：抬起一条腿（如左腿），进入单腿支撑
  - t = 0.5 s 至 t = 3.0 s：保持右腿单腿站立
  - 抬腿动作不由 MPC/WBC 规划，直接通过关节位置指令完成（或预设轨迹），实验核心验证单腿站立相的稳定控制

---
4. MPC 配置

4.1 状态与控制

| | 定义 | 维度 |
|---|---|---|
| 状态 $x$ | $[c, \dot{c}, L]^\top$ | 9 |
| 控制 $u$ | 接触力 $f \in \mathbb{R}^3$（仅支撑足） | 3 |

单支撑相只有 1 个接触点，因此控制输入只有 3 维（而非双足的 6 维）。

4.2 参考轨迹

- 期望 CoM 位置：$c_{\text{ref}} = [0, 0, h_{\text{com}}]^\top$（固定在支撑足正上方）
- 期望 CoM 速度：$\dot{c}_{\text{ref}} = [0, 0, 0]^\top$
- 期望角动量：$L_{\text{ref}} = [0, 0, 0]^\top$

4.3 动力学线性化

- 固定 $c_{\text{ref}}$ 计算 $B_k$ 矩阵中的 $(p_{\text{foot}} - c_{\text{ref}})$ 项
- 离散时间 $T_s = 0.05$ s（MPC 周期）
- 预测时域 $N = 10$（预测 0.5 s）

4.4 优化问题（QP）

$$\min_{x_{0:N}, u_{0:N-1}} \sum_{k=0}^{N-1} \left( \|x_k - x_{\text{ref}}\|Q^2 + \|u_k\|_R^2 \right) + \|x_N - x{\text{ref}}\|_{Q_N}^2$$

约束：
- 初始状态：$x_0 = \hat{x}$（当前状态估计）
- 动力学：$x_{k+1} = A_d x_k + B_d u_k + d_d$
- 摩擦锥（4 面线性近似）：$|f_x| \leq \mu f_z$, $|f_y| \leq \mu f_z$, $f_z \geq 0$
- ZMP = CoM（单点支撑）：隐式满足，无需额外约束

权重建议：
- $Q = \text{diag}(100, 100, 100, 1, 1, 1, 1, 1, 1)$（位置重罚）
- $R = \text{diag}(0.001, 0.001, 0.001)$（力小正则）
- $Q_N = Q$

4.5 求解器

- 使用 osqp 或 cvxpy
- 每次 MPC 求解输出：$\{x_k^{\text{ref}}\}_{k=0}^N$ 和 $u_0^{\text{ref}}$（仅当前步的控制力）

---
5. WBC 配置

5.1 决策变量

$$z = [\dot{v}; f] \in \mathbb{R}^{(n+6) + 3}$$

不直接优化 $\tau$，通过动力学方程后验计算：$\tau = S(M\dot{v} + C - J_c^\top f)$

5.2 目标函数

$$\min_z \|J_c \dot{v} + \dot{J}c v - \ddot{c}{\text{des}}\|{W_1}^2 + \|J_L \dot{v} - \dot{L}{\text{des}}\|{W_2}^2 + \|f - f{\text{ref}}\|{W_3}^2 + \|\dot{v}\|{W_4}^2$$

其中：
- $\ddot{c}{\text{des}} = \ddot{c}{\text{ref}} + K_p^c (c_{\text{ref}} - c_{\text{est}}) + K_d^c (\dot{c}{\text{ref}} - \dot{c}{\text{est}})$
- $\dot{L}{\text{des}} = \dot{L}{\text{ref}} + K_p^L (L_{\text{ref}} - L_{\text{est}}) + K_d^L (\dot{L}{\text{ref}} - \dot{L}{\text{est}})$

权重建议：
- $W_1 = 100 \cdot I_3$（CoM 跟踪优先）
- $W_2 = 10 \cdot I_3$（角动量跟踪）
- $W_3 = 0.1 \cdot I_3$（接触力软参考）
- $W_4 = 0.01 \cdot I_{n+6}$（最小化加速度）

PD 增益建议：
- $K_p^c = 100$, $K_d^c = 20$
- $K_p^L = 10$, $K_d^L = 2$

5.3 硬约束

| 约束 | 数学形式 |
|------|----------|
| 浮基动力学 | $M\dot{v} + C = S^\top \tau + J_c^\top f$ |
| 无滑动接触（固定足） | $J_c \dot{v} + \dot{J}_c v = 0$ |
| 摩擦锥 | $f \in \mathcal{K}$（4 面线性近似） |
| 关节力矩限幅 | $\tau_{\min} \leq \tau \leq \tau_{\max}$（通过 $\tau$ 表达式代入）|

5.4 求解器

- osqp 或 qpSWIFT
- 注意：约束中的 $\tau$ 需替换为 $S(M\dot{v} + C - J_c^\top f)$，使不等式约束成为 $\dot{v}$ 和 $f$ 的线性函数

---
6. 实验步骤

| 步骤 | 操作 | 验证点 |
|------|------|--------|
| 1 | 加载 URDF，设置初始姿势（双脚站立） | 模型加载成功，关节初始角合理 |
| 2 | 实现状态估计模块 | 从 PyBullet 读取 $q, v$，计算 $c, \dot{c}, L$ |
| 3 | 实现 Centroidal MPC | 固定单支撑模式，用 osqp 求解 QP |
| 4 | 实现 WBC QP | 构造目标函数和约束矩阵，输出 $\tau$ |
| 5 | 闭环运行 3 秒 | MPC 每 50 ms 更新，WBC 每 1 ms 发送力矩指令 |
| 6 | 数据记录与可视化 | 记录 $c, c_{\text{ref}}, f, \tau$，事后绘图 |

---
7. 评估指标

| 指标 | 目标值 | 说明 |
|------|--------|------|
| CoM 位置 RMSE | < 0.02 m | 相对 $c_{\text{ref}}$ 的偏差 |
| 支撑足滑移 | < 0.005 m | 水平位移 |
| MPC 求解时间 | < 20 ms | 保证实时性 |
| WBC 求解时间 | < 0.5 ms | 保证 1 kHz |
| QP 可行性率 | 100% | 所有周期均有解 |

---
8. 预期现象与调试提示

| 现象 | 可能原因 | 调试方向 |
|------|----------|----------|
| CoM 缓慢漂移 | $K_p^c$ 太小或 MPC 权重 $Q$ 不够大 | 增大位置跟踪权重 |
| 高频抖动 | WBC 求解的 $\tau$ 跳变 | 增大 $W_4$（最小化加速度），或降低 $K_d$ |
| QP 不可行 | 摩擦锥太紧或力矩限幅冲突 | 检查接触 Jacobian 是否正确，放松摩擦系数 |
| 仿真发散 | 接触力过大或方向错误 | 检查摩擦锥方向（$f_z \geq 0$），检查 $B_d$ 矩阵符号 |

---
9. 代码结构建议（供参考）

week10_single_leg_stand/
├── main.py              # 主循环：MPC + WBC + PyBullet
├── mpc.py               # Centroidal MPC QP 构造与求解
├── wbc.py               # WBC QP 构造与求解
├── robot_model.py       # URDF 加载，Jacobians，动力学量计算
├── state_estimator.py   # 从仿真读取状态，计算 centroidal quantities
├── config.py            # 所有参数集中配置
└── utils.py             # 绘图、日志、摩擦锥辅助函数


---
这个配置足够你开始编写代码了吗？有需要调整或补充的地方（比如是否加入双足切换、是否用具体某个 URDF 模型）？