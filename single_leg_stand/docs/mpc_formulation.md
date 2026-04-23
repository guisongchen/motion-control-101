Centroidal Dynamics MPC Formulation

---
1. 状态与控制输入

取状态：
$$x = \begin{bmatrix} c \\ \dot{c} \\ L \end{bmatrix} \in \mathbb{R}^9$$

- $c = p_{\text{com}} \in \mathbb{R}^3$：质心位置
- $\dot{c} \in \mathbb{R}^3$：质心速度
- $L = h_{\text{ang}} \in \mathbb{R}^3$：关于质心的角动量

控制输入：
$$u = \begin{bmatrix} f_1 \\ \vdots \\ f_{n_c} \end{bmatrix} \in \mathbb{R}^{3n_c}$$

所有接触点的接触力。

---
2. 连续时间动力学

$$\dot{x} = \begin{bmatrix} \dot{c} \\ \ddot{c} \\ \dot{L} \end{bmatrix} = \begin{bmatrix} \dot{c} \\ g + \frac{1}{m}\sum_{i=1}^{n_c} f_i \\ \sum_{i=1}^{n_c} (p_i - c) \times f_i \end{bmatrix}$$

注意第三项：$(p_i - c) \times f_i$ 中 $c$ 是状态量，这使系统关于状态-控制对双线性（bilinear）。

---
3. 线性化处理（MPC 实用策略）

直接求解非线性 MPC 计算量大。工程上最常见的处理：

固定参考 CoM 位置 $c_{\text{ref}}$（来自当前状态估计或上层轨迹），则：
$$\dot{L} \approx \sum_{i=1}^{n_c} (p_i - c_{\text{ref}}) \times f_i$$

此时系统关于 $u$ 线性，关于 $x$ 也是线性的（因为 $\dot{L}$ 不再显式依赖 $c$）。

连续时间线性形式：
$$\dot{x} = A x + B_k u + d$$

其中：
$$A = \begin{bmatrix} 0 & I & 0 \\ 0 & 0 & 0 \\ 0 & 0 & 0 \end{bmatrix}, \quad d = \begin{bmatrix} 0 \\ g \\ 0 \end{bmatrix}$$

$$B_k = \begin{bmatrix} 0 & \cdots & 0 \\ \frac{1}{m}I & \cdots & \frac{1}{m}I \\ [p_1 - c_{\text{ref}}]\times & \cdots & [p{n_c} - c_{\text{ref}}]_\times \end{bmatrix} \in \mathbb{R}^{9 \times 3n_c}$$

$[\cdot]_\times$ 为斜对称矩阵（叉乘算子）。$B_k$ 随时间变化因为 $p_i$ 和接触状态由 gait schedule 预设。

---
4. 离散时间预测模型

欧拉离散化（采样时间 $T_s$）：

$$x_{k+1} = \underbrace{(I + T_s A)}{A_d} x_k + \underbrace{T_s B_k}{B_{d,k}} u_k + \underbrace{T_s d}_{d_d}$$

即：
$$x_{k+1} = A_d x_k + B_{d,k} u_k + d_d$$

这是时变线性系统，标准 MPC 可直接处理。

---
5. MPC 优化问题

决策变量：$\mathbf{X} = \{x_0, ..., x_N\}$, $\mathbf{U} = \{u_0, ..., u_{N-1}\}$

目标函数：
$$\min_{\mathbf{X}, \mathbf{U}} \sum_{k=0}^{N-1} \left( \|x_k - x_k^{\text{ref}}\|Q^2 + \|u_k\|_R^2 \right) + \|x_N - x_N^{\text{ref}}\|{Q_N}^2$$

约束条件：

| 约束 | 数学形式 | 说明 |
|------|----------|------|
| 初始状态 | $x_0 = \hat{x}$ | 当前状态估计 |
| 动力学 | $x_{k+1} = A_d x_k + B_{d,k} u_k + d_d$ | 离散 centroidal dynamics |
| 摩擦锥 | $\|f_{i,k}^{xy}\|2 \leq \mu f{i,k}^z,\; f_{i,k}^z \geq f_{\min}$ | 防止滑移，保留法向力 |
| 力矩边界（可选） | $u_k \in \mathcal{U}$ | 关节力矩通过接触 Jacobian 映射的近似 |
| CoP/ZMP（可选） | $\frac{\sum p_i f_i^z}{\sum f_i^z} \in \mathcal{P}$ | 压力中心在支撑多边形内 |

摩擦锥是二阶锥约束（SOCP），如果用线性多面体近似则可退化为 QP。

---
6. 与 LIPM-MPC 的关系（Week 9 到 Week 10 的桥梁）

LIPM 是 centroidal dynamics 的强约束特例：

| 假设 | LIPM | Full Centroidal |
|------|------|-----------------|
| CoM 高度 | $c_z = h$（常数） | 自由变化 |
| 角动量 | $L = 0$（常数） | 自由变化 |
| 接触力 | 仅决定水平加速度 | 同时影响线/角动量 |

在 LIPM 假设下：
$$\ddot{c}{xy} = \frac{g}{h}(c{xy} - p_{\text{zmp}})$$

这正是 centroidal dynamics 中 $\ddot{c} = g + \frac{1}{m}\sum f_i$ 在 $c_z = h$ 和 $L = 0$ 约束下的降维形式。

结论：Week 9 的 LIPM-MPC 只优化了 CoM 水平位置，Week 10 的 centroidal MPC 同时优化 CoM 3D 轨迹 + 角动量 + 接触力分配，但数学结构（时变线性 MPC + 锥约束）完全一致。

---
7. 这个 MPC 输出什么？WBC 怎么接？

MPC 求解后输出：
- 参考轨迹：$c_{0:N}^{\text{ref}}$, $\dot{c}_{0:N}^{\text{ref}}$, $L_{0:N}^{\text{ref}}$
- 参考接触力：$f_{0:N-1}^{\text{ref}}$

WBC 在 1 kHz 下的任务：
求解关节力矩 $\tau$ 和实际接触力 $f$，使全身动力学产生的 centroidal 行为 匹配 MPC 的参考，同时处理关节极限、力矩饱和等 MPC 未考虑的约束。

这就是两层之间的数学接口。

---
下一步你想深入 WBC 的 QP formulation（如何把 MPC 的参考变成关节力矩），还是先做 PyBullet 单腿站立实验 来验证这个 MPC→WBC 数据流？