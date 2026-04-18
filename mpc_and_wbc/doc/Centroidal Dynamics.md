Centroidal Dynamics 推导脉络：从全身动力学到 MPC-WBC 接口

---
1. 全身动力学（出发点）

浮基机器人的动力学方程：

$$M(q)\dot{v} + C(q, v) = S^\top \tau + \sum_i J_{c,i}^\top f_i$$

- $q \in \mathbb{R}^{n+6}$：广义坐标（6 维浮基 + $n$ 个关节）
- $v$：广义速度
- $M$：质量矩阵
- $S$：选择矩阵（仅关节有驱动力）
- $f_i$：第 $i$ 个接触点的接触力

问题：这个方程有 $n+6$ 个自由度。MPC 直接优化它？状态空间维度过高，实时求解不可行。

---
2. 质心动量的定义

定义质心位置 $p_{\text{com}}$ 和质心动量：

$$\text{线动量:} \quad h_{\text{lin}} = m \dot{p}_{\text{com}}$$

$$\text{角动量:} \quad h_{\text{ang}} = \sum_j (r_j - p_{\text{com}}) \times (m_j \dot{r}_j)$$

合并为 centroidal momentum：
$$h = \begin{bmatrix} h_{\text{lin}} \\ h_{\text{ang}} \end{bmatrix} \in \mathbb{R}^6$$

关键数学事实：存在 Centroidal Momentum Matrix $A(q)$，使得：

$$h = A(q) \, v$$

$A(q)$ 是 $6 \times (n+6)$ 的矩阵，它把全身广义速度映射到 6 维质心动量。

---
3. 核心推导：对时间求导

$$ \dot{h} = \dot{A}v + A\dot{v} $$

将全身动力学方程中的 $M\dot{v}$ 项与 $A$ 联系起来。经过推导（利用 $A$ 与 $M$ 的特定关系），得到惊人的简化：

$$\dot{h}_{\text{lin}} = m g + \sum_i f_i$$

$$\dot{h}{\text{ang}} = \sum_i (p_i - p{\text{com}}) \times f_i$$

这就是 Centroidal Dynamics —— 仅用 6 个方程描述：

$$\dot{h} = \begin{bmatrix} mg + \sum f_i \\ \sum (p_i - p_{\text{com}}) \times f_i \end{bmatrix}$$

---
4. 为什么这改变了一切？

| 特性 | 全身动力学 | Centroidal Dynamics |
|------|-----------|---------------------|
| 维度 | $n+6$（30+） | 6 |
| 驱动变量 | 关节力矩 $\tau$ | 接触力 $f$ |
| 内部运动耦合 | 强耦合 | 完全解耦 |
| 实时优化 | 不可行 | 可行 |

核心洞察：centroidal dynamics 不依赖于关节角 $q_{\text{joint}}$ 和关节力矩 $\tau$。它只关心：
- 质心在哪里（$p_{\text{com}}$）
- 接触力多大（$f_i$）

这意味着：质心运动可以由接触力单独决定，与"身体怎么扭"无关。

---
5. 从数学到架构：MPC-WBC 接口

这就是现代分层的理论根基：

┌─────────────┐         6-dim traj         ┌─────────────┐
│    MPC      │ ── h_ref, p_com_ref ─────→ │    WBC      │
│  (optimizes │      contact forces f_ref   │  (enforces  │
│   centroidal│                             │   centroidal│
│   dynamics) │                             │   constraint│
└─────────────┘                             └──────┬──────┘
      ↑                                            │
      └────────── h_actual, p_com_actual ──────────┘
                        (反馈)


WBC 的 QP 中，centroidal dynamics 成为一个约束：

$$A(q)\dot{v} + \dot{A}v = \dot{h}_{\text{ref}}$$

这保证了：无论关节怎么分配，全身执行出来的质心动量变化必须等于 MPC 的指令。

---
6. 等价形式：Linear Inverted Pendulum

你之前学的 LIPM 其实是 centroidal dynamics 的一个特例：

- 假设角动量为零（$h_{\text{ang}} = 0$）
- 假设 CoM 高度恒定（$z = z_0$）
- 则线动量方程退化为：

$$\ddot{x}{\text{com}} = \frac{g}{z_0}(x{\text{com}} - x_{\text{zmp}})$$

LIPM-MPC 本质上就是在优化 centroidal dynamics 的简化版本。现在你知道为什么 MPC 输出 CoM 轨迹是"合理"的——它就是在优化质心动量的演化。

---
下一步建议

理解了这个推导后，读 Dai et al., 2014 的 Section 2 会非常快。论文的核心贡献是：把 centroidal dynamics 显式地作为一个低维动力学模型用于轨迹优化，而不是像之前那样把质心约束当作全身优化的附加项。

要不要我继续推导 $A(q)$ 的具体构造方式，或者直接进入 WBC 中 centroidal constraint 的 QP  formulation？