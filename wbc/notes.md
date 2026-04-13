目录已就位。那我们从底层逻辑开始，把 Week 8 的理论骨架搭起来。

---
Week 8：WBC 理论基础

1. 动机：为什么要从关节空间跳到任务空间？

前七周你主要在关节空间或简化模型里处理问题：
- LIPM/ZMP：把整个人抽象成一个带质心的倒摆，在水平面规划 CoM 轨迹。
- 站立平衡：在关节空间里规划关节角度，或者把质心投影到支撑多边形内。

但这些方法有个共同的问题：高层意图是笛卡尔空间的（"左脚放在这里"、"躯干保持水平"），而执行层是关节空间的。需要一个映射把笛卡尔指令转换成关节指令。

这个映射就是 Operational Space Control / Whole-Body Control (WBC) 的核心。它的思想最早由 Khatib (1987) 提出，现代人形机器人（Atlas, ANYmal, TALOS）的控制栈基本都基于此。

---
2. 速度级映射：雅可比矩阵

设机器人有 $n$ 个自由度（人形通常 $n \ge 30$），关节位置为 $\mathbf{q} \in \mathbb{R}^n$。

假设你关心一个任务空间坐标 $\mathbf{x} \in \mathbb{R}^m$（例如足端位置，$m=3$），它是关节位置的函数：
$$
\mathbf{x} = f(\mathbf{q})
$$

对时间求导得到速度关系：
$$
\dot{\mathbf{x}} = \mathbf{J}(\mathbf{q}) \dot{\mathbf{q}}
$$
其中 $\mathbf{J} = \frac{\partial f}{\partial \mathbf{q}} \in \mathbb{R}^{m \times n}$ 就是任务雅可比矩阵。

速度级逆运动学（Velocity-level IK）

给定期望任务速度 $\dot{\mathbf{x}}^*$，求关节速度 $\dot{\mathbf{q}}$：
$$
\dot{\mathbf{q}} = \mathbf{J}^\dagger \dot{\mathbf{x}}^*
$$
其中 $\mathbf{J}^\dagger$ 是Moore-Penrose 伪逆：
$$
\mathbf{J}^\dagger = \mathbf{J}^\top (\mathbf{J} \mathbf{J}^\top)^{-1} \quad \text{(当 } m \le n \text{，即行满秩)}
$$

关键观察：当 $m < n$ 时，机器人是冗余的（redundant）。逆解不唯一。对于人形机器人，$n$ 通常远大于任务数，这意味着你可以在完成主任务的同时，用剩余的"自由度"去做别的事情（比如保持平衡、避障、节能姿态）。

---
3. 多任务协调：问题的核心

现代 WBC 的本质是同时满足多个任务，并处理它们之间的冲突。典型的任务栈（task hierarchy）：

| 优先级 | 任务 | 示例 |
|--------|------|------|
| 1 (最高) | 动力学一致性 / 接触约束 | 脚不滑动（严格约束） |
| 2 | 平衡 | CoM 跟踪、ZMP 约束 |
| 3 | 足端跟踪 | 摆动腿按轨迹落地 |
| 4 | 姿态 | 躯干保持直立 |
| 5 (最低) | 关节限幅 / 节能 | 避免奇异点、最小扭矩 |

问题来了：如果两个任务冲突了怎么办？

比如：摆动腿想去某个位置，但这个指令会导致 CoM 超出支撑多边形，违背平衡任务。

协调冲突的方法分为三大类：

---
4. 第一类：解析零空间投影（Hard Priority）

这是数学家最喜欢的方法，因为它把优先级严格编码进去。

核心思想

假设有两个任务：
- 主任务：$\dot{\mathbf{x}}_1 = \mathbf{J}_1 \dot{\mathbf{q}}$，维度 $m_1$
- 次任务：$\dot{\mathbf{x}}_2 = \mathbf{J}_2 \dot{\mathbf{q}}$，维度 $m_2$

先求解主任务：
$$
\dot{\mathbf{q}}_1 = \mathbf{J}_1^\dagger \dot{\mathbf{x}}_1^*
$$

再求解次任务，但必须不干扰主任务。这意味着次任务的解必须落在主任务的零空间（null space）里。零空间里的关节速度不改变主任务的输出：
$$
\mathbf{J}_1 \dot{\mathbf{q}}_N = 0
$$

零空间投影矩阵

投影到 $\mathbf{J}_1$ 零空间的矩阵：
$$
\mathbf{N}_1 = \mathbf{I} - \mathbf{J}_1^\dagger \mathbf{J}_1
$$

对于任意向量 $\mathbf{z}$，$\mathbf{N}_1 \mathbf{z}$ 都满足 $\mathbf{J}_1 (\mathbf{N}_1 \mathbf{z}) = 0$。

两级任务的解

$$
\dot{\mathbf{q}} = \dot{\mathbf{q}}_1 + \mathbf{N}_1 (\mathbf{J}_2 \mathbf{N}_1)^\dagger \left( \dot{\mathbf{x}}_2^* - \mathbf{J}_2 \dot{\mathbf{q}}_1 \right)
$$

拆解一下：
- 第一项 $\dot{\mathbf{q}}_1$：完成主任务
- 括号内 $(\dot{\mathbf{x}}_2^* - \mathbf{J}_2 \dot{\mathbf{q}}_1)$：次任务的残差（主任务已经做了多少）
- $(\mathbf{J}_2 \mathbf{N}_1)^\dagger$：在零空间里完成残差的能力
- 第二项整体：只在主任务不影响的方向上追次任务

推广到 $k$ 层

$$
\dot{\mathbf{q}} = \sum_{i=1}^{k} \mathbf{N}{i-1} (\mathbf{J}_i \mathbf{N}{i-1})^\dagger (\dot{\mathbf{x}}i^* - \mathbf{J}_i \dot{\mathbf{q}}{i-1})
$$
其中 $\mathbf{N}_0 = \mathbf{I}$，$\dot{\mathbf{q}}_0 = 0$。

优点与缺点

| 优点 | 缺点 |
|------|------|
| 数学严格，优先级绝不违反 | 奇异点敏感：如果 $\mathbf{J}$ 降秩，伪逆爆炸 |
| 计算直观 | 硬优先级可能过于保守，低优先级任务完全饿死 |
| | 实现时要处理连续伪逆，数值上比较脆弱 |

工程上，纯解析零空间投影现在用得少了，人形上更常见的是下面两种方法。

---
5. 第二类：加权阻尼最小二乘（Weighted Damped Least Squares, WLN / DLS）

这是 softened priority 的思路：不严格区分谁比谁高，而是给不同任务加权，通过优化一个加权目标函数来协调。

基本问题

把多个任务堆成一个大的最小二乘问题：
$$
\min_{\dot{\mathbf{q}}} \| \mathbf{W}^{1/2} (\mathbf{J} \dot{\mathbf{q}} - \dot{\mathbf{x}}^*) \|^2_2
$$

其中：
- $\mathbf{J} = \begin{bmatrix} \mathbf{J}_1 \\ \mathbf{J}_2 \\ \vdots \end{bmatrix}$ 是所有任务的堆叠雅可比
- $\dot{\mathbf{x}}^ = \begin{bmatrix} \dot{\mathbf{x}}_1^ \\ \dot{\mathbf{x}}_2^* \\ \vdots \end{bmatrix}$ 是堆叠任务速度
- $\mathbf{W} = \text{diag}(w_1 \mathbf{I}, w_2 \mathbf{I}, \dots)$ 是任务权重矩阵

解析解

$$
\dot{\mathbf{q}} = (\mathbf{J}^\top \mathbf{W} \mathbf{J})^{-1} \mathbf{J}^\top \mathbf{W} \dot{\mathbf{x}}^*
$$

这是加权伪逆。当某个任务权重 $w_i$ 很大时，优化器会更尊重这个任务。

阻尼项（Damped Least Squares, Levenberg-Marquardt）
纯最小二乘在运动学奇异点（kinematic singularity，即 $\mathbf{J}$ 接近降秩）时会爆炸。经典例子：手臂完全伸直时，末端沿伸直方向的速度能力急剧下降。

解决办法：在 $\mathbf{J}^\top \mathbf{W} \mathbf{J}$ 上加一个小的正则项：
$$
\dot{\mathbf{q}} = (\mathbf{J}^\top \mathbf{W} \mathbf{J} + \lambda^2 \mathbf{I})^{-1} \mathbf{J}^\top \mathbf{W} \dot{\mathbf{x}}^*
$$

$\lambda$ 是阻尼系数。它的作用是：
- 在远离奇异点时，$\lambda^2 \mathbf{I}$ 几乎不影响结果；
- 在接近奇异点时，它保证矩阵可逆，让关节速度不会飙到天上去。

更高级的做法是用自适应阻尼（adaptive damping），让 $\lambda$ 随奇异值变化，例如：
$$
\lambda_i = 
\begin{cases}
0 & \text{if } \sigma_i \ge \epsilon \\
\lambda_{\max} \left(1 - \frac{\sigma_i}{\epsilon}\right)^2 & \text{if } \sigma_i < \epsilon
\end{cases}
$$

加权与阻尼的区别

| | 作用 | 类比 |
|--|------|------|
| 加权 $\mathbf{W}$ | 协调不同任务的重要性 | "平衡任务权重是 1000，足端跟踪权重是 10" |
| 阻尼 $\lambda$ | 处理同一个任务内部的奇异点问题 | "当手臂伸直时，别让关节速度发疯" |

这就是练习 2 的核心：手推 $(\mathbf{J}^\top \mathbf{W} \mathbf{J} + \lambda^2 \mathbf{I})^{-1} \mathbf{J}^\top \mathbf{W}$ 这个表达式。

---
6. 第三类：基于优化的 WBC（QP-WBC）

这是现代主流，也是 Week 8 练习 3 和整个 Phase 3 的重点。

为什么需要 QP？

最小二乘只能处理软约束（目标函数）。但机器人有很多硬约束是绝对不能违反的：
- 摩擦锥约束：地面反作用力必须在摩擦锥内，否则脚会滑动
- 关节限幅：关节角度、速度、扭矩必须在其物理极限内
- ZMP 约束：保证机器人不摔倒

这些硬约束不能简单地用加权目标函数来表达。它们需要用不等式约束来严格保证。

QP 的标准形式

标准的二次规划（Quadratic Programming）：
$$
\begin{aligned}
\min_{\mathbf{v}} \quad & \mathbf{v}^\top \mathbf{H} \mathbf{v} + \mathbf{g}^\top \mathbf{v} \\
\text{s.t.} \quad & \mathbf{A} \mathbf{v} = \mathbf{b} \\
& \mathbf{C} \mathbf{v} \le \mathbf{d}
\end{aligned}
$$

在 WBC 中，决策变量 $\mathbf{v}$ 通常是广义加速度 $\ddot{\mathbf{q}}$ 和接触力 $\mathbf{f}$：
$$
\mathbf{v} = \begin{bmatrix} \ddot{\mathbf{q}} \\ \mathbf{f} \end{bmatrix}
$$

等式约束（动力学一致性）

牛顿-欧拉方程必须严格满足：
$$
\mathbf{M} \ddot{\mathbf{q}} + \mathbf{h} = \mathbf{S}^\top \boldsymbol{\tau} + \mathbf{J}_c^\top \mathbf{f}
$$

不等式约束

1. 摩擦锥约束：地面接触力必须在摩擦锥内
   $$
   \mathbf{f} \in \text{friction cone}
   $$
   通常用线性多面锥近似：$\mathbf{C}_f \mathbf{f} \le 0$

2. 关节力矩限幅
   $$
   \boldsymbol{\tau}{\min} \le \boldsymbol{\tau} \le \boldsymbol{\tau}{\max}
   $$

3. 关节加速度限幅
   $$
   \ddot{\mathbf{q}}{\min} \le \ddot{\mathbf{q}} \le \ddot{\mathbf{q}}{\max}
   $$

目标函数（任务堆叠）

把多个任务编码为二次代价：
$$
\min \sum_i w_i \| \mathbf{J}_i \ddot{\mathbf{q}} + \dot{\mathbf{J}}_i \dot{\mathbf{q}} - \ddot{\mathbf{x}}_i^* \|^2
$$

比如：
- CoM 加速度跟踪
- 摆动足加速度跟踪
- 躯干角加速度跟踪
- 最小关节力矩（正则项）

为什么 QP-WBC 是现代标准？

| 特性 | 解析零空间 | 加权最小二乘 | QP-WBC |
|------|-----------|-------------|--------|
| 多任务 | 是 | 是 | 是 |
| 硬优先级 | 严格 | 无 | 可通过大权重近似 |
| 硬约束（不等式） | 否 | 否 | 是 |
| 奇异点稳定 | 需额外处理 | 阻尼可处理 | Hessian 正定即可 |
| 计算效率 | 快 | 很快 | 快（1-5 ms） |
| 工程使用 | 较少 | 中等 | 主流 |

---
7. 从速度级到加速度级：一步之遥

上面讲的解析零空间投影和 DLS 都是速度级的（给定 $\dot{\mathbf{x}}^$，求 $\dot{\mathbf{q}}$）。但现代 WBC 通常是加速度级的（给定 $\ddot{\mathbf{x}}^$，求 $\ddot{\mathbf{q}}$）。

为什么？

因为对 $\dot{\mathbf{x}} = \mathbf{J} \dot{\mathbf{q}}$ 再求导一次：
$$
\ddot{\mathbf{x}} = \mathbf{J} \ddot{\mathbf{q}} + \dot{\mathbf{J}} \dot{\mathbf{q}}
$$

加速度级的控制可以直接与动力学方程耦合，从而同时优化运动学和接触力。这就是 QP-WBC 的决策变量通常选 $\ddot{\mathbf{q}}$ 的原因。

速度级方法仍然有它的价值（比如简单的视觉伺服、动画），但在人形运控的物理世界里，加速度级是更自然的接口。

---
小结：Week 8 的知识地图

关节空间 q ──► 雅可比 J ──► 任务空间 x
                    │
                    ├──► 解析零空间投影：硬优先级，严格但脆弱
                    ├──► 加权阻尼最小二乘：软优先级，数值稳定
                    └──► QP-WBC：带硬约束的优化，现代主流


接下来你想深入哪个部分？

- 推导练习：手推练习 2 的阻尼最小二乘解，确认每个矩阵的维度？
- 直观理解：用几何方式解释零空间投影到底是怎么回事？
- 直接编码：搭一个 2D 三连杆臂，跑解析零空间 vs 加权最小二乘的对比？

----

好问题，这帮你把 Weeks 1-7 和 Week 8 的断层补上。

一句话：IK 是 WBC 的一个子集，WBC 是 IK 在动力学世界里的扩展。

---
1. 层次关系

| 层级 | 求解什么 | 关注什么 | 对应方法 |
|------|---------|---------|---------|
| 位置级 IK | 给定 $\mathbf{x}^*$，求 $\mathbf{q}$ | 末端位姿、关节角度 | 解析解、数值迭代（Jacobian transpose, Newton-Raphson） |
| 速度级 IK | 给定 $\dot{\mathbf{x}}^*$，求 $\dot{\mathbf{q}}$ | 运动学映射 | 雅可比伪逆、零空间投影、DLS |
| 加速度级 WBC | 给定 $\ddot{\mathbf{x}}^*$，求 $\ddot{\mathbf{q}}$ 和 $\mathbf{f}$ | 运动学 + 动力学 + 接触力 + 约束 | QP-WBC |

Weeks 1-7 你学过的内容：
- 站立平衡里的"关节角度规划让质心落在支撑多边形内" → 这是位置级 IK 思想的应用（虽然用的是简化模型）。
- LIPM 规划 CoM 轨迹 → 这是高层运动规划，不直接涉及关节怎么动。

Week 8 的 WBC 告诉你：实际机器人不是只关心"末端到没到那个点"，还要关心"全身怎么协调多个任务"，以及"动力学方程、摩擦力、关节力矩是否允许你这么干"。

---
2. 核心区别：IK 只回答"能不能到"，WBC 还要回答"怎么到才物理可行"

例子：机器人伸手够桌上的杯子

纯 IK 的视角：
- 输入：杯子位置 $\mathbf{x}_{\text{cup}}$
- 输出：手臂关节角度 $\mathbf{q}_{\text{arm}}$
- 问题：只算手臂，躯干固定。如果杯子太远，IK 无解。

WBC 的视角：
- 输入：杯子位置、同时要保持平衡（CoM 在支撑多边形内）、躯干不倾斜太大、足部站立不动
- 输出：全身关节加速度 $\ddot{\mathbf{q}}$ + 地面接触力 $\mathbf{f}$
- 问题：如果手臂够不到，允许躯干前倾来补偿，但前倾会导致 CoM 移动，所以必须同时调整髋和腿来保持平衡。所有这些都由同一个优化问题（QP）来协调。

所以 WBC 不仅做 IK，它还自动解决了"为了完成这个末端任务，身体其他部位应该怎么配合"的问题。

---
3. 联系：WBC 里的运动学部分就是多任务 IK

你 Week 8 练习 1 做的"三连杆臂主任务跟踪+次任务保持肘关节角度"，本质上就是一个多任务速度级 IK。

- 主任务：末端跟踪圆 → $\mathbf{J}_1 \dot{\mathbf{q}} = \dot{\mathbf{x}}_1^*$
- 次任务：肘关节角度 → $\mathbf{J}_2 \dot{\mathbf{q}} = \dot{\mathbf{x}}_2^*$

用解析零空间投影求解，这就是一个带优先级的速度级 IK 算法。

而 QP-WBC（练习 3）把这个思想推广了两步：
1. 从速度级升维到加速度级（引入 $\ddot{\mathbf{q}}$）；
2. 把动力学方程和不等式约束硬编码进优化问题。

---
4. 实际工程中的分界

| 场景 | 用 IK 就够了 | 必须用 WBC |
|------|-------------|-----------|
| 离线动画/机械臂轨迹生成 | 只需要末端位姿，不考虑力和约束 | |
| 机械臂视觉伺服 | 速度级伪逆就能跑 | |
| 固定基座机械臂（如工厂机器人） | 位置/速度级 IK 足够 | 除非要考虑碰撞避障、力控 |
| 人形机器人行走/操作 | | 必须。脚在滑动、躯干在晃动、多任务冲突，IK 不管物理 |
| 足式机器人不平地面 | | 必须。接触力必须满足摩擦锥约束 |

---
一句话总结

IK 关心"关节怎么动才能让末端到达目标"；WBC 关心"全身怎么动才能同时满足多个任务，并且不违反物理规律"。

Week 8 的练习 1 和 2 本质上是多任务 IK；练习 3 的 QP 才真正进入了 WBC 的完整形态（因为它把动力学和硬约束加进来了）。

你现在对这个关系的理解，可以支撑你继续往下走了。接下来想：
- 手推练习2的阻尼最小二乘，还是
- 直接搭三连杆臂的代码跑起来看效果？

---

练习 2：手推阻尼最小二乘（DLS/ WLN）

我们从标准最小二乘出发，一步一步引入阻尼项，最后得到通用的加权阻尼最小二乘公式。

---
第一步：标准最小二乘问题

给定任务速度 $\dot{\mathbf{x}}^* \in \mathbb{R}^m$，求关节速度 $\dot{\mathbf{q}} \in \mathbb{R}^n$：

$$
\min_{\dot{\mathbf{q}}} \frac{1}{2} \| \mathbf{J} \dot{\mathbf{q}} - \dot{\mathbf{x}}^* \|^2
$$

展开范数：

$$
L = \frac{1}{2} (\mathbf{J} \dot{\mathbf{q}} - \dot{\mathbf{x}}^)^\top (\mathbf{J} \dot{\mathbf{q}} - \dot{\mathbf{x}}^)
$$

对 $\dot{\mathbf{q}}$ 求梯度并令其为零：

$$
\nabla_{\dot{\mathbf{q}}} L = \mathbf{J}^\top (\mathbf{J} \dot{\mathbf{q}} - \dot{\mathbf{x}}^*) = \mathbf{0}
$$

$$
\mathbf{J}^\top \mathbf{J} \, \dot{\mathbf{q}} = \mathbf{J}^\top \dot{\mathbf{x}}^*
$$

这就是正规方程（normal equations）。当 $n > m$ 且 $\mathbf{J}$ 行满秩时，解为：

$$
\dot{\mathbf{q}} = (\mathbf{J}^\top \mathbf{J})^{-1} \mathbf{J}^\top \dot{\mathbf{x}}^ = \mathbf{J}^\dagger \dot{\mathbf{x}}^
$$

问题：如果在某个姿态下 $\mathbf{J}$ 接近降秩（奇异点），$\mathbf{J}^\top \mathbf{J}$ 的某些特征值趋于零，其逆矩阵会把很小的任务速度误差放大成巨大的关节速度。

---
第二步：引入阻尼项

为了抑制奇异点处的关节速度爆炸，我们在目标函数中加入对 $\dot{\mathbf{q}}$ 的惩罚：

$$
\min_{\dot{\mathbf{q}}} \frac{1}{2} \| \mathbf{J} \dot{\mathbf{q}} - \dot{\mathbf{x}}^* \|^2 + \frac{\lambda^2}{2} \| \dot{\mathbf{q}} \|^2
$$

其中 $\lambda > 0$ 是阻尼系数。

展开并求梯度：

$$
L = \frac{1}{2} (\mathbf{J} \dot{\mathbf{q}} - \dot{\mathbf{x}}^)^\top (\mathbf{J} \dot{\mathbf{q}} - \dot{\mathbf{x}}^) + \frac{\lambda^2}{2} \dot{\mathbf{q}}^\top \dot{\mathbf{q}}
$$

$$
\nabla_{\dot{\mathbf{q}}} L = \mathbf{J}^\top (\mathbf{J} \dot{\mathbf{q}} - \dot{\mathbf{x}}^*) + \lambda^2 \dot{\mathbf{q}} = \mathbf{0}
$$

整理：

$$
(\mathbf{J}^\top \mathbf{J} + \lambda^2 \mathbf{I}_n) \dot{\mathbf{q}} = \mathbf{J}^\top \dot{\mathbf{x}}^*
$$

于是得到阻尼最小二乘解：

$$
\boxed{ \dot{\mathbf{q}} = (\mathbf{J}^\top \mathbf{J} + \lambda^2 \mathbf{I}_n)^{-1} \mathbf{J}^\top \dot{\mathbf{x}}^* }
$$

---
第三步：引入任务权重

现在把多个任务堆叠起来，给不同任务分配不同权重。设权重矩阵 $\mathbf{W} \in \mathbb{R}^{m \times m}$ 为正定对角矩阵。

目标函数变为：

$$
\min_{\dot{\mathbf{q}}} \frac{1}{2} \| \mathbf{W}^{1/2} (\mathbf{J} \dot{\mathbf{q}} - \dot{\mathbf{x}}^*) \|^2 + \frac{\lambda^2}{2} \| \dot{\mathbf{q}} \|^2
$$

定义加权雅可比和加权任务速度：

$$
\tilde{\mathbf{J}} = \mathbf{W}^{1/2} \mathbf{J}, \quad \tilde{\dot{\mathbf{x}}} = \mathbf{W}^{1/2} \dot{\mathbf{x}}^*
$$

代入标准 DLS 公式：

$$
\dot{\mathbf{q}} = (\tilde{\mathbf{J}}^\top \tilde{\mathbf{J}} + \lambda^2 \mathbf{I}_n)^{-1} \tilde{\mathbf{J}}^\top \tilde{\dot{\mathbf{x}}}
$$

回代 $\tilde{\mathbf{J}}$ 和 $\tilde{\dot{\mathbf{x}}}$：

$$
\boxed{ \dot{\mathbf{q}} = (\mathbf{J}^\top \mathbf{W} \mathbf{J} + \lambda^2 \mathbf{I}_n)^{-1} \mathbf{J}^\top \mathbf{W} \dot{\mathbf{x}}^* }
$$

这就是加权阻尼最小二乘（Weighted Damped Least Squares, WLN）的通式。

---
第四步：等价的另一种形式（计算更高效）

当机器人冗余时（$n > m$），求 $n \times n$ 矩阵的逆很贵。利用矩阵逆引理可以推导出一个等价但只求 $m \times m$ 逆的公式。

从 $(\mathbf{J}^\top \mathbf{W} \mathbf{J} + \lambda^2 \mathbf{I}_n) \dot{\mathbf{q}} = \mathbf{J}^\top \mathbf{W} \dot{\mathbf{x}}^*$ 出发，左右两边同乘某个因子，可以验证：

$$
\boxed{ \dot{\mathbf{q}} = \mathbf{J}^\top \mathbf{W} (\mathbf{J} \mathbf{J}^\top \mathbf{W} + \lambda^2 \mathbf{I}_m)^{-1} \dot{\mathbf{x}}^* }
$$

验证等价性：
$$
(\mathbf{J}^\top \mathbf{W} \mathbf{J} + \lambda^2 \mathbf{I}_n) \mathbf{J}^\top \mathbf{W} = \mathbf{J}^\top \mathbf{W} \mathbf{J} \mathbf{J}^\top \mathbf{W} + \lambda^2 \mathbf{J}^\top \mathbf{W} = \mathbf{J}^\top \mathbf{W} (\mathbf{J} \mathbf{J}^\top \mathbf{W} + \lambda^2 \mathbf{I}_m)
$$
两边同时左乘 $(\mathbf{J}^\top \mathbf{W} \mathbf{J} + \lambda^2 \mathbf{I}_n)^{-1}$，右乘 $(\mathbf{J} \mathbf{J}^\top \mathbf{W} + \lambda^2 \mathbf{I}_m)^{-1}$，即得：
$$
(\mathbf{J}^\top \mathbf{W} \mathbf{J} + \lambda^2 \mathbf{I}_n)^{-1} \mathbf{J}^\top \mathbf{W} = \mathbf{J}^\top \mathbf{W} (\mathbf{J} \mathbf{J}^\top \mathbf{W} + \lambda^2 \mathbf{I}_m)^{-1}
$$

工程上：冗余机器人（人形、机械臂）通常 $n \gg m$，所以用 $m \times m$ 逆的形式计算量更小。

---
第五步：用 SVD 看阻尼到底做了什么
对 $\mathbf{J}$ 做 SVD：$\mathbf{J} = \mathbf{U} \mathbf{\Sigma} \mathbf{V}^\top$。

纯伪逆在速度空间的映射是：

$$
\mathbf{J}^\dagger = \mathbf{V} \mathbf{\Sigma}^\dagger \mathbf{U}^\top, \quad \text{其中 } \Sigma^\dagger_{ii} = \frac{1}{\sigma_i}
$$

阻尼最小二乘的映射变为：

$$
(\mathbf{J}^\top \mathbf{J} + \lambda^2 \mathbf{I})^{-1} \mathbf{J}^\top = \mathbf{V} \mathbf{D} \mathbf{U}^\top, \quad \text{其中 } D_{ii} = \frac{\sigma_i}{\sigma_i^2 + \lambda^2}
$$

对比：

| 奇异值大小 | 纯伪逆 $1/\sigma_i$ | 阻尼形式 $\frac{\sigma_i}{\sigma_i^2 + \lambda^2}$ |
|-----------|---------------------|--------------------------------------------------|
| $\sigma_i \gg \lambda$ | $\approx 1/\sigma_i$ | $\approx 1/\sigma_i$（几乎无影响） |
| $\sigma_i \approx \lambda$ | 很大 | $\approx 1/(2\lambda)$（被压制） |
| $\sigma_i \to 0$ | $\to \infty$ | $\to 0$（关节速度不会飙） |

物理意义：阻尼项 $\lambda$ 相当于在速度映射里设置了一个"截止频率"——对于机器人擅长的方向（大奇异值），几乎不改变行为；对于机器人即将失控的方向（小奇异值），自动降低响应幅度，换来数值稳定性。

---
第六步：维度检查

以 $\dot{\mathbf{q}} = (\mathbf{J}^\top \mathbf{W} \mathbf{J} + \lambda^2 \mathbf{I}_n)^{-1} \mathbf{J}^\top \mathbf{W} \dot{\mathbf{x}}^*$ 为例：

| 矩阵 | 维度 |
|------|------|
| $\mathbf{J}$ | $m \times n$ |
| $\mathbf{W}$ | $m \times m$ |
| $\mathbf{J}^\top \mathbf{W} \mathbf{J}$ | $n \times n$ |
| $\lambda^2 \mathbf{I}_n$ | $n \times n$ |
| $(\mathbf{J}^\top \mathbf{W} \mathbf{J} + \lambda^2 \mathbf{I}_n)^{-1}$ | $n \times n$ |
| $\mathbf{J}^\top \mathbf{W} \dot{\mathbf{x}}^*$ | $n \times 1$ |
| $\dot{\mathbf{q}}$ | $n \times 1$ ✅ |

以另一种形式 $\dot{\mathbf{q}} = \mathbf{J}^\top \mathbf{W} (\mathbf{J} \mathbf{J}^\top \mathbf{W} + \lambda^2 \mathbf{I}_m)^{-1} \dot{\mathbf{x}}^*$：

| 矩阵 | 维度 |
|------|------|
| $\mathbf{J} \mathbf{J}^\top \mathbf{W}$ | $m \times m$ |
| $(\mathbf{J} \mathbf{J}^\top \mathbf{W} + \lambda^2 \mathbf{I}_m)^{-1}$ | $m \times m$ |
| $\mathbf{J}^\top \mathbf{W} (\cdots)^{-1} \dot{\mathbf{x}}^*$ | $n \times 1$ ✅ |

---
一句话总结

阻尼最小二乘 = 在标准伪逆解的基础上，给关节速度加一个 $\ell_2$ 正则项，使得在奇异点附近解不会爆炸，代价是在该方向上跟踪精度略有下降。

没问题的话，下一步可以进入练习 1的三连杆臂代码实现，或者继续练习 3的 QP-WBC。你想先跑哪个？

---

好问题，这触及速度级控制的一个核心细节。

因为肘关节的期望角度是常数，它的“参考速度”就是 0。

———
解释

速度级 IK 的一般形式是：

$$
\dot{\mathbf{x}}^* = \underbrace{\dot{\mathbf{x}}{\text{des}}}{\text{参考速度}} + \underbrace{K_p (\mathbf{x}{\text{des}} - \mathbf{x}{\text{actual}})}_{\text{反馈修正}}
$$

这个式子对任何任务变量都成立，无论是末端位置还是关节角度。

主任务：末端跟踪圆形轨迹
圆的期望位置 $\mathbf{x}_{\text{des}}(t)$ 随时间变化，所以它有非零的参考速度：
$$
\dot{\mathbf{x}}_{\text{des}}(t) = \begin{bmatrix} -0.5\omega \sin(\omega t) \\ 0.5\omega \cos(\omega t) \end{bmatrix}
$$
因此完整指令是：
$$
\dot{\mathbf{x}}^* = \dot{\mathbf{x}}{\text{des}}(t) + K_p (\mathbf{x}{\text{des}} - \mathbf{x}_{\text{actual}})
$$

次任务：肘关节角度保持
设定 $\theta_{2,\text{des}} = 0.5$（常数），它不随时间变化：
$$
\dot{\theta}_{2,\text{des}} = \frac{d}{dt}(0.5) = 0
$$
所以完整指令其实是：
$$
\dot{\theta}2^* = \underbrace{0}{\text{参考速度}} + K_{\text{sec}} (\theta_{2,\text{des}} - \theta_{2,\text{actual}}) = K_{\text{sec}} (\theta_{2,\text{des}} - \theta_{2,\text{actual}})
$$

———
推广：如果肘关节也在跟踪一个时变轨迹呢？

比如你想让肘关节按正弦摆动：$\theta_{2,\text{des}}(t) = 0.5 + 0.2\sin(2t)$，那它的指令就必须包含参考速度项：

$$
\dot{\theta}2^* = \underbrace{0.4\cos(2t)}{\text{参考速度}} + K_{\text{sec}} \left[ \left(0.5 + 0.2\sin(2t)\right) - \theta_{2,\text{actual}} \right]
$$

———
一句话总结

常值调节问题中没有参考速度项，是因为参考速度本身就为 0，不是被省略了。 如果次任务也是一个时变的轨迹跟踪，同样需要加上对应的期望速度项。

---

从正运动学出发，推导速度映射和雅可比的过程非常直接：对时间求导 + 多元链式法则。

———
1. 抽象形式的推导

正运动学
末端位置是关节角的函数：
$$
\mathbf{x} = f(\mathbf{q})
$$
其中 $\mathbf{x} \in \mathbb{R}^m$，$\mathbf{q} \in \mathbb{R}^n$。

对时间求导
两边对时间 $t$ 求导：
$$
\frac{d\mathbf{x}}{dt} = \frac{\partial f}{\partial \mathbf{q}} \cdot \frac{d\mathbf{q}}{dt}
$$

定义速度和雅可比
• 末端速度：$\dot{\mathbf{x}} = \frac{d\mathbf{x}}{dt}$
• 关节速度：$\dot{\mathbf{q}} = \frac{d\mathbf{q}}{dt}$
• 雅可比矩阵：$\mathbf{J}(\mathbf{q}) = \frac{\partial f}{\partial \mathbf{q}} \in \mathbb{R}^{m \times n}$

于是得到核心关系：
$$
\boxed{ \dot{\mathbf{x}} = \mathbf{J}(\mathbf{q}) \, \dot{\mathbf{q}} }
$$

这就是速度级正运动学。雅可比的第 $i$ 列，就是第 $i$ 个关节以单位速度转动时，末端产生的线速度。

———
2. 以 2D 三连杆臂为例的具体推导

正运动学
$$
\begin{aligned}
x_e &= l_1 c_1 + l_2 c_{12} + l_3 c_{123} \\
y_e &= l_1 s_1 + l_2 s_{12} + l_3 s_{123}
\end{aligned}
$$
其中：
• $c_1 = \cos\theta_1$, $s_1 = \sin\theta_1$
• $c_{12} = \cos(\theta_1+\theta_2)$, $s_{12} = \sin(\theta_1+\theta_2)$
• $c_{123} = \cos(\theta_1+\theta_2+\theta_3)$, $s_{123} = \sin(\theta_1+\theta_2+\theta_3)$

计算 $\frac{\partial x_e}{\partial \theta_1}$

以 $x_e$ 对 $\theta_1$ 的偏导为例：
$$
\frac{\partial x_e}{\partial \theta_1} = -l_1 s_1 - l_2 s_{12} - l_3 s_{123}
$$

为什么？因为 $\theta_1$ 出现在所有三项中：
• $\frac{\partial}{\partial \theta_1}(l_1 c_1) = -l_1 s_1$
• $\frac{\partial}{\partial \theta_1}(l_2 c_{12}) = -l_2 s_{12} \cdot \frac{\partial(\theta_1+\theta_2)}{\partial \theta_1} = -l_2 s_{12}$
• $\frac{\partial}{\partial \theta_1}(l_3 c_{123}) = -l_3 s_{123} \cdot \frac{\partial(\theta_1+\theta_2+\theta_3)}{\partial \theta_1} = -l_3 s_{123}$

计算 $\frac{\partial x_e}{\partial \theta_2}$

$\theta_2$ 只出现在第二项和第三项：
$$
\frac{\partial x_e}{\partial \theta_2} = -l_2 s_{12} - l_3 s_{123}
$$

计算 $\frac{\partial x_e}{\partial \theta_3}$

$\theta_3$ 只出现在第三项：
$$
\frac{\partial x_e}{\partial \theta_3} = -l_3 s_{123}
$$

对 $y_e$ 同理
$$
\begin{aligned}
\frac{\partial y_e}{\partial \theta_1} &= l_1 c_1 + l_2 c_{12} + l_3 c_{123} \\
\frac{\partial y_e}{\partial \theta_2} &= l_2 c_{12} + l_3 c_{123} \\
\frac{\partial y_e}{\partial \theta_3} &= l_3 c_{123}
\end{aligned}
$$

组装雅可比矩阵
$$
\mathbf{J}_{\text{ee}} = \begin{bmatrix}
-l_1 s_1 - l_2 s_{12} - l_3 s_{123} & -l_2 s_{12} - l_3 s_{123} & -l_3 s_{123} \\
l_1 c_1 + l_2 c_{12} + l_3 c_{123} & l_2 c_{12} + l_3 c_{123} & l_3 c_{123}
\end{bmatrix}
$$

———
3. 几何直觉：雅可比列的物理意义

雅可比的每一列 $\mathbf{J}_i = \frac{\partial \mathbf{x}}{\partial \theta_i}$ 表示：仅让第 $i$ 个关节以单位角速度转动时，末端获得的速度。

对于平面旋转关节，这个速度的大小等于该关节到末端的距离，方向垂直于该连杆。

以第 2 列为例：
$$
\mathbf{J}2 = \begin{bmatrix} -l_2 s{12} - l_3 s_{123} \\ l_2 c_{12} + l_3 c_{123} \end{bmatrix}
$$

这正是从关节 2 到末端的位置向量旋转 90° 后的方向，长度为 $|l_2 + l_3|$ 在该平面上的投影。换句话说，转动 $\theta_2$ 时，连杆 2 和连杆 3 会一起转动，所以末端速度由这两段连杆共同贡献。

———
4. 速度映射的完整表达式

将雅可比代入 $\dot{\mathbf{x}} = \mathbf{J} \dot{\mathbf{q}}$，得到：
$$
\begin{bmatrix} \dot{x}_e \\ \dot{y}_e \end{bmatrix} = \begin{bmatrix}
-l_1 s_1 - l_2 s_{12} - l_3 s_{123} & -l_2 s_{12} - l_3 s_{123} & -l_3 s_{123} \\
l_1 c_1 + l_2 c_{12} + l_3 c_{123} & l_2 c_{12} + l_3 c_{123} & l_3 c_{123}
\end{bmatrix} \begin{bmatrix} \dot{\theta}_1 \\ \dot{\theta}_2 \\ \dot{\theta}_3 \end{bmatrix}
$$

展开第一行：
$$
\dot{x}e = (-l_1 s_1 - l_2 s{12} - l_3 s_{123})\dot{\theta}1 + (-l_2 s{12} - l_3 s_{123})\dot{\theta}2 + (-l_3 s{123})\dot{\theta}_3
$$

验证：当只有 $\theta_1$ 转动时（$\dot{\theta}_2 = \dot{\theta}_3 = 0$），
$$
\dot{x}e = (-l_1 s_1 - l_2 s{12} - l_3 s_{123})\dot{\theta}_1
$$
这与直接对 $x_e = l_1 c_1 + l_2 c_{12} + l_3 c_{123}$ 求时间导数（仅 $\theta_1$ 变）完全一致。

———
一句话总结

雅可比矩阵就是正运动学函数 $f(\mathbf{q})$ 对 $\mathbf{q}$ 的全导数。 求导规则没有任何特殊之处：对每个关节角用链式法则求偏导，然后把结果按列组装成矩阵。$\dot{\mathbf{x}} = \mathbf{J}\dot{\mathbf{q}}$ 本质上就是多元函数求导的链式法则。

如果你卡在了代码实现里某一步的偏导计算，把具体项发给我，我可以帮你逐个核对。

---

零空间投影矩阵的推导

我们从定义出发，一步步证明为什么零空间投影矩阵是 $\mathbf{N} = \mathbf{I} - \mathbf{J}^\dagger \mathbf{J}$。

———
1. 零空间的定义

给定任务雅可比 $\mathbf{J} \in \mathbb{R}^{m \times n}$（$m < n$，行满秩），零空间是所有不改变任务输出的关节速度的集合：

$$
\mathcal{N}(\mathbf{J}) = \{ \dot{\mathbf{q}} \in \mathbb{R}^n \mid \mathbf{J} \dot{\mathbf{q}} = \mathbf{0} \}
$$

维度：$\dim \mathcal{N}(\mathbf{J}) = n - m$。

我们的目标是找到一个矩阵 $\mathbf{N}$，使得对任意向量 $\mathbf{z}$，$\mathbf{N}\mathbf{z}$ 都落在 $\mathcal{N}(\mathbf{J})$ 里，而且如果 $\mathbf{z}$ 本身已经在 $\mathcal{N}(\mathbf{J})$ 里，那么 $\mathbf{N}\mathbf{z} = \mathbf{z}$。

———
2. 先理解 $\mathbf{J}^\dagger \mathbf{J}$ 投影到哪里

对于行满秩的 $\mathbf{J}$，右伪逆为：
$$
\mathbf{J}^\dagger = \mathbf{J}^\top (\mathbf{J}\mathbf{J}^\top)^{-1}
$$

有两个关键的恒等式：

（a） $\mathbf{J} \mathbf{J}^\dagger = \mathbf{I}_m$  
这说明 $\mathbf{J}^\dagger$ 把任务空间的速度映射回关节空间，而 $\mathbf{J}$ 再把它变回去时保持原样。

（b） $\mathbf{P} = \mathbf{J}^\dagger \mathbf{J} \in \mathbb{R}^{n \times n}$

这个矩阵有什么性质？

• 幂等性：
  $$
  \mathbf{P}^2 = (\mathbf{J}^\dagger \mathbf{J})(\mathbf{J}^\dagger \mathbf{J}) = \mathbf{J}^\dagger (\mathbf{J}\mathbf{J}^\dagger) \mathbf{J} = \mathbf{J}^\dagger \mathbf{I}_m \mathbf{J} = \mathbf{J}^\dagger \mathbf{J} = \mathbf{P}
  $$
  满足投影矩阵的定义。

• 对称性：
  $$
  \mathbf{P}^\top = (\mathbf{J}^\dagger \mathbf{J})^\top = \mathbf{J}^\top (\mathbf{J}^\dagger)^\top = \mathbf{J}^\top [(\mathbf{J}^\top)(\mathbf{J}\mathbf{J}^\top)^{-1}]^\top = \mathbf{J}^\top (\mathbf{J}\mathbf{J}^\top)^{-1} \mathbf{J} = \mathbf{J}^\dagger \mathbf{J} = \mathbf{P}
  $$

所以 $\mathbf{P} = \mathbf{J}^\dagger \mathbf{J}$ 是一个正交投影矩阵。它投影到哪个子空间？

$\mathbf{P}$ 的列空间（range space）就是 $\mathbf{J}^\top$ 的列空间，也就是 $\mathbf{J}$ 的行空间。换句话说：
$\mathbf{P}$ 把任意关节速度投影到能影响任务输出的那个方向上。

———
3. 零空间投影矩阵的构造

既然 $\mathbb{R}^n$ 可以正交分解为：
$$
\mathbb{R}^n = \underbrace{\mathcal{R}(\mathbf{J}^\top)}{\text{行空间}} \oplus \underbrace{\mathcal{N}(\mathbf{J})}{\text{零空间}}
$$

而 $\mathbf{P} = \mathbf{J}^\dagger \mathbf{J}$ 投影到行空间 $\mathcal{R}(\mathbf{J}^\top)$，那么投影到零空间的矩阵自然就是：

$$
\boxed{ \mathbf{N} = \mathbf{I}_n - \mathbf{J}^\dagger \mathbf{J} }
$$

———
4. 验证 $\mathbf{N}$ 确实是到 $\mathcal{N}(\mathbf{J})$ 的正交投影

我们需要验证四个性质：

性质 1：$\mathbf{N}$ 是幂等的（投影矩阵的基本要求）
$$
\mathbf{N}^2 = (\mathbf{I} - \mathbf{P})(\mathbf{I} - \mathbf{P}) = \mathbf{I} - 2\mathbf{P} + \mathbf{P}^2 = \mathbf{I} - 2\mathbf{P} + \mathbf{P} = \mathbf{I} - \mathbf{P} = \mathbf{N}
$$

性质 2：$\mathbf{N}$ 是对称的
$$
\mathbf{N}^\top = (\mathbf{I} - \mathbf{P})^\top = \mathbf{I} - \mathbf{P}^\top = \mathbf{I} - \mathbf{P} = \mathbf{N}
$$

性质 3：对任意 $\mathbf{z}$，$\mathbf{N}\mathbf{z}$ 都在零空间里
$$
\mathbf{J}(\mathbf{N}\mathbf{z}) = \mathbf{J}(\mathbf{I} - \mathbf{J}^\dagger \mathbf{J})\mathbf{z} = (\mathbf{J} - \mathbf{J}\mathbf{J}^\dagger \mathbf{J})\mathbf{z} = (\mathbf{J} - \mathbf{I}_m \mathbf{J})\mathbf{z} = \mathbf{0}
$$

性质 4：如果 $\mathbf{v}$ 已经在零空间里，则 $\mathbf{N}\mathbf{v} = \mathbf{v}$
若 $\mathbf{v} \in \mathcal{N}(\mathbf{J})$，则 $\mathbf{J}\mathbf{v} = \mathbf{0}$，于是：
$$
\mathbf{N}\mathbf{v} = (\mathbf{I} - \mathbf{J}^\dagger \mathbf{J})\mathbf{v} = \mathbf{v} - \mathbf{J}^\dagger (\mathbf{J}\mathbf{v}) = \mathbf{v} - \mathbf{0} = \mathbf{v}
$$

四个性质全部满足。所以 $\mathbf{N}$ 确实是到 $\mathcal{N}(\mathbf{J})$ 的正交投影矩阵。

———
5. 几何直觉

想象关节速度空间 $\mathbb{R}^n$ 被分解为两个正交方向：

• 蓝色方向（行空间）：这些关节速度会改变末端位置。$\mathbf{P} = \mathbf{J}^\dagger \mathbf{J}$ 把任意速度投影到蓝方向上。
• 红色方向（零空间）：这些关节速度不改变末端位置。$\mathbf{N} = \mathbf{I} - \mathbf{P}$ 把任意速度投影到红方向上。

        q̇ (任意关节速度)
        /|
       / |
      /  |  P·q̇  (影响末端的部分)
     /   |
    /____|____→ 行空间 R(Jᵀ)
    \    |
     \   |  N·q̇  (不影响末端的部分)
      \  |
       \ |
        \|/
         零空间 N(J)


次任务的作用：我们计算一个"理想次任务速度" $\dot{\mathbf{q}}_2^{\text{raw}}$，然后用 $\mathbf{N} \dot{\mathbf{q}}_2^{\text{raw}}$ 把它投影到红方向上——只留下那些不干扰主任务的份量。

———

6. 为什么次任务解法是 $\mathbf{N} (\mathbf{J}_2 \mathbf{N})^\dagger \mathbf{r}_2$？

这里还有一个容易困惑的地方：为什么不是直接用 $\mathbf{N} \mathbf{J}_2^\dagger \mathbf{r}_2$，而是 $(\mathbf{J}_2 \mathbf{N})^\dagger$？

原因：$\mathbf{N}$ 投影后的速度仍然需要满足次任务。我们需要找一个在零空间里的速度 $\dot{\mathbf{q}}_2$，使得：
$$
\mathbf{J}_2 \dot{\mathbf{q}}_2 = \mathbf{r}_2 \quad \text{且} \quad \dot{\mathbf{q}}_2 \in \mathcal{N}(\mathbf{J}_1)
$$

把约束 $\dot{\mathbf{q}}_2 = \mathbf{N}_1 \mathbf{z}$ 代入：
$$
\mathbf{J}_2 (\mathbf{N}_1 \mathbf{z}) = \mathbf{r}_2 \implies \mathbf{z} = (\mathbf{J}_2 \mathbf{N}_1)^\dagger \mathbf{r}_2
$$

于是：
$$
\dot{\mathbf{q}}_2 = \mathbf{N}_1 (\mathbf{J}_2 \mathbf{N}_1)^\dagger \mathbf{r}_2
$$

如果直接用 $\mathbf{N}_1 \mathbf{J}_2^\dagger \mathbf{r}_2$，你只是在把任意次任务解投影到零空间，但没有保证这个投影后的解能最好地完成次任务。$(\mathbf{J}_2 \mathbf{N}_1)^\dagger$ 才是在零空间约束下求解次任务的最小二乘最优解。

———
一句话总结

$\mathbf{N} = \mathbf{I} - \mathbf{J}^\dagger \mathbf{J}$ 是正交投影矩阵，因为 $\mathbf{J}^\dagger \mathbf{J}$ 投影到 $\mathbf{J}$ 的行空间（所有能影响任务输出的方向），而 $\mathbf{I}$ 减去它自然就把任意向量投影到了行空间的正交补——也就是零空间。