思路：速度级 QP 实现三连杆臂多任务 IK

这个 Demo 里没有接触力、没有完整动力学约束，所以 QP 的决策变量仍然是关节速度 $\dot{\mathbf{q}} \in \mathbb{R}^3$。它本质上是把“加权阻尼最小二乘”装进了一个带不等式约束的优化框架里。

1. 决策变量
$$
\dot{\mathbf{q}} = [\dot{\theta}_1, \dot{\theta}_2, \dot{\theta}_3]^\top
$$

2. 目标函数：多任务残差的加权平方和

把主任务（末端跟踪）和次任务（肘关节保持）统一写成残差形式，然后加权求和：

$$
\min_{\dot{\mathbf{q}}} \frac{1}{2} w_1 \| \mathbf{J}_1 \dot{\mathbf{q}} - \dot{\mathbf{x}}_1^ \|^2 + \frac{1}{2} w_2 \| \mathbf{J}_2 \dot{\mathbf{q}} - \dot{\mathbf{x}}_2^ \|^2 + \frac{1}{2} \lambda \| \dot{\mathbf{q}} \|^2
$$

其中：
• $\mathbf{J}_1 \in \mathbb{R}^{2 \times 3}$：末端位置雅可比
• $\dot{\mathbf{x}}_1^* \in \mathbb{R}^2$：末端期望速度（圆的切向速度 + 位置反馈）
• $\mathbf{J}_2 = [0, 1, 0] \in \mathbb{R}^{1 \times 3}$：肘关节雅可比
• $\dot{\mathbf{x}}2^* = K{\text{sec}}(\theta_{2,\text{des}} - \theta_{2,\text{actual}})$：肘关节期望“速度”
• $w_1, w_2$：任务权重
• $\lambda$：关节速度正则化（阻尼）

权重的物理意义：通过 $w_1 \gg w_2$ 来实现软优先级。比如 $w_1 = 1000, w_2 = 1$，优化器会优先压榨主任务残差，再在剩余自由度里尽量满足次任务。

3. 化为标准 QP 形式

标准 QP：
$$
\min_{\dot{\mathbf{q}}} \frac{1}{2} \dot{\mathbf{q}}^\top \mathbf{H} \dot{\mathbf{q}} + \mathbf{g}^\top \dot{\mathbf{q}}
$$

把目标函数展开、合并同类项，得到：

$$
\mathbf{H} = w_1 \mathbf{J}_1^\top \mathbf{J}_1 + w_2 \mathbf{J}_2^\top \mathbf{J}_2 + \lambda \mathbf{I}_3
$$

$$
\mathbf{g} = -w_1 \mathbf{J}_1^\top \dot{\mathbf{x}}_1^ - w_2 \mathbf{J}_2^\top \dot{\mathbf{x}}_2^
$$

注意 $\mathbf{H}$ 是对称半正定的。只要 $\lambda > 0$，它就是严格正定的，QP 有唯一解。

4. 不等式约束（可选，但体现 QP 的价值）

这是 QP 相对于解析方法的真正优势——你可以加硬约束：

关节速度限幅：
$$
\dot{\mathbf{q}}{\min} \le \dot{\mathbf{q}} \le \dot{\mathbf{q}}{\max}
$$

化成标准形式 $\mathbf{A} \dot{\mathbf{q}} \le \mathbf{b}$：
$$
\begin{bmatrix} \mathbf{I}3 \\ -\mathbf{I}_3 \end{bmatrix} \dot{\mathbf{q}} \le \begin{bmatrix} \dot{\mathbf{q}}{\max} \\ -\dot{\mathbf{q}}_{\min} \end{bmatrix}
$$

你也可以加关节位置限幅（把位置约束线性化到速度级），但这对于 2 秒仿真通常不是必须的。

5. 两种玩法：单层加权 QP vs 严格分层 QP

玩法 A：单层加权 QP（推荐先做这个）
只用一套 $(\mathbf{H}, \mathbf{g}, \mathbf{A}, \mathbf{b})$，通过权重悬殊来实现主次区分。
• 优点：一个 QP 搞定，求解快，调参直观。
• 缺点：主次是“软”的。当两个任务严重冲突时，次任务会“偷走”一部分主任务资源。

玩法 B：严格分层 QP（Hierarchical QP）
如果你希望主任务绝对优先、次任务绝对不能干扰主任务（和解析零空间投影等价），需要做两层 QP：

第 1 层（只优化主任务）：
$$
c_1^ = \min_{\dot{\mathbf{q}}} \frac{1}{2} \| \mathbf{J}_1 \dot{\mathbf{q}} - \dot{\mathbf{x}}_1^ \|^2 + \frac{1}{2} \lambda \| \dot{\mathbf{q}} \|^2
$$
记下最优值 $c_1^*$。

第 2 层（优化次任务，同时把主任务残差锁在 $c_1^*$）：
$$
\min_{\dot{\mathbf{q}}} \frac{1}{2} \| \mathbf{J}_2 \dot{\mathbf{q}} - \dot{\mathbf{x}}_2^* \|^2 + \frac{1}{2} \lambda \| \dot{\mathbf{q}} \|^2
$$
$$
\text{s.t.} \quad \frac{1}{2} \| \mathbf{J}_1 \dot{\mathbf{q}} - \dot{\mathbf{x}}_1^ \|^2 \le c_1^
$$

第二层的主任务约束可以进一步展开成二次不等式约束。对于三连杆臂这个简单例子，这个分层 QP 有点 overkill，但它演示了 QP 如何实现硬优先级。

6. 与之前方法的数学联系

• 如果 $w_2 = 0$ 且没有约束：QP 的解就是纯主任务的阻尼最小二乘解。
• 如果 $w_1, w_2 > 0$ 且没有约束：QP 的解析解等价于加权阻尼最小二乘（两个任务堆叠后用块对角权重矩阵的伪逆）。
• 有约束时：解析方法和 DLS 都束手无策，QP 能在约束边界上自动取折中。

7. 实现流程建议

1. 和之前一样，每步计算当前 $\mathbf{q}$ 对应的 $\mathbf{J}_1, \mathbf{J}_2$ 和任务速度 $\dot{\mathbf{x}}_1^, \dot{\mathbf{x}}_2^$。
2. 构造 $\mathbf{H}$ 和 $\mathbf{g}$。
3. （可选）构造约束矩阵 $\mathbf{A}, \mathbf{b}$。
4. 调用 QP 求解器（如 osqp, qpsolvers, cvxopt）解出 $\dot{\mathbf{q}}^*$。
5. 欧拉积分：$\mathbf{q} \leftarrow \mathbf{q} + \dot{\mathbf{q}}^* \Delta t$。