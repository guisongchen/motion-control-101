好的，我们来一步一步推导。为了具体化，我先写出 Cart-Table 模型的标准离散形式，然后把 $N=16$ 的稠密矩阵构造出来。

---
1. 模型设定

对于 LIPM / Cart-Table 模型，状态通常取 CoM 的位置、速度、加速度：

$$
x(k) = \begin{bmatrix} p_{\text{com}}(k) \\ v_{\text{com}}(k) \\ a_{\text{com}}(k) \end{bmatrix} \in \mathbb{R}^3
$$

控制输入是 jerk（加速度的变化率）：

$$
u(k) = \dot{a}_{\text{com}}(k) \in \mathbb{R}
$$

ZMP 输出：

$$
p_{\text{zmp}}(k) = c^\top x(k), \quad c^\top = \begin{bmatrix} 1 & 0 & -\frac{z_c}{g} \end{bmatrix}
$$

离散动力学（假设采样周期为 $T$）：

$$
x(k+1) = A x(k) + b \, u(k)
$$

其中：

$$
A = \begin{bmatrix} 1 & T & \frac{T^2}{2} \\ 0 & 1 & T \\ 0 & 0 & 1 \end{bmatrix}, \quad
b = \begin{bmatrix} \frac{T^3}{6} \\ \frac{T^2}{2} \\ T \end{bmatrix}
$$

---
2. 预测时域内的状态序列堆叠

定义预测时域 $N = 16$。把未来 $N$ 步的状态和控制堆叠成向量：

$$
X = \begin{bmatrix} x(1) \\ x(2) \\ \vdots \\ x(16) \end{bmatrix} \in \mathbb{R}^{48}, \quad
U = \begin{bmatrix} u(0) \\ u(1) \\ \vdots \\ u(15) \end{bmatrix} \in \mathbb{R}^{16}
$$

注意：$x(0)$ 是当前已知状态，$u(0)$ 是当前要施加的控制量。

递推展开

$$
\begin{aligned}
x(1) &= A x(0) + b u(0) \\
x(2) &= A x(1) + b u(1) = A^2 x(0) + A b u(0) + b u(1) \\
x(3) &= A x(2) + b u(2) = A^3 x(0) + A^2 b u(0) + A b u(1) + b u(2) \\
&\vdots \\
x(k) &= A^k x(0) + \sum_{j=0}^{k-1} A^{k-1-j} b \, u(j)
\end{aligned}
$$

把所有 $x(k)$ 堆叠起来，得到稠密形式：

$$
X = \mathcal{A} \, x(0) + \mathcal{B} \, U
$$

矩阵 $\mathcal{A}$ 和 $\mathcal{B}$ 的结构

$$
\mathcal{A} = \begin{bmatrix} A \\ A^2 \\ A^3 \\ \vdots \\ A^{16} \end{bmatrix} \in \mathbb{R}^{48 \times 3}
$$

$$
\mathcal{B} = \begin{bmatrix}
b & 0 & 0 & \cdots & 0 \\
Ab & b & 0 & \cdots & 0 \\
A^2b & Ab & b & \cdots & 0 \\
\vdots & \vdots & \vdots & \ddots & \vdots \\
A^{15}b & A^{14}b & A^{13}b & \cdots & b
\end{bmatrix} \in \mathbb{R}^{48 \times 16}
$$

注意 $\mathcal{B}$ 是下三角块-Toeplitz 结构。每一列对应某个 $u(j)$ 对所有未来状态的影响。

---
3. 输出序列（ZMP 预测）

把 $N$ 步的 ZMP 输出也堆叠：

$$
Y = \begin{bmatrix} p_{\text{zmp}}(1) \\ p_{\text{zmp}}(2) \\ \vdots \\ p_{\text{zmp}}(16) \end{bmatrix} \in \mathbb{R}^{16}
$$

因为 $p_{\text{zmp}}(k) = c^\top x(k)$，所以：

$$
Y = \mathcal{C} X
$$

其中 $\mathcal{C}$ 是块对角矩阵：

$$
\mathcal{C} = I_{16} \otimes c^\top = \begin{bmatrix}
c^\top & 0 & \cdots & 0 \\
0 & c^\top & \cdots & 0 \\
\vdots & \vdots & \ddots & \vdots \\
0 & 0 & \cdots & c^\top
\end{bmatrix} \in \mathbb{R}^{16 \times 48}
$$

进一步，可以把 $Y$ 直接写成关于 $U$ 的仿射函数：

$$
Y = \mathcal{C} \mathcal{A} \, x(0) + \mathcal{C} \mathcal{B} \, U = \mathcal{Y}_{\text{free}} + \mathcal{G} U
$$

其中：
• $\mathcal{Y}_{\text{free}} = \mathcal{C} \mathcal{A} x(0)$ 是零输入响应（已知常数）
• $\mathcal{G} = \mathcal{C} \mathcal{B} \in \mathbb{R}^{16 \times 16}$ 是下三角矩阵，表示控制到 ZMP 的脉冲响应

$\mathcal{G}$ 的第 $(i, j)$ 个元素（$i \geq j$）为：

$$
\mathcal{G}_{ij} = c^\top A^{i-j} b
$$

这在代码实现时非常有用——你可以直接循环计算，不需要真的做矩阵乘法。

---
4. QP 目标函数

标准 MPC 目标函数：

$$
\min_U \quad \| Y^{\text{ref}} - Y \|_Q^2 + \| U \|_R^2 + \| \Delta U \|_S^2
$$

把 $Y = \mathcal{Y}_{\text{free}} + \mathcal{G} U$ 代入：

$$
\min_U \quad \| Y^{\text{ref}} - \mathcal{Y}_{\text{free}} - \mathcal{G} U \|_Q^2 + \| U \|_R^2 + \| \Delta U \|_S^2
$$

关于 $\Delta U$ 的处理

$\Delta U$ 是控制增量的序列。定义差分矩阵 $D \in \mathbb{R}^{16 \times 16}$：

$$
\Delta U = D U - u_{-1} e_1
$$

其中 $u_{-1}$ 是上一时刻施加的控制量，$e_1 = [1, 0, \dots, 0]^\top$，而：

$$
D = \begin{bmatrix}
1 & 0 & 0 & \cdots & 0 \\
-1 & 1 & 0 & \cdots & 0 \\
0 & -1 & 1 & \cdots & 0 \\
\vdots & \vdots & \vdots & \ddots & \vdots \\
0 & 0 & 0 & -1 & 1
\end{bmatrix}
$$

（因为 $\Delta u(0) = u(0) - u_{-1}$，$\Delta u(k) = u(k) - u(k-1)$ for $k \geq 1$）

整理成标准 QP 形式

令 $E = Y^{\text{ref}} - \mathcal{Y}_{\text{free}}$（跟踪误差，已知），则目标函数展开为：

$$
J(U) = U^\top H U + 2 q^\top U + \text{const}
$$

其中 Hessian $H$ 和 梯度 $q$ 为：

$$

H = \mathcal{G}^\top Q \mathcal{G} + R + D^\top S D
$$

$$
q = -\mathcal{G}^\top Q E - D^\top S \, u_{-1} e_1
$$

---
5. 约束

控制量约束（jerk 有界）

$$
U_{\min} \leq U \leq U_{\max}
$$

增量约束（jerk 变化率有界）

$$
\Delta U_{\min} \leq D U - u_{-1} e_1 \leq \Delta U_{\max}
$$

ZMP 约束（在支撑多边形内）

$$
Y_{\min} \leq \mathcal{Y}{\text{free}} + \mathcal{G} U \leq Y{\max}
$$

---
6. 标准稠密 QP 形式

把所有约束写成 $l \leq M U \leq u$ 的形式：

$$
\begin{aligned}
\min_U \quad & \frac{1}{2} U^\top H U + q^\top U \\
\text{s.t.} \quad & \begin{bmatrix} I \\ D \\ \mathcal{G} \end{bmatrix} U \geq \begin{bmatrix} U_{\min} \\ \Delta U_{\min} + u_{-1}e_1 \\ Y_{\min} - \mathcal{Y}_{\text{free}} \end{bmatrix} \\
& \begin{bmatrix} I \\ D \\ \mathcal{G} \end{bmatrix} U \leq \begin{bmatrix} U_{\max} \\ \Delta U_{\max} + u_{-1}e_1 \\ Y_{\max} - \mathcal{Y}_{\text{free}} \end{bmatrix}
\end{aligned}
$$

OSQP 接受的就是这种形式。注意 OSQP 的目标函数默认带 $\frac{1}{2}$，所以调用时传入的 $H$ 要和我们推导的一致。

---
7. 代码实现时的要点

实际写代码时，不需要显式构造 $\mathcal{A}, \mathcal{B}, \mathcal{C}$ 这三个大矩阵。更高效的做法是：

1. 先算 $\mathcal{Y}_{\text{free}}$：从 $x(0)$ 开始，迭代 $N$ 步零输入 rollout，每一步取 $c^\top x(k)$
2. 再算 $\mathcal{G}$ 的各列：对 $j = 0, \dots, 15$，以 $x(0) = 0, u(j) = 1$、其余控制为 0 做 rollout，得到第 $j$ 列

或者更直接：利用 $A, b$ 的解析形式，$A^k$ 可以用递推预先算好。

---
总结：对于 $N=16$，你最终要求解的是一个
• 决策变量维度：$16$（一维 jerk 序列）
• 约束数量：$16 \times 3 = 48$（控制、增量、ZMP 各 16 个不等式）

的稠密 QP。这个规模对 OSQP 来说在亚毫秒级就能解完。

需要我接着推导 多坐标轴（x 和 y 方向同时优化） 的情况吗？那时候决策变量会变成 $32$ 维，但结构完全类似，只是块对角化。