# Centroidal MPC 中 QP 问题的构造详解

本文档总结 MPC（Model Predictive Control）中 QP（Quadratic Programming）问题的构造流程与关键概念。

---

## 1. 物理背景：摩擦锥（Friction Cone）

### 1.1 概念

摩擦锥描述的是地面能提供的切向力与法向力之间的约束关系。根据库仑摩擦定律：

$$\sqrt{f_x^2 + f_y^2} \leq \mu f_z$$

其中：
- $\mu$ 为地面摩擦系数（如水泥地约 0.6~0.8）
- $f_z$ 为法向接触力（必须为正，即脚要压在地面上）

几何上这是一个以法向为轴的圆锥。为了便于 QP 求解，通常将其线性化为四棱锥近似：

$$|f_x| \leq \mu f_z, \quad |f_y| \leq \mu f_z$$

### 1.2 在 MPC 中的作用

- **防止打滑**：若接触力超出摩擦锥边界，足端会在地面上滑动，导致机器人失稳
- **QP 可行性**：在优化问题中显式加入摩擦锥约束，求解器会自动调整力与力矩，使接触力始终处于可行域内
- **单腿站立尤为关键**：单腿时唯一接触点一旦失稳没有冗余，摩擦锥是硬约束

### 1.3 摩擦锥的矩阵形式

```python
A_fcon = np.array([
    [ 1.0,  0.0, -mu],
    [-1.0,  0.0, -mu],
    [ 0.0,  1.0, -mu],
    [ 0.0, -1.0, -mu],
])
b_fcon = np.zeros(4)
```

约束形式：$A_{\text{fcon}} \cdot f \leq 0$

展开即：
- $f_x \leq \mu f_z$
- $-f_x \leq \mu f_z \Rightarrow f_x \geq -\mu f_z$
- $f_y \leq \mu f_z$
- $-f_y \leq \mu f_z \Rightarrow f_y \geq -\mu f_z$

综合得 $|f_x| \leq \mu f_z$，$|f_y| \leq \mu f_z$。

> **注意**：上界 `u = 0` 限制的是不等式右端项，不是摩擦边界为 0。真正的摩擦边界由 $\mu$ 决定。

---

## 2. QP 问题的标准形式

OSQP 求解器要求的标准形式：

$$\begin{aligned}
\min_{z} \quad & \frac{1}{2} z^\top P z + q^\top z \\
\text{s.t.} \quad & l \leq A z \leq u
\end{aligned}$$

### 2.1 决策变量 $z$

$$z = [x_0, x_1, \ldots, x_N, u_0, u_1, \ldots, u_{N-1}]$$

维度计算：
- 状态序列：$x_0, x_1, \ldots, x_N$ 共 $N+1$ 个 $\Rightarrow (N+1) \cdot n_x$
- 控制序列：$u_0, u_1, \ldots, u_{N-1}$ 共 $N$ 个 $\Rightarrow N \cdot n_u$
- 总维度：$n_z = (N+1) \cdot n_x + N \cdot n_u$

**为什么状态比控制多 1 个？**

预测时域为 $N$，动力学约束定义在 $k=0$ 到 $N-1$ 上：

$$x_{k+1} = A_d x_k + B_d u_k + d_d$$

每个 $u_k$ 推动系统从 $x_k$ 演化到 $x_{k+1}$。初始状态 $x_0$ 是已知的当前测量值，最终状态 $x_N$ 由 $u_{N-1}$ 和 $x_{N-1}$ 决定，没有对应的 $u_N$。

---

## 3. 目标函数的构造

MPC 的目标是最小化跟踪误差的二次型：

$$J = \sum_{k=0}^{N} (x_k - x_{\text{ref}})^\top Q (x_k - x_{\text{ref}}) + \sum_{k=0}^{N-1} (u_k - u_{\text{ref}})^\top R (u_k - u_{\text{ref}})$$

### 3.1 展开为标准形式

以状态项为例展开：

$$(x - x_{\text{ref}})^\top Q (x - x_{\text{ref}}) = \underbrace{x^\top Q x}_{\text{二次项}} - \underbrace{2 x_{\text{ref}}^\top Q x}_{\text{一次项}} + \underbrace{x_{\text{ref}}^\top Q x_{\text{ref}}}_{\text{常数项}}$$

对应到 QP 标准形式：

| 项 | 数学表达 | 代码实现 |
|---|---|---|
| 二次项（Hessian） | $0.5 z^\top P z$ | `P = block_diag(Q, ..., QN, R, ..., R)` |
| 一次项（线性项） | $q^\top z$ | `q[k*nx:(k+1)*nx] = -Q @ x_ref` |
| 常数项 | — | 忽略（不影响最优解） |

### 3.2 关于 $q$ 的构造

```python
q = np.zeros(nz)
for k in range(N):
    q[k * nx : (k + 1) * nx] = -Q @ self.x_ref        # x_0 ~ x_{N-1}
q[N * nx : (N + 1) * nx] = -QN @ self.x_ref            # x_N
for k in range(N):
    q[(N + 1) * nx + k * nu : (N + 1) * nx + (k + 1) * nu] = -R @ self.u_ref  # u_0 ~ u_{N-1}
```

**关键点：**
- `setup` 时 `q=0` 只是占位符，每次 `solve()` 都会通过 `solver.update(q=q)` 重新填入
- $x_0$ 虽然在 `q` 里有对应的一次项，但被初始状态等式约束锁死，优化器无法调整，其代价贡献是一个常数

---

## 4. 约束条件的构造

### 4.1 约束矩阵 $A$ 的结构

$$A \in \mathbb{R}^{n_{\text{constr}} \times n_z}, \quad n_{\text{constr}} = n_x + N \cdot n_x + N \cdot 4$$

约束分为三行块：

| 行范围 | 约束类型 | 数量 |
|---|---|---|
| $0 : n_x$ | 初始状态等式 | $n_x$ |
| $n_x : n_x + N \cdot n_x$ | 动力学等式 | $N \cdot n_x$ |
| $n_x + N \cdot n_x : n_{\text{constr}}$ | 摩擦锥不等式 | $N \cdot 4$ |

### 4.2 初始状态约束：$x_0 = x_{0,\text{hat}}$

在 $A$ 矩阵的前 $n_x$ 行、前 $n_x$ 列放单位矩阵 $I$：

```python
for i in range(nx):
    A_dok[i, i] = 1.0
```

在 `solve` 中设置边界相等：

```python
l[:nx] = x0
u[:nx] = x0
```

OSQP 中 $l_i = u_i$ 时，双边不等式退化为等式：$A_i z = l_i = u_i$。

**为什么 x0 不放在目标函数里？**

$x_0$ 是已知的当前测量值（传感器/估计器给出），不是优化变量。把它放在约束里"硬锁死"，而不是在代价函数里"软引导"，这样更符合物理实际，也不会造成优化不一致。

### 4.3 动力学约束：$x_{k+1} = A_d x_k + B_d u_k + d_d$

```python
for k in range(N):
    row_base = nx + k * nx
    col_xk = k * nx
    col_xnext = (k + 1) * nx
    col_uk = (N + 1) * nx + k * nu

    for i in range(nx):
        A_dok[row_base + i, col_xnext + i] = 1.0      # x_{k+1} 系数: +I
        for j in range(nx):
            A_dok[row_base + i, col_xk + j] = -A_d[i, j]  # x_k 系数: -A_d
        for j in range(nu):
            A_dok[row_base + i, col_uk + j] = -B_d[i, j]  # u_k 系数: -B_d
```

这对应于：

$$x_{k+1} - A_d x_k - B_d u_k = d_d$$

边界设置（等式约束）：

```python
l[nx + k*nx : nx + (k+1)*nx] = d_d
u[nx + k*nx : nx + (k+1)*nx] = d_d
```

### 4.4 摩擦锥约束：$A_{\text{fcon}} u_k \leq 0$

```python
for k in range(N):
    row_base = nx + N*nx + k * 4
    col_uk = (N + 1) * nx + k * nu
    for i in range(4):
        for j in range(nu):
            A_dok[row_base + i, col_uk + j] = A_fcon[i, j]
```

边界设置（不等式约束）：

```python
l[nx + N*nx :] = -np.inf   # 下界: -inf（单向约束）
u[nx + N*nx :] = 0.0       # 上界: 0
```

> **为什么 l = -inf？** 因为摩擦锥约束是单向的，只要求 `A_fcon @ f <= 0`，不要求大于某个值。若设为 0 则变成等式，会强制接触力恰好落在摩擦锥的棱/面上，QP 可能无可行解。

---

## 5. $Q$ 与 $Q_N$ 的区别

| 符号 | 含义 | 维度 | 用途 |
|---|---|---|---|
| $Q$ | 中间状态权重 | $n_x \times n_x$ | 用于 $x_0, x_1, \ldots, x_{N-1}$ |
| $Q_N$ | 终端状态权重 | $n_x \times n_x$ | 仅用于 $x_N$ |
| $R$ | 控制权重 | $n_u \times n_u$ | 用于 $u_0, u_1, \ldots, u_{N-1}$ |

### 5.1 为什么要分开？

1. **稳定性理论**：在 MPC 理论中，$Q_N$ 应取无限时域 LQR 的 Riccati 方程解。这样可以把有限时域的终端代价当作"从 $x_N$ 到无穷远的剩余代价"，从而保证闭环稳定性。

2. **工程调参灵活性**：即使不做理论推导，也常把 $Q_N$ 设得比 $Q$ 更重（如 $Q_N = 10 \cdot Q$），强调"终点必须落在安全/稳定区域内"。

3. **当前实现**：`config.py` 中 `QN = Q.copy()`，两者相同。这是最简单的做法，后续可调参优化。

### 5.2 P 矩阵的块对角结构

```python
P_blocks = [Q] * N + [QN] + [R] * N
P = sparse.block_diag(P_blocks, format="csc")
```

对应的对角块顺序：

$$P = \text{diag}(\underbrace{Q, Q, \ldots, Q}_{N \text{ 个}}, Q_N, \underbrace{R, R, \ldots, R}_{N \text{ 个}})$$

---

## 6. 关键概念澄清

### 6.1 $x_0$ 与 $x_{\text{ref}}$ 的区别

| | $x_0$ | $x_{\text{ref}}$ |
|---|---|---|
| **含义** | 当前状态估计（传感器/估计器测得） | 期望达到的参考状态 |
| **性质** | 已知常数，不可优化 | 优化目标点 |
| **在 QP 中的位置** | 约束边界 $l, u$ | 目标函数一次项 $q$ |
| **代码体现** | `l[:nx] = x0; u[:nx] = x0` | `q = -Q @ x_ref` |

### 6.2 代价函数 vs 约束

| | 代价函数 | 约束 |
|---|---|---|
| **目的** | 衡量"偏离参考轨迹有多贵" | 强制"必须满足物理规律" |
| **内容** | 二次项 $x^\top Q x$ + 一次项 $-x_{\text{ref}}^\top Q x$ | 初始状态、动力学、摩擦锥 |
| **违反后果** | 代价增加（软惩罚） | QP 无可行解（硬限制） |

> **动力学不是被"惩罚"的，而是被严格锁定。** 如果动力学被放进代价函数做软惩罚，系统状态会"尽量"满足动力学但允许偏差，这与 MPC 的标准做法不同。

### 6.3 为什么 $P=Q$ 而不是 $P=2Q$？

真实代价展开含系数 2：

$$(x - x_{\text{ref}})^\top Q (x - x_{\text{ref}}) = x^\top Q x - 2x_{\text{ref}}^\top Q x + \text{const}$$

但代码中：
- $P = Q$（对应 $0.5 z^\top P z$ 里的二次项为 $0.5 x^\top Q x$）
- $q = -Q @ x_{\text{ref}}$（对应 $q^\top z$ 里的一次项为 $-x_{\text{ref}}^\top Q x$）

与真实代价相比整体差一个系数 2，但**整体缩放不改变最优解**，所以工程上直接用 `Q` 和 `-Q@x_ref` 即可。

---

## 7. 完整的 MPC QP 求解流程

```
1. 获取当前状态估计 x0
   ↓
2. 更新动力学矩阵 A_d, B_d, d_d（根据当前模型线性化）
   ↓
3. 构建/更新 QP：
   a. P 矩阵（块对角，含 Q, QN, R）
   b. q 向量（含 -Q@x_ref, -QN@x_ref, -R@u_ref）
   c. A 矩阵（含初始状态、动力学、摩擦锥结构）
   d. l, u 边界（等式约束用 l=u，不等式约束用 l=-inf, u=0）
   ↓
4. 调用 OSQP 求解
   ↓
5. 提取最优控制 u0（仅使用当前步控制量）
   ↓
6. 应用 u0 到系统，滚动时域，重复步骤 1
```

---

## 8. 一句话总结

**在已知当前状态 $x_0$ 的前提下，寻找最优控制序列 $u$，使得系统未来轨迹 $x$ 尽量接近期望轨迹 $x_{\text{ref}}$，同时严格遵守物理动力学和摩擦约束。**
