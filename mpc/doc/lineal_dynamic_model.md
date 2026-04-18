cart-pole 在 upright 平衡点附近的线性化是经典推导，我按步骤来。

———
1. 定义坐标与变量

| 符号 | 含义 |
|------|------|
| $x$ | 小车水平位移 |
| $\theta$ | 摆杆与竖直向上的夹角（$\theta=0$ 为 upright，逆时针为正） |
| $u = F$ | 作用在小车上的水平推力 |
| $M$ | 小车质量 |
| $m$ | 摆杆质量 |
| $l$ | 摆杆质心到转轴的距离 |
| $I$ | 摆杆绕质心的转动惯量 |

摆杆质心坐标：
$$x_p = x + l\sin\theta, \quad y_p = l\cos\theta$$

———
2. 非线性动力学方程（拉格朗日法）

动能 + 势能推导后得到：

$$
\begin{aligned}
(M+m)\ddot{x} + ml\ddot{\theta}\cos\theta - ml\dot{\theta}^2\sin\theta &= u \\
ml\ddot{x}\cos\theta + (ml^2+I)\ddot{\theta} - mgl\sin\theta &= 0
\end{aligned}
$$

———
3. 在 upright 平衡点线性化

平衡点：$x=0, \theta=0, \dot{x}=0, \dot{\theta}=0$

小角度近似：
• $\sin\theta \approx \theta$
• $\cos\theta \approx 1$
• 高阶项 $\dot{\theta}^2 \approx 0$ 舍去

代入得线性方程组：

$$
\begin{bmatrix} M+m & ml \\ ml & ml^2+I \end{bmatrix} \begin{bmatrix} \ddot{x} \\ \ddot{\theta} \end{bmatrix} = \begin{bmatrix} u \\ mgl\theta \end{bmatrix}
$$

令 $\Delta = (M+m)(ml^2+I) - m^2l^2 = M(ml^2+I) + mI$，求逆得：

$$
\begin{aligned}
\ddot{x} &= \underbrace{-\frac{m^2gl^2}{\Delta}}{a{23}}\theta + \underbrace{\frac{ml^2+I}{\Delta}}_{b_2} u \\[6pt]
\ddot{\theta} &= \underbrace{\frac{(M+m)mgl}{\Delta}}{a{43}}\theta \underbrace{-\frac{ml}{\Delta}}_{b_4} u
\end{aligned}
$$

———
4. 状态空间形式 $\dot{x} = A_c x + B_c u$

取状态 $x = [x, \dot{x}, \theta, \dot{\theta}]^\top$：

$$
A_c = \begin{bmatrix} 0 & 1 & 0 & 0 \\ 0 & 0 & a_{23} & 0 \\ 0 & 0 & 0 & 1 \\ 0 & 0 & a_{43} & 0 \end{bmatrix}, \quad
B_c = \begin{bmatrix} 0 \\ b_2 \\ 0 \\ b_4 \end{bmatrix}
$$

———
5. 简化版：质点模型（$I=0$，质量集中在末端）

如果近似认为摆杆全部质量集中在末端（常见于教材）：

$$
\Delta = Mml^2
$$

$$
A_c = \begin{bmatrix} 0 & 1 & 0 & 0 \\[4pt] 0 & 0 & -\dfrac{mg}{M} & 0 \\[8pt] 0 & 0 & 0 & 1 \\[4pt] 0 & 0 & \dfrac{(M+m)g}{Ml} & 0 \end{bmatrix}, \quad
B_c = \begin{bmatrix} 0 \\[4pt] \dfrac{1}{M} \\[8pt] 0 \\[4pt] -\dfrac{1}{Ml} \end{bmatrix}
$$

———
6. 代入 MuJoCo 默认参数

MuJoCo cart-pole 默认参数：
• $M = 1.0$ kg
• $m = 0.1$ kg  
• 杆长 $L = 0.5$ m（从转轴到末端）
• 均匀细杆：质心 $l = 0.25$ m，$I = \frac{1}{12}mL^2 \approx 0.00208$ kg·m²

分布质量模型（精确）：
$$
A_c \approx \begin{bmatrix} 0 & 1 & 0 & 0 \\ 0 & 0 & -0.718 & 0 \\ 0 & 0 & 0 & 1 \\ 0 & 0 & 31.58 & 0 \end{bmatrix}, \quad
B_c \approx \begin{bmatrix} 0 \\ 0.976 \\ 0 \\ -2.927 \end{bmatrix}
$$

质点简化模型（$I=0, l=0.5$）：
$$
A_c = \begin{bmatrix} 0 & 1 & 0 & 0 \\ 0 & 0 & -0.981 & 0 \\ 0 & 0 & 0 & 1 \\ 0 & 0 & 21.58 & 0 \end{bmatrix}, \quad
B_c = \begin{bmatrix} 0 \\ 1.0 \\ 0 \\ -2.0 \end{bmatrix}
$$

———
7. 下一步

拿到 $A_c, B_c$ 后，用零阶保持（ZOH）或前向欧拉法离散化：

$$A_d = e^{A_c T} \approx I + A_c T, \quad B_d = \int_0^T e^{A_c \tau} B_c \, d\tau \approx B_c T$$

其中控制周期 $T = 0.01$ s（100 Hz），就得到了 MPC 中需要的离散状态空间 $(A_d, B_d)$。

建议先用质点简化模型跑通实验，再对比分布质量模型的差异。