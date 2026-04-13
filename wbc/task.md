练习 1：2D 三连杆臂的多任务速度级 IK

---
任务目标

搭建一个 2D 平面三连杆臂的运动学仿真，实现以下两个控制目标：
- 主任务：臂末端跟踪一个圆形轨迹
- 次任务：保持肘关节（第二个关节）角度为固定值

分别用以下两种方法实现，并对比它们的行为差异：
1. 解析零空间投影（严格优先级）
2. 加权阻尼最小二乘（软优先级）

---
1. 系统建模

1.1 连杆参数
- 连杆数：3
- 杆长：$l_1 = 1.0, \, l_2 = 1.0, \, l_3 = 0.8$（单位任意，保持一致即可）
- 关节角：$\mathbf{q} = [\theta_1, \theta_2, \theta_3]^\top$

1.2 正运动学
末端位置 $(x_e, y_e)$ 关于关节角的表达式：
$$
\begin{aligned}
x_e &= l_1 c_1 + l_2 c_{12} + l_3 c_{123} \\
y_e &= l_1 s_1 + l_2 s_{12} + l_3 s_{123}
\end{aligned}
$$
其中 $c_1 = \cos\theta_1$, $c_{12} = \cos(\theta_1+\theta_2)$, $c_{123} = \cos(\theta_1+\theta_2+\theta_3)$，正弦同理。

1.3 雅可比矩阵
末端位置雅可比 $\mathbf{J}_{\text{ee}} \in \mathbb{R}^{2 \times 3}$：
$$
\mathbf{J}_{\text{ee}} = \begin{bmatrix}
\frac{\partial x_e}{\partial \theta_1} & \frac{\partial x_e}{\partial \theta_2} & \frac{\partial x_e}{\partial \theta_3} \\
\frac{\partial y_e}{\partial \theta_1} & \frac{\partial y_e}{\partial \theta_2} & \frac{\partial y_e}{\partial \theta_3}
\end{bmatrix}
$$

肘关节（第二个关节）角度是一个标量，所以其次任务雅可比非常简单：
$$
\mathbf{J}_{\text{elbow}} = \begin{bmatrix} 0 & 1 & 0 \end{bmatrix} \in \mathbb{R}^{1 \times 3}
$$

---
2. 主任务设定：末端跟踪圆形轨迹

圆形轨迹的中心在 $(2.0, 1.0)$，半径 $r = 0.5$。参数方程为：
$$
\mathbf{x}_{\text{des}}(t) = \begin{bmatrix} 2.0 + 0.5 \cos(\omega t) \\ 1.0 + 0.5 \sin(\omega t) \end{bmatrix}
$$

任务速度（末端期望速度）通过对时间求导得到：
$$
\dot{\mathbf{x}}_{\text{des}}(t) = \begin{bmatrix} -0.5 \omega \sin(\omega t) \\ 0.5 \omega \cos(\omega t) \end{bmatrix}
$$

建议取 $\omega = 2\pi$（一圈/秒），仿真时长 $T = 2$ 秒，时间步长固定为 $\Delta t$。

初始姿态建议：选择一个能让末端在圆附近、且不过分奇异的初始关节角，例如 $\mathbf{q}_0 = [0.3, 0.5, 0.2]^\top$。

位置反馈项：纯速度级开环积分会漂移。在主任务速度中加入位置误差反馈：
$$
\dot{\mathbf{x}}^* = \dot{\mathbf{x}}{\text{des}} + K_p (\mathbf{x}{\text{des}} - \mathbf{x}_{\text{actual}})
$$
建议 $K_p = 5.0$。

---
3. 次任务设定：肘关节角度保持

设定期望肘关节角度 $\theta_{2,\text{des}} = 0.5$（弧度）。次任务期望速度为：
$$
\dot{\theta}2^* = K{\text{sec}} (\theta_{2,\text{des}} - \theta_{2,\text{actual}})
$$
建议 $K_{\text{sec}} = 2.0$。

---
4. 方法 A：解析零空间投影

算法步骤
在每一步仿真中：

1. 计算当前状态：由 $\mathbf{q}(t)$ 计算末端位置 $\mathbf{x}{\text{actual}}$ 和雅可比 $\mathbf{J}{\text{ee}}$、$\mathbf{J}_{\text{elbow}}$
2. 计算主任务期望速度 $\dot{\mathbf{x}}^*$（含位置反馈）
3. 求解主任务关节速度：
   $$
   \dot{\mathbf{q}}1 = \mathbf{J}{\text{ee}}^\dagger \dot{\mathbf{x}}^*
   $$
   其中伪逆 $\mathbf{J}{\text{ee}}^\dagger = \mathbf{J}{\text{ee}}^\top (\mathbf{J}{\text{ee}} \mathbf{J}{\text{ee}}^\top)^{-1}$
4. 计算零空间投影矩阵：
   $$
   \mathbf{N}1 = \mathbf{I}_3 - \mathbf{J}{\text{ee}}^\dagger \mathbf{J}_{\text{ee}}
   $$
5. 求解次任务残差：
   $$
   \mathbf{r}2 = \dot{\theta}_2^* - \mathbf{J}{\text{elbow}} \dot{\mathbf{q}}_1
   $$
6. 在零空间中求解次任务：
   $$
   \dot{\mathbf{q}}2 = \mathbf{N}_1 (\mathbf{J}{\text{elbow}} \mathbf{N}_1)^\dagger \mathbf{r}_2
   $$
7. 总关节速度：
   $$
   \dot{\mathbf{q}} = \dot{\mathbf{q}}_1 + \dot{\mathbf{q}}_2
   $$
8. 积分更新：
   $$
   \mathbf{q}(t+\Delta t) = \mathbf{q}(t) + \dot{\mathbf{q}} \, \Delta t
   $$

注意事项
- 处理 $\mathbf{J}{\text{ee}} \mathbf{J}{\text{ee}}^\top$ 的奇异：如果行列式小于某个小阈值（如 $10^{-6}$），添加一个小阻尼或跳过该步
- $\mathbf{J}_{\text{elbow}} \mathbf{N}_1$ 是一个 $1 \times 3$ 矩阵，其伪逆可以用转置除以自身与转置的乘积来计算

---
5. 方法 B：加权阻尼最小二乘（WLN）

算法步骤
在每一步仿真中：

1. 计算当前状态、主任务和次任务期望速度（同方法 A）
2. 堆叠雅可比和任务速度：
   $$
   \mathbf{J}{\text{stack}} = \begin{bmatrix} \mathbf{J}{\text{ee}} \\ \mathbf{J}_{\text{elbow}} \end{bmatrix} \in \mathbb{R}^{3 \times 3}, \quad
   \dot{\mathbf{x}}_{\text{stack}} = \begin{bmatrix} \dot{\mathbf{x}}^ \\ \dot{\theta}_2^ \end{bmatrix} \in \mathbb{R}^{3}
   $$
3. 构造权重矩阵：
   $$
   \mathbf{W} = \text{diag}(w_{\text{ee}}, w_{\text{ee}}, w_{\text{elbow}}) \in \mathbb{R}^{3 \times 3}
   $$
   建议尝试两组参数对比：
   - 参数组 1：$w_{\text{ee}} = 100, \, w_{\text{elbow}} = 1$（主任务优先）

- 参数组 2：$w_{\text{ee}} = 10, \, w_{\text{elbow}} = 10$（两者接近同等重要）
4. 构造阻尼项：$\lambda = 0.01$ 或 $0.05$
5. 求解加权阻尼最小二乘：
   $$
   \dot{\mathbf{q}} = (\mathbf{J}{\text{stack}}^\top \mathbf{W} \mathbf{J}{\text{stack}} + \lambda^2 \mathbf{I}3)^{-1} \mathbf{J}{\text{stack}}^\top \mathbf{W} \dot{\mathbf{x}}_{\text{stack}}
   $$
   （也可用 $m \times m$ 逆的等价形式减少计算量）
6. 积分更新关节角

---
6. 仿真与验证要求

必须记录的数据
在每个时间步记录以下内容，用于后续分析和画图：
- 时间 $t$
- 关节角 $\mathbf{q}(t)$
- 末端实际位置 $(x_e, y_e)$ 与期望位置
- 肘关节实际角度 $\theta_2$ 与期望角度
- 关节速度 $\dot{\mathbf{q}}$

必须输出的图表
1. 动画或静态轨迹图：2D 平面中，臂的姿态随时间变化（至少画几个关键帧），以及末端实际轨迹与期望圆的对比
2. 末端跟踪误差：$\| \mathbf{x}{\text{des}} - \mathbf{x}{\text{actual}} \|$ 随时间变化曲线
3. 肘关节角度曲线：$\theta_2(t)$ 与 $\theta_{2,\text{des}}$ 的对比
4. 关节速度曲线：三个关节的 $\dot{\theta}_i(t)$，观察是否有奇异点处速度飙高的现象

预期观察结果（用于自测）
- 解析零空间投影：末端跟踪应该很好；肘关节角度在主任务不影响的零空间方向上尽量保持。如果主任务和次任务严格冲突，肘关节会被完全牺牲。
- WLN 组 1（$w_{\text{ee}}=100$）：行为应接近解析法，末端跟踪好，肘关节尽量保持。
- WLN 组 2（$w_{\text{ee}}=10$）：当末端运动和肘关节约束冲突时，你会看到两者都妥协——末端轨迹偏离圆，肘关节也偏离 $0.5$。这就是软优先级的特征。
- 奇异点观察：如果在仿真中让臂接近完全伸直，纯伪逆法可能出现关节速度峰值；而带阻尼的 WLN 能抑制这一现象。

---
7. 编程提示（不含代码）

- python
- 函数封装建议：把正运动学计算封装成函数 forward_kinematics(q) 返回末端位置；把雅可比计算封装成 jacobian_ee(q)
- 伪逆可用 np.linalg.pinv，但为了理解，建议先用公式手动实现行满秩时的简化伪逆
- 动画可用 matplotlib.animation（Python）或其他你熟悉的绘图工具；如果嫌麻烦，画关键帧的静态子图也可以

---
