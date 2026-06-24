# MPC 模型预测控制算法

## 1. 算法概述

**MPC (Model Predictive Control)** 是一种基于模型的优化控制方法。核心思想是：在每个控制周期，利用系统模型预测未来一段时域内的状态演变，求解一个带约束的有限时域最优控制问题，并将第一个控制量作用于系统，下一周期重复此过程（滚动优化）。

```
┌─────────────────────────────────────────────────────┐
│  MPC 控制流程                                        │
│                                                     │
│  ① 获取当前状态 x(t)                                 │
│       │                                              │
│       ▼                                              │
│  ② 求解有限时域优化问题 (N 步)                        │
│     min  Σ [跟踪误差 + 控制代价]                       │
│     s.t. 动力学约束 + 控制约束                         │
│       │                                              │
│       ▼                                              │
│  ③ 得最优控制序列 U* = [u₀, u₁, ..., u_{N-1}]        │
│       │                                              │
│       ▼                                              │
│  ④ 执行第一帧 u₀，系统演化到 x(t+dt)                   │
│       │                                              │
│       └──── 回到 ① (滚动优化)                         │
└─────────────────────────────────────────────────────┘
```

---

## 2. 车辆运动学模型

### 2.1 自行车模型 (Kinematic Bicycle Model)

以**后轮中心**为参考点的自行车运动学模型：

$$
\begin{aligned}
\dot{x} &= v \cdot \cos\theta \\[4pt]
\dot{y} &= v \cdot \sin\theta \\[4pt]
\dot{\theta} &= \frac{v \cdot \tan\delta}{L}
\end{aligned}
$$

| 符号 | 含义 | 单位 |
|------|------|------|
| $x, y$ | 后轮中心坐标 | m |
| $\theta$ | 航向角 | rad |
| $v$ | 线速度 | m/s |
| $\delta$ | 前轮转角 | rad |
| $L$ | 轴距 | m |

### 2.2 离散化

使用 Euler 法离散化（dt 为控制周期）：

$$
\begin{bmatrix} x_{k+1} \\ y_{k+1} \\ \theta_{k+1} \end{bmatrix}
=
\begin{bmatrix} x_k \\ y_k \\ \theta_k \end{bmatrix}
+
\begin{bmatrix}
v_k \cos\theta_k \\
v_k \sin\theta_k \\
\frac{v_k \tan\delta_k}{L}
\end{bmatrix} \cdot dt
$$

### 2.3 线性化（用于线性 MPC）

在当前工作点 $(\bar{x}, \bar{u})$ 做一阶 Taylor 展开，得到线性时变模型：

$$
\tilde{x}_{k+1} = A_k \tilde{x}_k + B_k \tilde{u}_k + C_k
$$

其中：

$$
A_k = I + \frac{\partial f}{\partial x}\Big|_{\bar{x},\bar{u}} \cdot dt
\qquad
B_k = \frac{\partial f}{\partial u}\Big|_{\bar{x},\bar{u}} \cdot dt
$$

$$
\frac{\partial f}{\partial x} =
\begin{bmatrix}
0 & 0 & -v\sin\theta \\
0 & 0 &  v\cos\theta \\
0 & 0 & 0
\end{bmatrix}
\qquad
\frac{\partial f}{\partial u} =
\begin{bmatrix}
\cos\theta & 0 \\
\sin\theta & 0 \\
\frac{\tan\delta}{L} & \frac{v}{L\cos^2\delta}
\end{bmatrix}
$$

---

## 3. 优化问题形式化

### 3.1 非线性 MPC (NMPC)

直接对非线性模型做优化：

$$
\begin{aligned}
\min_{u_0,\dots,u_{N-1}} \quad &
\sum_{k=0}^{N-1} \Big[ \|x_{k+1} - x_{k+1}^{ref}\|_Q^2 + \|u_k\|_R^2 \Big]
+ \|x_N - x_N^{ref}\|_{Q_f}^2 \\[8pt]
\text{s.t.} \quad & x_{k+1} = f(x_k, u_k), \quad k = 0,\dots,N-1 \\
& x_0 = x_{\text{current}} \\
& u_{\min} \leq u_k \leq u_{\max}, \quad k = 0,\dots,N-1
\end{aligned}
$$

### 3.2 代价函数组成

| 项 | 公式 | 含义 |
|----|------|------|
| 跟踪代价 | $\|x_k - x_k^{ref}\|_Q^2$ | 状态偏离参考的惩罚 |
| 控制代价 | $\|u_k\|_R^2$ | 控制量大小的惩罚（平滑性） |
| 终端代价 | $\|x_N - x_N^{ref}\|_{Q_f}^2$ | 终端状态惩罚（稳定性） |

### 3.3 控制约束

$$
v_{\min} \leq v_k \leq v_{\max}, \qquad
\delta_{\min} \leq \delta_k \leq \delta_{\max}
$$

典型取值：

| 参数 | 值 | 说明 |
|------|-----|------|
| $v_{\min}$ | -0.5 m/s | 允许小幅倒车 |
| $v_{\max}$ | 3.0 m/s | 最大前进速度 |
| $\delta_{\min}$ | -0.6 rad | 左转极限 (~35°) |
| $\delta_{\max}$ | +0.6 rad | 右转极限 (~35°) |

---

## 4. 求解方法

### 4.1 scipy.optimize.minimize (SLSQP)

本仿真使用 **Sequential Least Squares Quadratic Programming (SLSQP)** 求解器，适合中小规模非线性约束优化。

```python
result = minimize(
    fun=cost_function,      # 代价函数
    x0=warm_start,          # 上一帧解作为初始猜测
    method='SLSQP',
    bounds=Bounds(lb, ub),  # 控制约束
    options={'maxiter': 200, 'ftol': 1e-6}
)
```

**Warm Start 加速**：将上一周期的优化解平移到当前帧作为初始猜测，大幅减少迭代次数。

### 4.2 其他可选方法

| 方法 | 适用场景 |
|------|----------|
| SLSQP | 通用 NMPC，中小规模 |
| OSQP | 线性化 MPC → QP 问题 |
| acados / CasADi | 实时高性能 NMPC |
| IPOPT | 大规模非线性规划 |

---

## 5. 参数调优指南

### 5.1 预测时域 N

| N | 效果 |
|----|------|
| 太小 (5~8) | 短视，急弯处容易切角 / 振荡 |
| 适中 (10~20) | 平衡计算量与前瞻性 |
| 太大 (>30) | 计算负担重，收益递减 |

### 5.2 权重矩阵

| 增大... | 效果 |
|---------|------|
| $Q_{xx}, Q_{yy}$ | 更紧地跟踪位置，可能振荡 |
| $Q_{\theta\theta}$ | 更关注航向对齐 |
| $R_{vv}$ | 速度变化更平滑 |
| $R_{\delta\delta}$ | 转向更平滑 |
| $Q_f$ | 终端收敛更有保障 |

### 5.3 常见问题

| 现象 | 可能原因 | 解决方向 |
|------|----------|----------|
| 轨迹振荡 | Q 过大 / R 过小 | 增大 R 或减小 Q |
| 跟踪滞后 | N 太小 / v_max 太小 | 增大 N 或 v_max |
| 求解慢 | N 太大 | 减小 N 或用线性化 MPC |
| 撞边界震荡 | 约束太紧 | 适当放宽 v_min / delta_max |

---

## 6. MPC vs DWA 对比

| 维度 | MPC | DWA |
|------|-----|-----|
| 优化方式 | 求解数学优化问题 | 采样 + 评分 |
| 模型使用 | 显式预测模型 | 隐式（轨迹模拟） |
| 约束处理 | 硬约束（优化约束） | 软约束（代价惩罚） |
| 最优性 | 近似最优（局部） | 采样最优（离散） |
| 计算量 | 中等（依赖 N 和求解器） | 中等（依赖采样密度） |
| 调参难度 | 权重 + 时域 + 约束 | 权重 + 采样空间 |
| 适用场景 | 高速 / 结构化道路 | 低速 / 杂乱环境 |

**选择建议**：
- 结构化环境、速度较高 → **MPC**
- 杂乱环境、需要快速避障 → **DWA**
- 实际部署中两者可以级联使用：全局规划 → MPC 轨迹生成 → DWA 底层避障

---

## 7. 仿真使用

```bash
cd motion_planner/mpc_simulation
pip install -r requirements.txt
python mpc_sim.py
```

修改 `mpc_sim.py` 顶部参数切换轨迹或调节控制效果：

```python
REF_TYPE = "eight"       # "eight" | "circle" | "lane_change"
HORIZON = 15             # 预测时域
Q = np.diag([10, 10, 2]) # 跟踪代价权重
R = np.diag([0.1, 0.5])  # 控制代价权重
V_MIN, V_MAX = -0.5, 3.0 # 速度约束
```

---

## 8. 参考文献

1. Rawlings, J.B., Mayne, D.Q. *Model Predictive Control: Theory and Design*. Nob Hill, 2009.
2. Borrelli, F., Bemporad, A., Morari, M. *Predictive Control for Linear and Hybrid Systems*. Cambridge, 2017.
3. Rajamani, R. *Vehicle Dynamics and Control*. Springer, 2012.
