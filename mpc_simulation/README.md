# MPC Car Simulation

基于 **非线性模型预测控制 (NMPC)** 的小车轨迹跟踪仿真。

## 文件结构

```
mpc_simulation/
├── car_model.py        # 自行车运动学模型
├── mpc_controller.py   # MPC 控制器 (scipy SLSQP)
├── mpc_sim.py          # 主仿真 + 可视化
├── requirements.txt
└── README.md
```

## 算法概述

| 模块 | 说明 |
|------|------|
| 车辆模型 | 后轮参考点自行车运动学模型，状态 `[x, y, θ]`，控制 `[v, δ]` |
| 预测模型 | Euler 离散化，dt=0.1s |
| MPC 求解 | `scipy.optimize.minimize` (SLSQP)，warm start 加速 |
| 代价函数 | 跟踪误差 + 控制量 + 终端代价，加权求和 |
| 约束 | 速度/转角上下界 |

## 快速开始

```bash
cd mpc_simulation
pip install -r requirements.txt
python mpc_sim.py
```

## 可调参数

在 `mpc_sim.py` 顶部修改：

```python
REF_TYPE = "eight"       # "eight" | "circle" | "lane_change"
HORIZON = 15             # 预测时域
DT = 0.1                 # 步长
V_MIN, V_MAX = -0.5, 3.0
DELTA_MIN, DELTA_MAX = -0.6, 0.6
Q = np.diag([10, 10, 2])  # 跟踪代价
R = np.diag([0.1, 0.5])   # 控制代价
```

## 参考轨迹

- **eight**: 8 字形利萨如曲线
- **circle**: 圆形轨迹
- **lane_change**: 变道轨迹
