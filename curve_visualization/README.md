# 曲线生成算法测试与可视化

基于 `rmp::common::geometry::Curve` C++ 库的 Python 实现。

## 曲线类型

| 曲线 | 文件 | 说明 |
|---|---|---|
| BezierCurve | `curves/bezier.py` | 贝塞尔曲线，伯恩斯坦多项式 |
| BSplineCurve | `curves/bspline.py` | B样条曲线，Cox-deBoor递推 |
| CubicSplineCurve | `curves/cubic_spline.py` | 三次样条，三弯矩法 |
| DubinsCurve | `curves/dubins.py` | Dubins曲线，6种运动模式 |
| ReedsSheppCurve | `curves/reeds_shepp.py` | Reeds-Shepp曲线，48种模式 |
| QuinticPolynomial | `curves/quintic_polynomial.py` | 五次多项式，PVA约束 |

## 快速开始

```bash
# 安装依赖
pip install -r requirements.txt

# 运行所有测试和可视化
python main.py

# 仅运行测试
python main.py --test

# 仅运行可视化
python main.py --vis
```

## 项目结构

```
curve_visualization/
├── main.py                        # 主入口
├── config.py                      # 统一配置
├── requirements.txt               # 依赖
├── README.md
├── curves/                        # 曲线算法实现
│   ├── base_curve.py
│   ├── bezier.py
│   ├── bspline.py
│   ├── cubic_spline.py
│   ├── dubins.py
│   ├── reeds_shepp.py
│   └── quintic_polynomial.py
├── tests/
│   └── test_all_curves.py         # 单元测试
├── visualization/
│   ├── plot_utils.py              # 绘图工具
│   └── visualize_all.py           # 可视化脚本
└── output/                        # 输出图片
    ├── 01_individual_curves.png
    ├── 02_comparison.png
    ├── 03_bspline_params.png
    ├── 04_pose_paths.png
    └── 05_quintic_pva.png
```

## 输出图表

1. **各曲线独立展示** — 6条曲线各一张子图
2. **所有曲线对比** — 同一起始条件下叠加显示
3. **B-Spline 参数对比** — 阶数/参数化/插值vs逼近
4. **位姿路径对比** — Dubins vs Reeds-Shepp 多场景
5. **五次多项式 PVA** — 不同边界条件下的位置/速度/加速度/Jerk
