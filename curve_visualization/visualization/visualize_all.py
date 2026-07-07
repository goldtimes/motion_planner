"""
Visualize all curves

Generate multiple charts:
1. Individual curve display
2. All curves comparison
3. B-Spline parameter comparison
4. Dubins / Reeds-Shepp pose path comparison
5. Quintic polynomial PVA curves
"""

import os
import sys
import math
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, "/motion_ws/curve_visualization")

# 输出目录
OUTPUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "output")

from curves import (
    BezierCurve, BSplineCurve, CubicSplineCurve,
    DubinsCurve, ReedsSheppCurve, QuinticCurve, QuinticPolynomial
)
from config import (
    COLORS, waypoints_s_shape, waypoints_line, waypoints_turn,
    start_pose, goal_pose, quintic_start, quintic_end, quintic_T, STEP
)
from visualization.plot_utils import (
    plot_curve_with_points, plot_pose_path, plot_quintic_pva, save_figure
)


def visualize_individual_curves():
    """Figure 1: Individual curve display"""
    print("\n[Figure 1] Individual curves...")

    # Prepare all curves
    curves = [
        ("Bezier", BezierCurve(step=STEP), COLORS["bezier"]),
        ("B-Spline", BSplineCurve(step=STEP * 0.5), COLORS["bspline"]),
        ("Cubic Spline", CubicSplineCurve(step=STEP), COLORS["cubic_spline"]),
        ("Dubins", DubinsCurve(step=STEP), COLORS["dubins"]),
        ("Reeds-Shepp", ReedsSheppCurve(step=STEP), COLORS["reeds_shepp"]),
        ("Quintic", QuinticCurve(step=STEP), COLORS["quintic"]),
    ]

    fig, axes = plt.subplots(2, 3, figsize=(16, 10))
    axes = axes.flatten()

    for idx, (name, curve, color) in enumerate(curves):
        ax = axes[idx]
        try:
            if isinstance(curve, (DubinsCurve, ReedsSheppCurve)):
                path = curve.generation(start_pose, goal_pose)
            else:
                path = curve.run(waypoints_s_shape)
            # Extract x,y coordinates for pose-based curves
            points_2d = [(p[0], p[1]) for p in path]
            plot_curve_with_points(ax, points_2d, waypoints_s_shape, name, color, name)
        except Exception as e:
            ax.text(0.5, 0.5, f"Error:\n{e}", ha='center', va='center',
                    transform=ax.transAxes, fontsize=10)
            ax.set_title(f"{name} (Error)")

    fig.suptitle('Individual Curves (S-shaped Waypoints)', fontsize=14, y=1.02)
    plt.tight_layout()
    save_figure(fig, os.path.join(OUTPUT_DIR, "01_individual_curves.png"))
    plt.close()


def visualize_comparison():
    """Figure 2: All curves comparison"""
    print("[Figure 2] All curves comparison...")

    fig, ax = plt.subplots(figsize=(12, 8))

    curves = [
        ("Bezier", BezierCurve(step=STEP), COLORS["bezier"]),
        ("B-Spline", BSplineCurve(step=STEP * 0.5), COLORS["bspline"]),
        ("Cubic Spline", CubicSplineCurve(step=STEP), COLORS["cubic_spline"]),
        ("Dubins", DubinsCurve(step=STEP), COLORS["dubins"]),
        ("Reeds-Shepp", ReedsSheppCurve(step=STEP), COLORS["reeds_shepp"]),
        ("Quintic", QuinticCurve(step=STEP), COLORS["quintic"]),
    ]

    for name, curve, color in curves:
        try:
            if isinstance(curve, (DubinsCurve, ReedsSheppCurve)):
                path = curve.generation(start_pose, goal_pose)
            else:
                path = curve.run(waypoints_s_shape)
            xs = [p[0] for p in path]
            ys = [p[1] for p in path]
            ax.plot(xs, ys, '-', color=color, linewidth=2, label=name)
        except Exception as e:
            print(f"  {name}: Error - {e}")

    # 路径点
    wx = [p[0] for p in waypoints_s_shape]
    wy = [p[1] for p in waypoints_s_shape]
    ax.plot(wx, wy, 'ko', markersize=6, label='Waypoints', zorder=5)

    # 对位姿曲线标注起止朝向
    if any(isinstance(c, (DubinsCurve, ReedsSheppCurve)) for _, c, _ in curves):
        for pose, color, label in [(start_pose, 'green', 'Start'), (goal_pose, 'red', 'Goal')]:
            ax.plot(pose[0], pose[1], 'o', color=color, markersize=10, label=label)
            dx = 0.5 * math.cos(pose[2])
            dy = 0.5 * math.sin(pose[2])
            ax.arrow(pose[0], pose[1], dx, dy, head_width=0.2, head_length=0.2,
                     fc=color, ec=color)

    ax.set_title('All Curves Comparison (S-shaped Waypoints)', fontsize=14)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    ax.legend(loc='best')
    save_figure(fig, os.path.join(OUTPUT_DIR, "02_comparison.png"))
    plt.close()


def visualize_bspline_params():
    """Figure 3: B-Spline parameter comparison"""
    print("[Figure 3] B-Spline parameter comparison...")

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    axes = axes.flatten()

    # 3a: 不同阶数
    ax = axes[0]
    for order, color in [(2, 'blue'), (3, 'green'), (4, 'orange'), (5, 'red')]:
        curve = BSplineCurve(step=0.05, order=order)
        path = curve.run(waypoints_s_shape)
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        ax.plot(xs, ys, '-', color=color, linewidth=2, label=f'Order={order}')
    ax.plot([p[0] for p in waypoints_s_shape], [p[1] for p in waypoints_s_shape],
            'ko', markersize=5, label='Waypoints')
    ax.set_title('B-Spline: Different Orders')
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    ax.legend()

    # 3b: 不同参数化模式
    ax = axes[1]
    modes = [
        (BSplineCurve.PARAM_UNIFORM, 'Uniform', 'blue'),
        (BSplineCurve.PARAM_CHORDLENGTH, 'Chord Length', 'green'),
        (BSplineCurve.PARAM_CENTRIPETAL, 'Centripetal', 'orange'),
    ]
    for mode, label, color in modes:
        curve = BSplineCurve(step=0.05, order=3, param_mode=mode)
        path = curve.run(waypoints_s_shape)
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        ax.plot(xs, ys, '-', color=color, linewidth=2, label=label)
    ax.plot([p[0] for p in waypoints_s_shape], [p[1] for p in waypoints_s_shape],
            'ko', markersize=5, label='Waypoints')
    ax.set_title('B-Spline: Different Param Modes (Order=3)')
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    ax.legend()

    # 3c: 插值 vs 逼近
    ax = axes[2]
    curve = BSplineCurve(step=0.05, order=3, spline_mode=BSplineCurve.SPLINE_INTERPOLATION)
    path = curve.run(waypoints_s_shape)
    xs = [p[0] for p in path]
    ys = [p[1] for p in path]
    ax.plot(xs, ys, '-', color='blue', linewidth=2, label='Interpolation')

    curve.set_spline_mode(BSplineCurve.SPLINE_APPROXIMATION)
    path2 = curve.run(waypoints_s_shape)
    xs2 = [p[0] for p in path2]
    ys2 = [p[1] for p in path2]
    ax.plot(xs2, ys2, '--', color='orange', linewidth=2, label='Approximation')

    ax.plot([p[0] for p in waypoints_s_shape], [p[1] for p in waypoints_s_shape],
            'ko', markersize=5, label='Waypoints')
    ax.set_title('B-Spline: Interpolation vs Approximation')
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    ax.legend()

    # 3d: 不同路径形状
    ax = axes[3]
    for path_pts, label, color in [
        (waypoints_s_shape, 'S-shape', 'blue'),
        (waypoints_line, 'Line', 'green'),
        (waypoints_turn, 'Turn', 'orange'),
    ]:
        curve = BSplineCurve(step=0.05, order=3)
        path = curve.run(path_pts)
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        ax.plot(xs, ys, '-', color=color, linewidth=2, label=label)
        ax.plot([p[0] for p in path_pts], [p[1] for p in path_pts],
                'o', color=color, markersize=4, alpha=0.5)
    ax.set_title('B-Spline: Different Path Shapes')
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    ax.legend()

    fig.suptitle('B-Spline Parameter Comparison', fontsize=14, y=1.02)
    plt.tight_layout()
    save_figure(fig, os.path.join(OUTPUT_DIR, "03_bspline_params.png"))
    plt.close()


def visualize_poses():
    """Figure 4: Dubins / Reeds-Shepp pose path comparison"""
    print("[Figure 4] Dubins / Reeds-Shepp pose paths...")

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    # Multiple test scenarios
    scenarios = [
        ((0, 0, 0), (6, 2, 0.5), "Scene 1: forward-right"),
        ((0, 0, 0), (4, 4, 1.57), "Scene 2: vertical-up"),
        ((0, 0, 0), (5, -2, -0.5), "Scene 3: downward-right"),
        ((0, 0, 1.57), (5, 3, 0.0), "Scene 4: start facing up"),
    ]

    for idx, ax in enumerate(axes):
        ax.set_title(['Dubins Curves', 'Reeds-Shepp Curves'][idx])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.grid(True, alpha=0.3)

        CurveClass = [DubinsCurve, ReedsSheppCurve][idx]
        colors = ['blue', 'green', 'orange', 'red']

        for (start, goal, desc), c in zip(scenarios, colors):
            curve = CurveClass(step=STEP, max_curv=0.25)
            path = curve.generation(start, goal)
            if path:
                xs = [p[0] for p in path]
                ys = [p[1] for p in path]
                ax.plot(xs, ys, '-', color=c, linewidth=2, label=desc)
                ax.plot(start[0], start[1], 'o', color=c, markersize=6)
                ax.plot(goal[0], goal[1], 's', color=c, markersize=6)

        ax.legend(fontsize=8)
        ax.axis('equal')

    plt.tight_layout()
    save_figure(fig, os.path.join(OUTPUT_DIR, "04_pose_paths.png"))
    plt.close()


def visualize_quintic_pva():
    """Figure 5: Quintic polynomial PVA curves"""
    print("[Figure 5] Quintic polynomial PVA...")

    # Multiple boundary conditions
    test_cases = [
        ("Const Vel", (0.0, 0.0, 0.0), (10.0, 0.0, 0.0), 5.0),
        ("Init Vel", (0.0, 2.0, 0.0), (10.0, 0.0, 0.0), 4.0),
        ("Init/Final Acc", (0.0, 0.0, 1.0), (10.0, 0.0, -1.0), 5.0),
        ("Fast Motion", (0.0, 0.0, 0.0), (20.0, 0.0, 0.0), 3.0),
    ]

    fig, axes = plt.subplots(4, 4, figsize=(16, 14))

    for row, (name, start, end, T) in enumerate(test_cases):
        poly = QuinticPolynomial()
        poly.solve(start, end, T)
        plot_quintic_pva(axes[row], poly, T)
        axes[row][0].set_ylabel(f'{name}\nPosition')
        axes[row][1].set_ylabel(f'{name}\nVelocity')
        axes[row][2].set_ylabel(f'{name}\nAcceleration')
        axes[row][3].set_ylabel(f'{name}\nJerk')

    fig.suptitle('Quintic Polynomial PVA (Different Boundary Conditions)', fontsize=14, y=1.02)
    plt.tight_layout()
    save_figure(fig, os.path.join(OUTPUT_DIR, "05_quintic_pva.png"))
    plt.close()


def run_all():
    """运行所有可视化"""
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    print("=" * 50)
    print("Curve Generation Visualization")
    print("=" * 50)

    visualize_individual_curves()
    visualize_comparison()
    visualize_bspline_params()
    visualize_poses()
    visualize_quintic_pva()

    print("\nAll visualization completed!")
    print("Output directory: curve_visualization/output/")
    print("=" * 50)


if __name__ == "__main__":
    run_all()
