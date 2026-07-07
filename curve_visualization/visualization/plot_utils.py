"""
Plot utility functions
"""

import math
import matplotlib.pyplot as plt
import numpy as np
from typing import List, Tuple, Optional
from config import COLORS
from curves import (
    BezierCurve, BSplineCurve, CubicSplineCurve,
    DubinsCurve, ReedsSheppCurve, QuinticCurve
)


def plot_curve_with_points(ax, curve_result, waypoints, title, color, label):
    """
    Plot curve and waypoints on the given axes
    """
    if len(curve_result) > 0:
        xs = [p[0] for p in curve_result]
        ys = [p[1] for p in curve_result]
        ax.plot(xs, ys, '-', color=color, linewidth=2, label=label, zorder=3)

    # Draw waypoints
    wx = [p[0] for p in waypoints]
    wy = [p[1] for p in waypoints]
    ax.plot(wx, wy, 'o', color='gray', markersize=6, label='Waypoints', zorder=4)
    ax.plot(wx[0], wy[0], 'go', markersize=8, label='Start', zorder=5)
    ax.plot(wx[-1], wy[-1], 'ro', markersize=8, label='Goal', zorder=5)

    ax.set_title(title)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    ax.legend()


def plot_pose_path(ax, path, color, label, arrow_interval=5):
    """
    Plot path with direction arrows
    """
    if len(path) == 0:
        return

    xs = [p[0] for p in path]
    ys = [p[1] for p in path]
    ax.plot(xs, ys, '-', color=color, linewidth=2, label=label, zorder=3)

    # Draw direction arrows
    for i in range(0, len(path), arrow_interval):
        x, y, theta = path[i]
        dx = 0.3 * math.cos(theta)
        dy = 0.3 * math.sin(theta)
        ax.arrow(x, y, dx, dy, head_width=0.15, head_length=0.15,
                 fc=color, ec=color, alpha=0.6)


def plot_quintic_pva(axs, poly, T, step=0.1):
    """
    Plot quintic polynomial PVA curves
    axs: (ax_pos, ax_vel, ax_acc, ax_jerk) four subplots
    """
    ts = np.arange(0, T + step, step)
    pos = [poly.x(t) for t in ts]
    vel = [poly.dx(t) for t in ts]
    acc = [poly.ddx(t) for t in ts]
    jerk = [poly.dddx(t) for t in ts]

    axs[0].plot(ts, pos, 'b-', linewidth=2)
    axs[0].set_ylabel('Position')
    axs[0].grid(True, alpha=0.3)

    axs[1].plot(ts, vel, 'g-', linewidth=2)
    axs[1].set_ylabel('Velocity')
    axs[1].grid(True, alpha=0.3)

    axs[2].plot(ts, acc, 'r-', linewidth=2)
    axs[2].set_ylabel('Acceleration')
    axs[2].grid(True, alpha=0.3)

    axs[3].plot(ts, jerk, 'm-', linewidth=2)
    axs[3].set_ylabel('Jerk')
    axs[3].set_xlabel('Time (s)')
    axs[3].grid(True, alpha=0.3)


def save_figure(fig, filename):
    """Save figure to file"""
    fig.savefig(filename, dpi=150, bbox_inches='tight')
    print(f"  Saved: {filename}")
