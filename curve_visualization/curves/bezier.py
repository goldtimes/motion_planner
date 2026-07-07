"""
Bezier 曲线生成

对应 C++: bezier_curve.h / bezier_curve.cpp

算法:
  - 使用伯恩斯坦多项式计算曲线点
  - 启发式控制点生成
"""

import math
from typing import List, Tuple
from .base_curve import Curve, Point2d, Point3d, Points2d, Points3d


class BezierCurve(Curve):
    """Bezier 曲线，对应 C++ BezierCurve"""

    def __init__(self, step: float = 0.1, offset: float = 3.0):
        super().__init__(step)
        self.offset = offset

    def _comb(self, n: int, r: int) -> int:
        """计算组合数 C(n, r)"""
        return math.comb(n, r)

    def bezier(self, t: float, control_pts: Points2d) -> Point2d:
        """
        计算 Bezier 曲线上参数 t 对应的点
        对应 C++ bezier(double t, const Points2d& control_pts)
        """
        n = len(control_pts) - 1
        x, y = 0.0, 0.0
        for i, (cpx, cpy) in enumerate(control_pts):
            coeff = self._comb(n, i) * (t ** i) * ((1 - t) ** (n - i))
            x += coeff * cpx
            y += coeff * cpy
        return (x, y)

    def get_control_points(self, start: Point3d, goal: Point3d) -> Points2d:
        """
        启发式计算控制点
        对应 C++ getControlPoints(const Point3d& start, const Point3d& goal)
        """
        sx, sy, syaw = start
        gx, gy, gyaw = goal

        # 沿起始方向偏移
        p1 = (sx + self.offset * math.cos(syaw),
              sy + self.offset * math.sin(syaw))
        # 沿目标反方向偏移
        p2 = (gx - self.offset * math.cos(gyaw),
              gy - self.offset * math.sin(gyaw))

        return [(sx, sy), p1, p2, (gx, gy)]

    def generation(self, start: Point3d, goal: Point3d) -> Points3d:
        """
        从起始位姿到目标位姿生成 Bezier 曲线
        对应 C++ generation(const Point3d&, const Point3d&, Points3d&)
        """
        cps = self.get_control_points(start, goal)
        path: Points3d = []
        t = 0.0
        while t <= 1.0:
            px, py = self.bezier(t, cps)
            path.append((px, py, 0.0))
            t += self.step
        # 确保终点被包含
        if t - self.step < 1.0:
            px, py = self.bezier(1.0, cps)
            path.append((px, py, 0.0))
        return path

    def run(self, points: Points2d) -> Points3d:
        """
        通过多个路径点生成 Bezier 曲线（分段连接）
        对应 C++ run(const Points2d&, Points3d&)
        """
        # 对于多段路径，将每相邻两点视为 start/goal 分段生成
        # 这里简化处理：取第一个和最后一个点的朝向近似
        if len(points) < 2:
            return []

        # 估算起始终止朝向
        dx = points[1][0] - points[0][0]
        dy = points[1][1] - points[0][1]
        start_theta = math.atan2(dy, dx)

        dx = points[-1][0] - points[-2][0]
        dy = points[-1][1] - points[-2][1]
        goal_theta = math.atan2(dy, dx)

        start = (points[0][0], points[0][1], start_theta)
        goal = (points[-1][0], points[-1][1], goal_theta)

        return self.generation(start, goal)

    def set_offset(self, offset: float):
        """设置控制点偏移量"""
        self.offset = offset
