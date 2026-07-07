"""
B-Spline 曲线生成

对应 C++: bspline_curve.h / bspline_curve.cpp

算法:
  - Cox-deBoor 基函数递推
  - 支持均匀/弦长/向心参数化
  - 支持插值/逼近模式
"""

import math
import numpy as np
from typing import List, Tuple
from .base_curve import Curve, Point2d, Point3d, Points2d, Points3d


class BSplineCurve(Curve):
    """B-Spline 曲线，对应 C++ BSplineCurve"""

    PARAM_UNIFORM = 0
    PARAM_CENTRIPETAL = 1
    PARAM_CHORDLENGTH = 2

    SPLINE_INTERPOLATION = 0
    SPLINE_APPROXIMATION = 1

    def __init__(self, step: float = 0.01, order: int = 3,
                 param_mode: int = PARAM_CHORDLENGTH,
                 spline_mode: int = SPLINE_INTERPOLATION):
        super().__init__(step)
        self.order = order
        self.param_mode = param_mode
        self.spline_mode = spline_mode
        self.start_angle = 0.0
        self.goal_angle = 0.0

    def base_function(self, i: int, k: int, t: float, knot: List[float]) -> float:
        """
        Cox-deBoor 递推基函数
        对应 C++ baseFunction(int i, int k, double t, const vector<double>& knot)
        """
        if k == 0:
            return 1.0 if knot[i] <= t < knot[i + 1] else 0.0

        # 为避免除以零，加极小值
        eps = 1e-10

        left_val = 0.0
        denom = knot[i + k] - knot[i]
        if denom > eps:
            left_val = ((t - knot[i]) / denom) * self.base_function(i, k - 1, t, knot)

        right_val = 0.0
        denom = knot[i + k + 1] - knot[i + 1]
        if denom > eps:
            right_val = ((knot[i + k + 1] - t) / denom) * self.base_function(i + 1, k - 1, t, knot)

        return left_val + right_val

    def param_selection(self, points: Points2d) -> List[float]:
        """
        参数化：均匀 / 弦长 / 向心
        对应 C++ paramSelection(const Points2d& points)
        """
        n = len(points)
        if n <= 1:
            return [0.0]

        params = [0.0]
        if self.param_mode == self.PARAM_UNIFORM:
            # 均匀参数化
            for i in range(1, n):
                params.append(float(i) / (n - 1))
        elif self.param_mode == self.PARAM_CENTRIPETAL:
            # 向心参数化
            total = 0.0
            segments = [0.0]
            for i in range(1, n):
                dx = points[i][0] - points[i - 1][0]
                dy = points[i][1] - points[i - 1][1]
                d = math.sqrt(dx * dx + dy * dy)
                total += math.sqrt(d)
                segments.append(total)
            for i in range(1, n):
                params.append(segments[i] / total if total > 0 else float(i) / (n - 1))
        else:  # PARAM_CHORDLENGTH
            total = 0.0
            segments = [0.0]
            for i in range(1, n):
                dx = points[i][0] - points[i - 1][0]
                dy = points[i][1] - points[i - 1][1]
                d = math.sqrt(dx * dx + dy * dy)
                total += d
                segments.append(total)
            for i in range(1, n):
                params.append(segments[i] / total if total > 0 else float(i) / (n - 1))

        return params

    def knot_generation(self, param: List[float], n: int) -> List[float]:
        """
        生成节点向量
        对应 C++ knotGeneration(const vector<double>& param, int n)
        """
        k = self.order
        n_knots = n + k + 1
        knot = [0.0] * n_knots

        for i in range(n_knots):
            if i <= k:
                knot[i] = 0.0
            elif i >= n:
                knot[i] = 1.0
            else:
                # 平均值法
                sum_val = 0.0
                for j in range(i - k, i):
                    sum_val += param[j]
                knot[i] = sum_val / k

        return knot

    def interpolation(self, points: Points2d, param: List[float],
                      knot: List[float]) -> Points2d:
        """
        B-Spline 插值：求解控制点使曲线通过所有数据点
        对应 C++ interpolation(...)
        """
        n = len(points)
        k = self.order
        # 构建系数矩阵 N (n x n)
        N = np.zeros((n, n))
        for i in range(n):
            for j in range(n):
                N[i, j] = self.base_function(j, k, param[i], knot)

        # 右侧数据点
        X = np.array([p[0] for p in points])
        Y = np.array([p[1] for p in points])

        # 求解 N * P = D  =>  P = N^{-1} * D
        try:
            N_inv = np.linalg.inv(N)
            cx = N_inv @ X
            cy = N_inv @ Y
        except np.linalg.LinAlgError:
            # 若奇异，用最小二乘
            cx, _, _, _ = np.linalg.lstsq(N, X, rcond=None)
            cy, _, _, _ = np.linalg.lstsq(N, Y, rcond=None)

        return list(zip(cx, cy))

    def approximation(self, points: Points2d, param: List[float],
                      knot: List[float], h: int = 0) -> Points2d:
        """
        B-Spline 逼近：最小二乘拟合
        对应 C++ approximation(...)
        """
        n = len(points)
        k = self.order

        if h <= k or h >= n:
            h = n - 1  # 默认控制点数量

        # 构建系数矩阵 N (n x h)
        N = np.zeros((n, h))
        for i in range(n):
            for j in range(h):
                N[i, j] = self.base_function(j, k, param[i], knot)

        # 最小二乘: N^T N P = N^T D
        X = np.array([p[0] for p in points])
        Y = np.array([p[1] for p in points])

        A = N.T @ N
        bX = N.T @ X
        bY = N.T @ Y

        try:
            cx = np.linalg.solve(A, bX)
            cy = np.linalg.solve(A, bY)
        except np.linalg.LinAlgError:
            cx, _, _, _ = np.linalg.lstsq(N, X, rcond=None)
            cy, _, _, _ = np.linalg.lstsq(N, Y, rcond=None)

        return list(zip(cx, cy))

    def bspline_generation(self, k: int, knot: List[float],
                           control_pts: Points2d) -> Points2d:
        """
        根据控制点生成 B-Spline 上的密集点
        对应 C++ bsplineGeneration(...)
        """
        n = len(control_pts)
        path: Points2d = []

        t = knot[self.order]
        end_t = knot[n]
        while t <= end_t:
            x, y = 0.0, 0.0
            for i in range(n):
                bf = self.base_function(i, k, t, knot)
                x += bf * control_pts[i][0]
                y += bf * control_pts[i][1]
            path.append((x, y))
            t += self.step
        # 确保终点
        if t - self.step < end_t:
            x, y = 0.0, 0.0
            for i in range(n):
                bf = self.base_function(i, k, end_t, knot)
                x += bf * control_pts[i][0]
                y += bf * control_pts[i][1]
            path.append((x, y))

        return path

    def run(self, points: Points2d) -> Points3d:
        """
        运行 B-Spline 轨迹生成
        对应 C++ run(const Points2d&, Points3d&)
        """
        if len(points) < 2:
            return []

        param = self.param_selection(points)
        knot = self.knot_generation(param, len(points))

        if self.spline_mode == self.SPLINE_INTERPOLATION:
            control_pts = self.interpolation(points, param, knot)
        else:
            control_pts = self.approximation(points, param, knot)

        raw_path = self.bspline_generation(self.order, knot, control_pts)

        # 转换为 Points3d (theta 为 0，后续可计算)
        path = [(p[0], p[1], 0.0) for p in raw_path]
        return path

    def generation(self, start: Point3d, goal: Point3d) -> Points3d:
        """
        从起始位姿到目标位姿生成 B-Spline
        对应 C++ generation(...)
        """
        self.start_angle = start[2]
        self.goal_angle = goal[2]

        # 构造中间点：用启发式控制点
        sx, sy, syaw = start
        gx, gy, gyaw = goal

        # 沿方向延伸作为中间点
        dist = math.sqrt((gx - sx) ** 2 + (gy - sy) ** 2)
        mid1 = (sx + dist / 3.0 * math.cos(syaw),
                sy + dist / 3.0 * math.sin(syaw))
        mid2 = (gx - dist / 3.0 * math.cos(gyaw),
                gy - dist / 3.0 * math.sin(gyaw))

        points_2d = [(sx, sy), mid1, mid2, (gx, gy)]
        return self.run(points_2d)

    def set_order(self, order: int):
        self.order = order

    def set_param_mode(self, mode: int):
        self.param_mode = mode

    def set_spline_mode(self, mode: int):
        self.spline_mode = mode
