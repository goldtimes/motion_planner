"""
三次样条曲线生成 (Cubic Spline)

对应 C++: cubic_spline_curve.h / cubic_spline_curve.cpp

算法:
  - 以累积弦长为参数 s
  - 对 x(s) 和 y(s) 分别做三次样条插值
  - 使用自然边界条件 (二阶导为 0)
"""

import numpy as np
from typing import List, Tuple
from .base_curve import Curve, Point2d, Point3d, Points2d, Points3d


class CubicSplineCurve(Curve):
    """三次样条曲线，对应 C++ CubicSplineCurve"""

    def __init__(self, step: float = 0.1):
        super().__init__(step)
        self.start_angle = 0.0
        self.goal_angle = 0.0

    def _calc_s(self, points: Points2d) -> List[float]:
        """计算累积弦长参数 s"""
        n = len(points)
        s = [0.0]
        for i in range(1, n):
            dx = points[i][0] - points[i - 1][0]
            dy = points[i][1] - points[i - 1][1]
            ds = (dx * dx + dy * dy) ** 0.5
            s.append(s[i - 1] + ds)
        # 归一化到 [0, 1]
        if s[-1] > 0:
            s = [v / s[-1] for v in s]
        return s

    def _spline(self, s_list: List[float], dir_list: List[float],
                t_list: List[float]) -> List[float]:
        """
        在某一方向 (x 或 y) 上计算样条值
        对应 C++ spline(const vector<double>&, const vector<double>&, const vector<double>&)

        使用三弯矩法求解自然三次样条
        """
        n = len(s_list)
        h = [s_list[i + 1] - s_list[i] for i in range(n - 1)]

        # 构建三对角矩阵求解二阶导 M
        # 自然边界: M[0] = M[n-1] = 0
        A = np.zeros((n - 2, n - 2))
        b = np.zeros(n - 2)

        for i in range(n - 2):
            if i > 0:
                A[i, i - 1] = h[i] / (h[i] + h[i + 1])
            A[i, i] = 2.0
            if i < n - 3:
                A[i, i + 1] = h[i + 1] / (h[i] + h[i + 1])

            # 右侧: 6 * (差分商之差) / (h[i] + h[i+1])
            df1 = (dir_list[i + 2] - dir_list[i + 1]) / h[i + 1]
            df0 = (dir_list[i + 1] - dir_list[i]) / h[i]
            b[i] = 6.0 * (df1 - df0) / (h[i] + h[i + 1])

        # 求解
        M_interior = np.linalg.solve(A, b)
        M = [0.0] + M_interior.tolist() + [0.0]

        # 对每个目标 t 计算样条值
        result = []
        for t in t_list:
            # 找到 t 所在的区间
            if t <= s_list[0]:
                idx = 0
            elif t >= s_list[-1]:
                idx = n - 2
            else:
                idx = 0
                for i in range(n - 1):
                    if s_list[i] <= t <= s_list[i + 1]:
                        idx = i
                        break

            s0, s1 = s_list[idx], s_list[idx + 1]
            d0, d1 = dir_list[idx], dir_list[idx + 1]
            m0, m1 = M[idx], M[idx + 1]
            h_i = s1 - s0

            if h_i == 0:
                result.append(d0)
                continue

            # 三次 Hermite 插值
            a = (d0 * (s1 - t) + d1 * (t - s0)) / h_i
            b = ((m0 * ((s1 - t) ** 3 - h_i * h_i * (s1 - t))
                  + m1 * ((t - s0) ** 3 - h_i * h_i * (t - s0))) / (6.0 * h_i))
            result.append(a + b)

        return result

    def run(self, points: Points2d) -> Points3d:
        """
        运行三次样条插值
        对应 C++ run(const Points2d&, Points3d&)
        """
        if len(points) < 3:
            return []

        s_list = self._calc_s(points)
        x_list = [p[0] for p in points]
        y_list = [p[1] for p in points]

        # 生成密集插值点
        t_list = []
        t = 0.0
        while t <= 1.0:
            t_list.append(t)
            t += self.step
        if t - self.step < 1.0:
            t_list.append(1.0)

        x_interp = self._spline(s_list, x_list, t_list)
        y_interp = self._spline(s_list, y_list, t_list)

        path = [(x_interp[i], y_interp[i], 0.0) for i in range(len(t_list))]
        return path

    def generation(self, start: Point3d, goal: Point3d) -> Points3d:
        """
        从起始到目标位姿生成三次样条
        对应 C++ generation(const Point3d&, const Point3d&, Points3d&)
        """
        self.start_angle = start[2]
        self.goal_angle = goal[2]

        sx, sy, syaw = start
        gx, gy, gyaw = goal

        # 构造中间点
        dist = ((gx - sx) ** 2 + (gy - sy) ** 2) ** 0.5
        mid1 = (sx + dist / 3.0 * np.cos(syaw),
                sy + dist / 3.0 * np.sin(syaw))
        mid2 = (gx - dist / 3.0 * np.cos(gyaw),
                gy - dist / 3.0 * np.sin(gyaw))

        points_2d = [(sx, sy), mid1, mid2, (gx, gy)]
        return self.run(points_2d)
