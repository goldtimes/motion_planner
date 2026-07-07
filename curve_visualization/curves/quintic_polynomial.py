"""
五次多项式 (Quintic Polynomial)

对应 C++: quintic_polynomial.h / quintic_polynomial.cpp

算法:
  - x(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
  - 边界条件: 起始/终止 位置、速度、加速度
  - 6 个方程解 6 个系数
"""

from typing import List, Tuple
from .base_curve import Curve, Point2d, Point3d, Points2d, Points3d


class QuinticPolynomial:
    """
    五次多项式，对应 C++ QuinticPolynomial

    注意：这是一个独立的轨迹生成器，不同于 Curve 的子类。
    它生成的是 1-DOF 运动的位置/速度/加速度随时间的变化。
    """

    def __init__(self, params: Tuple[float, float, float, float, float, float] = None):
        """
        构造函数
        @param params: (a0, a1, a2, a3, a4, a5)
        """
        if params:
            self.a0, self.a1, self.a2, self.a3, self.a4, self.a5 = params
        else:
            self.a0 = self.a1 = self.a2 = self.a3 = self.a4 = self.a5 = 0.0

    def update(self, params: Tuple[float, float, float, float, float, float]):
        """更新多项式系数"""
        self.a0, self.a1, self.a2, self.a3, self.a4, self.a5 = params

    def solve(self, start_pva: Tuple[float, float, float],
              end_pva: Tuple[float, float, float], T: float):
        """
        根据边界条件求解五次多项式系数
        对应 C++ solve(const array<double,3>& start_pva, const array<double,3>& end_pva, double T)

        @param start_pva: (p0, v0, a0) 起始位置、速度、加速度
        @param end_pva:   (p1, v1, a1) 终止位置、速度、加速度
        @param T: 总时间
        """
        p0, v0, a0 = start_pva
        p1, v1, a1 = end_pva

        T2 = T * T
        T3 = T2 * T
        T4 = T3 * T
        T5 = T4 * T

        self.a0 = p0
        self.a1 = v0
        self.a2 = a0 / 2.0

        self.a3 = (20.0 * (p1 - p0) - (8.0 * v1 + 12.0 * v0) * T
                   - (3.0 * a0 - a1) * T2) / (2.0 * T3)
        self.a4 = (30.0 * (p0 - p1) + (14.0 * v1 + 16.0 * v0) * T
                   + (3.0 * a0 - 2.0 * a1) * T2) / (2.0 * T4)
        self.a5 = (12.0 * (p1 - p0) - 6.0 * (v1 + v0) * T
                   - (a0 - a1) * T2) / (2.0 * T5)

    def x(self, t: float) -> float:
        """位置，对应 C++ x(double t)"""
        return (self.a0 + self.a1 * t + self.a2 * t * t + self.a3 * t * t * t
                + self.a4 * t * t * t * t + self.a5 * t * t * t * t * t)

    def dx(self, t: float) -> float:
        """速度，对应 C++ dx(double t)"""
        return (self.a1 + 2.0 * self.a2 * t + 3.0 * self.a3 * t * t
                + 4.0 * self.a4 * t * t * t + 5.0 * self.a5 * t * t * t * t)

    def ddx(self, t: float) -> float:
        """加速度，对应 C++ ddx(double t)"""
        return (2.0 * self.a2 + 6.0 * self.a3 * t + 12.0 * self.a4 * t * t
                + 20.0 * self.a5 * t * t * t)

    def dddx(self, t: float) -> float:
        """加加速度 (Jerk)，对应 C++ dddx(double t)"""
        return (6.0 * self.a3 + 24.0 * self.a4 * t + 60.0 * self.a5 * t * t)

    def ddddx(self, t: float) -> float:
        """加加加速度 (Snap)，对应 C++ ddddx(double t)"""
        return 24.0 * self.a4 + 120.0 * self.a5 * t


class QuinticCurve(Curve):
    """
    使用五次多项式生成路径点的适配器
    对 x 和 y 方向分别用 QuinticPolynomial 做运动规划
    """

    def __init__(self, step: float = 0.1):
        super().__init__(step)

    def run(self, points: Points2d) -> Points3d:
        """通过路径点生成（简化：假设匀速分段）"""
        if len(points) < 2:
            return []
        # 使用首尾点
        sx, sy = points[0]
        gx, gy = points[-1]
        T = ((gx - sx) ** 2 + (gy - sy) ** 2) ** 0.5 / 2.0  # 假设速度约 2 m/s
        T = max(T, 1.0)

        poly_x = QuinticPolynomial()
        poly_y = QuinticPolynomial()

        poly_x.solve((sx, 0.0, 0.0), (gx, 0.0, 0.0), T)
        poly_y.solve((sy, 0.0, 0.0), (gy, 0.0, 0.0), T)

        path = []
        t = 0.0
        while t <= T:
            path.append((poly_x.x(t), poly_y.x(t), 0.0))
            t += self.step
        if t - self.step < T:
            path.append((poly_x.x(T), poly_y.x(T), 0.0))
        return path

    def generation(self, start: Point3d, goal: Point3d) -> Points3d:
        """从起始到目标位姿生成五次多项式轨迹"""
        sx, sy = start[0], start[1]
        gx, gy = goal[0], goal[1]
        points_2d = [(sx, sy), (gx, gy)]
        return self.run(points_2d)
