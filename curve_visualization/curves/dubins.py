"""
Dubins 曲线生成

对应 C++: dubins_curve.h / dubins_curve.cpp

算法:
  - 6 种运动模式: LSL, RSR, LSR, RSL, RLR, LRL
  - 选择最短路径
  - L: 左转, R: 右转, S: 直行
"""

import math
from typing import List, Tuple, Callable
from .base_curve import Curve, Point2d, Point3d, Points2d, Points3d

# 运动模式常量
DUBINS_L = 0  # 左转
DUBINS_S = 1  # 直行
DUBINS_R = 2  # 右转


class DubinsCurve(Curve):
    """Dubins 曲线，对应 C++ DubinsCurve"""

    def __init__(self, step: float = 0.1, max_curv: float = 0.25):
        super().__init__(step)
        self.max_curv = max_curv  # 最大曲率 = 1 / 转弯半径

    @staticmethod
    def _mod2pi(theta: float) -> float:
        """将角度规范化到 [0, 2*pi)"""
        while theta >= 2.0 * math.pi:
            theta -= 2.0 * math.pi
        while theta < 0.0:
            theta += 2.0 * math.pi
        return theta

    def _interpolate(self, mode: int, length: float,
                     init_pose: Point3d) -> Point3d:
        """
        单步插值
        对应 C++ interpolate(int mode, double length, const Point3d& init_pose)
        """
        x, y, yaw = init_pose
        if mode == DUBINS_S:  # 直行
            x += length * math.cos(yaw)
            y += length * math.sin(yaw)
        elif mode == DUBINS_L:  # 左转 (半径 = 1/max_curv)
            x += math.sin(yaw + length * self.max_curv) / self.max_curv \
                 - math.sin(yaw) / self.max_curv
            y += -math.cos(yaw + length * self.max_curv) / self.max_curv \
                 + math.cos(yaw) / self.max_curv
            yaw += length * self.max_curv
        elif mode == DUBINS_R:  # 右转
            x += -math.sin(yaw - length * self.max_curv) / self.max_curv \
                 + math.sin(yaw) / self.max_curv
            y += math.cos(yaw - length * self.max_curv) / self.max_curv \
                 - math.cos(yaw) / self.max_curv
            yaw -= length * self.max_curv
        return (x, y, yaw)

    # ---------- 6 种运动模式 ----------

    def LSL(self, alpha: float, beta: float, dist: float):
        """Left-Straight-Left"""
        sa = math.sin(alpha)
        sb = math.sin(beta)
        ca = math.cos(alpha)
        cb = math.cos(beta)
        c_ab = math.cos(alpha - beta)

        tmp0 = dist + sa - sb
        p_squared = 2 + (dist * dist) - 2 * c_ab + 2 * dist * (sa - sb)
        if p_squared < 0:
            return None
        tmp1 = math.atan2(cb - ca, tmp0)
        t = self._mod2pi(-alpha + tmp1)
        p = math.sqrt(max(0, p_squared))
        q = self._mod2pi(beta - tmp1)
        return (t, p, q)

    def RSR(self, alpha: float, beta: float, dist: float):
        """Right-Straight-Right"""
        sa = math.sin(alpha)
        sb = math.sin(beta)
        ca = math.cos(alpha)
        cb = math.cos(beta)
        c_ab = math.cos(alpha - beta)

        tmp0 = dist - sa + sb
        p_squared = 2 + (dist * dist) - 2 * c_ab + 2 * dist * (sb - sa)
        if p_squared < 0:
            return None
        tmp1 = math.atan2(ca - cb, tmp0)
        t = self._mod2pi(alpha - tmp1)
        p = math.sqrt(max(0, p_squared))
        q = self._mod2pi(-beta + tmp1)
        return (t, p, q)

    def LSR(self, alpha: float, beta: float, dist: float):
        """Left-Straight-Right"""
        sa = math.sin(alpha)
        sb = math.sin(beta)
        ca = math.cos(alpha)
        cb = math.cos(beta)
        c_ab = math.cos(alpha - beta)

        p_squared = -2 + (dist * dist) + 2 * c_ab + 2 * dist * (sa + sb)
        if p_squared < 0:
            return None
        p = math.sqrt(max(0, p_squared))
        tmp2 = math.atan2(-ca - cb, dist + sa + sb) - math.atan2(-2.0, p)
        t = self._mod2pi(-alpha + tmp2)
        q = self._mod2pi(-self._mod2pi(beta) + tmp2)
        return (t, p, q)

    def RSL(self, alpha: float, beta: float, dist: float):
        """Right-Straight-Left"""
        sa = math.sin(alpha)
        sb = math.sin(beta)
        ca = math.cos(alpha)
        cb = math.cos(beta)
        c_ab = math.cos(alpha - beta)

        p_squared = -2 + (dist * dist) + 2 * c_ab - 2 * dist * (sa + sb)
        if p_squared < 0:
            return None
        p = math.sqrt(max(0, p_squared))
        tmp2 = math.atan2(ca + cb, dist - sa - sb) - math.atan2(2.0, p)
        t = self._mod2pi(alpha - tmp2)
        q = self._mod2pi(beta - tmp2)
        return (t, p, q)

    def RLR(self, alpha: float, beta: float, dist: float):
        """Right-Left-Right"""
        sa = math.sin(alpha)
        sb = math.sin(beta)
        ca = math.cos(alpha)
        cb = math.cos(beta)
        c_ab = math.cos(alpha - beta)

        tmp_rlr = (6.0 - dist * dist + 2 * c_ab + 2 * dist * (sa - sb)) / 8.0
        if abs(tmp_rlr) > 1:
            return None
        p = self._mod2pi(2 * math.pi - math.acos(tmp_rlr))
        tmp2 = math.atan2(ca - cb, dist - sa + sb)
        t = self._mod2pi(-alpha + tmp2 - p / 2.0)
        q = self._mod2pi(beta - alpha - t + self._mod2pi(p))
        return (t, p, q)

    def LRL(self, alpha: float, beta: float, dist: float):
        """Left-Right-Left"""
        sa = math.sin(alpha)
        sb = math.sin(beta)
        ca = math.cos(alpha)
        cb = math.cos(beta)
        c_ab = math.cos(alpha - beta)

        tmp_lrl = (6.0 - dist * dist + 2 * c_ab + 2 * dist * (-sa + sb)) / 8.0
        if abs(tmp_lrl) > 1:
            return None
        p = self._mod2pi(2 * math.pi - math.acos(tmp_lrl))
        tmp2 = math.atan2(ca - cb, dist + sa - sb)
        t = self._mod2pi(-alpha + tmp2 - p / 2.0)
        q = self._mod2pi(beta - alpha - t + self._mod2pi(p))
        return (t, p, q)

    # ---------- 路径生成 ----------

    def generation(self, start: Point3d, goal: Point3d) -> Points3d:
        """
        生成 Dubins 曲线
        对应 C++ generation(const Point3d&, const Point3d&, Points3d&)
        """
        sx, sy, syaw = start
        gx, gy, gyaw = goal

        # 转换到相对坐标系
        dx = gx - sx
        dy = gy - sy
        dist = math.sqrt(dx * dx + dy * dy) * self.max_curv
        theta = self._mod2pi(math.atan2(dy, dx))
        alpha = self._mod2pi(syaw - theta)
        beta = self._mod2pi(gyaw - theta)

        # 6 种模式
        solvers = [self.LSL, self.RSR, self.LSR, self.RSL, self.RLR, self.LRL]
        mode_names = ["LSL", "RSR", "LSR", "RSL", "RLR", "LRL"]

        best_cost = float('inf')
        best_params = None
        best_mode_idx = -1

        for i, solver in enumerate(solvers):
            params = solver(alpha, beta, dist)
            if params is not None:
                t, p, q = params
                cost = abs(t) + abs(p) + abs(q)
                if cost < best_cost:
                    best_cost = cost
                    best_params = (t, p, q)
                    best_mode_idx = i

        if best_params is None or best_mode_idx == -1:
            return []

        t, p, q = best_params

        # 映射回原始坐标
        lengths = [t, p, q]
        # 从模式名解析分段运动类型
        # 如 "LSL" -> [DUBINS_L, DUBINS_S, DUBINS_L]
        mode_str = mode_names[best_mode_idx]
        mode_map = {'L': DUBINS_L, 'S': DUBINS_S, 'R': DUBINS_R}
        modes = [mode_map[c] for c in mode_str]

        # 插值生成路径
        path = []
        current = (0.0, 0.0, 0.0)
        path.append((sx, sy, syaw))

        for seg_len, seg_mode in zip(lengths, modes):
            seg_len_abs = abs(seg_len)
            seg_steps = max(1, int(seg_len_abs / (self.step * self.max_curv)))
            for step_i in range(1, seg_steps + 1):
                frac = step_i / seg_steps
                step_len = seg_len * frac - (seg_len * (step_i - 1) / seg_steps)
                # 简化：实际应该逐步计算，这里直接用插值
                # 我们用前面的累加方式
                pass

        # 重新实现更简单的分段插值
        path = self._interpolate_path(modes, lengths)
        # 转换到世界坐标
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        world_path = []
        for px, py, pyaw in path:
            # 缩放
            px /= self.max_curv
            py /= self.max_curv
            # 旋转 + 平移
            wx = cos_t * px - sin_t * py + sx
            wy = sin_t * px + cos_t * py + sy
            wyaw = self._mod2pi(pyaw + theta)
            world_path.append((wx, wy, wyaw))
        return world_path

    def _interpolate_path(self, modes: List[int],
                          lengths: List[float]) -> Points3d:
        """根据分段模式和长度生成插值路径（在相对坐标系中）"""
        path = [(0.0, 0.0, 0.0)]
        current = (0.0, 0.0, 0.0)

        for seg_mode, seg_len in zip(modes, lengths):
            if abs(seg_len) < 1e-10:
                continue
            # 分段步数
            seg_steps = max(2, int(abs(seg_len) / (self.step * self.max_curv)))
            for i in range(1, seg_steps + 1):
                frac = i / seg_steps
                step_len = seg_len * frac
                # 重新从起点计算更准确，这里简化处理
                # 实际上应逐步积分
                current = self._interpolate(seg_mode, step_len, (0.0, 0.0, 0.0))
                # 但这样不准，改用逐步方式
                pass

        # 改用逐步方式
        path = [(0.0, 0.0, 0.0)]
        current = (0.0, 0.0, 0.0)
        for seg_mode, seg_len in zip(modes, lengths):
            if abs(seg_len) < 1e-10:
                continue
            remaining = seg_len
            while abs(remaining) > 1e-6:
                step_len = min(abs(remaining), self.step * self.max_curv)
                step_len = step_len if seg_len > 0 else -step_len
                current = self._interpolate(seg_mode, step_len, current)
                path.append(current)
                remaining -= step_len

        return path

    def run(self, points: Points2d) -> Points3d:
        """通过多个路径点运行（简化：取首尾）"""
        if len(points) < 2:
            return []
        start = (points[0][0], points[0][1], 0.0)
        goal = (points[-1][0], points[-1][1], 0.0)
        return self.generation(start, goal)

    def set_max_curv(self, max_curv: float):
        self.max_curv = max_curv
