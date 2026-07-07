"""
Reeds-Shepp 曲线生成

对应 C++: reeds_shepp_curve.h / reeds_shepp_curve.cpp

算法:
  - 支持 backward 运动（倒车）
  - 比 Dubins 更多运动模式（CSC, CCC, CCCC, CCSC, CCSCC, SCS）
  - 使用反射、时间翻转和逆向操作生成 48 种模式
  - 选择最短有效路径
"""

import math
from typing import List, Tuple, Optional
from .base_curve import Curve, Point2d, Point3d, Points2d, Points3d

# 类型别名
RSLength = Tuple[float, float, float]
RSParams = Optional[RSLength]

# 运动模式常量
RS_NOP = 0
RS_LEFT = 1
RS_STRAIGHT = 2
RS_RIGHT = 3

# 方向常量
RS_FWD = 1
RS_BACK = -1


class RSPath:
    """Reeds-Shepp 路径段，对应 C++ RSPath"""

    def __init__(self, lengths: List[float], ctypes: List[int]):
        self.lengths = lengths
        self.ctypes = ctypes

    def len(self) -> float:
        return sum(abs(l) for l in self.lengths)

    def valid(self) -> bool:
        return all(not math.isnan(l) and not math.isinf(l) for l in self.lengths)

    def size(self) -> int:
        return len(self.lengths)

    def get(self, i: int) -> Tuple[float, int]:
        return self.lengths[i], self.ctypes[i]


class ReedsSheppCurve(Curve):
    """Reeds-Shepp 曲线，对应 C++ ReedsSheppCurve"""

    def __init__(self, step: float = 0.1, max_curv: float = 0.25):
        super().__init__(step)
        self.max_curv = max_curv

    @staticmethod
    def _R(x: float, y: float) -> Tuple[float, float]:
        """极坐标 (r, theta)，对应 C++ R(x, y)"""
        r = math.sqrt(x * x + y * y)
        theta = math.atan2(y, x)
        return r, theta

    @staticmethod
    def _M(theta: float) -> float:
        """将角度规范到 [-pi, pi]"""
        while theta > math.pi:
            theta -= 2.0 * math.pi
        while theta < -math.pi:
            theta += 2.0 * math.pi
        return theta

    @staticmethod
    def _polar(x: float, y: float) -> Tuple[float, float]:
        """Polar coordinates"""
        r = math.hypot(x, y)
        theta = math.atan2(y, x)
        return r, theta

    @staticmethod
    def _tau(x: float, y: float, phi: float) -> float:
        """辅助函数"""
        return ReedsSheppCurve._M(math.atan2(y, x) - phi)

    # ========== 基本模式 ==========

    def SLS(self, x: float, y: float, phi: float) -> Optional[RSLength]:
        """Straight-Left-Straight"""
        r, theta = self._polar(x, y)
        t = self._M(theta - phi)
        u = self._M(theta)
        v = r
        # 实际模式: S-L-S 或 S-R-S
        return (t, u, v)

    def LRL(self, x: float, y: float, phi: float) -> Optional[RSLength]:
        """Left-Right-Left (L+R-L-)"""
        r, theta = self._polar(x, y)
        if r < 4.0:
            return None
        acos_val = math.acos(min(1.0, 4.0 / r))
        t = self._M(theta - phi + acos_val)
        u = self._M(theta - phi - acos_val)
        v = math.sqrt(r * r - 16.0)
        return (t, u, v)

    def LSL(self, x: float, y: float, phi: float) -> Optional[RSLength]:
        """Left-Straight-Left (L+S+L+)"""
        r, theta = self._polar(x, y)
        u = self._M(theta - phi)
        v = self._M(theta)
        t = r
        return (t, u, v)

    def LSR(self, x: float, y: float, phi: float) -> Optional[RSLength]:
        """Left-Straight-Right (L+S+R+)"""
        r, theta = self._polar(x, y)
        if r < 2.0:
            return None
        asin_val = math.asin(min(1.0, 2.0 / r))
        t = self._M(theta - phi + asin_val)
        u = self._M(theta - phi - asin_val)
        v = math.sqrt(r * r - 4.0)
        return (t, u, v)

    def LRLRn(self, x: float, y: float, phi: float) -> Optional[RSLength]:
        """Left-Right(beta)-Left(beta)-Right (L+R+L-R-)"""
        r, theta = self._polar(x, y)
        xi = x - math.sin(phi)
        yi = y - 1.0 + math.cos(phi)
        r1, theta1 = self._polar(xi, yi)
        if r1 < 4.0:
            return None
        acos_val = math.acos(min(1.0, 4.0 / r1))
        u = 2.0 * acos_val
        t = self._M(theta1 + acos_val + math.pi / 2.0 - phi)
        v = self._M(theta1 - acos_val + math.pi / 2.0)
        return (t, u, v)

    def LRLRp(self, x: float, y: float, phi: float) -> Optional[RSLength]:
        """Left-Right(beta)-Left(beta)-Right (L+R-L-R+)"""
        r, theta = self._polar(x, y)
        xi = x + math.sin(phi)
        yi = y - 1.0 - math.cos(phi)
        r1, theta1 = self._polar(xi, yi)
        if r1 < 4.0:
            return None
        acos_val = math.acos(min(1.0, 4.0 / r1))
        u = 2.0 * acos_val
        t = self._M(theta1 + acos_val + math.pi / 2.0)
        v = self._M(theta1 - acos_val - math.pi / 2.0 - phi)
        return (t, u, v)

    def LRSR(self, x: float, y: float, phi: float) -> Optional[RSLength]:
        """Left-Right(pi/2)-Straight-Right (L+R-S-R-)"""
        xi = x - math.sin(phi)
        yi = y - 1.0 + math.cos(phi)
        r1, theta1 = self._polar(xi, yi)
        if r1 < 2.0:
            return None
        acos_val = math.acos(min(1.0, 2.0 / r1))
        t = self._M(theta1 + acos_val + math.pi / 2.0 - phi)
        u = self._M(theta1 - acos_val + math.pi / 2.0)
        v = math.sqrt(r1 * r1 - 4.0)
        return (t, u, v)

    def LRSL(self, x: float, y: float, phi: float) -> Optional[RSLength]:
        """Left-Right(pi/2)-Straight-Left (L+R-S-L-)"""
        xi = x + math.sin(phi)
        yi = y - 1.0 - math.cos(phi)
        r1, theta1 = self._polar(xi, yi)
        if r1 < 2.0:
            return None
        acos_val = math.acos(min(1.0, 2.0 / r1))
        t = self._M(theta1 + acos_val + math.pi / 2.0)
        u = self._M(theta1 - acos_val - math.pi / 2.0 - phi)
        v = math.sqrt(r1 * r1 - 4.0)
        return (t, u, v)

    def LRSLR(self, x: float, y: float, phi: float) -> Optional[RSLength]:
        """Left-Right(pi/2)-Straight-Left(pi/2)-Right (L+R-S-L-R+)"""
        xi = x + math.sin(phi)
        yi = y - 1.0 - math.cos(phi)
        r1, theta1 = self._polar(xi, yi)
        if r1 < 4.0:
            return None
        u = math.sqrt(r1 * r1 - 4.0)
        acos_val = math.acos(min(1.0, 2.0 / r1))
        t = self._M(theta1 + acos_val + math.pi / 2.0 - phi)
        v = self._M(theta1 - acos_val + math.pi / 2.0)
        return (t, u, v)

    # ========== 6 种模式族 ==========

    def _path_SCS(self, x: float, y: float, phi: float) -> List[RSPath]:
        """#2 Straight-Circle-Straight"""
        paths = []
        # SLS with 4 variants (reflect, timeflip, backwards)
        for (dx, dy, dphi, lengths_mult, ctypes_mode) in [
            (x, y, phi, [1, 1, 1], [RS_STRAIGHT, RS_LEFT, RS_STRAIGHT]),
            (-x, y, -phi, [-1, 1, -1], [RS_STRAIGHT, RS_RIGHT, RS_STRAIGHT]),
            (x, -y, -phi, [1, -1, -1], [RS_STRAIGHT, RS_RIGHT, RS_STRAIGHT]),
            (-x, -y, phi, [-1, -1, 1], [RS_STRAIGHT, RS_LEFT, RS_STRAIGHT]),
        ]:
            result = self.SLS(dx, dy, dphi)
            if result is not None:
                lengths = [result[0] * lengths_mult[0],
                           result[1] * lengths_mult[1],
                           result[2] * lengths_mult[2]]
                paths.append(RSPath(lengths, ctypes_mode))
        return paths

    def _path_CCC(self, x: float, y: float, phi: float) -> List[RSPath]:
        """#8 Circle-Circle-Circle"""
        paths = []
        for (dx, dy, dphi, mult, ctypes) in [
            (x, y, phi, [1, 1, 1], [RS_LEFT, RS_RIGHT, RS_LEFT]),
            (-x, y, -phi, [-1, 1, -1], [RS_LEFT, RS_RIGHT, RS_LEFT]),
            (x, -y, -phi, [1, -1, -1], [RS_RIGHT, RS_LEFT, RS_RIGHT]),
            (-x, -y, phi, [-1, -1, 1], [RS_RIGHT, RS_LEFT, RS_RIGHT]),
        ]:
            result = self.LRL(dx, dy, dphi)
            if result is not None:
                lengths = [result[0] * mult[0], result[1] * mult[1], result[2] * mult[2]]
                paths.append(RSPath(lengths, ctypes))
        return paths

    def _path_CSC(self, x: float, y: float, phi: float) -> List[RSPath]:
        """#8 Circle-Straight-Circle"""
        paths = []
        for (dx, dy, dphi, mult, ctypes) in [
            (x, y, phi, [1, 1, 1], [RS_LEFT, RS_STRAIGHT, RS_LEFT]),
            (-x, y, -phi, [-1, 1, -1], [RS_LEFT, RS_STRAIGHT, RS_LEFT]),
            (x, -y, -phi, [1, -1, -1], [RS_RIGHT, RS_STRAIGHT, RS_RIGHT]),
            (-x, -y, phi, [-1, -1, 1], [RS_RIGHT, RS_STRAIGHT, RS_RIGHT]),
        ]:
            result = self.LSL(dx, dy, dphi)
            if result is not None:
                lengths = [result[0] * mult[0], result[1] * mult[1], result[2] * mult[2]]
                paths.append(RSPath(lengths, ctypes))

        for (dx, dy, dphi, mult, ctypes) in [
            (x, y, phi, [1, 1, 1], [RS_LEFT, RS_STRAIGHT, RS_RIGHT]),
            (-x, y, -phi, [-1, 1, -1], [RS_LEFT, RS_STRAIGHT, RS_RIGHT]),
            (x, -y, -phi, [1, -1, -1], [RS_RIGHT, RS_STRAIGHT, RS_LEFT]),
            (-x, -y, phi, [-1, -1, 1], [RS_RIGHT, RS_STRAIGHT, RS_LEFT]),
        ]:
            result = self.LSR(dx, dy, dphi)
            if result is not None:
                lengths = [result[0] * mult[0], result[1] * mult[1], result[2] * mult[2]]
                paths.append(RSPath(lengths, ctypes))
        return paths

    def _path_CCCC(self, x: float, y: float, phi: float) -> List[RSPath]:
        """#8 Circle-Circle(beta)-Circle(beta)-Circle"""
        paths = []
        for (dx, dy, dphi, mult, ctypes) in [
            (x, y, phi, [1, 1, 1, 1], [RS_LEFT, RS_RIGHT, RS_LEFT, RS_RIGHT]),
            (-x, y, -phi, [-1, 1, -1, 1], [RS_LEFT, RS_RIGHT, RS_LEFT, RS_RIGHT]),
            (x, -y, -phi, [1, -1, 1, -1], [RS_RIGHT, RS_LEFT, RS_RIGHT, RS_LEFT]),
            (-x, -y, phi, [-1, -1, -1, -1], [RS_RIGHT, RS_LEFT, RS_RIGHT, RS_LEFT]),
        ]:
            result_n = self.LRLRn(dx, dy, dphi)
            if result_n is not None:
                lengths = [result_n[0] * mult[0], result_n[1] * mult[1],
                           result_n[1] * mult[2], result_n[2] * mult[3]]
                paths.append(RSPath(lengths, ctypes))

        for (dx, dy, dphi, mult, ctypes) in [
            (x, y, phi, [1, 1, 1, 1], [RS_LEFT, RS_RIGHT, RS_LEFT, RS_RIGHT]),
            (-x, y, -phi, [-1, 1, -1, 1], [RS_LEFT, RS_RIGHT, RS_LEFT, RS_RIGHT]),
            (x, -y, -phi, [1, -1, 1, -1], [RS_RIGHT, RS_LEFT, RS_RIGHT, RS_LEFT]),
            (-x, -y, phi, [-1, -1, -1, -1], [RS_RIGHT, RS_LEFT, RS_RIGHT, RS_LEFT]),
        ]:
            result_p = self.LRLRp(dx, dy, dphi)
            if result_p is not None:
                lengths = [result_p[0] * mult[0], result_p[1] * mult[1],
                           result_p[1] * mult[2], result_p[2] * mult[3]]
                paths.append(RSPath(lengths, ctypes))
        return paths

    def _path_CCSC(self, x: float, y: float, phi: float) -> List[RSPath]:
        """#16 Circle-Circle(pi/2)-Straight-Circle and Circle-Straight-Circle(pi/2)-Circle"""
        paths = []
        for (dx, dy, dphi, mult, ctypes) in [
            (x, y, phi, [1, 1, 1, 1], [RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_RIGHT]),
            (-x, y, -phi, [-1, 1, 1, -1], [RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_RIGHT]),
            (x, -y, -phi, [1, -1, 1, -1], [RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_LEFT]),
            (-x, -y, phi, [-1, -1, 1, 1], [RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_LEFT]),
        ]:
            result = self.LRSR(dx, dy, dphi)
            if result is not None:
                lengths = [result[0] * mult[0], result[1] * mult[1],
                           result[2] * mult[2], result[0] * mult[3]]
                paths.append(RSPath(lengths, ctypes))

        for (dx, dy, dphi, mult, ctypes) in [
            (x, y, phi, [1, 1, 1, 1], [RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT]),
            (-x, y, -phi, [-1, 1, 1, -1], [RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT]),
            (x, -y, -phi, [1, -1, 1, -1], [RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT]),
            (-x, -y, phi, [-1, -1, 1, 1], [RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT]),
        ]:
            result = self.LRSL(dx, dy, dphi)
            if result is not None:
                lengths = [result[0] * mult[0], result[1] * mult[1],
                           result[2] * mult[2], result[0] * mult[3]]
                paths.append(RSPath(lengths, ctypes))
        return paths

    def _path_CCSCC(self, x: float, y: float, phi: float) -> List[RSPath]:
        """#4 Circle-Circle(pi/2)-Straight-Circle(pi/2)-Circle"""
        paths = []
        for (dx, dy, dphi, mult, ctypes) in [
            (x, y, phi, [1, 1, 1, 1, 1], [RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_RIGHT]),
            (-x, y, -phi, [-1, 1, 1, 1, -1], [RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_RIGHT]),
            (x, -y, -phi, [1, -1, 1, -1, 1], [RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT]),
            (-x, -y, phi, [-1, -1, 1, -1, -1], [RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT]),
        ]:
            result = self.LRSLR(dx, dy, dphi)
            if result is not None:
                lengths = [result[0] * mult[0], result[1] * mult[1],
                           result[2] * mult[2], result[1] * mult[3], result[0] * mult[4]]
                paths.append(RSPath(lengths, ctypes))
        return paths

    # ========== 主函数 ==========

    def generation(self, start: Point3d, goal: Point3d) -> Points3d:
        """
        生成 Reeds-Shepp 曲线（选择最短路径）
        对应 C++ generation(const Point3d&, const Point3d&, Points3d&)
        """
        sx, sy, syaw = start
        gx, gy, gyaw = goal

        # 转换到相对坐标系
        dx = (gx - sx) * self.max_curv
        dy = (gy - sy) * self.max_curv
        dphi = self._M(gyaw - syaw)

        # 收集所有可能的路径
        all_paths = []
        all_paths.extend(self._path_CSC(dx, dy, dphi))
        all_paths.extend(self._path_CCC(dx, dy, dphi))
        all_paths.extend(self._path_CCCC(dx, dy, dphi))
        all_paths.extend(self._path_CCSC(dx, dy, dphi))
        all_paths.extend(self._path_CCSCC(dx, dy, dphi))
        all_paths.extend(self._path_SCS(dx, dy, dphi))

        # 选择最短有效路径
        best_path = None
        best_len = float('inf')
        for path in all_paths:
            if path.valid():
                l = path.len()
                if l < best_len:
                    best_len = l
                    best_path = path

        if best_path is None or best_path.size() == 0:
            return []

        # 插值生成路径点
        world_path = []
        current = (0.0, 0.0, 0.0)
        world_path.append((sx, sy, syaw))

        for i in range(best_path.size()):
            seg_len, seg_type = best_path.get(i)
            if abs(seg_len) < 1e-10:
                continue
            remaining = seg_len
            while abs(remaining) > 1e-6:
                step_len = min(abs(remaining), self.step * self.max_curv)
                step_len = step_len if seg_len > 0 else -step_len
                current = self._interpolate(seg_type, step_len, current)
                # 转换到世界坐标
                wx = current[0] / self.max_curv + sx
                wy = current[1] / self.max_curv + sy
                wyaw = self._M(current[2] + syaw)
                world_path.append((wx, wy, wyaw))
                remaining -= step_len

        return world_path

    def _interpolate(self, mode: int, length: float,
                     init_pose: Point3d) -> Point3d:
        """
        单步插值
        类似 DubinsCurve::interpolate
        """
        x, y, yaw = init_pose
        if mode == RS_STRAIGHT:
            x += length * math.cos(yaw)
            y += length * math.sin(yaw)
        elif mode == RS_LEFT:
            x += math.sin(yaw + length) - math.sin(yaw)
            y += -math.cos(yaw + length) + math.cos(yaw)
            yaw += length
        elif mode == RS_RIGHT:
            x += -math.sin(yaw - length) + math.sin(yaw)
            y += math.cos(yaw - length) - math.cos(yaw)
            yaw -= length
        return (x, y, yaw)

    def run(self, points: Points2d) -> Points3d:
        """通过多个路径点运行（简化：取首尾）"""
        if len(points) < 2:
            return []
        start = (points[0][0], points[0][1], 0.0)
        goal = (points[-1][0], points[-1][1], 0.0)
        return self.generation(start, goal)

    def set_max_curv(self, max_curv: float):
        self.max_curv = max_curv
