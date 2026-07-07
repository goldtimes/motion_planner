"""
曲线基类抽象

与 C++ 中 rmp::common::geometry::Curve 对应。
"""

from abc import ABC, abstractmethod
from typing import List, Tuple

# 类型别名
Point2d = Tuple[float, float]
Point3d = Tuple[float, float, float]
Points2d = List[Point2d]
Points3d = List[Point3d]


class Curve(ABC):
    """曲线基类，对应 C++ Curve"""

    def __init__(self, step: float = 0.1):
        self.step = step

    @abstractmethod
    def run(self, points: Points2d) -> Points3d:
        """
        运行轨迹生成 (对应 C++ run(const Points2d&, Points3d&))
        @param points: 路径点 <x, y>
        @return path: 生成轨迹 <x, y, theta>
        """
        pass

    @abstractmethod
    def generation(self, start: Point3d, goal: Point3d) -> Points3d:
        """
        从起始位姿到目标位姿生成路径 (对应 C++ generation)
        @param start: 初始位姿 (x, y, yaw)
        @param goal:  目标位姿 (x, y, yaw)
        @return path: 平滑轨迹点
        """
        pass

    def distance(self, path: Points3d) -> float:
        """计算路径总长度"""
        if len(path) < 2:
            return 0.0
        total = 0.0
        for i in range(1, len(path)):
            dx = path[i][0] - path[i - 1][0]
            dy = path[i][1] - path[i - 1][1]
            total += (dx * dx + dy * dy) ** 0.5
        return total

    def set_step(self, step: float):
        self.step = step
