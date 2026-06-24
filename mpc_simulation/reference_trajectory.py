"""
参考轨迹生成器 — 支持多种轨迹类型
"""

import numpy as np


class ReferenceGenerator:
    """生成参考轨迹"""

    def __init__(self, ref_type: str = "eight"):
        self.ref_type = ref_type

    def get_reference(self, t: float) -> np.ndarray:
        """返回 (x, y, theta) 单个参考点"""
        if self.ref_type == "eight":
            return self._eight(t)
        elif self.ref_type == "circle":
            return self._circle(t)
        elif self.ref_type == "lane_change":
            return self._lane_change(t)
        else:
            return self._eight(t)

    def get_reference_trajectory(self, t: float, N: int, dt: float) -> np.ndarray:
        """返回未来 N+1 步参考轨迹"""
        traj = np.zeros((N + 1, 3))
        for k in range(N + 1):
            traj[k] = self.get_reference(t + k * dt)
        return traj

    def _eight(self, t: float) -> np.ndarray:
        """8 字形轨迹 (利萨如曲线)"""
        a, b = 4.0, 3.0
        omega = 0.4
        x = a * np.sin(omega * t)
        y = b * np.sin(2 * omega * t)
        dx = a * omega * np.cos(omega * t)
        dy = 2 * b * omega * np.cos(2 * omega * t)
        theta = np.arctan2(dy, dx)
        return np.array([x, y, theta])

    def _circle(self, t: float) -> np.ndarray:
        """圆形轨迹"""
        R = 5.0
        omega = 0.5
        x = R * np.cos(omega * t)
        y = R * np.sin(omega * t)
        dx = -R * omega * np.sin(omega * t)
        dy = R * omega * np.cos(omega * t)
        theta = np.arctan2(dy, dx)
        return np.array([x, y, theta])

    def _lane_change(self, t: float) -> np.ndarray:
        """变道轨迹"""
        v_des = 2.0
        x = v_des * t
        y = 3.0 * np.tanh(0.3 * (t - 5.0))
        dy = 0.9 * (1 - np.tanh(0.3 * (t - 5.0)) ** 2)
        theta = np.arctan2(dy, v_des)
        return np.array([x, y, theta])
