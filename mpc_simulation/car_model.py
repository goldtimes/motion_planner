"""
自行车运动学模型 (Kinematic Bicycle Model)

状态: [x, y, theta]
控制: [v, delta]  — 线速度 / 前轮转角
"""

import numpy as np


class CarModel:
    """后轮参考点的自行车运动学模型"""

    def __init__(self, wheelbase: float = 2.5):
        """
        Args:
            wheelbase: 轴距 L (m)
        """
        self.L = wheelbase

    def dynamics(self, state: np.ndarray, control: np.ndarray, dt: float) -> np.ndarray:
        """
        连续时间导数: dx/dt = f(x, u)

        Args:
            state:  [x, y, theta]
            control: [v, delta]
            dt: 时间步长

        Returns:
            下一时刻状态 [x, y, theta]
        """
        x, y, theta = state
        v, delta = control

        dx = v * np.cos(theta)
        dy = v * np.sin(theta)
        dtheta = v * np.tan(delta) / self.L

        return state + np.array([dx, dy, dtheta]) * dt

    def linearize(self, state: np.ndarray, control: np.ndarray, dt: float):
        """
        在当前状态和控制处线性化，得到离散时间状态空间矩阵：
            x_{k+1} ≈ A x_k + B u_k + C

        Returns:
            A (3x3), B (3x2), C (3,)
        """
        x, y, theta = state
        v, delta = control

        # 连续时间 Jacobian
        A_cont = np.array([
            [0.0, 0.0, -v * np.sin(theta)],
            [0.0, 0.0,  v * np.cos(theta)],
            [0.0, 0.0,  0.0],
        ])

        B_cont = np.array([
            [np.cos(theta), 0.0],
            [np.sin(theta), 0.0],
            [np.tan(delta) / self.L, v / (self.L * np.cos(delta) ** 2)],
        ])

        # 离散化: A_d ≈ I + A_cont * dt, B_d ≈ B_cont * dt
        A = np.eye(3) + A_cont * dt
        B = B_cont * dt

        # 仿射项: C = (f(x,u) - A_cont@x - B_cont@u) * dt
        f0 = np.array([v * np.cos(theta), v * np.sin(theta), v * np.tan(delta) / self.L])
        C = (f0 - A_cont @ state - B_cont @ control) * dt

        return A, B, C

    def predict_trajectory(self, state: np.ndarray,
                           controls: np.ndarray, dt: float) -> np.ndarray:
        """
        用非线性模型预测一条轨迹

        Args:
            state: 初始状态 [x, y, theta]
            controls: (N, 2) 控制序列 [v, delta]
            dt: 时间步长

        Returns:
            trajectory: (N+1, 3) 状态轨迹
        """
        N = controls.shape[0]
        traj = np.zeros((N + 1, 3))
        traj[0] = state
        for k in range(N):
            traj[k + 1] = self.dynamics(traj[k], controls[k], dt)
        return traj
