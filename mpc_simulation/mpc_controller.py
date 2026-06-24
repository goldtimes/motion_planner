"""
MPC 控制器 — 非线性模型预测控制

使用 scipy.optimize.minimize (SLSQP) 求解有限时域最优控制问题。
"""

import numpy as np
from scipy.optimize import minimize, Bounds
from car_model import CarModel


class MPCController:
    """小车 MPC 轨迹跟踪控制器"""

    def __init__(
        self,
        wheelbase: float = 2.5,
        horizon: int = 15,
        dt: float = 0.1,
        v_min: float = -1.0,
        v_max: float = 3.0,
        delta_min: float = -0.6,   # rad (~35°)
        delta_max: float = 0.6,
        Q: np.ndarray = None,       # 状态代价权重 (3x3)
        R: np.ndarray = None,       # 控制代价权重 (2x2)
        Qf: np.ndarray = None,      # 终端代价权重 (3x3)
    ):
        self.model = CarModel(wheelbase)
        self.N = horizon
        self.dt = dt

        self.v_min, self.v_max = v_min, v_max
        self.delta_min, self.delta_max = delta_min, delta_max

        # 默认权重
        self.Q = Q if Q is not None else np.diag([5.0, 5.0, 2.0])
        self.R = R if R is not None else np.diag([0.5, 1.0])
        self.Qf = Qf if Qf is not None else np.diag([10.0, 10.0, 5.0])

        # 上次最优解的缓存（warm start）
        self._warm_start = None

    def set_reference(self, ref_trajectory: np.ndarray):
        """
        设置参考轨迹

        Args:
            ref_trajectory: (N+1, 3) 参考状态序列 [x_ref, y_ref, theta_ref]
        """
        self.ref = ref_trajectory

    def _build_bounds(self) -> Bounds:
        """构建控制变量的上下界"""
        n_vars = self.N * 2
        lb = np.zeros(n_vars)
        ub = np.zeros(n_vars)
        for k in range(self.N):
            lb[2 * k] = self.v_min
            ub[2 * k] = self.v_max
            lb[2 * k + 1] = self.delta_min
            ub[2 * k + 1] = self.delta_max
        return Bounds(lb, ub)

    def _cost_function(self, u_flat: np.ndarray, x0: np.ndarray) -> float:
        """
        计算累积代价

        Args:
            u_flat: (N*2,) 展平控制序列 [v0, delta0, v1, delta1, ...]
            x0: 当前状态 [x, y, theta]

        Returns:
            total_cost: 标量代价
        """
        u = u_flat.reshape(self.N, 2)
        x = x0.copy()
        cost = 0.0

        for k in range(self.N):
            x = self.model.dynamics(x, u[k], self.dt)
            # 跟踪误差代价
            err = x - self.ref[k + 1]
            cost += err @ self.Q @ err
            # 控制代价
            cost += u[k] @ self.R @ u[k]

            # 速度惩罚（禁止大幅倒车，按需调节）
            if u[k][0] < 0:
                cost += 5.0 * u[k][0] ** 2

        # 终端代价
        err_final = x - self.ref[-1]
        cost += err_final @ self.Qf @ err_final

        return cost

    def solve(self, x0: np.ndarray) -> np.ndarray:
        """
        求解 MPC 优化问题，返回最优控制序列

        Args:
            x0: 当前状态 [x, y, theta]

        Returns:
            u_opt: (N, 2) 最优控制序列
        """
        n_vars = self.N * 2

        # warm start
        if self._warm_start is not None and len(self._warm_start) == n_vars:
            u0 = self._warm_start
        else:
            # 初始猜测：匀速直行
            u0 = np.zeros(n_vars)
            u0[0::2] = 1.0    # v = 1 m/s
            u0[1::2] = 0.0    # delta = 0

        bounds = self._build_bounds()

        result = minimize(
            fun=lambda u: self._cost_function(u, x0),
            x0=u0,
            method='SLSQP',
            bounds=bounds,
            options={
                'maxiter': 200,
                'ftol': 1e-6,
                'disp': False,
            },
        )

        u_opt = result.x.reshape(self.N, 2)
        self._warm_start = u_opt.flatten()  # 缓存供下一帧使用

        return u_opt

    def get_first_control(self, x0: np.ndarray) -> np.ndarray:
        """
        只返回第一帧控制量（MPC 滚动优化的输出）

        Returns:
            control: [v, delta]
        """
        u_opt = self.solve(x0)
        return u_opt[0]
