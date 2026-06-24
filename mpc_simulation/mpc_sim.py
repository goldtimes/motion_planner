"""
MPC 小车仿真 —— 主程序

功能：
  - 自行车模型小车
  - MPC 跟踪一条 8 字形 / 圆形 / 变道参考轨迹
  - 实时 matplotlib 可视化

用法：
  python mpc_sim.py
"""

import numpy as np

from car_model import CarModel
from mpc_controller import MPCController
from reference_trajectory import ReferenceGenerator


# ═══════════════════════════════════════════════════════════════════
#  仿真参数（在此修改）
# ═══════════════════════════════════════════════════════════════════
DT = 0.1               # 仿真步长 (s)
SIM_TIME = 30.0        # 总仿真时间 (s)
WHEELBASE = 2.5        # 轴距 (m)

# 参考轨迹类型: "eight" | "circle" | "lane_change"
REF_TYPE = "eight"

# 初始状态
X0 = np.array([0.0, 2.0, 0.0])   # 小车初始 [x, y, theta]

# 控制约束
V_MIN, V_MAX = -0.5, 3.0
DELTA_MIN, DELTA_MAX = -0.6, 0.6   # ±35°

# MPC 参数
HORIZON = 15
Q = np.diag([10.0, 10.0, 2.0])    # 状态跟踪权重
R = np.diag([0.1, 0.5])            # 控制平滑权重
Qf = np.diag([20.0, 20.0, 5.0])   # 终端代价权重


# ═══════════════════════════════════════════════════════════════════
#  主仿真
# ═══════════════════════════════════════════════════════════════════
def run_simulation():
    """运行仿真，返回 (frame_data_list, full_ref) 供可视化使用"""
    print("=" * 60)
    print("  MPC Car Simulation")
    print(f"  Reference: {REF_TYPE}")
    print(f"  Horizon: {HORIZON}, dt: {DT}s, Sim time: {SIM_TIME}s")
    print("=" * 60)

    # 初始化
    np.random.seed(42)
    ref_gen = ReferenceGenerator(REF_TYPE)
    controller = MPCController(
        wheelbase=WHEELBASE,
        horizon=HORIZON,
        dt=DT,
        v_min=V_MIN, v_max=V_MAX,
        delta_min=DELTA_MIN, delta_max=DELTA_MAX,
        Q=Q, R=R, Qf=Qf,
    )
    model = CarModel(WHEELBASE)

    # 生成完整参考轨迹（用于可视化）
    t_span = np.arange(0, SIM_TIME, DT)
    full_ref = np.array([ref_gen.get_reference(t) for t in t_span])

    # 仿真状态
    state = X0.copy()
    t = 0.0
    frame_data_list = []

    # 主循环
    step = 0
    max_steps = int(SIM_TIME / DT)

    while step < max_steps:
        # 1. 获取当前参考轨迹
        ref_traj = ref_gen.get_reference_trajectory(t, HORIZON, DT)
        controller.set_reference(ref_traj)

        # 2. MPC 求解
        u_opt = controller.solve(state)
        control = u_opt[0]  # 只取第一帧

        # 3. 参考误差
        err = state - ref_traj[0]
        ref_error = np.linalg.norm(err[:2])

        # 4. 前向仿真一步
        state = model.dynamics(state, control, DT)

        # 5. 预测轨迹
        pred_traj = model.predict_trajectory(state, u_opt, DT)

        # 6. 记录
        frame_data_list.append({
            "state": state.copy(),
            "control": control.copy(),
            "pred_traj": pred_traj,
            "time": t,
            "cost": controller._cost_function(u_opt.flatten(), state),
            "ref_error": ref_error,
        })

        t += DT
        step += 1

        if step % 50 == 0:
            print(f"  Step {step}/{max_steps}, t={t:.1f}s, "
                  f"v={control[0]:.2f}, δ={np.degrees(control[1]):.1f}°")

    print(f"\nSimulation complete! {len(frame_data_list)} frames generated.")
    return frame_data_list, full_ref


# ═══════════════════════════════════════════════════════════════════
#  可视化（仅在 __main__ 中导入 matplotlib）
# ═══════════════════════════════════════════════════════════════════
def visualize(frame_data_list, full_ref):
    """matplotlib 动画可视化"""
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    from matplotlib.patches import Rectangle

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_xlim(-8, 8)
    ax.set_ylim(-6, 6)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title(f"MPC Car Simulation — {REF_TYPE.upper()}")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    # 参考轨迹
    ax.plot(full_ref[:, 0], full_ref[:, 1], 'k--', lw=1.0, alpha=0.4, label="Reference")

    # 动态元素
    traj_line, = ax.plot([], [], 'b-', lw=1.5, label="Actual")
    pred_line, = ax.plot([], [], 'orange', lw=1.0, alpha=0.6, label="Predicted")
    car_patch = Rectangle((0, 0), 1.2, 0.7, angle=0,
                          fc='dodgerblue', ec='navy', lw=1.5, zorder=5)
    ax.add_patch(car_patch)

    info_text = ax.text(
        0.02, 0.98, "", transform=ax.transAxes,
        verticalalignment="top", fontfamily="monospace", fontsize=9,
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.8)
    )

    ax.legend(loc="upper right")

    x_hist, y_hist = [], []

    def update(frame):
        state = frame["state"]
        x, y, theta = state
        pred_traj = frame.get("pred_traj", np.zeros((0, 3)))
        t = frame["time"]
        v = frame["control"][0]
        delta = frame["control"][1]
        cost = frame.get("cost", 0.0)
        ref_error = frame.get("ref_error", 0.0)

        x_hist.append(x)
        y_hist.append(y)
        traj_line.set_data(x_hist, y_hist)

        car_patch.set_xy((x - 0.6, y - 0.35))
        car_patch.angle = np.degrees(theta)

        if pred_traj.shape[0] > 0:
            pred_line.set_data(pred_traj[:, 0], pred_traj[:, 1])

        info_text.set_text(
            f"Time: {t:.1f}s\nv: {v:.2f} m/s\n"
            f"δ: {np.degrees(delta):.1f}°\ncost: {cost:.2f}\nerr: {ref_error:.3f} m"
        )
        return [traj_line, car_patch, pred_line, info_text]

    ani = animation.FuncAnimation(
        fig, update, frames=frame_data_list,
        interval=DT * 1000, blit=False, repeat=True
    )

    plt.tight_layout()
    plt.show()
    return ani


# ═══════════════════════════════════════════════════════════════════
#  入口
# ═══════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    frames, ref = run_simulation()
    visualize(frames, ref)
