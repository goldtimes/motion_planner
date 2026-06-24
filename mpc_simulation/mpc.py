import casadi as ca  # 导入 CasADi 用于符号计算和自动微分，是构建NMPC的核心库
import numpy as np   # 导入 NumPy 用于数值计算，特别是数组操作
import matplotlib.pyplot as plt  # 导入 Matplotlib.pyplot 用于绘图
import time  # 导入 time 模块，用于计时等操作
from matplotlib.patches import Rectangle  # 从 Rectangle 用于绘制车辆和障碍物形状
from matplotlib.transforms import Affine2D  # Affine2D 用于车辆的旋转和平移变换
# FuncAnimation 用于创建动画
from matplotlib.animation import FuncAnimation

# --- 中文字体设置 ---
# 尝试设置 Matplotlib 的字体以支持中文显示
try:
    # 设置 sans-serif 字体为 'SimHei' (黑体)，这是一种常用的中文字体
    plt.rcParams['font.sans-serif'] = ['SimHei']
    # 设置坐标轴负号显示为 False，以便正确显示负号 (在某些中文字体配置下，负号可能显示为方框)
    plt.rcParams['axes.unicode_minus'] = False
except Exception as e:
    # 如果字体设置失败，打印警告信息
    print(f"中文字体设置警告: {e}。如果中文显示不正确，请确保已安装如 SimHei 等中文字体，并正确配置 Matplotlib。")

# --- NMPC (非线性模型预测控制) 参数 ---
Ts = 0.1  # 采样时间 (s)：控制器更新和系统离散化的时间间隔
Np = 20   # 预测时域：控制器向前预测的步数
L = 2.5   # 车辆轴距 (m)：前轮和后轮轴之间的距离，用于运动学模型
v_max = 20.0  # 最大速度 (m/s)
v_min = -2.0  # 最小速度 (m/s) (允许倒车)
a_max = 3.0   # 最大加速度 (m/s^2)
a_min = -5.0  # 最大减速度 (m/s^2) (即负向加速度的最小值)
delta_max = np.deg2rad(30)  # 最大转向角 (rad)：将角度从度转换为弧度
delta_min = -delta_max  # 最小转向角 (rad)：通常与最大转向角对称

# 车辆绘图尺寸
vehicle_plot_length = 4.0  # 绘图用车辆长度 (m)
vehicle_plot_width = 1.8   # 绘图用车辆宽度 (m)

nx = 4  # 状态量数量：[x (全局坐标系X位置), y (全局坐标系Y位置), psi (车辆航向角), v (车辆速度)]
nu = 2  # 控制量数量：[a (加速度), delta (前轮转向角)]

# --- 权重矩阵调整 ---
# 状态权重矩阵 Q: 对应状态 [x, y, psi, v] 的误差惩罚
# Q矩阵参数说明：
# Q[0,0] = 1.0: x方向位置权重
#   - 增大：车辆会更严格地跟踪x方向的参考轨迹
#   - 减小：车辆在x方向有更大的自由度，可以更灵活地调整
# Q[1,1] = 100.0: y方向位置权重
#   - 增大：车辆会更严格地保持在车道中心，减少横向偏移
#   - 减小：车辆在y方向有更大的自由度，可以更灵活地变道
# Q[2,2] = 0.5: 航向角权重
#   - 增大：车辆会更严格地保持期望航向角，转向更平滑
#   - 减小：车辆可以更灵活地改变航向，但可能导致转向不够平滑
# Q[3,3] = 1.0: 速度权重
#   - 增大：车辆会更严格地跟踪目标速度
#   - 减小：车辆在速度控制上更灵活，但可能导致速度波动
Q = np.diag([1.0, 100.0, 0.5, 1.0])

# 控制权重矩阵 R: 对应控制量 [a, delta] 的变化惩罚
# R矩阵参数说明：
# R[0,0] = 0.1: 加速度权重
#   - 增大：加速度变化更平滑，但响应可能变慢
#   - 减小：加速度变化更灵活，但可能导致控制不够平滑
# R[1,1] = 1000.0: 转向角权重
#   - 增大：转向更平滑，但可能导致转向响应不够及时
#   - 减小：转向更灵活，但可能导致转向不够平滑
R = np.diag([0.1, 1000.0])

# 终端权重因子：用于增强终端状态的跟踪精度
# 增大：更注重终端状态的准确性，但可能导致中间过程不够平滑
# 减小：更注重整个过程的平滑性，但终端状态可能不够准确
P_factor = 0.01
P = Q * P_factor

# 航向角速度变化惩罚权重
# 增大：转向更平滑，但可能导致转向响应不够及时
# 减小：转向更灵活，但可能导致转向不够平滑
w_psi_dot_change = 18000.0

# --- 道路参数 ---
lane_width = 3.5  # 车道宽度 (m)
lane1_y = 0.0     # 第一车道中心线 y 坐标 (m)
lane2_y = lane_width  # 第二车道中心线 y 坐标 (m)

# --- 道路边界参数 ---
road_y_min_abs = lane1_y - lane_width / 2  # 道路绝对下边界
road_y_max_abs = lane2_y + lane_width / 2  # 道路绝对上边界
# 车辆中心Y坐标的约束，考虑车辆宽度
vehicle_y_constraint_lower = road_y_min_abs + vehicle_plot_width / 2
vehicle_y_constraint_upper = road_y_max_abs - vehicle_plot_width / 2

# --- 障碍物参数 ---
obs_x_initial = 35.0  # 障碍物初始 x 坐标 (m) (中心点)
obs_y_fixed = lane1_y # 障碍物 y 坐标 (m) (中心点, 假设障碍物在第一车道中心)
obs_speed_x = 4.0     # 障碍物向右移动的速度 (m/s)

obs_length = vehicle_plot_length  # 障碍物长度 (m)
obs_width = vehicle_plot_width    # 障碍物宽度 (m)
obs_color = 'orange'              # 障碍物颜色

# 碰撞检测参数
veh_r = vehicle_plot_length / 2.5 # 车辆等效碰撞半径 (m)
obs_effective_collision_radius = obs_length / 2.0 # 障碍物等效碰撞半径 (m)
# 安全距离参数说明：
# 增大：车辆会与障碍物保持更大的距离，更安全但可能导致变道不够及时
# 减小：车辆可以更接近障碍物，变道更及时但安全性降低
safe_dist = 0.05

target_v_val = 10.0 # 目标巡航速度 (m/s)

# --- CasADi 符号变量与模型 ---
# 定义状态变量
x_sym = ca.SX.sym('x_s')  # x位置
y_sym = ca.SX.sym('y_s')  # y位置
psi_sym = ca.SX.sym('psi_s')  # 航向角
v_sym = ca.SX.sym('v_s')  # 速度
states_sym = ca.vertcat(x_sym, y_sym, psi_sym, v_sym)

# 定义控制变量
a_sym = ca.SX.sym('a_s')  # 加速度
delta_sym = ca.SX.sym('delta_s')  # 转向角
controls_sym = ca.vertcat(a_sym, delta_sym)

# 定义车辆运动学模型
rhs = ca.vertcat(v_sym * ca.cos(psi_sym),  # x方向速度
                 v_sym * ca.sin(psi_sym),  # y方向速度
                 (v_sym / L) * ca.tan(delta_sym),  # 航向角变化率
                 a_sym)  # 加速度
f_discrete = ca.Function('f_discrete', [states_sym, controls_sym], [states_sym + Ts * rhs])

# --- NMPC 问题构建 ---
opti = ca.Opti()
X_dv = opti.variable(nx, Np + 1)  # 状态变量
U_dv = opti.variable(nu, Np)      # 控制变量
P_param = opti.parameter(nx + 5)  # 参数：[x_curr (nx), target_y, target_v, obs_x_center, obs_y_center, prev_psi_dot]

# 构建目标函数
obj = 0
for k in range(Np):
    # 目标x设为当前预测的x，不强制x方向的跟踪，让车辆自由前进
    x_ref_k = ca.vertcat(X_dv[0, k], P_param[nx], 0, P_param[nx+1]) # 目标航向角为0
    obj += ca.mtimes([(X_dv[:, k] - x_ref_k).T, Q, (X_dv[:, k] - x_ref_k)])  # 状态误差惩罚
    obj += ca.mtimes([U_dv[:, k].T, R, U_dv[:, k]])  # 控制量惩罚

# 终端状态惩罚
terminal_state_ref = ca.vertcat(X_dv[0, Np], P_param[nx], 0, P_param[nx+1])
terminal_state_error = X_dv[:, Np] - terminal_state_ref
obj += ca.mtimes([terminal_state_error.T, P, terminal_state_error])

# 航向角速度变化惩罚
psi_dot_preds_list = []
for k_pd in range(Np):
    current_psi_dot_expr = (X_dv[3, k_pd] / L) * ca.tan(U_dv[1, k_pd])
    psi_dot_preds_list.append(current_psi_dot_expr)

if Np > 0:
    obj += w_psi_dot_change * (psi_dot_preds_list[0] - P_param[nx+4])**2
for k_cost in range(1, Np):
    obj += w_psi_dot_change * (psi_dot_preds_list[k_cost] - psi_dot_preds_list[k_cost-1])**2

opti.minimize(obj)

# 添加约束条件
opti.subject_to(X_dv[:, 0] == P_param[:nx])  # 初始状态约束
for k in range(Np):
    opti.subject_to(X_dv[:, k+1] == f_discrete(X_dv[:, k], U_dv[:, k]))  # 系统动力学约束

# 控制量约束
opti.subject_to(opti.bounded(a_min, U_dv[0, :], a_max))  # 加速度约束
opti.subject_to(opti.bounded(delta_min, U_dv[1, :], delta_max))  # 转向角约束
opti.subject_to(opti.bounded(v_min, X_dv[3, :], v_max))  # 速度约束

# 碰撞约束与道路边界约束
for k in range(1, Np + 1): # 从 k=1 开始，因为 k=0 是当前状态
    # 车辆中心预测位置: X_dv[0, k], X_dv[1, k]
    
    # 1. 碰撞约束
    dist_sq_obs = (X_dv[0, k] - P_param[nx+2])**2 + \
                  (X_dv[1, k] - P_param[nx+3])**2
    min_dist_sq_allowed = (veh_r + obs_effective_collision_radius + safe_dist)**2
    opti.subject_to(dist_sq_obs >= min_dist_sq_allowed)

    # 2. 道路边界约束
    opti.subject_to(X_dv[1, k] >= vehicle_y_constraint_lower)
    opti.subject_to(X_dv[1, k] <= vehicle_y_constraint_upper)

# 求解器设置
opts = {
    'ipopt.print_level': 0, 'print_time': 0, 'ipopt.max_iter': 6000,
    'ipopt.tol': 1e-5, 'ipopt.acceptable_tol': 1e-4,
    'ipopt.mu_init': 1e-2, 'ipopt.constr_viol_tol': 1e-4
}
opti.solver('ipopt', opts)

# --- 仿真循环 ---
sim_time = 12.0 # 可以适当延长仿真时间观察返回车道行为
t_history = np.arange(0, sim_time, Ts)
x_history = np.zeros((nx, len(t_history)))
u_history = np.zeros((nu, len(t_history)-1))
target_y_history = np.zeros(len(t_history))

obs_x_center_history = np.zeros(len(t_history))
obs_y_center_history = np.zeros(len(t_history))

# 初始状态设置
x_current = np.array([0.0, lane1_y, 0.0, target_v_val * 0.8])
x_history[:, 0] = x_current
target_y_history[0] = lane1_y

obs_x_center_history[0] = obs_x_initial
obs_y_center_history[0] = obs_y_fixed

psi_dot_at_last_interval = 0.0
U_guess_current = np.zeros((nu, Np))
X_guess_current = np.tile(x_current, (Np + 1, 1)).T

# 初始控制猜测值设置
num_accel_steps = Np // 2
if x_current[3] < target_v_val and num_accel_steps > 0:
    desired_avg_accel = (target_v_val - x_current[3]) / (num_accel_steps * Ts)
    applied_accel = np.clip(desired_avg_accel, a_min, a_max)
    U_guess_current[0, :num_accel_steps] = applied_accel
elif x_current[3] > target_v_val and num_accel_steps > 0:
    desired_avg_decel = (target_v_val - x_current[3]) / (num_accel_steps * Ts)
    applied_decel = np.clip(desired_avg_decel, a_min, a_max)
    U_guess_current[0, :num_accel_steps] = applied_decel

# 初始状态预测
temp_x_guess = x_current.copy()
X_guess_current[:, 0] = temp_x_guess
for k_rollout in range(Np):
    u_rollout = U_guess_current[:, k_rollout]
    temp_x_next_full = f_discrete(temp_x_guess, u_rollout)
    temp_x_guess = temp_x_next_full.full().flatten()
    X_guess_current[:, k_rollout+1] = temp_x_guess

# 状态机参数设置
maneuver_state = "DRIVING_LANE_1"
current_target_y = lane1_y
evade_trigger_distance = 25.0  # 触发变道的距离阈值
pass_obstacle_distance = obs_length / 2 + vehicle_plot_length / 2 + 3.0  # 通过障碍物的距离阈值
return_trigger_x_offset = vehicle_plot_length / 2 + 6.0  # 触发返回车道的距离阈值
stabilize_distance_after_return = 15.0  # 返回车道后的稳定距离

print("开始仿真...")
for i in range(len(t_history) - 1):
    vehicle_x_position = x_current[0]
    current_sim_time_step = t_history[i]

    # 更新障碍物位置
    current_obs_x_center = obs_x_initial + obs_speed_x * current_sim_time_step
    current_obs_y_center = obs_y_fixed

    obs_x_center_history[i] = current_obs_x_center
    obs_y_center_history[i] = current_obs_y_center

    obstacle_front_x = current_obs_x_center - obs_length / 2
    vehicle_front_x_ref = vehicle_x_position

    # 状态机逻辑
    if maneuver_state == "DRIVING_LANE_1":
        current_target_y = lane1_y
        if vehicle_front_x_ref > obstacle_front_x - evade_trigger_distance and \
           vehicle_front_x_ref < current_obs_x_center + obs_length :
            maneuver_state = "CHANGING_TO_LANE_2"
            print(f"t={t_history[i]:.1f}s: State -> CHANGING_TO_LANE_2 (veh_x={vehicle_x_position:.1f}m, obs_front_x={obstacle_front_x:.1f}m)")
    elif maneuver_state == "CHANGING_TO_LANE_2":
        current_target_y = lane2_y
        if vehicle_front_x_ref > current_obs_x_center + pass_obstacle_distance:
            maneuver_state = "DRIVING_LANE_2"
            print(f"t={t_history[i]:.1f}s: State -> DRIVING_LANE_2 (veh_x={vehicle_x_position:.1f}m, obs_center_x={current_obs_x_center:.1f}m)")
    elif maneuver_state == "DRIVING_LANE_2":
        current_target_y = lane2_y
        if vehicle_front_x_ref > current_obs_x_center + return_trigger_x_offset:
            maneuver_state = "RETURNING_TO_LANE_1"
            print(f"t={t_history[i]:.1f}s: State -> RETURNING_TO_LANE_1 (veh_x={vehicle_x_position:.1f}m, obs_center_x={current_obs_x_center:.1f}m)")
    elif maneuver_state == "RETURNING_TO_LANE_1":
        current_target_y = lane1_y
        if vehicle_front_x_ref > current_obs_x_center + return_trigger_x_offset + stabilize_distance_after_return and \
           abs(x_current[1] - lane1_y) < 0.2 and \
           abs(x_current[3] - target_v_val) < 1.5 :
            maneuver_state = "COMPLETED_MANEUVER"
            print(f"t={t_history[i]:.1f}s: State -> COMPLETED_MANEUVER (veh_x={vehicle_x_position:.1f}m, y={x_current[1]:.2f}m, obs_center_x={current_obs_x_center:.1f}m)")
    elif maneuver_state == "COMPLETED_MANEUVER":
        current_target_y = lane1_y

    target_y_history[i] = current_target_y
    if i == len(t_history) - 2:
        target_y_history[i+1] = current_target_y

    # 设置NMPC求解器参数
    current_obstacle_center_for_nmpc = np.array([current_obs_x_center, current_obs_y_center])
    param_psi_dot_val = psi_dot_at_last_interval
    opti.set_value(P_param, np.concatenate([x_current,
                                           [current_target_y],
                                           [target_v_val],
                                           current_obstacle_center_for_nmpc,
                                           [param_psi_dot_val]]))
    opti.set_initial(X_dv, X_guess_current)
    opti.set_initial(U_dv, U_guess_current)

    try:
        # 求解NMPC问题
        sol = opti.solve()
        u_optimal_sequence = sol.value(U_dv)
        x_predicted_sequence = sol.value(X_dv)
        u_apply = u_optimal_sequence[:, 0]
        U_guess_current = np.hstack((u_optimal_sequence[:, 1:], u_optimal_sequence[:, -1].reshape(-1,1)))
        X_guess_current = np.hstack((x_predicted_sequence[:, 1:], x_predicted_sequence[:, -1].reshape(-1,1)))
    except Exception as e:
        print(f"求解器在步骤 {i} (t={t_history[i]:.1f}s) 失败: {e}。应用零控制或前一控制。")
        u_apply = np.array([0.0, 0.0]) if i == 0 else u_history[:, i-1]
        # 当求解失败时，重置猜测值
        U_guess_current = np.zeros((nu, Np))
        temp_x_guess_fail = x_current.copy()
        current_X_for_reset = temp_x_guess_fail.copy()
        X_guess_reset = np.zeros_like(X_guess_current)
        X_guess_reset[:,0] = current_X_for_reset
        for k_reset in range(Np):
            u_reset = U_guess_current[:, k_reset]
            temp_f_val = f_discrete(current_X_for_reset, u_reset)
            if isinstance(temp_f_val, ca.DM):
                current_X_for_reset = temp_f_val.full().flatten()
            else:
                current_X_for_reset = ca.evalf(f_discrete(current_X_for_reset, u_reset)).full().flatten()
            X_guess_reset[:, k_reset+1] = current_X_for_reset
        X_guess_current = X_guess_reset

    # 更新控制历史
    u_history[:, i] = u_apply

    # 计算航向角速度
    current_v_for_psi_dot = x_current[3]
    current_delta_for_psi_dot = u_apply[1]
    psi_dot_at_last_interval = (current_v_for_psi_dot / L) * np.tan(current_delta_for_psi_dot)

    # 更新系统状态
    x_next_full = f_discrete(x_current, u_apply)
    x_next = x_next_full.full().flatten()
    x_next[2] = np.arctan2(np.sin(x_next[2]), np.cos(x_next[2])) # 归一化角度
    x_next[3] = np.clip(x_next[3], v_min, v_max) # 限制速度

    x_history[:, i+1] = x_next
    x_current = x_next

# 更新最后一个时间步的障碍物位置
last_time_step = t_history[-1]
obs_x_center_history[-1] = obs_x_initial + obs_speed_x * last_time_step
obs_y_center_history[-1] = obs_y_fixed

# 更新最后一个时间步的目标车道
if maneuver_state == "COMPLETED_MANEUVER" or maneuver_state == "RETURNING_TO_LANE_1":
    target_y_history[-1] = lane1_y
elif maneuver_state == "DRIVING_LANE_2":
     target_y_history[-1] = lane2_y
else:
    target_y_history[-1] = current_target_y

print("仿真数据生成完毕，开始生成动画...")

# --- 动画与绘图 ---
fig, axs = plt.subplots(4, 1, figsize=(12, 15), sharex=True, gridspec_kw={'height_ratios': [3, 1, 1, 1]})
plt.subplots_adjust(hspace=0.3)

# 设置第一个子图（车辆轨迹）
axs[0].set_xlabel('x 坐标 (m)')
axs[0].set_ylabel('y 坐标 (m)')
axs[0].set_title('NMPC 车辆避障与换道仿真动画 (参数优化后)')

# 绘制道路边界和车道线
axs[0].axhline(road_y_min_abs, color='k', linestyle='-', lw=1.5, label='道路下边界')
axs[0].axhline(road_y_max_abs, color='k', linestyle='-', lw=1.5, label='道路上边界')
axs[0].axhline(lane1_y, color='lightgray', linestyle=':', lw=1, label='第一车道中心')
axs[0].axhline(lane2_y, color='lightgray', linestyle=':', lw=1, label='第二车道中心')

# 创建障碍物图形
obstacle_bottom_left_x_init = obs_x_center_history[0] - obs_length / 2
obstacle_bottom_left_y_init = obs_y_center_history[0] - obs_width / 2
obstacle_shape_anim = Rectangle((obstacle_bottom_left_x_init, obstacle_bottom_left_y_init),
                                  obs_length, obs_width,
                                  facecolor=obs_color, alpha=0.7, label='障碍物',
                                  edgecolor='black', lw=0.5, zorder=5)
axs[0].add_artist(obstacle_shape_anim)

# 设置网格和坐标轴
axs[0].grid(True, linestyle=':', alpha=0.7)
axs[0].axis('equal')

# 设置显示范围
min_x_traj, max_x_traj = np.min(x_history[0,:]), np.max(x_history[0,:])
max_obs_x_final = obs_x_initial + obs_speed_x * t_history[-1] + obs_length / 2
min_obs_x_initial = obs_x_initial - obs_length / 2
axs[0].set_xlim(min(min_x_traj, min_obs_x_initial) - 10, max(max_x_traj, max_obs_x_final) + 20)
axs[0].set_ylim(road_y_min_abs - lane_width*0.5, road_y_max_abs + lane_width*0.5)

# 创建车辆图形
vehicle_rect_anim = Rectangle((-vehicle_plot_length / 2, -vehicle_plot_width / 2), vehicle_plot_length, vehicle_plot_width,
                                  facecolor='cornflowerblue', alpha=0.6, edgecolor='black', lw=0.7, zorder=10)
arrow_anim_line, = axs[0].plot([], [], lw=1.5, color='darkblue', zorder=11)
rear_axle_path_line, = axs[0].plot([], [], color='royalblue', linestyle='-', lw=1.0, alpha=0.7, label='车辆轨迹 (参考点)')
axs[0].plot([],[], color='lime', linestyle='--', lw=1.0, alpha=0.9, label='目标Y轨迹')
axs[0].legend(loc='upper left', fontsize='small')

# 创建时间文本显示
time_text = axs[0].text(0.02, 0.95, '', transform=axs[0].transAxes, fontsize='medium',
                        bbox=dict(boxstyle="round,pad=0.3", fc="wheat", ec="black", lw=0.5, alpha=0.7))

# 设置速度子图
axs[1].set_ylabel('速度 (m/s)')
axs[1].axhline(target_v_val, color='green', linestyle='--', label='目标速度')
line_v, = axs[1].plot([], [], lw=1.5, color='purple', label='实际速度')
axs[1].legend(fontsize='small', loc='lower right')
axs[1].grid(True, linestyle=':', alpha=0.7)
axs[1].set_ylim(min(v_min, np.min(x_history[3, :])) -1, max(v_max, np.max(x_history[3, :])) + 1)

# 设置航向角子图
axs[2].set_ylabel('航向角 (度)')
line_psi, = axs[2].plot([], [], lw=1.5, color='teal', label='实际航向角')
axs[2].legend(fontsize='small', loc='lower right')
axs[2].grid(True, linestyle=':', alpha=0.7)
min_psi_deg = np.rad2deg(np.min(x_history[2, :])) if len(x_history[2,:]) > 0 else -10
max_psi_deg = np.rad2deg(np.max(x_history[2, :])) if len(x_history[2,:]) > 0 else 10
margin_psi = 10
if abs(max_psi_deg - min_psi_deg) < 1e-6 :
    min_psi_deg -= margin_psi
    max_psi_deg += margin_psi
axs[2].set_ylim(min_psi_deg - margin_psi, max_psi_deg + margin_psi)

# 设置加速度和转向角子图
axs[3].set_xlabel('时间 (s)')
axs[3].set_ylabel('加速度 (m/s^2)', color='orangered')
line_a, = axs[3].plot([], [], lw=1.5, color='orangered', label='加速度')
axs[3].tick_params(axis='y', labelcolor='orangered')
axs[3].legend(fontsize='small', loc='upper left')
axs[3].grid(True, linestyle=':', alpha=0.7)
axs[3].set_ylim(a_min - 1, a_max + 1)

# 创建双y轴显示转向角
ax3_twin = axs[3].twinx()
ax3_twin.set_ylabel('转向角 (度)', color='darkgoldenrod')
line_delta, = ax3_twin.plot([], [], lw=1.5, color='darkgoldenrod', linestyle='--', label='转向角')
ax3_twin.tick_params(axis='y', labelcolor='darkgoldenrod')
ax3_twin.legend(fontsize='small', loc='upper right')
ax3_twin.set_ylim(np.rad2deg(delta_min) - 5, np.rad2deg(delta_max) + 5)

# 绘制目标轨迹
axs[0].plot(x_history[0,:len(target_y_history)], target_y_history, color='lime', linestyle='--', lw=1.0, alpha=0.9, label='目标Y轨迹')
axs[0].legend(loc='upper left', fontsize='small')

# 动画初始化函数
def init():
    axs[0].add_patch(vehicle_rect_anim)
    obs_bl_x_init = obs_x_center_history[0] - obs_length / 2
    obs_bl_y_init = obs_y_center_history[0] - obs_width / 2
    obstacle_shape_anim.set_xy((obs_bl_x_init, obs_bl_y_init))

    arrow_anim_line.set_data([], [])
    rear_axle_path_line.set_data([], [])
    time_text.set_text('')
    line_v.set_data([], [])
    line_psi.set_data([], [])
    line_a.set_data([], [])
    line_delta.set_data([], [])
    return [vehicle_rect_anim, arrow_anim_line, rear_axle_path_line, time_text,
            line_v, line_psi, line_a, line_delta, obstacle_shape_anim]

# 动画更新函数
def update(frame):
    # 更新车辆位置和姿态
    veh_center_x = x_history[0, frame]
    veh_center_y = x_history[1, frame]
    psi_rad = x_history[2, frame]

    # 更新车辆图形变换
    transform = Affine2D().rotate_around(0, 0, psi_rad) + Affine2D().translate(veh_center_x, veh_center_y) + axs[0].transData
    vehicle_rect_anim.set_transform(transform)

    # 更新车辆方向箭头
    arrow_length = vehicle_plot_length * 0.4
    arrow_end_x = veh_center_x + arrow_length * np.cos(psi_rad)
    arrow_end_y = veh_center_y + arrow_length * np.sin(psi_rad)
    arrow_anim_line.set_data([veh_center_x, arrow_end_x], [veh_center_y, arrow_end_y])

    # 更新车辆轨迹
    rear_axle_path_line.set_data(x_history[0, :frame+1], x_history[1, :frame+1])
    
    # 更新障碍物位置
    current_frame_obs_x_center = obs_x_center_history[frame]
    current_frame_obs_y_center = obs_y_center_history[frame]
    obstacle_bottom_left_x = current_frame_obs_x_center - obs_length / 2
    obstacle_bottom_left_y = current_frame_obs_y_center - obs_width / 2
    obstacle_shape_anim.set_xy((obstacle_bottom_left_x, obstacle_bottom_left_y))

    # 更新状态显示
    current_display_maneuver_state = "UNKNOWN"
    target_y_idx = min(frame, len(target_y_history) - 1)
    current_target_y_for_display = target_y_history[target_y_idx]

    veh_x_pos_frame = x_history[0, frame]
    frame_obs_x_center = obs_x_center_history[frame]
    frame_obs_front_x = frame_obs_x_center - obs_length / 2
    
    # 更新状态显示文本
    actual_y_pos = x_history[1,frame]
    if abs(actual_y_pos - lane1_y) < 0.25 and \
       veh_x_pos_frame > frame_obs_x_center + return_trigger_x_offset + stabilize_distance_after_return * 0.8 and \
       abs(current_target_y_for_display - lane1_y) < 0.1 :
           current_display_maneuver_state = "完成机动"
    elif abs(current_target_y_for_display - lane1_y) < 0.1:
        if veh_x_pos_frame < frame_obs_front_x - evade_trigger_distance * 0.5 :
            current_display_maneuver_state = "第一车道行驶"
        elif veh_x_pos_frame > frame_obs_x_center + pass_obstacle_distance :
            current_display_maneuver_state = "返回第一车道"
        else:
             current_display_maneuver_state = "第一车道 (近障)"
    elif abs(current_target_y_for_display - lane2_y) < 0.1:
        if veh_x_pos_frame < frame_obs_x_center + pass_obstacle_distance * 0.9 :
            current_display_maneuver_state = "变道至第二车道"
        else:
            current_display_maneuver_state = "第二车道行驶"

    # 更新状态文本
    time_text.set_text(f'时间: {t_history[frame]:.1f} s\n速度: {x_history[3, frame]:.2f} m/s\n状态: {current_display_maneuver_state}\n目标Y: {current_target_y_for_display:.1f}m\n实际Y: {actual_y_pos:.2f}m')

    # 更新速度曲线
    line_v.set_data(t_history[:frame+1], x_history[3, :frame+1])
    
    # 更新航向角曲线
    line_psi.set_data(t_history[:frame+1], np.rad2deg(x_history[2, :frame+1]))

    # 更新控制量曲线
    if frame < len(u_history[0,:]):
        time_for_controls = t_history[:frame+1]
        control_slice_a = u_history[0, :frame+1]
        control_slice_delta = u_history[1, :frame+1]
        line_a.set_data(time_for_controls, control_slice_a)
        line_delta.set_data(time_for_controls, np.rad2deg(control_slice_delta))
    elif frame > 0 :
        time_for_controls = t_history[:frame]
        control_slice_a = u_history[0, :frame]
        control_slice_delta = u_history[1, :frame]
        line_a.set_data(time_for_controls, control_slice_a)
        line_delta.set_data(time_for_controls, np.rad2deg(control_slice_delta))
    else:
        line_a.set_data([], [])
        line_delta.set_data([], [])

    return [vehicle_rect_anim, arrow_anim_line, rear_axle_path_line, time_text,
            line_v, line_psi, line_a, line_delta, obstacle_shape_anim]

# 创建动画
num_frames = len(t_history)
ani = FuncAnimation(fig, update, frames=num_frames,
                    init_func=init, blit=True, interval=Ts*1000, repeat=False)

# 保存动画
animation_filename = 'NMPC_避障变道仿真动画_v6_优化边界.gif'
try:
    ani.save(animation_filename, writer='pillow', fps=int(1/Ts))
    print(f"动画已保存为: {animation_filename}")
except Exception as e:
    print(f"保存动画失败: {e}")
    print("请确保已安装 Pillow (for GIF) 或 FFmpeg (for MP4)，并将其添加到系统路径。")
    print("如果问题持续，尝试在 FuncAnimation 中设置 blit=False。")

plt.show()
