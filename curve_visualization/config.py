"""Global configuration: step size, colors, sample waypoints
"""

# ========== General Parameters ==========
STEP = 0.1          # Interpolation step size
MAX_CURV = 0.25     # Maximum curvature (Dubins / Reeds-Shepp)
OFFSET = 3.0        # Bezier control point offset
BSPLINE_ORDER = 3   # B-Spline order

# ========== Visualization Colors ==========
COLORS = {
    "bezier": "#1f77b4",
    "bspline": "#ff7f0e",
    "cubic_spline": "#2ca02c",
    "dubins": "#d62728",
    "reeds_shepp": "#9467bd",
    "quintic": "#8c564b",
}

# ========== Sample Waypoints ==========

# Example 1: Simple S-shape path (Points2d)
waypoints_s_shape = [
    (0.0, 0.0),
    (2.0, 1.0),
    (4.0, 0.0),
    (6.0, -1.0),
    (8.0, 0.0),
]

# Example 2: Straight line path
waypoints_line = [
    (0.0, 0.0),
    (3.0, 0.0),
    (6.0, 0.0),
    (9.0, 0.0),
]

# Example 3: Turning path
waypoints_turn = [
    (0.0, 0.0),
    (2.0, 2.0),
    (4.0, 4.0),
    (6.0, 4.0),
]

# Example 4: Closed loop shape
waypoints_loop = [
    (0.0, 0.0),
    (2.0, 2.0),
    (4.0, 0.0),
    (2.0, -2.0),
    (0.0, 0.0),
]

# ========== Pose Waypoints (Points3d) ==========
# Start/Goal for Dubins / Reeds-Shepp
start_pose = (0.0, 0.0, 0.0)         # (x, y, theta)
goal_pose = (8.0, 2.0, 1.57)         # (x, y, theta) approx 90 deg

# PVA boundary conditions for QuinticPolynomial
quintic_start = (0.0, 0.0, 0.0)      # (pos, vel, acc)
quintic_end = (10.0, 0.0, 0.0)       # (pos, vel, acc)
quintic_T = 5.0                       # Total time
