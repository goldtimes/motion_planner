"""
Unit tests: verify basic functionality of all curve generation algorithms
"""

import sys
import math
sys.path.insert(0, "/motion_ws/curve_visualization")

from curves import (
    BezierCurve, BSplineCurve, CubicSplineCurve,
    DubinsCurve, ReedsSheppCurve, QuinticPolynomial, QuinticCurve
)
from config import (
    waypoints_s_shape, start_pose, goal_pose,
    quintic_start, quintic_end, quintic_T
)


def _check_path_valid(path, name, min_len=2):
    """Check basic validity of the path"""
    assert len(path) >= min_len, f"{name}: too few points ({len(path)} < {min_len})"
    return True


def test_bezier():
    """Test Bezier curve"""
    print("[TEST] BezierCurve ... ", end="")
    curve = BezierCurve(step=0.1)

    # 1. Basic generation
    path = curve.generation(start_pose, goal_pose)
    _check_path_valid(path, "Bezier generation")
    print(f"OK (path: {len(path)} pts)")


def test_bspline():
    """Test B-Spline curve"""
    print("[TEST] BSplineCurve ... ", end="")
    curve = BSplineCurve(step=0.05, order=3)

    # 1. Interpolation mode
    path = curve.run(waypoints_s_shape)
    _check_path_valid(path, "BSpline interpolation")
    print(f"OK (path: {len(path)} pts)", end=" ")

    # 2. Approximation mode
    curve.set_spline_mode(BSplineCurve.SPLINE_APPROXIMATION)
    path2 = curve.run(waypoints_s_shape)
    _check_path_valid(path2, "BSpline approximation")
    print(f"/ approx OK ({len(path2)} pts)")

    # 3. Different parameterization
    curve.set_spline_mode(BSplineCurve.SPLINE_INTERPOLATION)
    curve.set_param_mode(BSplineCurve.PARAM_UNIFORM)
    path3 = curve.run(waypoints_s_shape)
    _check_path_valid(path3, "BSpline uniform")
    print(f"   Uniform: {len(path3)} pts", end="")

    curve.set_param_mode(BSplineCurve.PARAM_CENTRIPETAL)
    path4 = curve.run(waypoints_s_shape)
    _check_path_valid(path4, "BSpline centripetal")
    print(f" / Centripetal: {len(path4)} pts")


def test_cubic_spline():
    """Test cubic spline curve"""
    print("[TEST] CubicSplineCurve ... ", end="")
    curve = CubicSplineCurve(step=0.1)

    path = curve.run(waypoints_s_shape)
    _check_path_valid(path, "CubicSpline")
    print(f"OK (path: {len(path)} pts)")


def test_dubins():
    """Test Dubins curve"""
    print("[TEST] DubinsCurve ... ", end="")
    curve = DubinsCurve(step=0.1, max_curv=0.25)

    # Different start/goal orientations
    test_cases = [
        ((0, 0, 0), (5, 0, 0), "straight"),
        ((0, 0, 0), (5, 3, 1.57), "right-up"),
        ((0, 0, 0), (3, 4, 0), "parallel"),
        ((0, 0, 0), (0, 5, 1.57), "vertical"),
    ]
    for start, goal, desc in test_cases:
        path = curve.generation(start, goal)
        _check_path_valid(path, f"Dubins {desc}", min_len=2)
        print(f"{desc}({len(path)}pts) ", end="")
    print("OK")


def test_reeds_shepp():
    """Test Reeds-Shepp curve"""
    print("[TEST] ReedsSheppCurve ... ", end="")
    curve = ReedsSheppCurve(step=0.1, max_curv=0.25)

    test_cases = [
        ((0, 0, 0), (5, 0, 0), "straight"),
        ((0, 0, 0), (5, 3, 1.57), "right-up"),
        ((0, 0, 0), (3, 4, 0), "parallel"),
        ((0, 0, 0), (0, 5, 1.57), "vertical"),
    ]
    for start, goal, desc in test_cases:
        path = curve.generation(start, goal)
        _check_path_valid(path, f"ReedsShepp {desc}", min_len=2)
        print(f"{desc}({len(path)}pts) ", end="")
    print("OK")


def test_quintic():
    """Test quintic polynomial"""
    print("[TEST] QuinticPolynomial ... ", end="")

    # 1. Direct solver
    poly = QuinticPolynomial()
    poly.solve(quintic_start, quintic_end, quintic_T)

    # Verify boundary conditions
    assert abs(poly.x(0) - quintic_start[0]) < 1e-6, "Start position mismatch"
    assert abs(poly.x(quintic_T) - quintic_end[0]) < 1e-6, "Goal position mismatch"
    assert abs(poly.dx(0) - quintic_start[1]) < 1e-6, "Start velocity mismatch"
    assert abs(poly.dx(quintic_T) - quintic_end[1]) < 1e-6, "Goal velocity mismatch"
    assert abs(poly.ddx(0) - quintic_start[2]) < 1e-6, "Start acceleration mismatch"
    assert abs(poly.ddx(quintic_T) - quintic_end[2]) < 1e-6, "Goal acceleration mismatch"

    print(f"coeffs OK", end=" ")

    # 2. QuinticCurve adapter
    curve = QuinticCurve(step=0.1)
    points = [(0, 0), (5, 3)]
    path = curve.run(points)
    _check_path_valid(path, "QuinticCurve")
    print(f"/ curve({len(path)}pts) OK")


def run_all():
    """Run all tests"""
    print("=" * 50)
    print("Curve Generation Algorithm Unit Tests")
    print("=" * 50)

    tests = [
        test_bezier,
        test_bspline,
        test_cubic_spline,
        test_dubins,
        test_reeds_shepp,
        test_quintic,
    ]

    passed = 0
    failed = 0
    for test in tests:
        try:
            test()
            passed += 1
        except Exception as e:
            print(f"FAILED: {e}")
            failed += 1

    print("=" * 50)
    print(f"Results: {passed} passed, {failed} failed / {len(tests)} total")
    print("=" * 50)
    return failed == 0


if __name__ == "__main__":
    success = run_all()
    sys.exit(0 if success else 1)
