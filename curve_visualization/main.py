#!/usr/bin/env python3
"""
曲线生成算法：测试与可视化入口

用法:
  python main.py              # 运行所有测试 + 可视化
  python main.py --test       # 仅运行测试
  python main.py --vis        # 仅运行可视化
  python main.py --curve bezier  # 仅测试指定曲线
"""

import sys
import argparse

sys.path.insert(0, "/motion_ws/curve_visualization")

from tests.test_all_curves import run_all as run_tests
from visualization.visualize_all import run_all as run_visualization


def main():
    parser = argparse.ArgumentParser(description="曲线生成算法测试与可视化")
    parser.add_argument("--test", action="store_true", help="仅运行测试")
    parser.add_argument("--vis", action="store_true", help="仅运行可视化")
    parser.add_argument("--curve", type=str, default=None,
                        help="指定曲线 (bezier/bspline/cubic/dubins/reeds_shepp/quintic)")
    args = parser.parse_args()

    run_test = args.test or (not args.vis and not args.test)
    run_vis = args.vis or (not args.vis and not args.test)

    success = True
    if run_test:
        if args.curve:
            print(f"仅测试曲线: {args.curve}")
        success = run_tests()

    if run_vis:
        run_visualization()

    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
