"""Replay recorded rolling plans against the real URDF, without hardware I/O."""
import argparse
import importlib.util
import json
import statistics
import subprocess
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from franka_hand_teleop.deployment import load_profile
from franka_hand_teleop.rolling import State, checked_curve
from franka_hand_teleop.workspace_guard import WorkspaceGuard


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--compare", type=Path, help="Previous pure planner module for timing comparison")
    args = parser.parse_args()
    plans = [json.loads(line.split("ROLLING PLAN: ", 1)[1])
             for line in args.log.read_text().splitlines() if "ROLLING PLAN: " in line]
    assert plans, "No recorded rolling plans"
    share = Path(get_package_share_directory("franka_hand_teleop"))
    profile, limits = load_profile(share/"config/real_profile.yaml")
    urdf = subprocess.check_output(["xacro", str(share/"urdf/fake_fr3.urdf.xacro")], text=True)
    guard = WorkspaceGuard(urdf, limits)
    planners = [("current", checked_curve)]
    if args.compare:
        import sys
        spec = importlib.util.spec_from_file_location("previous_rolling", args.compare)
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        planners.insert(0, ("previous", module.checked_curve))
    for name, planner in planners:
        elapsed, failures = [], 0
        for plan in plans:
            started = time.monotonic()
            curve = planner(State(tuple(plan["initial_q"]), tuple(plan["initial_v"]),
                                  tuple(plan["initial_a"])), plan["target_q"], profile["motion"],
                            guard, budget_s=.02, minimum_duration_s=.2)
            elapsed.append((time.monotonic()-started)*1000)
            failures += curve is None
        print(f"{name}: plans={len(plans)}; missed 20 ms budget={failures}; "
              f"planning median={statistics.median(elapsed):.3f} ms; max={max(elapsed):.3f} ms", flush=True)
        if name == "current":
            assert failures == 0, "Recorded admissible plans missed the planning budget"


if __name__ == "__main__":
    main()
