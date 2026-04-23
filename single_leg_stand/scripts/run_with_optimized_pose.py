"""Run single-leg stand with the optimized 3s pose."""

import json
from pathlib import Path

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from direct_single_support import run_direct_single_support


def main() -> None:
    pose_path = Path(__file__).parent.parent / "results" / "optimized_pose_simulation_3s.json"
    with open(pose_path) as f:
        pose_override = json.load(f)

    # Remove extra keys that are not expected by run_direct_single_support
    pose_override.pop("optimization_info", None)
    pose_override.pop("history", None)
    pose_override.pop("params", None)

    print(f"Loading optimized pose from {pose_path}")
    print(f"base_position: {pose_override['base_position']}")
    print(f"base_orientation: {pose_override['base_orientation']}")
    print()

    result = run_direct_single_support(pose_override=pose_override, quiet=False)

    print("\n===== 结果汇总 =====")
    print(f"survived: {result.survived_s:.3f} s")
    print(f"final base z: {result.final_base_z:.3f} m")
    print(f"max contact slip: {result.max_contact_slip_mm:.2f} mm")
    print(f"max body slip: {result.max_body_slip_mm:.2f} mm")
    print(f"max CoP error: {result.max_cop_error_mm:.2f} mm")
    print(f"max swing force: {result.max_swing_force:.2f} N")
    print(f"max tangential force: {result.max_tangential_force:.2f} N")
    print(f"max friction ratio: {result.max_friction_ratio:.3f}")
    print(f"WBC failures: {result.wbc_failures}")
    print(f"success: {result.success}")


if __name__ == "__main__":
    main()
