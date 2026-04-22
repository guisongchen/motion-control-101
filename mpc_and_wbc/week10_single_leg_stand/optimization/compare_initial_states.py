"""Compare hand-tuned vs optimized initial pose for single-leg stand WBC benchmark."""

import json
from pathlib import Path

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from direct_single_support import run_direct_single_support, DirectSingleSupportResult


def print_result(label: str, r: DirectSingleSupportResult) -> None:
    print(f"  {label:25s}  survived={r.survived_s:.3f}s  base_z={r.final_base_z:.3f}m")
    print(f"    contact_slip={r.max_contact_slip_mm:6.2f}mm  body_slip={r.max_body_slip_mm:6.2f}mm")
    print(f"    cop_error={r.max_cop_error_mm:6.2f}mm  swing_force={r.max_swing_force:6.2f}N")
    print(f"    friction_ratio={r.max_friction_ratio:.3f}  tangential={r.max_tangential_force:6.2f}N")
    print(f"    wbc_failures={r.wbc_failures}  success={r.success}")


def main() -> None:
    pose_path = Path(__file__).parent.parent / "results" / "optimized_pose.json"
    if not pose_path.exists():
        print(f"Optimized pose not found at {pose_path}")
        print("Please run optimize_static_pose.py first.")
        return

    with open(pose_path) as f:
        pose_override = json.load(f)

    print("\n" + "=" * 70)
    print("Running benchmark with HAND-TUNED initial pose")
    print("=" * 70)
    hand_tuned_result = run_direct_single_support()
    print_result("Hand-tuned", hand_tuned_result)

    print("\n" + "=" * 70)
    print("Running benchmark with OPTIMIZED initial pose")
    print("=" * 70)
    optimized_result = run_direct_single_support(pose_override=pose_override)
    print_result("Optimized", optimized_result)

    print("\n" + "=" * 70)
    print("COMPARISON SUMMARY")
    print("=" * 70)

    def delta(old: float, new: float) -> str:
        if old == 0:
            return "N/A"
        pct = 100.0 * (new - old) / abs(old)
        return f"{pct:+.1f}%"

    rows = [
        ("Survived [s]", hand_tuned_result.survived_s, optimized_result.survived_s, "higher"),
        ("Final base z [m]", hand_tuned_result.final_base_z, optimized_result.final_base_z, "higher"),
        ("Max contact slip [mm]", hand_tuned_result.max_contact_slip_mm, optimized_result.max_contact_slip_mm, "lower"),
        ("Max body slip [mm]", hand_tuned_result.max_body_slip_mm, optimized_result.max_body_slip_mm, "lower"),
        ("Max CoP error [mm]", hand_tuned_result.max_cop_error_mm, optimized_result.max_cop_error_mm, "lower"),
        ("Max swing force [N]", hand_tuned_result.max_swing_force, optimized_result.max_swing_force, "lower"),
        ("Max friction ratio", hand_tuned_result.max_friction_ratio, optimized_result.max_friction_ratio, "lower"),
        ("Max tangential force [N]", hand_tuned_result.max_tangential_force, optimized_result.max_tangential_force, "lower"),
        ("WBC failures", hand_tuned_result.wbc_failures, optimized_result.wbc_failures, "lower"),
    ]

    print(f"{'Metric':<30} {'Hand-tuned':>12} {'Optimized':>12} {'Change':>10} {'Better?':>8}")
    print("-" * 70)
    for name, old, new, better_dir in rows:
        change = delta(old, new)
        if better_dir == "lower":
            is_better = "YES" if new < old else "NO" if new > old else "="
        else:
            is_better = "YES" if new > old else "NO" if new < old else "="
        print(f"{name:<30} {old:12.3f} {new:12.3f} {change:>10} {is_better:>8}")

    print("-" * 70)
    print(f"{'Success':<30} {str(hand_tuned_result.success):>12} {str(optimized_result.success):>12}")
    print("=" * 70)

    if optimized_result.success and not hand_tuned_result.success:
        print("\n>>> Optimized pose SUCCESS, hand-tuned FAILED.")
    elif not optimized_result.success and hand_tuned_result.success:
        print("\n>>> Hand-tuned pose SUCCESS, optimized FAILED.")
    elif optimized_result.success and hand_tuned_result.success:
        print("\n>>> Both poses succeeded.")
    else:
        print("\n>>> Both poses failed.")


if __name__ == "__main__":
    main()
