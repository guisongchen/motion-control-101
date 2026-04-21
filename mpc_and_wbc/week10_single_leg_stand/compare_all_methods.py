"""Compare hand-tuned, static-optimized, and simulation-optimized initial poses."""

import json
from pathlib import Path

from direct_single_support import run_direct_single_support, DirectSingleSupportResult


def print_result(label: str, r: DirectSingleSupportResult) -> None:
    print(f"  {label:25s}  survived={r.survived_s:.3f}s  base_z={r.final_base_z:.3f}m")
    print(f"    contact_slip={r.max_contact_slip_mm:6.2f}mm  body_slip={r.max_body_slip_mm:6.2f}mm")
    print(f"    cop_error={r.max_cop_error_mm:6.2f}mm  swing_force={r.max_swing_force:6.2f}N")
    print(f"    friction_ratio={r.max_friction_ratio:.3f}  tangential={r.max_tangential_force:6.2f}N")
    print(f"    wbc_failures={r.wbc_failures}  success={r.success}")


def main() -> None:
    print("\n" + "=" * 70)
    print("1. HAND-TUNED baseline")
    print("=" * 70)
    hand_tuned = run_direct_single_support(quiet=True)
    print_result("Hand-tuned", hand_tuned)

    print("\n" + "=" * 70)
    print("2. STATIC OPTIMIZED (gravity torque + CoM alignment + constraints)")
    print("=" * 70)
    static_path = Path(__file__).parent / "optimized_pose.json"
    static_pose = None
    if static_path.exists():
        with open(static_path) as f:
            static_pose = json.load(f)
        static = run_direct_single_support(pose_override=static_pose, quiet=True)
        print_result("Static", static)
    else:
        print("  (not found)")

    print("\n" + "=" * 70)
    print("3. SIMULATION OPTIMIZED (Nelder-Mead on 1.0s WBC, verified 3.0s)")
    print("=" * 70)
    sim_path = Path(__file__).parent / "optimized_pose_simulation.json"
    sim_pose = None
    if sim_path.exists():
        with open(sim_path) as f:
            sim_data = json.load(f)
        sim_pose = {
            "base_position": sim_data["base_position"],
            "base_orientation": sim_data["base_orientation"],
            "joint_angles": sim_data["joint_angles"],
        }
        sim = run_direct_single_support(pose_override=sim_pose, quiet=True)
        print_result("Simulation", sim)
    else:
        print("  (not found)")

    # Summary table
    print("\n" + "=" * 70)
    print("SUMMARY TABLE")
    print("=" * 70)

    results = [("Hand-tuned", hand_tuned)]
    if static_pose:
        results.append(("Static", static))
    if sim_pose:
        results.append(("Simulation", sim))

    headers = ["Metric"] + [name for name, _ in results]
    print(f"{'Metric':<22}" + "".join(f"{name:>14}" for name, _ in results))
    print("-" * (22 + 14 * len(results)))

    def row(metric, getter):
        vals = [getter(r) for _, r in results]
        print(f"{metric:<22}" + "".join(f"{v:14.3f}" for v in vals))

    row("Survived [s]", lambda r: r.survived_s)
    row("Final base z [m]", lambda r: r.final_base_z)
    row("Contact slip [mm]", lambda r: r.max_contact_slip_mm)
    row("Body slip [mm]", lambda r: r.max_body_slip_mm)
    row("CoP error [mm]", lambda r: r.max_cop_error_mm)
    row("Swing force [N]", lambda r: r.max_swing_force)
    row("Friction ratio", lambda r: r.max_friction_ratio)
    row("Tangential F [N]", lambda r: r.max_tangential_force)
    row("WBC failures", lambda r: r.wbc_failures)

    print("-" * (22 + 14 * len(results)))
    successes = [str(r.success) for _, r in results]
    print(f"{'Success':<22}" + "".join(f"{s:>14}" for s in successes))
    print("=" * (22 + 14 * len(results)))

    if sim_pose:
        print("\nSimulation optimizer info:")
        info = sim_data.get("optimization_info", {})
        print(f"  Inner-loop best cost: {info.get('best_inner_cost', 'N/A'):.3f}")
        print(f"  Evaluations:          {info.get('nfev', 'N/A')}")
        print(f"  Inner-loop time:      {info.get('inner_time_s', 'N/A'):.1f}s")
        fv = info.get("full_verification", {})
        print(f"  Full 3.0s slip:       {fv.get('slip_mm', 'N/A'):.2f}mm")
        print(f"  Full 3.0s CoP:        {fv.get('cop_mm', 'N/A'):.2f}mm")
        print(f"  Full 3.0s friction:   {fv.get('friction_ratio', 'N/A'):.3f}")


if __name__ == "__main__":
    main()
