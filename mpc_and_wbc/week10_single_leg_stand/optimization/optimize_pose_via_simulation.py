"""Simulation-based pose optimizer for single-leg stand (Plan A).

Optimizes the 12 reduced pose parameters using Nelder-Mead,
with the objective being max contact slip from a 3s WBC benchmark.
WBC controller parameters remain fixed.
"""

import json
import math
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.optimize import minimize

import sys
sys.path.insert(0, str(Path(__file__).parent.parent))

from direct_single_support import run_direct_single_support
from direct_single_support_config import DIRECT_SINGLE_SUPPORT_CONFIG as cfg
from robot_model import RobotModel


@dataclass
class PoseParams:
    """12 reduced pose parameters that map to the full robot configuration."""

    base_height: float = 0.72
    base_lateral_shift: float = 0.04
    base_roll: float = 0.02
    support_roll_delta: float = 0.04
    swing_roll_delta: float = 0.16
    support_pitch_delta: float = 0.02
    swing_hip_pitch: float = 0.10
    swing_knee: float = 1.05
    swing_ankle_pitch: float = -0.55
    support_arm_shoulder_pitch: float = -1.2
    support_arm_shoulder_roll: float = 0.3
    support_arm_elbow: float = 0.8

    def to_array(self) -> np.ndarray:
        return np.array(
            [
                self.base_height,
                self.base_lateral_shift,
                self.base_roll,
                self.support_roll_delta,
                self.swing_roll_delta,
                self.support_pitch_delta,
                self.swing_hip_pitch,
                self.swing_knee,
                self.swing_ankle_pitch,
                self.support_arm_shoulder_pitch,
                self.support_arm_shoulder_roll,
                self.support_arm_elbow,
            ],
            dtype=float,
        )

    @classmethod
    def from_array(cls, arr: np.ndarray) -> "PoseParams":
        return cls(*arr.tolist())

    @classmethod
    def from_config(cls) -> "PoseParams":
        p = cfg.pose
        return cls(
            base_height=p.base_height,
            base_lateral_shift=p.base_lateral_shift,
            base_roll=p.base_roll,
            support_roll_delta=p.support_roll_delta,
            swing_roll_delta=p.swing_roll_delta,
            support_pitch_delta=p.support_pitch_delta,
            swing_hip_pitch=p.swing_hip_pitch,
            swing_knee=p.swing_knee,
            swing_ankle_pitch=p.swing_ankle_pitch,
            support_arm_shoulder_pitch=p.support_arm_shoulder_pitch,
            support_arm_shoulder_roll=p.support_arm_shoulder_roll,
            support_arm_elbow=p.support_arm_elbow,
        )


def build_pose_override(params: PoseParams) -> dict:
    """Build a pose_override dict compatible with run_direct_single_support()."""
    env_cfg = cfg.env
    support_leg = "right" if env_cfg.support_foot_name.startswith("right") else "left"
    swing_leg = "left" if support_leg == "right" else "right"
    support_sign = 1.0 if support_leg == "right" else -1.0

    # Base pose
    base_pos = np.array(
        [
            0.0,
            -support_sign * params.base_lateral_shift,
            params.base_height,
        ],
        dtype=float,
    )
    base_orn = np.array(
        [math.sin(-support_sign * params.base_roll / 2.0), 0.0, 0.0, math.cos(-support_sign * params.base_roll / 2.0)],
        dtype=float,
    )

    # Joint angles (need RobotModel to get DOF order)
    pose = dict(env_cfg.standing_joint_angles)
    pose[f"{support_leg}_hip_pitch_joint"] = -0.28 - params.support_pitch_delta
    pose[f"{support_leg}_knee_joint"] = 0.62 + 0.4 * params.support_pitch_delta
    pose[f"{support_leg}_ankle_pitch_joint"] = -0.34 + 0.2 * params.support_pitch_delta
    pose[f"{support_leg}_hip_roll_joint"] = support_sign * params.support_roll_delta
    pose[f"{support_leg}_ankle_roll_joint"] = -support_sign * params.support_roll_delta
    pose[f"{swing_leg}_hip_pitch_joint"] = params.swing_hip_pitch
    pose[f"{swing_leg}_knee_joint"] = params.swing_knee
    pose[f"{swing_leg}_ankle_pitch_joint"] = params.swing_ankle_pitch
    pose[f"{swing_leg}_hip_roll_joint"] = support_sign * params.swing_roll_delta
    pose[f"{swing_leg}_ankle_roll_joint"] = -support_sign * params.swing_roll_delta
    pose[f"{support_leg}_shoulder_pitch_joint"] = params.support_arm_shoulder_pitch
    pose[f"{support_leg}_shoulder_roll_joint"] = support_sign * params.support_arm_shoulder_roll
    pose[f"{support_leg}_elbow_joint"] = support_sign * params.support_arm_elbow

    robot = RobotModel(env_cfg.model_path)
    q_target = np.zeros(len(robot.dof_joint_names))
    for idx, name in enumerate(robot.dof_joint_names):
        q_target[idx] = pose.get(name, 0.0)

    return {
        "base_position": base_pos.tolist(),
        "base_orientation": base_orn.tolist(),
        "joint_angles": q_target.tolist(),
        "params": params.to_array().tolist(),
    }


def run_benchmark_with_params(
    params: PoseParams, quiet: bool = True, duration: float | None = None
):
    """Run the WBC benchmark with the given pose parameters."""
    pose_override = build_pose_override(params)
    return run_direct_single_support(
        pose_override=pose_override, quiet=quiet, duration_override=duration
    )


class SimulationObjective:
    """Callable objective for scipy.optimize."""

    def __init__(self, full_duration: float, opt_duration: float):
        self.full_duration = full_duration
        self.opt_duration = opt_duration
        self.eval_count = 0
        self.best_cost = float("inf")
        self.best_params: np.ndarray | None = None
        self.start_time = time.time()
        self.history: list[dict] = []

    def __call__(self, x: np.ndarray) -> float:
        self.eval_count += 1
        params = PoseParams.from_array(x)
        t0 = time.time()
        result = run_benchmark_with_params(params, quiet=True, duration=self.opt_duration)
        elapsed = time.time() - t0

        # Primary objective: contact slip, with heavy penalty for falling
        if result.survived_s < self.opt_duration - 0.01:
            cost = 1e4 + (self.opt_duration - result.survived_s) * 1e4
        else:
            cost = result.max_contact_slip_mm
            # Small auxiliary penalties to guide the search
            cost += 0.3 * result.max_cop_error_mm
            cost += 20.0 * result.max_friction_ratio

        self.history.append(
            {
                "eval": self.eval_count,
                "cost": float(cost),
                "survived_s": float(result.survived_s),
                "slip_mm": float(result.max_contact_slip_mm),
                "cop_mm": float(result.max_cop_error_mm),
                "friction": float(result.max_friction_ratio),
                "success": bool(result.success),
                "time_s": float(elapsed),
            }
        )

        if cost < self.best_cost:
            self.best_cost = cost
            self.best_params = x.copy()
            print(
                f"  [eval {self.eval_count:3d}] NEW BEST cost={cost:8.3f}  "
                f"slip={result.max_contact_slip_mm:6.2f}mm  "
                f"cop={result.max_cop_error_mm:6.2f}mm  "
                f"fric={result.max_friction_ratio:.3f}  "
                f"surv={result.survived_s:.2f}s  ({elapsed:.1f}s)"
            )
        elif self.eval_count % 5 == 0:
            total_elapsed = time.time() - self.start_time
            print(
                f"  [eval {self.eval_count:3d}] cost={cost:8.3f}  "
                f"best={self.best_cost:8.3f}  "
                f"surv={result.survived_s:.2f}s  "
                f"[{total_elapsed:.0f}s elapsed]"
            )

        return float(cost)


def optimize_pose_via_simulation(
    maxiter: int = 200, opt_duration: float = 1.0
) -> dict:
    """Run Nelder-Mead to optimize pose parameters against WBC simulation.

    Uses a shorter ``opt_duration`` (default 1.0 s) for the inner loop to keep
    evaluation cost low, then verifies the best candidate with the full 3.0 s.
    """
    full_duration = cfg.control.duration
    x0 = PoseParams.from_config().to_array()

    # Build small initial simplex (1-5% perturbation around x0)
    scales = np.array(
        [0.02, 0.005, 0.005, 0.01, 0.02, 0.01, 0.05, 0.05, 0.05, 0.1, 0.05, 0.05],
        dtype=float,
    )
    simplex = np.zeros((len(x0) + 1, len(x0)), dtype=float)
    simplex[0] = x0
    for i in range(len(x0)):
        simplex[i + 1] = x0.copy()
        simplex[i + 1, i] += scales[i]

    # Bounds for reference (Nelder-Mead doesn't enforce bounds, but we clip manually)
    bounds = [
        (0.50, 0.90),      # base_height
        (-0.10, 0.10),     # base_lateral_shift
        (-0.20, 0.20),     # base_roll
        (-0.20, 0.20),     # support_roll_delta
        (-0.20, 0.20),     # swing_roll_delta
        (-0.20, 0.20),     # support_pitch_delta
        (-0.50, 0.50),     # swing_hip_pitch
        (-0.20, 1.50),     # swing_knee
        (-1.00, 0.20),     # swing_ankle_pitch
        (-2.00, 0.50),     # support_arm_shoulder_pitch
        (-0.50, 0.50),     # support_arm_shoulder_roll
        (-1.00, 1.00),     # support_arm_elbow
    ]

    # Wrap objective with clipping
    objective_raw = SimulationObjective(full_duration, opt_duration)

    def objective_clipped(x: np.ndarray) -> float:
        x_clipped = x.copy()
        for i, (lo, hi) in enumerate(bounds):
            x_clipped[i] = float(np.clip(x_clipped[i], lo, hi))
        return objective_raw(x_clipped)

    print("=" * 70)
    print("Simulation-based Pose Optimization (Plan A)")
    print("=" * 70)
    print(f"Variables:       {len(x0)} reduced pose parameters")
    print(f"Inner duration:  {opt_duration}s  (fast evaluation)")
    print(f"Full duration:   {full_duration}s  (final verification)")
    print(f"Optimizer:       Nelder-Mead (local search from hand-tuned x0)")
    print(f"Max iterations:  {maxiter}")
    print("-" * 70)

    # First evaluation at x0
    print("Evaluating hand-tuned baseline...")
    baseline_cost = objective_clipped(x0)
    print(f"Baseline cost: {baseline_cost:.3f}\n")

    result = minimize(
        objective_clipped,
        x0,
        method="Nelder-Mead",
        options={
            "maxiter": maxiter,
            "xatol": 1e-4,
            "fatol": 1e-3,
            "disp": False,
            "initial_simplex": simplex,
        },
    )

    inner_time = time.time() - objective_raw.start_time
    print("\n" + "=" * 70)
    print("Inner-loop Optimization Complete")
    print("=" * 70)
    print(f"Success:         {result.success}")
    print(f"Status:          {result.status}")
    print(f"Message:         {result.message}")
    print(f"Iterations:      {result.nit}")
    print(f"Function evals:  {result.nfev}")
    print(f"Inner-loop time: {inner_time:.1f}s")
    print(f"Baseline cost:   {baseline_cost:.3f}")
    print(f"Best cost:       {objective_raw.best_cost:.3f}")

    # Verify the best candidate with the full 3.0 s simulation
    best_params = PoseParams.from_array(
        objective_raw.best_params if objective_raw.best_params is not None else x0
    )
    print("\nVerifying best candidate with full 3.0 s simulation...")
    full_result = run_benchmark_with_params(best_params, quiet=True, duration=full_duration)
    print(
        f"Full verification: slip={full_result.max_contact_slip_mm:.2f}mm  "
        f"cop={full_result.max_cop_error_mm:.2f}mm  "
        f"fric={full_result.max_friction_ratio:.3f}  "
        f"surv={full_result.survived_s:.2f}s  success={full_result.success}"
    )

    # Build final pose_override
    pose_override = build_pose_override(best_params)
    pose_override["optimization_info"] = {
        "success": bool(result.success),
        "nit": int(result.nit),
        "nfev": int(result.nfev),
        "baseline_cost": float(baseline_cost),
        "best_inner_cost": float(objective_raw.best_cost),
        "inner_time_s": float(inner_time),
        "full_verification": {
            "slip_mm": float(full_result.max_contact_slip_mm),
            "cop_mm": float(full_result.max_cop_error_mm),
            "friction_ratio": float(full_result.max_friction_ratio),
            "survived_s": float(full_result.survived_s),
            "success": bool(full_result.success),
        },
    }
    pose_override["history"] = objective_raw.history

    return pose_override


def main() -> None:
    pose_override = optimize_pose_via_simulation(maxiter=100, opt_duration=3.0)

    output_path = Path(__file__).parent.parent / "results" / "optimized_pose_simulation_3s.json"
    with open(output_path, "w") as f:
        json.dump(pose_override, f, indent=2)

    print(f"\nOptimized pose saved to {output_path}")


if __name__ == "__main__":
    main()
