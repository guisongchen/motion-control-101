"""Offline static pose optimizer for single-leg stand initial configuration."""

import json
import math
from pathlib import Path

import numpy as np
from scipy.optimize import minimize

import sys
sys.path.insert(0, str(Path(__file__).parent.parent))

from direct_single_support_config import DIRECT_SINGLE_SUPPORT_CONFIG as cfg
from direct_single_support import build_direct_pose, quat_from_roll
from robot_model import RobotModel


def rpy_from_rotation_matrix(R: np.ndarray) -> np.ndarray:
    """Extract roll-pitch-yaw from a 3x3 rotation matrix."""
    roll = math.atan2(R[2, 1], R[2, 2])
    pitch = math.atan2(-R[2, 0], math.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2))
    yaw = math.atan2(R[1, 0], R[0, 0])
    return np.array([roll, pitch, yaw], dtype=float)


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Convert roll-pitch-yaw to xyzw quaternion."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * sy
    q = np.array([qx, qy, qz, qw], dtype=float)
    return q / np.linalg.norm(q)


def build_q_from_x(x: np.ndarray, nq: int) -> np.ndarray:
    """Build 30D q vector from 29D optimization variables."""
    q = np.zeros(nq)
    q[:3] = x[:3]
    q[3:7] = quat_from_rpy(x[3], x[4], x[5])
    q[7:] = x[6:]
    return q


class StaticPoseOptimizer:
    """Optimize a statically stable single-leg standing pose."""

    def __init__(self, robot: RobotModel, support_foot_name: str, weights: dict):
        self.robot = robot
        self.support_link = robot.link_name_to_index[support_foot_name]
        self.swing_link = next(
            link
            for link in [
                robot.link_name_to_index[name] for name in cfg.env.foot_link_names
            ]
            if link != self.support_link
        )
        self.weights = weights

        # Setup initial pose from hand-tuned config
        self._setup_initial_pose()

        # Read target support foot pose (from hand-tuned configuration)
        self.target_support_pos = np.array(
            robot.data.xpos[self.support_link], copy=True
        )
        self.target_support_rpy = rpy_from_rotation_matrix(
            np.array(robot.data.xmat[self.support_link]).reshape(3, 3)
        )

        # Read corner patch local positions for margin calculation
        initial_contact = robot.get_contact_metrics(self.support_link)["position"].copy()
        self.corner_local_positions = self._resolve_corner_positions(initial_contact)

        # Nominal joint angles for regularization
        self.q_nominal = build_direct_pose(robot.dof_joint_names, self.support_leg)

        # Build bounds
        self.bounds = self._build_bounds()

    def _setup_initial_pose(self) -> None:
        """Setup the hand-tuned initial pose to read constraint targets."""
        env_cfg = cfg.env
        pose_cfg = cfg.pose
        support_leg = (
            "right" if env_cfg.support_foot_name.startswith("right") else "left"
        )
        self.support_leg = support_leg
        support_sign = 1.0 if support_leg == "right" else -1.0

        base_pos = np.array(
            [
                0.0,
                -support_sign * pose_cfg.base_lateral_shift,
                pose_cfg.base_height,
            ],
            dtype=float,
        )
        base_orn = quat_from_roll(-support_sign * pose_cfg.base_roll)
        self.robot.reset_base_pose(base_pos, base_orn)

        q_target = build_direct_pose(self.robot.dof_joint_names, support_leg)
        self.robot.reset_joint_positions(q_target)

    def _resolve_corner_positions(self, initial_support_contact: np.ndarray) -> list:
        """Resolve corner patch local positions from the robot model."""
        import mujoco

        local_positions: list = []
        for geom_id in range(self.robot.model.ngeom):
            if int(self.robot.model.geom_bodyid[geom_id]) != self.support_link:
                continue
            if int(self.robot.model.geom_type[geom_id]) != mujoco.mjtGeom.mjGEOM_SPHERE:
                continue
            local_positions.append(np.array(self.robot.model.geom_pos[geom_id], copy=True))
        if not local_positions:
            return []
        return sorted(local_positions, key=lambda pos: (pos[0], pos[1]))

    def _build_bounds(self) -> list[tuple[float, float]]:
        """Build optimization variable bounds from MuJoCo joint limits."""
        x0 = self.x0
        bounds: list[tuple[float, float]] = []

        # Base position: +/- 0.2 m around initial
        for i in range(3):
            bounds.append((float(x0[i] - 0.2), float(x0[i] + 0.2)))

        # Base RPY: +/- 0.3 rad around initial
        for i in range(3, 6):
            bounds.append((float(x0[i] - 0.3), float(x0[i] + 0.3)))

        # Joints: from MuJoCo jnt_range
        for joint_id in self.robot.dof_joints:
            low = float(self.robot.model.jnt_range[joint_id, 0])
            high = float(self.robot.model.jnt_range[joint_id, 1])
            bounds.append((low, high))

        return bounds

    @property
    def x0(self) -> np.ndarray:
        """Initial guess from the hand-tuned pose."""
        q, _ = self.robot.get_state()
        base_rpy = rpy_from_rotation_matrix(
            np.array(self.robot.data.xmat[self.robot.root_body_id]).reshape(3, 3)
        )
        x0 = np.zeros(6 + self.robot.num_joints, dtype=float)
        x0[:3] = q[:3]
        x0[3:6] = base_rpy
        x0[6:] = q[7:]
        return x0

    def _sync_and_evaluate(self, x: np.ndarray) -> dict:
        """Set state from x and compute all kinematic/dynamic quantities."""
        q = build_q_from_x(x, self.robot.nq)
        self.robot._sync_state(q=q, v=np.zeros(self.robot.nv))

        # Gravity torques (v=0 so only gravity)
        C = self.robot.compute_coriolis_gravity(q, np.zeros(self.robot.nv))
        tau_gravity = C[6:]

        # Whole-body CoM
        com = self.robot.compute_com_position()

        # Support foot pose
        support_pos = np.array(self.robot.data.xpos[self.support_link], copy=True)
        support_rpy = rpy_from_rotation_matrix(
            np.array(self.robot.data.xmat[self.support_link]).reshape(3, 3)
        )

        # Swing foot height
        swing_z = float(self.robot.data.xpos[self.swing_link, 2])

        # Corner patch world positions (for support polygon margin and height)
        corner_world_xy: list[np.ndarray] = []
        corner_world_z: list[float] = []
        if len(self.corner_local_positions) == 4:
            foot_origin = np.array(self.robot.data.xpos[self.support_link], copy=True)
            foot_rotation = np.array(self.robot.data.xmat[self.support_link]).reshape(3, 3)
            for local_pos in self.corner_local_positions:
                world_pos = foot_origin + foot_rotation @ np.asarray(local_pos, dtype=float)
                corner_world_xy.append(world_pos[:2])
                corner_world_z.append(float(world_pos[2]))

        return {
            "tau_gravity": tau_gravity,
            "com": com,
            "support_pos": support_pos,
            "support_rpy": support_rpy,
            "swing_z": swing_z,
            "corner_world_xy": corner_world_xy,
            "corner_world_z": corner_world_z,
        }

    def _compute_margin(self, com_xy: np.ndarray, corners_xy: list[np.ndarray]) -> float:
        """Minimum distance from CoM projection to support polygon (bounding box of corners)."""
        xs = [float(c[0]) for c in corners_xy]
        ys = [float(c[1]) for c in corners_xy]
        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)
        dx = min(com_xy[0] - x_min, x_max - com_xy[0])
        dy = min(com_xy[1] - y_min, y_max - com_xy[1])
        return float(min(dx, dy))

    def objective(self, x: np.ndarray) -> float:
        """Scalar cost: torque + CoM alignment + margin + orientation + regularization."""
        vals = self._sync_and_evaluate(x)
        w = self.weights
        cost = 0.0

        # 1. Minimize gravity-induced joint torques
        cost += w["tau"] * float(np.sum(vals["tau_gravity"] ** 2))

        # 2. Keep CoM horizontally above support foot
        com_xy = vals["com"][:2]
        support_xy = vals["support_pos"][:2]
        cost += w["com"] * float(np.sum((com_xy - support_xy) ** 2))

        # 3. Stay away from support polygon edges
        if vals["corner_world_xy"]:
            margin = self._compute_margin(com_xy, vals["corner_world_xy"])
            cost += w["margin"] * max(0.0, 0.02 - margin) ** 2

        # 4. Keep base roll/pitch small
        base_rpy = x[3:6]
        cost += w["base_ori"] * (base_rpy[0] ** 2 + base_rpy[1] ** 2)

        # 5. Stay close to nominal (hand-tuned) pose
        joints = x[6:]
        cost += w["reg"] * float(np.sum((joints - self.q_nominal) ** 2))

        # 6. Keep swing foot clear of ground
        swing_z = vals["swing_z"]
        cost += w["swing_clearance"] * max(0.0, 0.02 - swing_z) ** 2

        return cost

    def constraints_eq_support_foot(self, x: np.ndarray) -> np.ndarray:
        """6D equality: support foot body origin position + orientation fixed."""
        vals = self._sync_and_evaluate(x)
        pos_err = vals["support_pos"] - self.target_support_pos
        rpy_err = vals["support_rpy"] - self.target_support_rpy
        rpy_err = np.array(
            [
                math.atan2(math.sin(rpy_err[0]), math.cos(rpy_err[0])),
                math.atan2(math.sin(rpy_err[1]), math.cos(rpy_err[1])),
                math.atan2(math.sin(rpy_err[2]), math.cos(rpy_err[2])),
            ],
            dtype=float,
        )
        return np.hstack([pos_err, rpy_err])

    def constraints_ineq_com_margin(self, x: np.ndarray) -> np.ndarray:
        """4D inequality: CoM xy projection inside support polygon (soft margin)."""
        vals = self._sync_and_evaluate(x)
        com_xy = vals["com"][:2]
        # Allow a small negative margin (hand-tuned baseline is ~ -25 mm).
        # We enforce margin >= -15 mm as a hard floor, which is stricter than
        # the baseline but still physically reachable.
        margin = -0.015
        if len(vals["corner_world_xy"]) != 4:
            return np.ones(4)
        xs = [float(c[0]) for c in vals["corner_world_xy"]]
        ys = [float(c[1]) for c in vals["corner_world_xy"]]
        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)
        return np.array(
            [
                com_xy[0] - (x_min + margin),
                (x_max - margin) - com_xy[0],
                com_xy[1] - (y_min + margin),
                (y_max - margin) - com_xy[1],
            ],
            dtype=float,
        )

    def constraints_ineq_swing_clearance(self, x: np.ndarray) -> np.ndarray:
        """1D inequality: swing foot at least 5 cm above ground."""
        vals = self._sync_and_evaluate(x)
        return np.array([vals["swing_z"] - 0.05], dtype=float)

    def constraints_ineq_torque_margin(self, x: np.ndarray) -> np.ndarray:
        """23D inequality: joint gravity torques stay within 90% of limits."""
        vals = self._sync_and_evaluate(x)
        tau = vals["tau_gravity"]
        limit = 0.9 * self.robot.tau_limits
        return limit - np.abs(tau)

    def optimize(self, maxiter: int = 500) -> dict:
        """Run SLSQP and return pose_override dict."""
        x0 = self.x0
        print("=" * 60)
        print("Offline Static Pose Optimization")
        print("=" * 60)
        print(f"Variables: {len(x0)} (base 6D + {self.robot.num_joints} joints)")
        print(f"Equality constraints:   6 (support foot pos+rpy)")
        print(f"Inequality constraints: 28 (CoM margin=4, swing clearance=1, torque margin=23)")
        print(f"Initial objective:     {self.objective(x0):.6f}")
        print(f"Initial support eq L2: {np.linalg.norm(self.constraints_eq_support_foot(x0)):.6e}")
        print(f"Initial ineq min:      {min(self.constraints_ineq_com_margin(x0).min(), self.constraints_ineq_swing_clearance(x0).min(), self.constraints_ineq_torque_margin(x0).min()):.6e}")
        print("=" * 60)

        all_constraints = [
            {"type": "eq", "fun": self.constraints_eq_support_foot},
            {"type": "ineq", "fun": self.constraints_ineq_com_margin},
            {"type": "ineq", "fun": self.constraints_ineq_swing_clearance},
            {"type": "ineq", "fun": self.constraints_ineq_torque_margin},
        ]

        result = minimize(
            self.objective,
            x0,
            method="SLSQP",
            bounds=self.bounds,
            constraints=all_constraints,
            options={"ftol": 1e-6, "maxiter": maxiter, "disp": True, "eps": 1e-5},
        )

        print("\n" + "=" * 60)
        print("Optimization Result")
        print("=" * 60)
        print(f"Success:               {result.success}")
        print(f"Status:                {result.status}")
        print(f"Message:               {result.message}")
        print(f"Iterations:            {result.nit}")
        print(f"Function evaluations:  {result.nfev}")
        print(f"Final objective:       {result.fun:.6f}")
        print(f"Final support eq L2:   {np.linalg.norm(self.constraints_eq_support_foot(result.x)):.6e}")

        # Evaluate final pose details
        vals = self._sync_and_evaluate(result.x)
        print(f"Final CoM  xy:         [{vals['com'][0]:.4f}, {vals['com'][1]:.4f}]")
        print(f"Final support xy:      [{vals['support_pos'][0]:.4f}, {vals['support_pos'][1]:.4f}]")
        print(f"Final ||tau_g||:       {np.linalg.norm(vals['tau_gravity']):.4f}")
        print(f"Final swing foot z:    {vals['swing_z']:.4f}")
        if vals["corner_world_xy"]:
            margin = self._compute_margin(vals["com"][:2], vals["corner_world_xy"])
            print(f"Final support margin:  {margin:.4f}")
        print(f"Final ineq min (CoM):  {self.constraints_ineq_com_margin(result.x).min():.6e}")
        print(f"Final ineq min (swing):{self.constraints_ineq_swing_clearance(result.x).min():.6e}")
        print(f"Final ineq min (torque):{self.constraints_ineq_torque_margin(result.x).min():.6e}")
        print("=" * 60)

        # Build pose_override dict compatible with run_direct_single_support
        q_opt = build_q_from_x(result.x, self.robot.nq)
        pose_override = {
            "base_position": q_opt[:3].tolist(),
            "base_orientation": q_opt[3:7].tolist(),
            "joint_angles": q_opt[7:].tolist(),
            "joint_names": self.robot.dof_joint_names,
            "optimization_info": {
                "success": bool(result.success),
                "nit": int(result.nit),
                "fun": float(result.fun),
                "constraint_violation": float(np.linalg.norm(self.constraints_eq_support_foot(result.x))),
                "tau_gravity_norm": float(np.linalg.norm(vals["tau_gravity"])),
            },
        }
        return pose_override


def main() -> None:
    weights = {
        "tau": 1.0,
        "com": 100.0,
        "margin": 10.0,
        "base_ori": 1.0,
        "reg": 0.1,
        "swing_clearance": 1000.0,
    }

    env_cfg = cfg.env
    robot = RobotModel(env_cfg.model_path)
    robot.model.opt.gravity[:] = env_cfg.gravity

    optimizer = StaticPoseOptimizer(robot, env_cfg.support_foot_name, weights)
    pose_override = optimizer.optimize(maxiter=500)

    output_path = Path(__file__).parent.parent / "results" / "optimized_pose.json"
    with open(output_path, "w") as f:
        json.dump(pose_override, f, indent=2)

    print(f"\nOptimized pose saved to {output_path}")


if __name__ == "__main__":
    main()
