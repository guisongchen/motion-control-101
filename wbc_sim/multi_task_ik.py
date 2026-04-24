from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt


@dataclass(frozen=True)
class ArmModel:
    link_lengths: np.ndarray


@dataclass(frozen=True)
class SimulationConfig:
    dt: float = 0.005
    duration: float = 2.0
    omega: float = 2.0 * np.pi
    kp_primary: float = 5.0
    kp_secondary: float = 2.0
    elbow_target: float = 0.5
    q0: np.ndarray = field(
        default_factory=lambda: np.array([0.3, 0.5, 0.2], dtype=float)
    )
    singularity_threshold: float = 1e-6
    pseudo_damping: float = 1e-4
    wln_damping: float = 0.05


def parse_vector3(text: str) -> np.ndarray:
    values = np.fromstring(text, sep=",", dtype=float)
    if values.shape != (3,):
        raise argparse.ArgumentTypeError("Expected three comma-separated values, e.g. 0.3,0.5,0.2")
    return values


def circular_reference(t: np.ndarray, omega: float) -> tuple[np.ndarray, np.ndarray]:
    positions = np.column_stack(
        (
            2.0 + 0.5 * np.cos(omega * t),
            1.0 + 0.5 * np.sin(omega * t),
        )
    )
    velocities = np.column_stack(
        (
            -0.5 * omega * np.sin(omega * t),
            0.5 * omega * np.cos(omega * t),
        )
    )
    return positions, velocities


def forward_kinematics(q: np.ndarray, arm: ArmModel) -> np.ndarray:
    theta1, theta2, theta3 = q
    l1, l2, l3 = arm.link_lengths
    theta12 = theta1 + theta2
    theta123 = theta12 + theta3
    return np.array(
        [
            l1 * np.cos(theta1) + l2 * np.cos(theta12) + l3 * np.cos(theta123),
            l1 * np.sin(theta1) + l2 * np.sin(theta12) + l3 * np.sin(theta123),
        ],
        dtype=float,
    )


def joint_positions(q: np.ndarray, arm: ArmModel) -> np.ndarray:
    theta1, theta2, theta3 = q
    l1, l2, l3 = arm.link_lengths
    theta12 = theta1 + theta2
    theta123 = theta12 + theta3

    points = np.array(
        [
            [0.0, 0.0],
            [l1 * np.cos(theta1), l1 * np.sin(theta1)],
            [
                l1 * np.cos(theta1) + l2 * np.cos(theta12),
                l1 * np.sin(theta1) + l2 * np.sin(theta12),
            ],
            [
                l1 * np.cos(theta1) + l2 * np.cos(theta12) + l3 * np.cos(theta123),
                l1 * np.sin(theta1) + l2 * np.sin(theta12) + l3 * np.sin(theta123),
            ],
        ],
        dtype=float,
    )
    return points


def jacobian_ee(q: np.ndarray, arm: ArmModel) -> np.ndarray:
    theta1, theta2, theta3 = q
    l1, l2, l3 = arm.link_lengths
    theta12 = theta1 + theta2
    theta123 = theta12 + theta3

    sin1, cos1 = np.sin(theta1), np.cos(theta1)
    sin12, cos12 = np.sin(theta12), np.cos(theta12)
    sin123, cos123 = np.sin(theta123), np.cos(theta123)

    return np.array(
        [
            [
                -l1 * sin1 - l2 * sin12 - l3 * sin123,
                -l2 * sin12 - l3 * sin123,
                -l3 * sin123,
            ],
            [
                l1 * cos1 + l2 * cos12 + l3 * cos123,
                l2 * cos12 + l3 * cos123,
                l3 * cos123,
            ],
        ],
        dtype=float,
    )


def damped_right_pseudoinverse(
    jacobian: np.ndarray,
    damping: float,
    singularity_threshold: float,
) -> np.ndarray:
    jj_t = jacobian @ jacobian.T
    determinant = np.linalg.det(jj_t)
    effective_damping = damping if determinant < singularity_threshold else 0.0
    regularized = jj_t + (effective_damping**2) * np.eye(jj_t.shape[0])
    return jacobian.T @ np.linalg.inv(regularized)


def row_pseudoinverse(row: np.ndarray, damping: float = 0.0) -> np.ndarray:
    denominator = (row @ row.T).item() + damping**2
    if denominator <= 1e-12:
        return np.zeros((row.shape[1], 1), dtype=float)
    return row.T / denominator


def solve_nullspace_priority(
    q: np.ndarray,
    x_des: np.ndarray,
    xdot_des: np.ndarray,
    arm: ArmModel,
    config: SimulationConfig,
) -> np.ndarray:
    x_actual = forward_kinematics(q, arm)
    jacobian = jacobian_ee(q, arm)
    elbow_jacobian = np.array([[0.0, 1.0, 0.0]], dtype=float)

    xdot_star = xdot_des + config.kp_primary * (x_des - x_actual)
    elbow_dot_star = config.kp_secondary * (config.elbow_target - q[1])

    jacobian_pinv = damped_right_pseudoinverse(
        jacobian,
        damping=config.pseudo_damping,
        singularity_threshold=config.singularity_threshold,
    )
    qdot_primary = jacobian_pinv @ xdot_star

    nullspace = np.eye(3) - jacobian_pinv @ jacobian
    residual = elbow_dot_star - (elbow_jacobian @ qdot_primary).item()
    projected_row = elbow_jacobian @ nullspace
    projected_row_pinv = row_pseudoinverse(projected_row, damping=config.pseudo_damping)
    qdot_secondary = (nullspace @ projected_row_pinv).reshape(-1) * residual
    return qdot_primary + qdot_secondary


def solve_weighted_damped_ls(
    q: np.ndarray,
    x_des: np.ndarray,
    xdot_des: np.ndarray,
    weights: tuple[float, float],
    arm: ArmModel,
    config: SimulationConfig,
) -> np.ndarray:
    x_actual = forward_kinematics(q, arm)
    jacobian = jacobian_ee(q, arm)
    elbow_jacobian = np.array([[0.0, 1.0, 0.0]], dtype=float)

    xdot_star = xdot_des + config.kp_primary * (x_des - x_actual)
    elbow_dot_star = config.kp_secondary * (config.elbow_target - q[1])

    stacked_jacobian = np.vstack((jacobian, elbow_jacobian))
    task_velocity = np.concatenate((xdot_star, np.array([elbow_dot_star], dtype=float)))

    w_ee, w_elbow = weights
    weight_matrix = np.diag([w_ee, w_ee, w_elbow])
    lhs = (
        stacked_jacobian.T @ weight_matrix @ stacked_jacobian
        + (config.wln_damping**2) * np.eye(3)
    )
    rhs = stacked_jacobian.T @ weight_matrix @ task_velocity
    return np.linalg.solve(lhs, rhs)


def run_simulation(
    solver_name: str,
    arm: ArmModel,
    config: SimulationConfig,
    weights: tuple[float, float] | None = None,
) -> dict[str, np.ndarray | str]:
    time = np.arange(0.0, config.duration + config.dt, config.dt)
    desired_positions, desired_velocities = circular_reference(time, config.omega)

    q = np.zeros((time.size, 3), dtype=float)
    qdot = np.zeros((time.size, 3), dtype=float)
    ee_actual = np.zeros((time.size, 2), dtype=float)
    tracking_error = np.zeros(time.size, dtype=float)

    q[0] = config.q0
    ee_actual[0] = forward_kinematics(q[0], arm)
    tracking_error[0] = np.linalg.norm(desired_positions[0] - ee_actual[0])

    for index in range(time.size - 1):
        current_q = q[index]
        if solver_name == "nullspace":
            current_qdot = solve_nullspace_priority(
                current_q,
                desired_positions[index],
                desired_velocities[index],
                arm,
                config,
            )
        elif solver_name == "wln":
            assert weights is not None
            current_qdot = solve_weighted_damped_ls(
                current_q,
                desired_positions[index],
                desired_velocities[index],
                weights,
                arm,
                config,
            )
        else:
            raise ValueError(f"Unsupported solver: {solver_name}")

        qdot[index] = current_qdot
        q[index + 1] = current_q + config.dt * current_qdot
        ee_actual[index + 1] = forward_kinematics(q[index + 1], arm)
        tracking_error[index + 1] = np.linalg.norm(
            desired_positions[index + 1] - ee_actual[index + 1]
        )

    qdot[-1] = qdot[-2]

    return {
        "time": time,
        "q": q,
        "qdot": qdot,
        "ee_actual": ee_actual,
        "ee_desired": desired_positions,
        "tracking_error": tracking_error,
        "elbow_actual": q[:, 1],
        "elbow_desired": np.full(time.shape, config.elbow_target, dtype=float),
    }


def export_csv(path: Path, result: dict[str, np.ndarray | str]) -> None:
    q = np.asarray(result["q"])
    qdot = np.asarray(result["qdot"])
    ee_actual = np.asarray(result["ee_actual"])
    ee_desired = np.asarray(result["ee_desired"])
    tracking_error = np.asarray(result["tracking_error"])
    elbow_actual = np.asarray(result["elbow_actual"])
    elbow_desired = np.asarray(result["elbow_desired"])

    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "time",
                "theta1",
                "theta2",
                "theta3",
                "theta1_dot",
                "theta2_dot",
                "theta3_dot",
                "x_actual",
                "y_actual",
                "x_desired",
                "y_desired",
                "tracking_error",
                "elbow_actual",
                "elbow_desired",
            ]
        )
        for index, t in enumerate(np.asarray(result["time"])):
            writer.writerow(
                [
                    t,
                    q[index, 0],
                    q[index, 1],
                    q[index, 2],
                    qdot[index, 0],
                    qdot[index, 1],
                    qdot[index, 2],
                    ee_actual[index, 0],
                    ee_actual[index, 1],
                    ee_desired[index, 0],
                    ee_desired[index, 1],
                    tracking_error[index],
                    elbow_actual[index],
                    elbow_desired[index],
                ]
            )


def plot_trajectories(
    output_path: Path,
    results: dict[str, dict[str, np.ndarray | str]],
    arm: ArmModel,
) -> None:
    figure, axes = plt.subplots(1, 3, figsize=(18, 5), constrained_layout=True)
    keyframe_count = 6

    for axis, (name, result) in zip(axes, results.items(), strict=True):
        ee_desired = np.asarray(result["ee_desired"])
        ee_actual = np.asarray(result["ee_actual"])
        q = np.asarray(result["q"])

        axis.plot(ee_desired[:, 0], ee_desired[:, 1], "--", color="black", label="desired circle")
        axis.plot(ee_actual[:, 0], ee_actual[:, 1], color="tab:blue", label="actual EE path")

        sample_indices = np.linspace(0, q.shape[0] - 1, keyframe_count, dtype=int)
        for sample_index in sample_indices:
            points = joint_positions(q[sample_index], arm)
            axis.plot(points[:, 0], points[:, 1], color="tab:orange", alpha=0.35)
            axis.scatter(points[:, 0], points[:, 1], color="tab:orange", s=12, alpha=0.35)

        axis.set_title(name)
        axis.set_xlabel("x")
        axis.set_ylabel("y")
        axis.set_aspect("equal", adjustable="box")
        axis.grid(True, linestyle=":")
        axis.legend(loc="upper right")

    figure.suptitle("2D three-link arm trajectories and key frames")
    figure.savefig(output_path, dpi=180)
    plt.close(figure)


def plot_tracking_error(
    output_path: Path,
    results: dict[str, dict[str, np.ndarray | str]],
) -> None:
    figure, axis = plt.subplots(figsize=(10, 5), constrained_layout=True)
    for name, result in results.items():
        axis.plot(
            np.asarray(result["time"]),
            np.asarray(result["tracking_error"]),
            label=name,
        )
    axis.set_title("End-effector tracking error")
    axis.set_xlabel("time [s]")
    axis.set_ylabel(r"$||x_{des} - x_{act}||$")
    axis.grid(True, linestyle=":")
    axis.legend()
    figure.savefig(output_path, dpi=180)
    plt.close(figure)


def plot_elbow_angle(
    output_path: Path,
    results: dict[str, dict[str, np.ndarray | str]],
) -> None:
    figure, axis = plt.subplots(figsize=(10, 5), constrained_layout=True)
    for name, result in results.items():
        axis.plot(
            np.asarray(result["time"]),
            np.asarray(result["elbow_actual"]),
            label=name,
        )
    axis.plot(
        np.asarray(next(iter(results.values()))["time"]),
        np.asarray(next(iter(results.values()))["elbow_desired"]),
        "--",
        color="black",
        label="elbow target",
    )
    axis.set_title("Elbow joint angle")
    axis.set_xlabel("time [s]")
    axis.set_ylabel(r"$\theta_2$ [rad]")
    axis.grid(True, linestyle=":")
    axis.legend()
    figure.savefig(output_path, dpi=180)
    plt.close(figure)


def plot_joint_velocities(
    output_path: Path,
    results: dict[str, dict[str, np.ndarray | str]],
) -> None:
    figure, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True, constrained_layout=True)
    colors = ["tab:blue", "tab:orange", "tab:green"]

    for joint_index, axis in enumerate(axes):
        for color, (name, result) in zip(colors, results.items()):
            axis.plot(
                np.asarray(result["time"]),
                np.asarray(result["qdot"])[:, joint_index],
                label=name,
                color=color,
            )
        axis.set_ylabel(rf"$\dot{{\theta}}_{joint_index + 1}$ [rad/s]")
        axis.grid(True, linestyle=":")
        axis.legend()

    axes[-1].set_xlabel("time [s]")
    figure.suptitle("Joint velocities")
    figure.savefig(output_path, dpi=180)
    plt.close(figure)


def summarize_results(results: dict[str, dict[str, np.ndarray | str]]) -> str:
    lines = []
    for name, result in results.items():
        tracking_error = np.asarray(result["tracking_error"])
        elbow_error = np.abs(
            np.asarray(result["elbow_actual"]) - np.asarray(result["elbow_desired"])
        )
        peak_speed = np.max(np.abs(np.asarray(result["qdot"])))
        lines.append(
            (
                f"{name}: mean tracking error={tracking_error.mean():.4f}, "
                f"max tracking error={tracking_error.max():.4f}, "
                f"mean elbow error={elbow_error.mean():.4f}, "
                f"peak |qdot|={peak_speed:.4f}"
            )
        )
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="2D three-link multi-task IK simulation for strict and soft priorities."
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("wbc") / "outputs",
        help="Directory used to store figures and CSV logs.",
    )
    parser.add_argument(
        "--q0",
        type=parse_vector3,
        default=parse_vector3("0.3,0.5,0.2"),
        help="Initial joint configuration as theta1,theta2,theta3 in radians.",
    )
    parser.add_argument(
        "--elbow-target",
        type=float,
        default=0.5,
        help="Desired elbow joint angle in radians.",
    )
    parser.add_argument(
        "--omega",
        type=float,
        default=2.0 * np.pi,
        help="Reference circle angular speed in rad/s.",
    )
    parser.add_argument(
        "--wln-damping",
        type=float,
        default=0.05,
        help="Damping factor used by the weighted least-squares solver.",
    )
    args = parser.parse_args()

    arm = ArmModel(link_lengths=np.array([1.0, 1.0, 0.8], dtype=float))
    config = SimulationConfig(
        q0=args.q0,
        elbow_target=args.elbow_target,
        omega=args.omega,
        wln_damping=args.wln_damping,
    )

    args.output_dir.mkdir(parents=True, exist_ok=True)

    results = {
        "Null-space priority": run_simulation("nullspace", arm, config),
        "WLN (w_ee=100, w_elbow=1)": run_simulation(
            "wln",
            arm,
            config,
            weights=(100.0, 1.0),
        ),
        "WLN (w_ee=10, w_elbow=10)": run_simulation(
            "wln",
            arm,
            config,
            weights=(10.0, 10.0),
        ),
    }

    for name, result in results.items():
        slug = (
            name.lower()
            .replace(" ", "_")
            .replace("(", "")
            .replace(")", "")
            .replace(",", "")
            .replace("=", "-")
        )
        export_csv(args.output_dir / f"{slug}.csv", result)

    plot_trajectories(args.output_dir / "trajectory_comparison.png", results, arm)
    plot_tracking_error(args.output_dir / "tracking_error.png", results)
    plot_elbow_angle(args.output_dir / "elbow_angle.png", results)
    plot_joint_velocities(args.output_dir / "joint_velocities.png", results)

    print(f"Saved outputs to: {args.output_dir}")
    print(summarize_results(results))

if __name__ == "__main__":
    main()
