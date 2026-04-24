from wbc.arm_model import ArmModel
from wbc.sim_config import SimConfig

import matplotlib
from pathlib import Path
import numpy as np

matplotlib.use("Agg")  # Use non-interactive backend for plotting
import matplotlib.pyplot as plt

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

def plot_trajectories(
    output_path: Path,
    results: dict[str, dict[str, np.ndarray | str]],
    arm: ArmModel,
) -> None:
    result_count = len(results)
    figure, axes = plt.subplots(1, result_count, figsize=(6 * result_count, 5), constrained_layout=True)
    if result_count == 1:
        axes = [axes]
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
    colors = ["tab:blue", "tab:orange", "tab:green", "tab:red"]

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

class Simulator:
    def __init__(self, sim_config: SimConfig):
        self.config = sim_config

    def target_reference(self) -> tuple[np.ndarray, np.ndarray]:
        t = np.arange(0, self.config.duration + self.config.dt, self.config.dt)
        omega = self.config.omega
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
    
    def run(self, 
            arm: ArmModel, 
            solver_name: str, 
            weights: tuple[float, float] | None = None
        ):

        # x_target: (N, 2) array of desired end-effector positions
        # xdot_target: (N, 2) array of desired end-effector velocities
        x_target, xdot_target = self.target_reference()

        step_count = len(x_target)

        for i in range(step_count-1):

            if solver_name == "nullspace":
                arm.solve_ik_nullspace(
                    x_target=x_target[i], 
                    xdot_target=xdot_target[i], 
                    config=self.config
                )
            elif solver_name == "damped_ls":
                arm.solve_weighted_damped_ls(
                    x_target=x_target[i], 
                    xdot_target=xdot_target[i], 
                    config=self.config,
                    weights=weights
                )
            elif solver_name == "qp":
                arm.solve_ik_qp(
                    x_target=x_target[i], 
                    xdot_target=xdot_target[i], 
                    config=self.config,
                    weights=weights
                )
            else:
                raise ValueError(f"Unknown solver name: {solver_name}")

        q = np.array(arm.q_history)
        qdot = np.array(arm.q_dot_history)
        x_actual = np.array(arm.x_history)
        tracking_error = np.linalg.norm(x_target - x_actual, axis=1)
        elbow_actual = q[:, 1]
        step_count = len(q)
        elbow_target = np.full(step_count, self.config.elbow_target)

        return {
            "q": q,
            "qdot": qdot,
            "ee_desired": x_target,
            "ee_actual": x_actual,
            "tracking_error": tracking_error,
            "elbow_actual": elbow_actual,
            "elbow_desired": elbow_target,
            "time": np.arange(0, step_count * self.config.dt, self.config.dt),
        }
    
if __name__ == "__main__":
    sim_config = SimConfig()
    simulator = Simulator(sim_config)

    output_dir = Path("output")
    output_dir.mkdir(exist_ok=True)

    link_lengths = np.array([1.0, 1.0, 0.8], dtype=float)

    results_dict = {
        "Nullspace IK": simulator.run(
            ArmModel(link_lengths, sim_config.q0), solver_name="nullspace"
        ),
        "DLS(100, 1)": simulator.run(
            ArmModel(link_lengths, sim_config.q0),
            solver_name="damped_ls",
            weights=(100.0, 1.0),
        ),
        "DLS(10, 10)": simulator.run(
            ArmModel(link_lengths, sim_config.q0),
            solver_name="damped_ls",
            weights=(10.0, 10.0),
        ),
        "QP(1, 1)": simulator.run(
            ArmModel(link_lengths, sim_config.q0),
            solver_name="qp",
            weights=(1.0, 1.0),
        ),
    }

    arm = ArmModel(link_lengths, sim_config.q0)
    plot_trajectories(output_dir / "trajectories.png", results_dict, arm)
    plot_tracking_error(output_dir / "tracking_error.png", results_dict)
    plot_elbow_angle(output_dir / "elbow_angle.png", results_dict)
    plot_joint_velocities(output_dir / "joint_velocities.png", results_dict)
    print(f"Plots saved to {output_dir}/")
    print(summarize_results(results_dict))  