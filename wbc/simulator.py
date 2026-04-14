from wbc.arm_model import ArmModel
from wbc.sim_config import SimConfig

import numpy as np

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
    
    def run(self, solver_name: str):
        link_lengths = np.array([1.0, 1.0, 0.8], dtype=float)
        arm = ArmModel(link_lengths, self.config.q0)

        # x_target: (N, 2) array of desired end-effector positions
        # xdot_target: (N, 2) array of desired end-effector velocities
        x_target, xdot_target = self.target_reference()

        step_count = len(x_target)

        for i in range(step_count-1):

            if solver_name == "nullspace":
                current_q_dot = arm.solve_ik_nullspace(
                    x_target=x_target[i], 
                    xdot_target=xdot_target[i], 
                    config=self.config
                )


        pass