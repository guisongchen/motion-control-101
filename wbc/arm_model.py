from wbc.sim_config import SimConfig
import numpy as np


class ArmModel:
    def __init__(self, link_lengths: list, initial_q: np.ndarray):
        
        self.link_lengths = link_lengths
        self.ndof = len(self.link_lengths)
        assert self.ndof == len(initial_q), "Number of joint angles must match number of links"
        
        self.q_curr = initial_q.copy()


    def forward_kinematics(self) -> np.ndarray:
        x = 0
        y = 0
        theta = 0

        for i in range(self.ndof):
            theta += self.q_curr[i]
            x += self.link_lengths[i] * np.cos(theta)
            y += self.link_lengths[i] * np.sin(theta)

        return np.array([x, y])
    
    def joint_positions(self) -> np.ndarray:
        positions = []
        x = 0
        y = 0
        theta = 0

        for i in range(self.ndof):
            theta += self.q_curr[i]
            x += self.link_lengths[i] * np.cos(theta)
            y += self.link_lengths[i] * np.sin(theta)
            positions.append((x, y))

        return np.array(positions)
    
    def jacobian_ee(self) -> np.ndarray:
        J = np.zeros((2, self.ndof))
        x = 0
        y = 0
        theta = 0

        for i in range(self.ndof):
            theta += self.q_curr[i]
            x += self.link_lengths[i] * np.cos(theta)
            y += self.link_lengths[i] * np.sin(theta)

            J[0, i] = -self.link_lengths[i] * np.sin(theta)
            J[1, i] = self.link_lengths[i] * np.cos(theta)

        return J
    
    def jacobian_end_to_elbow(self) -> np.ndarray:
        J = np.array([[0.0, 1.0 , 0.0]], dtype=float)
        return J
    
    def solve_ik_nullspace(
        self, 
        x_target: np.ndarray, 
        xdot_target: np.ndarray,
        config: SimConfig
    ) -> np.ndarray:
        
        # Compute current end-effector position and Jacobian at q_curr
        x_actual = self.forward_kinematics()
        J_ee = self.jacobian_ee()
        J_elbow = self.jacobian_end_to_elbow()

        # xdot_star: desired end-effector velocity with primary task PD control
        xdot_star = xdot_target + config.kp_primary * (x_target - x_actual) # PD control in task space
        
        # elbow_dot_star: desired elbow joint velocity with secondary task PD control
        elbow_dot_star = config.kp_secondary * (config.elbow_target - self.q_curr[1]) # PD control for elbow angle

        J_ee_pinv = np.linalg.pinv(J_ee, rcond=config.singularity_threshold)
        
        # Primary task: minimize end-effector velocity error
        qdot_primary = J_ee_pinv @ xdot_star
        
        # Secondary task: minimize elbow angle error while being in the nullspace of the primary task
        nullspace = np.eye(self.ndof) - J_ee_pinv @ J_ee
        residual_elbow_dot = elbow_dot_star - (self.jacobian_end_to_elbow() @ qdot_primary).item()

        projected_row = J_elbow @ nullspace