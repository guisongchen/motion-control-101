from wbc.sim_config import SimConfig
import numpy as np


class ArmModel:
    def __init__(self, link_lengths: list, initial_q: np.ndarray):
        
        self.link_lengths = link_lengths
        self.ndof = len(self.link_lengths)
        assert self.ndof == len(initial_q), "Number of joint angles must match number of links"
        
        self.q_curr = initial_q.copy()
        self.q_dot_curr = np.zeros_like(self.q_curr)

        self.q_history = [self.q_curr.copy()]
        self.q_dot_history = [self.q_dot_curr.copy()]
        self.x_history = [self.forward_kinematics()]


    def forward_kinematics(self) -> np.ndarray:
        x = 0
        y = 0
        theta = 0

        for i in range(self.ndof):
            theta += self.q_curr[i]
            x += self.link_lengths[i] * np.cos(theta)
            y += self.link_lengths[i] * np.sin(theta)

        return np.array([x, y])
    
    def jacobian_ee(self) -> np.ndarray:
        J = np.zeros((2, self.ndof))
        theta = 0
        thetas = []

        for i in range(self.ndof):
            theta += self.q_curr[i]
            thetas.append(theta)

        cumsum_x = 0.0
        cumsum_y = 0.0
        for i in range(self.ndof - 1, -1, -1):
            cumsum_x += self.link_lengths[i] * np.cos(thetas[i])
            cumsum_y += self.link_lengths[i] * np.sin(thetas[i])
            J[0, i] = -cumsum_y
            J[1, i] =  cumsum_x

        return J
    
    def jacobian_end_to_elbow(self) -> np.ndarray:
        J = np.array([[0.0, 1.0 , 0.0]], dtype=float)
        return J
    
    def solve_ik_nullspace(
        self, 
        x_target: np.ndarray, 
        xdot_target: np.ndarray,
        config: SimConfig
    ):
        
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
        nullspace_project = np.eye(self.ndof) - J_ee_pinv @ J_ee
        
        # Compute the residual for the secondary task (elbow velocity error after applying primary task)
        # which means primary task may not fully achieve the desired elbow velocity, 
        # so we compute how much is left to achieve the secondary task
        residual_elbow_dot = elbow_dot_star - (J_elbow @ qdot_primary).item()

        # Project the secondary task Jacobian into the nullspace of the primary task 
        # to find how much we can still achieve for the elbow velocity without affecting the end-effector velocity
        # why right multiply by nullspace_project? 
        # because we want to see how much of the elbow velocity can be achieved in the nullspace of the primary task
        # if we left multiply by nullspace_project, 
        # we would be projecting the entire Jacobian into the nullspace, which is not what we want.
        projected_row = J_elbow @ nullspace_project
        projected_row_pinv = np.linalg.pinv(projected_row, rcond=config.singularity_threshold)

        qdot_secondary = (projected_row_pinv * residual_elbow_dot).flatten()
        # Total joint velocity command is the sum of primary and secondary tasks
        qdot_total = qdot_primary + nullspace_project @ qdot_secondary

        self.q_dot_curr = qdot_total.copy()
        self.q_curr += self.q_dot_curr * config.dt

        self.q_history.append(self.q_curr.copy())
        self.q_dot_history.append(self.q_dot_curr.copy())
        self.x_history.append(self.forward_kinematics())