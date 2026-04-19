"""MuJoCo robot model wrapper for kinematics, dynamics, and contact queries."""

from pathlib import Path
from typing import Optional

import mujoco
import numpy as np


def _quat_xyzw_to_wxyz(quat_xyzw: np.ndarray) -> np.ndarray:
    """Convert external [x, y, z, w] quaternion ordering to MuJoCo [w, x, y, z]."""
    quat_xyzw = np.asarray(quat_xyzw, dtype=float)
    return np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])


def _quat_wxyz_to_xyzw(quat_wxyz: np.ndarray) -> np.ndarray:
    """Convert MuJoCo [w, x, y, z] quaternion ordering to external [x, y, z, w]."""
    quat_wxyz = np.asarray(quat_wxyz, dtype=float)
    return np.array([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]])


class RobotModel:
    """Encapsulate MuJoCo robot model access behind the existing MPC/WBC interface."""

    def __init__(
        self,
        model_path: str,
        base_position: Optional[np.ndarray] = None,
        base_orientation: Optional[np.ndarray] = None,
        use_fixed_base: bool = False,
    ):
        if use_fixed_base:
            raise ValueError("MuJoCo G1 model is configured with a floating base only.")

        self.model_path = str(Path(model_path).expanduser())
        self.model = mujoco.MjModel.from_xml_path(self.model_path)
        self.data = mujoco.MjData(self.model)

        if base_position is None:
            base_position = np.array([0.0, 0.0, 1.0])
        if base_orientation is None:
            base_orientation = np.array([0.0, 0.0, 0.0, 1.0])

        self.root_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "pelvis"
        )
        self.link_name_to_index = {
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body_id): body_id
            for body_id in range(1, self.model.nbody)
        }
        self.link_name_to_index["base"] = self.root_body_id

        self.dof_joints = [
            joint_id
            for joint_id in range(self.model.njnt)
            if self.model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_FREE
        ]
        self.dof_joint_names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            for joint_id in self.dof_joints
        ]
        self.dof_joint_name_to_index = {
            name: idx for idx, name in enumerate(self.dof_joint_names)
        }
        self.num_joints = len(self.dof_joints)
        self.nv = int(self.model.nv)
        self.nq = 7 + self.num_joints

        self.joint_to_actuator_id = {}
        for actuator_id in range(self.model.nu):
            joint_id = int(self.model.actuator_trnid[actuator_id, 0])
            self.joint_to_actuator_id[joint_id] = actuator_id

        self.tau_limits = np.array(
            [
                abs(float(self.model.jnt_actfrcrange[joint_id, 1]))
                if self.model.jnt_actfrclimited[joint_id]
                else np.inf
                for joint_id in self.dof_joints
            ]
        )
        self._total_mass = float(np.sum(self.model.body_mass[1:]))
        if self._total_mass <= 0.0:
            raise ValueError(
                f"Robot total mass must be positive, got {self._total_mass}. "
                "Check MuJoCo inertial definitions."
            )

        self.reset_base_pose(np.asarray(base_position, dtype=float), np.asarray(base_orientation, dtype=float))

    @property
    def total_mass(self) -> float:
        """Robot total mass [kg]."""
        return self._total_mass

    def _sync_state(self, q: Optional[np.ndarray] = None, v: Optional[np.ndarray] = None) -> None:
        if q is not None:
            q = np.asarray(q, dtype=float)
            quat = _quat_xyzw_to_wxyz(q[3:7])
            quat_norm = np.linalg.norm(quat)
            if quat_norm == 0.0:
                raise ValueError("Base quaternion must be non-zero.")
            quat /= quat_norm

            self.data.qpos[:3] = q[:3]
            self.data.qpos[3:7] = quat
            self.data.qpos[7:] = q[7:]

        if v is not None:
            self.data.qvel[:] = np.asarray(v, dtype=float)

        mujoco.mj_forward(self.model, self.data)

    def _current_q(self) -> np.ndarray:
        q = np.zeros(self.nq)
        q[:3] = self.data.qpos[:3]
        q[3:7] = _quat_wxyz_to_xyzw(self.data.qpos[3:7])
        q[7:] = self.data.qpos[7:]
        return q

    def _current_v(self) -> np.ndarray:
        return np.array(self.data.qvel, copy=True)

    def _body_rotation(self, body_id: int) -> np.ndarray:
        return np.array(self.data.xmat[body_id]).reshape(3, 3)

    def _restore_state(self, qpos: np.ndarray, qvel: np.ndarray) -> None:
        self.data.qpos[:] = qpos
        self.data.qvel[:] = qvel
        mujoco.mj_forward(self.model, self.data)

    def _centroidal_jacobian_from_unit_velocities(
        self, q: np.ndarray, quantity: str
    ) -> np.ndarray:
        self._sync_state(q=q, v=np.zeros(self.nv))
        saved_qpos = np.array(self.data.qpos, copy=True)
        saved_qvel = np.array(self.data.qvel, copy=True)

        J = np.zeros((3, self.nv))
        for dof in range(self.nv):
            self.data.qvel[:] = 0.0
            self.data.qvel[dof] = 1.0
            mujoco.mj_forward(self.model, self.data)
            mujoco.mj_subtreeVel(self.model, self.data)
            if quantity == "com":
                J[:, dof] = self.data.subtree_linvel[self.root_body_id]
            elif quantity == "angular_momentum":
                J[:, dof] = self.data.subtree_angmom[self.root_body_id]
            else:
                self._restore_state(saved_qpos, saved_qvel)
                raise ValueError(f"Unsupported centroidal quantity: {quantity}")

        self._restore_state(saved_qpos, saved_qvel)
        return J

    def get_state(self) -> tuple[np.ndarray, np.ndarray]:
        """Return current state (q, v)."""
        return self._current_q(), self._current_v()

    def compute_mass_matrix(self, q: np.ndarray) -> np.ndarray:
        """Compute mass matrix M(q), shape (nv, nv)."""
        self._sync_state(q=q)
        M = np.zeros((self.nv, self.nv))
        mujoco.mj_fullM(self.model, M, self.data.qM)
        return M

    def compute_coriolis_gravity(self, q: np.ndarray, v: np.ndarray) -> np.ndarray:
        """Compute bias forces C(q, v), shape (nv,)."""
        self._sync_state(q=q, v=v)
        return np.array(self.data.qfrc_bias, copy=True)

    def compute_com_position(self, q: np.ndarray | None = None) -> np.ndarray:
        """Compute whole-body center-of-mass position, shape (3,)."""
        if q is not None:
            self._sync_state(q=q)
        return np.array(self.data.subtree_com[self.root_body_id], copy=True)

    def compute_com_velocity(self, q: np.ndarray, v: np.ndarray) -> np.ndarray:
        """Compute whole-body center-of-mass velocity, shape (3,)."""
        self._sync_state(q=q, v=v)
        mujoco.mj_subtreeVel(self.model, self.data)
        return np.array(self.data.subtree_linvel[self.root_body_id], copy=True)

    def compute_centroidal_momentum(self, q: np.ndarray, v: np.ndarray) -> np.ndarray:
        """Compute centroidal angular momentum L, shape (3,)."""
        self._sync_state(q=q, v=v)
        mujoco.mj_subtreeVel(self.model, self.data)
        return np.array(self.data.subtree_angmom[self.root_body_id], copy=True)

    def get_foot_jacobian(
        self,
        foot_link: int,
        q: np.ndarray,
        local_position: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """Compute 6D foot Jacobian at the requested body point, shape (6, nv)."""
        self._sync_state(q=q)
        if local_position is None:
            local_position = np.zeros(3)
        else:
            local_position = np.asarray(local_position, dtype=float)

        body_origin = np.array(self.data.xpos[foot_link], copy=True)
        world_point = body_origin + self._body_rotation(foot_link) @ local_position

        jacp = np.zeros((3, self.nv))
        jacr = np.zeros((3, self.nv))
        mujoco.mj_jac(self.model, self.data, jacp, jacr, world_point, foot_link)
        return np.vstack([jacp, jacr])

    def get_com_jacobian(self, q: np.ndarray) -> np.ndarray:
        """Compute center-of-mass Jacobian, shape (3, nv)."""
        return self._centroidal_jacobian_from_unit_velocities(q, quantity="com")

    def get_angular_momentum_jacobian(self, q: np.ndarray) -> np.ndarray:
        """Compute centroidal angular-momentum Jacobian, shape (3, nv)."""
        return self._centroidal_jacobian_from_unit_velocities(
            q, quantity="angular_momentum"
        )

    def get_link_com_position(self, link_index: int) -> np.ndarray:
        """Get world-frame body CoM position, shape (3,)."""
        return np.array(self.data.xipos[link_index], copy=True)

    def get_link_velocity(self, link_index: int) -> tuple[np.ndarray, np.ndarray]:
        """Get world-frame body CoM linear and angular velocity."""
        jacp = np.zeros((3, self.nv))
        jacr = np.zeros((3, self.nv))
        mujoco.mj_jacBodyCom(self.model, self.data, jacp, jacr, link_index)
        lin_vel = jacp @ self.data.qvel
        ang_vel = jacr @ self.data.qvel
        return lin_vel, ang_vel

    def reset_joint_positions(self, joint_positions: np.ndarray) -> None:
        """Reset actuated joint positions in DOF order and zero their velocities."""
        joint_positions = np.asarray(joint_positions, dtype=float)
        for dof_idx, joint_id in enumerate(self.dof_joints):
            if dof_idx >= len(joint_positions):
                break
            qpos_adr = int(self.model.jnt_qposadr[joint_id])
            qvel_adr = int(self.model.jnt_dofadr[joint_id])
            self.data.qpos[qpos_adr] = float(joint_positions[dof_idx])
            self.data.qvel[qvel_adr] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def reset_base_pose(self, position: np.ndarray, orientation: np.ndarray) -> None:
        """Reset floating-base pose and clear base velocity."""
        quat = _quat_xyzw_to_wxyz(np.asarray(orientation, dtype=float))
        quat /= np.linalg.norm(quat)
        self.data.qpos[:3] = np.asarray(position, dtype=float)
        self.data.qpos[3:7] = quat
        self.data.qvel[:6] = 0.0
        self.data.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def set_joint_torques(self, torques: np.ndarray) -> None:
        """Apply actuated-joint torques in DOF order."""
        torques = np.asarray(torques, dtype=float)
        self.data.ctrl[:] = 0.0
        for dof_idx, joint_id in enumerate(self.dof_joints):
            actuator_id = self.joint_to_actuator_id.get(joint_id)
            if actuator_id is None or dof_idx >= len(torques):
                continue
            torque = float(np.clip(torques[dof_idx], -self.tau_limits[dof_idx], self.tau_limits[dof_idx]))
            self.data.ctrl[actuator_id] = torque

    def step(self) -> None:
        """Advance the MuJoCo simulation by one step."""
        mujoco.mj_step(self.model, self.data)

    def check_contact(self, link_index: int, other_body_id: int = -1) -> tuple[bool, float]:
        """Check whether a body is in contact and sum its normal contact force."""
        is_contact = False
        normal_force = 0.0
        for contact_id in range(self.data.ncon):
            contact = self.data.contact[contact_id]
            body1 = int(self.model.geom_bodyid[int(contact.geom1)])
            body2 = int(self.model.geom_bodyid[int(contact.geom2)])
            if link_index not in (body1, body2):
                continue

            other_body = body2 if body1 == link_index else body1
            if other_body_id >= 0 and other_body != other_body_id:
                continue

            wrench = np.zeros(6)
            mujoco.mj_contactForce(self.model, self.data, contact_id, wrench)
            normal_force += max(0.0, float(wrench[0]))
            is_contact = True

        return is_contact, normal_force

    def get_contact_metrics(self, link_index: int, other_body_id: int = -1) -> dict:
        """Aggregate contact centroid and wrench magnitudes for one body."""
        is_contact = False
        normal_force = 0.0
        tangential_force = 0.0
        positions = []
        for contact_id in range(self.data.ncon):
            contact = self.data.contact[contact_id]
            body1 = int(self.model.geom_bodyid[int(contact.geom1)])
            body2 = int(self.model.geom_bodyid[int(contact.geom2)])
            if link_index not in (body1, body2):
                continue

            other_body = body2 if body1 == link_index else body1
            if other_body_id >= 0 and other_body != other_body_id:
                continue

            wrench = np.zeros(6)
            mujoco.mj_contactForce(self.model, self.data, contact_id, wrench)
            normal_force += max(0.0, float(wrench[0]))
            tangential_force += float(np.linalg.norm(wrench[1:3]))
            positions.append(np.array(contact.pos[:3], copy=True))
            is_contact = True

        if positions:
            position = np.mean(positions, axis=0)
        else:
            position = self.get_link_com_position(link_index)

        friction_ratio = tangential_force / max(normal_force, 1e-6)
        return {
            "is_contact": is_contact,
            "normal_force": normal_force,
            "tangential_force": tangential_force,
            "friction_ratio": friction_ratio,
            "position": position,
            "contact_count": len(positions),
        }
