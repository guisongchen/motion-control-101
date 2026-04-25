"""Core enums, dataclasses, and phase-agnostic helpers for the control state machine."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional

import numpy as np

from config import GRAVITY
from robot_model import RobotModel


class ControlPhase(Enum):
    """High-level task phases for single-support standing."""

    INIT_SETTLE = auto()
    DOUBLE_SUPPORT_HOLD = auto()
    LOAD_SHIFT = auto()
    PRE_LIFTOFF = auto()
    SINGLE_SUPPORT = auto()


@dataclass
class StabilityGate:
    """Debounce gate: condition must stay true for a hold time before firing."""

    ready_since: Optional[float] = None

    def check(self, condition: bool, t: float, hold_time: float) -> bool:
        if not condition:
            self.ready_since = None
            return False
        if self.ready_since is None:
            self.ready_since = t
        return t - self.ready_since >= hold_time


@dataclass
class SingleSupportState:
    """Persistent state for the single-support balancing task."""

    com_ref: Optional[np.ndarray] = None
    joint_ref: Optional[np.ndarray] = None
    ready_since: Optional[float] = None
    established: bool = False
    # Filter state for single-support CoP / support-position feedback
    filtered_cop_world: Optional[np.ndarray] = None
    prev_filtered_cop_world: Optional[np.ndarray] = None
    filtered_support_position_world: Optional[np.ndarray] = None
    prev_filtered_support_position_world: Optional[np.ndarray] = None


@dataclass
class CentroidalState:
    """Full centroidal state at one timestep."""

    q: np.ndarray
    v: np.ndarray
    c: np.ndarray
    c_dot: np.ndarray
    L: np.ndarray


@dataclass
class TaskReference:
    """Desired centroidal targets for MPC / WBC."""

    c: np.ndarray
    c_dot: np.ndarray
    c_ddot: np.ndarray
    L: np.ndarray
    L_dot: np.ndarray


@dataclass
class SupportContext:
    """Geometric and contact context for the support / swing feet."""

    p_foot: np.ndarray
    support_foot_link: int
    swing_foot_link: int
    foot_contacts: list[dict]
    contact_local_positions: list[np.ndarray]
    initial_contact: np.ndarray
    initial_yaw: float


@dataclass
class SolverConfig:
    """MPC / WBC solvers and their execution periods."""

    mpc: object
    wbc_ss: object
    wbc_ss_with_swing: object
    mpc_period: int
    wbc_period: int


@dataclass
class ControlMemory:
    """Mutable control outputs carried across timesteps."""

    mpc_force_target: np.ndarray
    mpc_result: object
    wbc_result: object
    last_valid_support_tau: Optional[np.ndarray] = None


def skew(v: np.ndarray) -> np.ndarray:
    """向量叉乘矩阵。"""
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ])


def compute_centroidal_dynamics(
    m: float, c: np.ndarray, p_foot: np.ndarray, dt: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Linearize centroidal dynamics around the current support point."""
    A_d = np.eye(9)
    A_d[0:3, 3:6] = dt * np.eye(3)

    B_d = np.zeros((9, 3))
    B_d[3:6, :] = dt * np.eye(3) / m
    r = p_foot - c
    B_d[6:9, :] = dt * skew(r)

    d_d = np.zeros(9)
    d_d[3:6] = dt * (-GRAVITY)

    return A_d, B_d, d_d


def format_vector(vec: np.ndarray) -> str:
    """Compact vector formatter for runtime diagnostics."""
    return "[" + ", ".join(f"{x:.2f}" for x in vec) + "]"


def get_contact_entry(foot_contacts: list[dict], link: int) -> Optional[dict]:
    """Return the contact record for one foot link."""
    return next((fc for fc in foot_contacts if fc["link"] == link), None)


def support_name(foot_name_map: dict[int, str], link: int) -> str:
    """Map support link index back to the configured foot name."""
    return foot_name_map.get(link, str(link))


def world_point_to_local_body_point(
    robot: RobotModel,
    link: int,
    world_point: np.ndarray,
) -> np.ndarray:
    """Express a world-frame contact point in the queried body frame."""
    foot_origin = np.array(robot.data.xpos[link], copy=True)
    foot_rotation = np.array(robot.data.xmat[link]).reshape(3, 3)
    return foot_rotation.T @ (np.asarray(world_point, dtype=float) - foot_origin)


def compute_phase_com_target(
    nominal_c_ref: np.ndarray,
    phase: ControlPhase,
    single_support_com_ref: Optional[np.ndarray],
    initial_foot_pos: dict[int, np.ndarray],
    support_foot_link: int,
    swing_foot_link: int,
) -> np.ndarray:
    """Shift the CoM reference toward the support foot before liftoff."""
    from config import LOAD_SHIFT_COM_RATIO, PRE_LIFTOFF_COM_RATIO, SINGLE_SUPPORT_COM_RATIO

    c_ref = nominal_c_ref.copy()
    if phase in (ControlPhase.INIT_SETTLE, ControlPhase.DOUBLE_SUPPORT_HOLD):
        return c_ref
    if phase == ControlPhase.SINGLE_SUPPORT and single_support_com_ref is not None:
        return single_support_com_ref.copy()

    support_xy = initial_foot_pos[support_foot_link][:2]
    swing_xy = initial_foot_pos[swing_foot_link][:2]
    stance_midpoint = 0.5 * (support_xy + swing_xy)
    if phase == ControlPhase.LOAD_SHIFT:
        target_ratio = LOAD_SHIFT_COM_RATIO
    elif phase == ControlPhase.PRE_LIFTOFF:
        target_ratio = PRE_LIFTOFF_COM_RATIO
    else:
        target_ratio = SINGLE_SUPPORT_COM_RATIO
    target_xy = stance_midpoint + target_ratio * (support_xy - stance_midpoint)
    c_ref[:2] = target_xy
    return c_ref


def compute_forward_handoff_metrics(
    c: np.ndarray,
    c_dot: np.ndarray,
    c_ref: np.ndarray,
) -> tuple[float, float]:
    """Return forward CoM position error and x velocity for handoff gating."""
    return float(c[0] - c_ref[0]), float(c_dot[0])
