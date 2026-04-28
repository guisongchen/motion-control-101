"""Core helpers for the control state machine."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from config import GRAVITY


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


def skew(v: np.ndarray) -> np.ndarray:
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


def get_contact_entry(foot_contacts: list[dict], link: int) -> Optional[dict]:
    """Return the contact record for one foot link."""
    return next((fc for fc in foot_contacts if fc["link"] == link), None)


def support_name(foot_name_map: dict[int, str], link: int) -> str:
    """Map support link index back to the configured foot name."""
    return foot_name_map.get(link, str(link))
