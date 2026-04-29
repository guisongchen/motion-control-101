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


def is_com_inside_support_polygon(c: np.ndarray, corners_xy: list[np.ndarray], margin: float) -> bool:
    if not corners_xy:
        return False
    xs = [float(p[0]) for p in corners_xy]
    ys = [float(p[1]) for p in corners_xy]
    return (
        float(c[0]) >= min(xs) + margin
        and float(c[0]) <= max(xs) - margin
        and float(c[1]) >= min(ys) + margin
        and float(c[1]) <= max(ys) - margin
    )


def get_contacts(foot_contacts: list[dict]) -> dict[int, dict]:
    """Return all contacts keyed by link id so callers look up once."""
    return {fc["link"]: fc for fc in foot_contacts}


def support_name(foot_name_map: dict[int, str], link: int) -> str:
    """Map support link index back to the configured foot name."""
    return foot_name_map.get(link, str(link))
