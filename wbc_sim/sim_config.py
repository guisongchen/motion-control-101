from dataclasses import dataclass, field
import numpy as np

@dataclass(frozen=True)
class SimConfig:
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
    qdot_min: float = -20.0
    qdot_max: float = 20.0