"""Common types and helper functions for the control methods."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

__all__ = ["ControlOutput", "limit_vector"]


@dataclass
class ControlOutput:
    """Result of a control step.

    In addition to the actual reference voltage space vector ``(v_alpha_ref,
    v_beta_ref)``, diagnostic quantities are carried along that are useful for
    analysis and visualization.
    """

    v_alpha_ref: float
    v_beta_ref: float
    theta_field: float = 0.0  # field angle used [rad]
    i_d: float = 0.0  # measured d-axis current [A]
    i_q: float = 0.0  # measured q-axis current [A]
    i_d_ref: float = 0.0  # reference d-axis current [A]
    i_q_ref: float = 0.0  # reference q-axis current [A]
    v_d: float = 0.0  # reference d-axis voltage [V]
    v_q: float = 0.0  # reference q-axis voltage [V]


def limit_vector(d: float, q: float, max_magnitude: float) -> tuple[float, float]:
    """Limits the magnitude of a 2-D vector to a circle (circle limiting).

    If the magnitude ``sqrt(d² + q²)`` exceeds the limit ``max_magnitude``, both
    components are scaled down while preserving the angle. This corresponds to
    the voltage limiting of the current controllers to the achievable
    modulation circle of the inverter.
    """
    magnitude = float(np.hypot(d, q))
    if magnitude <= max_magnitude or magnitude < 1e-12:
        return d, q
    scale = max_magnitude / magnitude
    return d * scale, q * scale
