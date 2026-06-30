"""Numerical integration and recording of the simulation results."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np

__all__ = ["rk4_step", "SimulationResult"]


def rk4_step(
    derivatives: Callable[..., np.ndarray],
    x: np.ndarray,
    dt: float,
    *args,
) -> np.ndarray:
    """A single integration step using the classical 4th-order Runge-Kutta method.

    Parameters
    ----------
    derivatives:
        Function ``f(x, *args) -> dx/dt``.
    x:
        Current state vector.
    dt:
        Step size [s].
    *args:
        Additional arguments of ``derivatives`` that are held constant over
        the step (zero-order hold of the manipulated variable).

    Returns
    -------
    numpy.ndarray
        The state vector after the time step.
    """
    k1 = derivatives(x, *args)
    k2 = derivatives(x + 0.5 * dt * k1, *args)
    k3 = derivatives(x + 0.5 * dt * k2, *args)
    k4 = derivatives(x + dt * k3, *args)
    return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


@dataclass
class SimulationResult:
    """Collects the time-domain traces of a simulation.

    The fields are populated step by step during the simulation and are
    converted into ``numpy`` arrays at the end via :meth:`finalize`. The traces
    can then be accessed conveniently (e.g. ``result.speed``).
    """

    time: list[float] = field(default_factory=list)
    speed: list[float] = field(default_factory=list)
    speed_ref: list[float] = field(default_factory=list)
    torque: list[float] = field(default_factory=list)
    load_torque: list[float] = field(default_factory=list)
    current_magnitude: list[float] = field(default_factory=list)
    i_a: list[float] = field(default_factory=list)
    i_b: list[float] = field(default_factory=list)
    i_c: list[float] = field(default_factory=list)
    i_d: list[float] = field(default_factory=list)
    i_q: list[float] = field(default_factory=list)
    v_dc: list[float] = field(default_factory=list)
    dc_current: list[float] = field(default_factory=list)
    duty_a: list[float] = field(default_factory=list)
    tripped: list[bool] = field(default_factory=list)
    thermal_state: list[float] = field(default_factory=list)

    def record(self, **values) -> None:
        """Appends a data record. Unknown keys are ignored."""
        for key, value in values.items():
            target = getattr(self, key, None)
            if isinstance(target, list):
                target.append(value)

    def finalize(self) -> "SimulationResult":
        """Converts all lists into ``numpy`` arrays (in-place) and returns self."""
        for key, value in vars(self).items():
            if isinstance(value, list):
                setattr(self, key, np.asarray(value))
        return self

    def settling_time(self, tolerance: float = 0.02) -> float | None:
        """Estimates the settling time relative to the last speed setpoint.

        Returns the instant from which the speed stays permanently within the
        ``tolerance`` band (relative to the setpoint), or ``None``.
        """
        time = np.asarray(self.time)
        speed = np.asarray(self.speed)
        ref = np.asarray(self.speed_ref)
        if time.size == 0:
            return None
        target = ref[-1]
        if abs(target) < 1e-9:
            return None
        band = tolerance * abs(target)
        outside = np.abs(speed - target) > band
        if not outside.any():
            return float(time[0])
        last_outside = np.nonzero(outside)[0][-1]
        if last_outside + 1 >= time.size:
            return None
        return float(time[last_outside + 1])
