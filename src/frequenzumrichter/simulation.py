"""Numerische Integration und Aufzeichnung der Simulationsergebnisse."""

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
    """Ein Integrationsschritt nach dem klassischen Runge-Kutta-Verfahren 4. Ordnung.

    Parameters
    ----------
    derivatives:
        Funktion ``f(x, *args) -> dx/dt``.
    x:
        Aktueller Zustandsvektor.
    dt:
        Schrittweite [s].
    *args:
        Weitere, über den Schritt konstant gehaltene Argumente von
        ``derivatives`` (Zero-Order-Hold der Stellgröße).

    Returns
    -------
    numpy.ndarray
        Der Zustandsvektor nach dem Zeitschritt.
    """
    k1 = derivatives(x, *args)
    k2 = derivatives(x + 0.5 * dt * k1, *args)
    k3 = derivatives(x + 0.5 * dt * k2, *args)
    k4 = derivatives(x + dt * k3, *args)
    return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


@dataclass
class SimulationResult:
    """Sammelt die Zeitverläufe einer Simulation.

    Die Felder werden während der Simulation Schritt für Schritt befüllt und am
    Ende über :meth:`finalize` in ``numpy``-Arrays umgewandelt. Anschließend
    lässt sich bequem auf die Verläufe zugreifen (z.B. ``result.speed``).
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
        """Hängt einen Datensatz an. Unbekannte Schlüssel werden ignoriert."""
        for key, value in values.items():
            target = getattr(self, key, None)
            if isinstance(target, list):
                target.append(value)

    def finalize(self) -> "SimulationResult":
        """Wandelt alle Listen in ``numpy``-Arrays um (in-place) und gibt self zurück."""
        for key, value in vars(self).items():
            if isinstance(value, list):
                setattr(self, key, np.asarray(value))
        return self

    def settling_time(self, tolerance: float = 0.02) -> float | None:
        """Schätzt die Ausregelzeit bezogen auf den letzten Drehzahlsollwert.

        Liefert den Zeitpunkt, ab dem die Drehzahl dauerhaft innerhalb des
        ``tolerance``-Bands (relativ zum Sollwert) bleibt, oder ``None``.
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
