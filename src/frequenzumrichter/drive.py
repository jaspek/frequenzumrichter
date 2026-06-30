"""Gesamtmodell des Frequenzumrichters (Antriebsstrang).

Die Klasse :class:`Frequenzumrichter` verbindet alle Teilkomponenten zu einem
geschlossenen Antriebssystem:

    Gleichrichter -> Zwischenkreis -> Wechselrichter -> Regelung -> Motor

und stellt eine einfache Schnittstelle bereit, um das System Schritt für Schritt
(:meth:`step`) oder über eine ganze Trajektorie (:meth:`run`) zu simulieren.

Typische Verwendung::

    from frequenzumrichter import Frequenzumrichter, build_vf_drive

    fu = build_vf_drive()
    result = fu.run(t_end=2.0, speed_ref=lambda t: 150.0, load_torque=2.0)
    print("Enddrehzahl:", result.speed[-1])
"""

from __future__ import annotations

import inspect
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Protocol

import numpy as np

from .control_base import ControlOutput
from .motor import MotorMeasurements
from .power_stage import DCLink, Inverter, Rectifier
from .protection import Protection
from .simulation import SimulationResult, rk4_step

__all__ = ["Controller", "Motor", "Frequenzumrichter"]


class Motor(Protocol):
    """Schnittstelle, die ein Motormodell für die Simulation erfüllen muss."""

    n_states: int

    def initial_state(self) -> np.ndarray: ...

    def derivatives(
        self, x: np.ndarray, v_alpha: float, v_beta: float, load_torque: float
    ) -> np.ndarray: ...

    def measurements(self, x: np.ndarray) -> MotorMeasurements: ...


class Controller(Protocol):
    """Schnittstelle eines Regelverfahrens (U/f oder FOC)."""

    def reset(self) -> None: ...

    def compute(
        self,
        speed_ref: float,
        meas: MotorMeasurements,
        v_dc: float,
        dt: float,
    ) -> ControlOutput: ...


def _as_callable(value) -> Callable[[float], float]:
    """Erlaubt sowohl konstante Werte als auch Zeitfunktionen ``f(t)``."""
    if callable(value):
        return value
    return lambda _t: float(value)


def _resolve_load(load_torque, t: float, meas: MotorMeasurements) -> float:
    """Wertet die Lastvorgabe aus.

    ``load_torque`` darf sein:

    * eine Konstante (z.B. ``2.0``),
    * eine Zeitfunktion ``f(t)``,
    * eine zustandsabhängige Funktion ``f(t, meas)`` – damit lassen sich
      *passive* Lasten (Lüfter/Pumpe ∝ ω², Reibung) abbilden, die bei
      Stillstand verschwinden.
    """
    if not callable(load_torque):
        return float(load_torque)
    n_params = len(inspect.signature(load_torque).parameters)
    if n_params >= 2:
        return float(load_torque(t, meas))
    return float(load_torque(t))


@dataclass
class Frequenzumrichter:
    """Vollständiges Frequenzumrichter-Antriebsmodell.

    Parameters
    ----------
    motor:
        Das Motormodell (Asynchron- oder Synchronmaschine).
    controller:
        Das Regelverfahren (:class:`~frequenzumrichter.vf_control.VFControl`,
        :class:`~frequenzumrichter.foc.FOCPMSM` oder
        :class:`~frequenzumrichter.foc.FOCInduction`).
    inverter:
        Der Wechselrichter (Mittelwertmodell mit SVPWM).
    dc_link:
        Der Zwischenkreis.
    rectifier:
        Optionaler Gleichrichter (nur informativ / zur Spannungsvorgabe).
    protection:
        Optionale Schutzeinrichtung.
    control_period:
        Abtastzeit der Regelung [s]. Standard 100 µs (10 kHz).
    """

    motor: Motor
    controller: Controller
    inverter: Inverter = field(default_factory=Inverter)
    dc_link: DCLink = field(default_factory=DCLink)
    rectifier: Rectifier | None = None
    protection: Protection | None = None
    control_period: float = 1e-4

    state: np.ndarray = field(init=False)
    time: float = field(default=0.0, init=False)

    def __post_init__(self) -> None:
        self.state = self.motor.initial_state()

    # ------------------------------------------------------------------ #
    # Zustandsführung
    # ------------------------------------------------------------------ #
    def reset(self) -> None:
        """Setzt Motor, Regler, Schutz und Zwischenkreis in den Anfangszustand."""
        self.state = self.motor.initial_state()
        self.time = 0.0
        self.controller.reset()
        if self.protection is not None:
            self.protection.reset()
        self.dc_link.reset()

    @property
    def measurements(self) -> MotorMeasurements:
        return self.motor.measurements(self.state)

    # ------------------------------------------------------------------ #
    # Simulationsschritt
    # ------------------------------------------------------------------ #
    def step(self, speed_ref: float, load_torque: float, dt: float) -> dict:
        """Führt einen Regel-/Integrationsschritt der Dauer ``dt`` aus.

        Returns
        -------
        dict
            Momentaufnahme der wichtigsten Größen (für die Aufzeichnung).
        """
        meas = self.motor.measurements(self.state)
        v_dc = self.dc_link.voltage
        i_mag = float(np.hypot(meas.i_alpha, meas.i_beta))

        tripped = False
        if self.protection is not None:
            tripped = self.protection.check(i_mag, v_dc, meas.omega_m, dt)

        if tripped:
            # Pulssperre: Wechselrichter gibt keine Spannung aus
            ctrl = ControlOutput(v_alpha_ref=0.0, v_beta_ref=0.0)
            inv = self.inverter.modulate(0.0, 0.0, v_dc, meas.i_alpha, meas.i_beta)
        else:
            ctrl = self.controller.compute(speed_ref, meas, v_dc, dt)
            inv = self.inverter.modulate(
                ctrl.v_alpha_ref,
                ctrl.v_beta_ref,
                v_dc,
                meas.i_alpha,
                meas.i_beta,
            )

        # Plant: Motor mit der tatsächlich gestellten Spannung integrieren (ZOH)
        self.state = rk4_step(
            self.motor.derivatives,
            self.state,
            dt,
            inv.v_alpha,
            inv.v_beta,
            load_torque,
        )

        # Zwischenkreis aktualisieren (bei steifem Kreis konstant)
        self.dc_link.update(inv.i_dc, dt)
        self.time += dt

        thermal = self.protection.thermal_state if self.protection else 0.0
        return {
            "time": self.time,
            "speed": meas.omega_m,
            "speed_ref": speed_ref,
            "torque": meas.torque,
            "load_torque": load_torque,
            "current_magnitude": i_mag,
            "i_a": meas.i_abc[0],
            "i_b": meas.i_abc[1],
            "i_c": meas.i_abc[2],
            "i_d": ctrl.i_d,
            "i_q": ctrl.i_q,
            "v_dc": v_dc,
            "dc_current": inv.i_dc,
            "duty_a": inv.duties[0],
            "tripped": tripped,
            "thermal_state": thermal,
        }

    # ------------------------------------------------------------------ #
    # Trajektorien-Simulation
    # ------------------------------------------------------------------ #
    def run(
        self,
        t_end: float,
        speed_ref,
        load_torque=0.0,
        dt: float | None = None,
    ) -> SimulationResult:
        """Simuliert den Antrieb von ``t=0`` bis ``t_end``.

        Parameters
        ----------
        t_end:
            Simulationsdauer [s].
        speed_ref:
            Drehzahlsollwert [rad/s] – Konstante oder Funktion ``f(t)``.
        load_torque:
            Lastmoment [Nm] – Konstante, Funktion ``f(t)`` oder zustands­abhängige
            Funktion ``f(t, meas)`` (z.B. für passive Lüfter-/Pumpenlasten).
        dt:
            Schrittweite [s]; Standard ist :attr:`control_period`.

        Returns
        -------
        SimulationResult
            Aufgezeichnete Zeitverläufe (als ``numpy``-Arrays).
        """
        dt = self.control_period if dt is None else dt
        speed_fn = _as_callable(speed_ref)

        n_steps = int(round(t_end / dt))
        result = SimulationResult()
        for k in range(n_steps):
            t = k * dt
            meas = self.motor.measurements(self.state)
            load = _resolve_load(load_torque, t, meas)
            sample = self.step(speed_fn(t), load, dt)
            result.record(**sample)
        return result.finalize()
