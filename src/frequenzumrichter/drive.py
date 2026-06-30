"""Overall model of the variable-frequency drive (drivetrain).

The :class:`Frequenzumrichter` class connects all subcomponents into a
closed drive system:

    rectifier -> DC link -> inverter -> control -> motor

and provides a simple interface to simulate the system step by step
(:meth:`step`) or over an entire trajectory (:meth:`run`).

Typical usage::

    from frequenzumrichter import Frequenzumrichter, build_vf_drive

    fu = build_vf_drive()
    result = fu.run(t_end=2.0, speed_ref=lambda t: 150.0, load_torque=2.0)
    print("Final speed:", result.speed[-1])
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
    """Interface that a motor model must satisfy for the simulation."""

    n_states: int

    def initial_state(self) -> np.ndarray: ...

    def derivatives(
        self, x: np.ndarray, v_alpha: float, v_beta: float, load_torque: float
    ) -> np.ndarray: ...

    def measurements(self, x: np.ndarray) -> MotorMeasurements: ...


class Controller(Protocol):
    """Interface of a control method (V/f or FOC)."""

    def reset(self) -> None: ...

    def compute(
        self,
        speed_ref: float,
        meas: MotorMeasurements,
        v_dc: float,
        dt: float,
    ) -> ControlOutput: ...


def _as_callable(value) -> Callable[[float], float]:
    """Allows both constant values and time functions ``f(t)``."""
    if callable(value):
        return value
    return lambda _t: float(value)


def _resolve_load(load_torque, t: float, meas: MotorMeasurements) -> float:
    """Evaluates the load specification.

    ``load_torque`` may be:

    * a constant (e.g. ``2.0``),
    * a time function ``f(t)``,
    * a state-dependent function ``f(t, meas)`` – this can model
      *passive* loads (fan/pump ∝ ω², friction) that vanish at
      standstill.
    """
    if not callable(load_torque):
        return float(load_torque)
    n_params = len(inspect.signature(load_torque).parameters)
    if n_params >= 2:
        return float(load_torque(t, meas))
    return float(load_torque(t))


@dataclass
class Frequenzumrichter:
    """Complete variable-frequency drive model.

    Parameters
    ----------
    motor:
        The motor model (induction or synchronous machine).
    controller:
        The control method (:class:`~frequenzumrichter.vf_control.VFControl`,
        :class:`~frequenzumrichter.foc.FOCPMSM` or
        :class:`~frequenzumrichter.foc.FOCInduction`).
    inverter:
        The inverter (averaged model with SVPWM).
    dc_link:
        The DC link.
    rectifier:
        Optional rectifier (informational only / for voltage specification).
    protection:
        Optional protection device.
    control_period:
        Sampling time of the control [s]. Default 100 µs (10 kHz).
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
    # State management
    # ------------------------------------------------------------------ #
    def reset(self) -> None:
        """Resets motor, controller, protection and DC link to the initial state."""
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
    # Simulation step
    # ------------------------------------------------------------------ #
    def step(self, speed_ref: float, load_torque: float, dt: float) -> dict:
        """Performs one control/integration step of duration ``dt``.

        Returns
        -------
        dict
            Snapshot of the most important quantities (for recording).
        """
        meas = self.motor.measurements(self.state)
        v_dc = self.dc_link.voltage
        i_mag = float(np.hypot(meas.i_alpha, meas.i_beta))

        tripped = False
        if self.protection is not None:
            tripped = self.protection.check(i_mag, v_dc, meas.omega_m, dt)

        if tripped:
            # Pulse blocking: inverter outputs no voltage
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

        # Plant: integrate motor with the actually applied voltage (ZOH)
        self.state = rk4_step(
            self.motor.derivatives,
            self.state,
            dt,
            inv.v_alpha,
            inv.v_beta,
            load_torque,
        )

        # Update DC link (constant for a stiff link)
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
    # Trajectory simulation
    # ------------------------------------------------------------------ #
    def run(
        self,
        t_end: float,
        speed_ref,
        load_torque=0.0,
        dt: float | None = None,
    ) -> SimulationResult:
        """Simulates the drive from ``t=0`` to ``t_end``.

        Parameters
        ----------
        t_end:
            Simulation duration [s].
        speed_ref:
            Speed setpoint [rad/s] – constant or function ``f(t)``.
        load_torque:
            Load torque [Nm] – constant, function ``f(t)`` or state-dependent
            function ``f(t, meas)`` (e.g. for passive fan/pump loads).
        dt:
            Step size [s]; default is :attr:`control_period`.

        Returns
        -------
        SimulationResult
            Recorded time series (as ``numpy`` arrays).
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
