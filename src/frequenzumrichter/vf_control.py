"""Scalar V/f control (voltage-frequency characteristic control).

V/f control is the simplest and most robust method for operating an induction
machine on a variable-frequency drive. The stator voltage is varied
proportionally to the stator frequency, so that the magnetic flux remains
approximately constant:

.. math::

    U(f) = U_{boost} + \\frac{U_{rated}}{f_{rated}} \\cdot f

At low frequencies, the *boost* voltage ``U_boost`` ensures that the ohmic
voltage drop across the stator resistance is compensated and that sufficient
starting torque is available.

V/f control operates **open loop**: no feedback of current or speed is
required. The actual speed lies below the synchronous speed by the
load-dependent slip.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .control_base import ControlOutput
from .controllers import RateLimiter
from .motor import MotorMeasurements

__all__ = ["VFControl"]


@dataclass
class VFControl:
    """Scalar V/f characteristic control.

    Parameters
    ----------
    f_rated:
        Rated frequency [Hz] (e.g. 50 Hz).
    v_rated:
        Phase-voltage amplitude at rated frequency [V].
    pole_pairs:
        Number of pole pairs of the machine.
    v_boost:
        Voltage boost at f = 0 [V].
    freq_ramp:
        Maximum frequency change [Hz/s] (soft-start ramp).
    """

    f_rated: float = 50.0
    v_rated: float = 230.0
    pole_pairs: int = 2
    v_boost: float = 10.0
    freq_ramp: float = 25.0

    theta_e: float = field(default=0.0)
    _ramp: RateLimiter = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self._ramp = RateLimiter(rate_up=self.freq_ramp, value=0.0)

    def reset(self) -> None:
        self.theta_e = 0.0
        self._ramp.reset(0.0)

    def voltage_for_frequency(self, f_e: float) -> float:
        """V/f characteristic: phase-voltage amplitude for the frequency ``f_e`` [Hz]."""
        slope = self.v_rated / self.f_rated
        v = self.v_boost + slope * abs(f_e)
        # limit the voltage above the rated frequency (field weakening)
        return float(min(v, self.v_rated))

    def compute(
        self,
        speed_ref: float,
        meas: MotorMeasurements,
        v_dc: float,
        dt: float,
    ) -> ControlOutput:
        """Computes the reference voltage space vector.

        ``speed_ref`` is the desired **mechanical** angular velocity
        [rad/s]. From this follows the electrical reference frequency ``f_e``.
        """
        # mechanical reference speed -> electrical reference frequency [Hz]
        f_e_target = self.pole_pairs * speed_ref / (2.0 * np.pi)
        f_e = self._ramp.step(f_e_target, dt)

        omega_e = 2.0 * np.pi * f_e
        self.theta_e += omega_e * dt
        # keep the angle within the range [0, 2π)
        self.theta_e = float(np.mod(self.theta_e, 2.0 * np.pi))

        v_mag = self.voltage_for_frequency(f_e)
        v_alpha = v_mag * np.cos(self.theta_e)
        v_beta = v_mag * np.sin(self.theta_e)

        return ControlOutput(
            v_alpha_ref=float(v_alpha),
            v_beta_ref=float(v_beta),
            theta_field=self.theta_e,
        )
