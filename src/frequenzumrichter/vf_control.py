"""Skalare U/f-Steuerung (Spannungs-Frequenz-Kennliniensteuerung).

Die U/f-Steuerung ist das einfachste und robusteste Verfahren zum Betrieb einer
Asynchronmaschine am Frequenzumrichter. Die Statorspannung wird proportional zur
Statorfrequenz geführt, sodass der magnetische Fluss näherungsweise konstant
bleibt:

.. math::

    U(f) = U_{boost} + \\frac{U_{nenn}}{f_{nenn}} \\cdot f

Bei kleinen Frequenzen sorgt die *Boost*-Spannung ``U_boost`` dafür, dass der
ohmsche Spannungsabfall am Statorwiderstand kompensiert wird und ein
ausreichendes Anlaufmoment zur Verfügung steht.

Die U/f-Steuerung arbeitet **gesteuert** (open loop): Es ist keine Rückführung
von Strom oder Drehzahl erforderlich. Die tatsächliche Drehzahl liegt um den
lastabhängigen Schlupf unter der Synchrondrehzahl.
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
    """Skalare U/f-Kennliniensteuerung.

    Parameters
    ----------
    f_rated:
        Nennfrequenz [Hz] (z.B. 50 Hz).
    v_rated:
        Strangspannungs-Amplitude bei Nennfrequenz [V].
    pole_pairs:
        Polpaarzahl der Maschine.
    v_boost:
        Spannungsanhebung bei f = 0 [V].
    freq_ramp:
        Maximale Frequenzänderung [Hz/s] (Sanftanlauf-Rampe).
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
        """U/f-Kennlinie: Strangspannungsamplitude für die Frequenz ``f_e`` [Hz]."""
        slope = self.v_rated / self.f_rated
        v = self.v_boost + slope * abs(f_e)
        # oberhalb der Nennfrequenz Spannung begrenzen (Feldschwächung)
        return float(min(v, self.v_rated))

    def compute(
        self,
        speed_ref: float,
        meas: MotorMeasurements,
        v_dc: float,
        dt: float,
    ) -> ControlOutput:
        """Berechnet den Soll-Spannungsraumzeiger.

        ``speed_ref`` ist die gewünschte **mechanische** Winkelgeschwindigkeit
        [rad/s]. Daraus folgt die elektrische Sollfrequenz ``f_e``.
        """
        # mechanische Solldrehzahl -> elektrische Sollfrequenz [Hz]
        f_e_target = self.pole_pairs * speed_ref / (2.0 * np.pi)
        f_e = self._ramp.step(f_e_target, dt)

        omega_e = 2.0 * np.pi * f_e
        self.theta_e += omega_e * dt
        # Winkel im Bereich [0, 2π) halten
        self.theta_e = float(np.mod(self.theta_e, 2.0 * np.pi))

        v_mag = self.voltage_for_frequency(f_e)
        v_alpha = v_mag * np.cos(self.theta_e)
        v_beta = v_mag * np.sin(self.theta_e)

        return ControlOutput(
            v_alpha_ref=float(v_alpha),
            v_beta_ref=float(v_beta),
            theta_field=self.theta_e,
        )
