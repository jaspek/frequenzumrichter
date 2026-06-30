"""Feldorientierte Regelung (FOC / Vektorregelung).

Die feldorientierte Regelung entkoppelt die Drehmoment- und Flussbildung einer
Drehfeldmaschine, indem im rotorfluss- bzw. rotorlagefesten dq-Koordinatensystem
geregelt wird:

* der **d-Strom** ``i_d`` bestimmt den magnetischen Fluss,
* der **q-Strom** ``i_q`` bestimmt das Drehmoment.

Damit lässt sich eine Drehfeldmaschine so präzise und dynamisch regeln wie eine
fremderregte Gleichstrommaschine.

Implementiert sind:

* :class:`FOCPMSM` – feldorientierte Regelung der permanenterregten
  Synchronmaschine (Rotorlage direkt verfügbar, ``i_d* = 0``).
* :class:`FOCInduction` – indirekte rotorflussorientierte Regelung (IRFOC) der
  Asynchronmaschine mit Schlupf-/Feldwinkelberechnung.

Kaskadenstruktur in beiden Fällen:

    Drehzahlregler (PI)  ->  i_q*  ->  Stromregler (PI)  ->  v_d, v_q  ->  SVPWM
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .control_base import ControlOutput, limit_vector
from .controllers import PIController
from .motor import PMSM, InductionMotor, MotorMeasurements
from .pwm import MAX_LINEAR_SVPWM
from .transforms import inverse_park, park

__all__ = ["FOCPMSM", "FOCInduction"]


@dataclass
class FOCPMSM:
    """Feldorientierte Regelung der permanenterregten Synchronmaschine.

    Parameters
    ----------
    motor:
        Die zu regelnde :class:`~frequenzumrichter.motor.PMSM`. Die
        Maschinenparameter werden für die Entkopplungs-Vorsteuerung verwendet.
    speed_pi:
        Drehzahlregler (PI). Die Stellgröße ist der Soll-q-Strom.
    id_pi, iq_pi:
        Stromregler (PI) für d- und q-Achse. Stellgröße ist die dq-Spannung.
    decoupling:
        Aktiviert die Entkopplungs-Vorsteuerung der Spannungsgleichungen.
    """

    motor: PMSM
    speed_pi: PIController
    id_pi: PIController
    iq_pi: PIController
    decoupling: bool = True

    def reset(self) -> None:
        self.speed_pi.reset()
        self.id_pi.reset()
        self.iq_pi.reset()

    def compute(
        self,
        speed_ref: float,
        meas: MotorMeasurements,
        v_dc: float,
        dt: float,
    ) -> ControlOutput:
        p = self.motor.pole_pairs
        theta_e = p * meas.theta_m
        omega_e = p * meas.omega_m

        # Messung ins rotorfeste dq-System transformieren
        i_d, i_q = park(meas.i_alpha, meas.i_beta, theta_e)

        # Äußere Schleife: Drehzahlregler liefert Soll-q-Strom
        i_q_ref = self.speed_pi.step(speed_ref - meas.omega_m, dt)
        i_d_ref = 0.0  # Oberflächen-PMSM: kein Feldschwächbetrieb

        # Entkopplungs-Vorsteuerung
        ff_d = -omega_e * self.motor.l_q * i_q if self.decoupling else 0.0
        ff_q = (
            omega_e * (self.motor.l_d * i_d + self.motor.flux_linkage)
            if self.decoupling
            else 0.0
        )

        # Innere Schleife: Stromregler
        v_d = self.id_pi.step(i_d_ref - i_d, dt, feedforward=ff_d)
        v_q = self.iq_pi.step(i_q_ref - i_q, dt, feedforward=ff_q)

        # Spannungsbegrenzung auf den Aussteuerkreis
        v_max = MAX_LINEAR_SVPWM * v_dc
        v_d, v_q = limit_vector(v_d, v_q, v_max)

        v_alpha, v_beta = inverse_park(v_d, v_q, theta_e)
        return ControlOutput(
            v_alpha_ref=float(v_alpha),
            v_beta_ref=float(v_beta),
            theta_field=float(theta_e),
            i_d=float(i_d),
            i_q=float(i_q),
            i_d_ref=float(i_d_ref),
            i_q_ref=float(i_q_ref),
            v_d=float(v_d),
            v_q=float(v_q),
        )


@dataclass
class FOCInduction:
    """Indirekte rotorflussorientierte Regelung (IRFOC) der Asynchronmaschine.

    Der Feldwinkel wird nicht gemessen, sondern aus der Schlupfbeziehung
    berechnet (daher *indirekt*):

    .. math::

        \\omega_{sl} = \\frac{i_q^{*}}{\\tau_r \\, i_d^{*}}, \\qquad
        \\theta_{feld} = \\int (p\\,\\omega_m + \\omega_{sl})\\, dt

    Der d-Strom-Sollwert stellt den Rotorfluss ein
    (``i_d* = ψ_r,soll / L_m``), der q-Strom-Sollwert kommt aus dem
    Drehzahlregler.

    Parameters
    ----------
    motor:
        Die zu regelnde :class:`~frequenzumrichter.motor.InductionMotor`.
    speed_pi, id_pi, iq_pi:
        PI-Regler für Drehzahl- bzw. Stromregelung.
    flux_ref:
        Soll-Rotorfluss [Wb]. Bestimmt zusammen mit ``L_m`` den d-Strom.
    """

    motor: InductionMotor
    speed_pi: PIController
    id_pi: PIController
    iq_pi: PIController
    flux_ref: float = 0.5

    theta_field: float = field(default=0.0)

    def reset(self) -> None:
        self.speed_pi.reset()
        self.id_pi.reset()
        self.iq_pi.reset()
        self.theta_field = 0.0

    def compute(
        self,
        speed_ref: float,
        meas: MotorMeasurements,
        v_dc: float,
        dt: float,
    ) -> ControlOutput:
        p = self.motor.pole_pairs
        tau_r = self.motor.tau_r
        l_m = self.motor.l_m

        # Soll-d-Strom aus dem gewünschten Rotorfluss
        i_d_ref = self.flux_ref / l_m
        # Soll-q-Strom aus dem Drehzahlregler
        i_q_ref = self.speed_pi.step(speed_ref - meas.omega_m, dt)

        # Schlupfkreisfrequenz und Feldwinkel (indirekte Orientierung)
        omega_sl = i_q_ref / (tau_r * i_d_ref) if abs(i_d_ref) > 1e-9 else 0.0
        omega_e = p * meas.omega_m + omega_sl
        self.theta_field += omega_e * dt
        self.theta_field = float(np.mod(self.theta_field, 2.0 * np.pi))

        # Messung ins feldorientierte dq-System transformieren
        i_d, i_q = park(meas.i_alpha, meas.i_beta, self.theta_field)

        # Stromregler
        v_d = self.id_pi.step(i_d_ref - i_d, dt)
        v_q = self.iq_pi.step(i_q_ref - i_q, dt)

        # Spannungsbegrenzung auf den Aussteuerkreis
        v_max = MAX_LINEAR_SVPWM * v_dc
        v_d, v_q = limit_vector(v_d, v_q, v_max)

        v_alpha, v_beta = inverse_park(v_d, v_q, self.theta_field)
        return ControlOutput(
            v_alpha_ref=float(v_alpha),
            v_beta_ref=float(v_beta),
            theta_field=float(self.theta_field),
            i_d=float(i_d),
            i_q=float(i_q),
            i_d_ref=float(i_d_ref),
            i_q_ref=float(i_q_ref),
            v_d=float(v_d),
            v_q=float(v_q),
        )
