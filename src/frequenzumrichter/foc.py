"""Field-oriented control (FOC / vector control).

Field-oriented control decouples the torque and flux production of a rotating-field
machine by controlling in the rotor-flux- or rotor-position-fixed dq coordinate
system:

* the **d current** ``i_d`` determines the magnetic flux,
* the **q current** ``i_q`` determines the torque.

This allows a rotating-field machine to be controlled as precisely and dynamically as
a separately excited DC machine.

Implemented are:

* :class:`FOCPMSM` – field-oriented control of the permanent-magnet
  synchronous machine (rotor position directly available, ``i_d* = 0``).
* :class:`FOCInduction` – indirect rotor-flux-oriented control (IRFOC) of the
  induction machine with slip/field-angle computation.

Cascade structure in both cases:

    Speed controller (PI)  ->  i_q*  ->  Current controller (PI)  ->  v_d, v_q  ->  SVPWM
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
    """Field-oriented control of the permanent-magnet synchronous machine.

    Parameters
    ----------
    motor:
        The :class:`~frequenzumrichter.motor.PMSM` to be controlled. The
        machine parameters are used for the decoupling feedforward.
    speed_pi:
        Speed controller (PI). The control output is the reference q current.
    id_pi, iq_pi:
        Current controllers (PI) for the d and q axes. The control output is the dq voltage.
    decoupling:
        Enables the decoupling feedforward of the voltage equations.
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

        # Transform the measurement into the rotor (dq) reference frame
        i_d, i_q = park(meas.i_alpha, meas.i_beta, theta_e)

        # Outer loop: speed controller provides the reference q current
        i_q_ref = self.speed_pi.step(speed_ref - meas.omega_m, dt)
        i_d_ref = 0.0  # surface-mounted PMSM: no field-weakening operation

        # Decoupling feedforward
        ff_d = -omega_e * self.motor.l_q * i_q if self.decoupling else 0.0
        ff_q = (
            omega_e * (self.motor.l_d * i_d + self.motor.flux_linkage)
            if self.decoupling
            else 0.0
        )

        # Inner loop: current controllers
        v_d = self.id_pi.step(i_d_ref - i_d, dt, feedforward=ff_d)
        v_q = self.iq_pi.step(i_q_ref - i_q, dt, feedforward=ff_q)

        # Voltage limit to the voltage limit circle
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
    """Indirect rotor-flux-oriented control (IRFOC) of the induction machine.

    The field angle is not measured but computed from the slip relationship
    (hence *indirect*):

    .. math::

        \\omega_{sl} = \\frac{i_q^{*}}{\\tau_r \\, i_d^{*}}, \\qquad
        \\theta_{field} = \\int (p\\,\\omega_m + \\omega_{sl})\\, dt

    The d-current setpoint sets the rotor flux
    (``i_d* = ψ_r,ref / L_m``), the q-current setpoint comes from the
    speed controller.

    Parameters
    ----------
    motor:
        The :class:`~frequenzumrichter.motor.InductionMotor` to be controlled.
    speed_pi, id_pi, iq_pi:
        PI controllers for speed and current control, respectively.
    flux_ref:
        Reference rotor flux [Wb]. Together with ``L_m`` it determines the d current.
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

        # Reference d current from the desired rotor flux
        i_d_ref = self.flux_ref / l_m
        # Reference q current from the speed controller
        i_q_ref = self.speed_pi.step(speed_ref - meas.omega_m, dt)

        # Slip angular frequency and field angle (indirect orientation)
        omega_sl = i_q_ref / (tau_r * i_d_ref) if abs(i_d_ref) > 1e-9 else 0.0
        omega_e = p * meas.omega_m + omega_sl
        self.theta_field += omega_e * dt
        self.theta_field = float(np.mod(self.theta_field, 2.0 * np.pi))

        # Transform the measurement into the field-oriented dq frame
        i_d, i_q = park(meas.i_alpha, meas.i_beta, self.theta_field)

        # Current controllers
        v_d = self.id_pi.step(i_d_ref - i_d, dt)
        v_q = self.iq_pi.step(i_q_ref - i_q, dt)

        # Voltage limit to the voltage limit circle
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
