"""Pulse-width modulation (PWM) for the inverter bridge.

Implements the two most common modulation methods used in
variable-frequency drives:

* **Sinusoidal PWM (SPWM)** – comparison of three sinusoidal references with a
  triangular carrier.
* **Space-vector modulation (SVPWM)** – realized via the mathematically
  equivalent *min/max* zero-sequence injection. This extends the linear
  modulation range compared to pure sinusoidal PWM by a factor of
  ``2/√3 ≈ 1.155`` (phase-voltage amplitude up to ``V_dc/√3``).

The functions return *duty cycles* in the range
``[0, 1]`` for the three upper bridge transistors. From these, the average
phase voltage (averaged model) can be recovered with
:func:`inverter_voltages`.
"""

from __future__ import annotations

import numpy as np

from .transforms import clarke, inverse_clarke

__all__ = [
    "sinusoidal_pwm",
    "space_vector_pwm",
    "svpwm_duty",
    "inverter_voltages",
    "svpwm_sector",
    "MAX_LINEAR_SVPWM",
    "MAX_LINEAR_SPWM",
]

#: Maximum phase-voltage amplitude in the linear range of SVPWM (= V_dc/√3).
MAX_LINEAR_SVPWM = 1.0 / np.sqrt(3.0)
#: Maximum phase-voltage amplitude in the linear range of sinusoidal PWM (= V_dc/2).
MAX_LINEAR_SPWM = 0.5


def sinusoidal_pwm(v_a, v_b, v_c, v_dc):
    """Sinusoidal PWM: computes duty cycles from the reference phase voltages.

    Parameters
    ----------
    v_a, v_b, v_c:
        Reference phase voltages (relative to the virtual neutral point) [V].
    v_dc:
        DC-link voltage [V].

    Returns
    -------
    (d_a, d_b, d_c):
        Duty cycles limited to ``[0, 1]``.
    """
    d_a = np.clip(0.5 + v_a / v_dc, 0.0, 1.0)
    d_b = np.clip(0.5 + v_b / v_dc, 0.0, 1.0)
    d_c = np.clip(0.5 + v_c / v_dc, 0.0, 1.0)
    return d_a, d_b, d_c


def svpwm_duty(v_alpha, v_beta, v_dc):
    """Space-vector modulation via min/max injection.

    Parameters
    ----------
    v_alpha, v_beta:
        Reference voltage space vector in the stationary αβ frame [V].
    v_dc:
        DC-link voltage [V].

    Returns
    -------
    (d_a, d_b, d_c):
        Duty cycles in the range ``[0, 1]``.

    Notes
    -----
    Injecting the zero-sequence component ``v_off = -(max + min) / 2`` centers the
    phase voltages symmetrically around ``V_dc/2`` and corresponds exactly to
    the classic 7-segment space-vector modulation (SVPWM).
    """
    v_a, v_b, v_c = inverse_clarke(v_alpha, v_beta)
    v_max = np.maximum(np.maximum(v_a, v_b), v_c)
    v_min = np.minimum(np.minimum(v_a, v_b), v_c)
    v_off = -0.5 * (v_max + v_min)
    d_a = np.clip(0.5 + (v_a + v_off) / v_dc, 0.0, 1.0)
    d_b = np.clip(0.5 + (v_b + v_off) / v_dc, 0.0, 1.0)
    d_c = np.clip(0.5 + (v_c + v_off) / v_dc, 0.0, 1.0)
    return d_a, d_b, d_c


def space_vector_pwm(v_alpha, v_beta, v_dc):
    """Like :func:`svpwm_duty`, but additionally returns the realized αβ voltages.

    Due to the limiting of the duty cycles to ``[0, 1]`` (overmodulation),
    the actually applied voltage may deviate from the reference voltage.
    This *actual* voltage is also returned so that the control knows the
    voltage space vector that is actually effective at the motor.

    Returns
    -------
    (duties, v_alpha_act, v_beta_act):
        ``duties`` is the tuple ``(d_a, d_b, d_c)``; ``v_*_act`` is the
        actually applied voltage space vector [V].
    """
    duties = svpwm_duty(v_alpha, v_beta, v_dc)
    v_a, v_b, v_c = inverter_voltages(*duties, v_dc)
    v_alpha_act, v_beta_act = clarke(v_a, v_b, v_c)
    return duties, v_alpha_act, v_beta_act


def inverter_voltages(d_a, d_b, d_c, v_dc):
    """Average phase voltages from duty cycles (averaged model).

    The voltage of a bridge leg relative to the negative DC-link pole is
    ``d_x * V_dc``. For a symmetric load (isolated neutral point), the
    phase-to-neutral voltage results from subtracting the mean value.

    Returns
    -------
    (v_a, v_b, v_c):
        Phase voltages relative to the load neutral point [V].
    """
    v_a0 = d_a * v_dc
    v_b0 = d_b * v_dc
    v_c0 = d_c * v_dc
    v_n = (v_a0 + v_b0 + v_c0) / 3.0
    return v_a0 - v_n, v_b0 - v_n, v_c0 - v_n


def svpwm_sector(v_alpha, v_beta):
    """Determines the sector (1..6) of the voltage space vector in the αβ diagram.

    Primarily serves to illustrate the space-vector modulation.
    """
    angle = np.arctan2(v_beta, v_alpha)
    angle = np.mod(angle, 2.0 * np.pi)
    sector = int(np.floor(angle / (np.pi / 3.0))) + 1
    return sector
