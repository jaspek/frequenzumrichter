"""Power stage of the variable-frequency drive: rectifier, DC link, inverter.

The classic voltage-source DC-link converter (voltage-link converter) consists
of three stages:

1. **Rectifier** (:class:`Rectifier`) – converts the feeding three-phase grid
   into a DC voltage. Modeled here as an uncontrolled 6-pulse diode bridge.
2. **DC link** (:class:`DCLink`) – smooths the DC voltage via a capacitor and
   buffers energy.
3. **Inverter** (:class:`Inverter`) – generates a three-phase voltage system of
   variable frequency and amplitude from the DC voltage via PWM.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .pwm import inverter_voltages, space_vector_pwm
from .transforms import clarke

__all__ = ["Rectifier", "DCLink", "Inverter", "InverterOutput"]


@dataclass
class Rectifier:
    """Uncontrolled 6-pulse diode bridge (B6).

    From the line-to-line grid voltage ``v_ll_rms`` it provides the average
    no-load DC-link voltage. On average, for the B6 bridge:

    .. math::

        V_{dc} \\approx \\frac{3\\sqrt{2}}{\\pi}\\, V_{LL,rms} \\approx 1.35\\, V_{LL,rms}

    Parameters
    ----------
    v_ll_rms:
        Line-to-line grid RMS voltage [V] (e.g. 400 V).
    """

    v_ll_rms: float = 400.0

    AVG_FACTOR: float = field(default=3.0 * np.sqrt(2.0) / np.pi, repr=False)

    @property
    def dc_voltage(self) -> float:
        """Average DC-link voltage of the diode bridge [V]."""
        return self.AVG_FACTOR * self.v_ll_rms

    @property
    def peak_dc_voltage(self) -> float:
        """Peak value (no-load charging of the capacitor) [V]."""
        return np.sqrt(2.0) * self.v_ll_rms


@dataclass
class DCLink:
    """DC-link capacitor with voltage dynamics.

    Models the energy balance ``C · dV_dc/dt = i_source - i_load``. The feeding
    diode bridge can only **supply** current, not absorb it; it is represented
    as a voltage source with internal resistance:

    .. math::

        i_{source} = \\max\\!\\left(0,\\ \\frac{V_{nom} - V_{dc}}{R_i}\\right)

    During regenerative braking (``i_load < 0``) the capacitor therefore charges
    up – an overvoltage arises which, without a braking resistor, leads to a
    trip. If ``stiff=True``, the voltage is assumed to be ideally constant
    (stiff DC link), which is sufficient for many control considerations.

    Parameters
    ----------
    capacitance:
        Capacitance ``C`` [F].
    voltage:
        Initial voltage [V].
    nominal_voltage:
        No-load voltage of the feeding bridge [V].
    source_resistance:
        Internal resistance of the source ``R_i`` [Ω].
    stiff:
        If ``True``, the voltage remains constant.
    """

    capacitance: float = 1e-3
    voltage: float = 540.0
    nominal_voltage: float = 540.0
    source_resistance: float = 0.5
    stiff: bool = True

    def reset(self, voltage: float | None = None) -> None:
        self.voltage = self.nominal_voltage if voltage is None else voltage

    def source_current(self) -> float:
        """Current supplied by the rectifier (positive only)."""
        return max(0.0, (self.nominal_voltage - self.voltage) / self.source_resistance)

    def update(self, i_load: float, dt: float) -> float:
        """Updates the DC-link voltage by one time step.

        ``i_load`` is the DC current drawn by the inverter (negative during
        regeneration / generator operation).
        """
        if self.stiff:
            return self.voltage
        i_source = self.source_current()
        self.voltage += (i_source - i_load) / self.capacitance * dt
        # physical lower bound
        if self.voltage < 0.0:
            self.voltage = 0.0
        return self.voltage


@dataclass
class InverterOutput:
    """Result of one inverter step."""

    v_alpha: float
    v_beta: float
    v_abc: tuple[float, float, float]
    duties: tuple[float, float, float]
    i_dc: float  # DC-link current estimated from the power balance [A]


@dataclass
class Inverter:
    """Two-level voltage-source inverter (averaged model) with SVPWM.

    The inverter receives a reference voltage space vector ``(v_alpha_ref,
    v_beta_ref)`` and converts it into duty cycles via space-vector modulation.
    Because the duty cycles are limited (overmodulation), the actually applied
    space vector may turn out smaller; this *actual* vector is returned.

    Parameters
    ----------
    deadtime_compensation:
        Placeholder flag; the averaged model neglects the interlock time
        (dead time) of the bridge. For detailed studies a correction can be
        added here.
    """

    deadtime_compensation: bool = False

    def modulate(
        self,
        v_alpha_ref: float,
        v_beta_ref: float,
        v_dc: float,
        i_alpha: float = 0.0,
        i_beta: float = 0.0,
    ) -> InverterOutput:
        """Converts the reference voltage space vector into bridge control.

        Parameters
        ----------
        v_alpha_ref, v_beta_ref:
            Reference voltage space vector [V].
        v_dc:
            Current DC-link voltage [V].
        i_alpha, i_beta:
            Current stator current space vector [A] – used to estimate the
            drawn DC-link current (power balance).
        """
        duties, v_alpha_act, v_beta_act = space_vector_pwm(
            v_alpha_ref, v_beta_ref, v_dc
        )
        v_abc = inverter_voltages(*duties, v_dc)

        # Power balance: P_ac = 1.5 (vα iα + vβ iβ); i_dc = P_ac / V_dc
        p_ac = 1.5 * (v_alpha_act * i_alpha + v_beta_act * i_beta)
        i_dc = p_ac / v_dc if v_dc > 1e-9 else 0.0

        return InverterOutput(
            v_alpha=float(v_alpha_act),
            v_beta=float(v_beta_act),
            v_abc=(float(v_abc[0]), float(v_abc[1]), float(v_abc[2])),
            duties=(float(duties[0]), float(duties[1]), float(duties[2])),
            i_dc=float(i_dc),
        )
