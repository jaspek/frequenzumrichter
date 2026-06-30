"""Protection functions of the variable-frequency drive.

A real VFD continuously monitors critical quantities and trips when a limit is
violated (*trip*) in order to protect the device and the machine.
This module reproduces the most important protection functions:

* **Overcurrent** (short-circuit / overload protection)
* **Overvoltage** in the DC link (e.g. during regenerative braking)
* **Undervoltage** in the DC link (mains failure)
* **Overspeed**
* **Thermal overload** of the motor via an ``I²t`` model
"""

from __future__ import annotations

import enum
from dataclasses import dataclass, field

__all__ = ["FaultType", "ProtectionLimits", "Protection"]


class FaultType(enum.Enum):
    """Possible fault causes that lead to a trip."""

    OVERCURRENT = "Overcurrent"
    OVERVOLTAGE = "DC-link overvoltage"
    UNDERVOLTAGE = "DC-link undervoltage"
    OVERSPEED = "Overspeed"
    THERMAL_OVERLOAD = "Thermal overload (I²t)"


@dataclass
class ProtectionLimits:
    """Limit values for the protection functions.

    Parameters
    ----------
    max_current:
        Peak-current limit (magnitude of the current space vector) [A].
    max_dc_voltage, min_dc_voltage:
        Permissible range of the DC-link voltage [V].
    max_speed:
        Magnitude limit of the mechanical speed [rad/s].
    thermal_current:
        Continuous current (rated current), above which the thermal model
        charges up [A].
    thermal_time_constant:
        Thermal time constant of the ``I²t`` model [s].
    thermal_trip_level:
        Threshold of the (normalized) thermal load at which the drive trips.
        ``1.0`` corresponds to 100 % of the permissible heating.
    """

    max_current: float = 50.0
    max_dc_voltage: float = 800.0
    min_dc_voltage: float = 350.0
    max_speed: float = 400.0
    thermal_current: float = 10.0
    thermal_time_constant: float = 10.0
    thermal_trip_level: float = 1.0


@dataclass
class Protection:
    """Monitors measured quantities and triggers a trip when a limit is violated.

    As long as :attr:`tripped` is ``False``, the drive operates normally. After
    a trip the state is retained until :meth:`reset` is called – just as a real
    drive only restarts after acknowledgement.
    """

    limits: ProtectionLimits = field(default_factory=ProtectionLimits)
    tripped: bool = False
    faults: list[FaultType] = field(default_factory=list)
    thermal_state: float = 0.0  # normalized heating (I²t)

    def reset(self) -> None:
        """Acknowledges all faults and re-enables the drive."""
        self.tripped = False
        self.faults = []
        self.thermal_state = 0.0

    def _update_thermal(self, current_magnitude: float, dt: float) -> None:
        """Updates the I²t heating model (PT1-like)."""
        lim = self.limits
        # driving quantity: (I/I_rated)² - 1; above rated current -> heating
        drive = (current_magnitude / lim.thermal_current) ** 2 - 1.0
        self.thermal_state += drive * dt / lim.thermal_time_constant
        if self.thermal_state < 0.0:
            self.thermal_state = 0.0

    def check(
        self,
        current_magnitude: float,
        dc_voltage: float,
        speed: float,
        dt: float,
    ) -> bool:
        """Checks all protection limits for one time step.

        Parameters
        ----------
        current_magnitude:
            Magnitude of the stator current space vector [A].
        dc_voltage:
            DC-link voltage [V].
        speed:
            Mechanical speed [rad/s].
        dt:
            Time step [s] (for the thermal model).

        Returns
        -------
        bool
            ``True`` if the drive is (still) tripped.
        """
        lim = self.limits
        self._update_thermal(current_magnitude, dt)

        faults: list[FaultType] = []
        if current_magnitude > lim.max_current:
            faults.append(FaultType.OVERCURRENT)
        if dc_voltage > lim.max_dc_voltage:
            faults.append(FaultType.OVERVOLTAGE)
        if dc_voltage < lim.min_dc_voltage:
            faults.append(FaultType.UNDERVOLTAGE)
        if abs(speed) > lim.max_speed:
            faults.append(FaultType.OVERSPEED)
        if self.thermal_state > lim.thermal_trip_level:
            faults.append(FaultType.THERMAL_OVERLOAD)

        if faults:
            self.tripped = True
            # add new faults, preserving order and uniqueness
            for f in faults:
                if f not in self.faults:
                    self.faults.append(f)
        return self.tripped
