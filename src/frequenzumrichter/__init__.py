"""frequenzumrichter – Simulation of a variable-frequency drive (VFD) in Python.

This package models a complete three-phase drive with a
voltage-source DC-link converter:

* **Power stage** – rectifier, DC link, inverter (SVPWM)
* **Machines** – induction and permanent-magnet synchronous machine
* **Control** – scalar V/f control and field-oriented control (FOC)
* **Protection** – over-/undervoltage, overcurrent, overspeed, I²t thermal model

Quick start
-----------
>>> from frequenzumrichter import build_foc_pmsm_drive
>>> fu = build_foc_pmsm_drive()
>>> result = fu.run(t_end=0.5, speed_ref=100.0, load_torque=0.5)
>>> round(result.speed[-1])
100
"""

from __future__ import annotations

from .control_base import ControlOutput, limit_vector
from .controllers import LowPassFilter, PIController, RateLimiter
from .drive import Frequenzumrichter
from .factory import (
    build_foc_induction_drive,
    build_foc_pmsm_drive,
    build_vf_drive,
)
from .foc import FOCInduction, FOCPMSM
from .motor import PMSM, InductionMotor, MotorMeasurements
from .power_stage import DCLink, Inverter, InverterOutput, Rectifier
from .protection import FaultType, Protection, ProtectionLimits
from .pwm import (
    inverter_voltages,
    sinusoidal_pwm,
    space_vector_pwm,
    svpwm_duty,
    svpwm_sector,
)
from .simulation import SimulationResult, rk4_step
from .transforms import (
    abc_to_dq,
    clarke,
    dq_to_abc,
    inverse_clarke,
    inverse_park,
    park,
)
from .vf_control import VFControl

__version__ = "0.1.0"

__all__ = [
    "__version__",
    # Drive
    "Frequenzumrichter",
    "build_vf_drive",
    "build_foc_pmsm_drive",
    "build_foc_induction_drive",
    # Machines
    "InductionMotor",
    "PMSM",
    "MotorMeasurements",
    # Power stage
    "Rectifier",
    "DCLink",
    "Inverter",
    "InverterOutput",
    # Control
    "VFControl",
    "FOCPMSM",
    "FOCInduction",
    "PIController",
    "RateLimiter",
    "LowPassFilter",
    "ControlOutput",
    "limit_vector",
    # Protection
    "Protection",
    "ProtectionLimits",
    "FaultType",
    # Transforms
    "clarke",
    "inverse_clarke",
    "park",
    "inverse_park",
    "abc_to_dq",
    "dq_to_abc",
    # PWM
    "sinusoidal_pwm",
    "space_vector_pwm",
    "svpwm_duty",
    "svpwm_sector",
    "inverter_voltages",
    # Simulation
    "SimulationResult",
    "rk4_step",
]
