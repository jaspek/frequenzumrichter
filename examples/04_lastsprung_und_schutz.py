"""Example 4: Load disturbance and thermal overload protection (FOC induction).

First a load step is rejected (disturbance rejection), then the machine is
continuously overloaded until the I²t thermal model trips the converter.

Run:  python examples/04_lastsprung_und_schutz.py
"""

from __future__ import annotations

import numpy as np

import matplotlib.pyplot as plt

from _common import savefig
from frequenzumrichter import build_foc_induction_drive
from frequenzumrichter.motor import InductionMotor, MotorMeasurements


def main() -> None:
    print("Example 4: Load disturbance and thermal overload protection")
    motor = InductionMotor()
    fu = build_foc_induction_drive(motor=motor, max_current=30.0, with_protection=True)
    # Make the thermal model more sensitive so the trip becomes visible within the time window
    fu.protection.limits.thermal_current = 8.0
    fu.protection.limits.thermal_time_constant = 0.4

    def load(t: float, meas: MotorMeasurements) -> float:
        # Base load, or overload from 0.6 s on; modeled as a *passive* load
        # (vanishes at standstill -> no spin-up after the trip).
        nominal = 1.0 if t < 0.6 else 14.0
        return nominal * float(np.tanh(meas.omega_m / 5.0))

    result = fu.run(t_end=2.0, speed_ref=120.0, load_torque=load)

    tripped = bool(result.tripped[-1])
    print(f"  Tripped      : {tripped}")
    print(f"  Faults       : {[f.value for f in fu.protection.faults]}")

    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    axes[0].plot(result.time, result.speed, label="Actual speed")
    axes[0].plot(result.time, result.speed_ref, "--", label="Speed reference")
    axes[0].set_ylabel("ω [rad/s]")
    axes[0].set_title("Load disturbance and thermal trip (IRFOC)")
    axes[0].legend(loc="lower right")
    axes[0].grid(True)

    axes[1].plot(result.time, result.current_magnitude, color="tab:red",
                 label="|i_s| (current vector)")
    axes[1].plot(result.time, result.load_torque, "--", color="k", label="Load torque")
    axes[1].set_ylabel("|i| [A] / M [Nm]")
    axes[1].legend(loc="upper left")
    axes[1].grid(True)

    axes[2].plot(result.time, result.thermal_state, color="tab:orange",
                 label="thermal load (I²t)")
    axes[2].axhline(1.0, color="r", ls=":", label="Trip threshold")
    axes[2].set_ylabel("Heating [-]")
    axes[2].set_xlabel("Time [s]")
    axes[2].legend(loc="upper left")
    axes[2].grid(True)

    savefig(fig, "04_lastsprung_und_schutz.png")


if __name__ == "__main__":
    main()
