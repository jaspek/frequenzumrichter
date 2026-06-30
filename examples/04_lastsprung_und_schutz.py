"""Beispiel 4: Laststörung und thermischer Überlastschutz (FOC-Asynchron).

Erst wird ein Lastsprung ausgeregelt (Störgrößenverhalten), anschließend wird die
Maschine dauerhaft überlastet, bis das I²t-Thermomodell den Umrichter abschaltet.

Aufruf:  python examples/04_lastsprung_und_schutz.py
"""

from __future__ import annotations

import numpy as np

import matplotlib.pyplot as plt

from _common import savefig
from frequenzumrichter import build_foc_induction_drive
from frequenzumrichter.motor import InductionMotor, MotorMeasurements


def main() -> None:
    print("Beispiel 4: Laststörung und thermischer Überlastschutz")
    motor = InductionMotor()
    fu = build_foc_induction_drive(motor=motor, max_current=30.0, with_protection=True)
    # Thermomodell empfindlicher einstellen, damit die Abschaltung im Zeitfenster sichtbar wird
    fu.protection.limits.thermal_current = 8.0
    fu.protection.limits.thermal_time_constant = 0.4

    def load(t: float, meas: MotorMeasurements) -> float:
        # Grundlast bzw. Überlast ab 0.6 s; als *passive* Last modelliert
        # (verschwindet bei Stillstand -> kein Hochlaufen nach der Abschaltung).
        nominal = 1.0 if t < 0.6 else 14.0
        return nominal * float(np.tanh(meas.omega_m / 5.0))

    result = fu.run(t_end=2.0, speed_ref=120.0, load_torque=load)

    tripped = bool(result.tripped[-1])
    print(f"  Abgeschaltet : {tripped}")
    print(f"  Fehler       : {[f.value for f in fu.protection.faults]}")

    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    axes[0].plot(result.time, result.speed, label="Istdrehzahl")
    axes[0].plot(result.time, result.speed_ref, "--", label="Solldrehzahl")
    axes[0].set_ylabel("ω [rad/s]")
    axes[0].set_title("Laststörung und thermische Abschaltung (IRFOC)")
    axes[0].legend(loc="lower right")
    axes[0].grid(True)

    axes[1].plot(result.time, result.current_magnitude, color="tab:red",
                 label="|i_s| (Stromzeiger)")
    axes[1].plot(result.time, result.load_torque, "--", color="k", label="Lastmoment")
    axes[1].set_ylabel("|i| [A] / M [Nm]")
    axes[1].legend(loc="upper left")
    axes[1].grid(True)

    axes[2].plot(result.time, result.thermal_state, color="tab:orange",
                 label="thermische Belastung (I²t)")
    axes[2].axhline(1.0, color="r", ls=":", label="Abschaltschwelle")
    axes[2].set_ylabel("Erwärmung [-]")
    axes[2].set_xlabel("Zeit [s]")
    axes[2].legend(loc="upper left")
    axes[2].grid(True)

    savefig(fig, "04_lastsprung_und_schutz.png")


if __name__ == "__main__":
    main()
