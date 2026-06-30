"""Beispiel 2: Feldorientierte Regelung einer PMSM – Drehzahlsprung und Reversierung.

Demonstriert die hohe Dynamik der Vektorregelung: Sollwertsprünge werden in
wenigen Millisekunden ausgeregelt, eine Drehrichtungsumkehr erfolgt sauber durch
den Nulldurchgang.

Aufruf:  python examples/02_foc_pmsm_drehzahlsprung.py
"""

from __future__ import annotations

import matplotlib.pyplot as plt

from _common import savefig
from frequenzumrichter import build_foc_pmsm_drive


def speed_profile(t: float) -> float:
    if t < 0.15:
        return 100.0
    if t < 0.35:
        return 200.0
    return -150.0  # Reversierung


def main() -> None:
    print("Beispiel 2: FOC-PMSM mit Drehzahlsprüngen und Reversierung")
    fu = build_foc_pmsm_drive(with_protection=False)

    result = fu.run(t_end=0.6, speed_ref=speed_profile, load_torque=0.5)

    print(f"  Enddrehzahl  : {result.speed[-1]:7.2f} rad/s (Soll -150)")
    print(f"  Spitzenstrom : {result.current_magnitude.max():7.2f} A")

    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    axes[0].plot(result.time, result.speed, label="Istdrehzahl")
    axes[0].plot(result.time, result.speed_ref, "--", label="Solldrehzahl")
    axes[0].set_ylabel("ω [rad/s]")
    axes[0].set_title("FOC-PMSM: Drehzahlsprünge und Reversierung")
    axes[0].legend(loc="upper right")
    axes[0].grid(True)

    axes[1].plot(result.time, result.i_d, label="i_d (Fluss)")
    axes[1].plot(result.time, result.i_q, label="i_q (Moment)", color="tab:red")
    axes[1].set_ylabel("i_dq [A]")
    axes[1].legend(loc="upper right")
    axes[1].grid(True)

    axes[2].plot(result.time, result.torque, color="tab:green", label="Drehmoment")
    axes[2].set_ylabel("M [Nm]")
    axes[2].set_xlabel("Zeit [s]")
    axes[2].legend(loc="upper right")
    axes[2].grid(True)

    savefig(fig, "02_foc_pmsm_drehzahlsprung.png")


if __name__ == "__main__":
    main()
