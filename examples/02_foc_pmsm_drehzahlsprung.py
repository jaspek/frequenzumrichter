"""Example 2: Field-oriented control of a PMSM – speed step and reversal.

Demonstrates the high dynamics of vector control: setpoint steps are settled
within a few milliseconds, and a direction reversal passes cleanly through the
zero crossing.

Usage:  python examples/02_foc_pmsm_drehzahlsprung.py
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
    return -150.0  # reversal


def main() -> None:
    print("Example 2: FOC-PMSM with speed steps and reversal")
    fu = build_foc_pmsm_drive(with_protection=False)

    result = fu.run(t_end=0.6, speed_ref=speed_profile, load_torque=0.5)

    print(f"  Final speed  : {result.speed[-1]:7.2f} rad/s (ref -150)")
    print(f"  Peak current : {result.current_magnitude.max():7.2f} A")

    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    axes[0].plot(result.time, result.speed, label="Actual speed")
    axes[0].plot(result.time, result.speed_ref, "--", label="Reference speed")
    axes[0].set_ylabel("ω [rad/s]")
    axes[0].set_title("FOC-PMSM: speed steps and reversal")
    axes[0].legend(loc="upper right")
    axes[0].grid(True)

    axes[1].plot(result.time, result.i_d, label="i_d (flux)")
    axes[1].plot(result.time, result.i_q, label="i_q (torque)", color="tab:red")
    axes[1].set_ylabel("i_dq [A]")
    axes[1].legend(loc="upper right")
    axes[1].grid(True)

    axes[2].plot(result.time, result.torque, color="tab:green", label="Torque")
    axes[2].set_ylabel("M [Nm]")
    axes[2].set_xlabel("Time [s]")
    axes[2].legend(loc="upper right")
    axes[2].grid(True)

    savefig(fig, "02_foc_pmsm_drehzahlsprung.png")


if __name__ == "__main__":
    main()
