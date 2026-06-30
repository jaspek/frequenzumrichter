"""Example 1: Soft start of an induction machine with V/f (scalar) control.

The speed is ramped up via a frequency ramp while a constant load torque is
applied. Speed, torque and stator current are shown.

Usage:  python examples/01_vf_anlauf.py
"""

from __future__ import annotations

import matplotlib.pyplot as plt

from _common import savefig
from frequenzumrichter import build_vf_drive


def main() -> None:
    print("Example 1: V/f soft start (induction machine)")
    fu = build_vf_drive(with_protection=False)

    target = 150.0  # rad/s mechanical
    result = fu.run(t_end=2.5, speed_ref=target, load_torque=3.0)

    print(f"  Final speed : {result.speed[-1]:7.2f} rad/s (reference {target}, minus slip)")
    print(f"  Final torque: {result.torque[-1]:7.2f} Nm")
    print(f"  Peak current: {result.current_magnitude.max():7.2f} A")

    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    axes[0].plot(result.time, result.speed, label="Actual speed")
    axes[0].plot(result.time, result.speed_ref, "--", label="Speed reference")
    axes[0].set_ylabel("ω [rad/s]")
    axes[0].set_title("V/f soft start of the induction machine")
    axes[0].legend(loc="lower right")
    axes[0].grid(True)

    axes[1].plot(result.time, result.torque, color="tab:red", label="Torque")
    axes[1].plot(result.time, result.load_torque, "--", color="k", label="Load torque")
    axes[1].set_ylabel("M [Nm]")
    axes[1].legend(loc="upper right")
    axes[1].grid(True)

    axes[2].plot(result.time, result.i_a, label="i_a")
    axes[2].plot(result.time, result.i_b, label="i_b", alpha=0.7)
    axes[2].plot(result.time, result.i_c, label="i_c", alpha=0.7)
    axes[2].set_ylabel("i [A]")
    axes[2].set_xlabel("Time [s]")
    axes[2].legend(loc="upper right", ncol=3)
    axes[2].grid(True)

    savefig(fig, "01_vf_anlauf.png")


if __name__ == "__main__":
    main()
