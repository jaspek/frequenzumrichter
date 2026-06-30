"""Example 3: Space-vector modulation (SVPWM) made intuitive.

Shows the three duty cycles over one fundamental period as well as the
characteristic "saddle" (zero-sequence injection) that extends the linear
modulation range compared to pure sinusoidal PWM.

Usage:  python examples/03_svpwm_demo.py
"""

from __future__ import annotations

import numpy as np

import matplotlib.pyplot as plt

from _common import savefig
from frequenzumrichter.pwm import (
    MAX_LINEAR_SVPWM,
    sinusoidal_pwm,
    svpwm_duty,
)
from frequenzumrichter.transforms import inverse_clarke


def main() -> None:
    print("Example 3: SVPWM – duty cycles and extended modulation range")
    v_dc = 540.0
    amp = 0.95 * MAX_LINEAR_SVPWM * v_dc  # close to the linear limit
    theta = np.linspace(0.0, 2.0 * np.pi, 720)

    v_alpha = amp * np.cos(theta)
    v_beta = amp * np.sin(theta)

    d_sv = np.array([svpwm_duty(a, b, v_dc) for a, b in zip(v_alpha, v_beta)])

    # Comparison with sinusoidal PWM (same phase voltages)
    v_a, v_b, v_c = inverse_clarke(v_alpha, v_beta)
    d_sin = np.array(
        [sinusoidal_pwm(a, b, c, v_dc) for a, b, c in zip(v_a, v_b, v_c)]
    )

    print(f"  SVPWM duty-cycle range: [{d_sv.min():.3f}, {d_sv.max():.3f}]")
    print(f"  Sinusoidal-PWM duty-cycle range: [{d_sin.min():.3f}, {d_sin.max():.3f}]")

    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    deg = np.rad2deg(theta)
    axes[0].plot(deg, d_sv[:, 0], label="d_a")
    axes[0].plot(deg, d_sv[:, 1], label="d_b")
    axes[0].plot(deg, d_sv[:, 2], label="d_c")
    axes[0].axhline(0.0, color="k", lw=0.5)
    axes[0].axhline(1.0, color="k", lw=0.5)
    axes[0].set_title("SVPWM duty cycles (with zero-sequence 'saddle')")
    axes[0].set_ylabel("Duty cycle")
    axes[0].legend(loc="upper right", ncol=3)
    axes[0].grid(True)

    axes[1].plot(deg, d_sv[:, 0], label="SVPWM d_a")
    axes[1].plot(deg, d_sin[:, 0], "--", label="Sinusoidal PWM d_a")
    axes[1].set_title("Phase a: SVPWM vs. sinusoidal PWM")
    axes[1].set_ylabel("Duty cycle")
    axes[1].set_xlabel("Fundamental angle [°]")
    axes[1].legend(loc="upper right")
    axes[1].grid(True)

    savefig(fig, "03_svpwm_demo.png")


if __name__ == "__main__":
    main()
