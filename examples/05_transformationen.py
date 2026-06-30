"""Beispiel 5: Clarke-/Park-Transformation visualisiert.

Ein symmetrisches Drehstromsystem wird über Clarke (abc -> αβ) und Park
(αβ -> dq) transformiert. Im mitrotierenden dq-System werden die
Wechselgrößen zu Gleichgrößen – die Grundlage der feldorientierten Regelung.

Aufruf:  python examples/05_transformationen.py
"""

from __future__ import annotations

import numpy as np

import matplotlib.pyplot as plt

from _common import savefig
from frequenzumrichter.transforms import clarke, park


def main() -> None:
    print("Beispiel 5: Clarke-/Park-Transformation")
    f = 50.0
    t = np.linspace(0.0, 0.04, 1000)
    theta = 2.0 * np.pi * f * t
    amp = 1.0

    a = amp * np.cos(theta)
    b = amp * np.cos(theta - 2 * np.pi / 3)
    c = amp * np.cos(theta + 2 * np.pi / 3)

    alpha, beta = clarke(a, b, c)
    d, q = park(alpha, beta, theta)

    print(f"  d (≈ konstant): Mittelwert {d.mean():.3f}, Streuung {d.std():.2e}")
    print(f"  q (≈ 0):        Mittelwert {q.mean():.3e}")

    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    axes[0].plot(t * 1e3, a, label="a")
    axes[0].plot(t * 1e3, b, label="b")
    axes[0].plot(t * 1e3, c, label="c")
    axes[0].set_title("Drehstromsystem abc")
    axes[0].set_ylabel("Amplitude")
    axes[0].legend(loc="upper right", ncol=3)
    axes[0].grid(True)

    axes[1].plot(t * 1e3, alpha, label="α")
    axes[1].plot(t * 1e3, beta, label="β")
    axes[1].set_title("Clarke: stationäres αβ-System")
    axes[1].set_ylabel("Amplitude")
    axes[1].legend(loc="upper right")
    axes[1].grid(True)

    axes[2].plot(t * 1e3, d, label="d")
    axes[2].plot(t * 1e3, q, label="q")
    axes[2].set_title("Park: rotierendes dq-System (Gleichgrößen)")
    axes[2].set_ylabel("Amplitude")
    axes[2].set_xlabel("Zeit [ms]")
    axes[2].legend(loc="upper right")
    axes[2].grid(True)

    savefig(fig, "05_transformationen.png")


if __name__ == "__main__":
    main()
