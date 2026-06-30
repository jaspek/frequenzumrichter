"""Raum­zeiger-Transformationen (Clarke / Park) für die Drehfeldregelung.

Dieses Modul stellt die Standard-Koordinatentransformationen bereit, die in
jedem feldorientiert geregelten Frequenzumrichter benötigt werden:

* **Clarke-Transformation** ``abc -> αβ`` (3-phasig -> stationäres 2-Achsen-System)
* **Park-Transformation** ``αβ -> dq`` (stationär -> rotierendes 2-Achsen-System)

Es wird die *amplituden-invariante* Form (Faktor ``2/3``) verwendet, d.h. die
Amplitude eines symmetrischen Drehstromsystems bleibt im αβ-/dq-System erhalten.

Alle Funktionen arbeiten sowohl mit Skalaren (``float``) als auch mit
``numpy``-Arrays (elementweise), sodass ganze Zeitreihen vektorisiert
transformiert werden können.

English summary
---------------
Space-vector (Clarke/Park) transforms used by the field-oriented control.
The amplitude-invariant convention (``2/3`` scaling) is used throughout.
"""

from __future__ import annotations

import numpy as np

__all__ = [
    "clarke",
    "inverse_clarke",
    "park",
    "inverse_park",
    "abc_to_dq",
    "dq_to_abc",
    "SQRT3",
    "SQRT3_2",
]

SQRT3 = float(np.sqrt(3.0))
SQRT3_2 = SQRT3 / 2.0


def clarke(a, b, c):
    """Clarke-Transformation ``abc -> αβ`` (amplituden-invariant).

    Parameters
    ----------
    a, b, c:
        Die drei Phasengrößen (Strom oder Spannung). Skalare oder Arrays.

    Returns
    -------
    (alpha, beta):
        Die beiden Komponenten im stationären αβ-System.

    Notes
    -----
    Für ein symmetrisches System (``a + b + c == 0``) gilt ``alpha == a``.
    """
    alpha = (2.0 / 3.0) * (a - 0.5 * b - 0.5 * c)
    beta = (1.0 / SQRT3) * (b - c)
    return alpha, beta


def inverse_clarke(alpha, beta):
    """Inverse Clarke-Transformation ``αβ -> abc``.

    Returns
    -------
    (a, b, c):
        Die rekonstruierten Phasengrößen (Nullsystem wird zu 0 angenommen).
    """
    a = alpha
    b = -0.5 * alpha + SQRT3_2 * beta
    c = -0.5 * alpha - SQRT3_2 * beta
    return a, b, c


def park(alpha, beta, theta):
    """Park-Transformation ``αβ -> dq``.

    Parameters
    ----------
    alpha, beta:
        Komponenten im stationären System.
    theta:
        Elektrischer Winkel des rotierenden Bezugssystems [rad].

    Returns
    -------
    (d, q):
        Komponenten im rotierenden dq-System.
    """
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    d = alpha * cos_t + beta * sin_t
    q = -alpha * sin_t + beta * cos_t
    return d, q


def inverse_park(d, q, theta):
    """Inverse Park-Transformation ``dq -> αβ``."""
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    alpha = d * cos_t - q * sin_t
    beta = d * sin_t + q * cos_t
    return alpha, beta


def abc_to_dq(a, b, c, theta):
    """Direkter Weg ``abc -> dq`` (Clarke gefolgt von Park)."""
    alpha, beta = clarke(a, b, c)
    return park(alpha, beta, theta)


def dq_to_abc(d, q, theta):
    """Direkter Weg ``dq -> abc`` (inverse Park gefolgt von inverser Clarke)."""
    alpha, beta = inverse_park(d, q, theta)
    return inverse_clarke(alpha, beta)
