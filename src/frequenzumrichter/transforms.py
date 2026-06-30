"""Space-vector transforms (Clarke / Park) for rotating-field control.

This module provides the standard coordinate transforms required in every
field-oriented controlled variable-frequency drive:

* **Clarke transform** ``abc -> αβ`` (3-phase -> stationary 2-axis frame)
* **Park transform** ``αβ -> dq`` (stationary -> rotating 2-axis frame)

The *amplitude-invariant* form (factor ``2/3``) is used, i.e. the amplitude of
a balanced three-phase system is preserved in the αβ/dq frame.

All functions work with both scalars (``float``) and ``numpy`` arrays
(element-wise), so that entire time series can be transformed in a vectorized
manner.

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
    """Clarke transform ``abc -> αβ`` (amplitude-invariant).

    Parameters
    ----------
    a, b, c:
        The three phase quantities (current or voltage). Scalars or arrays.

    Returns
    -------
    (alpha, beta):
        The two components in the stationary αβ frame.

    Notes
    -----
    For a balanced system (``a + b + c == 0``), ``alpha == a`` holds.
    """
    alpha = (2.0 / 3.0) * (a - 0.5 * b - 0.5 * c)
    beta = (1.0 / SQRT3) * (b - c)
    return alpha, beta


def inverse_clarke(alpha, beta):
    """Inverse Clarke transform ``αβ -> abc``.

    Returns
    -------
    (a, b, c):
        The reconstructed phase quantities (zero-sequence is assumed to be 0).
    """
    a = alpha
    b = -0.5 * alpha + SQRT3_2 * beta
    c = -0.5 * alpha - SQRT3_2 * beta
    return a, b, c


def park(alpha, beta, theta):
    """Park transform ``αβ -> dq``.

    Parameters
    ----------
    alpha, beta:
        Components in the stationary frame.
    theta:
        Electrical angle of the rotating reference frame [rad].

    Returns
    -------
    (d, q):
        Components in the rotating dq frame.
    """
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    d = alpha * cos_t + beta * sin_t
    q = -alpha * sin_t + beta * cos_t
    return d, q


def inverse_park(d, q, theta):
    """Inverse Park transform ``dq -> αβ``."""
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    alpha = d * cos_t - q * sin_t
    beta = d * sin_t + q * cos_t
    return alpha, beta


def abc_to_dq(a, b, c, theta):
    """Direct path ``abc -> dq`` (Clarke followed by Park)."""
    alpha, beta = clarke(a, b, c)
    return park(alpha, beta, theta)


def dq_to_abc(d, q, theta):
    """Direct path ``dq -> abc`` (inverse Park followed by inverse Clarke)."""
    alpha, beta = inverse_park(d, q, theta)
    return inverse_clarke(alpha, beta)
