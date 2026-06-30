"""Tests for the Clarke/Park transformations."""

import numpy as np
import pytest

from frequenzumrichter.transforms import (
    abc_to_dq,
    clarke,
    dq_to_abc,
    inverse_clarke,
    inverse_park,
    park,
)


def balanced_abc(amplitude, theta):
    a = amplitude * np.cos(theta)
    b = amplitude * np.cos(theta - 2 * np.pi / 3)
    c = amplitude * np.cos(theta + 2 * np.pi / 3)
    return a, b, c


@pytest.mark.parametrize("theta", np.linspace(0, 2 * np.pi, 13))
def test_clarke_amplitude_invariant(theta):
    """For a balanced system, the amplitude is preserved in the αβ frame."""
    amp = 7.5
    a, b, c = balanced_abc(amp, theta)
    alpha, beta = clarke(a, b, c)
    assert alpha == pytest.approx(amp * np.cos(theta), abs=1e-9)
    assert beta == pytest.approx(amp * np.sin(theta), abs=1e-9)
    assert np.hypot(alpha, beta) == pytest.approx(amp, abs=1e-9)


def test_clarke_roundtrip():
    a, b, c = balanced_abc(3.0, 0.9)
    alpha, beta = clarke(a, b, c)
    a2, b2, c2 = inverse_clarke(alpha, beta)
    assert (a2, b2, c2) == pytest.approx((a, b, c), abs=1e-9)


@pytest.mark.parametrize("theta", [0.0, 0.5, 1.7, 3.3, 6.0])
def test_park_roundtrip(theta):
    alpha, beta = 1.3, -2.1
    d, q = park(alpha, beta, theta)
    a2, b2 = inverse_park(d, q, theta)
    assert (a2, b2) == pytest.approx((alpha, beta), abs=1e-12)


def test_abc_to_dq_constant_for_synchronous_frame():
    """A system rotating at ω appears constant in the co-rotating dq frame."""
    amp = 5.0
    for theta in np.linspace(0, 2 * np.pi, 17):
        a, b, c = balanced_abc(amp, theta)
        d, q = abc_to_dq(a, b, c, theta)
        assert d == pytest.approx(amp, abs=1e-9)
        assert q == pytest.approx(0.0, abs=1e-9)


def test_dq_to_abc_roundtrip():
    for theta in [0.0, 1.1, 2.5, 5.9]:
        d, q = 4.0, 1.0
        a, b, c = dq_to_abc(d, q, theta)
        d2, q2 = abc_to_dq(a, b, c, theta)
        assert (d2, q2) == pytest.approx((d, q), abs=1e-9)


def test_transforms_vectorized():
    """The functions must also operate element-wise on numpy arrays."""
    theta = np.linspace(0, 2 * np.pi, 50)
    a, b, c = balanced_abc(2.0, theta)
    alpha, beta = clarke(a, b, c)
    assert alpha.shape == theta.shape
    np.testing.assert_allclose(np.hypot(alpha, beta), 2.0, atol=1e-9)
