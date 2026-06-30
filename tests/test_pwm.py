"""Tests for the pulse-width modulation (SPWM / SVPWM)."""

import numpy as np
import pytest

from frequenzumrichter.pwm import (
    MAX_LINEAR_SPWM,
    MAX_LINEAR_SVPWM,
    inverter_voltages,
    sinusoidal_pwm,
    space_vector_pwm,
    svpwm_duty,
    svpwm_sector,
)
from frequenzumrichter.transforms import clarke


def test_duties_within_bounds():
    v_dc = 540.0
    for angle in np.linspace(0, 2 * np.pi, 60):
        v_alpha = 200.0 * np.cos(angle)
        v_beta = 200.0 * np.sin(angle)
        d_a, d_b, d_c = svpwm_duty(v_alpha, v_beta, v_dc)
        for d in (d_a, d_b, d_c):
            assert 0.0 <= d <= 1.0


def test_inverter_voltages_sum_to_zero():
    """Phase voltages referenced to the load neutral point must add up to 0."""
    v_a, v_b, v_c = inverter_voltages(0.8, 0.5, 0.2, 540.0)
    assert v_a + v_b + v_c == pytest.approx(0.0, abs=1e-9)


def test_svpwm_reproduces_reference_in_linear_range():
    """Within the linear range, the SVPWM reproduces the reference vector exactly."""
    v_dc = 540.0
    amp = 0.9 * MAX_LINEAR_SVPWM * v_dc  # within the voltage limit circle
    for angle in np.linspace(0, 2 * np.pi, 40):
        v_alpha = amp * np.cos(angle)
        v_beta = amp * np.sin(angle)
        _duties, v_alpha_act, v_beta_act = space_vector_pwm(v_alpha, v_beta, v_dc)
        assert v_alpha_act == pytest.approx(v_alpha, abs=1e-6)
        assert v_beta_act == pytest.approx(v_beta, abs=1e-6)


def test_svpwm_extends_range_beyond_spwm():
    """SVPWM reaches a higher linear voltage than pure sinusoidal PWM."""
    assert MAX_LINEAR_SVPWM > MAX_LINEAR_SPWM
    # specifically by the factor 2/√3
    assert MAX_LINEAR_SVPWM / MAX_LINEAR_SPWM == pytest.approx(2.0 / np.sqrt(3.0))


def test_spwm_duty_midpoint_is_half():
    """Without a voltage reference, the duty cycle must be 0.5."""
    d_a, d_b, d_c = sinusoidal_pwm(0.0, 0.0, 0.0, 540.0)
    assert (d_a, d_b, d_c) == pytest.approx((0.5, 0.5, 0.5))


@pytest.mark.parametrize(
    "angle_deg,expected_sector",
    [(30, 1), (90, 2), (150, 3), (210, 4), (270, 5), (330, 6)],
)
def test_svpwm_sector(angle_deg, expected_sector):
    angle = np.deg2rad(angle_deg)
    assert svpwm_sector(np.cos(angle), np.sin(angle)) == expected_sector


def test_roundtrip_duty_to_voltage():
    """inverter_voltages and clarke must reconstruct the αβ vector consistently."""
    v_dc = 600.0
    v_alpha, v_beta = 120.0, -80.0
    duties, v_alpha_act, v_beta_act = space_vector_pwm(v_alpha, v_beta, v_dc)
    v_abc = inverter_voltages(*duties, v_dc)
    a2, b2 = clarke(*v_abc)
    assert a2 == pytest.approx(v_alpha_act, abs=1e-6)
    assert b2 == pytest.approx(v_beta_act, abs=1e-6)
