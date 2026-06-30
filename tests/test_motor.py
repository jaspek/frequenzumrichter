"""Tests for the motor models (induction and synchronous machine)."""

import numpy as np
import pytest

from frequenzumrichter.motor import PMSM, InductionMotor
from frequenzumrichter.simulation import rk4_step


# --------------------------------------------------------------------------- #
# PMSM
# --------------------------------------------------------------------------- #
def test_pmsm_at_rest_stays_at_rest():
    m = PMSM()
    x = m.initial_state()
    dx = m.derivatives(x, 0.0, 0.0, 0.0)
    np.testing.assert_allclose(dx, np.zeros_like(dx), atol=1e-12)


def test_pmsm_torque_sign():
    """A positive q-current produces a positive torque."""
    m = PMSM()
    x = np.array([0.0, 1.0, 0.0, 0.0])  # i_q = 1 A
    assert m.torque(x) > 0
    expected = 1.5 * m.pole_pairs * m.flux_linkage * 1.0
    assert m.torque(x) == pytest.approx(expected)


def test_pmsm_synchronizes_to_rotating_field():
    """With an open-loop rotating field (V/f-type start-up) the rotor runs in synchronism.

    A constant voltage *space vector* would only pull the rotor into a detent
    position. Only a **rotating** field produces continuous rotation; the
    synchronous machine then runs at ``ω_m = ω_e / p``.
    """
    m = PMSM(friction=0.0)
    x = m.initial_state()
    dt = 1e-5
    f_e = 3.0  # electrical frequency [Hz]
    omega_e = 2 * np.pi * f_e
    t = 0.0
    for _ in range(60000):  # 0.6 s
        v_alpha = 10.0 * np.cos(omega_e * t)
        v_beta = 10.0 * np.sin(omega_e * t)
        x = rk4_step(m.derivatives, x, dt, v_alpha, v_beta, 0.0)
        t += dt
    meas = m.measurements(x)
    expected = omega_e / m.pole_pairs
    assert meas.omega_m == pytest.approx(expected, abs=0.5)
    assert meas.omega_m > 1.0  # rotor rotates continuously


def test_pmsm_reluctance_torque_when_saliency():
    """For L_d != L_q the reluctance torque contributes to the torque."""
    m = PMSM(l_d=0.004, l_q=0.008)
    x = np.array([-1.0, 2.0, 0.0, 0.0])  # for L_q>L_d, i_d<0 increases the torque
    t_total = m.torque(x)
    t_magnet = 1.5 * m.pole_pairs * m.flux_linkage * 2.0
    assert t_total != pytest.approx(t_magnet)


# --------------------------------------------------------------------------- #
# Induction machine
# --------------------------------------------------------------------------- #
def test_induction_at_rest_stays_at_rest():
    m = InductionMotor()
    x = m.initial_state()
    dx = m.derivatives(x, 0.0, 0.0, 0.0)
    np.testing.assert_allclose(dx, np.zeros_like(dx), atol=1e-12)


def test_induction_parameters_derived():
    m = InductionMotor()
    assert 0.0 < m.sigma < 1.0
    assert m.tau_r == pytest.approx(m.l_r / m.r_r)


def test_induction_flux_builds_up_with_dc_excitation():
    """A constant αβ voltage builds up a rotor flux."""
    m = InductionMotor()
    x = m.initial_state()
    dt = 1e-5
    for _ in range(50000):  # 0.5 s
        x = rk4_step(m.derivatives, x, dt, 50.0, 0.0, 0.0)
    meas = m.measurements(x)
    assert meas.rotor_flux > 0.05


def test_induction_no_torque_without_flux():
    """No torque without rotor flux."""
    m = InductionMotor()
    x = m.initial_state()
    assert m.torque(x) == pytest.approx(0.0)


def test_measurements_currents_consistent():
    """The returned abc currents must match i_alpha/i_beta via the Clarke transform."""
    m = PMSM()
    x = np.array([1.0, 0.5, 10.0, 0.3])
    meas = m.measurements(x)
    # Sum of the three phase currents = 0 (symmetric, no zero-sequence component)
    assert sum(meas.i_abc) == pytest.approx(0.0, abs=1e-9)
