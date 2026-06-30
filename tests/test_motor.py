"""Tests für die Motormodelle (Asynchron- und Synchronmaschine)."""

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
    """Positiver q-Strom erzeugt positives Drehmoment."""
    m = PMSM()
    x = np.array([0.0, 1.0, 0.0, 0.0])  # i_q = 1 A
    assert m.torque(x) > 0
    expected = 1.5 * m.pole_pairs * m.flux_linkage * 1.0
    assert m.torque(x) == pytest.approx(expected)


def test_pmsm_synchronizes_to_rotating_field():
    """Bei offenem Drehfeld (V/f-artiger Anlauf) läuft der Rotor synchron mit.

    Ein konstanter Spannungs-*Raumzeiger* würde den Rotor nur in eine Raststellung
    ziehen. Erst ein **rotierendes** Drehfeld erzeugt eine kontinuierliche
    Drehung; die Synchronmaschine läuft dann mit ``ω_m = ω_e / p``.
    """
    m = PMSM(friction=0.0)
    x = m.initial_state()
    dt = 1e-5
    f_e = 3.0  # elektrische Frequenz [Hz]
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
    assert meas.omega_m > 1.0  # Rotor dreht kontinuierlich


def test_pmsm_reluctance_torque_when_saliency():
    """Bei L_d != L_q trägt das Reluktanzmoment zum Drehmoment bei."""
    m = PMSM(l_d=0.004, l_q=0.008)
    x = np.array([-1.0, 2.0, 0.0, 0.0])  # i_d<0 erhöht bei L_q>L_d das Moment
    t_total = m.torque(x)
    t_magnet = 1.5 * m.pole_pairs * m.flux_linkage * 2.0
    assert t_total != pytest.approx(t_magnet)


# --------------------------------------------------------------------------- #
# Asynchronmaschine
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
    """Eine konstante αβ-Spannung baut einen Rotorfluss auf."""
    m = InductionMotor()
    x = m.initial_state()
    dt = 1e-5
    for _ in range(50000):  # 0.5 s
        x = rk4_step(m.derivatives, x, dt, 50.0, 0.0, 0.0)
    meas = m.measurements(x)
    assert meas.rotor_flux > 0.05


def test_induction_no_torque_without_flux():
    """Ohne Rotorfluss kein Drehmoment."""
    m = InductionMotor()
    x = m.initial_state()
    assert m.torque(x) == pytest.approx(0.0)


def test_measurements_currents_consistent():
    """Die zurückgegebenen abc-Ströme müssen über Clarke zu i_alpha/i_beta passen."""
    m = PMSM()
    x = np.array([1.0, 0.5, 10.0, 0.3])
    meas = m.measurements(x)
    # Summe der drei Strangströme = 0 (symmetrisch, kein Nullsystem)
    assert sum(meas.i_abc) == pytest.approx(0.0, abs=1e-9)
