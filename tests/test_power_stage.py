"""Tests for the power stage (rectifier, DC link, inverter)."""

import numpy as np
import pytest

from frequenzumrichter.power_stage import DCLink, Inverter, Rectifier


def test_rectifier_average_voltage():
    """B6 bridge: V_dc ≈ 1.35 · V_LL."""
    r = Rectifier(v_ll_rms=400.0)
    assert r.dc_voltage == pytest.approx(1.35 * 400.0, rel=0.01)
    assert r.peak_dc_voltage == pytest.approx(np.sqrt(2) * 400.0)


def test_dclink_stiff_is_constant():
    dc = DCLink(voltage=560.0, nominal_voltage=560.0, stiff=True)
    for _ in range(100):
        dc.update(i_load=20.0, dt=1e-4)
    assert dc.voltage == pytest.approx(560.0)


def test_dclink_regen_raises_voltage():
    """Regenerative braking (i_load < 0) charges the capacitor."""
    dc = DCLink(
        capacitance=1e-3,
        voltage=560.0,
        nominal_voltage=560.0,
        source_resistance=0.5,
        stiff=False,
    )
    for _ in range(1000):
        dc.update(i_load=-10.0, dt=1e-4)  # regeneration
    assert dc.voltage > 560.0


def test_dclink_motoring_draws_from_source():
    """During motoring the voltage drops slightly and the source replenishes it."""
    dc = DCLink(
        capacitance=1e-3,
        voltage=560.0,
        nominal_voltage=560.0,
        source_resistance=0.5,
        stiff=False,
    )
    dc.update(i_load=5.0, dt=1e-4)
    assert dc.voltage < 560.0
    assert dc.source_current() > 0.0


def test_inverter_duties_in_range_and_voltage_tracks():
    inv = Inverter()
    out = inv.modulate(120.0, -50.0, 540.0, i_alpha=2.0, i_beta=1.0)
    for d in out.duties:
        assert 0.0 <= d <= 1.0
    # in the linear range the actual voltage follows the setpoint
    assert out.v_alpha == pytest.approx(120.0, abs=1e-6)
    assert out.v_beta == pytest.approx(-50.0, abs=1e-6)


def test_inverter_power_balance_sign():
    """Positive active power => positive DC-link current."""
    inv = Inverter()
    # current in phase with voltage -> power consumption
    out = inv.modulate(100.0, 0.0, 540.0, i_alpha=5.0, i_beta=0.0)
    assert out.i_dc > 0.0
    # current in anti-phase -> regeneration
    out_regen = inv.modulate(100.0, 0.0, 540.0, i_alpha=-5.0, i_beta=0.0)
    assert out_regen.i_dc < 0.0


def test_inverter_saturates_above_linear_range():
    """Above the voltage limit circle the output voltage is saturated."""
    inv = Inverter()
    v_dc = 540.0
    out = inv.modulate(1000.0, 0.0, v_dc)  # far above V_dc/√3
    magnitude = np.hypot(out.v_alpha, out.v_beta)
    assert magnitude < 1000.0
