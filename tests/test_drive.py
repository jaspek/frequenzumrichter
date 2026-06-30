"""Integrationstests des gesamten Antriebs (geschlossener Regelkreis)."""

import numpy as np
import pytest

from frequenzumrichter import (
    build_foc_induction_drive,
    build_foc_pmsm_drive,
    build_vf_drive,
)
from frequenzumrichter.protection import FaultType


def test_vf_reaches_setpoint_minus_slip():
    """U/f-Steuerung erreicht annähernd die Synchrondrehzahl (minus Schlupf)."""
    fu = build_vf_drive(with_protection=False)
    target = 150.0
    result = fu.run(t_end=2.5, speed_ref=target, load_torque=2.0)
    final = float(result.speed[-1])
    # Schlupf zieht die Drehzahl etwas unter den Sollwert, aber nahe daran
    assert 0.9 * target < final <= target + 1.0


def test_foc_pmsm_tracks_setpoint():
    fu = build_foc_pmsm_drive(with_protection=False)
    target = 100.0
    result = fu.run(t_end=0.5, speed_ref=target, load_torque=0.5)
    assert float(result.speed[-1]) == pytest.approx(target, abs=2.0)
    # schnelle Ausregelung erwartet (< 100 ms)
    settling = result.settling_time(tolerance=0.02)
    assert settling is not None and settling < 0.1


def test_foc_pmsm_speed_reversal():
    """Drehrichtungsumkehr: Sollwertsprung von +80 auf -80 rad/s."""
    fu = build_foc_pmsm_drive(with_protection=False)
    speed_ref = lambda t: 80.0 if t < 0.25 else -80.0  # noqa: E731
    result = fu.run(t_end=0.5, speed_ref=speed_ref, load_torque=0.0)
    assert float(result.speed[-1]) == pytest.approx(-80.0, abs=3.0)


def test_foc_induction_tracks_setpoint():
    fu = build_foc_induction_drive(with_protection=False)
    target = 120.0
    result = fu.run(t_end=2.0, speed_ref=target, load_torque=1.0)
    assert float(result.speed[-1]) == pytest.approx(target, abs=3.0)


def test_load_disturbance_rejection_pmsm():
    """Ein Lastsprung darf die Drehzahl nur kurz einbrechen lassen."""
    fu = build_foc_pmsm_drive(with_protection=False)
    target = 120.0
    load = lambda t: 0.0 if t < 0.3 else 3.0  # noqa: E731
    result = fu.run(t_end=0.6, speed_ref=target, load_torque=load)
    # am Ende wieder auf Sollwert ausgeregelt
    assert float(result.speed[-1]) == pytest.approx(target, abs=2.0)
    # das stationäre Drehmoment entspricht der Last
    assert float(result.torque[-1]) == pytest.approx(3.0, abs=0.8)


def test_overspeed_protection_trips_drive():
    """Ein zu hoher Sollwert führt über die Schutzfunktion zur Abschaltung."""
    fu = build_foc_pmsm_drive(with_protection=True)
    fu.protection.limits.max_speed = 50.0
    result = fu.run(t_end=0.5, speed_ref=200.0, load_torque=0.0)
    assert bool(result.tripped[-1])
    assert FaultType.OVERSPEED in fu.protection.faults


def test_reset_restores_initial_state():
    fu = build_foc_pmsm_drive(with_protection=False)
    fu.run(t_end=0.2, speed_ref=100.0)
    fu.reset()
    assert float(fu.measurements.omega_m) == pytest.approx(0.0)
    np.testing.assert_allclose(fu.state, np.zeros_like(fu.state))


def test_speed_dependent_passive_load():
    """Eine zustandsabhängige Last ``f(t, meas)`` wird korrekt ausgewertet."""
    fu = build_foc_pmsm_drive(with_protection=False)
    target = 100.0
    # passive Lüfterlast ∝ ω² (richtungstreu), verschwindet bei Stillstand
    def fan_load(t, meas):
        w = meas.omega_m
        return 1e-4 * w * abs(w)

    result = fu.run(t_end=0.6, speed_ref=target, load_torque=fan_load)
    assert float(result.speed[-1]) == pytest.approx(target, abs=2.0)
    # stationäres Moment entspricht der Lüfterkennlinie bei Solldrehzahl
    expected_load = 1e-4 * target * target
    assert float(result.torque[-1]) == pytest.approx(expected_load, abs=0.3)


def test_run_records_consistent_lengths():
    fu = build_vf_drive(with_protection=False)
    dt = 1e-4
    result = fu.run(t_end=0.1, speed_ref=100.0, dt=dt)
    n = int(round(0.1 / dt))
    assert len(result.time) == n
    assert len(result.speed) == n
    assert len(result.torque) == n
