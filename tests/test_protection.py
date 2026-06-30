"""Tests für die Schutzfunktionen."""

import pytest

from frequenzumrichter.protection import FaultType, Protection, ProtectionLimits


def make_protection():
    return Protection(
        limits=ProtectionLimits(
            max_current=50.0,
            max_dc_voltage=800.0,
            min_dc_voltage=350.0,
            max_speed=400.0,
            thermal_current=10.0,
            thermal_time_constant=1.0,
            thermal_trip_level=1.0,
        )
    )


def test_no_trip_in_normal_operation():
    p = make_protection()
    tripped = p.check(current_magnitude=20.0, dc_voltage=560.0, speed=150.0, dt=1e-3)
    assert not tripped
    assert not p.faults


def test_overcurrent_trips():
    p = make_protection()
    assert p.check(current_magnitude=60.0, dc_voltage=560.0, speed=150.0, dt=1e-3)
    assert FaultType.OVERCURRENT in p.faults


def test_overvoltage_trips():
    p = make_protection()
    assert p.check(current_magnitude=10.0, dc_voltage=850.0, speed=0.0, dt=1e-3)
    assert FaultType.OVERVOLTAGE in p.faults


def test_undervoltage_trips():
    p = make_protection()
    assert p.check(current_magnitude=10.0, dc_voltage=300.0, speed=0.0, dt=1e-3)
    assert FaultType.UNDERVOLTAGE in p.faults


def test_overspeed_trips():
    p = make_protection()
    assert p.check(current_magnitude=10.0, dc_voltage=560.0, speed=500.0, dt=1e-3)
    assert FaultType.OVERSPEED in p.faults


def test_thermal_overload_accumulates_then_trips():
    """Dauerhafter Überstrom lädt das I²t-Modell, bis abgeschaltet wird."""
    p = make_protection()
    tripped = False
    # 3·Nennstrom => starke Erwärmung; tau=1 s
    for _ in range(5000):
        tripped = p.check(current_magnitude=30.0, dc_voltage=560.0, speed=0.0, dt=1e-3)
        if tripped:
            break
    assert tripped
    assert FaultType.THERMAL_OVERLOAD in p.faults
    assert p.thermal_state > 1.0


def test_thermal_does_not_trip_below_rated():
    p = make_protection()
    for _ in range(5000):
        assert not p.check(
            current_magnitude=5.0, dc_voltage=560.0, speed=0.0, dt=1e-3
        )
    assert p.thermal_state == pytest.approx(0.0)


def test_reset_clears_faults():
    p = make_protection()
    p.check(current_magnitude=60.0, dc_voltage=560.0, speed=0.0, dt=1e-3)
    assert p.tripped
    p.reset()
    assert not p.tripped
    assert not p.faults
    assert p.thermal_state == 0.0


def test_trip_state_latches():
    """Nach einem Trip bleibt der Fehler bis zum Reset bestehen."""
    p = make_protection()
    p.check(current_magnitude=60.0, dc_voltage=560.0, speed=0.0, dt=1e-3)
    # auch bei wieder gesundem Strom bleibt getrippt
    assert p.check(current_magnitude=1.0, dc_voltage=560.0, speed=0.0, dt=1e-3)
