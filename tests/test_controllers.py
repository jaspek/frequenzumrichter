"""Tests für die Regler-Bausteine (PI, Rampe, Tiefpass)."""

import pytest

from frequenzumrichter.controllers import LowPassFilter, PIController, RateLimiter


def test_pi_drives_error_to_zero():
    """Geschlossener Regelkreis an einer Integrator-Strecke erreicht den Sollwert."""
    pi = PIController(kp=2.0, ki=10.0)
    plant = 0.0
    target = 5.0
    dt = 1e-3
    for _ in range(20000):
        u = pi.step(target - plant, dt)
        plant += u * dt  # Integrator-Strecke
    assert plant == pytest.approx(target, abs=1e-2)


def test_pi_output_clamped():
    pi = PIController(kp=100.0, ki=0.0, output_min=-1.0, output_max=1.0)
    assert pi.step(10.0, 1e-3) == 1.0
    assert pi.step(-10.0, 1e-3) == -1.0


def test_pi_anti_windup_limits_integral_and_speeds_recovery():
    """Anti-Windup hält den Integralspeicher klein und beschleunigt die Erholung."""
    aw = PIController(
        kp=1.0, ki=50.0, output_min=-1.0, output_max=1.0, anti_windup_gain=20.0
    )
    no_aw = PIController(kp=1.0, ki=50.0, output_min=-1.0, output_max=1.0)

    # 1 s dauerhaft in Sättigung treiben
    for _ in range(1000):
        aw.step(5.0, 1e-3)
        no_aw.step(5.0, 1e-3)
    assert abs(aw.integral) < abs(no_aw.integral)

    # Vorzeichen umkehren: zählen, wie viele Schritte bis zum Verlassen der Sättigung
    def steps_to_leave_saturation(pi: PIController) -> int:
        n = 0
        while pi.step(-5.0, 1e-3) >= 1.0 and n < 100000:
            n += 1
        return n

    assert steps_to_leave_saturation(aw) < steps_to_leave_saturation(no_aw)


def test_pi_without_anti_windup_winds_up():
    """Ohne Anti-Windup wächst der Integrator in der Sättigung deutlich stärker."""
    with_aw = PIController(
        kp=1.0, ki=50.0, output_min=-1.0, output_max=1.0, anti_windup_gain=50.0
    )
    without_aw = PIController(kp=1.0, ki=50.0, output_min=-1.0, output_max=1.0)
    for _ in range(500):
        with_aw.step(5.0, 1e-3)
        without_aw.step(5.0, 1e-3)
    assert without_aw.integral > with_aw.integral


def test_rate_limiter_ramps():
    rl = RateLimiter(rate_up=10.0)
    value = 0.0
    for _ in range(100):
        value = rl.step(100.0, 0.01)
    # nach 1 s mit 10/s darf der Wert höchstens 10 erreicht haben
    assert value == pytest.approx(10.0, abs=1e-6)


def test_rate_limiter_asymmetric():
    rl = RateLimiter(rate_up=10.0, rate_down=5.0, value=20.0)
    v = rl.step(0.0, 1.0)  # ein 1-s-Schritt nach unten
    assert v == pytest.approx(15.0)  # nur 5/s Abfall


def test_low_pass_filter_steady_state():
    lpf = LowPassFilter(tau=0.1)
    for _ in range(10000):
        out = lpf.step(3.0, 1e-3)
    assert out == pytest.approx(3.0, abs=1e-3)


def test_low_pass_filter_zero_tau_passthrough():
    lpf = LowPassFilter(tau=0.0)
    assert lpf.step(42.0, 1e-3) == 42.0
