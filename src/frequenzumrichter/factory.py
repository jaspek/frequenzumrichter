"""Vorkonfigurierte Antriebe mit sinnvoll abgestimmten Reglern.

Diese Bauhelfer (*factory functions*) liefern einsatzbereite
:class:`~frequenzumrichter.drive.Frequenzumrichter`-Objekte, sodass man ohne
manuelle Reglerauslegung sofort simulieren kann. Die PI-Verstärkungen sind über
die *Betragsoptimum*-Faustformeln an die jeweilige Maschine angepasst.
"""

from __future__ import annotations

from .controllers import PIController
from .drive import Frequenzumrichter
from .foc import FOCInduction, FOCPMSM
from .motor import PMSM, InductionMotor
from .power_stage import DCLink, Inverter, Rectifier
from .protection import Protection, ProtectionLimits
from .vf_control import VFControl

__all__ = [
    "build_vf_drive",
    "build_foc_pmsm_drive",
    "build_foc_induction_drive",
]


def build_vf_drive(
    motor: InductionMotor | None = None,
    v_dc: float = 560.0,
    with_protection: bool = True,
) -> Frequenzumrichter:
    """Asynchronmaschine mit skalarer U/f-Steuerung.

    Robustes, gesteuertes Verfahren ohne Sensorrückführung – ideal für Lüfter-
    und Pumpenantriebe.
    """
    motor = motor or InductionMotor()
    controller = VFControl(
        f_rated=50.0,
        v_rated=300.0,
        pole_pairs=motor.pole_pairs,
        v_boost=15.0,
        freq_ramp=25.0,
    )
    protection = (
        Protection(
            limits=ProtectionLimits(
                max_current=40.0,
                max_dc_voltage=1.4 * v_dc,
                min_dc_voltage=0.6 * v_dc,
                max_speed=250.0,
            )
        )
        if with_protection
        else None
    )
    return Frequenzumrichter(
        motor=motor,
        controller=controller,
        inverter=Inverter(),
        dc_link=DCLink(voltage=v_dc, nominal_voltage=v_dc, stiff=True),
        rectifier=Rectifier(v_ll_rms=400.0),
        protection=protection,
        control_period=1e-4,
    )


def build_foc_pmsm_drive(
    motor: PMSM | None = None,
    v_dc: float = 300.0,
    max_current: float = 25.0,
    with_protection: bool = True,
) -> Frequenzumrichter:
    """Permanenterregte Synchronmaschine mit feldorientierter Regelung.

    Hochdynamischer, präziser Servoantrieb. Die Stromregler sind nach dem
    Betragsoptimum, der überlagerte Drehzahlregler weicher ausgelegt.
    """
    motor = motor or PMSM()

    # Stromregler (Betragsoptimum): kp = L*ω_c, ki = R*ω_c, ω_c ≈ 2π·200 Hz
    omega_ci = 2.0 * 3.141592653589793 * 200.0
    id_pi = PIController(
        kp=motor.l_d * omega_ci,
        ki=motor.r_s * omega_ci,
        output_min=-v_dc,
        output_max=v_dc,
        anti_windup_gain=50.0,
    )
    iq_pi = PIController(
        kp=motor.l_q * omega_ci,
        ki=motor.r_s * omega_ci,
        output_min=-v_dc,
        output_max=v_dc,
        anti_windup_gain=50.0,
    )
    # Drehzahlregler -> Soll-q-Strom (auf Maximalstrom begrenzt)
    speed_pi = PIController(
        kp=0.4,
        ki=6.0,
        output_min=-max_current,
        output_max=max_current,
        anti_windup_gain=20.0,
    )
    controller = FOCPMSM(
        motor=motor, speed_pi=speed_pi, id_pi=id_pi, iq_pi=iq_pi, decoupling=True
    )
    protection = (
        Protection(
            limits=ProtectionLimits(
                max_current=max_current * 1.8,
                max_dc_voltage=1.4 * v_dc,
                min_dc_voltage=0.6 * v_dc,
                max_speed=600.0,
                thermal_current=max_current * 0.6,
            )
        )
        if with_protection
        else None
    )
    return Frequenzumrichter(
        motor=motor,
        controller=controller,
        inverter=Inverter(),
        dc_link=DCLink(voltage=v_dc, nominal_voltage=v_dc, stiff=True),
        protection=protection,
        control_period=1e-4,
    )


def build_foc_induction_drive(
    motor: InductionMotor | None = None,
    v_dc: float = 560.0,
    flux_ref: float = 0.5,
    max_current: float = 30.0,
    with_protection: bool = True,
) -> Frequenzumrichter:
    """Asynchronmaschine mit indirekter feldorientierter Regelung (IRFOC)."""
    motor = motor or InductionMotor()

    omega_ci = 2.0 * 3.141592653589793 * 150.0
    sigma_ls = motor.sigma * motor.l_s
    id_pi = PIController(
        kp=sigma_ls * omega_ci,
        ki=motor.r_s * omega_ci,
        output_min=-v_dc,
        output_max=v_dc,
        anti_windup_gain=30.0,
    )
    iq_pi = PIController(
        kp=sigma_ls * omega_ci,
        ki=motor.r_s * omega_ci,
        output_min=-v_dc,
        output_max=v_dc,
        anti_windup_gain=30.0,
    )
    speed_pi = PIController(
        kp=1.5,
        ki=15.0,
        output_min=-max_current,
        output_max=max_current,
        anti_windup_gain=10.0,
    )
    controller = FOCInduction(
        motor=motor,
        speed_pi=speed_pi,
        id_pi=id_pi,
        iq_pi=iq_pi,
        flux_ref=flux_ref,
    )
    protection = (
        Protection(
            limits=ProtectionLimits(
                max_current=max_current * 1.8,
                max_dc_voltage=1.4 * v_dc,
                min_dc_voltage=0.6 * v_dc,
                max_speed=250.0,
            )
        )
        if with_protection
        else None
    )
    return Frequenzumrichter(
        motor=motor,
        controller=controller,
        inverter=Inverter(),
        dc_link=DCLink(voltage=v_dc, nominal_voltage=v_dc, stiff=True),
        rectifier=Rectifier(v_ll_rms=400.0),
        protection=protection,
        control_period=1e-4,
    )
