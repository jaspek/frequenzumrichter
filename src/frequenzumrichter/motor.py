"""Dynamische Motormodelle für die Simulation.

Es werden zwei Drehfeldmaschinen modelliert, die typischerweise an einem
Frequenzumrichter betrieben werden:

* :class:`InductionMotor` – Asynchronmaschine (Käfigläufer), Modell im
  stationären αβ-Bezugssystem mit Statorstrom und Rotorfluss als Zuständen.
* :class:`PMSM` – permanenterregte Synchronmaschine, Modell im rotorfesten
  dq-Bezugssystem.

Beide Klassen bieten eine einheitliche Schnittstelle, sodass sie austauschbar
in der Simulation verwendet werden können:

* ``n_states`` – Anzahl der Zustandsgrößen
* ``initial_state()`` – Anfangszustandsvektor
* ``derivatives(x, v_alpha, v_beta, load_torque)`` – Zustandsableitungen
* ``measurements(x)`` – physikalisch messbare Größen (Ströme, Drehzahl, ...)

Der Eingang ist stets der **stationäre** Spannungsraumzeiger ``(v_alpha,
v_beta)``, wie ihn der Wechselrichter liefert. Maschinen, die intern im
dq-System rechnen (PMSM), transformieren ihn selbst.

Alle physikalischen Größen sind in SI-Einheiten angegeben.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .transforms import inverse_clarke, inverse_park, park

__all__ = ["MotorMeasurements", "InductionMotor", "PMSM"]


@dataclass
class MotorMeasurements:
    """Messbare Größen der Maschine zu einem Zeitpunkt."""

    i_alpha: float
    i_beta: float
    i_abc: tuple[float, float, float]
    omega_m: float  # mechanische Winkelgeschwindigkeit [rad/s]
    theta_m: float  # mechanischer Rotorwinkel [rad]
    torque: float  # elektromagnetisches Drehmoment [Nm]
    rotor_flux: float = 0.0  # Betrag des Rotorflusses [Wb] (nur ASM)


@dataclass
class InductionMotor:
    """Asynchronmaschine (Käfigläufer) im stationären αβ-Bezugssystem.

    Zustandsvektor ``x = [i_sα, i_sβ, ψ_rα, ψ_rβ, ω_m, θ_m]``.

    Das Modell basiert auf dem gekoppelten Stator-/Rotor-Gleichungssystem mit
    der Streuziffer ``σ = 1 - L_m² / (L_s L_r)`` und der Rotorzeitkonstante
    ``τ_r = L_r / R_r``.

    Parameters
    ----------
    r_s, r_r:
        Stator-/Rotorwiderstand [Ω].
    l_s, l_r, l_m:
        Stator-, Rotor- und Hauptinduktivität [H].
    pole_pairs:
        Polpaarzahl ``p``.
    inertia:
        Trägheitsmoment ``J`` [kg·m²].
    friction:
        Viskose Reibungskonstante ``B`` [Nm·s].
    """

    r_s: float = 2.0
    r_r: float = 2.0
    l_s: float = 0.2
    l_r: float = 0.2
    l_m: float = 0.19
    pole_pairs: int = 2
    inertia: float = 0.01
    friction: float = 0.001

    n_states: int = 6

    def __post_init__(self) -> None:
        self.sigma = 1.0 - self.l_m**2 / (self.l_s * self.l_r)
        self.tau_r = self.l_r / self.r_r

    def initial_state(self) -> np.ndarray:
        return np.zeros(self.n_states, dtype=float)

    def derivatives(
        self,
        x: np.ndarray,
        v_alpha: float,
        v_beta: float,
        load_torque: float = 0.0,
    ) -> np.ndarray:
        i_sa, i_sb, psi_ra, psi_rb, omega_m, _theta = x
        p = self.pole_pairs
        omega_e = p * omega_m  # elektrische Kreisfrequenz des Rotors

        # Hilfsgrößen
        k = self.l_m / (self.sigma * self.l_s * self.l_r)
        gamma = self.r_s / (self.sigma * self.l_s) + (
            self.r_r * self.l_m**2
        ) / (self.sigma * self.l_s * self.l_r**2)
        inv_sls = 1.0 / (self.sigma * self.l_s)

        # Statorstrom-Dynamik
        di_sa = (
            -gamma * i_sa
            + k / self.tau_r * psi_ra
            + k * omega_e * psi_rb
            + inv_sls * v_alpha
        )
        di_sb = (
            -gamma * i_sb
            - k * omega_e * psi_ra
            + k / self.tau_r * psi_rb
            + inv_sls * v_beta
        )

        # Rotorfluss-Dynamik
        dpsi_ra = self.l_m / self.tau_r * i_sa - psi_ra / self.tau_r - omega_e * psi_rb
        dpsi_rb = self.l_m / self.tau_r * i_sb + omega_e * psi_ra - psi_rb / self.tau_r

        # elektromagnetisches Drehmoment
        torque = (
            1.5 * p * (self.l_m / self.l_r) * (psi_ra * i_sb - psi_rb * i_sa)
        )

        # mechanische Bewegungsgleichung
        domega = (torque - load_torque - self.friction * omega_m) / self.inertia
        dtheta = omega_m

        return np.array([di_sa, di_sb, dpsi_ra, dpsi_rb, domega, dtheta])

    def torque(self, x: np.ndarray) -> float:
        i_sa, i_sb, psi_ra, psi_rb, _omega, _theta = x
        return float(
            1.5
            * self.pole_pairs
            * (self.l_m / self.l_r)
            * (psi_ra * i_sb - psi_rb * i_sa)
        )

    def measurements(self, x: np.ndarray) -> MotorMeasurements:
        i_sa, i_sb, psi_ra, psi_rb, omega_m, theta_m = x
        i_a, i_b, i_c = inverse_clarke(i_sa, i_sb)
        rotor_flux = float(np.hypot(psi_ra, psi_rb))
        return MotorMeasurements(
            i_alpha=float(i_sa),
            i_beta=float(i_sb),
            i_abc=(float(i_a), float(i_b), float(i_c)),
            omega_m=float(omega_m),
            theta_m=float(theta_m),
            torque=self.torque(x),
            rotor_flux=rotor_flux,
        )


@dataclass
class PMSM:
    """Permanenterregte Synchronmaschine im rotorfesten dq-Bezugssystem.

    Zustandsvektor ``x = [i_d, i_q, ω_m, θ_m]``.

    Für eine Oberflächen-PMSM gilt ``L_d == L_q``; bei ``L_d != L_q`` wird auch
    das Reluktanzmoment berücksichtigt.

    Parameters
    ----------
    r_s:
        Statorwiderstand [Ω].
    l_d, l_q:
        Längs-/Querinduktivität [H].
    flux_linkage:
        Permanentmagnet-Flussverkettung ``ψ_f`` [Wb].
    pole_pairs:
        Polpaarzahl ``p``.
    inertia:
        Trägheitsmoment ``J`` [kg·m²].
    friction:
        Viskose Reibungskonstante ``B`` [Nm·s].
    """

    r_s: float = 0.5
    l_d: float = 0.005
    l_q: float = 0.005
    flux_linkage: float = 0.1
    pole_pairs: int = 4
    inertia: float = 0.001
    friction: float = 0.0005

    n_states: int = 4

    def initial_state(self) -> np.ndarray:
        return np.zeros(self.n_states, dtype=float)

    def derivatives(
        self,
        x: np.ndarray,
        v_alpha: float,
        v_beta: float,
        load_torque: float = 0.0,
    ) -> np.ndarray:
        i_d, i_q, omega_m, theta_m = x
        p = self.pole_pairs
        theta_e = p * theta_m
        omega_e = p * omega_m

        # Spannungsraumzeiger ins rotorfeste dq-System drehen
        v_d, v_q = park(v_alpha, v_beta, theta_e)

        # Strom-Dynamik
        di_d = (v_d - self.r_s * i_d + omega_e * self.l_q * i_q) / self.l_d
        di_q = (
            v_q
            - self.r_s * i_q
            - omega_e * (self.l_d * i_d + self.flux_linkage)
        ) / self.l_q

        # Drehmoment (inkl. Reluktanzanteil)
        torque = 1.5 * p * (
            self.flux_linkage * i_q + (self.l_d - self.l_q) * i_d * i_q
        )

        domega = (torque - load_torque - self.friction * omega_m) / self.inertia
        dtheta = omega_m
        return np.array([di_d, di_q, domega, dtheta])

    def torque(self, x: np.ndarray) -> float:
        i_d, i_q, _omega, _theta = x
        return float(
            1.5
            * self.pole_pairs
            * (self.flux_linkage * i_q + (self.l_d - self.l_q) * i_d * i_q)
        )

    def measurements(self, x: np.ndarray) -> MotorMeasurements:
        i_d, i_q, omega_m, theta_m = x
        theta_e = self.pole_pairs * theta_m
        i_alpha, i_beta = inverse_park(i_d, i_q, theta_e)
        i_a, i_b, i_c = inverse_clarke(i_alpha, i_beta)
        return MotorMeasurements(
            i_alpha=float(i_alpha),
            i_beta=float(i_beta),
            i_abc=(float(i_a), float(i_b), float(i_c)),
            omega_m=float(omega_m),
            theta_m=float(theta_m),
            torque=self.torque(x),
            rotor_flux=float(self.flux_linkage),
        )
