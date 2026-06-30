"""Dynamic motor models for the simulation.

Two rotating-field machines are modeled, typically operated on a
variable-frequency drive:

* :class:`InductionMotor` – induction machine (squirrel-cage), modeled in the
  stationary αβ reference frame with stator current and rotor flux as states.
* :class:`PMSM` – permanent-magnet synchronous machine, modeled in the rotor
  (dq) reference frame.

Both classes provide a uniform interface so that they can be used
interchangeably in the simulation:

* ``n_states`` – number of state variables
* ``initial_state()`` – initial state vector
* ``derivatives(x, v_alpha, v_beta, load_torque)`` – state derivatives
* ``measurements(x)`` – physically measurable quantities (currents, speed, ...)

The input is always the **stationary** voltage space vector ``(v_alpha,
v_beta)`` as delivered by the inverter. Machines that compute internally in the
dq frame (PMSM) transform it themselves.

All physical quantities are given in SI units.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .transforms import inverse_clarke, inverse_park, park

__all__ = ["MotorMeasurements", "InductionMotor", "PMSM"]


@dataclass
class MotorMeasurements:
    """Measurable quantities of the machine at a point in time."""

    i_alpha: float
    i_beta: float
    i_abc: tuple[float, float, float]
    omega_m: float  # mechanical angular velocity [rad/s]
    theta_m: float  # mechanical rotor angle [rad]
    torque: float  # electromagnetic torque [Nm]
    rotor_flux: float = 0.0  # magnitude of the rotor flux [Wb] (induction machine only)


@dataclass
class InductionMotor:
    """Induction machine (squirrel-cage) in the stationary αβ reference frame.

    State vector ``x = [i_sα, i_sβ, ψ_rα, ψ_rβ, ω_m, θ_m]``.

    The model is based on the coupled stator/rotor system of equations with
    the leakage coefficient ``σ = 1 - L_m² / (L_s L_r)`` and the rotor time
    constant ``τ_r = L_r / R_r``.

    Parameters
    ----------
    r_s, r_r:
        Stator/rotor resistance [Ω].
    l_s, l_r, l_m:
        Stator, rotor and mutual inductance [H].
    pole_pairs:
        Number of pole pairs ``p``.
    inertia:
        Moment of inertia ``J`` [kg·m²].
    friction:
        Viscous friction constant ``B`` [Nm·s].
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
        omega_e = p * omega_m  # electrical angular frequency of the rotor

        # auxiliary quantities
        k = self.l_m / (self.sigma * self.l_s * self.l_r)
        gamma = self.r_s / (self.sigma * self.l_s) + (
            self.r_r * self.l_m**2
        ) / (self.sigma * self.l_s * self.l_r**2)
        inv_sls = 1.0 / (self.sigma * self.l_s)

        # stator current dynamics
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

        # rotor flux dynamics
        dpsi_ra = self.l_m / self.tau_r * i_sa - psi_ra / self.tau_r - omega_e * psi_rb
        dpsi_rb = self.l_m / self.tau_r * i_sb + omega_e * psi_ra - psi_rb / self.tau_r

        # electromagnetic torque
        torque = (
            1.5 * p * (self.l_m / self.l_r) * (psi_ra * i_sb - psi_rb * i_sa)
        )

        # mechanical equation of motion
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
    """Permanent-magnet synchronous machine in the rotor (dq) reference frame.

    State vector ``x = [i_d, i_q, ω_m, θ_m]``.

    For a surface-mounted PMSM ``L_d == L_q`` holds; when ``L_d != L_q`` the
    reluctance torque is also taken into account.

    Parameters
    ----------
    r_s:
        Stator resistance [Ω].
    l_d, l_q:
        Direct-/quadrature-axis inductance [H].
    flux_linkage:
        Permanent-magnet flux linkage ``ψ_f`` [Wb].
    pole_pairs:
        Number of pole pairs ``p``.
    inertia:
        Moment of inertia ``J`` [kg·m²].
    friction:
        Viscous friction constant ``B`` [Nm·s].
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

        # rotate the voltage space vector into the rotor (dq) frame
        v_d, v_q = park(v_alpha, v_beta, theta_e)

        # current dynamics
        di_d = (v_d - self.r_s * i_d + omega_e * self.l_q * i_q) / self.l_d
        di_q = (
            v_q
            - self.r_s * i_q
            - omega_e * (self.l_d * i_d + self.flux_linkage)
        ) / self.l_q

        # torque (incl. reluctance component)
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
