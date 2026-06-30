# Changelog

All notable changes to this project are documented here.
The format is based on [Keep a Changelog](https://keepachangelog.com/de/1.0.0/).

## [0.1.0] – 2026-06-30

### Added

* **Power stage**
  * `Rectifier` – uncontrolled B6 diode bridge (`V_dc ≈ 1.35·V_LL`)
  * `DCLink` – DC-link capacitor, stiff or dynamic (regeneration overvoltage)
  * `Inverter` – 2-level voltage-source inverter (averaged model) with SVPWM
* **Modulation** (`pwm`)
  * Sinusoidal PWM and space-vector modulation (SVPWM via min/max zero-sequence injection)
  * Averaged model of the phase voltages, sector determination
* **Machines** (`motor`)
  * `InductionMotor` – induction machine in the stationary αβ model
  * `PMSM` – permanent-magnet synchronous machine in the rotor (dq) model
* **Control**
  * `VFControl` – scalar V/f characteristic control with voltage boost and ramp
  * `FOCPMSM`, `FOCInduction` – field-oriented control (PMSM & indirect IRFOC)
  * `PIController` with anti-windup (back-calculation), `RateLimiter`, `LowPassFilter`
* **Transformations** (`transforms`) – Clarke/Park (amplitude-invariant)
* **Protection** (`protection`) – over-/undervoltage, overcurrent, overspeed, I²t thermal model
* **Simulation**
  * `rk4_step` (4th-order Runge-Kutta), `SimulationResult` with settling-time estimation
  * `Frequenzumrichter` – complete model of the closed control loop
  * Load specification as a constant, `f(t)`, or state-dependent `f(t, meas)` (passive loads)
* **Factory** – `build_vf_drive`, `build_foc_pmsm_drive`, `build_foc_induction_drive`
* **Examples** – 5 runnable scripts with plots (ramp-up, reversal, SVPWM, protection, transformations)
* **Tests** – 76 unit and integration tests
* **Documentation** – theory, control, architecture (`docs/`)
