# Frequenzumrichter

[![CI](https://github.com/jaspek/frequenzumrichter/actions/workflows/ci.yml/badge.svg)](https://github.com/jaspek/frequenzumrichter/actions/workflows/ci.yml)
![Python](https://img.shields.io/badge/python-3.9%2B-blue)
![License](https://img.shields.io/badge/license-MIT-green)

A complete, well-documented **simulation of a variable-frequency drive** (VFD) in
pure Python/NumPy. The package models the entire drivetrain – from the supplying
grid through the power stage to the controlled rotating-field machine – and is
suitable for teaching, controller design and pre-simulation studies.

> **What is a variable-frequency drive?** A variable-frequency drive generates,
> from a grid with fixed frequency, a three-phase voltage system of **variable
> frequency and amplitude**, and thereby controls the speed and torque of a
> three-phase motor continuously. It is the heart of nearly every modern electrical
> drive – from pumps and conveyor belts to electric vehicles.

---

## Contents / Features

| Component | Module | Description |
|----------|-------|--------------|
| **Rectifier** | `power_stage.Rectifier` | Uncontrolled B6 diode bridge (`V_dc ≈ 1.35·V_LL`) |
| **DC link** | `power_stage.DCLink` | Capacitor model, stiff or dynamic (regenerative overvoltage) |
| **Inverter** | `power_stage.Inverter` | 2-level voltage-source inverter (averaged model) with SVPWM |
| **PWM** | `pwm` | Sine PWM and space-vector modulation (SVPWM via min/max injection) |
| **Induction machine** | `motor.InductionMotor` | Squirrel-cage rotor in the stationary αβ model |
| **Synchronous machine** | `motor.PMSM` | Permanent-magnet PMSM in the rotor (dq) model |
| **V/f control** | `vf_control.VFControl` | Scalar characteristic-curve control with voltage boost |
| **Field-oriented control** | `foc.FOCPMSM`, `foc.FOCInduction` | Vector control (PMSM & indirect IRFOC of the induction machine) |
| **Controllers** | `controllers` | PI controllers with anti-windup, ramp, PT1 filter |
| **Protection** | `protection.Protection` | Over-/undervoltage, overcurrent, overspeed, I²t thermal model |
| **Transformations** | `transforms` | Clarke / Park (amplitude-invariant) |
| **Simulation** | `simulation`, `drive.Frequenzumrichter` | RK4 integrator, closed control loop, data logging |

---

## Installation

```bash
git clone https://github.com/jaspek/frequenzumrichter.git
cd frequenzumrichter
pip install -e .            # core package (NumPy only)
pip install -e ".[dev]"     # additionally pytest + matplotlib (tests & plots)
```

Requires Python ≥ 3.9 and NumPy. Matplotlib is used for the example plots.

---

## Quick start

```python
from frequenzumrichter import build_foc_pmsm_drive

# Pre-configured servo drive (PMSM + field-oriented control)
fu = build_foc_pmsm_drive()

# Speed step to 100 rad/s with 0.5 Nm load, simulate for 0.5 s
result = fu.run(t_end=0.5, speed_ref=100.0, load_torque=0.5)

print("Final speed  :", round(float(result.speed[-1]), 2), "rad/s")
print("Settling time:", round(result.settling_time(), 4), "s")
```

```
Final speed  : 100.0 rad/s
Settling time: 0.0278 s
```

### Time- and speed-dependent references

Both the speed setpoint and the load torque may be constants **or** functions:

```python
# Speed reversal via a setpoint step
speed_ref = lambda t: 80.0 if t < 0.25 else -80.0

# Passive fan load ∝ ω² (vanishes at standstill): f(t, meas)
fan = lambda t, meas: 1e-4 * meas.omega_m * abs(meas.omega_m)

result = fu.run(t_end=0.5, speed_ref=speed_ref, load_torque=fan)
```

---

## Examples

The [`examples/`](examples/) folder contains runnable scripts, each of which writes
a plot to `examples/output/`:

```bash
python examples/run_all.py          # all examples at once
python examples/01_vf_anlauf.py     # individually
```

| Script | Contents |
|--------|--------|
| `01_vf_anlauf.py` | Soft start of an induction machine with V/f control |
| `02_foc_pmsm_drehzahlsprung.py` | Highly dynamic PMSM control, steps & reversal |
| `03_svpwm_demo.py` | SVPWM duty cycles and extended modulation range |
| `04_lastsprung_und_schutz.py` | Disturbance rejection + thermal trip |
| `05_transformationen.py` | Clarke / Park transformation illustrated |

---

## Architecture

```
            Grid (3~)
               │
        ┌──────▼──────┐
        │  Rectifier  │  AC ➜ DC
        └──────┬──────┘
        ┌──────▼──────┐
        │   DC link   │  smoothing / energy buffer
        └──────┬──────┘
        ┌──────▼──────┐
        │  Inverter   │  SVPWM,  DC ➜ AC (variable)
        └──────┬──────┘
               │  v_alpha, v_beta
        ┌──────▼──────┐        ┌─────────────┐
        │    Motor    │◄───────│   Control   │  U/f  or  FOC
        │  IM / PMSM  │ measure│ (PI cascade)│
        └─────────────┘───────►└─────────────┘
                  ▲
            ┌─────┴─────┐
            │ Protection│  (trip)
            └───────────┘
```

The `Frequenzumrichter` class (`drive.py`) connects all components into a closed
control loop and integrates the motor model with a 4th-order Runge-Kutta method
(zero-order hold of the control voltage over the sampling period).

A detailed description can be found in [`docs/`](docs/):

* [`docs/theorie.md`](docs/theorie.md) – operating principle of the variable-frequency drive
* [`docs/regelung.md`](docs/regelung.md) – V/f control and field-oriented control
* [`docs/architektur.md`](docs/architektur.md) – code structure & extension

---

## Tests

```bash
pytest                      # 76 tests (unit + integration)
```

The test suite covers transformations, PWM, controllers, motor models, the power
stage, protection functions, as well as the closed control loop (setpoint tracking,
disturbance rejection, reversal, protection trip).

---

## Model assumptions & limitations

* **Averaged model** of the inverter – the switching events (PWM harmonics,
  dead time) are averaged over the sampling period. This is adequate for
  current-controller and speed considerations; for EMC / switching-loss analyses a
  switched model would be required.
* **Linear magnetics** – saturation and iron losses are not modeled.
* **Ideal measurement** – current/position sensors without noise or delay.

These simplifications are deliberately chosen in order to depict the operating
principle and control clearly and with numerical robustness. The modular structure
makes it possible to selectively refine individual building blocks (e.g. a switched
inverter model or a saturation characteristic).

---

## License

MIT – see [`LICENSE`](LICENSE).
