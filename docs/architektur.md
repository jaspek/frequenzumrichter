# Code Structure & Extension

## Module Overview

```
src/frequenzumrichter/
├── __init__.py        public API (re-exports)
├── transforms.py      Clarke/Park transformations
├── pwm.py             sinusoidal PWM, SVPWM, inverter average model
├── controllers.py     PIController, RateLimiter, LowPassFilter
├── control_base.py    ControlOutput, voltage limiting (limit_vector)
├── motor.py           InductionMotor (induction machine), PMSM
├── power_stage.py     Rectifier, DCLink, Inverter
├── vf_control.py      VFControl (scalar V/f control)
├── foc.py             FOCPMSM, FOCInduction (vector control)
├── protection.py      Protection, ProtectionLimits, FaultType
├── simulation.py      rk4_step, SimulationResult
├── drive.py           Frequenzumrichter (overall model)
└── factory.py         preconfigured drives (build_*)
```

## Data Flow of a Simulation Step

`Frequenzumrichter.step(speed_ref, load_torque, dt)`:

1. **Measurement:** `motor.measurements(state)` → currents (αβ/abc), speed, position,
   torque, rotor flux (`MotorMeasurements`).
2. **Protection:** `protection.check(...)` checks current, voltage, speed, and
   thermal limits. On a trip → pulse inhibit (control voltage 0).
3. **Control:** `controller.compute(speed_ref, meas, v_dc, dt)` returns the
   reference voltage space vector (`ControlOutput`).
4. **Modulation:** `inverter.modulate(...)` converts it into duty cycles via SVPWM
   and returns the *actually* applied vector
   (including overmodulation limiting) as well as the DC-link current.
5. **Integration:** `rk4_step(motor.derivatives, ...)` integrates the motor model
   over `dt` (zero-order hold of the voltage).
6. **DC link:** `dc_link.update(i_dc, dt)` updates the DC voltage.

## Unified Interfaces

Thanks to two lean `Protocol` interfaces (`drive.Motor`,
`drive.Controller`), machines and control schemes can be combined freely.

### Custom Motor Model

```python
class MyMotor:
    n_states = 4
    def initial_state(self): ...
    def derivatives(self, x, v_alpha, v_beta, load_torque): ...
    def measurements(self, x) -> MotorMeasurements: ...
```

### Custom Controller

```python
class MyController:
    def reset(self): ...
    def compute(self, speed_ref, meas, v_dc, dt) -> ControlOutput: ...
```

Both can be plugged directly into `Frequenzumrichter(motor=..., controller=...)`.

## Conventions

* **Units:** consistently SI (rad/s, A, V, Wb, Nm, s).
* **Reference frames:** machine input/output is always in the **stationary αβ frame**
  (inverter perspective). Machines that compute internally in the dq frame (PMSM)
  perform the transformation themselves.
* **Transformations:** amplitude-invariant form (factor 2/3).
* **Speed:** `speed_ref` is the **mechanical** angular velocity
  $\omega_m$ [rad/s], not the electrical one.

## Numerics

* Integrator: classical **fourth-order Runge-Kutta** (`rk4_step`).
* Default sampling/step size: `control_period = 100 µs` (10 kHz). For very
  fast electrical time constants (small inductance), reduce `dt` if necessary.
* The control voltage is held constant over the step (ZOH) – consistent with
  a digitally sampled control loop.

## Tests

`tests/` contains unit tests for each module plus integration tests of the closed
control loop (`test_drive.py`). Run with `pytest`.

## Possible Extensions

* switching inverter model (PWM harmonics, dead time, switching losses)
* magnetic saturation / nonlinear inductances
* sensorless control (flux observers, back-EMF / MRAS methods)
* field weakening of the PMSM ($i_d^* < 0$ above the corner speed)
* brake chopper / regeneration unit in the DC link
* current-measurement noise and encoder resolution
