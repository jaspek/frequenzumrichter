# Operating Principle of the Frequency Converter

A frequency converter (variable-frequency drive, VFD) feeds a three-phase motor
with a voltage system of **variable frequency and amplitude**, thereby controlling
the motor's speed and torque. It thus replaces the formerly common lossy methods
(series resistors, pole-changing motors, mechanical gearboxes) with an electronic,
nearly lossless manipulated variable.

## Why vary the frequency at all?

The synchronous speed of a rotating-field machine is

$$ n_s = \frac{60 \cdot f}{p} \quad [\text{min}^{-1}] \qquad\text{or}\qquad \omega_s = \frac{2\pi f}{p}\ [\text{rad/s}] $$

with the stator frequency `f` and the number of pole pairs `p`. The speed is thus
directly coupled to the frequency. Anyone who wants to vary the speed continuously
must vary the frequency — and that is precisely what the frequency converter does.

## Structure: the voltage-source DC-link converter

By far the most common topology is the **voltage-source converter** (voltage DC
link) with three stages:

```
  Grid 3~ ──►   Rectifier   ──►     DC link     ──►     Inverter    ──► Motor
  (f fixed)    (AC ➜ DC)         (DC, smoothed)      (DC ➜ AC, f variable)
```

### 1. Rectifier

An uncontrolled **6-pulse diode bridge (B6)** rectifies the three-phase grid. On
average, this produces the DC-link voltage

$$ V_{dc} \approx \frac{3\sqrt{2}}{\pi}\,V_{LL,\text{eff}} \approx 1.35 \cdot V_{LL,\text{eff}} $$

At a 400 V line-to-line grid voltage this amounts to about 540 V; at no-load the
capacitor charges up to the peak value $\sqrt 2 \cdot 400\,\text{V} \approx 566\,\text{V}$.
→ `power_stage.Rectifier`

### 2. DC link

A **capacitor** smooths the DC voltage and buffers energy. Its voltage dynamics
follow the energy balance

$$ C\,\frac{dV_{dc}}{dt} = i_{\text{source}} - i_{\text{load}} $$

The feeding diode bridge can only **supply** current, not absorb it. During
regenerative braking, energy flows back into the DC link and the voltage rises —
without a braking resistor (*brake chopper*) up to the overvoltage trip.
→ `power_stage.DCLink`

### 3. Inverter

Three half-bridges of turn-off power semiconductors (IGBT/MOSFET) regenerate a
three-phase voltage system from the DC voltage. The desired fundamental of
frequency and amplitude is set via **pulse-width modulation**: each bridge leg
switches at a few kHz between the upper and lower DC-link rail; the ratio of the
on-times (duty cycle) determines the average output voltage.
→ `power_stage.Inverter`, `pwm`

## Pulse-width modulation (PWM)

Two methods are implemented:

* **Sinusoidal PWM (SPWM):** three sinusoidal references are compared with a
  triangular carrier. Maximum linear phase-voltage amplitude: $V_{dc}/2$.
* **Space-vector modulation (SVPWM):** treats the three bridge voltages as a single
  rotating voltage *space vector* and switches between the eight possible switching
  states. By injecting a zero-sequence component (`v_off = -(max+min)/2`,
  equivalent to 7-segment SVPWM), the linear range is extended to $V_{dc}/\sqrt 3$ —
  roughly **15 % more voltage** than with pure sinusoidal PWM.

→ `pwm.sinusoidal_pwm`, `pwm.svpwm_duty`

## Power flow and efficiency

The efficiency of a frequency converter is typically 95–98 %. Losses arise mainly
as conduction and switching losses in the semiconductors. In the averaged model
used here, these are neglected; what is modeled is the *ideal* power flow via the
balance

$$ P_{ac} = \frac{3}{2}\left(v_\alpha i_\alpha + v_\beta i_\beta\right) = V_{dc}\, i_{dc} $$

from which the DC-link current is estimated (`Inverter.modulate`). In regenerative
operation $i_{dc} < 0$ — the machine feeds energy back.

## Further reading

* Control methods (V/f, FOC): [`regelung.md`](regelung.md)
* Code structure and extension: [`architektur.md`](architektur.md)
