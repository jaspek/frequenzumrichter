# Control Methods

The frequency converter must compute the correct inverter voltages from the
speed setpoint. Two fundamentally different methods are implemented for this: the
**scalar V/f control** and **field-oriented control (FOC)**.

---

## 1. Scalar V/f Control

The simplest method keeps the voltage-to-frequency ratio constant, so that the
magnetic flux (and thus the available torque) stays approximately the same across
the speed range:

$$ U(f) = U_{boost} + \frac{U_{rated}}{f_{rated}} \cdot f $$

* **Voltage boost `U_boost`** compensates for the ohmic voltage drop across the
  stator resistance at low frequencies and secures the starting torque.
* Above the rated frequency the voltage stays constant (limited by the
  DC link) → **field-weakening range** with decreasing torque.

V/f control operates in **open loop** (without feedback): no current or speed
sensor is required. Depending on the load, the actual speed lies below the
synchronous speed by the **slip**. The method is extremely robust and
inexpensive – ideal for fans, pumps, and simple conveyor drives.

→ Implementation: `vf_control.VFControl`

```python
from frequenzumrichter import build_vf_drive
fu = build_vf_drive()
result = fu.run(t_end=2.5, speed_ref=150.0, load_torque=3.0)
# -> Final speed ≈ 149 rad/s (synchronous speed minus slip)
```

**Limitations:** no defined torque, weak behavior at low speeds, slow dynamics.
For highly dynamic or position-accurate drives, field-oriented control is
required.

---

## 2. Field-Oriented Control (FOC / Vector Control)

FOC transfers the principle of the separately excited DC machine to the
rotating-field machine: in a **rotating dq coordinate system** that runs along
with the flux, flux and torque are controlled in a **decoupled** manner:

* the **d current** $i_d$ sets the magnetic flux,
* the **q current** $i_q$ sets the torque $\;M = \tfrac{3}{2}p\,\psi\,i_q$.

### Coordinate Transformations

```
abc ──Clarke──► αβ ──Park(θ)──► dq      (measurement)
dq  ──Park⁻¹(θ)──► αβ ──SVPWM──► bridge  (control output)
```

The decisive point is the **field angle θ**:

* **PMSM** – the flux is fixed in the rotor, so $\theta_e = p\,\theta_m$
  is known directly from the rotor position (encoder). Here $i_d^* = 0$ applies
  (surface-mounted PMSM); the entire current produces torque.
  → `foc.FOCPMSM`
* **Induction machine** – the rotor flux "slips" relative to the rotor. The
  field angle is computed **indirectly** via the slip relation (IRFOC):

  $$ \omega_{sl} = \frac{i_q^{*}}{\tau_r\, i_d^{*}}, \qquad
     \theta_{field} = \int \left(p\,\omega_m + \omega_{sl}\right) dt $$

  Here $i_d^*$ sets the rotor flux ($\psi_r = L_m i_d^*$), and $i_q^*$ sets the
  torque.
  → `foc.FOCInduction`

### Cascade Structure

```
 ω_ref ─►(PI speed)──► i_q* ─►(PI current q)─► v_q ┐
                                                  ├─► Park⁻¹ ─► SVPWM
 i_d* ──────────────►(PI current d)─► v_d ────────┘
```

* **Inner current controllers** (fast, ~kHz bandwidth) – designed according to
  the *magnitude optimum*: $K_p = L\,\omega_c,\; K_i = R\,\omega_c$.
* **Outer speed controller** (slower) – its control output is the
  q-current reference, limited to the maximum current (= torque limit).
* **Decoupling feedforward** compensates for the speed-proportional
  cross-coupling terms ($\omega_e L_q i_q$ and $\omega_e(L_d i_d + \psi_f)$,
  respectively) and the back-EMF – this makes the two current control loops
  approximately linear and independent.
* **Voltage limiting** to the modulation range circle $V_{dc}/\sqrt3$
  (circle limiting, `control_base.limit_vector`).

→ Controller block: `controllers.PIController` (with anti-windup via
back-calculation).

```python
from frequenzumrichter import build_foc_pmsm_drive
fu = build_foc_pmsm_drive()
result = fu.run(t_end=0.5, speed_ref=100.0, load_torque=0.5)
# -> Settling time < 30 ms, exact setpoint tracking, clean load rejection
```

---

## 3. Anti-Windup

If the control output of a PI controller runs into its limit (e.g. the current
limit during strong acceleration), the integral term would keep integrating up
uncontrolled (*windup*) and cause a large overshoot once the limit is left. The
implementation uses **back-calculation**:

$$ I_{k+1} = I_k + \Big(K_i\,e + K_{aw}\,(u_{sat} - u_{unsat})\Big)\,\Delta t $$

The term $K_{aw}(u_{sat}-u_{unsat})$ is only active during saturation and drives
the integrator back, so that the controller responds immediately again once the
limit is left.

---

## Comparison

| Property | V/f Control | Field-Oriented Control |
|-------------|:-------------:|:------------------------:|
| Feedback | none | current + position/speed |
| Dynamics | low | very high |
| Torque at n≈0 | weak | full torque |
| Accuracy | slip-affected | exact |
| Effort | low | high |
| Typical application | pump, fan | servo, traction, machine tool |
