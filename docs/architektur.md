# Codeaufbau & Erweiterung

## Modulübersicht

```
src/frequenzumrichter/
├── __init__.py        öffentliche API (Re-Exports)
├── transforms.py      Clarke-/Park-Transformationen
├── pwm.py             Sinus-PWM, SVPWM, Wechselrichter-Mittelwertmodell
├── controllers.py     PIController, RateLimiter, LowPassFilter
├── control_base.py    ControlOutput, Spannungsbegrenzung (limit_vector)
├── motor.py           InductionMotor (ASM), PMSM
├── power_stage.py     Rectifier, DCLink, Inverter
├── vf_control.py      VFControl (skalare U/f-Steuerung)
├── foc.py             FOCPMSM, FOCInduction (Vektorregelung)
├── protection.py      Protection, ProtectionLimits, FaultType
├── simulation.py      rk4_step, SimulationResult
├── drive.py           Frequenzumrichter (Gesamtmodell)
└── factory.py         vorkonfigurierte Antriebe (build_*)
```

## Datenfluss eines Simulationsschritts

`Frequenzumrichter.step(speed_ref, load_torque, dt)`:

1. **Messung:** `motor.measurements(state)` → Ströme (αβ/abc), Drehzahl, Lage,
   Moment, Rotorfluss (`MotorMeasurements`).
2. **Schutz:** `protection.check(...)` prüft Strom-, Spannungs-, Drehzahl- und
   Thermo-Grenzen. Bei Trip → Pulssperre (Stellspannung 0).
3. **Regelung:** `controller.compute(speed_ref, meas, v_dc, dt)` liefert den
   Soll-Spannungsraumzeiger (`ControlOutput`).
4. **Modulation:** `inverter.modulate(...)` setzt ihn per SVPWM in
   Tastverhältnisse um und gibt den *tatsächlich* gestellten Zeiger zurück
   (inkl. Übermodulationsbegrenzung) sowie den Zwischenkreisstrom.
5. **Integration:** `rk4_step(motor.derivatives, ...)` integriert das Motormodell
   über `dt` (Zero-Order-Hold der Spannung).
6. **Zwischenkreis:** `dc_link.update(i_dc, dt)` aktualisiert die DC-Spannung.

## Einheitliche Schnittstellen

Dank zweier schlanker `Protocol`-Schnittstellen (`drive.Motor`,
`drive.Controller`) sind Maschinen und Regelverfahren frei kombinierbar.

### Eigenes Motormodell

```python
class MyMotor:
    n_states = 4
    def initial_state(self): ...
    def derivatives(self, x, v_alpha, v_beta, load_torque): ...
    def measurements(self, x) -> MotorMeasurements: ...
```

### Eigener Regler

```python
class MyController:
    def reset(self): ...
    def compute(self, speed_ref, meas, v_dc, dt) -> ControlOutput: ...
```

Beide lassen sich direkt in `Frequenzumrichter(motor=..., controller=...)`
einsetzen.

## Konventionen

* **Einheiten:** durchgängig SI (rad/s, A, V, Wb, Nm, s).
* **Bezugssysteme:** Maschinen-Ein-/Ausgang stets im **stationären αβ-System**
  (Wechselrichter-Sicht). Maschinen, die intern im dq-System rechnen (PMSM),
  transformieren selbst.
* **Transformationen:** amplituden-invariante Form (Faktor 2/3).
* **Drehzahl:** `speed_ref` ist die **mechanische** Winkelgeschwindigkeit
  $\omega_m$ [rad/s], nicht die elektrische.

## Numerik

* Integrator: klassisches **Runge-Kutta 4. Ordnung** (`rk4_step`).
* Standard-Abtast-/Schrittweite: `control_period = 100 µs` (10 kHz). Für sehr
  schnelle elektrische Zeitkonstanten (kleine Induktivität) ggf. `dt` verkleinern.
* Die Stellspannung wird über den Schritt konstant gehalten (ZOH) – konsistent mit
  einer digital abgetasteten Regelung.

## Tests

`tests/` enthält Einheitstests je Modul plus Integrationstests des geschlossenen
Regelkreises (`test_drive.py`). Ausführen mit `pytest`.

## Mögliche Erweiterungen

* schaltendes Wechselrichtermodell (PWM-Oberschwingungen, Totzeit, Schaltverluste)
* magnetische Sättigung / nichtlineare Induktivitäten
* geberlose Regelung (Flussbeobachter, EMK-/MRAS-Verfahren)
* Feldschwächung der PMSM ($i_d^* < 0$ oberhalb der Eckdrehzahl)
* Brake-Chopper / Rückspeise-Einheit im Zwischenkreis
* Strommess-Rauschen und Geberauflösung
