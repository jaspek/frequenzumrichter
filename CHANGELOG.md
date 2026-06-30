# Changelog

Alle nennenswerten Änderungen an diesem Projekt werden hier dokumentiert.
Das Format orientiert sich an [Keep a Changelog](https://keepachangelog.com/de/1.0.0/).

## [0.1.0] – 2026-06-30

### Hinzugefügt

* **Leistungsteil**
  * `Rectifier` – ungesteuerte B6-Diodenbrücke (`V_dc ≈ 1.35·V_LL`)
  * `DCLink` – Zwischenkreis-Kondensator, steif oder dynamisch (Rückspeise-Überspannung)
  * `Inverter` – 2-Level-Spannungswechselrichter (Mittelwertmodell) mit SVPWM
* **Modulation** (`pwm`)
  * Sinus-PWM und Raumzeigermodulation (SVPWM via Min/Max-Nullsystem-Injektion)
  * Mittelwertmodell der Strangspannungen, Sektorbestimmung
* **Maschinen** (`motor`)
  * `InductionMotor` – Asynchronmaschine im stationären αβ-Modell
  * `PMSM` – permanenterregte Synchronmaschine im rotorfesten dq-Modell
* **Regelung**
  * `VFControl` – skalare U/f-Kennliniensteuerung mit Spannungsanhebung und Rampe
  * `FOCPMSM`, `FOCInduction` – feldorientierte Regelung (PMSM & indirekte IRFOC)
  * `PIController` mit Anti-Windup (Back-Calculation), `RateLimiter`, `LowPassFilter`
* **Transformationen** (`transforms`) – Clarke/Park (amplituden-invariant)
* **Schutz** (`protection`) – Über-/Unterspannung, Überstrom, Überdrehzahl, I²t-Thermomodell
* **Simulation**
  * `rk4_step` (Runge-Kutta 4. Ordnung), `SimulationResult` mit Ausregelzeit-Schätzung
  * `Frequenzumrichter` – Gesamtmodell des geschlossenen Regelkreises
  * Lastvorgabe als Konstante, `f(t)` oder zustandsabhängig `f(t, meas)` (passive Lasten)
* **Factory** – `build_vf_drive`, `build_foc_pmsm_drive`, `build_foc_induction_drive`
* **Beispiele** – 5 lauffähige Skripte mit Plots (Anlauf, Reversierung, SVPWM, Schutz, Transformationen)
* **Tests** – 76 Einheits- und Integrationstests
* **Dokumentation** – Theorie, Regelung, Architektur (`docs/`)
