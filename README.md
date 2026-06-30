# Frequenzumrichter

[![CI](https://github.com/jaspek/frequenzumrichter/actions/workflows/ci.yml/badge.svg)](https://github.com/jaspek/frequenzumrichter/actions/workflows/ci.yml)
![Python](https://img.shields.io/badge/python-3.9%2B-blue)
![License](https://img.shields.io/badge/license-MIT-green)

Eine vollständige, gut dokumentierte **Simulation eines Frequenzumrichters** (engl.
*variable-frequency drive*, VFD) in reinem Python/NumPy. Das Paket modelliert den
gesamten Antriebsstrang – vom speisenden Netz über den Leistungsteil bis zur
geregelten Drehfeldmaschine – und eignet sich für Lehre, Reglerentwurf und
Vorab-Simulationen.

> **Was ist ein Frequenzumrichter?** Ein Frequenzumrichter erzeugt aus einem Netz
> mit fester Frequenz ein Drehspannungssystem **variabler Frequenz und Amplitude**
> und steuert damit Drehzahl und Drehmoment eines Drehstrommotors stufenlos. Er ist
> das Herzstück nahezu jedes modernen elektrischen Antriebs – von der Pumpe über das
> Förderband bis zum Elektrofahrzeug.

---

## Inhalt / Features

| Baustein | Modul | Beschreibung |
|----------|-------|--------------|
| **Gleichrichter** | `power_stage.Rectifier` | Ungesteuerte B6-Diodenbrücke (`V_dc ≈ 1.35·V_LL`) |
| **Zwischenkreis** | `power_stage.DCLink` | Kondensatormodell, steif oder dynamisch (Rückspeise-Überspannung) |
| **Wechselrichter** | `power_stage.Inverter` | 2-Level-Spannungswechselrichter (Mittelwertmodell) mit SVPWM |
| **PWM** | `pwm` | Sinus-PWM und Raumzeigermodulation (SVPWM via Min/Max-Injektion) |
| **Asynchronmaschine** | `motor.InductionMotor` | Käfigläufer im stationären αβ-Modell |
| **Synchronmaschine** | `motor.PMSM` | Permanenterregte PMSM im rotorfesten dq-Modell |
| **U/f-Steuerung** | `vf_control.VFControl` | Skalare Kennliniensteuerung mit Spannungsanhebung |
| **Feldorientierte Regelung** | `foc.FOCPMSM`, `foc.FOCInduction` | Vektorregelung (PMSM & indirekte IRFOC der ASM) |
| **Regler** | `controllers` | PI-Regler mit Anti-Windup, Rampe, PT1-Filter |
| **Schutz** | `protection.Protection` | Über-/Unterspannung, Überstrom, Überdrehzahl, I²t-Thermomodell |
| **Transformationen** | `transforms` | Clarke / Park (amplituden-invariant) |
| **Simulation** | `simulation`, `drive.Frequenzumrichter` | RK4-Integrator, geschlossener Regelkreis, Datenaufzeichnung |

---

## Installation

```bash
git clone https://github.com/jaspek/frequenzumrichter.git
cd frequenzumrichter
pip install -e .            # Kernpaket (nur NumPy)
pip install -e ".[dev]"     # zusätzlich pytest + matplotlib (Tests & Plots)
```

Benötigt Python ≥ 3.9 und NumPy. Für die Beispiel-Plots wird Matplotlib verwendet.

---

## Schnellstart

```python
from frequenzumrichter import build_foc_pmsm_drive

# Vorkonfigurierter Servoantrieb (PMSM + feldorientierte Regelung)
fu = build_foc_pmsm_drive()

# Drehzahlsprung auf 100 rad/s mit 0.5 Nm Last, 0.5 s simulieren
result = fu.run(t_end=0.5, speed_ref=100.0, load_torque=0.5)

print("Enddrehzahl :", round(float(result.speed[-1]), 2), "rad/s")
print("Ausregelzeit:", round(result.settling_time(), 4), "s")
```

```
Enddrehzahl : 100.0 rad/s
Ausregelzeit: 0.0278 s
```

### Zeit- und drehzahlabhängige Vorgaben

Sowohl Drehzahlsollwert als auch Lastmoment dürfen Konstanten **oder** Funktionen
sein:

```python
# Reversierung über einen Sollwertsprung
speed_ref = lambda t: 80.0 if t < 0.25 else -80.0

# Passive Lüfterlast ∝ ω² (verschwindet bei Stillstand): f(t, meas)
fan = lambda t, meas: 1e-4 * meas.omega_m * abs(meas.omega_m)

result = fu.run(t_end=0.5, speed_ref=speed_ref, load_torque=fan)
```

---

## Beispiele

Im Ordner [`examples/`](examples/) liegen lauffähige Skripte, die jeweils einen
Plot nach `examples/output/` schreiben:

```bash
python examples/run_all.py          # alle Beispiele auf einmal
python examples/01_vf_anlauf.py     # einzeln
```

| Skript | Inhalt |
|--------|--------|
| `01_vf_anlauf.py` | Sanftanlauf einer ASM mit U/f-Steuerung |
| `02_foc_pmsm_drehzahlsprung.py` | Hochdynamische PMSM-Regelung, Sprünge & Reversierung |
| `03_svpwm_demo.py` | SVPWM-Tastverhältnisse und erweiterter Aussteuerbereich |
| `04_lastsprung_und_schutz.py` | Störgrößenverhalten + thermische Abschaltung |
| `05_transformationen.py` | Clarke-/Park-Transformation anschaulich |

---

## Architektur

```
            Netz (3~)
               │
        ┌──────▼──────┐
        │ Gleichrichter│  Rectifier      AC ➜ DC
        └──────┬──────┘
        ┌──────▼──────┐
        │ Zwischenkreis│  DCLink         Glättung / Energiepuffer
        └──────┬──────┘
        ┌──────▼──────┐
        │Wechselrichter│  Inverter+SVPWM DC ➜ AC (variabel)
        └──────┬──────┘
               │  v_alpha, v_beta
        ┌──────▼──────┐        ┌─────────────┐
        │    Motor    │◄───────│   Regelung   │  U/f  oder  FOC
        │ ASM / PMSM  │ Messung│  (PI-Kaskade)│
        └─────────────┘───────►└─────────────┘
                  ▲
            ┌─────┴─────┐
            │   Schutz   │  Protection (Trip)
            └───────────┘
```

Die Klasse `Frequenzumrichter` (`drive.py`) verbindet alle Komponenten zu einem
geschlossenen Regelkreis und integriert das Motormodell mit einem
Runge-Kutta-Verfahren 4. Ordnung (Zero-Order-Hold der Stellspannung über die
Abtastperiode).

Eine ausführliche Beschreibung findet sich in [`docs/`](docs/):

* [`docs/theorie.md`](docs/theorie.md) – Funktionsprinzip des Frequenzumrichters
* [`docs/regelung.md`](docs/regelung.md) – U/f-Steuerung und feldorientierte Regelung
* [`docs/architektur.md`](docs/architektur.md) – Aufbau des Codes & Erweiterung

---

## Tests

```bash
pytest                      # 76 Tests (Einheit + Integration)
```

Die Testsuite deckt Transformationen, PWM, Regler, Motormodelle, Leistungsteil,
Schutzfunktionen sowie den geschlossenen Regelkreis (Sollwertfolge,
Störgrößenverhalten, Reversierung, Schutzauslösung) ab.

---

## Modellannahmen & Grenzen

* **Mittelwertmodell** des Wechselrichters – die Schaltvorgänge (PWM-Oberschwingungen,
  Totzeit) werden über die Abtastperiode gemittelt. Für Stromregler- und
  Drehzahlbetrachtungen ist das adäquat; für EMV-/Schaltverlust-Analysen wäre ein
  schaltendes Modell nötig.
* **Lineare Magnetik** – Sättigung und Eisenverluste sind nicht modelliert.
* **Ideale Messung** – Strom-/Lagegeber ohne Rauschen und Verzögerung.

Diese Vereinfachungen sind bewusst gewählt, um Funktionsprinzip und Regelung klar
und numerisch robust abzubilden. Die Modulstruktur erlaubt es, einzelne Bausteine
(z.B. ein schaltendes Wechselrichtermodell oder eine Sättigungskennlinie) gezielt
zu verfeinern.

---

## Lizenz

MIT – siehe [`LICENSE`](LICENSE).
