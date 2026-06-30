# Funktionsprinzip des Frequenzumrichters

Ein Frequenzumrichter (engl. *variable-frequency drive*, VFD) speist einen
Drehstrommotor mit einem Spannungssystem **variabler Frequenz und Amplitude** und
steuert so dessen Drehzahl und Drehmoment. Er ersetzt damit die früher üblichen
verlustbehafteten Verfahren (Vorwiderstände, polumschaltbare Motoren, mechanische
Getriebe) durch eine elektronische, nahezu verlustfreie Stellgröße.

## Warum überhaupt die Frequenz verstellen?

Die synchrone Drehzahl einer Drehfeldmaschine ist

$$ n_s = \frac{60 \cdot f}{p} \quad [\text{min}^{-1}] \qquad\text{bzw.}\qquad \omega_s = \frac{2\pi f}{p}\ [\text{rad/s}] $$

mit der Statorfrequenz `f` und der Polpaarzahl `p`. Die Drehzahl ist also direkt an
die Frequenz gekoppelt. Wer die Drehzahl stufenlos verstellen will, muss die
Frequenz verstellen – genau das leistet der Frequenzumrichter.

## Aufbau: der Spannungszwischenkreis-Umrichter

Der mit Abstand häufigste Aufbau ist der **U-Umrichter** (Spannungszwischenkreis)
mit drei Stufen:

```
  Netz 3~ ──►  Gleichrichter ──►  Zwischenkreis ──►  Wechselrichter ──► Motor
   (f fest)     (AC ➜ DC)         (DC, geglättet)     (DC ➜ AC, f variabel)
```

### 1. Gleichrichter

Eine ungesteuerte **6-Puls-Diodenbrücke (B6)** richtet das Drehstromnetz gleich. Im
Mittel entsteht die Zwischenkreisspannung

$$ V_{dc} \approx \frac{3\sqrt{2}}{\pi}\,V_{LL,\text{eff}} \approx 1{,}35 \cdot V_{LL,\text{eff}} $$

Bei 400 V verketteter Netzspannung sind das ca. 540 V; im Leerlauf lädt sich der
Kondensator bis auf den Scheitelwert $\sqrt 2 \cdot 400\,\text{V} \approx 566\,\text{V}$ auf.
→ `power_stage.Rectifier`

### 2. Zwischenkreis

Ein **Kondensator** glättet die Gleichspannung und puffert Energie. Seine
Spannungsdynamik folgt der Energiebilanz

$$ C\,\frac{dV_{dc}}{dt} = i_{\text{Quelle}} - i_{\text{Last}} $$

Die speisende Diodenbrücke kann nur Strom **liefern**, nicht aufnehmen. Beim
generatorischen Bremsen fließt Energie zurück in den Zwischenkreis, die Spannung
steigt – ohne Bremswiderstand (*Brake-Chopper*) bis zur Überspannungsabschaltung.
→ `power_stage.DCLink`

### 3. Wechselrichter

Drei Halbbrücken aus abschaltbaren Leistungshalbleitern (IGBT/MOSFET) erzeugen aus
der Gleichspannung wieder ein Drehspannungssystem. Über **Pulsweitenmodulation**
wird die gewünschte Grundschwingung von Frequenz und Amplitude eingestellt: Jeder
Brückenzweig schaltet mit einigen kHz zwischen oberem und unterem
Zwischenkreis-Pol; das Verhältnis der Einschaltdauern (Tastverhältnis) bestimmt die
mittlere Ausgangsspannung. → `power_stage.Inverter`, `pwm`

## Pulsweitenmodulation (PWM)

Zwei Verfahren sind implementiert:

* **Sinus-PWM (SPWM):** drei sinusförmige Referenzen werden mit einem
  Dreieck-Träger verglichen. Maximale lineare Phasenspannungsamplitude: $V_{dc}/2$.
* **Raumzeigermodulation (SVPWM):** betrachtet die drei Brückenspannungen als einen
  rotierenden Spannungs-*Raumzeiger* und schaltet zwischen den acht möglichen
  Schaltzuständen. Durch Injektion eines Nullsystems (`v_off = -(max+min)/2`,
  äquivalent zur 7-Segment-SVPWM) wird der lineare Bereich auf $V_{dc}/\sqrt 3$
  erweitert – rund **15 % mehr Spannung** als bei reiner Sinus-PWM.

→ `pwm.sinusoidal_pwm`, `pwm.svpwm_duty`

## Energiefluss und Wirkungsgrad

Der Wirkungsgrad eines Frequenzumrichters liegt typisch bei 95–98 %. Verluste
entstehen vor allem als Durchlass- und Schaltverluste der Halbleiter. Im
vorliegenden Mittelwertmodell werden diese vernachlässigt; modelliert wird der
*ideale* Leistungsfluss über die Bilanz

$$ P_{ac} = \frac{3}{2}\left(v_\alpha i_\alpha + v_\beta i_\beta\right) = V_{dc}\, i_{dc} $$

woraus der Zwischenkreisstrom geschätzt wird (`Inverter.modulate`). Bei
generatorischem Betrieb wird $i_{dc} < 0$ – die Maschine speist zurück.

## Weiterführend

* Regelungsverfahren (U/f, FOC): [`regelung.md`](regelung.md)
* Codeaufbau und Erweiterung: [`architektur.md`](architektur.md)
