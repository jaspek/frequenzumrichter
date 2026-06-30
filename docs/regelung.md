# Regelungsverfahren

Der Frequenzumrichter muss aus dem Drehzahl-Sollwert die richtigen
Wechselrichter-Spannungen berechnen. Dafür sind zwei grundlegend verschiedene
Verfahren implementiert: die **skalare U/f-Steuerung** und die **feldorientierte
Regelung (FOC)**.

---

## 1. Skalare U/f-Steuerung

Die einfachste Methode hält das Verhältnis von Spannung zu Frequenz konstant,
damit der magnetische Fluss (und damit das verfügbare Drehmoment) über den
Drehzahlbereich annähernd gleich bleibt:

$$ U(f) = U_{boost} + \frac{U_{nenn}}{f_{nenn}} \cdot f $$

* **Spannungsanhebung `U_boost`** kompensiert bei kleinen Frequenzen den ohmschen
  Spannungsabfall am Statorwiderstand und sichert das Anlaufmoment.
* Oberhalb der Nennfrequenz bleibt die Spannung konstant (Begrenzung durch den
  Zwischenkreis) → **Feldschwächbereich** mit sinkendem Moment.

Die U/f-Steuerung arbeitet **gesteuert** (ohne Rückführung): kein Strom-, kein
Drehzahlgeber nötig. Die tatsächliche Drehzahl liegt lastabhängig um den
**Schlupf** unter der Synchrondrehzahl. Das Verfahren ist extrem robust und
preiswert – ideal für Lüfter, Pumpen und einfache Förderantriebe.

→ Implementierung: `vf_control.VFControl`

```python
from frequenzumrichter import build_vf_drive
fu = build_vf_drive()
result = fu.run(t_end=2.5, speed_ref=150.0, load_torque=3.0)
# -> Enddrehzahl ≈ 149 rad/s (Synchrondrehzahl minus Schlupf)
```

**Grenzen:** kein definiertes Drehmoment, schwaches Verhalten bei kleinen
Drehzahlen, langsame Dynamik. Für hochdynamische oder positioniergenaue Antriebe
ist die feldorientierte Regelung nötig.

---

## 2. Feldorientierte Regelung (FOC / Vektorregelung)

Die FOC überträgt das Prinzip der fremderregten Gleichstrommaschine auf die
Drehfeldmaschine: In einem **rotierenden dq-Koordinatensystem**, das mit dem Fluss
mitläuft, werden Fluss und Moment **entkoppelt** geregelt:

* der **d-Strom** $i_d$ stellt den magnetischen Fluss,
* der **q-Strom** $i_q$ stellt das Drehmoment $\;M = \tfrac{3}{2}p\,\psi\,i_q$.

### Koordinatentransformationen

```
abc ──Clarke──► αβ ──Park(θ)──► dq      (Messung)
dq  ──Park⁻¹(θ)──► αβ ──SVPWM──► Brücke  (Stellgröße)
```

Der entscheidende Punkt ist der **Feldwinkel θ**:

* **PMSM** – der Fluss steht fest im Rotor, also ist $\theta_e = p\,\theta_m$
  direkt aus der Rotorlage bekannt (Geber). Es gilt $i_d^* = 0$
  (Oberflächen-PMSM); der gesamte Strom bildet Moment.
  → `foc.FOCPMSM`
* **Asynchronmaschine** – der Rotorfluss „schlüpft" gegenüber dem Rotor. Der
  Feldwinkel wird **indirekt** über die Schlupfbeziehung berechnet (IRFOC):

  $$ \omega_{sl} = \frac{i_q^{*}}{\tau_r\, i_d^{*}}, \qquad
     \theta_{feld} = \int \left(p\,\omega_m + \omega_{sl}\right) dt $$

  Hier stellt $i_d^*$ den Rotorfluss ($\psi_r = L_m i_d^*$), $i_q^*$ das Moment.
  → `foc.FOCInduction`

### Kaskadenstruktur

```
 ω_soll ─►(PI Drehzahl)─► i_q* ─►(PI Strom q)─► v_q ┐
                                                    ├─► Park⁻¹ ─► SVPWM
 i_d* ──────────────────►(PI Strom d)─► v_d ────────┘
```

* **Innere Stromregler** (schnell, ~kHz-Bandbreite) – nach dem *Betragsoptimum*
  ausgelegt: $K_p = L\,\omega_c,\; K_i = R\,\omega_c$.
* **Äußerer Drehzahlregler** (langsamer) – seine Stellgröße ist der
  Soll-q-Strom, begrenzt auf den Maximalstrom (= Momentbegrenzung).
* **Entkopplungs-Vorsteuerung** kompensiert die geschwindigkeitsproportionalen
  Kreuzkopplungsterme ($\omega_e L_q i_q$ bzw. $\omega_e(L_d i_d + \psi_f)$) und
  die Gegen-EMK – dadurch werden die beiden Stromregelkreise näherungsweise
  linear und unabhängig.
* **Spannungsbegrenzung** auf den Aussteuerkreis $V_{dc}/\sqrt3$ (Circle-Limiting,
  `control_base.limit_vector`).

→ Reglerbaustein: `controllers.PIController` (mit Anti-Windup per
Back-Calculation).

```python
from frequenzumrichter import build_foc_pmsm_drive
fu = build_foc_pmsm_drive()
result = fu.run(t_end=0.5, speed_ref=100.0, load_torque=0.5)
# -> Ausregelzeit < 30 ms, exakte Sollwertfolge, sauberer Lastausgleich
```

---

## 3. Anti-Windup

Läuft die Stellgröße eines PI-Reglers in die Begrenzung (z.B. Stromgrenze bei
starker Beschleunigung), würde der Integralanteil unkontrolliert weiter
aufintegrieren (*Windup*) und nach dem Verlassen der Begrenzung ein großes
Überschwingen verursachen. Die Implementierung nutzt **Back-Calculation**:

$$ I_{k+1} = I_k + \Big(K_i\,e + K_{aw}\,(u_{sat} - u_{unsat})\Big)\,\Delta t $$

Der Term $K_{aw}(u_{sat}-u_{unsat})$ ist nur in der Sättigung aktiv und führt den
Integrator zurück, sodass der Regler beim Verlassen der Begrenzung sofort wieder
reagiert.

---

## Vergleich

| Eigenschaft | U/f-Steuerung | Feldorientierte Regelung |
|-------------|:-------------:|:------------------------:|
| Rückführung | keine | Strom + Lage/Drehzahl |
| Dynamik | gering | sehr hoch |
| Drehmoment bei n≈0 | schwach | volles Moment |
| Genauigkeit | Schlupf-behaftet | exakt |
| Aufwand | gering | hoch |
| typische Anwendung | Pumpe, Lüfter | Servo, Traktion, Werkzeugmaschine |
