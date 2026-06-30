"""Pulsweitenmodulation (PWM) für die Wechselrichter-Brücke.

Implementiert die beiden in Frequenzumrichtern gebräuchlichsten
Modulationsverfahren:

* **Sinus-PWM (SPWM)** – Vergleich dreier Sinus-Referenzen mit einem
  Dreieck-Träger.
* **Raumzeiger-Modulation (SVPWM)** – realisiert über die mathematisch
  äquivalente *Min/Max*-Nullsystem-Injektion. Dadurch wird der lineare
  Aussteuerbereich gegenüber der reinen Sinus-PWM um den Faktor
  ``2/√3 ≈ 1.155`` erweitert (Phasenspannungsamplitude bis ``V_dc/√3``).

Die Funktionen liefern *Tastverhältnisse* (Duty-Cycles) im Bereich
``[0, 1]`` für die drei oberen Brückentransistoren. Aus diesen lässt sich mit
:func:`inverter_voltages` die mittlere Strangspannung (Mittelwertmodell)
zurückrechnen.
"""

from __future__ import annotations

import numpy as np

from .transforms import clarke, inverse_clarke

__all__ = [
    "sinusoidal_pwm",
    "space_vector_pwm",
    "svpwm_duty",
    "inverter_voltages",
    "svpwm_sector",
    "MAX_LINEAR_SVPWM",
    "MAX_LINEAR_SPWM",
]

#: Maximale Phasenspannungsamplitude im linearen Bereich der SVPWM (= V_dc/√3).
MAX_LINEAR_SVPWM = 1.0 / np.sqrt(3.0)
#: Maximale Phasenspannungsamplitude im linearen Bereich der Sinus-PWM (= V_dc/2).
MAX_LINEAR_SPWM = 0.5


def sinusoidal_pwm(v_a, v_b, v_c, v_dc):
    """Sinus-PWM: berechnet Tastverhältnisse aus den Strang-Sollspannungen.

    Parameters
    ----------
    v_a, v_b, v_c:
        Soll-Strangspannungen (gegen den virtuellen Sternpunkt) [V].
    v_dc:
        Zwischenkreis-Gleichspannung [V].

    Returns
    -------
    (d_a, d_b, d_c):
        Auf ``[0, 1]`` begrenzte Tastverhältnisse.
    """
    d_a = np.clip(0.5 + v_a / v_dc, 0.0, 1.0)
    d_b = np.clip(0.5 + v_b / v_dc, 0.0, 1.0)
    d_c = np.clip(0.5 + v_c / v_dc, 0.0, 1.0)
    return d_a, d_b, d_c


def svpwm_duty(v_alpha, v_beta, v_dc):
    """Raumzeiger-Modulation über Min/Max-Injektion.

    Parameters
    ----------
    v_alpha, v_beta:
        Soll-Raumzeiger der Spannung im stationären αβ-System [V].
    v_dc:
        Zwischenkreisspannung [V].

    Returns
    -------
    (d_a, d_b, d_c):
        Tastverhältnisse im Bereich ``[0, 1]``.

    Notes
    -----
    Die Injektion des Nullsystems ``v_off = -(max + min) / 2`` zentriert die
    Strangspannungen symmetrisch um ``V_dc/2`` und entspricht exakt der
    klassischen 7-Segment-Raumzeigermodulation (SVPWM).
    """
    v_a, v_b, v_c = inverse_clarke(v_alpha, v_beta)
    v_max = np.maximum(np.maximum(v_a, v_b), v_c)
    v_min = np.minimum(np.minimum(v_a, v_b), v_c)
    v_off = -0.5 * (v_max + v_min)
    d_a = np.clip(0.5 + (v_a + v_off) / v_dc, 0.0, 1.0)
    d_b = np.clip(0.5 + (v_b + v_off) / v_dc, 0.0, 1.0)
    d_c = np.clip(0.5 + (v_c + v_off) / v_dc, 0.0, 1.0)
    return d_a, d_b, d_c


def space_vector_pwm(v_alpha, v_beta, v_dc):
    """Wie :func:`svpwm_duty`, gibt zusätzlich die realisierten αβ-Spannungen zurück.

    Durch die Begrenzung der Tastverhältnisse auf ``[0, 1]`` (Übermodulation)
    kann die tatsächlich gestellte Spannung von der Sollspannung abweichen.
    Diese *Ist*-Spannung wird mit zurückgegeben, damit die Regelung den
    tatsächlich am Motor wirksamen Spannungsraumzeiger kennt.

    Returns
    -------
    (duties, v_alpha_act, v_beta_act):
        ``duties`` ist das Tupel ``(d_a, d_b, d_c)``; ``v_*_act`` ist der
        tatsächlich gestellte Spannungsraumzeiger [V].
    """
    duties = svpwm_duty(v_alpha, v_beta, v_dc)
    v_a, v_b, v_c = inverter_voltages(*duties, v_dc)
    v_alpha_act, v_beta_act = clarke(v_a, v_b, v_c)
    return duties, v_alpha_act, v_beta_act


def inverter_voltages(d_a, d_b, d_c, v_dc):
    """Mittlere Strangspannungen aus Tastverhältnissen (Mittelwertmodell).

    Die Spannung eines Brückenzweigs gegen den negativen Zwischenkreis-Pol ist
    ``d_x * V_dc``. Für eine symmetrische Last (isolierter Sternpunkt) ergibt
    sich die Strang-Sternpunkt-Spannung durch Abzug des Mittelwerts.

    Returns
    -------
    (v_a, v_b, v_c):
        Strangspannungen gegen den Laststernpunkt [V].
    """
    v_a0 = d_a * v_dc
    v_b0 = d_b * v_dc
    v_c0 = d_c * v_dc
    v_n = (v_a0 + v_b0 + v_c0) / 3.0
    return v_a0 - v_n, v_b0 - v_n, v_c0 - v_n


def svpwm_sector(v_alpha, v_beta):
    """Bestimmt den Sektor (1..6) des Spannungsraumzeigers im αβ-Diagramm.

    Dient vor allem der Veranschaulichung der Raumzeigermodulation.
    """
    angle = np.arctan2(v_beta, v_alpha)
    angle = np.mod(angle, 2.0 * np.pi)
    sector = int(np.floor(angle / (np.pi / 3.0))) + 1
    return sector
