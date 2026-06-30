"""Leistungsteil des Frequenzumrichters: Gleichrichter, Zwischenkreis, Wechselrichter.

Der klassische Spannungszwischenkreis-Umrichter (U-Umrichter) besteht aus drei
Stufen:

1. **Gleichrichter** (:class:`Rectifier`) – wandelt das speisende Drehstromnetz
   in eine Gleichspannung. Hier als ungesteuerte 6-Puls-Diodenbrücke modelliert.
2. **Zwischenkreis** (:class:`DCLink`) – glättet die Gleichspannung über einen
   Kondensator und puffert Energie.
3. **Wechselrichter** (:class:`Inverter`) – erzeugt aus der Gleichspannung über
   PWM ein Drehspannungssystem variabler Frequenz und Amplitude.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .pwm import inverter_voltages, space_vector_pwm
from .transforms import clarke

__all__ = ["Rectifier", "DCLink", "Inverter", "InverterOutput"]


@dataclass
class Rectifier:
    """Ungesteuerte 6-Puls-Diodenbrücke (B6).

    Liefert aus der verketteten Netzspannung ``v_ll_rms`` die mittlere
    Leerlauf-Zwischenkreisspannung. Im Mittel gilt für die B6-Brücke

    .. math::

        V_{dc} \\approx \\frac{3\\sqrt{2}}{\\pi}\\, V_{LL,rms} \\approx 1.35\\, V_{LL,rms}

    Parameters
    ----------
    v_ll_rms:
        Verkettete Netz-Effektivspannung [V] (z.B. 400 V).
    """

    v_ll_rms: float = 400.0

    AVG_FACTOR: float = field(default=3.0 * np.sqrt(2.0) / np.pi, repr=False)

    @property
    def dc_voltage(self) -> float:
        """Mittlere Zwischenkreisspannung der Diodenbrücke [V]."""
        return self.AVG_FACTOR * self.v_ll_rms

    @property
    def peak_dc_voltage(self) -> float:
        """Spitzenwert (Leerlauf-Aufladung des Kondensators) [V]."""
        return np.sqrt(2.0) * self.v_ll_rms


@dataclass
class DCLink:
    """Zwischenkreis-Kondensator mit Spannungsdynamik.

    Modelliert die Energiebilanz ``C · dV_dc/dt = i_quelle - i_last``. Die
    speisende Diodenbrücke kann nur Strom **liefern**, nicht aufnehmen; sie wird
    als Spannungsquelle mit Innenwiderstand abgebildet:

    .. math::

        i_{quelle} = \\max\\!\\left(0,\\ \\frac{V_{nenn} - V_{dc}}{R_i}\\right)

    Beim generatorischen Bremsen (``i_last < 0``) lädt sich der Kondensator
    daher auf – es entsteht eine Überspannung, die ohne Bremswiderstand zur
    Abschaltung führt. Ist ``stiff=True``, wird die Spannung als ideal konstant
    angenommen (steifer Zwischenkreis), was für viele
    Regelungsbetrachtungen ausreicht.

    Parameters
    ----------
    capacitance:
        Kapazität ``C`` [F].
    voltage:
        Anfangsspannung [V].
    nominal_voltage:
        Leerlaufspannung der speisenden Brücke [V].
    source_resistance:
        Innenwiderstand der Quelle ``R_i`` [Ω].
    stiff:
        Wenn ``True``, bleibt die Spannung konstant.
    """

    capacitance: float = 1e-3
    voltage: float = 540.0
    nominal_voltage: float = 540.0
    source_resistance: float = 0.5
    stiff: bool = True

    def reset(self, voltage: float | None = None) -> None:
        self.voltage = self.nominal_voltage if voltage is None else voltage

    def source_current(self) -> float:
        """Vom Gleichrichter gelieferter Strom (nur positiv)."""
        return max(0.0, (self.nominal_voltage - self.voltage) / self.source_resistance)

    def update(self, i_load: float, dt: float) -> float:
        """Aktualisiert die Zwischenkreisspannung um einen Zeitschritt.

        ``i_load`` ist der vom Wechselrichter entnommene Gleichstrom (negativ bei
        Rückspeisung / generatorischem Betrieb).
        """
        if self.stiff:
            return self.voltage
        i_source = self.source_current()
        self.voltage += (i_source - i_load) / self.capacitance * dt
        # physikalische Untergrenze
        if self.voltage < 0.0:
            self.voltage = 0.0
        return self.voltage


@dataclass
class InverterOutput:
    """Ergebnis eines Wechselrichter-Schritts."""

    v_alpha: float
    v_beta: float
    v_abc: tuple[float, float, float]
    duties: tuple[float, float, float]
    i_dc: float  # aus der Leistungsbilanz geschätzter Zwischenkreisstrom [A]


@dataclass
class Inverter:
    """Zweistufiger Spannungswechselrichter (Mittelwertmodell) mit SVPWM.

    Der Wechselrichter erhält einen Soll-Spannungsraumzeiger ``(v_alpha_ref,
    v_beta_ref)`` und setzt ihn per Raumzeigermodulation in Tastverhältnisse um.
    Aufgrund der Begrenzung der Tastverhältnisse (Übermodulation) kann der
    tatsächlich gestellte Raumzeiger kleiner ausfallen; dieser *Ist*-Zeiger wird
    zurückgegeben.

    Parameters
    ----------
    deadtime_compensation:
        Platzhalter-Flag; das Mittelwertmodell vernachlässigt die Verriegelungs-
        zeit (Totzeit) der Brücke. Für detaillierte Studien kann hier eine
        Korrektur ergänzt werden.
    """

    deadtime_compensation: bool = False

    def modulate(
        self,
        v_alpha_ref: float,
        v_beta_ref: float,
        v_dc: float,
        i_alpha: float = 0.0,
        i_beta: float = 0.0,
    ) -> InverterOutput:
        """Setzt den Soll-Spannungsraumzeiger in Brückenansteuerung um.

        Parameters
        ----------
        v_alpha_ref, v_beta_ref:
            Soll-Spannungsraumzeiger [V].
        v_dc:
            Aktuelle Zwischenkreisspannung [V].
        i_alpha, i_beta:
            Aktueller Statorstromraumzeiger [A] – wird für die Schätzung des
            entnommenen Zwischenkreisstroms (Leistungsbilanz) verwendet.
        """
        duties, v_alpha_act, v_beta_act = space_vector_pwm(
            v_alpha_ref, v_beta_ref, v_dc
        )
        v_abc = inverter_voltages(*duties, v_dc)

        # Leistungsbilanz: P_ac = 1.5 (vα iα + vβ iβ); i_dc = P_ac / V_dc
        p_ac = 1.5 * (v_alpha_act * i_alpha + v_beta_act * i_beta)
        i_dc = p_ac / v_dc if v_dc > 1e-9 else 0.0

        return InverterOutput(
            v_alpha=float(v_alpha_act),
            v_beta=float(v_beta_act),
            v_abc=(float(v_abc[0]), float(v_abc[1]), float(v_abc[2])),
            duties=(float(duties[0]), float(duties[1]), float(duties[2])),
            i_dc=float(i_dc),
        )
