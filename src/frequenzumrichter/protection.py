"""Schutzfunktionen des Frequenzumrichters.

Ein realer Frequenzumrichter überwacht kontinuierlich kritische Größen und
schaltet bei Grenzwertverletzung ab (*Trip*), um Gerät und Maschine zu schützen.
Dieses Modul bildet die wichtigsten Schutzfunktionen nach:

* **Überstrom** (Kurzschluss-/Überlastschutz)
* **Überspannung** im Zwischenkreis (z.B. bei generatorischem Bremsen)
* **Unterspannung** im Zwischenkreis (Netzausfall)
* **Überdrehzahl**
* **Thermische Überlast** des Motors über ein ``I²t``-Modell
"""

from __future__ import annotations

import enum
from dataclasses import dataclass, field

__all__ = ["FaultType", "ProtectionLimits", "Protection"]


class FaultType(enum.Enum):
    """Mögliche Fehlerursachen, die zu einer Abschaltung führen."""

    OVERCURRENT = "Überstrom"
    OVERVOLTAGE = "Überspannung Zwischenkreis"
    UNDERVOLTAGE = "Unterspannung Zwischenkreis"
    OVERSPEED = "Überdrehzahl"
    THERMAL_OVERLOAD = "Thermische Überlast (I²t)"


@dataclass
class ProtectionLimits:
    """Grenzwerte für die Schutzfunktionen.

    Parameters
    ----------
    max_current:
        Spitzenstrom-Grenze (Betrag des Stromraumzeigers) [A].
    max_dc_voltage, min_dc_voltage:
        Zulässiger Bereich der Zwischenkreisspannung [V].
    max_speed:
        Betragsgrenze der mechanischen Drehzahl [rad/s].
    thermal_current:
        Dauerstrom (Nennstrom), oberhalb dessen sich das thermische Modell
        auflädt [A].
    thermal_time_constant:
        Thermische Zeitkonstante des ``I²t``-Modells [s].
    thermal_trip_level:
        Schwellwert der (normierten) thermischen Belastung, bei dem abgeschaltet
        wird. ``1.0`` entspricht 100 % zulässiger Erwärmung.
    """

    max_current: float = 50.0
    max_dc_voltage: float = 800.0
    min_dc_voltage: float = 350.0
    max_speed: float = 400.0
    thermal_current: float = 10.0
    thermal_time_constant: float = 10.0
    thermal_trip_level: float = 1.0


@dataclass
class Protection:
    """Überwacht Messgrößen und löst bei Grenzwertverletzung einen Trip aus.

    Solange :attr:`tripped` ``False`` ist, arbeitet der Umrichter normal. Nach
    einem Trip bleibt der Zustand erhalten, bis :meth:`reset` aufgerufen wird –
    so wie ein realer Umrichter erst nach Quittierung wieder anläuft.
    """

    limits: ProtectionLimits = field(default_factory=ProtectionLimits)
    tripped: bool = False
    faults: list[FaultType] = field(default_factory=list)
    thermal_state: float = 0.0  # normierte Erwärmung (I²t)

    def reset(self) -> None:
        """Quittiert alle Fehler und gibt den Umrichter wieder frei."""
        self.tripped = False
        self.faults = []
        self.thermal_state = 0.0

    def _update_thermal(self, current_magnitude: float, dt: float) -> None:
        """Aktualisiert das I²t-Erwärmungsmodell (PT1-artig)."""
        lim = self.limits
        # treibende Größe: (I/I_nenn)² - 1; oberhalb Nennstrom -> Erwärmung
        drive = (current_magnitude / lim.thermal_current) ** 2 - 1.0
        self.thermal_state += drive * dt / lim.thermal_time_constant
        if self.thermal_state < 0.0:
            self.thermal_state = 0.0

    def check(
        self,
        current_magnitude: float,
        dc_voltage: float,
        speed: float,
        dt: float,
    ) -> bool:
        """Prüft alle Schutzgrenzen für einen Zeitschritt.

        Parameters
        ----------
        current_magnitude:
            Betrag des Statorstromraumzeigers [A].
        dc_voltage:
            Zwischenkreisspannung [V].
        speed:
            Mechanische Drehzahl [rad/s].
        dt:
            Zeitschritt [s] (für das thermische Modell).

        Returns
        -------
        bool
            ``True``, wenn der Umrichter (weiterhin) abgeschaltet ist.
        """
        lim = self.limits
        self._update_thermal(current_magnitude, dt)

        faults: list[FaultType] = []
        if current_magnitude > lim.max_current:
            faults.append(FaultType.OVERCURRENT)
        if dc_voltage > lim.max_dc_voltage:
            faults.append(FaultType.OVERVOLTAGE)
        if dc_voltage < lim.min_dc_voltage:
            faults.append(FaultType.UNDERVOLTAGE)
        if abs(speed) > lim.max_speed:
            faults.append(FaultType.OVERSPEED)
        if self.thermal_state > lim.thermal_trip_level:
            faults.append(FaultType.THERMAL_OVERLOAD)

        if faults:
            self.tripped = True
            # neue Fehler ergänzen, Reihenfolge/Eindeutigkeit wahren
            for f in faults:
                if f not in self.faults:
                    self.faults.append(f)
        return self.tripped
