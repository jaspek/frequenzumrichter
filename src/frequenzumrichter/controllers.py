"""Regler-Bausteine: PI-Regler, Begrenzer und Filter.

Die hier definierten Klassen bilden die zeitdiskreten Grundbausteine der
Regelung eines Frequenzumrichters. Alle Regler arbeiten mit einer festen
Abtastzeit ``dt``, die bei jedem Aufruf von :meth:`step` übergeben wird.
"""

from __future__ import annotations

from dataclasses import dataclass, field

__all__ = ["PIController", "RateLimiter", "LowPassFilter"]


@dataclass
class PIController:
    """Zeitdiskreter PI-Regler mit Stellgrößenbegrenzung und Anti-Windup.

    Der Regler implementiert das Stellgesetz

    .. math::

        u(t) = K_p \, e(t) + K_i \int e(\\tau)\, d\\tau

    Die Integration erfolgt nach der Vorwärts-Euler-Methode. Gegen
    Integrator-Windup wird *Back-Calculation* (Anti-Windup über die
    Rückführung der Sättigungsdifferenz) eingesetzt: Sobald die Stellgröße in
    die Begrenzung läuft, wird der Integralanteil entsprechend zurückgerechnet.

    Parameters
    ----------
    kp:
        Proportionalverstärkung.
    ki:
        Integralverstärkung [1/s].
    output_min, output_max:
        Untere/obere Begrenzung der Stellgröße. ``None`` => unbegrenzt.
    anti_windup_gain:
        Verstärkung ``Kaw`` der Back-Calculation. Üblich ist ``1/Tt`` mit der
        Nachstellzeit ``Tt``. ``0`` deaktiviert das Anti-Windup (reines Clamping
        bleibt aktiv, sofern Begrenzungen gesetzt sind).
    """

    kp: float
    ki: float
    output_min: float | None = None
    output_max: float | None = None
    anti_windup_gain: float = 0.0
    integral: float = field(default=0.0)
    _last_output: float = field(default=0.0, repr=False)

    def reset(self, integral: float = 0.0) -> None:
        """Setzt den Integralspeicher (und damit den Regler) zurück."""
        self.integral = integral
        self._last_output = 0.0

    def step(self, error: float, dt: float, feedforward: float = 0.0) -> float:
        """Berechnet einen Reglerschritt.

        Parameters
        ----------
        error:
            Regelabweichung ``sollwert - istwert``.
        dt:
            Abtastzeit [s].
        feedforward:
            Optionaler Vorsteueranteil, der vor der Begrenzung addiert wird
            (z.B. Entkopplungsterme bei der feldorientierten Regelung).

        Returns
        -------
        float
            Die begrenzte Stellgröße.

        Notes
        -----
        :attr:`integral` enthält den bereits mit ``Ki`` gewichteten Integral­anteil
        in Stellgrößen-Einheiten. Die Stellgröße wird zunächst aus dem aktuellen
        Integralspeicher gebildet und begrenzt; anschließend wird der Speicher
        per Back-Calculation aktualisiert. Läuft die Stellgröße in die Sättigung,
        bremst der Term ``Kaw·(u_sat - u_unsat)`` das weitere Aufintegrieren
        (Anti-Windup).
        """
        unbounded = self.kp * error + self.integral + feedforward
        bounded = self._clamp(unbounded)

        # Integrator nach klassischer Back-Calculation aktualisieren
        self.integral += (
            self.ki * error + self.anti_windup_gain * (bounded - unbounded)
        ) * dt

        self._last_output = bounded
        return bounded

    def _clamp(self, value: float) -> float:
        if self.output_min is not None and value < self.output_min:
            return self.output_min
        if self.output_max is not None and value > self.output_max:
            return self.output_max
        return value


@dataclass
class RateLimiter:
    """Begrenzt die Änderungsgeschwindigkeit eines Signals (Rampe).

    Wird typischerweise verwendet, um Sollwertsprünge (Drehzahl, Frequenz) in
    sanfte Rampen zu wandeln und so Strom-/Drehmomentstöße zu vermeiden.

    Parameters
    ----------
    rate_up, rate_down:
        Maximale Anstiegs-/Abfallgeschwindigkeit [Einheit/s]. ``rate_down``
        verwendet bei ``None`` denselben Wert wie ``rate_up``.
    """

    rate_up: float
    rate_down: float | None = None
    value: float = 0.0

    def reset(self, value: float = 0.0) -> None:
        self.value = value

    def step(self, target: float, dt: float) -> float:
        rate_down = self.rate_up if self.rate_down is None else self.rate_down
        max_up = self.rate_up * dt
        max_down = rate_down * dt
        delta = target - self.value
        if delta > max_up:
            delta = max_up
        elif delta < -max_down:
            delta = -max_down
        self.value += delta
        return self.value


@dataclass
class LowPassFilter:
    """Zeitdiskretes PT1-Tiefpassfilter erster Ordnung.

    Realisiert ``G(s) = 1 / (1 + s*tau)`` mittels Vorwärts-Euler-Diskretisierung.
    """

    tau: float
    value: float = 0.0

    def reset(self, value: float = 0.0) -> None:
        self.value = value

    def step(self, input_value: float, dt: float) -> float:
        if self.tau <= 0.0:
            self.value = input_value
            return self.value
        alpha = dt / (self.tau + dt)
        self.value += alpha * (input_value - self.value)
        return self.value
