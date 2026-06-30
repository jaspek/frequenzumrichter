"""Gemeinsame Typen und Hilfsfunktionen für die Regelungsverfahren."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

__all__ = ["ControlOutput", "limit_vector"]


@dataclass
class ControlOutput:
    """Ergebnis eines Regelschritts.

    Neben dem eigentlichen Soll-Spannungsraumzeiger ``(v_alpha_ref,
    v_beta_ref)`` werden diagnostische Größen mitgeführt, die für Analyse und
    Visualisierung nützlich sind.
    """

    v_alpha_ref: float
    v_beta_ref: float
    theta_field: float = 0.0  # verwendeter Feldwinkel [rad]
    i_d: float = 0.0  # gemessener d-Strom [A]
    i_q: float = 0.0  # gemessener q-Strom [A]
    i_d_ref: float = 0.0  # Soll-d-Strom [A]
    i_q_ref: float = 0.0  # Soll-q-Strom [A]
    v_d: float = 0.0  # Soll-d-Spannung [V]
    v_q: float = 0.0  # Soll-q-Spannung [V]


def limit_vector(d: float, q: float, max_magnitude: float) -> tuple[float, float]:
    """Begrenzt den Betrag eines 2-D-Vektors auf einen Kreis (Circle-Limiting).

    Übersteigt der Betrag ``sqrt(d² + q²)`` die Grenze ``max_magnitude``, werden
    beide Komponenten winkeltreu herunterskaliert. Dies entspricht der
    Spannungsbegrenzung der Stromregler auf den realisierbaren
    Aussteuer-Kreis des Wechselrichters.
    """
    magnitude = float(np.hypot(d, q))
    if magnitude <= max_magnitude or magnitude < 1e-12:
        return d, q
    scale = max_magnitude / magnitude
    return d * scale, q * scale
