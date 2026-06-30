"""Gemeinsame Hilfsfunktionen für die Beispielskripte."""

from __future__ import annotations

import os

import matplotlib

# Headless-Backend, damit die Beispiele auch ohne Display laufen (CI, Server).
matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "output")


def output_path(filename: str) -> str:
    """Liefert einen Pfad im Ausgabeverzeichnis und legt dieses bei Bedarf an."""
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    return os.path.join(OUTPUT_DIR, filename)


def savefig(fig, filename: str) -> str:
    """Speichert eine Figur als PNG und gibt den Pfad zurück."""
    path = output_path(filename)
    fig.savefig(path, dpi=120, bbox_inches="tight")
    plt.close(fig)
    print(f"  -> gespeichert: {path}")
    return path
