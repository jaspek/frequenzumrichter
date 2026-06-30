"""Common helper functions for the example scripts."""

from __future__ import annotations

import os

import matplotlib

# Headless backend so the examples also run without a display (CI, server).
matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "output")


def output_path(filename: str) -> str:
    """Returns a path in the output directory and creates it if needed."""
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    return os.path.join(OUTPUT_DIR, filename)


def savefig(fig, filename: str) -> str:
    """Saves a figure as PNG and returns the path."""
    path = output_path(filename)
    fig.savefig(path, dpi=120, bbox_inches="tight")
    plt.close(fig)
    print(f"  -> saved: {path}")
    return path
