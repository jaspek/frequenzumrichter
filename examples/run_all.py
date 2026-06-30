"""Runs all example scripts in sequence and generates the plots.

Usage:  python examples/run_all.py
"""

from __future__ import annotations

import importlib
import sys

EXAMPLES = [
    "01_vf_anlauf",
    "02_foc_pmsm_drehzahlsprung",
    "03_svpwm_demo",
    "04_lastsprung_und_schutz",
    "05_transformationen",
]


def main() -> None:
    sys.path.insert(0, ".")
    for name in EXAMPLES:
        module = importlib.import_module(name)
        module.main()
        print()
    print("All examples executed. Plots are located in examples/output/.")


if __name__ == "__main__":
    main()
