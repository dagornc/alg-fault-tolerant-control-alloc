#!/usr/bin/env python3
"""Mesure stricte de parité Rust/Python pour ALG_FAULT_TOLERANT_CONTROL_ALLOC."""

import argparse
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
REFERENCE = ROOT / "reference" / "sim_fault_tolerant_control_alloc.py"
SCENARIOS = ("nominal", "port_failure", "bow_degraded", "dual_fault", "saturation")


def data_line(command):
    completed = subprocess.run(command, check=True, capture_output=True, text=True)
    return completed.stdout.strip().splitlines()[-1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binaire", default=str(ROOT / "target" / "release" / "fault_tolerant_control_alloc_rs"))
    parser.add_argument("--graines", nargs=2, type=int, default=(1000, 1050), metavar=("DEBUT", "FIN"))
    parser.add_argument("--scenarios", nargs="*", default=SCENARIOS, choices=SCENARIOS)
    parser.add_argument("--steps", type=int, default=64)
    args = parser.parse_args()
    binary = Path(args.binaire)
    if not binary.exists():
        print(f"ERREUR : binaire introuvable : {binary}", file=sys.stderr)
        return 2
    total = 0
    differences = 0
    for scenario in args.scenarios:
        identical = 0
        scenario_total = 0
        for seed in range(args.graines[0], args.graines[1]):
            py_line = data_line([sys.executable, str(REFERENCE), str(seed), scenario, str(args.steps)])
            rs_line = data_line([str(binary), str(seed), scenario, str(args.steps)])
            total += 1
            scenario_total += 1
            if py_line == rs_line:
                identical += 1
            else:
                differences += 1
                if differences <= 10:
                    print(f"DIVERGENCE [{scenario}] seed={seed}")
                    print(f"  python: {py_line}")
                    print(f"  rust:   {rs_line}")
        print(f"scénario {scenario:14s}: {identical}/{scenario_total} identiques")
    print(f"TOTAL : {total - differences}/{total} identiques")
    print(f"COMPARAISONS : {total}")
    print(f"ÉCARTS : {differences}")
    if differences == 0:
        print("PARITÉ BIT-À-BIT VÉRIFIÉE")
        return 0
    print("PARITÉ NON VÉRIFIÉE")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
