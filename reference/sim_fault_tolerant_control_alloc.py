#!/usr/bin/env python3
"""Référence normative miroir pour ALG_FAULT_TOLERANT_CONTROL_ALLOC."""

import math
import random
import sys

ACTUATORS = 4
DOF = 3
REGULARIZATION = 1.0e-9
ALLOCATION = (
    (1.0, 1.0, 0.0, 0.0),
    (0.0, 0.0, 1.0, 1.0),
    (-1.0, 1.0, 1.0, -1.0),
)
SCENARIOS = {
    "nominal": ((1.0, 1.0, 1.0, 1.0), (1.0, 1.0, 0.8, 0.8), 0.8),
    "port_failure": ((0.0, 1.0, 1.0, 1.0), (1.0, 1.0, 0.8, 0.8), 0.8),
    "bow_degraded": ((1.0, 1.0, 0.35, 1.0), (1.0, 1.0, 0.8, 0.8), 0.8),
    "dual_fault": ((0.0, 1.0, 0.0, 1.0), (1.0, 1.0, 0.8, 0.8), 0.8),
    "saturation": ((1.0, 1.0, 1.0, 1.0), (0.35, 0.35, 0.35, 0.35), 2.5),
}


def solve_3x3(a, b):
    a = [list(row) for row in a]
    b = list(b)
    for col in range(DOF):
        pivot = col
        for row in range(col + 1, DOF):
            if abs(a[row][col]) > abs(a[pivot][col]):
                pivot = row
        if pivot != col:
            a[col], a[pivot] = a[pivot], a[col]
            b[col], b[pivot] = b[pivot], b[col]
        if abs(a[col][col]) < 1.0e-15:
            continue
        for row in range(col + 1, DOF):
            factor = a[row][col] / a[col][col]
            for k in range(col, DOF):
                a[row][k] = a[row][k] - factor * a[col][k]
            b[row] = b[row] - factor * b[col]
    x = [0.0] * DOF
    for row in range(DOF - 1, -1, -1):
        rhs = b[row]
        for k in range(row + 1, DOF):
            rhs = rhs - a[row][k] * x[k]
        if abs(a[row][row]) >= 1.0e-15:
            x[row] = rhs / a[row][row]
    return x


def allocate(demand, health, limits):
    commands = [0.0] * ACTUATORS
    fixed = [False] * ACTUATORS
    failed = 0
    for i in range(ACTUATORS):
        if health[i] <= 0.0 or limits[i] <= 0.0:
            fixed[i] = True
            failed += 1
    saturation_events = 0
    for _ in range(ACTUATORS):
        rhs_demand = list(demand)
        for i in range(ACTUATORS):
            if fixed[i]:
                for axis in range(DOF):
                    rhs_demand[axis] = rhs_demand[axis] - ALLOCATION[axis][i] * health[i] * commands[i]
        normal = [[0.0] * DOF for _ in range(DOF)]
        for row in range(DOF):
            for col in range(DOF):
                total = 0.0
                for i in range(ACTUATORS):
                    if not fixed[i]:
                        br = ALLOCATION[row][i] * health[i]
                        bc = ALLOCATION[col][i] * health[i]
                        weight = 1.0 / (health[i] * health[i])
                        total = total + br * bc / weight
                normal[row][col] = total
            normal[row][row] = normal[row][row] + REGULARIZATION
        lambdas = solve_3x3(normal, rhs_demand)
        worst = None
        worst_ratio = 1.0
        for i in range(ACTUATORS):
            if fixed[i]:
                continue
            value = 0.0
            for axis in range(DOF):
                value = value + ALLOCATION[axis][i] * health[i] * lambdas[axis]
            weight = 1.0 / (health[i] * health[i])
            commands[i] = value / weight
            ratio = abs(commands[i]) / limits[i]
            if ratio > worst_ratio:
                worst_ratio = ratio
                worst = (i, commands[i])
        if worst is None:
            break
        index, value = worst
        commands[index] = -limits[index] if value < 0.0 else limits[index]
        fixed[index] = True
        saturation_events += 1
    achieved = [0.0] * DOF
    for axis in range(DOF):
        for i in range(ACTUATORS):
            achieved[axis] = achieved[axis] + ALLOCATION[axis][i] * health[i] * commands[i]
    squared = 0.0
    for axis in range(DOF):
        error = demand[axis] - achieved[axis]
        squared = squared + error * error
    return math.sqrt(squared), saturation_events, failed


def maneuver_envelope(health, limits):
    nominal_health, nominal_limits, _ = SCENARIOS["nominal"]
    envelope = float("inf")
    for axis in range(DOF):
        available = 0.0
        nominal = 0.0
        for i in range(ACTUATORS):
            available = available + abs(ALLOCATION[axis][i]) * health[i] * limits[i]
            nominal = nominal + abs(ALLOCATION[axis][i]) * nominal_health[i] * nominal_limits[i]
        if nominal > 0.0:
            ratio = available / nominal
            if ratio < envelope:
                envelope = ratio
    return envelope


def simulate(seed, scenario, steps):
    health, limits, scale = SCENARIOS[scenario]
    rng = random.Random(seed)
    squared_sum = 0.0
    saturation_events = 0
    failed = 0
    for _ in range(steps):
        demand = (
            (rng.random() * 2.0 - 1.0) * scale,
            (rng.random() * 2.0 - 1.0) * scale,
            (rng.random() * 2.0 - 1.0) * scale,
        )
        residual, saturated, failed = allocate(demand, health, limits)
        squared_sum = squared_sum + residual * residual
        saturation_events += saturated
    residual_rms = 0.0 if steps == 0 else math.sqrt(squared_sum / float(steps))
    return residual_rms, saturation_events, maneuver_envelope(health, limits), failed


def rust_scientific(value):
    """Reproduit le format `Rust {:.12e}` (exposant sans signe + ni zéro de tête)."""
    mantissa, exponent = f"{value:.12e}".split("e")
    return f"{mantissa}e{int(exponent)}"


def main():
    if len(sys.argv) < 2:
        print("sim_fault_tolerant_control_alloc.py <seed> [scenario] [steps]", file=sys.stderr)
        return 0
    seed = int(sys.argv[1])
    scenario = sys.argv[2] if len(sys.argv) > 2 else "nominal"
    steps = int(sys.argv[3]) if len(sys.argv) > 3 else 64
    if scenario not in SCENARIOS:
        print(f"scenario inconnu: {scenario}", file=sys.stderr)
        return 2
    residual, saturated, envelope, failed = simulate(seed, scenario, steps)
    print("seed,scenario,steps,residual_rms,saturation_events,saturation_avoided,maneuver_envelope,failed_actuators")
    residual_text = rust_scientific(residual)
    envelope_text = rust_scientific(envelope)
    print(f"{seed},{scenario},{steps},{residual_text},{saturated},{saturated},{envelope_text},{failed}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
