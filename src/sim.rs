//! Allocation pondérée 3-DOF vers quatre actionneurs avec projection active des saturations.

// Les accumulateurs et boucles indicées reproduisent volontairement l'ordre des
// opérations de la référence Python (miroir bit-à-bit) ; les lints stylistiques
// `assign_op_pattern` et `needless_range_loop` sont donc désactivés ici.
#![allow(clippy::assign_op_pattern, clippy::needless_range_loop)]

use crate::rng::PyRandom;

pub const ACTUATORS: usize = 4;
pub const DOF: usize = 3;
const REGULARIZATION: f64 = 1.0e-9;

/// Colonnes de la matrice d'allocation : port, tribord, étrave latérale, poupe latérale.
const ALLOCATION: [[f64; ACTUATORS]; DOF] = [
    [1.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 1.0],
    [-1.0, 1.0, 1.0, -1.0],
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Scenario {
    Nominal,
    PortFailure,
    BowDegraded,
    DualFault,
    Saturation,
}

impl Scenario {
    pub const ALL: [Self; 5] = [
        Self::Nominal,
        Self::PortFailure,
        Self::BowDegraded,
        Self::DualFault,
        Self::Saturation,
    ];

    pub fn parse(value: &str) -> Option<Self> {
        match value {
            "nominal" => Some(Self::Nominal),
            "port_failure" => Some(Self::PortFailure),
            "bow_degraded" => Some(Self::BowDegraded),
            "dual_fault" => Some(Self::DualFault),
            "saturation" => Some(Self::Saturation),
            _ => None,
        }
    }

    pub const fn name(self) -> &'static str {
        match self {
            Self::Nominal => "nominal",
            Self::PortFailure => "port_failure",
            Self::BowDegraded => "bow_degraded",
            Self::DualFault => "dual_fault",
            Self::Saturation => "saturation",
        }
    }

    pub const fn health(self) -> [f64; ACTUATORS] {
        match self {
            Self::Nominal | Self::Saturation => [1.0, 1.0, 1.0, 1.0],
            Self::PortFailure => [0.0, 1.0, 1.0, 1.0],
            Self::BowDegraded => [1.0, 1.0, 0.35, 1.0],
            Self::DualFault => [0.0, 1.0, 0.0, 1.0],
        }
    }

    pub const fn limits(self) -> [f64; ACTUATORS] {
        match self {
            Self::Saturation => [0.35, 0.35, 0.35, 0.35],
            _ => [1.0, 1.0, 0.8, 0.8],
        }
    }

    const fn demand_scale(self) -> f64 {
        match self {
            Self::Saturation => 2.5,
            _ => 0.8,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct AllocationResult {
    pub commands: [f64; ACTUATORS],
    pub achieved: [f64; DOF],
    pub residual_norm: f64,
    pub saturation_events: usize,
    pub failed_actuators: usize,
}

#[derive(Clone, Debug, PartialEq)]
pub struct SimulationResult {
    pub seed: u64,
    pub scenario: Scenario,
    pub steps: usize,
    pub residual_rms: f64,
    pub saturation_events: usize,
    pub saturation_avoided: usize,
    pub maneuver_envelope: f64,
    pub failed_actuators: usize,
}

/// Résout un système 3×3 par élimination de Gauss avec pivot partiel déterministe.
fn solve_3x3(mut a: [[f64; DOF]; DOF], mut b: [f64; DOF]) -> [f64; DOF] {
    for col in 0..DOF {
        let mut pivot = col;
        for row in (col + 1)..DOF {
            if a[row][col].abs() > a[pivot][col].abs() {
                pivot = row;
            }
        }
        if pivot != col {
            a.swap(col, pivot);
            b.swap(col, pivot);
        }
        if a[col][col].abs() < 1.0e-15 {
            continue;
        }
        for row in (col + 1)..DOF {
            let factor = a[row][col] / a[col][col];
            for k in col..DOF {
                a[row][k] = a[row][k] - factor * a[col][k];
            }
            b[row] = b[row] - factor * b[col];
        }
    }
    let mut x = [0.0; DOF];
    for row in (0..DOF).rev() {
        let mut rhs = b[row];
        for k in (row + 1)..DOF {
            rhs = rhs - a[row][k] * x[k];
        }
        if a[row][row].abs() >= 1.0e-15 {
            x[row] = rhs / a[row][row];
        }
    }
    x
}

/// Alloue une consigne `[surge, sway, yaw]` en tenant compte de la santé et des limites.
pub fn allocate(
    demand: [f64; DOF],
    health: [f64; ACTUATORS],
    limits: [f64; ACTUATORS],
) -> AllocationResult {
    let mut commands = [0.0; ACTUATORS];
    let mut fixed = [false; ACTUATORS];
    let mut failed = 0usize;
    for i in 0..ACTUATORS {
        if health[i] <= 0.0 || limits[i] <= 0.0 {
            fixed[i] = true;
            failed += 1;
        }
    }

    let mut saturation_events = 0usize;
    for _ in 0..ACTUATORS {
        let mut rhs_demand = demand;
        for i in 0..ACTUATORS {
            if fixed[i] {
                for axis in 0..DOF {
                    rhs_demand[axis] =
                        rhs_demand[axis] - ALLOCATION[axis][i] * health[i] * commands[i];
                }
            }
        }

        let mut normal = [[0.0; DOF]; DOF];
        for row in 0..DOF {
            for col in 0..DOF {
                let mut sum = 0.0;
                for i in 0..ACTUATORS {
                    if !fixed[i] {
                        let br = ALLOCATION[row][i] * health[i];
                        let bc = ALLOCATION[col][i] * health[i];
                        let weight = 1.0 / (health[i] * health[i]);
                        sum = sum + br * bc / weight;
                    }
                }
                normal[row][col] = sum;
            }
            normal[row][row] = normal[row][row] + REGULARIZATION;
        }
        let lambda = solve_3x3(normal, rhs_demand);

        let mut worst: Option<(usize, f64)> = None;
        let mut worst_ratio = 1.0;
        for i in 0..ACTUATORS {
            if fixed[i] {
                continue;
            }
            let mut value = 0.0;
            for axis in 0..DOF {
                value = value + ALLOCATION[axis][i] * health[i] * lambda[axis];
            }
            // Une santé dégradée augmente le coût de l'actionneur sans changer l'ordre des opérations.
            let weight = 1.0 / (health[i] * health[i]);
            commands[i] = value / weight;
            let ratio = commands[i].abs() / limits[i];
            if ratio > worst_ratio {
                worst_ratio = ratio;
                worst = Some((i, commands[i]));
            }
        }
        if let Some((index, value)) = worst {
            commands[index] = if value < 0.0 {
                -limits[index]
            } else {
                limits[index]
            };
            fixed[index] = true;
            saturation_events += 1;
        } else {
            break;
        }
    }

    let mut achieved = [0.0; DOF];
    for axis in 0..DOF {
        for i in 0..ACTUATORS {
            achieved[axis] = achieved[axis] + ALLOCATION[axis][i] * health[i] * commands[i];
        }
    }
    let mut squared = 0.0;
    for axis in 0..DOF {
        let error = demand[axis] - achieved[axis];
        squared = squared + error * error;
    }

    AllocationResult {
        commands,
        achieved,
        residual_norm: squared.sqrt(),
        saturation_events,
        failed_actuators: failed,
    }
}

pub fn maneuver_envelope(health: [f64; ACTUATORS], limits: [f64; ACTUATORS]) -> f64 {
    let nominal_health = Scenario::Nominal.health();
    let nominal_limits = Scenario::Nominal.limits();
    let mut envelope = f64::INFINITY;
    for axis in 0..DOF {
        let mut available = 0.0;
        let mut nominal = 0.0;
        for i in 0..ACTUATORS {
            available = available + ALLOCATION[axis][i].abs() * health[i] * limits[i];
            nominal = nominal + ALLOCATION[axis][i].abs() * nominal_health[i] * nominal_limits[i];
        }
        if nominal > 0.0 {
            let ratio = available / nominal;
            if ratio < envelope {
                envelope = ratio;
            }
        }
    }
    envelope
}

pub fn simulate(seed: u64, scenario: Scenario, steps: usize) -> SimulationResult {
    let mut rng = PyRandom::new(seed);
    let health = scenario.health();
    let limits = scenario.limits();
    let mut squared_sum = 0.0;
    let mut saturation_events = 0usize;
    let mut failed_actuators = 0usize;
    for _ in 0..steps {
        let scale = scenario.demand_scale();
        let demand = [
            (rng.random() * 2.0 - 1.0) * scale,
            (rng.random() * 2.0 - 1.0) * scale,
            (rng.random() * 2.0 - 1.0) * scale,
        ];
        let result = allocate(demand, health, limits);
        squared_sum = squared_sum + result.residual_norm * result.residual_norm;
        saturation_events += result.saturation_events;
        failed_actuators = result.failed_actuators;
    }
    let residual_rms = if steps == 0 {
        0.0
    } else {
        (squared_sum / steps as f64).sqrt()
    };
    SimulationResult {
        seed,
        scenario,
        steps,
        residual_rms,
        saturation_events,
        saturation_avoided: saturation_events,
        maneuver_envelope: maneuver_envelope(health, limits),
        failed_actuators,
    }
}
