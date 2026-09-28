//! Spécifications exécutables de l'allocation de commande tolérante aux fautes.

use fault_tolerant_control_alloc_rs::sim::{Scenario, allocate, simulate};

#[test]
fn nominal_tracks_feasible_wrench() {
    let r = allocate(
        [0.8, 0.4, 0.1],
        Scenario::Nominal.health(),
        Scenario::Nominal.limits(),
    );
    assert!(r.residual_norm < 1.0e-6, "résidu={}", r.residual_norm);
}

#[test]
fn failed_actuator_receives_zero_command() {
    let r = allocate(
        [0.7, 0.2, 0.2],
        Scenario::PortFailure.health(),
        Scenario::PortFailure.limits(),
    );
    assert_eq!(r.commands[0], 0.0);
    assert_eq!(r.failed_actuators, 1);
}

#[test]
fn commands_never_exceed_limits() {
    for scenario in Scenario::ALL {
        let r = allocate([3.0, -2.0, 1.5], scenario.health(), scenario.limits());
        for (u, limit) in r.commands.iter().zip(scenario.limits()) {
            assert!(
                u.abs() <= limit + f64::EPSILON,
                "{scenario:?}: {u} > {limit}"
            );
        }
    }
}

#[test]
fn degraded_weighted_solution_still_tracks_feasible_wrench() {
    let r = allocate(
        [0.4, 0.3, 0.05],
        Scenario::BowDegraded.health(),
        Scenario::BowDegraded.limits(),
    );
    assert!(r.residual_norm < 1.0e-6, "résidu={}", r.residual_norm);
}

#[test]
fn healthy_redundancy_beats_uncompensated_failure() {
    let demand = [0.8, 0.3, 0.2];
    let nominal = allocate(
        demand,
        Scenario::Nominal.health(),
        Scenario::Nominal.limits(),
    );
    let failed = allocate(
        demand,
        Scenario::DualFault.health(),
        Scenario::DualFault.limits(),
    );
    assert!(nominal.residual_norm < failed.residual_norm);
}

#[test]
fn simulation_is_deterministic() {
    for scenario in Scenario::ALL {
        assert_eq!(simulate(1001, scenario, 64), simulate(1001, scenario, 64));
    }
}

#[test]
fn maneuver_envelope_degrades_after_fault() {
    let nominal = simulate(1001, Scenario::Nominal, 64);
    let failed = simulate(1001, Scenario::DualFault, 64);
    assert!(nominal.maneuver_envelope > failed.maneuver_envelope);
}

#[test]
fn saturation_projection_is_exercised() {
    let result = simulate(1001, Scenario::Saturation, 64);
    assert!(result.saturation_events > 0);
    assert!(result.saturation_avoided > 0);
}
