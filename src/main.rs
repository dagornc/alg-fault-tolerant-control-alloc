use std::env;
use std::process::ExitCode;

use fault_tolerant_control_alloc_rs::{Scenario, simulate};

fn main() -> ExitCode {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.is_empty() || args[0] == "-h" || args[0] == "--aide" {
        eprintln!("fault_tolerant_control_alloc_rs <seed> [scenario] [steps]");
        eprintln!("scenarios: nominal port_failure bow_degraded dual_fault saturation");
        return ExitCode::SUCCESS;
    }
    let seed = match args[0].parse::<u64>() {
        Ok(value) => value,
        Err(_) => {
            eprintln!("graine invalide: {}", args[0]);
            return ExitCode::from(2);
        }
    };
    let scenario_name = args.get(1).map(String::as_str).unwrap_or("nominal");
    let Some(scenario) = Scenario::parse(scenario_name) else {
        eprintln!("scenario inconnu: {scenario_name}");
        return ExitCode::from(2);
    };
    let steps = args
        .get(2)
        .and_then(|value| value.parse().ok())
        .unwrap_or(64);
    let r = simulate(seed, scenario, steps);
    println!(
        "seed,scenario,steps,residual_rms,saturation_events,saturation_avoided,maneuver_envelope,failed_actuators"
    );
    println!(
        "{},{},{},{:.12e},{},{},{:.12e},{}",
        r.seed,
        r.scenario.name(),
        r.steps,
        r.residual_rms,
        r.saturation_events,
        r.saturation_avoided,
        r.maneuver_envelope,
        r.failed_actuators
    );
    ExitCode::SUCCESS
}
