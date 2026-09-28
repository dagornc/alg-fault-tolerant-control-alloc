//! Portage Rust de `ALG_FAULT_TOLERANT_CONTROL_ALLOC`.

pub mod rng;
pub mod sim;

pub use sim::{AllocationResult, Scenario, SimulationResult, allocate, simulate};
