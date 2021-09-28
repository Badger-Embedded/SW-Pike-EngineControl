#![no_std]

use state_governor::create_states;
pub mod can_driver;

pub mod libs;
pub use libs::*;

// https://github.com/Badger-Embedded/Badger-Pike#engine-control
create_states!(IDLE, READY, IGNITION, PROPULSION, BURNOUT);
