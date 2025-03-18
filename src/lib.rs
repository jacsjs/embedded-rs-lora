#![no_std]
#![no_main]
pub mod at_command_handler;
pub mod data_structures;
pub mod message_handler;
pub mod util;

#[cfg(test)]
use defmt_rtt as _; // global logger

// use some_hal as _; // memory layout
use nrf52840_hal as _; // memory layout

use panic_rtt_target as _;

// defmt-test 0.3.0 has the limitation that this `#[tests]` attribute can only be used
// once within a crate. the module can be in any file but there can only be at most
// one `#[tests]` module in this library crate
#[cfg(test)]
#[defmt_test::tests]
mod unit_tests {
    use defmt::assert;

    #[test]
    fn it_works() {
        assert!(true)
    }
}
