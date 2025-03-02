#![no_std]
#![no_main]

#[cfg(test)]
use defmt_rtt as _; // global logger

// TODO(5) adjust HAL import
// use some_hal as _; // memory layout
use nrf52840_hal as _; // memory layout

use panic_halt as _;

// See https://crates.io/crates/defmt-test/0.3.0 for more documentation (e.g. about the 'state'
// feature)
#[defmt_test::tests]
mod tests {
    use defmt::assert;

    #[test]
    fn it_works() {
        assert!(true)
    }
}
