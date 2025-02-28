#![no_main]
#![no_std]

use panic_halt as _;
#[rtic::app(device = nrf52840_hal::pac)]
mod app {
    use rtt_target::{rprintln, rtt_init_print};
    use cortex_m::asm;
    use nrf52840_hal::pac::TIMER0;
    use embedded_rs_lora::mono::MonoTimer;

    #[monotonic(binds = TIMER0, default = true)]
    type MyMono = MonoTimer<TIMER0>;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        rtt_init_print!();

        // Timers
        let mono = MonoTimer::new(cx.device.TIMER0);

        (Shared { }, Local { }, init::Monotonics(mono))
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            rprintln!("sleeping...");
            asm::wfi();
        }
    }
}
