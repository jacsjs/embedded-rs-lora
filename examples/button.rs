#![no_main]
#![no_std]

use panic_halt as _;
#[rtic::app(device = nrf52840_hal::pac, dispatchers = [SWI0_EGU0])]
mod app {
    use cortex_m::asm;
    use embedded_hal::digital::{OutputPin, StatefulOutputPin};
    use nrf52840_hal::{
        gpio::{p0, Level, Output, Pin, PushPull}, 
        gpiote::Gpiote,
        pac::TIMER0,
    };
    use embedded_rs_lora::mono::MonoTimer;

    #[monotonic(binds = TIMER0, default = true)]
    type MyMono = MonoTimer<TIMER0>;
    
    #[shared]
    struct Shared {
        gpiote: Gpiote,
    }

    #[local]
    struct Local {
        btn1_led: Pin<Output<PushPull>>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {

        // Timers
        let mono = MonoTimer::new(cx.device.TIMER0);
        
        // Ownership of Peripherals.
        let p0 = p0::Parts::new(cx.device.P0);
        let gpiote = Gpiote::new(cx.device.GPIOTE);
        
        // LED for status.
        let btn1_led = p0.p0_13.into_push_pull_output(Level::Low).degrade();

        // Input pins for GPIOTE ISR.
        let btn1 = p0.p0_11.into_pullup_input().degrade();

        // Configure GPIOTE.
        gpiote.channel0().input_pin(&btn1).hi_to_lo().enable_interrupt();

        (Shared { gpiote }, Local { btn1_led }, init::Monotonics(mono))
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            asm::wfi();
        }
    }

    #[task(binds = GPIOTE, local=[btn1_led], shared = [gpiote])]
    fn gpiote_cb(mut ctx: gpiote_cb::Context){
        ctx.shared.gpiote.lock(|gpiote| {
            if gpiote.channel0().is_event_triggered() {
                let b_led = ctx.local.btn1_led;
                if b_led.is_set_low().unwrap() {
                    b_led.set_high().ok();
                } else {
                    b_led.set_low().ok();
                }
            } else if gpiote.channel1().is_event_triggered() {
            } 
            gpiote.reset_events();
        });
    }
}
