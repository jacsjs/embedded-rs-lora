#![no_main]
#![no_std]

use panic_halt as _;
#[rtic::app(device = nrf52840_hal::pac, dispatchers = [SWI0_EGU0])]
mod app {
    use cortex_m::asm;
    use embedded_rs_lora::mono::{ExtU32, MonoTimer};
    use nrf52840_hal::{
        gpio::{p0, Disconnected},
        pac::TIMER1,
        saadc::SaadcConfig,
        Saadc,
    };
    use rtt_target::{rprintln, rtt_init_print};

    // Holds all analog pins in use.
    struct SaadcPins {
        pin3: p0::P0_03<Disconnected>,
    }
    #[monotonic(binds = TIMER1, default = true)]
    type MyMono = MonoTimer<TIMER1>;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        saadc: Saadc,
        saadc_pins: SaadcPins,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        rtt_init_print!();

        let mut mono = MonoTimer::new(cx.device.TIMER1);
        let p0 = p0::Parts::new(cx.device.P0);

        let saadc = Saadc::new(cx.device.SAADC, SaadcConfig::default());

        // Update struct based on desired analog pins here:
        let saadc_pins = SaadcPins { pin3: p0.p0_03 };

        // Initiate periodic processes.
        read_analog::spawn(mono.now()).unwrap();

        (
            Shared {},
            Local { saadc, saadc_pins },
            init::Monotonics(mono),
        )
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            rprintln!("Sleeping...");
            asm::wfi();
        }
    }

    #[task(local = [saadc, saadc_pins])]
    fn read_analog(ctx: read_analog::Context, instant: fugit::TimerInstantU32<1_000_000>) {
        let pins = ctx.local.saadc_pins;

        // Result is collected for 20 µs.
        let result = ctx.local.saadc.read_channel(&mut pins.pin3).unwrap();

        // Assuming VDD = 3.3V.
        const VREF: f32 = 3.3;
        const RESOLUTION: f32 = 16_384.0;

        // Calculate voltage at the ADC pin.
        let voltage = (result as f32 / RESOLUTION) * VREF;

        rprintln!("Measured voltage: {}", voltage);

        // Poll once every second.
        let next_instant = instant + 1.secs();
        read_analog::spawn_at(next_instant, next_instant).unwrap();
    }
}
