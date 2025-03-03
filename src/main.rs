#![no_main]
#![no_std]

use panic_halt as _;
#[rtic::app(device = nrf52840_hal::pac, dispatchers = [SWI0_EGU0])]
mod app {
    use embedded_rs_lora::{at_command_handler::AtCommandHandler, mono::{ExtU32, MonoTimer}};
    use embedded_hal::digital::{OutputPin, StatefulOutputPin};
    use cortex_m::asm;
    use nrf52840_hal::{
        gpio::{p0, Disconnected, Level, Output, Pin, PushPull}, 
        pac::{TIMER0, TIMER1, UARTE0}, 
        saadc::SaadcConfig, 
        uarte::{Baudrate, Parity, Pins}, 
        Saadc, Timer, Uarte
    };

    // Buffer size for AT commands.
    const COM_BUF_SIZE: usize = 64;

    // How many millisecons between sensor poll and send.
    const TMP_SENSOR_POLL_RATE: u32 = 1_000;

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
        status_led: Pin<Output<PushPull>>,
        saadc: Saadc,
        saadc_pins: SaadcPins,
        at: AtCommandHandler<UARTE0, TIMER0>,
        com_buf: [u8; COM_BUF_SIZE],
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {

        let mut mono = MonoTimer::new(cx.device.TIMER1);

        let p0 = p0::Parts::new(cx.device.P0);
        let saadc = Saadc::new(cx.device.SAADC, SaadcConfig::default());

        // Update struct based on desired analog pins here:
        let saadc_pins = SaadcPins { 
            pin3: p0.p0_03 
        };

        // Set RX and TX pins for UARTE0 peripheral. 
        let tx_pin = p0.p0_04.into_push_pull_output(Level::High).degrade();
        let rx_pin = p0.p0_31.into_floating_input().degrade();

        let status_led = p0.p0_13.into_push_pull_output(Level::Low).degrade();

        let uarte0 = Uarte::new(
            cx.device.UARTE0, 
            Pins{rxd: rx_pin, txd: tx_pin, cts: None, rts: None}, 
            Parity::EXCLUDED, 
            Baudrate::BAUD9600);

        let timer0 = Timer::new(cx.device.TIMER0);
        
        let at = AtCommandHandler::new(uarte0, timer0, 1);

        // Initiate periodic processes.
        read_analog::spawn(mono.now()).unwrap();

        (Shared {}, Local { status_led, saadc, saadc_pins, at, com_buf: [0; COM_BUF_SIZE] }, init::Monotonics(mono))
    }

    #[idle(local = [status_led])]
    fn idle(cx: idle::Context) -> ! {
        loop {
            // To know that the system is running and sleeping correctly.
            if cx.local.status_led.is_set_low().unwrap() {
                cx.local.status_led.set_high().ok();
            } else {
                cx.local.status_led.set_low().ok();
            }
            asm::wfi();
        }
    }

    #[task(local = [saadc, saadc_pins, at, com_buf])]
    fn read_analog(ctx: read_analog::Context, instant: fugit::TimerInstantU32<1_000_000>){

        let pins = ctx.local.saadc_pins;

        // Result is collected for 20 µs.
        let result = ctx.local.saadc.read_channel(&mut pins.pin3).unwrap();

        // Send result as AT command.
        match ctx.local.at.set_no_response(ctx.local.com_buf,  
            |builder| builder.named("+TMPC_S").with_int_parameter(result).finish())
            {
                Ok(_) => (),
                Err(_) => ()
            };

        let next_instant = instant + TMP_SENSOR_POLL_RATE.millis();
        read_analog::spawn_at(next_instant, next_instant).unwrap();
    }
}
