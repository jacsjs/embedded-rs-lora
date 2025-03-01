#![no_main]
#![no_std]

use panic_halt as _;
#[rtic::app(device = nrf52840_hal::pac, dispatchers = [SWI0_EGU0])]
mod app {
    use rtt_target::{rprintln, rtt_init_print};
    use cortex_m::asm;
    use embedded_hal::digital::{OutputPin, StatefulOutputPin};
    use nrf52840_hal::{
        gpio::{p0, Level, Output, Pin, PushPull}, pac::{TIMER0, TIMER1, UARTE0}, uarte::{Baudrate, Parity, Pins}, Timer, Uarte
    };
    use embedded_rs_lora::{at_command_handler::AtCommandHandler, mono::{ExtU32, MonoTimer}};

    #[monotonic(binds = TIMER1, default = true)]
    type MyMono = MonoTimer<TIMER1>;

    #[shared]
    struct Shared {
    }

    #[local]
    struct Local {
        blink_led: Pin<Output<PushPull>>,
        at: AtCommandHandler<UARTE0, TIMER0>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        rtt_init_print!();

        let mut mono = MonoTimer::new(cx.device.TIMER1);

        let p0 = p0::Parts::new(cx.device.P0);
        
        let blink_led = p0.p0_13.into_push_pull_output(Level::High).degrade();

        // Set RX and TX pins for UARTE0 perhebial. 
        let tx_pin = p0.p0_04.into_push_pull_output(Level::High).degrade();
        let rx_pin = p0.p0_31.into_floating_input().degrade();

        let uarte0 = Uarte::new(
            cx.device.UARTE0, 
            Pins{rxd: rx_pin, txd: tx_pin, cts: None, rts: None}, 
            Parity::EXCLUDED, 
            Baudrate::BAUD9600);

        let timer0 = Timer::new(cx.device.TIMER0);
        
        let at = AtCommandHandler::new(uarte0, timer0, 1_000_000);

        // Initiate periodic process
        blink::spawn_after(1.secs(), mono.now()).unwrap();
        (Shared {  }, Local { blink_led, at }, init::Monotonics(mono))
    }

    #[idle(local = [at])]
    fn idle(cx: idle::Context) -> ! {
        let mut res_buf = [0; 128];
        loop {
            let (x,y) = match cx.local.at.send_expect_response(&mut res_buf,
                |builder| builder.named("+SET").with_int_parameter(42).with_int_parameter(31).finish(), 
                |parser| parser
                    .expect_identifier(b"AT+SET=")
                    .expect_int_parameter()
                    .expect_int_parameter()
                    .expect_identifier(b"\r\n")
                    .finish()) {
                        Ok(tu) => tu,
                        Err(at_err) => {
                            rprintln!("Error: {:?}", at_err);
                            (0,0)
                        } 
                    };
            // If no crash, then it's working as expected
            assert_eq!((x, y), (42, 31));
            rprintln!("sleeping...");
        }
    }

    #[task(local = [blink_led])]
    fn blink(ctx: blink::Context, instant: fugit::TimerInstantU32<1_000_000>) {
        let led = ctx.local.blink_led;
        if led.is_set_low().unwrap() {
            led.set_high().ok();
        } else {
            led.set_low().ok();
        }
        let next_instant = instant + 1000.millis();
        blink::spawn_at(next_instant, next_instant).unwrap();
    }
}
