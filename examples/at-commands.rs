#![no_main]
#![no_std]

use panic_halt as _;
#[rtic::app(device = nrf52840_hal::pac, dispatchers = [SWI0_EGU0])]
mod app {
    use rtt_target::{rprintln, rtt_init_print};
    use cortex_m::asm;
    use embedded_hal::digital::{OutputPin, StatefulOutputPin};
    use nrf52840_hal::{
        gpio::{p0, Level, Output, Pin, PushPull}, 
        uarte::{Baudrate, Parity, Pins},
        pac::TIMER1, 
        Timer, 
        Uarte
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
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        rtt_init_print!();

        let mut mono = MonoTimer::new(cx.device.TIMER1);

        let p0 = p0::Parts::new(cx.device.P0);
        
        let blink_led = p0.p0_13.into_push_pull_output(Level::High).degrade();

        // Set RX and TX pins for UARTE0 perhebial. 
        let tx_pin = p0.p0_04.into_push_pull_output(Level::High).degrade();
        let rx_pin = p0.p0_24.into_floating_input().degrade();

        let uarte0 = Uarte::new(
            cx.device.UARTE0, 
            Pins{rxd: rx_pin, txd: tx_pin, cts: None, rts: None}, 
            Parity::EXCLUDED, 
            Baudrate::BAUD9600);

        let mut res_buf = [0; 128];
        let timer0 = Timer::new(cx.device.TIMER0);
        
        let mut at = AtCommandHandler::new(uarte0, timer0, 1_000_000);
        let (x,) = at.send_expect_response(&mut res_buf,
            |builder| builder.named("+TESTQUERY").finish(), 
            |parser| parser
                .expect_identifier(b"AT+RESPONSE=")
                .expect_int_parameter()
                .expect_identifier(b"\r\n")
                .finish()
            ).unwrap_or((0,));
        
        rprintln!("VALUE: {}", x);

        // Initiate periodic process
        blink::spawn_after(1.secs(), mono.now()).unwrap();
        (Shared {  }, Local { blink_led }, init::Monotonics(mono))
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            rprintln!("sleeping...");
            asm::wfi();
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
