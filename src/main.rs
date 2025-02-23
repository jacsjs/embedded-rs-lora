#![no_main]
#![no_std]
use panic_halt as _;
mod mono;
#[rtic::app(device = nrf52840_hal::pac, dispatchers = [SWI0_EGU0])]
mod app {
    use cortex_m::asm;
    use embedded_hal::digital::{OutputPin, StatefulOutputPin};
    use nrf52840_hal::{
        gpio::{p0::Parts, Level, Output, Pin, PushPull}, 
        gpiote::Gpiote, 
        pac::{ TIMER0, UARTE0, UARTE1},
    };
    use rtt_target::{rprintln, rtt_init_print};

    use crate::mono::{ExtU32, MonoTimer};

    #[monotonic(binds = TIMER0, default = true)]
    type MyMono = MonoTimer<TIMER0>;

    #[shared]
    struct Shared {
        gpiote: Gpiote,
    }

    #[local]
    struct Local {
        led: Pin<Output<PushPull>>,
        uarte0: UARTE0,
        rx_buffer0: [u8; 1],
        uarte1: UARTE1,
        rx_buffer1: [u8; 1],
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        rtt_init_print!();
        let rx_buffer0: [u8; 1] = [0; 1];
        let rx_buffer1: [u8; 1] = [0; 1];

        let mut mono = MonoTimer::new(cx.device.TIMER0);

        let p0 = Parts::new(cx.device.P0);
        let led = p0.p0_13.into_push_pull_output(Level::High).degrade();

        // Get UARTE raw pac device.
        let uarte0 = cx.device.UARTE0;
        let uarte1 = cx.device.UARTE1;

        // Set baud rate
        uarte0.baudrate.write(|w| w.baudrate().baud9600());

        // Configure the UARTE
        // Set Parity to EXCLUDED, Flow control disabled
        uarte0.config.write(|w| {
            w.parity().excluded(); // No parity
            w.hwfc().disabled();   // Disable hardware flow control
            // Other configurations can be set here
            w
        });

        // Set RX and TX pins for UARTE0 perhebial.
        let tx_pin = p0.p0_23.into_push_pull_output(Level::High).degrade();
        let rx_pin = p0.p0_24.into_floating_input().degrade();
        uarte0.psel.rxd.write(|w| unsafe { w.bits(rx_pin.psel_bits()) });
        uarte0.psel.txd.write(|w| unsafe { w.bits(tx_pin.psel_bits()) });

        // Enable UARTE
        uarte0.enable.write(|w| w.enable().enabled());

        // Enable ENDRX interrupt. ENDRX event is invoked when bytes flushed to buffer is full.
        uarte0.intenset.write(|w| w.endrx().set_bit());

        // Setup complete; ready to start reception.
        uarte0.tasks_startrx.write(|w| unsafe { w.bits(1) });


        uarte1.baudrate.write(|w| w.baudrate().baud9600());
        uarte1.config.write(|w| {
            w.parity().excluded(); // No parity
            w.hwfc().disabled();   // Disable hardware flow control
            w
        });
        let tx_pin1 = p0.p0_19.into_push_pull_output(Level::High).degrade();
        let rx_pin1 = p0.p0_20.into_floating_input().degrade();
        uarte1.psel.rxd.write(|w| unsafe { w.bits(rx_pin1.psel_bits()) });
        uarte1.psel.txd.write(|w| unsafe { w.bits(tx_pin1.psel_bits()) });
        uarte1.enable.write(|w| w.enable().enabled());
        uarte1.intenset.write(|w| w.endrx().set_bit());
        uarte1.tasks_startrx.write(|w| unsafe { w.bits(1) });


        let gpiote = Gpiote::new(cx.device.GPIOTE);

        let button = p0.p0_11.into_pullup_input().degrade();

        // Configure GPIOTE channel 0 for the button
        gpiote.channel0().input_pin(&button).hi_to_lo().enable_interrupt();

        // Initiate periodic process
        blink::spawn_after(1.secs(), mono.now()).unwrap();
        (Shared { gpiote }, Local { led, uarte0, rx_buffer0, uarte1, rx_buffer1 }, init::Monotonics(mono))
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            rprintln!("sleeping...");
            asm::wfi();
        }
    }

    #[task(local = [led])]
    fn blink(ctx: blink::Context, instant: fugit::TimerInstantU32<1_000_000>) {
        rprintln!("Blink!");
        let led = ctx.local.led;
        if led.is_set_low().unwrap() {
            led.set_high().ok();
        } else {
            led.set_low().ok();
        }
        let next_instant = instant + 1000.millis();
        blink::spawn_at(next_instant, next_instant).unwrap();
    }

    #[task(binds = GPIOTE, shared = [gpiote])]
    fn button_pressed(mut ctx: button_pressed::Context){
        ctx.shared.gpiote.lock(|gpiote| {
            if gpiote.channel0().is_event_triggered() {
                rprintln!("Interrupt from channel 0 event");
            }
            // Reset all events
            gpiote.reset_events();
            // Debounce
            //debounce::spawn_after(50.millis()).ok();
        });
    }

    #[task(binds = UARTE0_UART0, local = [uarte0, rx_buffer0])]
    fn uart0_rx(ctx: uart0_rx::Context){
        // Stop reception.
        ctx.local.uarte0.tasks_stoprx.write(|w| unsafe { w.bits(1) });

        // Wait for the reception to have stopped.
        while ctx.local.uarte0.events_rxto.read().bits() == 0 {}

        // Set Easy DMA buffer addresses and size for transfer from rxd register.
        ctx.local.uarte0.rxd.maxcnt.write(|w| unsafe { w.bits(1) });
        ctx.local.uarte0.rxd.ptr.write(|w| unsafe { w.ptr().bits(ctx.local.rx_buffer0.as_ptr() as u32) });

        // Debugging info
        let amout = ctx.local.uarte0.rxd.amount.read().bits();
        rprintln!("Amount0: {}", amout);
        for &mut byte in ctx.local.rx_buffer0{
            rprintln!("data0: {}", byte);
        }
        // Reset the event.
        ctx.local.uarte0.events_endrx.write(|w| w);

        // Now that this byte has been handled, start receiver in wait for another byte.
        ctx.local.uarte0.tasks_startrx.write(|w| unsafe { w.bits(1) });
    }

    #[task(binds = UARTE1, local = [uarte1, rx_buffer1])]
    fn uart1_rx(ctx: uart1_rx::Context){
        // Stop reception.
        ctx.local.uarte1.tasks_stoprx.write(|w| unsafe { w.bits(1) });

        // Wait for the reception to have stopped.
        while ctx.local.uarte1.events_rxto.read().bits() == 0 {}

        // Set Easy DMA buffer addresses and size for transfer from rxd register.
        ctx.local.uarte1.rxd.maxcnt.write(|w| unsafe { w.bits(1) });
        ctx.local.uarte1.rxd.ptr.write(|w| unsafe { w.ptr().bits(ctx.local.rx_buffer1.as_ptr() as u32) });

        // Debugging info
        let amout = ctx.local.uarte1.rxd.amount.read().bits();
        rprintln!("Amount1: {}", amout);
        for &mut byte in ctx.local.rx_buffer1{
            rprintln!("data1: {}", byte);
        }
        // Reset the event.
        ctx.local.uarte1.events_endrx.write(|w| w);

        // Now that this byte has been handled, start receiver in wait for another byte.
        ctx.local.uarte1.tasks_startrx.write(|w| unsafe { w.bits(1) });
    }
}
