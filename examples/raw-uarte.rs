#![no_main]
#![no_std]

#[rtic::app(device = nrf52840_hal::pac, dispatchers = [SWI0_EGU0])]
mod app {
    use cortex_m::asm;
    use embedded_hal::digital::{OutputPin, StatefulOutputPin};
    use nrf52840_hal::{
        gpio::{p0, p1, Level, Output, Pin, PushPull}, 
        gpiote::Gpiote, 
        pac::{UARTE1, TIMER0},
    };
    use panic_halt as _;
    use rtt_target::{rprintln, rtt_init_print};
    use embedded_rs_lora::mono::{ExtU32, MonoTimer};

    #[monotonic(binds = TIMER0, default = true)]
    type MyMono = MonoTimer<TIMER0>;

    #[shared]
    struct Shared {
        gpiote: Gpiote, 
        uarte1: UARTE1,
    }

    #[local]
    struct Local {
        blink_led: Pin<Output<PushPull>>,
        button_led: Pin<Output<PushPull>>,
        rx_led1: Pin<Output<PushPull>>,
        rx_buf1: [u8; 1],
        tx_buf1: [u8; 14],
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        rtt_init_print!();

        // Timers
        let mut mono = MonoTimer::new(cx.device.TIMER0);
        
        // Ownership of Peripherals.
        let p0 = p0::Parts::new(cx.device.P0);
        let p1 = p1::Parts::new(cx.device.P1);
        let gpiote = Gpiote::new(cx.device.GPIOTE); 
        let uarte1 = cx.device.UARTE1;
        
        // LED for status.
        let blink_led = p0.p0_13.into_push_pull_output(Level::High).degrade();
        let button_led = p0.p0_14.into_push_pull_output(Level::Low).degrade();
        let rx_led1 = p0.p0_15.into_push_pull_output(Level::Low).degrade();

        // Buttons for GPIOTE ISR.
        let button1 = p0.p0_11.into_pullup_input().degrade();
        let button2 = p0.p0_12.into_pullup_input().degrade();

        // For u trasmissions.
        let sen_tx = p1.p1_15.into_push_pull_output(Level::High).degrade();
        let sen_rx = p1.p1_14.into_floating_input().degrade();

        // Configure button interrupt.
        gpiote.channel0().input_pin(&button1).hi_to_lo().enable_interrupt();
        gpiote.channel1().input_pin(&button2).hi_to_lo().enable_interrupt();

        // Configure the u.
        uarte1.config.write(|w| {
            w.parity().excluded(); // No parity
            w.hwfc().disabled();   // Disable hardware flow control
            // Other configurations can be set here
            w
        });

        // Set baud rate
        uarte1.baudrate.write(|w| w.baudrate().baud9600());

        // Set RX and TX pins for uarte1 perhebial. 
        uarte1.psel.rxd.write(|w| unsafe { w.bits(sen_rx.psel_bits()) });
        uarte1.psel.txd.write(|w| unsafe { w.bits(sen_tx.psel_bits()) });

        // Enable u
        uarte1.enable.write(|w| w.enable().enabled());

        // Enable ENDRX interrupt. ENDRX event is invoked when bytes flushed to buffer is full.
        uarte1.intenset.write(|w| w.endrx().set_bit());

        // Setup complete; ready to start reception.
        uarte1.tasks_startrx.write(|w| unsafe { w.bits(1) });

        let rx_buf1 = [0; 1];

        // "Hello Arduino\n"
        let tx_buf1 = [72, 101, 108, 108, 111, 32, 65, 114, 100, 117, 105, 110, 111, b'\n'];

        // Initiate periodic processes
        blink::spawn_after(1.secs(), mono.now()).unwrap();

        (Shared { gpiote, uarte1 }, Local { blink_led, button_led, rx_led1, rx_buf1, tx_buf1}, init::Monotonics(mono))
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            rprintln!("Sleeping...");
            asm::wfi();
        }
    }

    #[task(local = [blink_led])]
    fn blink(cx: blink::Context, instant: fugit::TimerInstantU32<1_000_000>) {
        let led = cx.local.blink_led;
        if led.is_set_low().unwrap() {
            led.set_high().ok();
        } else {
            led.set_low().ok();
        }
        let next_instant = instant + 1000.millis();
        blink::spawn_at(next_instant, next_instant).unwrap();
    }

    #[task(binds = GPIOTE, local=[button_led], shared = [gpiote])]
    fn gpiote_cb(mut cx: gpiote_cb::Context){
        cx.shared.gpiote.lock(|gpiote| {
            if gpiote.channel0().is_event_triggered() {
                let b_led = cx.local.button_led;
                if b_led.is_set_low().unwrap() {
                    b_led.set_high().ok();
                } else {
                    b_led.set_low().ok();
                }
            } else if gpiote.channel1().is_event_triggered() {
                uart1_tx::spawn().unwrap();
            } 
            // Reset all events
            gpiote.reset_events();
            // Debounce
            //debounce::spawn_after(50.millis()).ok();
        });
    }

    #[task(binds = UARTE1, shared=[uarte1], local = [rx_buf1, rx_led1])]
    fn uart0_rx(mut cx: uart0_rx::Context){
        cx.shared.uarte1.lock(|u| {
            // Stop reception.
            u.tasks_stoprx.write(|w| unsafe { w.bits(1) });

            // Wait for the reception to have stopped.
            while u.events_rxto.read().bits() == 0 {}

            // Set Easy DMA buffer addresses and size for transfer from rxd register.
            u.rxd.maxcnt.write(|w| unsafe { w.bits(cx.local.rx_buf1.len() as u32) });
            u.rxd.ptr.write(|w| unsafe { w.ptr().bits(cx.local.rx_buf1.as_ptr() as u32) });

            // Debugging info
            let amout = u.rxd.amount.read().bits();
            rprintln!("Amount0: {}", amout);
            for byte in cx.local.rx_buf1.iter(){
                rprintln!("data0: {}", byte);
            }
            
            // Status led; LED3
            let rx_led = cx.local.rx_led1;
            if rx_led.is_set_low().unwrap() {
                rx_led.set_high().ok();
            } else {
                rx_led.set_low().ok();
            }
            // Reset the event.
            u.events_endrx.write(|w| w);

            // Now that this byte has been handled, start receiver in wait for another byte.
            u.tasks_startrx.write(|w| unsafe { w.bits(1) });
        });
    }

    #[task(shared=[uarte1], local = [tx_buf1])]
    fn uart1_tx(mut cx: uart1_tx::Context){

        cx.shared.uarte1.lock(|u| {
            // Reset the events.
            u.events_endtx.reset();
            u.events_txstopped.reset();

            u.txd.ptr.write(|w| unsafe { w.ptr().bits(cx.local.tx_buf1.as_ptr() as u32) });
            u.txd.maxcnt.write(|w| unsafe { w.maxcnt().bits(cx.local.tx_buf1.len() as _) });

            u.tasks_starttx.write(|w| unsafe { w.bits(1) });

            // Wait for transmission to end.
            while u.events_endtx.read().bits() == 0 {}

            // Reset the event
            u.events_txstopped.reset();

            u.tasks_stoptx.write(|w| unsafe { w.bits(1) });
        });

    }
}
