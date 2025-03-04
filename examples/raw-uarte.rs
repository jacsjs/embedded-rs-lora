#![no_main]
#![no_std]

use panic_halt as _;
#[rtic::app(device = nrf52840_hal::pac, dispatchers = [SWI0_EGU0, SWI1_EGU1])]
mod app {
    use defmt::println;
    use fugit::Instant;
    use rtt_target::{rprintln, rtt_init_print};
    use cortex_m::asm;
    use embedded_hal::digital::{OutputPin, StatefulOutputPin};
    use nrf52840_hal::{
        gpio::{p0, p1, Level, Output, Pin, PushPull}, 
        gpiote::Gpiote, 
        pac::{twim0::events_rxstarted, Interrupt, TIMER0, TIMER1, UARTE1}, Timer,
    };
    use embedded_rs_lora::{mono::{ExtU32, Instance32, MonoTimer}};

    #[monotonic(binds = TIMER1, default = true)]
    type MyMono = MonoTimer<TIMER1>;

    // How long uarte timeout is.
    const RX_TIMEOUT_TICKS: u32 = 1_000_000;

    #[shared]
    struct Shared {
        gpiote: Gpiote, 
        #[lock_free]
        uarte1: UARTE1,
        #[lock_free]
        rx_buf1: [u8; 4],
        #[lock_free]
        timeout_timer: Timer<TIMER0>,
    }

    #[local]
    struct Local {
        blink_led: Pin<Output<PushPull>>,
        button_led: Pin<Output<PushPull>>,
        rx_led1: Pin<Output<PushPull>>,
        tx_buf1: [u8; 17],
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        rtt_init_print!();

        // Timers
        let mut mono = MonoTimer::new(cx.device.TIMER1);

        let timeout_timer = Timer::one_shot(cx.device.TIMER0);
        
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

        // Configure the uarte1 peripheral
        uarte1.config.write(|w| { w.parity().excluded().hwfc().disabled() });

        // Set baud rate
        uarte1.baudrate.write(|w| w.baudrate().baud9600());

        // Set RX and TX pins for uarte1 perhebial. 
        uarte1.psel.rxd.write(|w| unsafe { w.bits(sen_rx.psel_bits()) });
        uarte1.psel.txd.write(|w| unsafe { w.bits(sen_tx.psel_bits()) });

        let rx_buf1 = [0; 4];
        let tx_buf1 = "Hello Arduino!:D\n".as_bytes().try_into().unwrap();

        // Enable uarte peripheral.
        uarte1.enable.write(|w| w.enable().enabled());
        // START_RX() shortcut from an ENDRX event signifying that the buffer is full.
        uarte1.shorts.write(|w| { w.endrx_startrx().enabled() });

        // When the start rx shortcut is triggered (after the buffer has been filled)
        // we want to prepare for the next buffer by flipping etc. (double buffered)
        uarte1.intenset.write(|w| w.rxstarted().set_bit());

        // Set Easy DMA buffer addresses and size for transfer from rxd register.
        uarte1.rxd.maxcnt.write(|w| unsafe { w.bits(rx_buf1.len() as u32) });
        uarte1.rxd.ptr.write(|w| unsafe { w.ptr().bits(rx_buf1.as_ptr() as u32) });

        // Start the Uarte receiver.
        uarte1.tasks_startrx.write(|w| w.tasks_startrx().set_bit() );

        rtic::pend(Interrupt::UARTE1);


        // Initiate periodic processes
        blink::spawn_after(1.secs(), mono.now()).unwrap();

        (Shared { gpiote, uarte1, rx_buf1, timeout_timer }, Local { blink_led, button_led, rx_led1, tx_buf1, }, init::Monotonics(mono))
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            rprintln!("Sleeping...");
            asm::wfi();
        }
    }

    #[task(binds = UARTE1, priority=5, shared=[uarte1, rx_buf1], local = [rx_led1])]
    fn rx_interrupt(mut cx: rx_interrupt::Context){

        rprintln!("RX_INTERRUPT");
        // BEGIN STATE ENDRX
        if cx.shared.uarte1.events_endrx.read().events_endrx().bit() {
            rprintln!("STATE_ENDRX");

            cx.shared.uarte1.events_endrx.reset();


            // HERE WE WANT TO PROCESS THE BUFFER BEFORE A NEW ONE IS STARTING TO FILL
            // IF WE DONT TAKE CARE OF BYTES IN THE BUFFER UNTIL THIS ISR ENDS
            // STARTRX WILL START FILLING THE BUFFER AGAIN, AND WE LOSE DATA.
            let bytes_since_last = cx.shared.uarte1.rxd.amount.read().bits(); 

            let test = cx.shared.uarte1.rxd.ptr.as_ptr();

            rprintln!("Since last time: {}", bytes_since_last);
        } 
        // BEGIN STATE TIMEOUT
        else if cx.shared.uarte1.events_rxto.read().events_rxto().bit(){

            rprintln!("STATE_TIMEOUT");

            // OFF(rxto)
            cx.shared.uarte1.intenclr.write(|w| w.rxto().clear() );
            cx.shared.uarte1.events_rxto.reset();

            cx.shared.uarte1.events_endrx.reset();

            // HERE WE WANT TO PROCESS THE BUFFER BEFORE A NEW ONE IS STARTING TO FILL
            // IF WE DONT TAKE CARE OF BYTES IN THE BUFFER UNTIL THIS ISR ENDS
            // STARTRX WILL START FILLING THE BUFFER AGAIN, AND WE LOSE DATA.

            let bytes_since_last = cx.shared.uarte1.rxd.amount.read().bits(); 

            let test = cx.shared.uarte1.rxd.ptr.as_ptr();

            rprintln!("Since last time: {}", bytes_since_last);

            // FLUSH RXD into BUFFER. 
            // Up to 4 bytes are received after END_RX before TIMEOUT is triggered.
            cx.shared.uarte1.tasks_flushrx.write(|w| w.tasks_flushrx().set_bit() );
        }
        // BEGIN STATE RXSTARTED
        else if cx.shared.uarte1.events_rxstarted.read().events_rxstarted().bit() {
            rprintln!("STATE_RXSTARTED");

            // OFF(endrx)
            cx.shared.uarte1.intenclr.write(|w| w.endrx().clear() );
            cx.shared.uarte1.events_endrx.reset();
            cx.shared.uarte1.events_rxstarted.reset();

            // HERE INSERT BUFFER FLIPPING OR BUFFER UPDATING
            // The datasheet implementation just increments 
            // the buffer by maxcnt * datasize.

            // Set Easy DMA buffer addresses and size for transfer from rxd register.
            cx.shared.uarte1.rxd.maxcnt.write(|w| unsafe { w.bits(cx.shared.rx_buf1.len() as u32) });
            cx.shared.uarte1.rxd.ptr.write(|w| unsafe { w.ptr().bits(cx.shared.rx_buf1.as_ptr() as u32) });

            // ON(rxdrdy)
            cx.shared.uarte1.intenset.write(|w| w.rxdrdy().set() );
        }
        // BEGIN STATE RDXRDY
        else if cx.shared.uarte1.events_rxdrdy.read().events_rxdrdy().bit(){
            rprintln!("STATE_RXDRDY");

            // OFF(rxdrdy)
            cx.shared.uarte1.intenclr.write(|w|w.rxdrdy().clear() );
            cx.shared.uarte1.events_rxdrdy.reset();

            // ON(rxto)
            cx.shared.uarte1.intenset.write(|w| w.rxto().set() );

            // ON(endrx)
            cx.shared.uarte1.intenset.write(|w| w.endrx().set() );
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
            } 
            gpiote.reset_events();
        });
    }
}
