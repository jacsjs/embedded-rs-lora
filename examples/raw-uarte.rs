#![no_main]
#![no_std]

use panic_halt as _;
#[rtic::app(device = nrf52840_hal::pac, dispatchers = [SWI0_EGU0, SWI1_EGU1])]
mod app {
    use rtt_target::{rprintln, rtt_init_print};
    use cortex_m::asm;
    use embedded_hal::digital::{OutputPin, StatefulOutputPin};
    use nrf52840_hal::{
        gpio::{p0, Level, Output, Pin, PushPull}, 
        pac::{TIMER1, TIMER4, UARTE1},
        timer,
    };
    use embedded_rs_lora::{
        mono::{ExtU32, MonoTimer}, 
        trace::{Trace, TraceState}};

    #[monotonic(binds = TIMER1, default = true)]
    type MyMono = MonoTimer<TIMER1>;

    #[shared]
    struct Shared {
        #[lock_free]
        data_buf: [u8; 128],
        #[lock_free]
        trace: Trace<TIMER4>,
    }

    #[local]
    struct Local {
        uarte1: UARTE1,
        rx_buf1: [u8; 6],
        blink_led: Pin<Output<PushPull>>,
        data_current_len: usize,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        rtt_init_print!();

        // Timers
        let mut mono = MonoTimer::new(cx.device.TIMER1);
        let trace_timer = timer::Timer::into_periodic(timer::Timer::new(cx.device.TIMER4));
        
        // Ownership of Peripherals.
        let p0 = p0::Parts::new(cx.device.P0);
        let uarte1 = cx.device.UARTE1;
        
        // LED for status.
        let blink_led = p0.p0_13.into_push_pull_output(Level::High).degrade();

        // For uarte1 transmissions.
        let sen_tx = p0.p0_04.into_push_pull_output(Level::High).degrade();
        let sen_rx = p0.p0_31.into_floating_input().degrade();

        // Configure the uarte1 peripheral
        uarte1.config.write(|w| { w.parity().excluded().hwfc().disabled() });

        // Set baud rate
        uarte1.baudrate.write(|w| w.baudrate().baud9600());

        // Set RX and TX pins for uarte1 perhebial. 
        uarte1.psel.rxd.write(|w| unsafe { w.bits(sen_rx.psel_bits()) });
        uarte1.psel.txd.write(|w| unsafe { w.bits(sen_tx.psel_bits()) });

        let rx_buf1 = [0; 6];
        let data_buf = [0;128];

        let trace = Trace::new(trace_timer);

        // Enable uarte peripheral.
        uarte1.enable.write(|w| w.enable().enabled());

        // START_RX() shortcut from an ENDRX event signifying that the buffer is full.
        uarte1.shorts.write(|w| { w.endrx_startrx().enabled() });

        // When the start rx shortcut is triggered (after the buffer has been filled)
        // we want to prepare for the next buffer by flipping etc. (double buffered)
        // However, if the buffer is not full, it means we are done and we must disable
        // this shortcut in order for the timeout to be able to trigger.
        uarte1.intenset.write(|w| w.rxstarted().set_bit());

        // Set Easy DMA buffer addresses and size for transfer from rxd register.
        uarte1.rxd.maxcnt.write(|w| unsafe { w.bits(rx_buf1.len() as u32) });
        uarte1.rxd.ptr.write(|w| unsafe { w.ptr().bits(rx_buf1.as_ptr() as u32) });

        // Start the Uarte receiver.
        uarte1.tasks_startrx.write(|w| w.tasks_startrx().set_bit() );

        // Initiate periodic status blink, just as a sign of life.
        blink::spawn_after(1.secs(), mono.now()).unwrap();

        (Shared { data_buf, trace }, Local { uarte1, rx_buf1, blink_led, data_current_len: 0 }, init::Monotonics(mono))
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            rprintln!("Sleeping...");
            asm::wfi();
        }
    }

    // Same priority as the corresponding interrupt, in order to share lock-free variables.
    // They wont interleave anyways because that's the entire point of this async function.
    // Its called asynchronously when a rx_to event is invoked (timeout).
    #[task(priority=5, shared=[trace])]
    fn process_trace(cx: process_trace::Context) {
        cx.shared.trace.display_trace();
    }

    // Highest priority; we wouldn't want to miss any precious data would we?
    #[task(binds = UARTE1, priority=5, shared=[data_buf, trace], local = [uarte1, rx_buf1, data_current_len])]
    fn rx_interrupt(cx: rx_interrupt::Context){

        let uarte = cx.local.uarte1;

        let trace = cx.shared.trace;

        // BEGIN STATE ENDRX
        if uarte.inten.read().endrx().bit_is_set() && uarte.events_endrx.read().events_endrx().bit() {

            let bytes_since_last = uarte.rxd.amount.read().bits(); 

            trace.log(TraceState::Endrx, bytes_since_last as usize);

            // Buffer processing.
            let end_position: usize = *cx.local.data_current_len + bytes_since_last as usize;

            for i in 0..bytes_since_last as usize {
                cx.shared.data_buf[*cx.local.data_current_len + i] = cx.local.rx_buf1[i];
            }
            *cx.local.data_current_len = end_position;

            if bytes_since_last < 6  {
                // DONE
                // Disable endrx_startrx so that the buffer doesn't start listening again after ISR ends.
                uarte.shorts.write(|w| w.endrx_startrx().disabled() );

                // Clearing an interrupt by writing 0 to an event register, or disabling an interrupt using the INTENCLR
                // register, can take up to four CPU clock cycles to take effect. This means that an interrupt may reoccur
                // immediately, even if a new event has not come, if the program exits an interrupt handler after the
                // interrupt is cleared or disabled but before four clock cycles have passed.
                asm::nop();
                asm::nop();
                asm::nop();
                asm::nop();

                //uarte.tasks_stoprx.write(|w| w.tasks_stoprx().set_bit() );
            }
            uarte.events_endrx.reset();
        } 

        // BEGIN STATE TIMEOUT
        else if uarte.inten.read().rxto().bit_is_set() && uarte.events_rxto.read().events_rxto().bit(){
            trace.log(TraceState::Timeout, 0);

            // OFF(rxto)
            uarte.intenclr.write(|w| w.rxto().clear() );
            uarte.events_rxto.reset();

            // OFF(endrx)
            uarte.intenclr.write(|w| w.endrx().clear() );
            uarte.events_endrx.reset();

            // On timeout, it should indicate we are done for now.
            // Reset states completely by enabling the shortcut to start event again.
            uarte.shorts.write(|w| w.endrx_startrx().enabled() );

            // HERE WE WANT TO PROCESS THE BUFFER BEFORE A NEW ONE IS STARTING TO FILL
            // IF WE DO NOT TAKE CARE OF BYTES IN THE BUFFER UNTIL THIS ISR ENDS
            // STARTRX WILL START FILLING THE BUFFER AGAIN, AND WE LOSE DATA.

            // Display log when there is no critical tasks being performed.
            process_trace::spawn_after(fugit::TimerDurationU32::from_ticks(2_000_000)).unwrap();

            // FLUSH RXD into BUFFER. 
            // Up to 4 bytes are received after END_RX before TIMEOUT is triggered.
            uarte.tasks_flushrx.write(|w| w.tasks_flushrx().set_bit() );
        }

        // BEGIN STATE RXSTARTED
        else if uarte.inten.read().rxstarted().bit_is_set() && uarte.events_rxstarted.read().events_rxstarted().bit() {
            trace.log(TraceState::Rxstarted, uarte.rxd.ptr.read().bits() as usize);

            // OFF(endrx)
            uarte.intenclr.write(|w| w.endrx().clear() );
            uarte.events_endrx.reset();

            // ON(rxdrdy)
            uarte.intenset.write(|w| w.rxdrdy().set() );

            uarte.events_rxstarted.reset();

            // HERE INSERT BUFFER FLIPPING OR BUFFER UPDATING
            // The nRF52840 Product Specification implementation just increments 
            // the buffer by max_cnt * data_size.

            // Set Easy DMA buffer addresses and size for transfer from rxd register.
            uarte.rxd.maxcnt.write(|w| unsafe { w.bits(cx.local.rx_buf1.len() as u32) });
            uarte.rxd.ptr.write(|w| unsafe { w.ptr().bits(cx.local.rx_buf1.as_ptr() as u32) });
        }

        // BEGIN STATE RDXRDY
        else if uarte.inten.read().rxdrdy().bit_is_set() && uarte.events_rxdrdy.read().events_rxdrdy().bit(){
            trace.log(TraceState::Rxdrdy, 0);

            // OFF(rxdrdy)
            uarte.intenclr.write(|w|w.rxdrdy().clear() );
            uarte.events_rxdrdy.reset();

            // ON(rxto)
            uarte.intenset.write(|w| w.rxto().set() );

            // ON(endrx)
            uarte.intenset.write(|w| w.endrx().set() );

            // From Nordic nRF52840 Product Specification:
            //
            // Note: If the ENDRX event has not been generated when the UARTE receiver stops, indicating that
            // all pending content in the RX FIFO has been moved to the RX buffer, the UARTE will generate the
            // ENDRX event explicitly even though the RX buffer is not full. In this scenario the ENDRX event will
            // be generated before the RXTO event is generated.
            uarte.tasks_stoprx.write(|w| w.tasks_stoprx().set_bit() );

            // Clearing an interrupt by writing 0 to an event register, or disabling an interrupt using the INTENCLR
            // register, can take up to four CPU clock cycles to take effect. This means that an interrupt may reoccur
            // immediately, even if a new event has not come, if the program exits an interrupt handler after the
            // interrupt is cleared or disabled but before four clock cycles have passed.
            asm::nop();
            asm::nop();
            asm::nop();
            asm::nop();
        }
    }        

    // Lowest priority.
    #[task(priority = 1, local = [blink_led])]
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
}
