#![no_main]
#![no_std]

use panic_halt as _;
#[rtic::app(device = nrf52840_hal::pac, dispatchers = [SWI0_EGU0, SWI1_EGU1])]
mod app {
    use defmt::println;
    use rtt_target::{rprintln, rtt_init_print};
    use cortex_m::asm;
    use embedded_hal::digital::{OutputPin, StatefulOutputPin};
    use nrf52840_hal::{
        gpio::{p0, Level, Output, Pin, PushPull}, pac::{aar::enable::W, TIMER1, TIMER3, TIMER4, UARTE1}, ppi::{self, ConfigurablePpi, Ppi}, timer::{self, Periodic}, Timer
    };
    use embedded_rs_lora::{
        mono::{ExtU32, MonoTimer}, 
        trace::{Trace, TraceState}};

    #[monotonic(binds = TIMER1, default = true)]
    type MyMono = MonoTimer<TIMER1>;

    const RXDRDY_TIMEOUT_CYCLES: u32 = 1_000_000;

    #[shared]
    struct Shared {
        #[lock_free]
        uarte1: UARTE1,
        #[lock_free]
        data_buf: [u8; 256],
        #[lock_free]
        trace: Trace<TIMER4>,
    }

    #[local]
    struct Local {
        rx_buf1: [u8; 6],
        blink_led: Pin<Output<PushPull>>,
        data_current_len: usize,
        uarte1_timer: Timer<TIMER3>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        rtt_init_print!();

        // Timers
        let mut mono = MonoTimer::new(cx.device.TIMER1);
        let trace_timer = timer::Timer::into_periodic(timer::Timer::new(cx.device.TIMER4));

        // Ownership of Peripherals.
        let p0 = p0::Parts::new(cx.device.P0);
        let ppi = ppi::Parts::new(cx.device.PPI);
        let uarte1 = cx.device.UARTE1;


        // This is the uarte timer.
        // It is supposed to use PPI, with compare event triggering tasks within
        // the uarte, such as STOP_RX
        // A basic pipeline is as follows:
        //                                                
        //  STARTRX  -> RXDRDY
        //                 |
        //                  \---> timer.start()
        //                                |
        //                                |                                CHEN[n]
        //                                 \-----> Compare[m] = CH[n].EEP --------> CH[n].TEP = STOP_RX -> RX_TO -> END_RX  
        //                                             ^                                                     ^
        //                                             |                                                     |
        //                                   RXDRDY_TIMEOUT_CYCLES                                      disable short:
        //                                                                                            END_RX -> START_RX
        //                 
        //  Then also after End_RX I would want to start the same timer again for startrx:
        //
        // 
        cx.device.TIMER3.cc[0].write(|w| unsafe { w.bits(RXDRDY_TIMEOUT_CYCLES) });
        let mut uarte1_timer = timer::Timer::new(cx.device.TIMER3).into_oneshot();
        uarte1_timer.enable_interrupt();

        // Task and event handles.
        let event_compare0_ref_for_ppi = uarte1_timer.event_compare_cc0();
        let task_stoprx_ref_for_ppi = &uarte1.tasks_stoprx;

        let event_rdxrdy1_ref_for_ppi = &uarte1.events_rxdrdy;
        let task_start_timer_ref_for_ppi = uarte1_timer.task_start();
        let task_clear_timer_ref_for_ppi = uarte1_timer.task_count();

        let event_endrx_ref_for_ppi = &uarte1.events_endrx;
        let task_stop_timer_ref_for_ppi = uarte1_timer.task_stop();

        // Setup PPI channel 0:    TIMER3.compare --------------> STOP_RX           CH[0].TEP
        //                                |              \------>                   CH[0].FORK
        //                                 \
        //                                  \-------------------> TIMER3_clear()    Timer shortcut
        //                                                \-----> TIMER3_stop()     Timer shortcut
        // 
        //
        // Setup PPI channel 1:    UARTE1.rxd_rdy --------------> TIMER3.start()    CH[1].TEP
        //                                               \------> TIMER3.clear()    CH[1].FORK
        //
        //
        // Setup PPI channel 2:    UARTE1.end_rx ---------------> TIMER3.clear()    CH[1].TEP
        //                                               \------> TIMER3_stop()     CH[1].FORK
        //
        //
        // On each PPI channel, the signals are synchronized to the 16 MHz clock to avoid any internal violation
        // of setup and hold timings. As a consequence, events that are synchronous to the 16 MHz clock will be
        // delayed by one clock period, while other asynchronous events will be delayed by up to one 16 MHz clock
        // period.
        // Note: Shortcuts (as defined in the SHORTS register in each peripheral) are not affected by this 16
        // MHz synchronization, and are therefore not delayed.
        let mut ppi_channel0= ppi.ppi0;
        ppi_channel0.set_event_endpoint(event_compare0_ref_for_ppi);
        ppi_channel0.set_task_endpoint(task_stoprx_ref_for_ppi);
        ppi_channel0.enable();

        let mut ppi_channel1 = ppi.ppi1;
        ppi_channel1.set_event_endpoint(event_rdxrdy1_ref_for_ppi);
        ppi_channel1.set_task_endpoint(task_start_timer_ref_for_ppi);
        ppi_channel1.set_fork_task_endpoint(task_clear_timer_ref_for_ppi);
        ppi_channel1.enable();

        let mut ppi_channel2= ppi.ppi2;
        ppi_channel2.set_event_endpoint(event_endrx_ref_for_ppi);
        ppi_channel2.set_fork_task_endpoint(task_clear_timer_ref_for_ppi);
        ppi_channel2.set_task_endpoint(task_stop_timer_ref_for_ppi);
        ppi_channel2.enable();

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
        let data_buf = [0;256];

        let trace = Trace::new(trace_timer);

        // Enable uarte peripheral.
        uarte1.enable.write(|w| w.enable().enabled());


        // When the start rx shortcut is triggered (after the buffer has been filled)
        // we want to prepare for the next buffer by flipping etc. (double buffered)
        // However, if the buffer is not full, it means we are done and we must disable
        // this shortcut in order for the timeout to be able to trigger.
        uarte1.intenset.write(|w| w.rxstarted().set_bit());
        // ON(endrx)
        uarte1.intenset.write(|w| w.endrx().set_bit() );

        // ON(rx_to)
        uarte1.intenset.write(|w| w.rxto().set_bit() ); 

        // Set Easy DMA buffer addresses and size for transfer from rxd register.
        uarte1.rxd.maxcnt.write(|w| unsafe { w.bits(rx_buf1.len() as u32) });
        uarte1.rxd.ptr.write(|w| unsafe { w.ptr().bits(rx_buf1.as_ptr() as u32) });

        // Start the Uarte receiver.
        uarte1.tasks_startrx.write(|w| w.tasks_startrx().set_bit() );

        // Initiate periodic status blink, just as a sign of life.
        blink::spawn_after(1.secs(), mono.now()).unwrap();

        (Shared { uarte1, data_buf, trace }, Local { rx_buf1, blink_led, data_current_len: 0, uarte1_timer}, init::Monotonics(mono))
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            asm::wfi();
        }
    }

    #[task(priority=5, shared=[trace])]
    fn process_trace(cx: process_trace::Context) {
        cx.shared.trace.display_trace();
    }

    // Same priority as the corresponding interrupt, in order to share lock-free variables.
    // They wont interleave anyways because that's the entire point of this async function.
    // Its called asynchronously when a uart1_timer.compare event iss triggered.
    #[task(binds = TIMER3, priority=5, shared=[uarte1], local=[uarte1_timer])]
    fn rx_timeout(cx: rx_timeout::Context){
        cx.local.uarte1_timer.reset_event();
        cx.shared.uarte1.shorts.reset();
        process_trace::spawn_after(2.secs()).unwrap();
    }

    // Highest priority; we wouldn't want to miss any precious data would we?
    #[task(binds = UARTE1, priority=5, shared=[data_buf, trace, uarte1], local = [rx_buf1, data_current_len])]
    fn rx_interrupt(cx: rx_interrupt::Context){
        let uarte = cx.shared.uarte1;
        let trace = cx.shared.trace;

        // BEGIN STATE TIMEOUT
        if uarte.inten.read().rxto().bit_is_set() && uarte.events_rxto.read().events_rxto().bit(){
            uarte.events_endrx.reset();
            uarte.events_rxto.reset();
            
            trace.log(TraceState::Timeout, uarte.rxd.amount.read().bits() as usize);

            uarte.shorts.write(|w| { w.endrx_startrx().enabled() });
            // HERE WE WANT TO PROCESS THE BUFFER BEFORE A NEW ONE IS STARTING TO FILL
            // IF WE DO NOT TAKE CARE OF BYTES IN THE BUFFER UNTIL THIS ISR ENDS
            // STARTRX WILL START FILLING THE BUFFER AGAIN, AND WE LOSE DATA.

            // FLUSH RXD into BUFFER. 
            // Up to 4 bytes are received after END_RX before TIMEOUT is triggered.
            uarte.tasks_flushrx.write(|w| w.tasks_flushrx().set_bit() );
        }

        // BEGIN STATE ENDRX
        else if uarte.inten.read().endrx().bit_is_set() && uarte.events_endrx.read().events_endrx().bit() {
            uarte.events_endrx.reset();
            
            let bytes_since_last = uarte.rxd.amount.read().bits(); 
            trace.log(TraceState::Endrx, bytes_since_last as usize);

            // Buffer processing.
            // Here maybe I should offload buffer handlinng to a software task with lower priority
            // or maybe setup PPI?
            // I dont know if it will work, since the buffer is starting to fill as soon as we leave this interrupt
            // oh wait, it said it is double buffered, maybe I can setup the new buffer in RX_STARTED
            // that has a shortcut so it 100% happens after endRX.
            // which means I setup the new buffer, then RXDRDY spawns
            // which means I can process the currently filled buffer after RXDRDY (which is pointing to the new buffer)
            // In this way, I can process the old (current) buffer while the new one is filling via easyDMA! :D
            let end_position: usize = *cx.local.data_current_len + bytes_since_last as usize;

            for i in 0..bytes_since_last as usize {
                cx.shared.data_buf[*cx.local.data_current_len + i] = cx.local.rx_buf1[i];
            }
            *cx.local.data_current_len = end_position;
        } 

        // BEGIN STATE RXSTARTED
        else if uarte.inten.read().rxstarted().bit_is_set() && uarte.events_rxstarted.read().events_rxstarted().bit() {
            uarte.events_rxstarted.reset();
            uarte.events_endrx.reset();

            trace.log(TraceState::Rxstarted, uarte.rxd.amount.read().bits() as usize);

            // Set Easy DMA buffer addresses and size for transfer from rxd register.
            uarte.rxd.maxcnt.write(|w| unsafe { w.bits(cx.local.rx_buf1.len() as u32) });
            uarte.rxd.ptr.write(|w| unsafe { w.ptr().bits(cx.local.rx_buf1.as_ptr() as u32) });
        }

        else {
            trace.log(TraceState::Rxdrdy, 9999);
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
