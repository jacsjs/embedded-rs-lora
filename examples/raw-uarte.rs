#![no_main]
#![no_std]

use panic_rtt_target as _;
#[rtic::app(device = nrf52840_hal::pac, dispatchers = [SWI0_EGU0, SWI1_EGU1, SWI2_EGU2, SWI3_EGU3])]
mod app {
    use rtt_target::{rprintln, rtt_init_print};
    use cortex_m::asm;
    use embedded_hal::digital::{OutputPin, StatefulOutputPin};
    use nrf52840_hal::{
        gpio::{p0, Level, Output, Pin, PushPull}, 
        pac::{TIMER1, TIMER3, TIMER4, UARTE1}, 
        ppi::{self, ConfigurablePpi, Ppi}, 
        timer::Timer
    };
    use heapless::{
        spsc::{Consumer, Producer, Queue}, FnvIndexMap, Vec};
    use embedded_rs_lora::{
        data_structures::double_buffer::{BufferConsumer, BufferProducer, DoubleBuffer, DoubleBufferConsumer, DoubleBufferProducer}, 
        util::{
            mono::{ExtU32, MonoTimer},
            trace::{Trace, TraceState}}};

    #[monotonic(binds = TIMER1, default = true)]
    type MyMono = MonoTimer<TIMER1>;

    const RXDRDY_TIMEOUT_CYCLES: u32 = 500_000;
    const RX_BUF_SIZE_BYTES: usize =  256;
    const QUEUE_SIZE: usize = 8;

    #[shared]
    struct Shared {
        uarte1: UARTE1,
        #[lock_free]
        trace: Trace<TIMER4>,
        #[lock_free]
        response_actions: FnvIndexMap::<Vec<u8, 5>, RxAction, 8>,
    }

    #[local]
    struct Local {
        blink_led: Pin<Output<PushPull>>,
        uarte1_timer: Timer<TIMER3>,
        rp: DoubleBufferProducer<'static, [u8; RX_BUF_SIZE_BYTES]>,
        rc: DoubleBufferConsumer<'static, [u8; RX_BUF_SIZE_BYTES]>,
        tx_queue_producer: Producer<'static, Vec<u8, RX_BUF_SIZE_BYTES>, QUEUE_SIZE>,
        tx_queue_consumer: Consumer<'static, Vec<u8, RX_BUF_SIZE_BYTES>, QUEUE_SIZE>,
    }

    #[init(local = [
        rb: DoubleBuffer<[u8; RX_BUF_SIZE_BYTES]> = DoubleBuffer::new([0; RX_BUF_SIZE_BYTES], [0; RX_BUF_SIZE_BYTES]),
        tx_queue: Queue<Vec<u8, RX_BUF_SIZE_BYTES>, QUEUE_SIZE> = Queue::new(),
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        rtt_init_print!();

        // Timers
        let mut mono = MonoTimer::new(cx.device.TIMER1);
        let trace_timer = Timer::into_periodic(Timer::new(cx.device.TIMER4));

        let trace = Trace::new(trace_timer);

        let (rp, rc) = cx.local.rb.split();
        let (tx_queue_producer, tx_queue_consumer) = cx.local.tx_queue.split();

        // Heapless hashmap of AT tags and actions, for dynamic decision making
        // of asynchronous tx/rx events.
        let response_actions = FnvIndexMap::new();

        // Ownership of Peripherals.
        let p0 = p0::Parts::new(cx.device.P0);
        let ppi = ppi::Parts::new(cx.device.PPI);
        let uarte1 = cx.device.UARTE1;

        // Maybe this is not needed, I just included it as a precaution.
        if uarte1.enable.read().bits() != 0 {

            uarte1.tasks_stoptx.write(|w| unsafe { w.bits(1) });
            while uarte1.events_txstopped.read().bits() == 0 {}

            uarte1.enable.write(|w| w.enable().disabled());
        }

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
        let mut uarte1_timer = Timer::new(cx.device.TIMER3).into_oneshot();
        uarte1_timer.enable_interrupt();

        // Task and event handles.
        let event_compare0_ref_for_ppi = uarte1_timer.event_compare_cc0();
        let event_rdxrdy1_ref_for_ppi = &uarte1.events_rxdrdy;
        let task_stoprx_ref_for_ppi = &uarte1.tasks_stoprx;
        let task_start_timer_ref_for_ppi = uarte1_timer.task_start();
        let task_clear_timer_ref_for_ppi = uarte1_timer.task_count();

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
        uarte1.psel.rxd.write(|w| unsafe { w.bits(sen_rx.psel_bits()).connect().connected() });
        uarte1.psel.txd.write(|w| unsafe { w.bits(sen_tx.psel_bits()).connect().connected() });

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

        // Start the Uarte receiver.
        uarte1.tasks_startrx.write(|w| w.tasks_startrx().set_bit() );

        uarte1.intenset.write(|w| w.error().set() );

        // Initiate periodic status blink, just as a sign of life.
        blink::spawn_after(1.secs(), mono.now()).unwrap();
        display::spawn_after(3.secs(), mono.now()).unwrap();
        test_uarte::spawn().ok().unwrap();

        (Shared { uarte1, trace, response_actions}, Local { blink_led, uarte1_timer, rp, rc, tx_queue_producer, tx_queue_consumer }, init::Monotonics(mono))
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            rprintln!("Sleeping...");
            asm::wfi();
        }
    }

    #[task(priority=6, shared=[trace])]
    fn display(cx: display::Context, instant: fugit::TimerInstantU32<1_000_000>){
        cx.shared.trace.display_trace();
        let next_instant = instant + 3.secs();
        display::spawn_at(next_instant, next_instant).unwrap();

    }
    pub enum RxAction {
        ATID,
        ATTEST,
        ATMODE,
    }
    impl core::fmt::Debug for RxAction {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            match self {
                RxAction::ATID =>   write!(f, "ATID  "),
                RxAction::ATTEST => write!(f, "ATTEST"),
                RxAction::ATMODE => write!(f, "ATMODE"),
            }
        }
    }

    #[task(priority=4, shared=[response_actions], local=[tx_queue_producer])]
    fn test_uarte(cx: test_uarte::Context){
        // Simulate TX for now.
        // Later this method will receive commands from other tasks
        // such as sensor collection tasks, or network watchdogs.
        let example_tag= heapless::Vec::<u8, 5>::from_slice(
            b"+ID"
        ).unwrap();
        let example_msg= heapless::Vec::<u8, 256>::from_slice(
            b"AT+ID\r\n"
        ).unwrap();

        // Prepare the proper data structures for sending.
        cx.shared.response_actions.insert(example_tag, RxAction::ATTEST).unwrap();
        cx.local.tx_queue_producer.enqueue(example_msg).unwrap();

        // Pre-sending preparations done, can send now!
        start_tx::spawn().unwrap();
    }

    /// Start tx from the tag_msg_map queue, if there is a transmission pending.
    /// TODO: temporarily panics if the queue is empty; during testing only.
    #[task(priority = 4, shared=[uarte1], local=[tx_queue_consumer])]
    fn start_tx(mut cx: start_tx::Context){
        cx.shared.uarte1.lock(|uarte1|{

            // Check if there is work to be done...
            if let Some(msg) = cx.local.tx_queue_consumer.dequeue() {

                // Reset the events.
                uarte1.events_endtx.reset();
                uarte1.events_txstopped.reset();

                // Set Easy DMA buffer addresses based on the message memory address and size.
                uarte1.txd.ptr.write(|w| unsafe { w.ptr().bits(msg.as_ptr() as u32) });
                uarte1.txd.maxcnt.write(|w| unsafe { w.bits(msg.len() as u32) });

                rprintln!("tx msg: {:?}", msg.utf8_chunks());

                // ON(endtx)
                uarte1.intenset.write(|w| w.endtx().set() );

                // Start transmission
                uarte1.tasks_starttx.write(|w| w.tasks_starttx().set_bit() );

            } else {
                // Something here to try again after a while?
                rprintln!("empty tx work-queue");
            }
        });
    }


    // Process the currently completed rx_buffer
    // in the background with lower priority.
    #[task(priority=4, shared=[response_actions], local=[rc])]
    fn process_rx(cx: process_rx::Context, bytes_to_process: usize) {
        let buf = cx.local.rc.consume().split_at(bytes_to_process).0;
        if buf.is_empty() { return } 
        let res_slice = buf.split(|&b| b == b':').nth(0).unwrap();
        let res_tag = match Vec::<u8, 5>::from_slice(res_slice) {
            Ok(v) => v,
            Err(e) => {
                rprintln!("Error in process_rx: {:?}", e);
                Vec::<u8, 5>::new()
            },
        };
        if cx.shared.response_actions.contains_key(&res_tag) {
            // It was an expected response, fire it's mapped action.
            let action = cx.shared.response_actions.get(&res_tag).unwrap();
            rprintln!("rx msg: {:?}", buf.utf8_chunks());
            rprintln!("Action to take: {:?}", action);

        } else {
            // Async message from the world outside. In need of further
            // processing to determine what they want.
            rprintln!("Not awaited response is: {:?}", res_slice);
        }
        // Just redo the test, for fun
        test_uarte::spawn_after(fugit::TimerDurationU32::from_ticks(1_000_000)).unwrap();
    }

    // Same priority as the corresponding interrupt, in order to share lock-free variables.
    // They wont interleave anyways because that's the entire point of this async function.
    // Its called asynchronously when a uart1_timer.compare event iss triggered.
    #[task(binds = TIMER3, priority=6, shared=[uarte1], local=[uarte1_timer])]
    fn rx_timeout(mut cx: rx_timeout::Context){
        cx.shared.uarte1.lock(|uarte1|{
            cx.local.uarte1_timer.reset_event();
            uarte1.shorts.reset();
        });
    }

    // Highest priority; we wouldn't want to miss any precious data would we?
    #[task(binds = UARTE1, priority=6, shared=[trace, uarte1], local = [rp])]
    fn uarte1_interrupt(mut cx: uarte1_interrupt::Context){
        cx.shared.uarte1.lock(| uarte| {
            let trace = cx.shared.trace;
            // BEGIN STATE TIMEOUT
            if uarte.events_rxto.read().events_rxto().bit() {
                uarte.events_endrx.reset();
                uarte.events_rxto.reset();
                
                trace.log(TraceState::RxTo, uarte.rxd.amount.read().bits() as usize);

                uarte.shorts.write(|w| { w.endrx_startrx().enabled() });

                // TODO: consider spawning "process_rxbuffer" here aswell in
                // edge-cases where we have almost full buffer yet bytes on
                // the rdx line waiting for a flush_rx operation.

                // FLUSH RXD into BUFFER. 
                // Up to 4 bytes are received after END_RX before TIMEOUT is triggered.
                uarte.tasks_flushrx.write(|w| w.tasks_flushrx().set_bit() );
            }
            // BEGIN STATE ENDRX
            else if uarte.events_endrx.read().events_endrx().bit() {
                uarte.events_endrx.reset();
                
                let bytes_that_require_processing = uarte.rxd.amount.read().bits(); 
                trace.log(TraceState::Endrx, bytes_that_require_processing as usize);

                // Start processing buffer in background with lower priority.
                match process_rx::spawn(bytes_that_require_processing as usize) {
                    Ok(_) => (),
                    Err(err) => rprintln!("Error: Process_rxbuffer task already pending! {:?}", err),
                }
            } 
            // BEGIN STATE RXSTARTED
            else if uarte.events_rxstarted.read().events_rxstarted().bit() {
                uarte.events_rxstarted.reset();
                uarte.events_endrx.reset();

                trace.log(TraceState::Rxstarted, 0);
                //Swap buffer.
                cx.local.rp.swap();

                // Set Easy DMA buffer addresses and size for transfer from rxd register.
                uarte.rxd.maxcnt.write(|w| unsafe { w.bits(RX_BUF_SIZE_BYTES as u32) });
                uarte.rxd.ptr.write(|w| unsafe { w.ptr().bits(cx.local.rp.as_ref().as_ptr() as u32) });

            }
            // BEGIN STATE ENDTX
            if uarte.events_endtx.read().events_endtx().bit() {
                uarte.events_endtx.reset();
                uarte.tasks_stoptx.write(|w| w.tasks_stoptx().set_bit() );

                let bytes_transmitted = uarte.txd.amount.read().bits();
                trace.log(TraceState::Endtx, bytes_transmitted as usize);

                // Done! Mark next transmission as pending, 
                // if it's not already pending by another task of course.
                match start_tx::spawn() {
                    Ok(_) => (),
                    Err(err) => rprintln!("Error: start_tx task already pending! {:?}", err),
                }
            }
            // BEGIN STATE ERROR
            if uarte.events_error.read().events_error().bit() {
                // Undefined?
                rprintln!("error: {}", uarte.errorsrc.read().bits());
                uarte.events_error.reset();
            }
        });
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
        let next_instant = instant + 1.secs();
        blink::spawn_at(next_instant, next_instant).unwrap();
    }
}
