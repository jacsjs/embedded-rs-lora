#![no_main]
#![no_std]

use panic_rtt_target as _;
#[rtic::app(device = nrf52840_hal::pac, dispatchers = [SWI0_EGU0, SWI1_EGU1, SWI2_EGU2, SWI3_EGU3])]
mod app {
    use core::ops::DerefMut;

    use at_commands::builder::CommandBuilder;
    use cortex_m::asm;
    use embedded_hal::digital::{OutputPin, StatefulOutputPin};
    use embedded_rs_lora::util::{
        mono::{ExtU32, MonoTimer},
        trace::{Trace, TraceState},
    };
    use heapless::{
        object_pool,
        pool::object::{Object, ObjectBlock},
        spsc::{Consumer, Producer, Queue},
        FnvIndexMap, Vec,
    };
    use nrf52840_hal::{
        gpio::{p0, Level, Output, Pin, PushPull},
        pac::{TIMER1, TIMER3, TIMER4, UARTE1},
        ppi::{self, ConfigurablePpi, Ppi},
        timer::Timer,
        Uarte,
    };
    use rtt_target::{rprintln, rtt_init_print};

    #[monotonic(binds = TIMER1, default = true)]
    type MyMono = MonoTimer<TIMER1>;

    const MSG_BUF_SIZE: usize = 256;
    const POOL_CAPACITY: usize = 5;

    object_pool!(RxPool: [u8; MSG_BUF_SIZE]);
    object_pool!(TxPool: [u8; MSG_BUF_SIZE]);

    #[shared]
    struct Shared {
        uarte1: UARTE1,
        #[lock_free]
        trace: Trace<TIMER4>,
        #[lock_free]
        response_actions: FnvIndexMap<Vec<u8, 5>, RxAction, 8>,
        #[lock_free]
        tx_queue_consumer: Consumer<'static, (Object<TxPool>, usize), POOL_CAPACITY>,
    }

    #[local]
    struct Local {
        blink_led: Pin<Output<PushPull>>,
        rx_timer: TIMER3,
        tx_queue_producer: Producer<'static, (Object<TxPool>, usize), POOL_CAPACITY>,
    }

    #[init(local = [
        tx_queue: Queue<(Object<TxPool>,usize), POOL_CAPACITY> = Queue::new(),
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        rtt_init_print!();

        // Timers
        let mut mono = MonoTimer::new(cx.device.TIMER1);
        let trace_timer = Timer::into_periodic(Timer::new(cx.device.TIMER4));

        let trace = Trace::new(trace_timer);

        let (tx_queue_producer, tx_queue_consumer) = cx.local.tx_queue.split();

        // Heapless hashmap of AT tags and actions, for dynamic decision making
        // of asynchronous tx/rx events.
        let response_actions = FnvIndexMap::new();

        let rx_blocks: &'static mut [ObjectBlock<[u8; MSG_BUF_SIZE]>] = {
            const BLOCK: ObjectBlock<[u8; MSG_BUF_SIZE]> = ObjectBlock::new([0; MSG_BUF_SIZE]); // <=
            static mut RX_BLOCKS: [ObjectBlock<[u8; MSG_BUF_SIZE]>; POOL_CAPACITY] =
                [BLOCK; POOL_CAPACITY];
            #[allow(static_mut_refs)]
            unsafe {
                &mut RX_BLOCKS
            }
        };
        let tx_blocks: &'static mut [ObjectBlock<[u8; MSG_BUF_SIZE]>] = {
            const BLOCK: ObjectBlock<[u8; MSG_BUF_SIZE]> = ObjectBlock::new([0; MSG_BUF_SIZE]); // <=
            static mut TX_BLOCKS: [ObjectBlock<[u8; MSG_BUF_SIZE]>; POOL_CAPACITY] =
                [BLOCK; POOL_CAPACITY];
            #[allow(static_mut_refs)]
            unsafe {
                &mut TX_BLOCKS
            }
        };
        for rx_block in rx_blocks {
            RxPool.manage(rx_block);
        }
        for tx_block in tx_blocks {
            TxPool.manage(tx_block);
        }

        // Ownership of Peripherals.
        let p0 = p0::Parts::new(cx.device.P0);
        let ppi = ppi::Parts::new(cx.device.PPI);
        let uarte1 = cx.device.UARTE1;
        let rx_timer = cx.device.TIMER3;

        // Maybe this is not needed, I just included it as a precaution.
        if uarte1.enable.read().bits() != 0 {
            uarte1.tasks_stoptx.write(|w| unsafe { w.bits(1) });
            while uarte1.events_txstopped.read().bits() == 0 {}

            uarte1.enable.write(|w| w.enable().disabled());
        }

        // Interface for efficient message management.
        // With uarte, timers and ppi interfaces. A basic pipeline is as follows:
        //
        //  STARTRX  -> RXDRDY
        //                 |
        //                  \---> timer.start()
        //                                |                                CHEN[n]
        //                                 \-----> Compare[m] = CH[n].EEP --------> CH[n].TEP = STOP_RX -> RX_TO -> END_RX
        //                                             ^                                                     ^
        //                                             |                                                     |
        //                                        cycle_time                                          disable short
        //
        //  cycle_time = 2^(bitmode + prescaler) / 16_000_000 ~ 16.4 ms. ~8.2 ms works as rxdrdy events happen once every ~6 ms.
        //
        rx_timer.bitmode.write(|w| w.bitmode()._16bit());
        rx_timer.prescaler.write(|w| unsafe { w.bits(2) });
        rx_timer.cc[0].write(|w| unsafe { w.bits(u32::max_value()) });
        rx_timer.shorts.write(|w| w.compare0_stop().set_bit());
        rx_timer.intenset.write(|w| w.compare0().set());

        // Task and event handles.
        let event_timer3_compare0_ppi = &rx_timer.events_compare[0];
        let event_uarte1_rdxrdy_ppi = &uarte1.events_rxdrdy;
        let task_uarte1_stoprx_ppi = &uarte1.tasks_stoprx;
        let task_timer3_start_ppi = &rx_timer.tasks_start;
        let task_timer3_capture_ppi = &rx_timer.tasks_capture[0];

        // Setup PPI channel 0:    TIMER3.compare --------------> STOP_RX           CH[0].TEP
        //                                |              \------>                   CH[0].FORK
        //                                 \
        //                                  \-------------------> TIMER3_stop       Timer shortcut
        //
        // Setup PPI channel 1:    UARTE1.rxd_rdy --------------> TIMER3.start      CH[1].TEP
        //                                               \------> TIMER3.capture0   CH[1].FORK
        //
        // On each PPI channel, the signals are synchronized to the 16 MHz clock to avoid any internal violation
        // of setup and hold timings. As a consequence, events that are synchronous to the 16 MHz clock will be
        // delayed by one clock period, while other asynchronous events will be delayed by up to one 16 MHz clock
        // period.
        // Note: Shortcuts (as defined in the SHORTS register in each peripheral) are not affected by this 16
        // MHz synchronization, and are therefore not delayed.
        let mut ppi_channel0 = ppi.ppi0;
        ppi_channel0.set_event_endpoint(event_timer3_compare0_ppi);
        ppi_channel0.set_task_endpoint(task_uarte1_stoprx_ppi);
        ppi_channel0.enable();

        let mut ppi_channel1 = ppi.ppi1;
        ppi_channel1.set_event_endpoint(event_uarte1_rdxrdy_ppi);
        ppi_channel1.set_task_endpoint(task_timer3_start_ppi);
        ppi_channel1.set_fork_task_endpoint(task_timer3_capture_ppi);
        ppi_channel1.enable();

        // LED for status.
        let blink_led = p0.p0_13.into_push_pull_output(Level::High).degrade();

        // For uarte1 transmissions.
        let sen_tx = p0.p0_04.into_push_pull_output(Level::High).degrade();
        let sen_rx = p0.p0_31.into_floating_input().degrade();

        // Configure the uarte1 peripheral
        uarte1
            .config
            .write(|w| w.parity().excluded().hwfc().disabled());

        // Set baud rate
        uarte1.baudrate.write(|w| w.baudrate().baud9600());

        // Set RX and TX pins for uarte1 perhebial.
        uarte1
            .psel
            .rxd
            .write(|w| unsafe { w.bits(sen_rx.psel_bits()).connect().connected() });
        uarte1
            .psel
            .txd
            .write(|w| unsafe { w.bits(sen_tx.psel_bits()).connect().connected() });

        // Enable uarte peripheral.
        uarte1.enable.write(|w| w.enable().enabled());

        uarte1.intenset.write(|w| w.rxstarted().set_bit());

        // ON(endrx)
        uarte1.intenset.write(|w| w.endrx().set_bit());

        // ON(rx_to)
        uarte1.intenset.write(|w| w.rxto().set_bit());

        // Start the Uarte receiver.
        uarte1.tasks_startrx.write(|w| w.tasks_startrx().set_bit());

        uarte1.intenset.write(|w| w.error().set());

        test_uarte::spawn().ok().unwrap();

        // Initiate periodic status blink, just as a sign of life.
        blink::spawn_after(1.secs()).unwrap();
        //display::spawn_after(3.secs(), mono.now()).unwrap();

        (
            Shared {
                uarte1,
                trace,
                response_actions,
                tx_queue_consumer,
            },
            Local {
                blink_led,
                rx_timer,
                tx_queue_producer,
            },
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

    #[task(priority=6, shared=[trace])]
    fn display(cx: display::Context, instant: fugit::TimerInstantU32<1_000_000>) {
        let next_instant = instant + 3.secs();
        display::spawn_at(next_instant, next_instant).unwrap();
        cx.shared.trace.display_trace();
    }
    pub enum RxAction {
        ATID,
        ATTEST,
        ATMODE,
    }
    impl core::fmt::Debug for RxAction {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            match self {
                RxAction::ATID => write!(f, "ATID  "),
                RxAction::ATTEST => write!(f, "ATTEST"),
                RxAction::ATMODE => write!(f, "ATMODE"),
            }
        }
    }

    #[task(priority=4, shared=[response_actions],)]
    fn test_uarte(cx: test_uarte::Context) {
        // Simulate TX for now.
        // Later this method will receive commands from other tasks
        // such as sensor collection tasks, or network watchdogs.
        let example_tag = heapless::Vec::<u8, 5>::from_slice(b"+ID").unwrap();
        let mut msg = TxPool.request().unwrap();
        let mut msg2 = TxPool.request().unwrap();
        let mut msg3 = TxPool.request().unwrap();
        let mut msg4 = TxPool.request().unwrap();
        let com_len = CommandBuilder::create_query(msg.deref_mut(), true)
            .named("+ID")
            .finish()
            .unwrap()
            .len();
        let com_len2 = CommandBuilder::create_query(msg2.deref_mut(), true)
            .named("+ID")
            .finish()
            .unwrap()
            .len();
        let com_len3 = CommandBuilder::create_query(msg3.deref_mut(), true)
            .named("+ID")
            .finish()
            .unwrap()
            .len();
        let com_len4 = CommandBuilder::create_query(msg4.deref_mut(), true)
            .named("+ID")
            .finish()
            .unwrap()
            .len();
        // Prepare the proper data structures for sending.
        cx.shared
            .response_actions
            .insert(example_tag, RxAction::ATTEST)
            .unwrap();

        tx_ready::spawn(msg, com_len).unwrap();
        tx_ready::spawn(msg2, com_len2).unwrap();
        tx_ready::spawn(msg3, com_len3).unwrap();
        tx_ready::spawn(msg4, com_len4).unwrap();
    }

    /// When a transmission is requested, this function is called with
    /// an already allocated buffer. (See test_uarte as a example)
    #[task(priority=1,capacity=4, local=[tx_queue_producer])]
    fn tx_ready(cx: tx_ready::Context, rdy_buf: Object<TxPool>, len: usize) {
        rprintln!("rdy addr: {:?}", rdy_buf.as_ptr());
        cx.local.tx_queue_producer.enqueue((rdy_buf, len)).unwrap();
        if let Ok(_) = start_tx::spawn() {
            ()
        }
    }

    /// Internal function not to be called explicitly.
    /// This function works through a queue of ready transmission jobs.
    #[task(priority = 4, shared=[uarte1,tx_queue_consumer], local=[])]
    fn start_tx(mut cx: start_tx::Context) {
        cx.shared.uarte1.lock(|uarte1| {
            // Check if there is work to be done...
            if let Some((msg, len)) = cx.shared.tx_queue_consumer.peek() {
                // Reset the events.
                uarte1.events_endtx.reset();
                uarte1.events_txstopped.reset();

                // Set Easy DMA buffer addresses based on the message memory address and size.
                uarte1
                    .txd
                    .ptr
                    .write(|w| unsafe { w.ptr().bits(msg.as_ptr() as u32) });
                uarte1.txd.maxcnt.write(|w| unsafe { w.bits(*len as u32) });

                // ON(endtx)
                uarte1.intenset.write(|w| w.endtx().set());

                // Start transmission
                uarte1.tasks_starttx.write(|w| w.tasks_starttx().set_bit());
            } else {
                // Something here to fill upp the buffer again?
                // Or in a real scenario, try again after awhile?
                test_uarte::spawn_after(150.millis()).unwrap();
            }
        });
    }

    /// Internal function not to be called explicitly.
    /// Cleans up completed transmission jobs and
    /// mark as ready for subsequent jobs (if any).
    #[task(priority=4, shared=[uarte1, tx_queue_consumer])]
    fn finished_tx(cx: finished_tx::Context, bytes_sent: usize) {
        let (drop_buf, len) = cx.shared.tx_queue_consumer.dequeue().unwrap();
        if len == bytes_sent {
            //rprintln!("Ok! len is the same!");
        }
        rprintln!("drop addr: {:?}", drop_buf.as_ptr());
        drop((drop_buf, len));
        if let Ok(_) = start_tx::spawn() {
            ()
        }
    }

    /// Process the currently completed rx_buffer
    /// in the background with lower priority.
    /// This function's content depends on required use-case.
    #[task(priority=4, shared=[response_actions])]
    fn process_rx(cx: process_rx::Context, buf: Object<RxPool>, bytes_to_process: usize) {
        let response = buf.split_at(bytes_to_process).0;
        //rprintln!("Response buf: {:?}", response.utf8_chunks());

        let tag = response.split(|&b| b == b':').nth(0).unwrap_or_default();

        let vec = Vec::<u8, 5>::from_slice(tag).unwrap_or_default();
        if cx.shared.response_actions.contains_key(&vec) {
            // It was an expected response, fire it's mapped action.
            let action = cx.shared.response_actions.get(&vec).unwrap();
            rprintln!("Action to take: {:?}", action);
        } else {
            // Async message from the world outside. In need of further
            // processing to determine what they want.
            rprintln!("Not awaited response is: {:?}", response);
        }
    }

    /// Timeout when the rxd line has been quiet for about 16 ms.
    #[task(binds = TIMER3, priority=6, shared=[uarte1], local=[rx_timer])]
    fn rx_timeout(mut cx: rx_timeout::Context) {
        cx.shared.uarte1.lock(|uarte1| {
            cx.local.rx_timer.events_compare[0].reset();
            uarte1.shorts.reset();
        });
    }

    /// Highest priority; we wouldn't want to miss any precious data would we?
    #[task(binds = UARTE1, priority=6, shared=[trace, uarte1])]
    fn uarte1_interrupt(mut cx: uarte1_interrupt::Context) {
        cx.shared.uarte1.lock(|uarte| {
            let trace = cx.shared.trace;
            // BEGIN STATE TIMEOUT
            if uarte.events_rxto.read().events_rxto().bit() {
                uarte.events_endrx.reset();
                uarte.events_rxto.reset();

                trace.log(TraceState::RxTo, uarte.rxd.amount.read().bits() as usize);

                uarte.shorts.write(|w| w.endrx_startrx().enabled());

                // TODO: consider spawning "process_rxbuffer" here aswell in
                // edge-cases where we have almost full buffer yet bytes on
                // the rdx line waiting for a flush_rx operation.

                // FLUSH RXD into BUFFER.
                // Up to 4 bytes are received after END_RX before TIMEOUT is triggered.
                uarte.tasks_flushrx.write(|w| w.tasks_flushrx().set_bit());
            }
            // BEGIN STATE ENDRX
            else if uarte.events_endrx.read().events_endrx().bit() {
                uarte.events_endrx.reset();

                // How many bytes did we get?
                let bytes_that_require_processing = uarte.rxd.amount.read().bits();
                trace.log(TraceState::Endrx, bytes_that_require_processing as usize);

                // Start preparing next buffer, the ptr register is
                // double buffered per datasheet.
                let top_free_block = RxPool.request().unwrap();
                uarte
                    .rxd
                    .maxcnt
                    .write(|w| unsafe { w.bits(top_free_block.len() as u32) });
                uarte
                    .rxd
                    .ptr
                    .write(|w| unsafe { w.ptr().bits(top_free_block.as_ptr() as u32) });

                // Start processing buffer in background with lower priority.
                match process_rx::spawn(top_free_block, bytes_that_require_processing as usize) {
                    Ok(_) => (),
                    Err(err) => {
                        rprintln!("Error: Process_rxbuffer task already pending! {:?}", err)
                    }
                }
            }
            // BEGIN STATE RXSTARTED
            else if uarte.events_rxstarted.read().events_rxstarted().bit() {
                uarte.events_rxstarted.reset();
                uarte.events_endrx.reset();

                trace.log(TraceState::Rxstarted, 0);
            }
            // BEGIN STATE ENDTX
            if uarte.events_endtx.read().events_endtx().bit() {
                uarte.events_endtx.reset();
                uarte.tasks_stoptx.write(|w| w.tasks_stoptx().set_bit());

                // For safety? I don't know how strictly required this is.
                // Check Product Specification whether this is
                // guaranteed in hardware already or not.
                while uarte.events_txstopped.read().bits() == 0 {}
                let bytes_transmitted = uarte.txd.amount.read().bits() as usize;
                trace.log(TraceState::Endtx, bytes_transmitted);

                finished_tx::spawn(bytes_transmitted).ok().unwrap();
            }
            // BEGIN STATE ERROR
            if uarte.events_error.read().events_error().bit() {
                // Undefined?
                rprintln!("error: {}", uarte.errorsrc.read().bits());
                uarte.events_error.reset();
            }
        });
    }

    /// Lowest priority.
    #[task(priority = 1, local = [blink_led])]
    fn blink(cx: blink::Context) {
        let led = cx.local.blink_led;
        if led.is_set_low().unwrap() {
            led.set_high().ok();
        } else {
            led.set_low().ok();
        }
        blink::spawn_after(fugit::TimerDurationU32::secs(1)).unwrap();
    }
}
