#![no_main]
#![no_std]

use panic_rtt_target as _;
#[rtic::app(device = nrf52840_hal::pac, dispatchers = [SWI0_EGU0, SWI1_EGU1, SWI2_EGU2, SWI3_EGU3])]
mod app {
    use core::ops::{Deref, DerefMut};

    use at_commands::{builder::CommandBuilder, parser::CommandParser};
    use cortex_m::asm;
    use embedded_hal::digital::{OutputPin, StatefulOutputPin};
    use embedded_rs_lora::util::{
        mono::{ExtU32, MonoTimer},
        trace::{Trace, TraceState},
    };
    use heapless as new_heapless;
    use hex::ToHex;
    use new_heapless::{
        object_pool,
        pool::object::{Object, ObjectBlock},
        spsc::{Consumer, Producer, Queue},
        FnvIndexMap, String, Vec,
    };
    use nrf52840_hal::{
        gpio::{p0, Level, Output, Pin, PushPull},
        gpiote::Gpiote,
        pac::{TIMER1, TIMER3, TIMER4, UARTE1},
        ppi::{self, ConfigurablePpi, Ppi},
        timer::Timer,
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
        gpiote: Gpiote,
        #[lock_free]
        trace: Trace<TIMER4>,
        #[lock_free]
        tx_queue_consumer: Consumer<'static, (Object<TxPool>, usize), POOL_CAPACITY>,
        state: State,
        #[lock_free]
        test: Test,
        lora_params: LoraParams,
    }

    #[local]
    struct Local {
        blink_led: Pin<Output<PushPull>>,
        status_led: Pin<Output<PushPull>>,
        rx_timer: TIMER3,
        tx_queue_producer: Producer<'static, (Object<TxPool>, usize), POOL_CAPACITY>,
        config: Config,
        net_pkt_map: FnvIndexMap<Vec<u8, 11>, usize, 16>,
    }
    pub enum State {
        Config,
        Run,
        Error,
    }
    pub enum Config {
        ConfigAddress,
        ConfigMode(Mode),
        ConfigReset,
        ConfigSleep,
    }
    #[derive(Debug, Default)]
    pub struct LoraParams {
        address: Vec<u8, 11>,
        _freq: u32,
    }
    pub enum Mode {
        ModeTest(Test),
        ModeLWABP,
    }
    // TODO: these will not be used in config eventually.
    // They are to be handled within network manager since
    // this is a strictly lora issue to handle.
    pub enum Test {
        TestStop,
        TestRx,
        TestTx,
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
        let lora_params = LoraParams::default();
        let net_pkt_map = FnvIndexMap::default();
        let state = State::Config;
        let config = Config::ConfigAddress;
        let test = Test::TestRx;

        let (tx_queue_producer, tx_queue_consumer) = cx.local.tx_queue.split();

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
        let gpiote = Gpiote::new(cx.device.GPIOTE);
        let ppi = ppi::Parts::new(cx.device.PPI);
        let uarte1 = cx.device.UARTE1;
        let rx_timer = cx.device.TIMER3;

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
        rx_timer.cc[0].write(|w| unsafe { w.bits(u32::MAX) });
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
        let status_led = p0.p0_16.into_push_pull_output(Level::High).degrade();
        let btn1 = p0.p0_11.into_pullup_input().degrade();

        // Configure GPIOTE.
        gpiote
            .channel0()
            .input_pin(&btn1)
            .lo_to_hi()
            .enable_interrupt();

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

        // ON(rxstarted)
        uarte1.intenset.write(|w| w.rxstarted().set_bit());

        // ON(endrx)
        uarte1.intenset.write(|w| w.endrx().set_bit());

        // ON(rx_to)
        uarte1.intenset.write(|w| w.rxto().set_bit());

        // ON(error)
        uarte1.intenset.write(|w| w.error().set());

        // Start the Uarte receiver.
        uarte1.tasks_startrx.write(|w| w.tasks_startrx().set_bit());

        blink::spawn_after(1.secs()).unwrap();
        //display::spawn_after(3.secs(), mono.now()).unwrap();

        (
            Shared {
                uarte1,
                gpiote,
                trace,
                tx_queue_consumer,
                state,
                test,
                lora_params,
            },
            Local {
                blink_led,
                status_led,
                rx_timer,
                tx_queue_producer,
                config,
                net_pkt_map,
            },
            init::Monotonics(mono),
        )
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            //rprintln!("sleeping...");
            asm::wfi();
        }
    }

    #[task(priority=6, shared=[trace])]
    fn display(cx: display::Context) {
        cx.shared.trace.display_trace();
    }

    #[task(priority=5, local = [config], shared = [state, lora_params])]
    fn config(cx: config::Context, config_msg: Object<RxPool>, bytes_sent: usize) {
        //rprintln!("config_msg: {:?}", config_msg.split_at(bytes_sent).0.utf8_chunks());
        let (command, expected, next_command, on_success): (
            &[u8],
            &[u8],
            &[u8],
            fn(&mut Config, &mut State, &str, &mut LoraParams),
        ) = match *cx.local.config {
            Config::ConfigAddress => (
                b"+ID",
                b"+ID: DevAddr,",
                b"+MODE=TEST",
                |next_conf, _, param, lora_params| {
                    *next_conf = Config::ConfigMode(Mode::ModeTest(Test::TestStop));
                    // In AT format it is expected to be 11 bytes of data.
                    lora_params.address = Vec::<u8, 11>::from_slice(param.as_bytes()).unwrap();
                },
            ),
            Config::ConfigMode(Mode::ModeLWABP) => (
                // Not used currently.
                b"+MODE",
                b"+MODE:",
                b"+ID",
                |next_conf, _, _, _| {
                    *next_conf = Config::ConfigAddress;
                },
            ),
            Config::ConfigMode(Mode::ModeTest(_)) => (
                b"+MODE=TEST",
                b"+MODE: TEST",
                b"+TEST=RXLRPKT",
                |next_conf, state_transition, _, _| {
                    *next_conf = Config::ConfigMode(Mode::ModeTest(Test::TestRx));
                    *state_transition = State::Run;
                },
            ),
            Config::ConfigReset => (b"+RESET", b"+RESET: OK", b"+ID", |next_conf, _, _, _| {
                *next_conf = Config::ConfigAddress;
            }),
            Config::ConfigSleep => (b"+MODE", b"+MODE:", b"+ID", |next_conf, _, _, _| {
                *next_conf = Config::ConfigAddress;
            }),
        };

        fn send_command(command: &[u8]) {
            let mut tx_buf = TxPool.request().unwrap();
            let command_len = CommandBuilder::create_execute(tx_buf.deref_mut(), true)
                .named(command)
                .finish()
                .unwrap()
                .len();
            // The LoRa module seem to not be able to handle faster message transactions.
            start_tx::spawn_after(150.millis(), tx_buf, command_len).unwrap();
        }

        // Attempt to parse the incoming message
        if let Ok((param,)) = CommandParser::parse(config_msg.deref())
            .expect_identifier(expected)
            .expect_raw_string()
            .finish()
        {
            // Successfully received the expected message
            (cx.shared.state, cx.shared.lora_params).lock(|state, lora_params| {
                on_success(cx.local.config, state, param, lora_params);
            });
            send_command(next_command);
        } else {
            // Not successful in parsing the expected message
            send_command(command);
        }
    }

    #[task(priority=4, local=[net_pkt_map], shared=[test, lora_params])]
    fn network_handler(mut cx: network_handler::Context, pkt_in: Object<RxPool>, pkt_len: usize) {
        match cx.shared.test {
            Test::TestStop => {
                rprintln!("TestStop");
            }
            Test::TestRx => {
                if let Ok((pkt_len, pkt_rssi, pkt_snr, pkt_data)) =
                    CommandParser::parse(pkt_in.split_at(pkt_len).0)
                        .expect_identifier(b"+TEST: LEN:")
                        .expect_int_parameter()
                        .expect_identifier(b"RSSI:")
                        .expect_int_parameter()
                        .expect_identifier(b"SNR:")
                        .expect_int_parameter()
                        .expect_identifier(b"\r\n+TEST: RX")
                        .expect_raw_string()
                        .finish()
                {
                    // Packet successfully parsed.
                    rprintln!("pkt_metadata: {:?}", (pkt_len, pkt_rssi, pkt_snr));
                    for (index, part) in pkt_data.as_bytes().split(|&b| b == b';').enumerate() {
                        match index {
                            0 => {
                                rprintln!("pkt_addr: {:?}", part.utf8_chunks());
                                cx.shared.lora_params.lock(|params| {
                                    rprintln!("lora_addr: {:?}", params.address.utf8_chunks());

                                    // Attempt to create the Vec<u8> from the address
                                    if let Ok(pkt_addr_vec) = Vec::<u8, 11>::from_slice(part) {
                                        // Check for packet in hashmap
                                        if let Some(pkt_cnt) =
                                            cx.local.net_pkt_map.get_mut(&pkt_addr_vec)
                                        {
                                            rprintln!("packet in hashmap: {:?}", pkt_cnt);
                                            *pkt_cnt += 1;
                                        } else {
                                            rprintln!("packet not in hashmap");
                                            cx.local.net_pkt_map.insert(pkt_addr_vec, 0).unwrap();
                                        }
                                    } else {
                                        rprintln!("Failed to create Vec from address: {:?}", part);
                                    }
                                });
                            }
                            1 => rprintln!("pkt_cnt: {:?}", part),
                            2 => rprintln!("pkt_data: {:?}", part),
                            _ => rprintln!("pkt_other: {:?}", part),
                        }
                    }
                } else {
                    // Failed to parse LoRa packet.
                    rprintln!("error parsing network packet");
                }
            }
            Test::TestTx => {
                if CommandParser::parse(pkt_in.split(|&b| b == b'\n').nth(1).unwrap())
                    .expect_identifier(b"+TEST: TX DONE")
                    .finish()
                    .is_ok()
                {
                    rprintln!("tx sent successfully!");

                    *cx.shared.test = Test::TestRx;
                } else {
                    rprintln!("tx sent not parsed correctly: {:?}", pkt_in.utf8_chunks());
                }
            }
        }
    }
    #[task(priority=4, shared=[test, lora_params])]
    fn send_packet(mut cx: send_packet::Context, pkt_msg: Vec<u8, 32>) {
        let mut tx_buf = TxPool.request().unwrap();
        cx.shared.lora_params.lock(|lr| {
            let pkt_len = CommandBuilder::create_set(tx_buf.deref_mut(), true)
                .named(b"+TEST")
                .with_raw_parameter(b"TXLRPKT")
                .with_string_parameter(pkt_msg.encode_hex_upper::<String<64>>())
                .finish()
                .unwrap()
                .len();
            *cx.shared.test = Test::TestTx;
            start_tx::spawn(tx_buf, pkt_len).unwrap();
        });
    }
    /// When a transmission is requested, this function is called with
    /// an already allocated buffer. (See test_uarte as a example)
    //#[task(priority=4,capacity=3, local=[])]
    //fn tx_ready(cx: tx_ready::Context, rdy_buf: Object<TxPool>, len: usize) {
    //    rprintln!("tx_ready addr: {:?}", rdy_buf.as_ptr());
    //    cx.local.tx_queue_producer.enqueue((rdy_buf, len)).unwrap();
    //    rprintln!(
    //        "tx_ready queue len after enqueue: {:?}",
    //        cx.local.tx_queue_producer.len()
    //    );
    //    if start_tx::spawn(rdy_buf, len).is_ok() {}
    //}

    /// Internal function not to be called explicitly.
    /// This function works through a queue of ready transmission jobs.
    #[task(priority = 4, capacity=3, shared=[uarte1], local=[tx_queue_producer])]
    fn start_tx(mut cx: start_tx::Context, tx_buf: Object<TxPool>, tx_len: usize) {
        cx.shared.uarte1.lock(|uarte1| {
            // Check if there is work to be done...
            rprintln!(
                "tx_ready queue len before enqueue: {:?}",
                cx.local.tx_queue_producer.len()
            );
            rprintln!("tx_buf enqueue addr: {:?}", tx_buf.as_ptr());
            rprintln!("tx_buf: {:?}", tx_buf.split_at(tx_len).0.utf8_chunks());
            if cx.local.tx_queue_producer.ready() {
                // There was space to enqueue.
                // Reset the events.
                uarte1.events_endtx.reset();
                uarte1.events_txstopped.reset();

                // Set Easy DMA buffer addresses based on the message memory address and size.
                uarte1
                    .txd
                    .ptr
                    .write(|w| unsafe { w.ptr().bits(tx_buf.as_ptr() as u32) });
                uarte1
                    .txd
                    .maxcnt
                    .write(|w| unsafe { w.bits(tx_len as u32) });

                cx.local
                    .tx_queue_producer
                    .enqueue((tx_buf, tx_len))
                    .unwrap();

                // ON(endtx)
                uarte1.intenset.write(|w| w.endtx().set());

                // Start transmission
                uarte1.tasks_starttx.write(|w| w.tasks_starttx().set_bit());
                if display::spawn_after(fugit::TimerDurationU32::secs(1)).is_ok() {}
            } else {
                // Queue was full, backfilled with sending messages,
                rprintln!("error: tx queue is full");
            }
        });
    }

    /// Internal function not to be called explicitly.
    /// Cleans up completed transmission jobs and
    /// mark as ready for subsequent jobs (if any).
    ///
    /// TODO BUG: if multiple subsequent transmissions occur finished_tx would only be called once
    /// instead of the expected number of transmissions amount (most likely due to endtx event only
    /// triggering when all transmissions are completed; done in nordic hardware).
    /// Meaning tx_queue unintentionally continues accumilating over time until it's full.
    #[task(priority=4, shared=[uarte1, tx_queue_consumer])]
    fn finished_tx(cx: finished_tx::Context, bytes_sent: usize) {
        let (drop_buf, len) = cx.shared.tx_queue_consumer.dequeue().unwrap();
        if len == bytes_sent {
            //rprintln!("Ok! len is the same!");
        }
        rprintln!("drop addr: {:?}", drop_buf.as_ptr());
        rprintln!(
            "tx_ready queue len after dequeue: {:?}",
            cx.shared.tx_queue_consumer.len()
        );
        drop((drop_buf, len));
    }

    /// Process the currently completed rx_buffer
    /// in the background with lower priority.
    /// This function's content depends on required use-case.
    #[task(priority=5, shared=[state])]
    fn process_rx(mut cx: process_rx::Context, buf: Object<RxPool>, bytes_to_process: usize) {
        //rprintln!("process_rx: {:?}", buf.split_at(bytes_to_process).0.utf8_chunks());
        cx.shared.state.lock(|state| match state {
            State::Config => {
                //rprint!("config_state");
                if config::spawn(buf, bytes_to_process).is_ok() {}
            }
            State::Run => {
                //rprint!("run_state");
                if network_handler::spawn(buf, bytes_to_process).is_ok() {}
            }
            State::Error => {}
        });
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
                // double buffered per Nordic Product Specification.
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
        blink::spawn_after(fugit::TimerDurationU32::secs(4)).unwrap();
    }
    /// Lowest priority.
    #[task(priority = 1, capacity = 10, local = [status_led])]
    fn status_led(cx: status_led::Context, is_high: Option<bool>) {
        if is_high.unwrap_or(false) {
            cx.local.status_led.set_high().ok();
        } else {
            let _ =
                status_led::spawn_after(fugit::TimerDurationU32::millis(50), Some(true)).is_ok();
            cx.local.status_led.set_low().ok();
        }
    }

    /// Mock "sensor", just to send packages.
    #[task(priority=7, binds = GPIOTE, local=[], shared = [gpiote, state])]
    fn gpiote_interrupt(mut cx: gpiote_interrupt::Context) {
        cx.shared.gpiote.lock(|gpiote| {
            if gpiote.channel0().is_event_triggered() {
                gpiote.channel0().event().reset();
                status_led::spawn(None).unwrap();
                if send_packet::spawn(Vec::from_slice(b"hello").unwrap()).is_ok() {}
            }
            gpiote.reset_events();
        });
    }
}
