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
    use heapless::{
        object_pool,
        pool::object::{Object, ObjectBlock},
        String,
    };
    use nrf52840_hal::{
        gpio::{p0, Level, Output, Pin, PushPull}, gpiote::Gpiote, 
        pac::{TIMER1, TIMER3, TIMER4, UARTE1}, 
        ppi::{self, ConfigurablePpi, Ppi}, 
        timer::Timer
    };
    use rtt_target::{rprintln, rtt_init_print};

    #[monotonic(binds = TIMER1, default = true)]
    type MyMono = MonoTimer<TIMER1>;

    const MSG_BUF_SIZE: usize = 256;
    const POOL_CAPACITY: usize = 4;

    object_pool!(RxPool: [u8; MSG_BUF_SIZE]);
    object_pool!(TxPool: [u8; MSG_BUF_SIZE]);

    pub enum State {
        Config(Config),
        Run,
        Error,
    }
    pub enum Config {
        ConfigAddress,
        ConfigTest,
        ConfigSleep,
        ConfigMode,
        ConfigReset,
    }

    #[derive(Default)]
    pub struct LoraParams {
        device_address: String<11>,
        _frequency: u32,
    }
    #[shared]
    struct Shared {
        state: State,
        uarte1: UARTE1,
        #[lock_free]
        trace: Trace<TIMER4>,
        gpiote: Gpiote,
        lora_params: LoraParams,
    }

    #[local]
    struct Local {
        config: Config,
        uarte_timer: TIMER3,
        blink_led: Pin<Output<PushPull>>,
        rx_led: Pin<Output<PushPull>>,
        tx_led: Pin<Output<PushPull>>,
    }

    #[init(local = [])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        rtt_init_print!();

        let mono = MonoTimer::new(cx.device.TIMER1);
        let trace_timer = Timer::into_periodic(Timer::new(cx.device.TIMER4));
        let trace = Trace::new(trace_timer);

        let rx_blocks: &'static mut [ObjectBlock<[u8; MSG_BUF_SIZE]>] = {
            const BLOCK: ObjectBlock<[u8; MSG_BUF_SIZE]> = ObjectBlock::new([0; MSG_BUF_SIZE]); // <=
            static mut RX_BLOCKS: [ObjectBlock<[u8; MSG_BUF_SIZE]>; POOL_CAPACITY] = [BLOCK; POOL_CAPACITY];
            #[allow(static_mut_refs)]
            unsafe { &mut RX_BLOCKS }
        };
        let tx_blocks: &'static mut [ObjectBlock<[u8; MSG_BUF_SIZE]>] = {
            const BLOCK: ObjectBlock<[u8; MSG_BUF_SIZE]> = ObjectBlock::new([0; MSG_BUF_SIZE]); // <=
            static mut TX_BLOCKS: [ObjectBlock<[u8; MSG_BUF_SIZE]>; POOL_CAPACITY] = [BLOCK; POOL_CAPACITY];
            #[allow(static_mut_refs)]
            unsafe { &mut TX_BLOCKS }
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
        let uarte_timer = cx.device.TIMER3;

        uarte_timer.bitmode.write(|w| w.bitmode()._16bit());
        uarte_timer.prescaler.write(|w| unsafe { w.bits(2) });
        uarte_timer.cc[0].write(|w| unsafe { w.bits(u32::max_value()) });
        uarte_timer.shorts.write(|w| w.compare0_stop().set_bit());
        uarte_timer.intenset.write(|w| w.compare0().set());

        let blink_led = p0.p0_13.into_push_pull_output(Level::High).degrade();
        let rx_led = p0.p0_15.into_push_pull_output(Level::High).degrade();
        let tx_led = p0.p0_16.into_push_pull_output(Level::High).degrade();
        let btn1 = p0.p0_11.into_pullup_input().degrade();

        // Configure GPIOTE.
        gpiote.channel0().input_pin(&btn1).hi_to_lo().enable_interrupt();

        // Task and event handles.
        let event_timer3_compare0_ppi = &uarte_timer.events_compare[0];
        let event_uarte1_rdxrdy_ppi = &uarte1.events_rxdrdy;
        let task_uarte1_stoprx_ppi = &uarte1.tasks_stoprx;
        let task_timer3_start_ppi = &uarte_timer.tasks_start;
        let task_timer3_capture_ppi = &uarte_timer.tasks_capture[0];

        let mut ppi_channel0 = ppi.ppi0;
        ppi_channel0.set_event_endpoint(event_timer3_compare0_ppi);
        ppi_channel0.set_task_endpoint(task_uarte1_stoprx_ppi);
        ppi_channel0.enable();

        let mut ppi_channel1 = ppi.ppi1;
        ppi_channel1.set_event_endpoint(event_uarte1_rdxrdy_ppi);
        ppi_channel1.set_task_endpoint(task_timer3_start_ppi);
        ppi_channel1.set_fork_task_endpoint(task_timer3_capture_ppi);
        ppi_channel1.enable();

        let uarte1_tx = p0.p0_04.into_push_pull_output(Level::High).degrade();
        let uarte1_rx = p0.p0_31.into_floating_input().degrade();

        uarte1
            .config
            .write(|w| w.parity().excluded().hwfc().disabled());
        uarte1.baudrate.write(|w| w.baudrate().baud9600());
        uarte1
            .psel
            .rxd
            .write(|w| unsafe { w.bits(uarte1_rx.psel_bits()).connect().connected() });
        uarte1
            .psel
            .txd
            .write(|w| unsafe { w.bits(uarte1_tx.psel_bits()).connect().connected() });

        uarte1.enable.write(|w| w.enable().enabled());
        uarte1.intenset.write(|w| w.rxstarted().set_bit());
        uarte1.intenset.write(|w| w.endrx().set_bit());
        uarte1.intenset.write(|w| w.rxto().set_bit());
        uarte1.intenset.write(|w| w.error().set());
        uarte1.tasks_startrx.write(|w| w.tasks_startrx().set_bit());

        let config = Config::ConfigReset;
        let state = State::Config(Config::ConfigReset);
        let lora_params: LoraParams = Default::default();

        blink::spawn().unwrap();
        (
            Shared { 
                state, 
                uarte1, 
                trace,
                gpiote,
                lora_params,
            },
            Local {
                config,
                uarte_timer,
                blink_led,
                rx_led,
                tx_led,
            },
            init::Monotonics(mono),
        )
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            asm::wfi();
        }
    }

    #[task(priority=5, local=[], shared=[lora_params])]
    fn network_handler(_cx: network_handler::Context, _net_pkt_in: Option<Object<RxPool>>) {
    }

    #[task(priority=6, local=[config], shared=[state, lora_params, trace])]
    fn config(mut cx: config::Context, config_msg: Object<RxPool>) {
        let trace = cx.shared.trace;
        match cx.local.config {
            Config::ConfigAddress => {
                trace.log(TraceState::ConfigAddress, 0);
                if let Ok((dev_addr,)) = CommandParser::parse(config_msg.deref())
                    .expect_identifier(b"+ID: DevAddr,")
                    .expect_raw_string()
                    .finish()
                {
                    // Save the address for sending purposes 
                    cx.shared.lora_params.lock(|lr| {
                        lr.device_address = String::try_from(dev_addr).unwrap();

                        let mut msg = TxPool.request().unwrap();
                        CommandBuilder::create_query(msg.deref_mut(), true)
                            .named("+MODE")
                            .finish()
                            .unwrap();
                        *cx.local.config = Config::ConfigMode;
                        start_tx::spawn(msg).ok().unwrap();
                    });
                } else {
                    // Error, could not parse ID response. Maybe try again after a while?
                    trace.log(TraceState::ConfigError, 0);
                    *cx.local.config = Config::ConfigReset;
                    if let Ok(_) = config::spawn_after(100.millis(), RxPool.request().unwrap()) {()};
                }
            },
            Config::ConfigTest => {
                trace.log(TraceState::ConfigTest, 0);
                let (test_mode, _test_params) = {
                    let mut segments= config_msg.split_inclusive(|&b| b == b'\n');
                    (segments.next().unwrap_or_default(), segments.next().unwrap_or_default())
                };
                if let Ok((net_mode,)) = CommandParser::parse(test_mode)
                    .expect_identifier(b"+TEST:")
                    .expect_raw_string()
                    .finish()
                {
                    match net_mode {
                        "RXLRPKT" => {
                            // Here we are actively listening for incoming packages.
                            // The network handler will take over.
                            cx.shared.state.lock(|state| {
                                trace.log(TraceState::Run, 0);
                                // Maybe a good time?
                                *state = State::Run;
                            });

                            // I don't know where else to invoke this.
                            display::spawn().unwrap();
                        },
                        _ => {
                            // collect parameters then maybe set in reciever mode?
                            // Only go out of reception while sensor data is ready.
                            *cx.local.config = Config::ConfigTest;
                            let mut msg = TxPool.request().unwrap();
                            CommandBuilder::create_set(msg.deref_mut(), true)
                                .named(b"+TEST")
                                .with_string_parameter(b"RXLRPKT")
                                .finish()
                                .unwrap();
                            start_tx::spawn(msg).ok().unwrap();
                        },
                    }
                } else {
                    // Error, could not parse test response. Maybe try again after a while?
                    *cx.local.config = Config::ConfigReset;
                    if let Ok(_) = config::spawn_after(100.millis(), RxPool.request().unwrap()) {()};
                }
            },
            Config::ConfigSleep => {
                trace.log(TraceState::ConfigSleep, 0);
                if let Ok(sleep_status) = CommandParser::parse(config_msg.deref())
                    .expect_identifier(b"+LOWPOWER:")
                    .expect_raw_string()
                    .finish()
                {
                    match sleep_status {
                        ("SLEEP",) => {

                        },
                        ("WAKEUP",) => {

                        },
                        _ => {

                        },
                    }
                } else {
                    // Error, could not parse sleep response. Maybe try again after a while?
                }
            },
            Config::ConfigMode => {
                trace.log(TraceState::ConfigMode, 0);
                if let Ok(mode) = CommandParser::parse(config_msg.deref())
                    .expect_identifier(b"+MODE:")
                    .expect_raw_string()
                    .finish()
                {
                    match mode {
                        ("TEST",) => {
                            *cx.local.config = Config::ConfigTest;
                            let mut msg = TxPool.request().unwrap();
                            CommandBuilder::create_query(msg.deref_mut(), true)
                                .named(b"+TEST")
                                .finish()
                                .unwrap();
                            start_tx::spawn(msg).ok().unwrap();

                        },
                        _ => {
                            *cx.local.config = Config::ConfigMode;
                            let mut msg = TxPool.request().unwrap();
                            CommandBuilder::create_set(msg.deref_mut(), true)
                                .named(b"+MODE")
                                .with_string_parameter(b"TEST")
                                .finish()
                                .unwrap();
                            start_tx::spawn(msg).ok().unwrap();
                        },
                    }
                } else {
                    // Error, could not parse mode response. Maybe try again after a while?
                    // TODO, for some reason this works instead of ConfigReset, I dont know why yet
                    trace.log(TraceState::ConfigError, 0);
                    *cx.local.config = Config::ConfigAddress;
                    if let Ok(_) = config::spawn_after(100.millis(), RxPool.request().unwrap()) {()};
                }
            },
            Config::ConfigReset => {
                trace.log(TraceState::ConfigReset, 0);
                let mut reset_msg = TxPool.request().unwrap();
                CommandBuilder::create_execute(reset_msg.deref_mut(), true)
                    .named("+RESET")
                    .finish()
                    .unwrap();
                start_tx::spawn(reset_msg).ok().unwrap();
                let mut msg = TxPool.request().unwrap();
                CommandBuilder::create_query(msg.deref_mut(), true)
                    .named("+ID")
                    .finish()
                    .unwrap();
                *cx.local.config = Config::ConfigAddress;
                start_tx::spawn(msg).ok().unwrap();
            },
        }
    }

    #[task(priority=6, shared=[trace])]
    fn display(cx: display::Context) {
        cx.shared.trace.display_trace();
    }

    /// Start tx from the tag_msg_map queue, if there is a transmission pending.
    /// Capacity less than TxPool because in an attempt to avoid a currently
    /// used TxPool allocation to be reused early.
    #[task(priority = 4, capacity=3, shared=[uarte1], local=[])]
    fn start_tx(mut cx: start_tx::Context, tx_buf: Object<TxPool>) {
        cx.shared.uarte1.lock(|uarte1| {
            uarte1.events_endtx.reset();
            uarte1.events_txstopped.reset();
            uarte1
                .txd
                .ptr
                .write(|w| unsafe { w.ptr().bits(tx_buf.as_ptr() as u32) });
            uarte1
                .txd
                .maxcnt
                .write(|w| unsafe { w.bits(tx_buf.len() as u32) });
            uarte1.intenset.write(|w| w.endtx().set());
            uarte1.tasks_starttx.write(|w| w.tasks_starttx().set_bit());
        });
    }

    /// Process the currently completed rx_buffer
    #[task(priority=5, capacity=3,shared=[state])]
    fn process_rx(mut cx: process_rx::Context, rx_buf: Object<RxPool>, bytes_to_process: usize) {
        cx.shared.state.lock(|state| {
            let _response = rx_buf.split_at(bytes_to_process).0;
            match state {
                State::Config(_) => {
                    if let Ok(_) = config::spawn(rx_buf) { () }
                },
                State::Run => {
                    // TODO, just a start; probably want more processing before spawning.
                    if let Ok(_) = network_handler::spawn(Some(rx_buf)) { () }
                },
                State::Error => {

                },
            }
        });
    }

    /// Timeout when the rxd line has been quiet for about 16 ms.
    #[task(binds = TIMER3, priority=6, shared=[uarte1], local=[uarte_timer])]
    fn rx_timeout(mut cx: rx_timeout::Context) {
        cx.shared.uarte1.lock(|uarte1| {
            cx.local.uarte_timer.events_compare[0].reset();
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
                uarte.tasks_flushrx.write(|w| w.tasks_flushrx().set_bit());
            }
            // BEGIN STATE ENDRX
            else if uarte.events_endrx.read().events_endrx().bit() {
                uarte.events_endrx.reset();
                let bytes_that_require_processing = uarte.rxd.amount.read().bits();
                trace.log(TraceState::Endrx, bytes_that_require_processing as usize);
                let top_free_block = RxPool.request().unwrap();
                uarte
                    .rxd
                    .maxcnt
                    .write(|w| unsafe { w.bits(top_free_block.len() as u32) });
                uarte
                    .rxd
                    .ptr
                    .write(|w| unsafe { w.ptr().bits(top_free_block.as_ptr() as u32) });
                if let Ok(_) =
                    process_rx::spawn(top_free_block, bytes_that_require_processing as usize)
                {
                    if let Ok(_) = status_led::spawn(StatusLed::RxLed, None){()}
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
                let bytes_transmitted = uarte.txd.amount.read().bits();
                trace.log(TraceState::Endtx, bytes_transmitted as usize);
                if let Ok(_) = status_led::spawn(StatusLed::TxLed, None){()}
            }
            // BEGIN STATE ERROR
            if uarte.events_error.read().events_error().bit() {
                uarte.events_error.reset();
                rprintln!("error: {}", uarte.errorsrc.read().bits());
            }
        });
    }

    enum StatusLed {
        RxLed,
        TxLed,
    }

    /// Lowest priority.
    #[task(priority = 1, capacity = 10, local = [rx_led, tx_led])]
    fn status_led(cx: status_led::Context, led: StatusLed, is_high: Option<bool>) {
        let sel_led = cx.local.rx_led;
        if is_high.unwrap_or(false) {
            sel_led.set_high().ok();
        } else {
            if let Ok(_) = status_led::spawn_after(fugit::TimerDurationU32::millis(50), led, Some(true)) {()}
            sel_led.set_low().ok();
        }
    }
    /// Lowest priority.
    #[task(priority = 1, local = [blink_led])]
    fn blink(cx: blink::Context){
        let led = cx.local.blink_led;
        if led.is_set_low().unwrap() {
            led.set_high().ok();
        } else {
            led.set_low().ok();
        }
        blink::spawn_after(fugit::TimerDurationU32::secs(1)).unwrap();
    }

    /// Mock "sensor", just to send packages.
    #[task(binds = GPIOTE, local=[], shared = [gpiote, state])]
    fn gpiote_interrupt(mut cx: gpiote_interrupt::Context){
        cx.shared.gpiote.lock(|gpiote| {
            if gpiote.channel0().is_event_triggered() {
                if let Ok(_) = status_led::spawn(StatusLed::RxLed, None){()}
                cx.shared.state.lock(|state| {
                    *state = State::Config(Config::ConfigTest);
                    let mut msg = TxPool.request().unwrap();
                    CommandBuilder::create_set(msg.deref_mut(), true)
                        .named("+TEST")
                        .with_string_parameter(b"TXLRPKT")
                        .with_string_parameter(b"AABBCCDDEEFF")
                        .finish()
                        .unwrap();
                    start_tx::spawn(msg).unwrap();

                    //TODO: handle TX DOne case!
                });
            }
            gpiote.reset_events();
        });
    }
}
