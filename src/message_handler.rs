use core::ops::Deref;

use heapless::{
    object_pool,
    pool::object::{Object, ObjectBlock},
};
use nrf52840_hal::{
    gpio::{Floating, Input, Output, Pin, PushPull},
    pac::{uarte0, UARTE0, UARTE1},
    ppi::ConfigurablePpi,
    timer,
};

pub trait Instance: Deref<Target = uarte0::RegisterBlock> {
    fn ptr() -> *const uarte0::RegisterBlock;
}
impl Instance for UARTE0 {
    fn ptr() -> *const uarte0::RegisterBlock {
        UARTE0::ptr()
    }
}
impl Instance for UARTE1 {
    fn ptr() -> *const uarte0::RegisterBlock {
        UARTE1::ptr()
    }
}
impl<U, T, P1, P2> Deref for MessageHandler<U, T, P1, P2>
where
    U: Instance,
    T: timer::Instance,
    P1: ConfigurablePpi,
    P2: ConfigurablePpi,
{
    type Target = U;

    fn deref(&self) -> &Self::Target {
        &self.uarte
    }
}
const MSG_BUF_SIZE: usize = 256;
const POOL_CAPACITY: usize = 4;

object_pool!(RxPool: [u8; MSG_BUF_SIZE]);
object_pool!(TxPool: [u8; MSG_BUF_SIZE]);
struct MessageHandler<U, T, P1, P2>
where
    U: Instance,
    T: timer::Instance,
    P1: ConfigurablePpi,
    P2: ConfigurablePpi,
{
    uarte: U,
    timer: T,
    ppi1: P1,
    ppi2: P2,
    rxp: Pin<Input<Floating>>,
    txp: Pin<Output<PushPull>>,
}

impl<U, T, P1, P2> MessageHandler<U, T, P1, P2>
where
    U: Instance,
    T: timer::Instance,
    P1: ConfigurablePpi,
    P2: ConfigurablePpi,
{
    fn new(
        uarte: U,
        timer: T,
        mut ppi1: P1,
        mut ppi2: P2,
        rxp: Pin<Input<Floating>>,
        txp: Pin<Output<PushPull>>,
    ) -> Self {
        if uarte.enable.read().bits() != 0 {
            uarte.tasks_stoptx.write(|w| unsafe { w.bits(1) });
            while uarte.events_txstopped.read().bits() == 0 {}
            uarte.enable.write(|w| w.enable().disabled());
        }
        let rx_blocks: &'static mut [ObjectBlock<[u8; MSG_BUF_SIZE]>] = {
            const BLOCK: ObjectBlock<[u8; MSG_BUF_SIZE]> = ObjectBlock::new([0; MSG_BUF_SIZE]);
            static mut RX_BLOCKS: [ObjectBlock<[u8; MSG_BUF_SIZE]>; POOL_CAPACITY] =
                [BLOCK; POOL_CAPACITY];
            #[allow(static_mut_refs)]
            unsafe {
                &mut RX_BLOCKS
            }
        };
        let tx_blocks: &'static mut [ObjectBlock<[u8; MSG_BUF_SIZE]>] = {
            const BLOCK: ObjectBlock<[u8; MSG_BUF_SIZE]> = ObjectBlock::new([0; MSG_BUF_SIZE]);
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
        timer.as_timer0().bitmode.write(|w| w.bitmode()._16bit());
        timer.as_timer0().prescaler.write(|w| unsafe { w.bits(2) });
        timer.as_timer0().cc[0].write(|w| unsafe { w.bits(u32::max_value()) });
        timer
            .as_timer0()
            .shorts
            .write(|w| w.compare0_stop().set_bit());
        timer.as_timer0().intenset.write(|w| w.compare0().set());

        // Task and event handles.
        let event_timer3_compare0_ppi = &timer.as_timer0().events_compare[0];
        let event_uarte1_rdxrdy_ppi = &uarte.events_rxdrdy;
        let task_uarte1_stoprx_ppi = &uarte.tasks_stoprx;
        let task_timer3_start_ppi = &timer.as_timer0().tasks_start;
        let task_timer3_capture_ppi = &timer.as_timer0().tasks_capture[0];

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
        ppi1.set_event_endpoint(event_timer3_compare0_ppi);
        ppi1.set_task_endpoint(task_uarte1_stoprx_ppi);
        ppi1.enable();

        ppi2.set_event_endpoint(event_uarte1_rdxrdy_ppi);
        ppi2.set_task_endpoint(task_timer3_start_ppi);
        ppi2.set_fork_task_endpoint(task_timer3_capture_ppi);
        ppi2.enable();

        uarte
            .config
            .write(|w| w.parity().excluded().hwfc().disabled());
        uarte.baudrate.write(|w| w.baudrate().baud9600());
        uarte
            .psel
            .rxd
            .write(|w| unsafe { w.bits(rxp.psel_bits()).connect().connected() });
        uarte
            .psel
            .txd
            .write(|w| unsafe { w.bits(txp.psel_bits()).connect().connected() });

        uarte.enable.write(|w| w.enable().enabled());
        uarte.intenset.write(|w| w.rxstarted().set_bit());
        uarte.intenset.write(|w| w.endrx().set_bit());
        uarte.intenset.write(|w| w.rxto().set_bit());
        uarte.intenset.write(|w| w.error().set());
        uarte.tasks_startrx.write(|w| w.tasks_startrx().set_bit());

        MessageHandler {
            uarte,
            timer,
            ppi1,
            ppi2,
            rxp,
            txp,
        }
    }

    /// Start sending a message with the given message buffer
    fn start_tx(&self, tx_buf: Object<TxPool>) {
        self.uarte.events_endtx.reset();
        self.uarte.events_txstopped.reset();
        self.uarte
            .txd
            .ptr
            .write(|w| unsafe { w.ptr().bits(tx_buf.as_ptr() as u32) });
        self.uarte
            .txd
            .maxcnt
            .write(|w| unsafe { w.bits(tx_buf.len() as u32) });
        self.uarte.intenset.write(|w| w.endtx().set());
        self.uarte
            .tasks_starttx
            .write(|w| w.tasks_starttx().set_bit());
    }

    // Must be called within the corresponding UARTE perephiral device.
    fn rx_interrupt(&self) {
        if self.uarte.events_rxto.read().events_rxto().bit() {
            self.uarte.events_endrx.reset();
            self.uarte.events_rxto.reset();
            self.uarte.shorts.write(|w| w.endrx_startrx().enabled());
            self.uarte
                .tasks_flushrx
                .write(|w| w.tasks_flushrx().set_bit());
        }
        // BEGIN STATE ENDRX
        else if self.uarte.events_endrx.read().events_endrx().bit() {
            self.uarte.events_endrx.reset();
            let _bytes_that_require_processing = self.uarte.rxd.amount.read().bits();
            let top_free_block = RxPool.request().unwrap();
            self.uarte
                .rxd
                .maxcnt
                .write(|w| unsafe { w.bits(top_free_block.len() as u32) });
            self.uarte
                .rxd
                .ptr
                .write(|w| unsafe { w.ptr().bits(top_free_block.as_ptr() as u32) });
        }
        // BEGIN STATE RXSTARTED
        else if self.uarte.events_rxstarted.read().events_rxstarted().bit() {
            self.uarte.events_rxstarted.reset();
            self.uarte.events_endrx.reset();
        }
        // BEGIN STATE ENDTX
        if self.uarte.events_endtx.read().events_endtx().bit() {
            self.uarte.events_endtx.reset();
            self.uarte
                .tasks_stoptx
                .write(|w| w.tasks_stoptx().set_bit());
            let _bytes_transmitted = self.uarte.txd.amount.read().bits();
        }
        // BEGIN STATE ERROR
        if self.uarte.events_error.read().events_error().bit() {
            self.uarte.events_error.reset();
        }
    }
}
