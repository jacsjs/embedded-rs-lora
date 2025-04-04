use heapless::HistoryBuffer;
use nrf52840_hal::{
    timer::{self, Periodic},
    Timer,
};
use rtt_target::rprintln;

const MAX_TRACE_ENTRIES: usize = 8;

pub enum TraceState {
    Endrx,
    Endtx,
    RxTo,
    Rxstarted,
    Rxdrdy,
    ConfigAddress,
    ConfigMode,
    ConfigReset,
    ConfigSleep,
    ConfigTest,
    ConfigError,
    Run,
}

// Implementing the Display trait for the State enum
impl core::fmt::Display for TraceState {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            TraceState::Endrx => write!(f, "STATE_ENDRX    "),
            TraceState::Endtx => write!(f, "STATE_ENDTX    "),
            TraceState::RxTo => write!(f, "STATE_RXTO     "),
            TraceState::Rxstarted => write!(f, "STATE_RXSTARTED"),
            TraceState::Rxdrdy => write!(f, "STATE_RXDRDY   "),
            TraceState::ConfigAddress => write!(f, "CONFIG_ADDRESS "),
            TraceState::ConfigMode => write!(f, "CONFIG_MODE    "),
            TraceState::ConfigReset => write!(f, "CONFIG_RESET   "),
            TraceState::ConfigSleep => write!(f, "CONFIG_SLEEP   "),
            TraceState::ConfigTest => write!(f, "CONFIG_TEST    "),
            TraceState::ConfigError => write!(f, "CONFIG_ERROR   "),
            TraceState::Run => write!(f, "STATE_RUN      "),
        }
    }
}

pub struct TraceEntry {
    state: TraceState, // Represent the current state (could be an enum as well)
    debug_var: usize,  // Example debug variable
    timestamp: u32,
}

pub struct Trace<T>
where
    T: timer::Instance,
{
    timer: timer::Timer<T, Periodic>,
    entries: HistoryBuffer<TraceEntry, MAX_TRACE_ENTRIES>,
}

impl<T> Trace<T>
where
    T: timer::Instance,
{
    pub fn new(mut timer: Timer<T, Periodic>) -> Self {
        timer.start(u32::max_value());
        Trace {
            timer: timer,
            entries: HistoryBuffer::new(),
        }
    }

    // This function ASSUMES that the buffer won't overflow,
    // in order to avoid branches within ISRs as much as possible.
    #[inline(always)]
    pub fn log(&mut self, state: TraceState, debug_var: usize) {
        // Get the timestamp directly at the time of logging
        let timestamp = self.timer.read();

        let entry = TraceEntry {
            state,
            debug_var,
            timestamp,
        };
        self.entries.write(entry);
    }

    pub fn display_trace(&mut self) {
        rprintln!("==================== Display Trace ====================");
        for entry in self.entries.oldest_ordered() {
            rprintln!(
                "State: {} | Debug Var: {:<10} | Timestamp: {}",
                entry.state,
                entry.debug_var,
                entry.timestamp,
            );
        }
    }
}
