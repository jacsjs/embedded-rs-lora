use nrf52840_hal::{timer::{self, Periodic}, Timer};
use rtt_target::rprintln;

const MAX_TRACE_ENTRIES: usize = 128; // Adjust as necessary

pub enum TraceState {
    Endrx,
    Timeout,
    Rxstarted,
    Rxdrdy
}

// Implementing the Display trait for the State enum
impl core::fmt::Display for TraceState {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            TraceState::Endrx => write!(f, "STATE_ENDRX    "),
            TraceState::Timeout => write!(f, "STATE_TIMEOUT  "),
            TraceState::Rxstarted => write!(f, "STATE_RXSTARTED"),
            TraceState::Rxdrdy => write!(f, "STATE_RXDRDY   "),
        }
    }
}

pub struct TraceEntry {
    state: TraceState, // Represent the current state (could be an enum as well)
    debug_var: usize, // Example debug variable
    timestamp: u32,
}

pub struct Trace<T> 
where 
    T: timer::Instance
{
    timer: timer::Timer<T, Periodic>,
    entries: [Option<TraceEntry>; MAX_TRACE_ENTRIES],
    current_index: usize, // Track where we are for display purposes
    stored_count: usize, // Track how many entries have been stored
}

impl<T> Trace<T> 
where
    T: timer::Instance
{
    pub fn new(mut timer: Timer<T, Periodic>) -> Self {
        timer.start(u32::max_value());
        Trace {
            timer: timer,
            entries: [const { None }; MAX_TRACE_ENTRIES],
            current_index: 0, // Initialize at 0
            stored_count: 0, // No entries stored initially
        }
    }

    // This function ASSUMES that the buffer won't overflow,
    // in order to avoid branches within ISRs as much as possible.
    #[inline(always)]
    pub fn log(&mut self, state: TraceState, debug_var: usize) {

        // Get the timestamp directly at the time of logging
        let timestamp = self.timer.read();
        
        self.entries[self.stored_count] = Some(TraceEntry {
            state,
            debug_var,
            timestamp
        });
        self.stored_count += 1;
    }

    pub fn display_trace(&mut self) {
        while self.current_index < self.stored_count {
            if let Some(ref entry) = self.entries[self.current_index] {
                rprintln!(
                    "State: {} | Debug Var: {:<10} | Timestamp: {}",
                    entry.state,
                    entry.debug_var,
                    entry.timestamp,
                );
                self.current_index += 1; // Move to the next index after printing
            }
        }
    }
}