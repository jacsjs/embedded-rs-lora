use at_commands::{builder::{CommandBuilder, Initialized, Query}, parser::{CommandParser, ParseError}};
use nrf52840_hal::{timer, uarte::{self, Error}, Timer, Uarte};

/// Handler for managing AT commands on the nrf52840 device.
/// Must attach a [Uarte] peripheral for serial communication consisting of AT requests and responses.
/// For determining UART read timeouts, the hardware nrf [Timer] is used.
/// 
/// AT Commands are built using [CommandBuilder], along with [CommandParser] for expected responses.
/// 
/// Since the responses must be configured beforehand, associating a [CommandBuilder] instance with
/// the appropriate [CommandParser] is imperative, otherwise [ParseError] is invoked.
/// 
/// # Examples
/// 
///```
/// // Create a new instance with the desired timeout in cycles.
/// let mut at = AtCommandHandler::new(uarte0, timer0, 1_000_000);
/// 
/// // Define queries and perform tx-rx AT transaction.
/// let (x,) = at.send_expect_response(
///     &mut res_buf,
///     |builder | builder
///         .named("+TESTQUERY")
///         .finish(), 
///     |parser | parser
///         .expect_identifier(b"AT+RESPONSE=")
///         .expect_int_parameter()
///         .expect_identifier(b"\r\n")
///         .finish(),
///     ).unwrap_or((0,));
///
/// rprintln!("VALUE: {}", x);
///```
/// 
/// 
pub struct AtCommandHandler<U, T>{
    uarte: Uarte<U>,
    timer: Timer<T>,
    buf: [u8; 128],
    cycle_wait: u32,
}

impl<U, T> AtCommandHandler<U, T>
where
    U: uarte::Instance,
    T: timer::Instance,
{
    pub fn new(uarte: Uarte<U>, timer: Timer<T>, cycle_wait: u32) -> Self {
        AtCommandHandler { uarte, timer, buf: [0; 128], cycle_wait }
    }

    /// Compact way of both AT command creation and (query, response) pair transactions.
    pub fn send_expect_response<'a, F, G, D>
    (&'a mut self, rx_buffer: &'a mut [u8], at_builder: G, parser: F) -> Result<D, Error>
    where
        F: FnOnce(CommandParser<'a, ()>) -> Result<D, ParseError>,
        G: FnOnce(CommandBuilder<'a, Initialized<Query>>) -> Result<&'a [u8], usize>,
    {
        let at_com = match at_builder(CommandBuilder::create_query(&mut self.buf, true)) {
            Ok(data) => data,
            Err(_) => return Err(Error::Transmit),
        };
        match self.uarte.write(&at_com) {
            Ok(_) => {
                match self.uarte.read_timeout(rx_buffer, &mut self.timer, self.cycle_wait) {
                    _ => {
                        match parser(CommandParser::parse(rx_buffer)) {
                            Ok(data) => Ok(data),
                            Err(_) => Err(Error::Receive),
                        }
                    },
                }
            },
            Err(e) => {
                Err(e)
            },
        }  
    }

    pub fn send_expect_string<'a, G>
    (&'a mut self, rx_buffer: &'a mut [u8], at_builder: G) -> Result<(), Error>
    where
        G: FnOnce(CommandBuilder<'a, Initialized<Query>>) -> Result<&'a [u8], usize>,
    {
        let at_com = match at_builder(CommandBuilder::create_query(&mut self.buf, true)) {
            Ok(data) => data,
            Err(_) => return Err(Error::Transmit),
        };
        match self.uarte.write(&at_com) {
            Ok(_) => {
                self.uarte.read_timeout(rx_buffer, &mut self.timer, self.cycle_wait)
            },
            Err(e) => {
                Err(e)
            },
        }
    }

    /// In case no particular response message is expected or wished upon.
    pub fn send_no_response<'a, G>
    (&'a mut self, at_builder: G) -> Result<(), Error>
    where
        G: FnOnce(CommandBuilder<'a, Initialized<Query>>) -> Result<&'a [u8], usize>,
    {
        let at_com = match at_builder(CommandBuilder::create_query(&mut self.buf, true)) {
            Ok(data) => data,
            Err(_) => return Err(Error::Transmit),
        };
        self.uarte.write(&at_com)
    }
}