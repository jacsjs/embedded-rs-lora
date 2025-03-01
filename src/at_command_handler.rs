use at_commands::{builder::{CommandBuilder, Initialized, Set, Query}, parser::{CommandParser, ParseError}};
use nrf52840_hal::{timer, uarte, Timer, Uarte};

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
    cycle_wait: usize,
}

#[derive(Debug)]
pub enum AtCommandError {
    Uart(uarte::Error),        // Wrapping UARTE errors
    Parse(ParseError),         // Wrapping parsing errors
    Other(usize),
}

impl From<uarte::Error> for AtCommandError {
    fn from(err: uarte::Error) -> Self {
        AtCommandError::Uart(err)
    }
}

impl From<ParseError> for AtCommandError {
    fn from(err: ParseError) -> Self {
        AtCommandError::Parse(err)
    }
}

impl<U, T> AtCommandHandler<U, T>
where
    U: uarte::Instance,
    T: timer::Instance,
{
    pub fn new(uarte: Uarte<U>, timer: Timer<T>, cycle_wait: usize) -> Self {
        AtCommandHandler { uarte, timer, buf: [0; 128], cycle_wait }
    }

    /// Common internal function to handle sending AT commands and reading responses
    fn execute_command<'a, F, G, D>(
        &'a mut self,
        rx_buffer: &'a mut [u8],
        custom_cycle_wait: Option<usize>,
        at_builder: G,
        parser: F,
    ) -> Result<D, AtCommandError>
    where
        F: FnOnce(CommandParser<'a, ()>) -> Result<D, ParseError>,
        G: FnOnce(CommandBuilder<'a, Initialized<Set>>) -> Result<&'a [u8], usize>,
    {
        let at_com = at_builder(CommandBuilder::create_set(&mut self.buf, true))
            .map_err(|size_req| AtCommandError::Other(size_req.into()))?;

        //self.uarte.write(&at_com).map_err(AtCommandError::from)?;
        self.uarte.write(&at_com).unwrap();

        match self.uarte.read_timeout(rx_buffer, &mut self.timer, custom_cycle_wait.unwrap_or(self.cycle_wait) as u32) {
            Ok(_) => {
                // Successfully read data; now parse using provided parser
                parser(CommandParser::parse(rx_buffer)).map_err(AtCommandError::from)
            }
            Err(err) => {
                if let uarte::Error::Timeout(_) = err {
                    // Attempt to parse even on timeout; this assumes the buffer may still contain usable data
                    parser(CommandParser::parse(rx_buffer)).map_err(AtCommandError::from)
                } else {
                    // Propagate other errors
                    Err(AtCommandError::from(err))
                }
            }
        }
    }

    #[inline(always)]
    pub fn send_expect_response<'a, F, G, D>(
        &'a mut self,
        rx_buffer: &'a mut [u8],
        at_builder: G,
        parser: F,
    ) -> Result<D, AtCommandError>
    where
        F: FnOnce(CommandParser<'a, ()>) -> Result<D, ParseError>,
        G: FnOnce(CommandBuilder<'a, Initialized<Set>>) -> Result<&'a [u8], usize>,
    {
        self.execute_command(rx_buffer, None, at_builder, parser)
    }

    /// In case the returned string is of interest, instead of parsing a specific response.
    /// 
    /// Useful for debugging and testing.
    #[inline(always)]
    pub fn send_expect_string<'a, G>(
        &'a mut self,
        rx_buffer: &'a mut [u8],
        at_builder: G,
    ) -> Result<&'a str, AtCommandError>
    where
        G: FnOnce(CommandBuilder<'a, Initialized<Set>>) -> Result<&'a [u8], usize>,
    {
        self.execute_command(rx_buffer, None, at_builder, |parser| {
            parser.expect_raw_string().finish()
        })
        .map(|tu| tu.0 )
    }

    /// In case no particular response message is expected or wished upon.
    #[inline(always)]
    pub fn send_no_response<'a, G>(
        &'a mut self,
        at_builder: G,
    ) -> Result<(), AtCommandError>
    where
        G: FnOnce(CommandBuilder<'a, Initialized<Set>>) -> Result<&'a [u8], usize>,
    {
        let at_com = at_builder(CommandBuilder::create_set(&mut self.buf, true))
            .map_err(|size_req| AtCommandError::Other(size_req.into()))?;

        // Send the command but do not attempt to read a response.
        self.uarte.write(&at_com).map_err(AtCommandError::from)
    }
}