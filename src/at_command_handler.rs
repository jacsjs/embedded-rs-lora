use at_commands::{
    builder::{CommandBuilder, Execute, Initialized, Query, Set},
    parser::{CommandParser, ParseError},
};
use nrf52840_hal::{timer, uarte, Timer, Uarte};

/// Handler for managing AT commands on the nrf52840 device.
/// Must attach a [Uarte] peripheral for serial communication consisting of AT requests and responses.
/// For determining UART read timeouts, the hardware nrf [Timer] is used.
///
/// AT Commands are built using a wrapper around [CommandBuilder] for sending, and [CommandParser] for expected responses.
///
/// # Examples
///
///```
/// // In these examples, we assume the information sent is bounced back.
/// // Create a new instance with the desired maximum timeout in cycles.
/// let mut at = AtCommandHandler::new(uarte0, timer0, 1_000_000);
///
/// // We can create a set command and expect a particular response.
/// // Invokes a ParseError if expected response does not match received.
/// let (x,) = at.set_expect_response(
///     &mut com_buf,                   // The sending command is filled here
///     &mut res_buf,                   // Byte array response resides here
///     |builder | builder
///         .named("+SET")
///         .with_int_parameter(42)
///         .finish(),
///     |parser | parser
///         .expect_identifier(b"AT+SET=")
///         .expect_int_parameter()
///         .expect_identifier(b"\r\n")
///         .finish(),
///     ).unwrap_or((0,));
///
/// assert_eq!(x, 42);
///
/// // If a specific format is of no interest, the whole string can examined.
/// // Notice that here we send a query, different from a set command.
/// let string = match cx.local.at.query_expect_string(&mut com_buf, &mut res_buf,
///     |builder| builder.named("+TESTSTRING").with_int_parameter(5555).finish())
///     {
///         Ok(tu) => tu,
///         Err(at_err) => {
///             rprintln!("Error: {:?}", at_err);
///             ""
///         }
///     };
/// assert_eq!(string, "AT+TESTSTRING=5555");
///
/// // Set command without any care for what returns (if anything).
/// cx.local.at.set_no_response(&mut com_buf,
/// |builder| builder
///     .named("+LEAVE_ME_ALONE")
///     .finish()
///     ).expect("Try again");
///```
pub struct AtCommandHandler<U, T> {
    uarte: Uarte<U>,
    timer: Timer<T>,
    cycle_wait: usize,
}

#[derive(Debug)]
pub enum AtCommandError {
    Uart(uarte::Error), // Wrapping UARTE errors
    Parse(ParseError),  // Wrapping parsing errors
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
    /// Create a new AT Command Handler.
    ///
    /// Takes ownsership of one of the Uarte peripherals. Take care that the given Uarte and Timer are not used elsewhere.
    /// The timer determines how long it should read UARTE data before processing; to avoid recieving and processing irrelevant data.
    pub fn new(uarte: Uarte<U>, timer: Timer<T>, cycle_wait: usize) -> Self {
        AtCommandHandler {
            uarte,
            timer,
            cycle_wait,
        }
    }

    /// Common internal function to handle sending AT commands and reading responses. Do not use directly.
    fn execute_command<'a, F, D>(
        &'a mut self,
        rx_buffer: &'a mut [u8],
        at_com: &'a [u8],
        custom_cycle_wait: Option<usize>,
        parser: F,
    ) -> Result<D, AtCommandError>
    where
        F: FnOnce(CommandParser<'a, ()>) -> Result<D, ParseError>,
    {
        self.uarte.write(&at_com).map_err(AtCommandError::from)?;

        match self.uarte.read_timeout(
            rx_buffer,
            &mut self.timer,
            custom_cycle_wait.unwrap_or(self.cycle_wait) as u32,
        ) {
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

    /// Send Set AT command expecting a specific response for extraction of values.
    #[inline(always)]
    pub fn set_expect_response<'a, F, G, D>(
        &'a mut self,
        com_buf: &'a mut [u8],
        read_buf: &'a mut [u8],
        at_builder: G,
        parser: F,
    ) -> Result<D, AtCommandError>
    where
        F: FnOnce(CommandParser<'a, ()>) -> Result<D, ParseError>,
        G: FnOnce(CommandBuilder<'a, Initialized<Set>>) -> Result<&'a [u8], usize>,
    {
        let at_com = at_builder(CommandBuilder::create_set(com_buf, true))
            .map_err(|size_req| AtCommandError::Other(size_req.into()))?;
        self.execute_command(read_buf, at_com, None, parser)
    }

    /// Send Query AT command expecting a specific response for extraction of values.
    #[inline(always)]
    pub fn query_expect_response<'a, F, G, D>(
        &'a mut self,
        com_buf: &'a mut [u8],
        read_buf: &'a mut [u8],
        at_builder: G,
        parser: F,
    ) -> Result<D, AtCommandError>
    where
        F: FnOnce(CommandParser<'a, ()>) -> Result<D, ParseError>,
        G: FnOnce(CommandBuilder<'a, Initialized<Query>>) -> Result<&'a [u8], usize>,
    {
        let at_com = at_builder(CommandBuilder::create_query(com_buf, true))
            .map_err(|size_req| AtCommandError::Other(size_req.into()))?;
        self.execute_command(read_buf, at_com, None, parser)
    }

    /// Send Execute AT command expecting a specific response for extraction of values.
    #[inline(always)]
    pub fn execute_expect_response<'a, F, G, D>(
        &'a mut self,
        com_buf: &'a mut [u8],
        read_buf: &'a mut [u8],
        at_builder: G,
        parser: F,
    ) -> Result<D, AtCommandError>
    where
        F: FnOnce(CommandParser<'a, ()>) -> Result<D, ParseError>,
        G: FnOnce(CommandBuilder<'a, Initialized<Execute>>) -> Result<&'a [u8], usize>,
    {
        let at_com = at_builder(CommandBuilder::create_execute(com_buf, true))
            .map_err(|size_req| AtCommandError::Other(size_req.into()))?;
        self.execute_command(read_buf, at_com, None, parser)
    }

    /// Send Set AT command, returning the whole string.
    ///
    /// Useful for debugging and testing.
    #[inline(always)]
    pub fn set_expect_string<'a, G>(
        &'a mut self,
        com_buf: &'a mut [u8],
        read_buf: &'a mut [u8],
        at_builder: G,
    ) -> Result<&'a str, AtCommandError>
    where
        G: FnOnce(CommandBuilder<'a, Initialized<Set>>) -> Result<&'a [u8], usize>,
    {
        let at_com = at_builder(CommandBuilder::create_set(com_buf, true))
            .map_err(|size_req| AtCommandError::Other(size_req.into()))?;
        self.execute_command(read_buf, at_com, None, |parser| {
            parser.expect_raw_string().finish()
        })
        .map(|tu| tu.0)
    }

    /// Send Query AT command, returning the whole string.
    ///
    /// Useful for debugging and testing.
    #[inline(always)]
    pub fn query_expect_string<'a, G>(
        &'a mut self,
        com_buf: &'a mut [u8],
        read_buf: &'a mut [u8],
        at_builder: G,
    ) -> Result<&'a str, AtCommandError>
    where
        G: FnOnce(CommandBuilder<'a, Initialized<Set>>) -> Result<&'a [u8], usize>,
    {
        let at_com = at_builder(CommandBuilder::create_set(com_buf, true))
            .map_err(|size_req| AtCommandError::Other(size_req.into()))?;
        self.execute_command(read_buf, at_com, None, |parser| {
            parser.expect_raw_string().finish()
        })
        .map(|tu| tu.0)
    }

    /// Send Execute AT command, returning the whole string.
    ///
    /// Useful for debugging and testing.
    #[inline(always)]
    pub fn execute_expect_string<'a, G>(
        &'a mut self,
        com_buf: &'a mut [u8],
        read_buf: &'a mut [u8],
        at_builder: G,
    ) -> Result<&'a str, AtCommandError>
    where
        G: FnOnce(CommandBuilder<'a, Initialized<Execute>>) -> Result<&'a [u8], usize>,
    {
        let at_com = at_builder(CommandBuilder::create_execute(com_buf, true))
            .map_err(|size_req| AtCommandError::Other(size_req.into()))?;
        self.execute_command(read_buf, at_com, None, |parser| {
            parser.expect_raw_string().finish()
        })
        .map(|tu| tu.0)
    }

    /// Send Set AT command without any care for potential responses.
    /// TODO: Not working as expected right now, I think its because of the Arduino but not sure yet.
    #[inline(always)]
    pub fn set_no_response<'a, G>(
        &'a mut self,
        com_buf: &'a mut [u8],
        at_builder: G,
    ) -> Result<(), AtCommandError>
    where
        G: FnOnce(CommandBuilder<'a, Initialized<Set>>) -> Result<&'a [u8], usize>,
    {
        let at_com = at_builder(CommandBuilder::create_set(com_buf, true))
            .map_err(|size_req| AtCommandError::Other(size_req.into()))?;

        // Send the command but do not attempt to read a response.
        self.uarte.write(&at_com).map_err(AtCommandError::from)
    }

    /// Send Execute AT command without any care for potential responses.
    /// TODO: Not working as expected right now, I think its because of the Arduino but not sure yet.
    #[inline(always)]
    pub fn execute_no_response<'a, G>(
        &'a mut self,
        com_buf: &'a mut [u8],
        at_builder: G,
    ) -> Result<(), AtCommandError>
    where
        G: FnOnce(CommandBuilder<'a, Initialized<Execute>>) -> Result<&'a [u8], usize>,
    {
        let at_com = at_builder(CommandBuilder::create_execute(com_buf, true))
            .map_err(|size_req| AtCommandError::Other(size_req.into()))?;

        // Send the command but do not attempt to read a response.
        self.uarte.write(&at_com).map_err(AtCommandError::from)
    }
}
