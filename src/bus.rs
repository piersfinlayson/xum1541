//! [`Bus`] is the main interface for accessing Commodore disk drives and the IEC/IEEE-488 bus via the XUM1541.  Its use it preferred over direct use of [`Device`].

use crate::buscmd::{BusCommand, BusMode, DeviceChannel};
#[allow(unused_imports)]
use crate::constants::*;
#[allow(unused_imports)]
use crate::Xum1541Error::{self, *};
use crate::{Device, DeviceInfo};

#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};
use rusb::{Context, UsbContext};
use std::time::Duration;

/// Default Bus timeout - currently unused
pub const DEFAULT_BUS_TIMEOUT: Duration = Duration::from_secs(60);

/// The [`Bus`] struct is the main interface for accessing Commodore disk drives.
/// It provides key bus-level primitives, such as
/// * new (create the [`Bus`] object and associated [`Device`] object)
/// * reset (this bus)
/// * read (from the bus)
/// * write (to the bus)
/// * talk (instruct a drive to talk)
/// * listen (instruct a drive to listen)
/// * open (open a file on a drive)
/// * close (close a file on a drive)
/// * untalk
/// * unlisten
///
/// Use [`BusBuilder`] to create a new [`Bus`] (and [`Device`]) instance.
#[derive(Debug)]
pub struct Bus {
    device: Device,
    _timeout: Duration,
    mode: BusMode,
}

/// Public [`Bus`] functions
impl Bus {
    /// Creates a new bus instance. Use if you have manually created a [`Device`] instance.
    ///
    /// # Args:
    /// * device: Device - the device to use for communication
    /// * timeout: Duration - the timeout for bus operations, currently unused
    ///
    /// # Returns:
    /// * [`Bus`] - the new bus instance
    ///
    /// Prefer using [`BusBuilder`] to create a new [`Bus`] instance.
    ///
    /// # Example
    /// ```no_run
    /// use xum1541::{Bus, Device};
    /// let device = Device::new(0).unwrap();
    /// let mut bus = Bus::new(device, std::time::Duration::from_secs(60));
    /// // Now initialize both the Bus and Device simultaneously
    /// bus.initialize();
    /// ```
    pub fn new(device: Device, timeout: Duration) -> Self {
        trace!("Bus::new");
        Bus {
            device,
            _timeout: timeout,
            mode: BusMode::default(),
        }
    }

    /// Initialize both the [`Bus`] and [`Device`] instances simultaneously
    ///
    /// # Returns
    /// * `Ok(())` - if successful
    /// * `Err(Xum1541Error)` - if an error occurred
    ///
    /// # Example
    /// See [`Bus::new`]
    pub fn initialize(&mut self) -> Result<(), Xum1541Error> {
        trace!("Bus::initialize");
        self.device.init()
    }

    /// Reset the IEC/IEEE-488 bus. Will reboot all attached drives and reset the talk/listen state.
    ///
    /// # Returns
    /// * `Ok(())` - if successful
    /// * `Err(Xum1541Error)` - if an error occurred
    pub fn reset(&mut self) -> Result<(), Xum1541Error> {
        trace!("Entered Bus::reset");
        self.mode = BusMode::Idle;
        self.device.control_write(CTRL_RESET, 0, &[])
    }

    /// Instruct a drive to talk on the bus.
    ///
    /// # Args:
    /// * dc: DeviceChannel - the device and channel to instruct to talk
    ///
    /// # Returns
    /// * `Ok(())` - if successful
    /// * `Err(Xum1541Error)` - if an error occurred
    pub fn talk(&mut self, dc: DeviceChannel) -> Result<(), Xum1541Error> {
        self.execute_command(BusCommand::Talk(dc))
    }

    /// Instruct a drive to listen on the bus.
    ///
    /// # Args:
    /// * dc: DeviceChannel - the device and channel to instruct to listen
    ///
    /// # Returns
    /// * `Ok(())` - if successful
    /// * `Err(Xum1541Error)` - if an error occurred
    pub fn listen(&mut self, dc: DeviceChannel) -> Result<(), Xum1541Error> {
        self.execute_command(BusCommand::Listen(dc))
    }

    /// Instruct a drive to stop talking on the bus.
    ///
    /// # Returns
    /// * `Ok(())` - if successful
    /// * `Err(Xum1541Error)` - if an error occurred
    pub fn untalk(&mut self) -> Result<(), Xum1541Error> {
        self.execute_command(BusCommand::Untalk)
    }

    /// Instruct a drive to stop listening on the bus.
    ///
    /// # Returns
    /// * `Ok(())` - if successful
    /// * `Err(Xum1541Error)` - if an error occurred
    pub fn unlisten(&mut self) -> Result<(), Xum1541Error> {
        self.execute_command(BusCommand::Unlisten)
    }

    /// Open a file on a drive and channel.
    /// Normally followed by write(filename) and then unlisten().
    ///
    /// # Args:
    /// * dc: DeviceChannel - the device and channel to open
    ///
    /// # Returns
    /// * `Ok(())` - if successful
    /// * `Err(Xum1541Error)` - if an error occurred
    pub fn open(&mut self, dc: DeviceChannel) -> Result<(), Xum1541Error> {
        self.execute_command(BusCommand::Open(dc))
    }

    /// Close a file on a drive and channel.
    ///
    /// # Args:
    /// * dc: DeviceChannel - the device and channel to close
    ///
    /// # Returns
    /// * `Ok(())` - if successful
    /// * `Err(Xum1541Error)` - if an error occurred
    pub fn close(&mut self, dc: DeviceChannel) -> Result<(), Xum1541Error> {
        self.execute_command(BusCommand::Close(dc))
    }

    /// Read data from the bus (from a drive that is in talk mode).
    ///
    /// # Args:
    /// * buf: &mut [u8] - buffer to read data into
    ///
    /// # Returns
    /// * `Ok(usize)` - number of bytes read if successful
    /// * `Err(Xum1541Error)` - if an error occurred
    ///
    /// # Note
    /// Will warn if bus is in listening mode when this is called.
    pub fn read(&self, buf: &mut [u8]) -> Result<usize, Xum1541Error> {
        trace!("Entered Bus::read buf.len(): {}", buf.len());
        Self::validate_read_params(buf, None, false)?;
        if let BusMode::Listening(_) = self.mode {
            warn!("Instructed to write data when Bus in mode {}", self.mode);
        }
        self.device.read_data(PROTO_CBM, buf)
    }

    /// Read data from the bus until either buf.len() bytes are read or pattern is matched.
    ///
    /// # Args:
    /// * `buf` - buffer to read data into
    /// * `pattern` - pattern to match against
    ///
    /// # Returns
    /// * `Ok(usize)` - number of bytes read (including the pattern if found)
    /// * `Err(Xum1541Error)` - if an error occurred or invalid parameters
    ///
    /// # Note
    /// The pattern must not be empty and must be smaller than the buffer size.
    pub fn read_until(&self, buf: &mut Vec<u8>, pattern: &[u8]) -> Result<usize, Xum1541Error> {
        let size = buf.len();
        trace!(
            "Bus::read_until buf.len() {size} pattern.len() {}",
            pattern.len()
        );

        Self::validate_read_params(buf, Some(pattern), true)?;

        let mut total_read = 0;
        let pattern_len = pattern.len();

        while total_read < size {
            match self.read_one_byte()? {
                None => break, // EOF
                Some(byte) => {
                    buf[total_read] = byte;
                    total_read += 1;

                    if total_read >= pattern_len {
                        if buf[total_read - pattern_len..total_read] == pattern[..] {
                            trace!("Found pattern in read data");
                            break;
                        }
                    }
                }
            }
        }

        Ok(total_read)
    }

    /// Read data from the bus until either size bytes are read or any byte from pattern is matched.
    ///
    /// # Args:
    /// * `buf` - buffer to read data into
    /// * `pattern` - bytes to match against
    ///
    /// # Returns
    /// * `Ok(usize)` - number of bytes read (including the matching byte if found)
    /// * `Err(Xum1541Error)` - if an error occurred or invalid parameters
    ///
    /// # Note
    /// The pattern must not be empty and the buffer must be the size of the
    /// maximum desired read length.
    pub fn read_until_any(&self, buf: &mut Vec<u8>, pattern: &[u8]) -> Result<usize, Xum1541Error> {
        let size = buf.len();
        trace!(
            "Bus::read_until_any buf.len() {size} pattern.len() {}",
            pattern.len()
        );
        if buf.len() < size {
            return Err(Args {
                message: format!(
                    "Buffer length {} shorter than requested read size{}",
                    buf.len(),
                    size
                ),
            });
        }

        Self::validate_read_params(buf, Some(pattern), false)?;

        let mut total_read = 0;
        while total_read < size {
            match self.read_one_byte()? {
                None => break, // EOF
                Some(byte) => {
                    buf[total_read] = byte;
                    total_read += 1;

                    if pattern.contains(&byte) {
                        break;
                    }
                }
            }
        }

        Ok(total_read)
    }

    /// Write data to the bus (to a drive that is in listen mode).
    ///
    /// # Args:
    /// * `buf` - data to write to the bus
    ///
    /// # Returns
    /// * `Ok(usize)` - number of bytes written if successful
    /// * `Err(Xum1541Error)` - if an error occurred or buffer is empty
    ///
    /// # Note
    /// Will warn if bus is in talking mode when this is called.
    pub fn write(&self, buf: &[u8]) -> Result<usize, Xum1541Error> {
        trace!("Entered Bus::write");
        let size = buf.len();
        if size == 0 {
            warn!("Attempt to write 0 bytes");
            return Err(Args {
                message: format!("Attempt to write 0 bytes"),
            });
        }
        if let BusMode::Talking(_) = self.mode {
            warn!("Instructed to write data when Bus in mode {}", self.mode);
        }
        self.device.write_data(PROTO_CBM, buf)
    }

    /// Retrieve the underlying Device context.
    ///
    /// May be used to share with other rusb instances, and to set the log
    /// level of rusb.
    ///
    /// # Returns
    /// * `&Context` - the context used by the device
    pub fn device_context(&self) -> &Context {
        self.device.context()
    }

    /// Retrieve information about the underlying device.
    ///
    /// # Returns
    /// * `Option<&DeviceInfo>` - device information if available
    pub fn device_info(&self) -> Option<&DeviceInfo> {
        self.device.info()
    }

    /// Check if the bus is in listening mode.
    ///
    /// # Returns
    /// * `Option<&DeviceChannel>` - the device and channel if in listening mode, None otherwise
    pub fn is_listening(&self) -> Option<&DeviceChannel> {
        match &self.mode {
            BusMode::Listening(dev) => Some(dev),
            _ => None,
        }
    }

    /// Check if the bus is in talking mode.
    ///
    /// # Returns
    /// * `Option<&DeviceChannel>` - the device and channel if in talking mode, None otherwise
    pub fn is_talking(&self) -> Option<&DeviceChannel> {
        match &self.mode {
            BusMode::Talking(dev) => Some(dev),
            _ => None,
        }
    }
}

/// Private functions for Bus
impl Bus {
    /// Read a single byte from the bus.
    ///
    /// # Returns
    /// * `Ok(Option<u8>)` - the byte read if successful, None if EOF
    /// * `Err(Xum1541Error)` - if an error occurred
    fn read_one_byte(&self) -> Result<Option<u8>, Xum1541Error> {
        trace!("Bus::read_one_byte");
        let mut temp = vec![0u8; 1];
        match self.device.read_data(PROTO_CBM, &mut temp) {
            Ok(0) => Ok(None), // EOF
            Ok(1) => {
                trace!("Got a byte 0x{:02x}", temp[0]);
                Ok(Some(temp[0]))
            }
            Ok(_) => unreachable!(), // We only requested 1 byte
            Err(e) => Err(e),
        }
    }

    /// Validate parameters for read operations.
    ///
    /// # Args:
    /// * `buf` - buffer to validate
    /// * `pattern` - if provided, validates pattern length requirements
    /// * `check_pattern_size`: bool - if true and pattern provided, checks pattern isn't larger than size
    ///
    /// # Returns:
    /// * `Ok(())` - if validation successful
    /// * `Err(Xum1541Error)` - if validation failed
    fn validate_read_params(
        buf: &[u8],
        pattern: Option<&[u8]>,
        check_pattern_size: bool,
    ) -> Result<(), Xum1541Error> {
        let size = buf.len();
        let pattern_len = pattern.map(|p| p.len());
        trace!("Bus::validate_read_params buf.len(): {size} pattern_len: {pattern_len:?} check_pattern_size: {check_pattern_size}");

        if size == 0 {
            warn!("Attempt to read 0 bytes");
            return Err(Args {
                message: format!("Attempted to read 0 bytes"),
            });
        }

        if let Some(pattern_len) = pattern_len {
            if pattern_len == 0 {
                warn!("Attempt to read match against empty pattern");
                return Err(Args {
                    message: format!("Attempt to read match against empty pattern"),
                });
            }

            if check_pattern_size && pattern_len > size {
                warn!("Attempt to read match against pattern of larger {pattern_len} than requested read size {size}");
                return Err(Args { message: format!("Attempt to read match against pattern of larger {pattern_len} than requested read size {size}") });
            }
        }

        Ok(())
    }

    /// Execute a command on the IEC/IEEE-488 bus and update the bus state.
    ///
    /// # Args:
    /// * command: BusCommand - the command to execute (Talk, Listen, Open, Close, etc.)
    ///
    /// # Returns:
    /// * `Ok(())` - if command executed successfully
    /// * `Err(Xum1541Error)` - if an error occurred during command execution
    ///
    /// # Note:
    /// * Validates the command against current bus state and warns if inappropriate
    /// * Updates the bus mode based on the command:
    ///   * Open/Listen -> Listening mode
    ///   * Talk -> Talking mode
    ///   * Close/Unlisten/Untalk -> Idle mode
    fn execute_command(&mut self, command: BusCommand) -> Result<(), Xum1541Error> {
        trace!(
            "Entered Bus::execute_command command {command} bus in mode {}",
            self.mode
        );

        // Check the the command is valid for the current mode, and set the
        // bus to the new state
        match command {
            BusCommand::Open(dc) => {
                if self.mode != BusMode::Idle {
                    warn!("Open called when the bus was in state {}", self.mode);
                }

                // Now expect a write_data, followed by an Unlisten
                // Therefore we consider the Bus in Listening mode
                self.mode = BusMode::Listening(dc);
            }
            BusCommand::Close(_) => {
                if self.mode != BusMode::Idle {
                    warn!("Close called when the bus was in state {}", self.mode);
                }

                // The bus is now Idle
                self.mode = BusMode::Idle;
            }
            BusCommand::Listen(dc) => {
                if self.mode != BusMode::Idle {
                    warn!("Listen called when bus was in state {}", self.mode);
                }

                // Ths bus is now Listening
                self.mode = BusMode::Listening(dc);
            }
            BusCommand::Unlisten => {
                if let BusMode::Listening(_) = self.mode {
                    // Do nothing
                } else {
                    warn!("Unisten called when bus was in state {}", self.mode);
                }

                // Ths bus is now idle
                self.mode = BusMode::Idle;
            }
            BusCommand::Talk(dc) => {
                if self.mode != BusMode::Idle {
                    warn!("Talk called when bus was in state {}", self.mode);
                }

                // Ths bus is now Talking
                self.mode = BusMode::Talking(dc);
            }
            BusCommand::Untalk => {
                if let BusMode::Talking(_) = self.mode {
                    // Do nothing
                } else {
                    warn!("Unisten called when bus was in state {}", self.mode);
                }

                // Ths bus is now idle
                self.mode = BusMode::Idle;
            }
        }

        // Execute the command
        trace!("{}", command.trace_message());
        self.device
            .write_data(command.protocol(), &command.command_bytes())
            .map(|_| ())
    }
}

/// A builder pattern implementation for creating [`Bus`] instances with custom configuration.
///
/// Allows setting optional parameters like serial number, timeout, and USB context
/// before creating the final [`Bus`] instance.
///
/// # Example
///
/// # A simple example
///
/// ```no_run
/// use xum1541::BusBuilder;
///
/// let bus = BusBuilder::new()
///     .build()
///     .unwrap();
/// ```
/// 
/// # A more complex example
///
/// ```no_run
/// use xum1541::BusBuilder;
/// use std::time::Duration;
///
/// let bus = BusBuilder::new()
///     .serial_number(1)
///     .timeout(std::time::Duration::from_secs(60))
///     .build()
///     .unwrap();
/// ```
pub struct BusBuilder {
    xum_serial_number: Option<u8>,
    timeout: Option<Duration>,
    context: Option<Context>,
    device: Option<Device>,
}

impl BusBuilder {
    /// Creates a new [`BusBuilder`] instance with default values.
    ///
    /// All fields are initialized to None and can be set using the builder methods.
    ///
    /// # Returns
    /// * [`BusBuilder`] - new builder instance with default values
    ///
    /// # Example
    /// See [`BusBuilder`] documentation
    pub fn new() -> Self {
        BusBuilder {
            xum_serial_number: None,
            timeout: None,
            context: None,
            device: None,
        }
    }

    /// Sets the XUM-1541 device serial number to use.
    ///
    /// # Args:
    /// * serial - the serial number to match when finding the device
    ///
    /// # Returns
    /// * `&mut Self` - builder instance for method chaining
    ///
    /// # Note:
    /// If not set, serial defaults to 0 when building
    pub fn serial_number(&mut self, serial: u8) -> &mut Self {
        self.xum_serial_number = Some(serial);
        self
    }

    /// Sets the timeout duration for bus operations.
    ///
    /// # Args:
    /// * duration - the period to use for the Bus timeout
    ///
    /// # Returns
    /// * `&mut Self` - builder instance for method chaining
    ///
    /// # Note:
    /// If not set, defaults to [`DEFAULT_BUS_TIMEOUT`] when building
    pub fn timeout(&mut self, duration: Duration) -> &mut Self {
        self.timeout = Some(duration);
        self
    }

    /// Sets a custom USB context for device communication.
    ///
    /// This allows setting the USB debug log level via context.set_log_level()
    /// using rusb::LogLevel.
    ///
    /// # Args:
    /// * context - the [`rusb::Context`]` to use
    ///
    /// # Returns
    /// * `Self` - builder instance for method chaining
    ///
    /// ```rust,no_run
    /// use rusb::{Context, UsbContext};
    /// use xum1541::BusBuilder;
    /// 
    /// let mut context = Context::new().unwrap();
    /// context.set_log_level(rusb::LogLevel::Debug);
    /// 
    /// let bus = BusBuilder::new()
    ///     .context(context)
    ///     .build()
    ///     .unwrap();
    /// ```
    /// 
    /// # Note:
    /// If not set, a new default [`rusb::Context`]` will be created with LogLevel::Info
    pub fn context(mut self, context: Context) -> Self {
        self.context = Some(context);
        self
    }

    /// Builds and returns a new [`Bus`] instance using the configured parameters.
    ///
    /// # Returns
    /// * `Ok(Bus)` - the constructed Bus instance if successful
    /// * `Err(Xum1541Error)` - if an error occurred during construction
    ///
    /// # Example
    /// See [`BusBuilder`]
    /// 
    /// # Notes:
    /// * Uses default values for any parameters that weren't set:
    ///   * serial_number: 0 (take first device found)
    ///   * timeout: [`DEFAULT_BUS_TIMEOUT`]
    ///   * context: new Context with LogLevel::Info
    /// * Will create a new Device unless one was explicitly provided
    pub fn build(&mut self) -> Result<Bus, Xum1541Error> {
        // Create Device
        let xum_serial_number = self.xum_serial_number.unwrap_or(0);
        let device = if self.device.is_none() {
            if self.context.is_some() {
                let context = self.context.take().unwrap();
                Device::with_context(context, xum_serial_number)?
            } else {
                let mut context = rusb::Context::new()?;
                context.set_log_level(rusb::LogLevel::Info);
                Device::with_context(context, xum_serial_number)?
            }
        } else {
            self.device.take().unwrap()
        };

        // Create Bus
        let timeout = self.timeout.unwrap_or(DEFAULT_BUS_TIMEOUT);
        Ok(Bus::new(device, timeout))
    }
}
