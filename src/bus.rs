#[allow(unused_imports)]
use crate::constants::*;
#[allow(unused_imports)]
use crate::Xum1541Error::{self, *};
use crate::{Device, DeviceInfo};
use crate::buscmd::{BusMode, BusCommand, DeviceChannel};

#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};
use rusb::{Context, UsbContext};
use std::time::Duration;

const DEFAULT_BUS_TIMEOUT: Duration = Duration::from_secs(60);

/// The Bus struct is the main interface for accessing Commodore disk drives.
/// It provides key bus-level primitives, such as
/// * reset (this bus)
/// * read (from the bus)
/// * write (to the bus)
/// * talk (instruct a drive to talk)
/// * listen (instruct a drive to listen)
/// * open (open a file on a drive)
/// * close (close a file on a drive)
/// * untalk
/// * unlisten
#[derive(Debug)]
pub struct Bus {
    device: Device,
    _timeout: Duration,
    mode: BusMode,
}

impl Bus {
    /// Create a new Bus instance
    /// Consider using BusBuilder to create a Bus instance as this will also
    /// create the required Device instance
    pub fn new(device: Device, timeout: Duration) -> Self {
        trace!("Bus::new");
        Bus {
            device,
            _timeout: timeout,
            mode: BusMode::default(),
        }
    }

    pub fn initialize(&mut self) -> Result<(), Xum1541Error> {
        trace!("Bus::initialize");
        self.device.init()
    }

    /// Get the underlying Device context - may be used to share with other
    /// rusb instances, and to set the log level of rusb
    pub fn device_context(&self) -> &Context {
        self.device.context()
    }

    pub fn device_info(&self) -> Option<&DeviceInfo> {
        self.device.info()
    }

    /// Reset the IEC/IEEE-488 bus
    /// Will reboot all attached drives and reset the talk/listen state
    pub fn reset(&mut self) -> Result<(), Xum1541Error> {
        trace!("Entered Bus::reset");
        self.mode = BusMode::Idle;
        self.device.control_write(CTRL_RESET, 0, &[])
    }

    // Instruct a drive to talk
    pub fn talk(&mut self, dc: DeviceChannel) -> Result<(), Xum1541Error> {
        self.execute_command(BusCommand::Talk(dc))
    }

    // Instruct a drive to listen
    pub fn listen(&mut self, dc: DeviceChannel) -> Result<(), Xum1541Error> {
        self.execute_command(BusCommand::Listen(dc))
    }

    // Instruct a drive to stop talking
    pub fn untalk(&mut self) -> Result<(), Xum1541Error> {
        self.execute_command(BusCommand::Untalk)
    }

    // Instruct a drive to stop listening
    pub fn unlisten(&mut self) -> Result<(), Xum1541Error> {
        self.execute_command(BusCommand::Unlisten)
    }

    /// Open a file on a drive and channel
    /// Normally followed by write(filename) and then unlisten()
    pub fn open(&mut self, dc: DeviceChannel) -> Result<(), Xum1541Error> {
        self.execute_command(BusCommand::Open(dc))
    }

    /// Close a file on a drive and channel
    pub fn close(&mut self, dc: DeviceChannel) -> Result<(), Xum1541Error> {
        self.execute_command(BusCommand::Close(dc))
    }

    /// Read data from the bus (from a drive that is in talk mode)
    pub fn read(&self, buf: &mut [u8]) -> Result<usize, Xum1541Error> {
        trace!("Entered Bus::read buf.len(): {}", buf.len());

        Self::validate_read_params(buf, None, false)?;

        self.device.read_data(PROTO_CBM, buf)
    }

    /// Write data to the bus (to a drive that is in listen mode)
    pub fn write(&self, buf: &[u8]) -> Result<usize, Xum1541Error> {
        trace!("Entered Bus::write");
        let size = buf.len();
        if size == 0 {
            warn!("Attempt to write 0 bytes");
            return Err(Args {
                message: format!("Attempt to write 0 bytes"),
            });
        }
        self.device.write_data(PROTO_CBM, buf)
    }

    pub fn is_listening(&self) -> Option<&DeviceChannel> {
        match &self.mode {
            BusMode::Listening(dev) => Some(dev),
            _ => None,
        }
    }

    pub fn is_talking(&self) -> Option<&DeviceChannel> {
        match &self.mode {
            BusMode::Talking(dev) => Some(dev),
            _ => None,
        }
    }

    /// Read data from the bus until either buf.len() bytes are read or pattern is matched
    /// Returns the number of bytes read (including the pattern if found)
    pub fn read_until(&self, buf: &mut Vec<u8>, pattern: &[u8]) -> Result<usize, Xum1541Error> {
        let size = buf.len();
        trace!(
            "Bus::read_until buf.len() {size} pattern.len() {}",
            pattern.len()
        );

        Self::validate_read_params(buf, Some(pattern.len()), true)?;

        let mut total_read = 0;
        let pattern_len = pattern.len();

        while total_read < size {
            match self.read_one_byte()? {
                None => break, // EOF
                Some(byte) => {
                    buf[total_read] = byte;
                    total_read += 1;

                    // Check if we have enough bytes to match the pattern
                    if total_read >= pattern_len {
                        // Compare the last pattern_len bytes with our pattern
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

    /// Read data from the bus until either size bytes are read or any byte from pattern is matched
    /// Returns the number of bytes read (including the matching byte if found)
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

        Self::validate_read_params(buf, Some(pattern.len()), false)?;

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
}

/// Private functions for Bus
impl Bus {
    /// Common helper function for reading a single byte
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

    /// Common validation for read parameters
    /// pattern_len: Option<usize> - if provided, validates pattern length requirements
    /// check_pattern_size: bool - if true and pattern_len provided, checks pattern isn't larger than size
    fn validate_read_params(
        buf: &[u8],
        pattern_len: Option<usize>,
        check_pattern_size: bool,
    ) -> Result<(), Xum1541Error> {
        let size = buf.len();
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

    /// Execute a bus command
    fn execute_command(&mut self, command: BusCommand) -> Result<(), Xum1541Error> {
        trace!("Entered Bus::execute_command command {command}");
        // Handle open/close requiring listen mode first
        if command.requires_listen_first() {
            match &self.mode {
                BusMode::Listening(dc) => {
                    if let Some(cmd_dc) = command.device_channel() {
                        if *dc != cmd_dc {
                            warn!("Opening/closing channel on different device than currently listening");
                        }
                    }
                }
                _ => warn!("Must set device to listen mode before open/close"),
            }
        }

        // Check for mode conflicts
        if let Some(warning) = command.conflicts_with(&self.mode) {
            match &self.mode {
                BusMode::Talking(dc) | BusMode::Listening(dc) => {
                    warn!("{} {}:{}", warning, dc.device(), dc.channel());
                }
                BusMode::Idle => {
                    warn!("{}", warning);
                }
            }
        }

        // Update mode before executing command
        self.mode = command.next_mode();

        // Execute the command
        trace!("{}", command.trace_message());
        self.device
            .write_data(command.protocol(), &command.command_bytes())
            .map(|_| ())
    }
}

pub struct BusBuilder {
    xum_serial_number: Option<u8>,
    timeout: Option<Duration>,
    context: Option<Context>,
    device: Option<Device>,
}

impl BusBuilder {
    pub fn new() -> Self {
        BusBuilder {
            xum_serial_number: None,
            timeout: None,
            context: None,
            device: None,
        }
    }

    pub fn serial_number(&mut self, serial: u8) -> &mut Self {
        self.xum_serial_number = Some(serial);
        self
    }

    pub fn timeout(&mut self, duration: Duration) -> &mut Self {
        self.timeout = Some(duration);
        self
    }

    /// Set a rusb::Context to use for device communication.
    /// This allows setting the USB debug log level via context.set_log_level()
    /// using rusb::LogLevel.
    /// If not set, a new default context will be created.
    pub fn context(mut self, context: Context) -> Self {
        self.context = Some(context);
        self
    }

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
