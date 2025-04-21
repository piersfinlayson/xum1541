//! [`Bus`] is the main interface for accessing Commodore disk drives and the IEC/IEEE-488 bus via the XUM1541.  Its use it preferred over direct use of [`Device`].
use crate::constants::{CTRL_RESET, PROTO_CBM};
use crate::device::SwDebugInfo;
#[allow(unused_imports)]
use crate::DeviceChannel;
use crate::{CommunicationError, DeviceAccessError, DeviceConfig, Error};
use crate::{DeviceInfo, DeviceType, RemoteUsbDeviceConfig, UsbDeviceConfig};
use crate::{Ioctl, RemoteUsbInfo, UsbInfo};
use cmd::{BusCommand, BusMode};

#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};
use std::time::Duration;

pub mod builder;
pub mod cmd;

pub use builder::BusBuilder;

/// Default Bus timeout - currently unused, but may be passed to [`Bus::new`]
/// and [`crate::UsbBusBuilder::timeout`]
pub const DEFAULT_TIMEOUT: Duration = Duration::from_secs(60);

/// Types of device failure recovery supported by the Bus
#[derive(Debug, Clone, PartialEq, Default)]
pub enum BusRecoveryType {
    /// Will not attempt to auto recover the device
    #[default]
    Off,

    /// Will attempt to auto recover the device.  First it will try to create
    /// a device with the same serial number as before.  If that fails, it
    /// will accept any serial number
    All,

    /// Will attempt to auto recover the device, but only if the serial number
    /// matches the original device
    Serial,
}

/// The [`Bus`] struct is the main interface for accessing Commodore disk
/// drives.  it provides key bus-level primitives.
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
/// * ioctl (sends an ioctl to the XUM1541 device)
///
/// Use [`BusBuilder`] to create a new [`Bus`] (and [`Device`]) instance.
#[derive(Debug)]
pub struct Bus {
    device: DeviceType,
    _timeout: Duration,
    mode: BusMode,
    serial: Option<String>,
    auto_recover: BusRecoveryType,
}

/// Public [`Bus`] functions
impl Bus {
    /// Creates a new bus instance. Use if you have manually created a [`Device`] instance.
    ///
    /// Using [`BusBuilder`] is strongly preferred over using this method
    /// directly
    ///
    /// # Args:
    /// * device: - the device to use for communication
    /// * timeout: - the timeout for bus operations, currently unused
    ///
    /// # Returns:
    /// * [`Bus`] - the new bus instance
    ///
    /// Prefer using [`BusBuilder`] to create a new [`Bus`] instance.
    ///
    /// # Example
    /// ```no_run
    /// use xum1541::{Bus, DeviceType, DeviceConfig, UsbDeviceConfig, BUS_DEFAULT_TIMEOUT};
    /// let usb_config = UsbDeviceConfig::default();
    /// let device = DeviceType::new(DeviceConfig::Usb(usb_config)).unwrap();
    /// let mut bus = Bus::new(device, BUS_DEFAULT_TIMEOUT);
    /// // Now initialize both the Bus and Device simultaneously
    /// bus.initialize();
    /// ```
    #[must_use]
    pub fn new(device: DeviceType, timeout: Duration) -> Self {
        trace!("Bus::new");
        Bus {
            device,
            _timeout: timeout,
            mode: BusMode::default(),
            serial: None,
            auto_recover: BusRecoveryType::default(),
        }
    }

    /// Sets this Bus to try to auto recover from a device failure
    ///
    /// The auto recover mechansim will try to create a new device object
    /// of the same type as the original device, and with the same serial
    /// number.  It will only
    pub fn set_recovery_type(&mut self, recovery_type: BusRecoveryType) {
        self.auto_recover = recovery_type;
    }

    /// Initialize both the [`Bus`] and [`Device`] instances simultaneously
    ///
    /// # Returns
    /// * `Ok(())` - if successful
    /// * `Err(Error)` - if an error occurred
    ///
    /// # Example
    /// See [`Bus::new`]
    ///
    /// # Errors
    /// If device initialization fails.
    pub fn initialize(&mut self) -> Result<(), Error> {
        self.initialize_retry(true)
    }

    /// Reset the IEC/IEEE-488 bus. Will reboot all attached drives and reset the talk/listen state.
    ///
    /// # Returns
    /// * `Ok(())` - if successful
    /// * `Err(Error)` - if an error occurred
    ///
    /// # Errors
    /// If command fails
    pub fn reset(&mut self) -> Result<(), Error> {
        trace!("Entered Bus::reset");
        self.mode = BusMode::Idle;
        self.with_retry(|device| device.write_control(CTRL_RESET, 0, &[]))
    }

    /// Instruct a drive to talk on the bus.
    ///
    /// # Args:
    /// * dc: `DeviceChannel` - the device and channel to instruct to talk
    ///
    /// # Returns
    /// * `Ok(())` - if successful
    /// * `Err(Error)` - if an error occurred
    ///
    /// # Errors
    /// If command fails
    pub fn talk(&mut self, dc: DeviceChannel) -> Result<(), Error> {
        self.execute_command(&BusCommand::Talk(dc))
    }

    /// Instruct a drive to listen on the bus.
    ///
    /// # Args:
    /// * dc: `DeviceChannel` - the device and channel to instruct to listen
    ///
    /// # Returns
    /// * `Ok(())` - if successful
    /// * `Err(Error)` - if an error occurred
    ///
    /// # Errors
    /// If command fails
    pub fn listen(&mut self, dc: DeviceChannel) -> Result<(), Error> {
        self.execute_command(&BusCommand::Listen(dc))
    }

    /// Instruct a drive to listen on the bus without a channel.  This is
    /// strictly allowed, at least in IEEE-488, although not typically
    /// supported by a Commodore disk drive.
    ///
    /// # Args:
    /// * device: u8 - the device number to instruct to listen
    ///
    /// # Returns
    /// * `Ok(())` - if successful
    /// * `Err(Error)` - if an error occurred
    ///
    /// # Errors
    /// If command fails
    pub fn listen_no_channel(&mut self, device: u8) -> Result<(), Error> {
        self.execute_command(&BusCommand::ListenNoChannel(device))
    }

    /// Instruct a drive to stop talking on the bus.
    ///
    /// # Returns
    /// * `Ok(())` - if successful
    /// * `Err(Error)` - if an error occurred
    ///
    /// # Errors
    /// If command fails
    pub fn untalk(&mut self) -> Result<(), Error> {
        self.execute_command(&BusCommand::Untalk)
    }

    /// Instruct a drive to stop listening on the bus.
    ///
    /// # Returns
    /// * `Ok(())` - if successful
    /// * `Err(Error)` - if an error occurred
    ///
    /// # Errors
    /// If command fails
    pub fn unlisten(&mut self) -> Result<(), Error> {
        self.execute_command(&BusCommand::Unlisten)
    }

    /// Open a file on a drive and channel.
    /// Normally followed by write(filename) and then `unlisten()`.
    ///
    /// # Args:
    /// * dc: `DeviceChannel` - the device and channel to open
    ///
    /// # Returns
    /// * `Ok(())` - if successful
    /// * `Err(Error)` - if an error occurred
    ///
    /// # Errors
    /// If command fails
    pub fn open(&mut self, dc: DeviceChannel) -> Result<(), Error> {
        self.execute_command(&BusCommand::Open(dc))
    }

    /// Close a file on a drive and channel.
    ///
    /// # Args:
    /// * dc: `DeviceChannel` - the device and channel to close
    ///
    /// # Returns
    /// * `Ok(())` - if successful
    /// * `Err(Error)` - if an error occurred
    ///
    /// # Errors
    /// If command fails
    pub fn close(&mut self, dc: DeviceChannel) -> Result<(), Error> {
        self.execute_command(&BusCommand::Close(dc))
    }

    /// Read data from the bus (from a drive that is in talk mode).
    ///
    /// # Args:
    /// * buf: &mut [u8] - buffer to read data into
    ///
    /// # Returns
    /// * `Ok(usize)` - number of bytes read if successful
    /// * `Err(Error)` - if an error occurred
    ///
    /// # Note
    /// Will warn if bus is in listening mode when this is called.
    ///
    /// # Errors
    /// If command fails
    pub fn read(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        trace!("Entered Bus::read buf.len(): {}", buf.len());
        Self::validate_read_params(buf, None, false)?;
        if let BusMode::Listening(_) = self.mode {
            warn!("Instructed to write data when Bus in mode {}", self.mode);
        }
        self.with_retry(|device| device.read_data(PROTO_CBM, buf))
    }

    /// Read data from the bus until either `buf.len()` bytes are read or
    /// pattern is matched.
    ///
    /// # Args:
    /// * `buf` - buffer to read data into
    /// * `pattern` - pattern to match against
    ///
    /// # Returns
    /// * `Ok(usize)` - number of bytes read (including the pattern if found)
    /// * `Err(Error)` - if an error occurred or invalid parameters
    ///
    /// # Note
    /// The pattern must not be empty and must be smaller than the buffer size.
    ///
    /// # Errors
    /// If command fails
    pub fn read_until(&mut self, buf: &mut [u8], pattern: &[u8]) -> Result<usize, Error> {
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

                    if total_read >= pattern_len
                        && buf[total_read - pattern_len..total_read] == pattern[..]
                    {
                        trace!("Found pattern in read data");
                        break;
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
    /// * `Err(Error)` - if an error occurred or invalid parameters
    ///
    /// # Note
    /// The pattern must not be empty and the buffer must be the size of the
    /// maximum desired read length.
    ///
    /// # Errors
    /// If command fails
    pub fn read_until_any(&mut self, buf: &mut [u8], pattern: &[u8]) -> Result<usize, Error> {
        let size = buf.len();
        trace!(
            "Bus::read_until_any buf.len() {size} pattern.len() {}",
            pattern.len()
        );

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
    /// * `Err(Error)` - if an error occurred or buffer is empty
    ///
    /// # Note
    /// Will warn if bus is in talking mode when this is called.
    ///
    /// # Errors
    /// If command fails
    pub fn write(&mut self, buf: &[u8]) -> Result<usize, Error> {
        trace!("Entered Bus::write buf.len(): {}", buf.len());
        let size = buf.len();
        if size == 0 {
            warn!("Attempt to write 0 bytes");
            return Err(Error::Args {
                message: "Attempt to write 0 bytes".to_string(),
            });
        }
        if let BusMode::Talking(_) = self.mode {
            warn!("Instructed to write data when Bus in mode {}", self.mode);
        }
        self.with_retry(|device| device.write_data(PROTO_CBM, buf))
    }

    /// Sends an ioctl command to the device.  Reads status from the device
    /// after sending, except where the Ioctl is asyncronous
    ///
    /// # Arguments
    /// * `ioctl` - The `request_type` of type [`crate::constants::Ioctl`]
    /// * `address` - The address to target with this ioctl.  May be 0.
    /// * `secondary_address` - The secondary to target with this ioctl. May be 0.
    ///
    /// # Returns
    /// * `Ok(Option<u16>)` - 2 byte status from the device, or None for an asyncronous ioctl, as this function does not wait after async ioctl
    /// * `Err(Error)` - On failure
    ///
    /// Note - this function may be deprecated in a future version, and replaced
    /// with specific functions for required ioctls
    ///
    /// # Errors
    /// If command fails
    pub fn ioctl(
        &mut self,
        ioctl: Ioctl,
        address: u8,
        secondary_address: u8,
    ) -> Result<Option<u16>, Error> {
        self.with_retry(|device| device.ioctl(ioctl, address, secondary_address))
    }

    /// Get EOI from the bus
    ///
    /// # Returns
    /// - `u16` - On success, a 2 byte status from the device
    /// - `Error` - On failure, including if there is no status `response`
    ///   (because `GetEoi` is a syncronous command, hence we expect a status)
    ///
    /// # Errors
    /// If command fails
    pub fn get_eoi(&mut self) -> Result<u16, Error> {
        trace!("Bus::get_eoi");
        self.device
            .ioctl(Ioctl::GetEoi, 0, 0)?
            .ok_or(Error::Communication {
                kind: CommunicationError::IoctlFailed,
            })
    }

    /// Clear EOI on the bus
    ///
    /// # Returns
    /// - `u16` - On success, a 2 byte status from the device
    /// - `Error` - On failure, including if there is no status response
    ///   (because `GetEoi` is a syncronous command, hence we expect a status)
    ///
    /// # Errors
    /// If command fails
    pub fn clear_eoi(&mut self) -> Result<u16, Error> {
        trace!("Bus::clear_eoi");
        self.device
            .ioctl(Ioctl::ClearEoi, 0, 0)?
            .ok_or(Error::Communication {
                kind: CommunicationError::IoctlFailed,
            })
    }

    /// Waits for status from the XUM1541 device, after certain commands and
    /// ioctls.
    ///
    /// This will block, so should be used with care.  Consider spawning a
    /// thread for this call, especially if waiting for status after an
    /// asyncronous ioctl.
    ///
    /// It is unlikely that this will be needed unless an asyncronous
    /// [`crate::constants::Ioctl`] is issued.
    ///
    /// # Returns
    /// `Ok(u16)` - the 2 byte status value from the device
    /// `Err(Error)` - the error on failure
    ///
    /// # Errors
    /// If command fails
    pub fn wait_for_status(&mut self) -> Result<u16, Error> {
        self.with_retry(super::device::DeviceType::wait_for_status)
    }

    /// Retrieve information about the underlying device.
    ///
    /// # Returns
    /// * `Option<&DeviceInfo>` - device information if available
    pub fn device_info(&mut self) -> Option<DeviceInfo> {
        self.device.info() // Can't fail, so don't retry
    }

    pub fn usb_device_info(&mut self) -> Option<UsbInfo> {
        self.device.usb_info() // Can't fail, so don't retry
    }

    pub fn remote_usb_device_info(&mut self) -> Option<RemoteUsbInfo> {
        self.device.remote_usb_info() // Can't fail, so don't retry
    }

    pub fn device_sw_debug_info(&mut self) -> SwDebugInfo {
        self.device.sw_debug_info() // Can't fail, so don't retry
    }

    /// Check if the bus is in listening mode.
    ///
    /// # Returns
    /// * `Option<&DeviceChannel>` - the device and channel if in listening mode, None otherwise
    #[must_use]
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
    #[must_use]
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
    /// * `Err(Error)` - if an error occurred
    fn read_one_byte(&mut self) -> Result<Option<u8>, Error> {
        trace!("Bus::read_one_byte");
        let mut temp = vec![0u8; 1];
        let result = self.with_retry(|device| device.read_data(PROTO_CBM, &mut temp));
        match result {
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
    /// * `Err(Error)` - if validation failed
    fn validate_read_params(
        buf: &[u8],
        pattern: Option<&[u8]>,
        check_pattern_size: bool,
    ) -> Result<(), Error> {
        let size = buf.len();
        let pattern_len = pattern.map(<[u8]>::len);
        trace!("Bus::validate_read_params buf.len(): {size} pattern_len: {pattern_len:?} check_pattern_size: {check_pattern_size}");

        if size == 0 {
            let message = "Attempt to read 0 bytes".to_string();
            warn!("{message}");
            return Err(Error::Args { message });
        }

        if let Some(pattern_len) = pattern_len {
            if pattern_len == 0 {
                let message = "Attempt to read match against empty pattern".to_string();
                warn!("{message}");
                return Err(Error::Args { message });
            }

            if check_pattern_size && pattern_len > size {
                let message = format!("Attempt to read match against pattern of larger {pattern_len} than requested read size {size}");
                warn!("{message}");
                return Err(Error::Args { message });
            }
        }

        Ok(())
    }

    /// Execute a command on the IEC/IEEE-488 bus and update the bus state.
    ///
    /// # Args:
    /// * command: `BusCommand` - the command to execute (Talk, Listen, Open, Close, etc.)
    ///
    /// # Returns:
    /// * `Ok(())` - if command executed successfully
    /// * `Err(Error)` - if an error occurred during command execution
    ///
    /// # Note:
    /// * Validates the command against current bus state and warns if inappropriate
    /// * Updates the bus mode based on the command:
    ///   * Open/Listen -> Listening mode
    ///   * Talk -> Talking mode
    ///   * Close/Unlisten/Untalk -> Idle mode
    fn execute_command(&mut self, command: &BusCommand) -> Result<(), Error> {
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

                // An open is followed by a write for the filename followed
                // an unlisten.  Hence we consider an open to put the bus into
                // listen mode.
                self.mode = BusMode::Listening(*dc);
            }
            BusCommand::Close(_) => {
                if self.mode != BusMode::Idle {
                    warn!("Close called when the bus was in state {}", self.mode);
                }

                // A close isn't a bus state - it's a device statue.
            }
            BusCommand::Listen(dc) => {
                if self.mode != BusMode::Idle {
                    warn!("Listen called when bus was in state {}", self.mode);
                }

                // Ths bus is now Listening
                self.mode = BusMode::Listening(*dc);
            }
            BusCommand::ListenNoChannel(dc) => {
                if self.mode != BusMode::Idle {
                    warn!("ListenNoChannel called when bus was in state {}", self.mode);
                }

                // Ths bus is now Listening
                // Strictly we should handle the lack of a channel better, but
                // that requires interface breaking change to DeviceChannel to
                // allow a None channel.
                self.mode = BusMode::Listening(DeviceChannel::new(*dc, 0).unwrap());
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
                self.mode = BusMode::Talking(*dc);
            }
            BusCommand::Untalk => {
                if let BusMode::Talking(_) = self.mode {
                    // Do nothing
                } else {
                    warn!("Untalk called when bus was in state {}", self.mode);
                }

                // Ths bus is now idle
                self.mode = BusMode::Idle;
            }
        }

        // Execute the command
        debug!("Bus:execute_command {command}");
        let result = self.with_retry(|device| {
            device
                .write_data(command.protocol(), &command.command_bytes())
                .map(|_| ())
        });

        if let Err(Error::Communication {
            kind: CommunicationError::StatusValue { value: _ },
        }) = result
        {
            // Note: 'result' needs to be the expression you're matching on
            match command {
                BusCommand::Talk(_) | BusCommand::Listen(_) | BusCommand::Open(_) => {
                    debug!("Bus command {command} failed, setting bus back to Idle");
                    self.mode = BusMode::Idle;
                }
                _ => debug!(
                    "Bus command {command} failed, leaving bus in state {}",
                    self.mode
                ),
            }
        }
        result
    }

    // If the operation fails, and the failure looked like the device has
    // become disconnected, try to recreate the device.  If that succeeds,
    // try the operation again.
    //
    // Call like this:
    //     self.with_retry(|device| device.ioctl(ioctl, 8, 15)
    fn with_retry<T>(
        &mut self,
        mut f: impl FnMut(&mut DeviceType) -> Result<T, Error>,
    ) -> Result<T, Error> {
        match f(&mut self.device) {
            Ok(result) => Ok(result),
            Err(e) => match e.clone() {
                Error::DeviceAccess { kind } => match kind {
                    DeviceAccessError::NoDevice
                    | DeviceAccessError::Permission
                    | DeviceAccessError::NetworkConnection { .. } => {
                        self.attempt_device_recovery(e.clone())?;
                        info!("Device successfully recovered after error {}", e);
                        f(&mut self.device)
                    }
                    _ => {
                        warn!("Fatal device error - not retrying due to error type ({e})");
                        Err(e)
                    }
                },
                e => {
                    warn!("Fatal device error - not retrying due to error type ({e})");
                    Err(e)
                }
            },
        }
    }

    fn initialize_retry(&mut self, retry: bool) -> Result<(), Error> {
        trace!("Bus::initialize");
        if retry {
            self.with_retry(super::device::DeviceType::init)?;
        } else {
            self.device.init()?;
        }
        let info = self.device.info(); // Can't fail, don't retry
        if let Some(info) = info {
            self.serial = info.serial_number;
        }
        Ok(())
    }

    fn attempt_device_creation(&self, serial_num: Option<u8>) -> Result<DeviceType, Error> {
        match &self.device {
            DeviceType::Usb(_) => DeviceType::new(DeviceConfig::Usb(UsbDeviceConfig {
                context: None,
                serial_num,
            })),
            DeviceType::RemoteUsb(device) => {
                let remote_addr = device.get_remote_addr();
                DeviceType::new(DeviceConfig::RemoteUsb(RemoteUsbDeviceConfig {
                    serial_num,
                    remote_addr,
                }))
            }
        }
    }

    // Attempts to recreate the device.  If this fails, we return the error
    // argument as our Error, rather than an attempt_device_recovery error.
    // This allows the caller to just pass our error back up the stack.
    fn attempt_device_recovery(&mut self, error: Error) -> Result<(), Error> {
        // First check that we should actually recover
        if self.auto_recover == BusRecoveryType::Off {
            debug!("Didn't try to auto-recover Device, as recovery disabled");
            return Err(error);
        }

        // First of all we try and recover using the current serial number.
        // If there isn't a serial number and we're in All mode, that's OK.
        let serial = match &self.serial {
            None => {
                if self.auto_recover == BusRecoveryType::All {
                    None
                } else {
                    warn!("Auto-recovery of device requested, but we didn't have the device's serial bumber");
                    return Err(error);
                }
            }
            Some(serial_string) => match serial_string.parse::<u8>() {
                Ok(num) => Some(num),
                Err(e) => {
                    warn!("Auto-recovery of device requested, but couldn't convert device's serial: {serial_string} to u8: {e}");
                    return Err(error);
                }
            },
        };

        // Store off whether the current device was initialized - we will
        // need to initialize the new one, if so
        let was_initialized = self.device.initialized();

        // Attempt to (re-)create the device.  If this fails, and we were
        // trying with a specific serial number, and recovery type if All
        // try again with a None serial number.
        let result = self.attempt_device_creation(serial);
        let result = match result {
            Ok(device) => Ok(device),
            Err(Error::DeviceAccess {
                kind: DeviceAccessError::SerialMismatch { .. },
            }) if serial.is_some() && self.auto_recover == BusRecoveryType::All => {
                self.attempt_device_creation(None)
            }
            Err(e) => Err(e),
        };
        self.device = match result {
            Ok(device) => device,
            Err(e) => {
                warn!("Device auto-recovery failed: {e}");
                return Err(error);
            }
        };

        // (Re-)Initialize if necessary
        if was_initialized {
            match self.initialize_retry(false) {
                Ok(()) => Ok(()),
                Err(e) => {
                    warn!("Device auto-recovery initialization failed: {e}");
                    Err(error)
                }
            }
        } else {
            Ok(())
        }
    }
}
