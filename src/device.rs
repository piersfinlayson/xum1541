//! The [`Device`] module provides the low-level interface to the XUM1541 USB adapter.  Prefer [`crate::Bus`] for most use cases.
//!
//! It is unlikely you need to use this interface directly unless you are
//! re-implementing [`crate::Bus`] or adding to it.
#[allow(unused_imports)]
use crate::constants::*;
use crate::error::InternalError;
use crate::DeviceAccessKind::*;
use crate::Xum1541Error::{self, *};

#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};
use rusb::Device as RusbDevice;
use rusb::DeviceHandle as RusbDeviceHandle;
use rusb::{constants, Context, DeviceDescriptor, UsbContext};
use std::cmp::min;
use std::str::from_utf8;
use std::thread::sleep;

/// DeviceDebugInfo contains some additional data from XUM1541 supporting
/// firmware version 8 and onwards.
#[derive(Debug)]
pub struct DeviceDebugInfo {
    /// Git revision of the code the device firmware was built from
    pub git_rev: Option<String>,
    /// AVR gcc version the device firmware was built with
    pub gcc_ver: Option<String>,
    /// AVR libc version the device firmware was built with
    pub libc_ver: Option<String>,
}

impl Default for DeviceDebugInfo {
    fn default() -> Self {
        DeviceDebugInfo {
            git_rev: None,
            gcc_ver: None,
            libc_ver: None,
        }
    }
}

/// DeviceInfo contains information read from the XUM1541 device.
#[derive(Debug)]
pub struct DeviceInfo {
    /// Product [`String`] from the USB device
    pub product: String,
    /// Manufacturer [`String`]` from the USB device
    pub manufacturer: Option<String>,
    /// Serial number [`String`] from the USB device
    pub serial_number: Option<String>,
    /// Firmware version from the USB device
    pub firmware_version: u8,
    /// Capabilities from the USB device.  Use [`DeviceInfo::print_capabilities`] to get a human readable list of these
    pub capabilities: u8,
    /// Status retrieved from the device at initialization.  Use [`DeviceInfo::print_status`] to get a human readable list of any flags
    pub status: u8,
    /// Debug information from the device, if available
    pub debug_info: Option<DeviceDebugInfo>,
}

impl DeviceInfo {
    /// Prints DeviceInfo to stdout in a human-readable format
    pub fn print_capabilities(&self) {
        let capability_flags = [
            (CAP_CBM, "CBM commands"),
            (CAP_NIB, "Parallel nibbler"),
            (CAP_NIB_SRQ, "1571 serial nibbler"),
            (CAP_IEEE488, "IEEE-488 bus"),
            (CAP_TAP, "153x tape support"),
        ];

        for (flag, description) in capability_flags {
            if self.capabilities & flag != 0 {
                println!("  - {}", description);
            }
        }

        if self.capabilities == 0 {
            println!("  - no capabilities");
        }
    }

    /// Prints Device status to stdout in a human-readable format
    pub fn print_status(&self) {
        let status_flags = [
            (STATUS_DOING_RESET, "Device wasn't cleanly shutdown"),
            (STATUS_IEEE488_PRESENT, "IEEE-488 device connected"),
            (STATUS_TAPE_PRESENT, "153x tape device connected"),
        ];

        for (flag, description) in status_flags {
            if self.status & flag != 0 {
                println!("  - {}", description);
            }
        }

        if self.status == 0 {
            println!("  - no status flags set");
        }
    }

    /// Prints the device debug information to stdout in a human-readable format,
    /// if present
    pub fn print_debug(&self) {
        if let Some(debug_info) = &self.debug_info {
            if let Some(rev) = &debug_info.git_rev {
                println!("  - Git revision: {}", rev);
            }
            if let Some(gcc) = &debug_info.gcc_ver {
                println!("  - GCC version: {}", gcc);
            }
            if let Some(libc) = &debug_info.libc_ver {
                println!("  - libc version: {}", libc);
            }
        } else {
            println!("  - no debug information")
        }
    }
}

/// Device represents the physical XUM1541 USB adapter.
///
/// It is unlikely to be necessary to implement directly using this struct
/// as [`crate::Bus`] provides Commodore IEC/IEEE-488 primitives and a slightly higher
/// level interface, exposing the key functionality of the device.
///
/// Instead of using [`Device::new`] or [`Device::with_context()`], it is recommended
/// to use [`crate::BusBuilder::build`], which will create both the Bus object, and at the
/// same time the Device object.  You can configure the Device object via
/// [`crate::BusBuilder`] if needed.
#[derive(Debug)]
pub struct Device {
    handle: RusbDeviceHandle<Context>,
    info: Option<DeviceInfo>,
    context: Context,
}

/// Public Device functions
impl Device {
    /// Creates a new Device using a default USB context.
    ///
    /// This is a simpler alternative to [`Device::with_context`] when you don't need
    /// to configure the USB context.
    ///
    /// # Arguments
    /// * `serial_num` - Serial number to match, or 0 to use first available device
    ///
    /// # Returns
    /// * `Ok(Device)` - Successfully created device instance
    /// * `Err(Xum1541Error)` - If device creation fails (no device found or USB error)
    ///
    /// Note that the Device returned has not been initialized.  It must be
    /// initialized before use with [`Device::init`].
    /// # Example
    ///
    /// ```rust,no_run
    /// use xum1541::Device;
    /// // Create device using first available (serial_num = 0)
    ///
    /// let device = Device::new(0).unwrap();
    ///
    /// // Now do device.init().unwrap(); etc
    /// ```
    pub fn new(serial_num: u8) -> Result<Self, Xum1541Error> {
        trace!("Device::new serial_num {serial_num}");
        let context = Context::new()?;
        Self::with_context(context, serial_num)
    }

    /// Create a Device with provided [`rusb::Context`]. Helpful if you want
    /// to configure rusb logging.  Otherwise use [`Device::new`].
    ///
    /// # Arguments
    /// * `context` - The USB context to use for device operations
    /// * `serial_number` - Device serial number to match, or 0 to use first device found
    ///
    /// # Returns
    /// * `Ok(Device)` - Successfully created device instance
    /// * `Err(Xum1541Error)` - If device creation fails (no device found or USB error)
    ///
    /// Note that the Device returned has not been initialized.  It must be
    /// initialized before use with [`Device::init`].
    ///
    /// # Example
    /// ```rust,no_run
    /// use rusb::{Context, UsbContext};
    /// use xum1541::Device;
    ///
    /// let mut context = rusb::Context::new().unwrap();
    /// context.set_log_level(rusb::LogLevel::Info);
    ///
    /// // Create device using first available (serial_num = 0)
    /// let device = Device::with_context(context, 0).unwrap();
    ///
    /// // Now do device.init().unwrap(); etc
    /// ```
    pub fn with_context(context: Context, serial_num: u8) -> Result<Self, Xum1541Error> {
        trace!("Device::with_context context {context:?} serial_num {serial_num}");
        let (_device, handle) = Self::find_device(&context, serial_num)?;

        Ok(Self {
            handle,
            info: None,
            context,
        })
    }

    /// Initialize the Device.
    ///
    /// This function
    /// * reads XUM1541 information
    /// * performs a hard reset of the USB device (to make sure it is in a
    ///   clean state)
    /// * resets the device if the device indicates it was shut down uncleanly (which is unlikely but not impossible at it performs a hard reset first)
    ///
    /// # Returns
    /// * `Ok(())` - Successfully initialized device
    /// * `Err(Xum1541Error)` - If device initialization fails
    pub fn init(&mut self) -> Result<(), Xum1541Error> {
        trace!("Device::init");
        self.info = Some(self.initialize_device()?);
        Ok(())
    }

    /// Returns the [`DeviceInfo`] as an [`Option<&DeviceInfo>`].
    ///
    /// May be None, in which case the Device has not been initialized
    pub fn info(&self) -> Option<&DeviceInfo> {
        trace!("Device::info");
        self.info.as_ref()
    }

    /// Returns a refernce to the Context stored within this device
    pub fn context(&self) -> &Context {
        trace!("Device::context");
        &self.context
    }

    /// Do a hard reset of the USB device, and reinitialize it afterwards.
    ///
    /// Use with caution, as this function can fail, in which case you may be
    /// in a worse state than before, as if it fails you will be left with an
    /// un-initialized device.
    ///
    /// The [`DeviceInfo`] may change during this process.
    ///
    /// # Returns
    /// * `Ok(())` - If successful
    /// * `Err(Xum1541Error)` - On error
    pub fn hard_reset_and_re_init(&mut self) -> Result<(), Xum1541Error> {
        trace!("Device::hard_reset_and_re_init");
        // Drop the info - this will be reinitialized shortly
        // Resetting the device doesn't require this
        self.info = None;

        self.hard_reset_pre_init()?;

        self.init()
    }

    /// Sends a control message and reads the response
    ///
    /// # Arguments
    /// * `request` - The request_type byte
    /// * `value` - The 2 byte request value
    /// * `buffer` - A buffer which will be filled in with the read data
    ///
    /// # Returns
    /// * `Ok(usize)` - On success, with the number of bytes read
    /// * `Err(Xum1541Error)` - On failure
    pub fn control_read(
        &self,
        request: u8,
        value: u16,
        buffer: &mut [u8],
    ) -> Result<usize, Xum1541Error> {
        trace!(
            "Device::control_read request 0x{request:02x} value 0x{value:02x} buffer.len() {}",
            buffer.len()
        );

        // Class-specific request, device as recipient, with IN direction
        const REQUEST_TYPE: u8 = constants::LIBUSB_REQUEST_TYPE_CLASS
            | constants::LIBUSB_RECIPIENT_ENDPOINT
            | constants::LIBUSB_ENDPOINT_IN;

        // Perform control transfer and return number of bytes read
        self.handle
            .read_control(
                REQUEST_TYPE,
                request,
                value,
                0, // index
                buffer,
                DEFAULT_CONTROL_TIMEOUT,
            )
            .map_err(|e| e.into())
    }

    /// Sends a control message and attempts to retrieve the status from the
    /// device.  It does not attempt to retrieve status if SHUTDOWN or RESET
    /// was issued, as this would likely hang.
    ///
    /// # Arguments
    /// * `request` - The request_type byte
    /// * `value` - The 2 byte request value
    /// * `buffer` - A buffer with any additional data to send
    ///
    /// # Returns
    /// * `Ok(())` - On success
    /// * `Err(Xum1541Error)` - On failure
    pub fn control_write(
        &self,
        request: u8,
        value: u16,
        buffer: &[u8],
    ) -> Result<(), Xum1541Error> {
        trace!(
            "Device::control_write request 0x{request:02x} value 0x{value:02x} buffer.len() {}",
            buffer.len()
        );
        const REQUEST_TYPE: u8 = constants::LIBUSB_REQUEST_TYPE_CLASS
            | constants::LIBUSB_RECIPIENT_ENDPOINT
            | constants::LIBUSB_ENDPOINT_OUT;

        // Send control transfer
        self.handle.write_control(
            REQUEST_TYPE,
            request,
            value,
            0, // index
            buffer,
            DEFAULT_CONTROL_TIMEOUT,
        )?;

        // Device doesn't seem to want to respond to use if we've shut it
        // down or called reset
        if (request != CTRL_SHUTDOWN) && (request != CTRL_RESET) {
            // Wait for status response
            self.wait_status()?;
        }

        trace!("Exited control_write - success");
        Ok(())
    }

    /// Sends a command to the device.  Reads status from the device after sending
    ///
    /// # Arguments
    /// * `cmd` - The request_type byte
    /// * `device` - The Commodore device to target with this cmd
    /// * `channel` - The Commodore device channel to target with this cmd
    ///
    /// # Returns
    /// * `Ok(u16)` - 2 byte status from the device
    /// * `Err(Xum1541Error)` - On failure
    pub fn command_only(&self, cmd: u8, device: u8, channel: u8) -> Result<u16, Xum1541Error> {
        trace!("Device::command_only cmd 0x{cmd:02x} device {device} channel {channel}");

        // Build 4-byte command buffer
        let cmd_buf = [cmd, device, channel, 0u8];

        // Send command
        match self
            .handle
            .write_bulk(BULK_OUT_ENDPOINT, &cmd_buf, DEFAULT_WRITE_TIMEOUT)
        {
            Ok(_) => {
                trace!(
                    "Successfully send command 0x{cmd:02x} to device {device} channel {channel}",
                );
                ()
            }
            Err(e) => {
                warn!("Failed to send command 0x{cmd:02x} to device {device} channel {channel}",);
                return Err(e.into());
            }
        }
        self.wait_status()
    }

    /// Writes data to the device
    ///
    /// Will attempt to write all data, but will return the amount written on
    /// success, even if all the data could not be written.
    ///
    /// # Argumemts
    /// * `mode` - Mode, one of xum1541::PROTO_*, see [`constants`]
    /// * `data` - Data to write
    ///
    /// # Returns
    /// * `Ok(usize)` - On success, number of bytes written
    /// * `Err(Xum1541Error>` - On failure
    pub fn write_data(&self, mode: u8, data: &[u8]) -> Result<usize, Xum1541Error> {
        let size = data.len();
        trace!("Device::write_data mode b{mode:08b} data.len() {size}");

        // Validate inputs
        if size > MAX_XFER_SIZE {
            warn!(
                "Attempted to write {size} more than max supported number of bytes {MAX_XFER_SIZE}"
            );
            return Err(Args { message: format!("Attempted to write {size} more than max supported number of bytes {MAX_XFER_SIZE}") });
        }

        // Send the write command with 4-byte command buffer
        let cmd_buf = [WRITE, mode, (size & 0xff) as u8, ((size >> 8) & 0xff) as u8];

        self.handle
            .write_bulk(BULK_OUT_ENDPOINT, &cmd_buf, DEFAULT_WRITE_TIMEOUT)?;

        // Write the actual data in chunks
        let mut bytes_written = 0;
        let mut data_slice = &data.as_ref()[..size];

        while bytes_written < size {
            let chunk_size = min(MAX_XFER_SIZE, size - bytes_written);
            let chunk = &data_slice[..chunk_size];

            let written =
                self.handle
                    .write_bulk(BULK_OUT_ENDPOINT, chunk, DEFAULT_WRITE_TIMEOUT)?;

            // If we wrote less than requested, we're done
            if written < chunk_size {
                bytes_written += written;
                break;
            }

            bytes_written += written;
            data_slice = &data_slice[written..];
        }

        // For CBM protocol, wait for status
        if (mode & PROTO_MASK) == PROTO_CBM {
            let status = self.wait_status()?;
            trace!("Retrieved status {:04x}", status);
            Ok(status as usize)
        } else {
            Ok(bytes_written)
        }
    }

    /// Reads data from the device
    ///
    /// Will attempt to read data to fill the provided buffer but will stop
    /// and return the size read if the device stops sending data.  Will also
    /// stop if buffer is filled up.
    ///
    /// # Argumemts
    /// * `mode` - Mode, one of xum1541::PROTO_*
    /// * `data` - Data to write
    ///
    /// # Returns
    /// * `Ok(usize)` - On success, number of bytes written
    /// * `Err(Xum1541Error>` - On failure
    pub fn read_data(&self, mode: u8, buffer: &mut [u8]) -> Result<usize, Xum1541Error> {
        let size = buffer.len();
        trace!("Device::read_data mode b{mode:08b} buffer.len() {size}");

        // Check inputs
        if size > MAX_XFER_SIZE {
            warn!(
                "Attempted to read {size} more than max supported number of bytes {MAX_XFER_SIZE}"
            );
            return Err(Args { message: format!("Attempted to read {size} more than max supported number of bytes {MAX_XFER_SIZE}") });
        }
        let buf_len = buffer.len();
        if size > buf_len {
            warn!("Attempted to read {size} more than buffer length {buf_len}");
            return Err(Args {
                message: format!("Attempted to read {size} more than buffer length {buf_len}"),
            });
        }

        // Send read command with 4-byte command buffer
        let cmd_buf = [READ, mode, (size & 0xff) as u8, ((size >> 8) & 0xff) as u8];

        self.handle
            .write_bulk(BULK_OUT_ENDPOINT, &cmd_buf, DEFAULT_WRITE_TIMEOUT)?;

        // Read data in chunks
        let mut bytes_read = 0;
        while bytes_read < size {
            let chunk_size = std::cmp::min(MAX_XFER_SIZE, size - bytes_read);
            let chunk = &mut buffer.as_mut()[bytes_read..bytes_read + chunk_size];

            let read = self
                .handle
                .read_bulk(BULK_IN_ENDPOINT, chunk, DEFAULT_READ_TIMEOUT)?;

            bytes_read += read;

            // If we read less than requested, transfer is done
            if read < chunk_size {
                break;
            }
        }

        trace!("Read {} bytes", bytes_read);
        Ok(bytes_read)
    }
}

/// Private Device functions
impl Device {
    /// Enumerate the bus, find the appropriate device and open it
    fn find_device(
        context: &Context,
        serial_num: u8,
    ) -> Result<(RusbDevice<Context>, RusbDeviceHandle<Context>), Xum1541Error> {
        trace!("Device::find_device context {context:?} serial_num {serial_num}");

        let mut found_any_xum1541 = false;
        let mut found_serial_nums = vec![];
        for device in context.devices()?.iter() {
            let device_desc = device.device_descriptor()?;
            debug!(
                "Found USB device {:04x}:{:04x}",
                device_desc.vendor_id(),
                device_desc.product_id()
            );

            if device_desc.vendor_id() == XUM1541_VID && device_desc.product_id() == XUM1541_PID {
                debug!("Found XUM1541 device");
                found_any_xum1541 = true;
                let handle = device.open()?;

                // Try and read the serial number, whether we are looking for
                // it or not
                trace!("Port number requested {}", serial_num);
                match handle.read_serial_number_string_ascii(&device_desc) {
                    Ok(serial) => {
                        if let Ok(num) = serial.parse::<u8>() {
                            if num == serial_num {
                                info!("Found device with matching serial number {num}");
                                return Ok((device, handle));
                            }
                            found_serial_nums.push(num);
                            debug!(
                                "Device serial number {num} didn't match requested {serial_num}",
                            );
                        }
                    }
                    Err(e) => {
                        warn!("Couldn't read device serial number: {}", e);
                        if serial_num == 0 {
                            // Despite failing to read the serial number
                            // we can still use the device if serial
                            // 0 was specified
                            return Ok((device, handle));
                        }
                    }
                }
            }
        }

        let err = match found_any_xum1541 {
            true => {
                error!("No matching XUM1541 device found, but found non-matching serial");
                DeviceAccess {
                    kind: SerialMismatch {
                        vid: XUM1541_VID,
                        pid: XUM1541_PID,
                        actual: found_serial_nums,
                        expected: serial_num,
                    },
                }
            }
            false => {
                error!("No suitable XUM1541 device found");
                DeviceAccess {
                    kind: NotFound {
                        vid: XUM1541_VID,
                        pid: XUM1541_PID,
                    },
                }
            }
        };
        Err(err)
    }

    // The device must be (re) initialized after this function.  It is
    // deliberately made a non-public function to avoid external parties
    // accidently resetting and then not re-initializing
    fn hard_reset_pre_init(&self) -> Result<(), Xum1541Error> {
        trace!("Device::hard_reset_pre_init");
        info!("Hard reset the device");
        self.handle.reset()?;
        sleep(DEFAULT_USB_RESET_SLEEP);
        trace!("Reset should be commplete, continue");
        Ok(())
    }

    /// Initialize device and get info
    fn initialize_device(&mut self) -> Result<DeviceInfo, Xum1541Error> {
        trace!("Device::initialize_device");

        // Hard reset the device
        // This fixes a multitude of problems - including
        // - Kicks anyone else off this USB device (assuming we have
        //   permissions)
        // - Seems to avoid various USB errors talking to the device if it's
        //   been left in a dodgy state.
        self.hard_reset_pre_init()?;

        let device_info = self.verify_and_setup_device()?;
        let mut info = self.read_basic_info(&device_info)?;

        // Handle device reset if needed
        if (info.status & STATUS_DOING_RESET) != 0 {
            warn!("xum1541 needs to be reset - resetting now");
            self.clear_halt()?;
        }

        // Add debug info for newer firmware versions
        if info.firmware_version >= 8 {
            info.debug_info = match self.read_debug_info() {
                Ok(info) => Some(info),
                Err(e) => match e {
                    InternalError::PublicError { error } => {
                        // This suggests we actually hit a USB or similar error
                        // so we treat this as failure
                        error!("Failed to read debug info from device: {}", error);
                        return Err(error);
                    }
                    e => {
                        // Hit some sort of processing error - we'll treat this as a non-failure mode
                        warn!("Failed to read debug info from device: {}", e);
                        None
                    }
                },
            };
        } else {
            debug!("Firmware earlier than 8 doesn't support extra debug info");
        }

        Ok(info)
    }

    fn clear_halt(&self) -> Result<(), Xum1541Error> {
        trace!("Device::clear_halt");

        match self.handle.clear_halt(BULK_IN_ENDPOINT) {
            Ok(_) => (),
            Err(e) => {
                error!("USB clear halt request failed for in endpoint: {}", e);
                return Err(e.into());
            }
        }

        match self.handle.clear_halt(BULK_OUT_ENDPOINT) {
            Ok(_) => Ok(()),
            Err(e) => {
                error!("USB clear halt request failed for out endpoint: {}", e);
                Err(e.into())
            }
        }
    }

    fn get_status(buf: &[u8]) -> u8 {
        trace!("Device::get_status buf.len() {}", buf.len());
        assert!(buf.len() >= 1);
        buf[0]
    }

    fn get_status_val(buf: &[u8]) -> u16 {
        trace!("Device::get_status_val buf.len() {}", buf.len());
        assert!(buf.len() >= 3);
        u16::from(buf[2]) << 8 | u16::from(buf[1])
    }

    fn wait_status(&self) -> Result<u16, Xum1541Error> {
        trace!("Device::wait_status");

        let mut status_buf = vec![0u8; STATUS_BUF_SIZE_SIZE];
        loop {
            trace!("Read bulk endpoint 0x{:02x}", BULK_IN_ENDPOINT);
            match self
                .handle
                .read_bulk(BULK_IN_ENDPOINT, &mut status_buf, DEFAULT_READ_TIMEOUT)
            {
                Ok(len) if len == STATUS_BUF_SIZE_SIZE => {
                    let status = Self::get_status(&status_buf);

                    match status {
                        IO_READY => {
                            let status = Self::get_status_val(&status_buf);
                            debug!("Got status value 0x{:04x}", status);
                            break Ok(status);
                        }
                        IO_BUSY => {
                            // Device busy, keep polling
                            trace!("Device busy");
                            let _ = sleep(DEFAULT_USB_LOOP_SLEEP);
                            continue;
                        }
                        IO_ERROR => {
                            trace!("Device IO error");
                            break Err(Communication {
                                message: "Device returned IO error".into(),
                            });
                        }
                        num => {
                            warn!("Unexpected debug status {}", num);
                            break Err(Communication {
                                message: format!("Unexpected error from device {}", num),
                            });
                        }
                    }
                }
                Ok(len) => {
                    trace!(
                        "Device returned wrong number of bytes {} vs {}",
                        len,
                        STATUS_BUF_SIZE_SIZE
                    );
                    break Err(Communication {
                        message: format!("Device returned wrong number of status bytes {}", len),
                    });
                }
                Err(e) => match e {
                    e @ rusb::Error::Timeout => {
                        debug!("Timeout waiting for device status {}", e);
                        let _ = sleep(DEFAULT_USB_LOOP_SLEEP);
                        continue;
                    }
                    other_error => {
                        debug!("Error reading from USB device {}", other_error);
                        break Err(other_error.into());
                    }
                },
            }
        }
    }

    /// Verify device identity and set up initial configuration
    fn verify_and_setup_device(&mut self) -> Result<DeviceInfo, Xum1541Error> {
        trace!("Device::verify_and_setup_device");
        let device = self.handle.device();
        let device_desc = device.device_descriptor()?;

        // Read device strings
        let product = self.read_product_string(&device_desc)?;
        let manufacturer = self
            .handle
            .read_manufacturer_string_ascii(&device_desc)
            .ok();
        let serial_number = self
            .handle
            .read_serial_number_string_ascii(&device_desc)
            .ok();

        // Verify product string
        debug!("Product string: {}", product);
        if !product.contains("xum1541") {
            return Err(Init {
                message: format!("Device product string {} didn't contain xum1541", product),
            });
        }

        // Configure device
        self.setup_device_configuration()?;

        Ok(DeviceInfo {
            product,
            manufacturer,
            serial_number,
            firmware_version: 0, // Will be set later
            capabilities: 0,     // Will be set later
            status: 0,           // Will be set later
            debug_info: None,    // Will be set later
        })
    }

    /// Read product string with error handling
    fn read_product_string(&self, device_desc: &DeviceDescriptor) -> Result<String, Xum1541Error> {
        trace!("Device::read_product_string device_desc {device_desc:?}");
        self.handle
            .read_product_string_ascii(device_desc)
            .map_err(|_| Init {
                message: "Couldn't read device product string".into(),
            })
    }

    /// Set up device configuration and claim interface
    fn setup_device_configuration(&mut self) -> Result<(), Xum1541Error> {
        trace!("Device::setup_device_configuration");
        let config = self.handle.active_configuration()?;
        debug!("Current configuration is {}", config);
        if config != 1 {
            debug!("Set active configuration to 1");
            self.handle.set_active_configuration(1)?;
        }

        debug!("Claim interface");
        self.handle.claim_interface(0)?;
        Ok(())
    }

    /// Read basic device information
    fn read_basic_info(&mut self, initial_info: &DeviceInfo) -> Result<DeviceInfo, Xum1541Error> {
        trace!("Device::read_basic_info initial_info {initial_info:?}");
        let mut info_buf = [0u8; DEV_INFO_SIZE];
        let len = self.control_read(CTRL_INIT as u8, 0, &mut info_buf)?;

        trace!(
            "Control transfer succeeded, got {} bytes: {:?}",
            len,
            info_buf
        );

        let info = DeviceInfo {
            product: initial_info.product.clone(),
            manufacturer: initial_info.manufacturer.clone(),
            serial_number: initial_info.serial_number.clone(),
            firmware_version: info_buf[0],
            capabilities: info_buf[1],
            status: info_buf[2],
            debug_info: None,
        };

        debug!(
            "Device info: version={}, capabilities={:02x}, status={:02x}",
            info.firmware_version, info.capabilities, info.status
        );

        // Verify firmware version
        if info.firmware_version < MIN_FW_VERSION {
            return Err(DeviceAccess {
                kind: FirmwareVersion {
                    actual: info.firmware_version,
                    expected: MIN_FW_VERSION,
                },
            });
        } else if info.firmware_version > CUR_FW_VERSION {
            warn!(
                "XUM1541 device has later firmware {} than expected {CUR_FW_VERSION}",
                info.firmware_version
            );
        }

        Ok(info)
    }

    /// Read debug information from device
    fn read_debug_info(&mut self) -> Result<DeviceDebugInfo, InternalError> {
        trace!("Device::read_debug_info");
        let ver_cmds = [CTRL_GITREV, CTRL_GCCVER, CTRL_LIBCVER];
        let mut debug_info = DeviceDebugInfo::default();
        let mut info_buf = [0u8; DEV_INFO_SIZE];

        // Process each command independently, ignoring failures
        for cmd in ver_cmds {
            match self.read_debug_command(cmd, &mut info_buf) {
                Ok(debug_str) => {
                    self.update_debug_info(&mut debug_info, cmd, debug_str);
                }
                Err(e) => match e {
                    InternalError::DeviceInfo { message } => {
                        debug!("No debug data for command {}: {} - ignoring", cmd, message);
                    }
                    e => {
                        warn!("Failed to read debug info for command {}: {}", cmd, e);
                        return Err(e);
                    }
                },
            }
        }
        Ok(debug_info)
    }

    /// Read a single debug command
    fn read_debug_command(
        &mut self,
        cmd: u8,
        info_buf: &mut [u8],
    ) -> Result<String, InternalError> {
        trace!(
            "Device::read_debug_command cmd {cmd} info_buf.len() {}",
            info_buf.len()
        );
        match self.control_read(cmd, 0, info_buf) {
            Ok(len) if len > 0 => from_utf8(info_buf)
                .map(|s| s.trim_matches(char::from(0)).to_string())
                .map_err(|_| InternalError::DeviceInfo {
                    message: "Invalid UTF-8 in debug info".into(),
                }),
            Ok(_) => {
                debug!("No debug data for command {:?}", cmd);
                Err(InternalError::DeviceInfo {
                    message: "No debug data available".into(),
                })
            }
            Err(e) => {
                // Propagate the error
                debug!("Failed to read debug info for command {:?}: {:?}", cmd, e);
                Err(e.into())
            }
        }
    }

    /// Update debug info structure with command results
    fn update_debug_info(&self, debug_info: &mut DeviceDebugInfo, cmd: u8, debug_str: String) {
        trace!(
            "Device::update_debug_info debug_info {debug_info:?} cmd {cmd} debug_str {debug_str}"
        );
        let description = match cmd {
            CTRL_GITREV => {
                debug_info.git_rev = Some(debug_str.clone());
                "Firmware git revision:"
            }
            CTRL_GCCVER => {
                debug_info.gcc_ver = Some(debug_str.clone());
                "Firmware compiled with avr-gcc version:"
            }
            CTRL_LIBCVER => {
                debug_info.libc_ver = Some(debug_str.clone());
                "Firmware using avr-libc version:"
            }
            _ => unreachable!(),
        };
        debug!("{} {}", description, debug_str);
    }
}

impl Drop for Device {
    fn drop(&mut self) {
        trace!("Device::drop");
        // Send shutdown command - ignoring errors since we're in drop
        let _ = self.control_write(CTRL_SHUTDOWN, 0, &[]);

        // Release interface - again ignoring errors
        let _ = self.handle.release_interface(0);
        trace!("Exited drop");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_device_not_found() {
        let result = Device::new(99);
        assert!(matches!(
            result,
            Err(DeviceAccess {
                kind: NotFound {
                    vid: XUM1541_VID,
                    pid: XUM1541_PID
                }
            })
        ));
    }
}
