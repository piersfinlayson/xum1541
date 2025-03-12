use crate::constants::{
    BULK_IN_ENDPOINT, BULK_OUT_ENDPOINT, CTRL_GCCVER, CTRL_GITREV, CTRL_INIT, CTRL_LIBCVER,
    CTRL_RESET, CTRL_SHUTDOWN, CUR_FW_VERSION, DEFAULT_CONTROL_TIMEOUT, DEFAULT_READ_TIMEOUT,
    DEFAULT_USB_LOOP_SLEEP, DEFAULT_USB_RESET_SLEEP, DEFAULT_WRITE_TIMEOUT, DEV_INFO_SIZE, IO_BUSY,
    IO_ERROR, IO_READY, MAX_XFER_SIZE, MIN_FW_VERSION, PROTO_CBM, PROTO_MASK, READ,
    STATUS_BUF_SIZE, STATUS_DOING_RESET, WRITE, XUM1541_PID, XUM1541_VID,
    PRODUCT_STRINGS,
};
use crate::device::SwDebugInfo;
use crate::Ioctl;
use crate::{CommunicationError, DeviceAccessError, Error, InternalError};
use crate::{Device, DeviceDebugInfo, DeviceInfo, SpecificDeviceInfo};

#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};
use parking_lot::Mutex;
use rusb::Device as RusbDevice;
use rusb::DeviceHandle as RusbDeviceHandle;
use rusb::{constants, Context, DeviceDescriptor, UsbContext};
use serde::{Deserialize, Serialize};
use std::cmp::min;
use std::str::from_utf8;
use std::sync::Arc;
use std::thread::sleep;

/// Device represents the physical XUM1541 USB adapter.
///
/// It is unlikely to be necessary to implement directly using this struct
/// as [`crate::Bus`] provides Commodore IEC/IEEE-488 primitives and a slightly higher
/// level interface, exposing the key functionality of the device.
///
/// Instead of using [`UsbDevice::new`] it is recommended
/// to use [`crate::BusBuilder::build`], which will create both the Bus object, and at the
/// same time the Device object.  You can configure the Device object via
/// [`crate::BusBuilder`] if needed.
#[derive(Debug, Clone)]
pub struct UsbDevice {
    handle: Arc<Mutex<RusbDeviceHandle<Context>>>,
    info: Option<DeviceInfo>,
    usb_info: Option<UsbInfo>,
    sw_debug_info: SwDebugInfo,
    initialized: bool,
}

#[derive(Debug, Clone)]
pub struct UsbDeviceConfig {
    /// The [`rusb::Context`] to use for this device.  May be None to use
    /// default context.
    pub context: Option<Context>,

    /// The serial number of the device to use.  If omitted, the first
    /// detected device of the correct type will be used.  If set to 0, only
    /// a device with a zero serial number will be matched.
    pub serial_num: Option<u8>,
}

impl Default for UsbDeviceConfig {
    fn default() -> Self {
        Self {
            context: None,
            serial_num: None,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UsbInfo {
    pub vendor_id: u16,
    pub product_id: u16,
    pub bus_number: u8,
    pub device_address: u8,
}

impl SpecificDeviceInfo for UsbInfo {
    type Info = UsbInfo;

    fn print(&self) {
        println!("  - device: {:04x}:{:04x}", self.vendor_id, self.product_id);
        println!(
            "  - bus/address: {:03}-{:03}",
            self.bus_number, self.device_address
        );
    }
}

/// Public Device functions
impl Device for UsbDevice {
    type Config = UsbDeviceConfig;
    type SpecificDeviceInfo = UsbInfo;

    fn new(config: Option<Self::Config>) -> Result<Self, Error> {
        trace!("UsbDevice::new");

        // Set up a config object if we don't already have one
        let mut config = if let Some(config) = config {
            trace!("UsbDevice::new config {:?}", config);
            config
        } else {
            trace!("UsbDevice::new default config");
            UsbDeviceConfig {
                context: None,
                serial_num: None,
            }
        };
        trace!("DeviceConfig::serial_num {:?}", config.serial_num);
        trace!("DeviceConfig::context {:?}", config.context);

        // Create the rusb:Context if we weren't provided one
        let context = match config.context {
            Some(ctx) => ctx,
            None => Context::new()?,
        };
        config.context = Some(context.clone());

        // Get hold of the USB device
        let (_device, handle) = Self::find_device(&context, config.serial_num)?;

        // Return Self
        Ok(Self {
            handle: Arc::new(Mutex::new(handle)),
            info: None,
            usb_info: None,
            sw_debug_info: SwDebugInfo::default(),
            initialized: false,
        })
    }

    fn init(&mut self) -> Result<(), Error> {
        trace!("UsbDevice::init");
        self.sw_debug_info.increment_api_call();
        let (info, usb_info) = self.initialize_device()?;
        self.info = Some(info);
        self.usb_info = Some(usb_info);

        // An init can cause the xum1541 to stall its endpoints - if a reset
        // was required.  So, we need to check and clear the stall if
        // necessary
        //self.check_and_clear_stall()?;

        self.initialized = true;
        Ok(())
    }

    fn initialized(&mut self) -> bool {
        self.initialized
    }

    fn info(&mut self) -> Option<DeviceInfo> {
        trace!("UsbDevice::info");
        self.info.clone()
    }

    fn specific_info(&mut self) -> Option<Self::SpecificDeviceInfo> {
        trace!("UsbDevice::specific_info");
        self.usb_info.clone()
    }

    fn sw_debug_info(&mut self) -> SwDebugInfo {
        trace!("UsbDevice::sw_debug_info");
        self.sw_debug_info.clone()
    }

    fn hard_reset_and_re_init(&mut self) -> Result<(), Error> {
        trace!("UsbDevice::hard_reset_and_re_init");
        self.sw_debug_info.increment_api_call();
        // Drop the info - this will be reinitialized shortly
        // Resetting the device doesn't require this
        self.info = None;

        self.hard_reset_pre_init()?;

        self.init()
    }

    fn read_control(&mut self, request: u8, value: u16, buffer: &mut [u8]) -> Result<usize, Error> {
        trace!(
            "Device::read_control request 0x{request:02x} value 0x{value:02x} buffer.len() {}",
            buffer.len()
        );
        self.sw_debug_info.increment_api_call();

        // Class-specific request, device as recipient, with IN direction
        const REQUEST_TYPE: u8 = constants::LIBUSB_REQUEST_TYPE_CLASS
            | constants::LIBUSB_RECIPIENT_DEVICE
            | constants::LIBUSB_ENDPOINT_IN;

        // Perform control transfer and return number of bytes read
        let index = 0; // default control endpoint
        {
            self.handle
                .lock()
                .read_control(
                    REQUEST_TYPE,
                    request,
                    value,
                    index,
                    buffer,
                    DEFAULT_CONTROL_TIMEOUT,
                )
                .map_err(|e| e.into())
        }
    }

    fn write_control(&mut self, request: u8, value: u16, buffer: &[u8]) -> Result<(), Error> {
        trace!(
            "Device::write_control request 0x{request:02x} value 0x{value:02x} buffer.len() {}",
            buffer.len()
        );
        self.sw_debug_info.increment_api_call();
        const REQUEST_TYPE: u8 = constants::LIBUSB_REQUEST_TYPE_CLASS
            | constants::LIBUSB_RECIPIENT_DEVICE
            | constants::LIBUSB_ENDPOINT_OUT;

        // Send control transfer
        let index = 0; // default control endpoint
        {
            self.handle.lock().write_control(
                REQUEST_TYPE,
                request,
                value,
                index,
                buffer,
                DEFAULT_CONTROL_TIMEOUT,
            )?;
        }

        // Device doesn't seem to want to respond to use if we've shut it
        // down or called reset
        if (request != CTRL_SHUTDOWN) && (request != CTRL_RESET) {
            // Wait for status response
            let status = self.wait_for_status()?;
            if status == 0 {
                return Err(Error::Communication {
                    kind: CommunicationError::StatusValue { value: status },
                }
                .into());
            }
        }

        trace!("Exited control_write - success");
        Ok(())
    }

    fn write_data(&mut self, mode: u8, data: &[u8]) -> Result<usize, Error> {
        let size = data.len();
        trace!("UsbDevice::write_data mode b{mode:08b} data.len() {size}");
        self.sw_debug_info.increment_api_call();

        // Validate inputs
        if size > MAX_XFER_SIZE {
            let message = format!(
                "Attempted to write {size} more than max supported number of bytes {MAX_XFER_SIZE}"
            );
            warn!("{message}");
            return Err(Error::Args { message });
        }

        // Send the write command with 4-byte command buffer
        let cmd_buf = [WRITE, mode, (size & 0xff) as u8, ((size >> 8) & 0xff) as u8];

        {
            trace!("Write bulk to endpoint {BULK_OUT_ENDPOINT}, cmd_buf {cmd_buf:?}");
            self.handle
                .lock()
                .write_bulk(BULK_OUT_ENDPOINT, &cmd_buf, DEFAULT_WRITE_TIMEOUT)?;
        }

        // Write the actual data in chunks
        let mut bytes_written = 0;
        let mut data_slice = &data.as_ref()[..size];

        while bytes_written < size {
            let chunk_size = min(MAX_XFER_SIZE, size - bytes_written);
            let chunk = &data_slice[..chunk_size];

            let written = {
                trace!("Write bulk data length: {}", chunk.len());
                self.handle
                    .lock()
                    .write_bulk(BULK_OUT_ENDPOINT, chunk, DEFAULT_WRITE_TIMEOUT)?
            };

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
            let status = self.wait_for_status()?;
            trace!("Retrieved status 0x{:04x}", status);
            if status != 0 {
                Ok(status as usize)
            } else {
                Err(Error::Communication {
                    kind: CommunicationError::StatusValue { value: status },
                }
                .into())
            }
        } else {
            trace!("Write data succeeded {}", bytes_written);
            Ok(bytes_written)
        }
    }

    fn read_data(&mut self, mode: u8, buffer: &mut [u8]) -> Result<usize, Error> {
        let size = buffer.len();
        trace!("UsbDevice::read_data mode b{mode:08b} buffer.len() {size}");
        self.sw_debug_info.increment_api_call();

        // Check inputs
        if size > MAX_XFER_SIZE {
            let message = format!(
                "Attempted to read {size} more than max supported number of bytes {MAX_XFER_SIZE}"
            );
            warn!("{message}");
            return Err(Error::Args { message });
        }
        let buf_len = buffer.len();
        if size > buf_len {
            let message = format!("Attempted to read {size} more than buffer length {buf_len}");
            warn!("{message}");
            return Err(Error::Args { message });
        }

        // Send read command with 4-byte command buffer
        let cmd_buf = [READ, mode, (size & 0xff) as u8, ((size >> 8) & 0xff) as u8];

        {
            self.handle
                .lock()
                .write_bulk(BULK_OUT_ENDPOINT, &cmd_buf, DEFAULT_WRITE_TIMEOUT)?;
        }

        // Read data in chunks
        let mut bytes_read = 0;
        while bytes_read < size {
            let chunk_size = std::cmp::min(MAX_XFER_SIZE, size - bytes_read);
            let chunk = &mut buffer.as_mut()[bytes_read..bytes_read + chunk_size];

            let read = {
                self.handle
                    .lock()
                    .read_bulk(BULK_IN_ENDPOINT, chunk, DEFAULT_READ_TIMEOUT)?
            };

            bytes_read += read;

            // If we read less than requested, transfer is done
            if read < chunk_size {
                break;
            }
        }

        trace!("Read {} bytes", bytes_read);
        Ok(bytes_read)
    }

    fn wait_for_status(&mut self) -> Result<u16, Error> {
        trace!("UsbDevice::wait_for_status");
        self.sw_debug_info.increment_api_call();

        let mut status_buf = vec![0u8; STATUS_BUF_SIZE];
        let mut timeouts = 0;
        loop {
            trace!("Read bulk endpoint 0x{:02x}", BULK_IN_ENDPOINT);
            let result = {
                self.handle.lock().read_bulk(
                    BULK_IN_ENDPOINT,
                    &mut status_buf,
                    DEFAULT_READ_TIMEOUT,
                )
            };
            match result {
                // The XUM1541 status response continues of 3 bytes
                // The 1st byte is the status of the response - which can be
                // * IO_READY
                // * IO_BUSY
                // * IO_ERROR
                //
                // IO_READY signifies the status value has been provided.
                // IO_BUSY means the device is busy, and the request shouild
                // be retried.
                // IO_ERROR is a failure condition
                //
                // The status value is provied as bytes 2-3 of the response.
                // With low byte followed by high byte
                Ok(len) if len == STATUS_BUF_SIZE => {
                    let status = Self::get_status(&status_buf);

                    match status {
                        IO_READY => {
                            let status = Self::get_status_val(&status_buf);
                            trace!("Got status value 0x{:04x}", status);
                            break Ok(status);
                        }
                        IO_BUSY => {
                            trace!("Device busy");
                            let _ = sleep(DEFAULT_USB_LOOP_SLEEP);
                            continue;
                        }
                        IO_ERROR => {
                            trace!("Device IO error");
                            break Err(Error::Communication {
                                kind: CommunicationError::StatusIo,
                            });
                        }
                        num => {
                            let message = format!("Unexpected status from device {num}");
                            warn!("{message}");
                            break Err(Error::Communication {
                                kind: CommunicationError::StatusResponse { value: num },
                            });
                        }
                    }
                }
                Ok(len) => {
                    trace!(
                        "Device returned unexpected number of status bytes {} vs {}",
                        len,
                        STATUS_BUF_SIZE
                    );
                    break Err(Error::Communication {
                        kind: CommunicationError::StatusFormat,
                    });
                }
                Err(e) => match e {
                    e @ rusb::Error::Timeout => {
                        if timeouts > 5 {
                            // TODO this might be a good time to reset the
                            // device
                            warn!("Too many timeouts waiting for xum1541");
                            break Err(Error::Timeout {
                                dur: DEFAULT_READ_TIMEOUT * timeouts,
                            });
                        }
                        debug!("Timeout waiting for device status {}", e);
                        let _ = sleep(DEFAULT_USB_LOOP_SLEEP);
                        timeouts += 1;
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
    fn ioctl(
        &mut self,
        ioctl: Ioctl,
        address: u8,
        secondary_address: u8,
    ) -> Result<Option<u16>, Error> {
        trace!("UsbDevice::ioctl {ioctl} address {address} secondary_address {secondary_address}");
        self.sw_debug_info.increment_api_call();

        // Build 4-byte command buffer
        let cmd_buf = [ioctl as u8, address, secondary_address, 0u8];

        // Send command
        let result = {
            self.handle
                .lock()
                .write_bulk(BULK_OUT_ENDPOINT, &cmd_buf, DEFAULT_WRITE_TIMEOUT)
        };
        match result {
            Ok(_) => {
                trace!(
                    "Successfully sent ioctl {ioctl} to address {address} secondary_address {secondary_address}",
                );
                ()
            }
            Err(e) => {
                warn!("Failed to send ioctl {ioctl} to address {address} secondary_address {secondary_address}",);
                return Err(e.into());
            }
        }
        if ioctl.is_sync() {
            debug!("ioctl {ioctl} is syncronous, waiting for status");
            Ok(Some(self.wait_for_status()?))
        } else {
            debug!("ioctl {ioctl} is asyncronous, not waiting for status");
            Ok(None)
        }
    }
}

/// Private Device functions
impl UsbDevice {
    /// Enumerate the bus, find the appropriate device and open it
    fn find_device(
        context: &Context,
        serial_num: Option<u8>,
    ) -> Result<(RusbDevice<Context>, RusbDeviceHandle<Context>), Error> {
        trace!("UsbDevice::find_device context {context:?} serial_num {serial_num:?}");

        let mut found_serial_nums = vec![];
        for device in context.devices()?.iter() {
            let device_desc = device.device_descriptor()?;
            trace!(
                "Found USB device {:04x}:{:04x}",
                device_desc.vendor_id(),
                device_desc.product_id()
            );

            if device_desc.vendor_id() == XUM1541_VID && device_desc.product_id() == XUM1541_PID {
                debug!("Found XUM1541 device");
                let handle = device.open()?;

                // Try and read the serial number, whether we are looking for
                // it or not
                trace!("Port number requested {serial_num:?}");
                match handle.read_serial_number_string_ascii(&device_desc) {
                    Ok(serial) => {
                        if let Ok(num) = serial.parse::<u8>() {
                            if serial_num.is_none() {
                                debug!("Was looking for any serial number, found {num}");
                                return Ok((device, handle));
                            } else if Some(num) == serial_num {
                                debug!("Found device with matching serial number {num}");
                                return Ok((device, handle));
                            } else {
                                found_serial_nums.push(num);
                                debug!(
                                    "Device serial number {num} didn't match requested {}",
                                    serial_num.unwrap(),
                                );
                            }
                        } else {
                            warn!("Failed to parse serial number {serial}");
                        }
                    }
                    Err(e) => {
                        info!("Couldn't read device serial number: {}", e);
                        if serial_num.is_none() {
                            // Despite failing to read the serial number
                            // we can still use the device if serial
                            // 0 was specified
                            debug!("Was looking for any serial number, found device with no serial number");
                            return Ok((device, handle));
                        }
                    }
                }
            }
        }

        let err = if found_serial_nums.len() > 0 {
            info!("No matching XUM1541 device found {serial_num:?}, but found non-matching serial(s) {found_serial_nums:?}");
            Error::DeviceAccess {
                kind: DeviceAccessError::SerialMismatch {
                    vid: XUM1541_VID,
                    pid: XUM1541_PID,
                    actual: found_serial_nums,
                    expected: serial_num,
                },
            }
        } else {
            info!("No suitable XUM1541 device found");
            Error::DeviceAccess {
                kind: DeviceAccessError::NotFound {
                    vid: XUM1541_VID,
                    pid: XUM1541_PID,
                },
            }
        };
        Err(err)
    }

    // The device must be (re) initialized after this function.  It is
    // deliberately made a non-public function to avoid external parties
    // accidently resetting and then not re-initializing
    fn hard_reset_pre_init(&self) -> Result<(), Error> {
        trace!("UsbDevice::hard_reset_pre_init");
        debug!("Hard reset the device");
        {
            self.handle.lock().reset()?;
        }
        sleep(DEFAULT_USB_RESET_SLEEP);
        trace!("Reset should be commplete, continue");
        Ok(())
    }

    /// Initialize device and get info
    fn initialize_device(&mut self) -> Result<(DeviceInfo, UsbInfo), Error> {
        trace!("UsbDevice::initialize_device");

        // Hard reset the device
        // This fixes a multitude of problems - including
        // - Kicks anyone else off this USB device (assuming we have
        //   permissions)
        // - Seems to avoid various USB errors talking to the device if it's
        //   been left in a dodgy state.
        //self.hard_reset_pre_init()?;

        let (device_info, usb_info) = self.verify_and_setup_device()?;
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
                        warn!("Failed to read debug info from device: {}", error);
                        return Err(error);
                    }
                    e => {
                        // Hit some sort of processing error - we'll treat this as a non-failure mode
                        info!("Failed to read debug info from device: {}", e);
                        None
                    }
                },
            };
        } else {
            debug!("Firmware earlier than 8 doesn't support extra debug info");
        }

        Ok((info, usb_info))
    }

    fn clear_halt(&self) -> Result<(), Error> {
        trace!("UsbDevice::clear_halt");

        let result = { self.handle.lock().clear_halt(BULK_IN_ENDPOINT) };
        match result {
            Ok(_) => (),
            Err(e) => {
                warn!("USB clear halt request failed for in endpoint: {}", e);
                return Err(e.into());
            }
        }

        let result = { self.handle.lock().clear_halt(BULK_OUT_ENDPOINT) };
        match result {
            Ok(_) => Ok(()),
            Err(e) => {
                warn!("USB clear halt request failed for out endpoint: {}", e);
                Err(e.into())
            }
        }
    }

    fn get_status(buf: &[u8]) -> u8 {
        trace!("UsbDevice::get_status buf.len() {}", buf.len());
        assert!(buf.len() >= 1);
        buf[0]
    }

    fn get_status_val(buf: &[u8]) -> u16 {
        trace!("UsbDevice::get_status_val buf.len() {}", buf.len());
        assert!(buf.len() >= 3);
        u16::from(buf[2]) << 8 | u16::from(buf[1])
    }

    /// Verify device identity and set up initial configuration
    fn verify_and_setup_device(&mut self) -> Result<(DeviceInfo, UsbInfo), Error> {
        // TO DO - this function is a bit of a mess.  Would prob be better
        // to default() the structs we need up front as mut and then add to
        // as we go through, rather than pass info we want back from locking
        // scopes.

        trace!("UsbDevice::verify_and_setup_device");
        let (device_desc, bus_number, address) = {
            let guard = self.handle.lock();

            let device = guard.device();
            (
                device.device_descriptor()?,
                device.bus_number(),
                device.address(),
            )
        };

        // Read device strings
        let product = self.read_product_string(&device_desc)?;

        let (manufacturer, serial_number) = {
            let guard = self.handle.lock();
            let manufacturer = guard.read_manufacturer_string_ascii(&device_desc).ok();
            let serial_number = guard.read_serial_number_string_ascii(&device_desc).ok();
            (manufacturer, serial_number)
        };

        // Verify product string
        debug!("Product string: {}", product);
        if !PRODUCT_STRINGS.iter().any(|s| product.contains(s)) {
            return Err(Error::Init {
                message: format!("Device product string {} didn't contain an expected value", product),
            });
        }

        let device_info = DeviceInfo {
            product,
            manufacturer,
            serial_number,
            firmware_version: 0, // Will be set later
            capabilities: 0,     // Will be set later
            status: 0,           // Will be set later
            debug_info: None,    // Will be set later
        };

        // Set up USB device info
        let usb_info = UsbInfo {
            vendor_id: device_desc.vendor_id(),
            product_id: device_desc.product_id(),
            bus_number: bus_number,
            device_address: address,
        };

        // Configure device
        self.setup_device_configuration()?;

        // Set up device info

        Ok((device_info, usb_info))
    }

    /// Read product string with error handling
    fn read_product_string(&self, device_desc: &DeviceDescriptor) -> Result<String, Error> {
        trace!("UsbDevice::read_product_string device_desc {device_desc:?}");
        self.handle
            .lock()
            .read_product_string_ascii(device_desc)
            .inspect_err(|e|
                warn!("Hit error trying to read xum1541 product string: {e:?}")
            )
            .map_err(|_| Error::Init {
                message: "Couldn't read device product string".into(),
            })
    }

    /// Set up device configuration and claim interface
    fn setup_device_configuration(&mut self) -> Result<(), Error> {
        trace!("UsbDevice::setup_device_configuration");
        {
            let guard = self.handle.lock();

            let config = guard.active_configuration()?;
            debug!("Current configuration is {}", config);
            if config != 1 {
                debug!("Set active configuration to 1");
                guard.set_active_configuration(1)?;
            }
        }

        debug!("Claim interface");
        self.handle.lock().claim_interface(0)?;
        Ok(())
    }

    /// Read basic device information
    fn read_basic_info(&mut self, initial_info: &DeviceInfo) -> Result<DeviceInfo, Error> {
        trace!("UsbDevice::read_basic_info initial_info {initial_info:?}");
        let mut info_buf = [0u8; DEV_INFO_SIZE];
        let len = self.read_control(CTRL_INIT as u8, 0, &mut info_buf)?;

        trace!(
            "Control transfer succeeded, got {} bytes: {:?}",
            len,
            info_buf
        );

        // Now get the device info
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
            return Err(Error::DeviceAccess {
                kind: DeviceAccessError::FirmwareVersion {
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
        trace!("UsbDevice::read_debug_info");
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
        match self.read_control(cmd, 0, info_buf) {
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

    #[allow(dead_code)]
    fn check_and_clear_stall(&mut self) -> Result<(), Error> {
        trace!("UsbDevice::check_and_clear_stall");
        let handle = self.handle.lock();
        for ep in [BULK_IN_ENDPOINT, BULK_OUT_ENDPOINT].iter() {
            // Direction is always IN for GET_STATUS, even for OUT endpoint
            let direction = constants::LIBUSB_ENDPOINT_IN;
            let mut status = [0u8; 2];
            let request_type = constants::LIBUSB_REQUEST_TYPE_STANDARD |
            constants::LIBUSB_RECIPIENT_ENDPOINT |
            direction;
            trace!("Read control request ep: 0x{ep:02x} type: 0x{:02x}", request_type);
            handle.read_control(
                    request_type,
                    constants::LIBUSB_REQUEST_GET_STATUS,
                    0, // no specific value needed for GET_STATUS
                    *ep as u16,
                    &mut status,
                    DEFAULT_CONTROL_TIMEOUT,
                )
                .map_err(Error::from)?;

            let is_stalled = (status[0] & 1) != 0;

            if is_stalled {
                info!("Endpoint 0x{ep:02x} is stalled - clearing");
            } else {
                debug!("Endpoint 0x{ep:02x} is not stalled - clearing anyway", ep = ep);
            }

            // Direction is always OUT for CLEAR_FEATURE, even for IN
            // endpoint
            let direction = constants::LIBUSB_ENDPOINT_OUT;
            let request_type = constants::LIBUSB_REQUEST_TYPE_STANDARD |
            constants::LIBUSB_RECIPIENT_ENDPOINT |
            direction;
            trace!("Write control request 0x{:02x}", request_type);
            handle.write_control(
                    request_type,
                    constants::LIBUSB_REQUEST_CLEAR_FEATURE,
                    0, // ENDPOINT_HALT feature selector
                    *ep as u16,
                    &[],
                    DEFAULT_CONTROL_TIMEOUT,
            )
            .map_err(Error::from)?;
        }
        Ok(())
    }
}

impl Drop for UsbDevice {
    fn drop(&mut self) {
        trace!("UsbDevice::drop");
        // Send shutdown command - ignoring errors since we're in drop
        let _ = self.write_control(CTRL_SHUTDOWN, 0, &[]);

        // Release interface - again ignoring errors
        let _ = self.handle.lock().release_interface(0);
        trace!("Exited drop");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_device_not_found() {
        let config = UsbDeviceConfig {
            context: None,
            serial_num: Some(99),
        };
        let result = UsbDevice::new(Some(config));
        println!("{result:?}");
        assert!(matches!(
            // If no XUM1541 is connected
            result,
            Err(Error::DeviceAccess {
                kind: DeviceAccessError::NotFound {
                    vid: XUM1541_VID,
                    pid: XUM1541_PID
                }
            }) |
            // If a device with serial anything other than 99 is connected
            Err(Error::DeviceAccess {
                kind: DeviceAccessError::SerialMismatch {
                    vid: XUM1541_VID,
                    pid: XUM1541_PID,
                    actual: _,
                    expected: Some(99)
                }
            })
        ));
    }
}
