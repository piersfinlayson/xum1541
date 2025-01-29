#[allow(unused_imports)]
use crate::constants::*;
use crate::error::{CommunicationKind, InternalError};
use crate::DeviceAccessKind::*;
use crate::Error::{self, *};
use crate::{Device, DeviceDebugInfo, DeviceInfo};

#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};
use rusb::Device as RusbDevice;
use rusb::DeviceHandle as RusbDeviceHandle;
use rusb::{constants, Context, DeviceDescriptor, UsbContext};
use std::cmp::min;
use std::str::from_utf8;
use std::thread::sleep;

use super::SpecificDeviceInfo;

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
#[derive(Debug)]
pub struct UsbDevice {
    handle: RusbDeviceHandle<Context>,
    config: UsbDeviceConfig,
    info: Option<DeviceInfo>,
    usb_info: Option<UsbInfo>,
}

#[derive(Debug)]
pub struct UsbDeviceConfig {
    /// The [`rusb::Context`] to use for this device.  May be None to use
    /// default context.
    pub context: Option<Context>,

    /// The serial number of the device to use.  If omitted, the first
    /// detected device of the correct type will be used.  If set to 0, only
    /// a device with a zero serial number will be matched.
    pub serial_num: Option<u8>,
}

#[derive(Debug)]
pub struct UsbInfo {
    pub vendor_id: u16,
    pub product_id: u16,
    pub bus_number: u8,
    pub device_address: u8,
}

impl SpecificDeviceInfo for UsbDevice {
    type Info = UsbInfo;

    fn specific_info(&self) -> Option<&Self::Info> {
        if let Some(info) = &self.usb_info {
            Some(&info)
        } else {
            None
        }
    }

    fn print_specific(&self) {
        if let Some(info) = self.specific_info() {
            println!("  - device: {:04x}:{:04x}", info.vendor_id, info.product_id);
            println!(
                "  - bus/address: {:03}-{:03}",
                info.bus_number, info.device_address
            );
        } else {
            println!("  - no USB device information");
        }
    }
}

/// Public Device functions
impl Device for UsbDevice {
    type Config = UsbDeviceConfig;

    fn new(config: Option<Self::Config>) -> Result<Self, Error> {
        trace!("Device::new");

        // Set up a config object if we don't already have one
        let mut config = if let Some(config) = config {
            trace!("Device::new config {:?}", config);
            config
        } else {
            trace!("Device::new default config");
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
            handle,
            config,
            info: None,
            usb_info: None,
        })
    }

    fn current_config(&self) -> Option<&Self::Config> {
        trace!("Device::current_config");
        Some(&self.config)
    }

    fn init(&mut self) -> Result<(), Error> {
        trace!("Device::init");
        let (info, usb_info) = self.initialize_device()?;
        self.info = Some(info);
        self.usb_info = Some(usb_info);
        Ok(())
    }

    fn info(&self) -> Option<&DeviceInfo> {
        trace!("Device::info");
        self.info.as_ref()
    }

    fn hard_reset_and_re_init(&mut self) -> Result<(), Error> {
        trace!("Device::hard_reset_and_re_init");
        // Drop the info - this will be reinitialized shortly
        // Resetting the device doesn't require this
        self.info = None;

        self.hard_reset_pre_init()?;

        self.init()
    }

    fn read_control(&self, request: u8, value: u16, buffer: &mut [u8]) -> Result<usize, Error> {
        trace!(
            "Device::read_control request 0x{request:02x} value 0x{value:02x} buffer.len() {}",
            buffer.len()
        );

        // Class-specific request, device as recipient, with IN direction
        const REQUEST_TYPE: u8 = constants::LIBUSB_REQUEST_TYPE_CLASS
            | constants::LIBUSB_RECIPIENT_ENDPOINT
            | constants::LIBUSB_ENDPOINT_IN;

        // Perform control transfer and return number of bytes read
        let index = 0; // default control endpoint
        self.handle
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

    fn write_control(&self, request: u8, value: u16, buffer: &[u8]) -> Result<(), Error> {
        trace!(
            "Device::write_control request 0x{request:02x} value 0x{value:02x} buffer.len() {}",
            buffer.len()
        );
        const REQUEST_TYPE: u8 = constants::LIBUSB_REQUEST_TYPE_CLASS
            | constants::LIBUSB_RECIPIENT_ENDPOINT
            | constants::LIBUSB_ENDPOINT_OUT;

        // Send control transfer
        let index = 0; // default control endpoint
        self.handle.write_control(
            REQUEST_TYPE,
            request,
            value,
            index,
            buffer,
            DEFAULT_CONTROL_TIMEOUT,
        )?;

        // Device doesn't seem to want to respond to use if we've shut it
        // down or called reset
        if (request != CTRL_SHUTDOWN) && (request != CTRL_RESET) {
            // Wait for status response
            let status = self.wait_for_status()?;
            if status == 0 {
                return Err(Communication {
                    kind: CommunicationKind::StatusValue { value: status },
                }
                .into());
            }
        }

        trace!("Exited control_write - success");
        Ok(())
    }

    fn write_data(&self, mode: u8, data: &[u8]) -> Result<usize, Error> {
        let size = data.len();
        trace!("Device::write_data mode b{mode:08b} data.len() {size}");

        // Validate inputs
        if size > MAX_XFER_SIZE {
            let message = format!(
                "Attempted to write {size} more than max supported number of bytes {MAX_XFER_SIZE}"
            );
            warn!("{message}");
            return Err(Args { message });
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
            let status = self.wait_for_status()?;
            trace!("Retrieved status 0x{:04x}", status);
            if status != 0 {
                Ok(status as usize)
            } else {
                Err(Communication {
                    kind: CommunicationKind::StatusValue { value: status },
                }
                .into())
            }
        } else {
            Ok(bytes_written)
        }
    }

    fn read_data(&self, mode: u8, buffer: &mut [u8]) -> Result<usize, Error> {
        let size = buffer.len();
        trace!("Device::read_data mode b{mode:08b} buffer.len() {size}");

        // Check inputs
        if size > MAX_XFER_SIZE {
            let message = format!(
                "Attempted to read {size} more than max supported number of bytes {MAX_XFER_SIZE}"
            );
            warn!("{message}");
            return Err(Args { message });
        }
        let buf_len = buffer.len();
        if size > buf_len {
            let message = format!("Attempted to read {size} more than buffer length {buf_len}");
            warn!("{message}");
            return Err(Args { message });
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

    fn wait_for_status(&self) -> Result<u16, Error> {
        trace!("Device::wait_for_status");

        let mut status_buf = vec![0u8; STATUS_BUF_SIZE];
        let mut timeouts = 0;
        loop {
            trace!("Read bulk endpoint 0x{:02x}", BULK_IN_ENDPOINT);
            match self
                .handle
                .read_bulk(BULK_IN_ENDPOINT, &mut status_buf, DEFAULT_READ_TIMEOUT)
            {
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
                            break Err(Communication {
                                kind: CommunicationKind::StatusIo,
                            });
                        }
                        num => {
                            let message = format!("Unexpected status from device {num}");
                            warn!("{message}");
                            break Err(Communication {
                                kind: CommunicationKind::StatusResponse { value: num },
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
                    break Err(Communication {
                        kind: CommunicationKind::StatusFormat,
                    });
                }
                Err(e) => match e {
                    e @ rusb::Error::Timeout => {
                        if timeouts > 5 {
                            // TODO this might be a good time to reset the
                            // device
                            warn!("Too many timeouts waiting for xum1541");
                            break Err(Timeout {
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
        &self,
        ioctl: Ioctl,
        address: u8,
        secondary_address: u8,
    ) -> Result<Option<u16>, Error> {
        trace!("Device::ioctl {ioctl} address {address} secondary_address {secondary_address}");

        // Build 4-byte command buffer
        let cmd_buf = [ioctl as u8, address, secondary_address, 0u8];

        // Send command
        match self
            .handle
            .write_bulk(BULK_OUT_ENDPOINT, &cmd_buf, DEFAULT_WRITE_TIMEOUT)
        {
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
        trace!("Device::find_device context {context:?} serial_num {serial_num:?}");

        let mut found_serial_nums = vec![];
        let serial_num = serial_num.unwrap_or(0);
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
                            if num == serial_num {
                                debug!("Found device with matching serial number {num}");
                                return Ok((device, handle));
                            }
                            found_serial_nums.push(num);
                            debug!(
                                "Device serial number {num} didn't match requested {serial_num}",
                            );
                        }
                    }
                    Err(e) => {
                        info!("Couldn't read device serial number: {}", e);
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

        let err = if found_serial_nums.len() > 0 {
            info!("No matching XUM1541 device found {serial_num}, but found non-matching serial(s) {found_serial_nums:?}");
            DeviceAccess {
                kind: SerialMismatch {
                    vid: XUM1541_VID,
                    pid: XUM1541_PID,
                    actual: found_serial_nums,
                    expected: serial_num,
                },
            }
        } else {
            info!("No suitable XUM1541 device found");
            DeviceAccess {
                kind: NotFound {
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
        trace!("Device::hard_reset_pre_init");
        debug!("Hard reset the device");
        self.handle.reset()?;
        sleep(DEFAULT_USB_RESET_SLEEP);
        trace!("Reset should be commplete, continue");
        Ok(())
    }

    /// Initialize device and get info
    fn initialize_device(&mut self) -> Result<(DeviceInfo, UsbInfo), Error> {
        trace!("Device::initialize_device");

        // Hard reset the device
        // This fixes a multitude of problems - including
        // - Kicks anyone else off this USB device (assuming we have
        //   permissions)
        // - Seems to avoid various USB errors talking to the device if it's
        //   been left in a dodgy state.
        self.hard_reset_pre_init()?;

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
        trace!("Device::clear_halt");

        match self.handle.clear_halt(BULK_IN_ENDPOINT) {
            Ok(_) => (),
            Err(e) => {
                warn!("USB clear halt request failed for in endpoint: {}", e);
                return Err(e.into());
            }
        }

        match self.handle.clear_halt(BULK_OUT_ENDPOINT) {
            Ok(_) => Ok(()),
            Err(e) => {
                warn!("USB clear halt request failed for out endpoint: {}", e);
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

    /// Verify device identity and set up initial configuration
    fn verify_and_setup_device(&mut self) -> Result<(DeviceInfo, UsbInfo), Error> {
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

        // Set up device info
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
            bus_number: device.bus_number(),
            device_address: device.address(),
        };

        Ok((device_info, usb_info))
    }

    /// Read product string with error handling
    fn read_product_string(&self, device_desc: &DeviceDescriptor) -> Result<String, Error> {
        trace!("Device::read_product_string device_desc {device_desc:?}");
        self.handle
            .read_product_string_ascii(device_desc)
            .map_err(|_| Init {
                message: "Couldn't read device product string".into(),
            })
    }

    /// Set up device configuration and claim interface
    fn setup_device_configuration(&mut self) -> Result<(), Error> {
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
    fn read_basic_info(&mut self, initial_info: &DeviceInfo) -> Result<DeviceInfo, Error> {
        trace!("Device::read_basic_info initial_info {initial_info:?}");
        let mut info_buf = [0u8; DEV_INFO_SIZE];
        let len = self.read_control(CTRL_INIT as u8, 0, &mut info_buf)?;

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
}

impl Drop for UsbDevice {
    fn drop(&mut self) {
        trace!("Device::drop");
        // Send shutdown command - ignoring errors since we're in drop
        let _ = self.write_control(CTRL_SHUTDOWN, 0, &[]);

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
        let config = UsbDeviceConfig {
            context: None,
            serial_num: Some(99),
        };
        let result = UsbDevice::new(Some(config));
        println!("{result:?}");
        assert!(matches!(
            // If no XUM1541 is connected
            result,
            Err(DeviceAccess {
                kind: NotFound {
                    vid: XUM1541_VID,
                    pid: XUM1541_PID
                }
            }) |
            // If a device with serial anything other than 99 is connected
            Err(DeviceAccess {
                kind: SerialMismatch {
                    vid: XUM1541_VID,
                    pid: XUM1541_PID,
                    actual: _,
                    expected: 99
                }
            })
        ));
    }
}
