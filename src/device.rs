use crate::{Error, Result};
use crate::{
    BULK_IN_ENDPOINT, BULK_OUT_ENDPOINT, CAP_CBM, CAP_IEEE488, CAP_NIB, CAP_NIB_SRQ, CAP_TAP,
    CTRL_GCCVER, CTRL_GITREV, CTRL_INIT, CTRL_LIBCVER, CTRL_RESET, CTRL_SHUTDOWN,
    DEFAULT_CONTROL_TIMEOUT, DEFAULT_READ_TIMEOUT, DEFAULT_USB_LOOP_SLEEP, DEFAULT_USB_RESET_SLEEP,
    DEFAULT_WRITE_TIMEOUT, IO_BUSY, IO_ERROR, IO_READY, MIN_FW_VERSION, PROTO_CBM, PROTO_MASK,
    READ, STATUS_DOING_RESET, STATUS_IEEE488_PRESENT, STATUS_TAPE_PRESENT, WRITE, XUM1541_PID,
    XUM1541_VID, XUM_DEVINFO_SIZE, XUM_MAX_XFER_SIZE, XUM_STATUSBUF_SIZE,
};
use rusb::Device as RusbDevice;
use rusb::DeviceHandle as RusbDeviceHandle;
use rusb::{constants, Context, DeviceDescriptor, UsbContext};
use std::cmp::min;
use std::str::from_utf8;
use std::thread::sleep;
#[allow(unused_imports)]
use tracing::{debug, error, info, trace, warn};

#[derive(Debug)]
pub struct DeviceDebugInfo {
    pub git_rev: Option<String>,
    pub gcc_ver: Option<String>,
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

#[derive(Debug)]
pub struct DeviceInfo {
    pub product: String,
    pub manufacturer: Option<String>,
    pub serial_number: Option<String>,
    pub firmware_version: u8,
    pub capabilities: u8,
    pub status: u8,
    pub debug_info: Option<DeviceDebugInfo>,
}

impl DeviceInfo {
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

#[derive(Debug)]

pub struct Device {
    handle: RusbDeviceHandle<Context>,
    info: Option<DeviceInfo>,
    context: Context,
}

impl Device {
    pub fn new(serial_num: u8) -> Result<Self> {
        let mut context = Context::new()?;
        context.set_log_level(rusb::LogLevel::Debug);
        Self::with_context(context, serial_num)
    }

    pub fn context(&self) -> &Context {
        &self.context
    }

    pub fn info(&self) -> Option<&DeviceInfo> {
        self.info.as_ref()
    }

    /// Create new device connection with provided context
    pub fn with_context(context: Context, serial_num: u8) -> Result<Self> {
        let (_device, handle) = Self::find_device(&context, serial_num)?;

        Ok(Self {
            handle,
            info: None,
            context,
        })
    }

    pub fn init(&mut self) -> Result<()> {
        self.info = Some(self.initialize_device()?);
        Ok(())
    }

    /// Find XUM1541 device and get handle
    fn find_device(
        context: &Context,
        serial_num: u8,
    ) -> Result<(RusbDevice<Context>, RusbDeviceHandle<Context>)> {
        debug!(
            "Searching for XUM1541 device with serial_num: {}",
            serial_num
        );

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

                // Check serial number if serial_num specified
                if serial_num > 0 {
                    trace!("Port number requested {}", serial_num);
                    match handle.read_serial_number_string_ascii(&device_desc) {
                        Ok(serial) => {
                            debug!("Device serial number: {}", serial);
                            if let Ok(num) = serial.parse::<u8>() {
                                if num == serial_num {
                                    info!("Found device with matching serial number");
                                    return Ok((device, handle));
                                }
                                found_serial_nums.push(num);
                                debug!(
                                    "Serial number {} didn't match requested {}",
                                    num, serial_num
                                );
                            }
                        }
                        Err(e) => {
                            warn!("Couldn't read device serial number: {}", e)
                        }
                    }
                } else {
                    debug!("Taking first available device");
                    return Ok((device, handle));
                }
            }
        }

        let err = match found_any_xum1541 {
            true => {
                error!("No matching XUM1541 device found, but found non-matching serial");
                Error::SerialMismatch {
                    vid: XUM1541_VID,
                    pid: XUM1541_PID,
                    actual: found_serial_nums,
                    expected: serial_num,
                }
            }
            false => {
                error!("No suitable XUM1541 device found");
                Error::DeviceNotFound {
                    vid: XUM1541_VID,
                    pid: XUM1541_PID,
                }
            }
        };
        Err(err)
    }

    /// Do a hard reset of the USB device.
    ///
    /// This function can fail, in which case you may be in a worse state
    /// than before.
    ///
    /// DeviceInfo may change during this process.
    pub fn hard_reset_and_re_init(&mut self) -> Result<()> {
        // Drop the info - this will be reinitialized shortly
        // Resetting the device doesn't require this
        self.info = None;

        self.hard_reset_pre_init()?;

        self.init()
    }

    // The device must be (re) initialized after this function.  It is
    // deliberately made a non-public function to avoid external parties
    // accidently resetting and then not re-initializing
    fn hard_reset_pre_init(&self) -> Result<()> {
        info!("Hard reset the device");
        self.handle.reset()?;
        sleep(DEFAULT_USB_RESET_SLEEP);
        trace!("Reset should be commplete, continue");
        Ok(())
    }

    /// Initialize device and get info
    fn initialize_device(&mut self) -> Result<DeviceInfo> {
        debug!("Starting device initialization");

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
            info.debug_info = Some(self.read_debug_info()?);
        } else {
            debug!("Firmware earlier than 8 doesn't support extra debug info");
        }

        Ok(info)
    }

    fn clear_halt(&self) -> Result<()> {
        // Clear halt on both endpoints

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
        buf[0]
    }

    fn get_status_val(buf: &[u8]) -> u16 {
        u16::from(buf[2]) << 8 | u16::from(buf[1])
    }

    pub fn control_write(&self, request: u8, value: u16, buffer: &[u8]) -> Result<()> {
        trace!("Entered control_write");
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

    /// Implements xum1541_control_mg - reading type
    pub fn control_read(&self, request: u8, value: u16, buffer: &mut [u8]) -> Result<usize> {
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

    /// Implements IOctl()
    pub fn command_only(&self, cmd: u8, addr: u8, secaddr: u8) -> Result<u16> {
        // Build 4-byte command buffer
        let cmd_buf = [cmd, addr, secaddr, 0u8];

        // Send command
        match self
            .handle
            .write_bulk(BULK_OUT_ENDPOINT, &cmd_buf, DEFAULT_WRITE_TIMEOUT)
        {
            Ok(_) => {
                trace!(
                    "Successfully send command {} to device {} {}",
                    cmd,
                    addr,
                    secaddr
                );
                ()
            }
            Err(e) => {
                warn!(
                    "Failed to send command {} to device {} {}",
                    cmd, addr, secaddr
                );
                return Err(e.into());
            }
        }

        self.wait_status()
    }

    fn wait_status(&self) -> Result<u16> {
        debug!("Wait status");
        let mut status_buf = vec![0u8; XUM_STATUSBUF_SIZE];
        loop {
            trace!("Read bulk endpoint 0x{:02x}", BULK_IN_ENDPOINT);
            match self
                .handle
                .read_bulk(BULK_IN_ENDPOINT, &mut status_buf, DEFAULT_READ_TIMEOUT)
            {
                Ok(len) if len == XUM_STATUSBUF_SIZE => {
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
                            break Err(Error::CommunicationError {
                                message: "Device returned IO error".into(),
                            });
                        }
                        num => {
                            warn!("Unexpected debug status {}", num);
                            break Err(Error::CommunicationError {
                                message: format!("Unexpected error from device {}", num),
                            });
                        }
                    }
                }
                Ok(len) => {
                    trace!(
                        "Device returned wrong number of bytes {} vs {}",
                        len,
                        XUM_STATUSBUF_SIZE
                    );
                    break Err(Error::CommunicationError {
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

    /// Implements WRITE()
    pub fn write_data(&self, mode: u8, data: &[u8]) -> Result<usize> {
        let size = data.len();
        if size > XUM_MAX_XFER_SIZE {
            return Err(Error::SizeTooLarge {
                attempt: size,
                max: XUM_MAX_XFER_SIZE,
            });
        }

        // Send the write command with 4-byte command buffer
        let cmd_buf = [WRITE, mode, (size & 0xff) as u8, ((size >> 8) & 0xff) as u8];

        self.handle
            .write_bulk(BULK_OUT_ENDPOINT, &cmd_buf, DEFAULT_WRITE_TIMEOUT)?;

        // Write the actual data in chunks
        let mut bytes_written = 0;
        let mut data_slice = &data.as_ref()[..size];

        while bytes_written < size {
            let chunk_size = min(XUM_MAX_XFER_SIZE, size - bytes_written);
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

    /// Implements xum1541_read()
    pub fn read_data(&self, mode: u8, buffer: &mut [u8], size: usize) -> Result<usize> {
        if size > XUM_MAX_XFER_SIZE {
            return Err(Error::SizeTooLarge {
                attempt: size,
                max: XUM_MAX_XFER_SIZE,
            });
        }
        if size > buffer.len() {
            return Err(Error::SizeTooLarge {
                attempt: size,
                max: buffer.len(),
            });
        }
        // Send read command with 4-byte command buffer
        let cmd_buf = [READ, mode, (size & 0xff) as u8, ((size >> 8) & 0xff) as u8];

        self.handle
            .write_bulk(BULK_OUT_ENDPOINT, &cmd_buf, DEFAULT_WRITE_TIMEOUT)?;

        // Read data in chunks
        let mut bytes_read = 0;
        while bytes_read < size {
            let chunk_size = std::cmp::min(XUM_MAX_XFER_SIZE, size - bytes_read);
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

        debug!("Read {} bytes", bytes_read);
        Ok(bytes_read)
    }

    /// Verify device identity and set up initial configuration
    fn verify_and_setup_device(&mut self) -> Result<DeviceInfo> {
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
            return Err(Error::InitError {
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
    fn read_product_string(&self, device_desc: &DeviceDescriptor) -> Result<String> {
        self.handle
            .read_product_string_ascii(device_desc)
            .map_err(|_| Error::InitError {
                message: "Couldn't read device product string".into(),
            })
    }

    /// Set up device configuration and claim interface
    fn setup_device_configuration(&mut self) -> Result<()> {
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
    fn read_basic_info(&mut self, initial_info: &DeviceInfo) -> Result<DeviceInfo> {
        let mut info_buf = [0u8; XUM_DEVINFO_SIZE];
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
            return Err(Error::FirmwareVersion {
                actual: info.firmware_version,
                expected: MIN_FW_VERSION,
            });
        }

        Ok(info)
    }

    /// Read debug information from device
    fn read_debug_info(&mut self) -> Result<DeviceDebugInfo> {
        let ver_cmds = [CTRL_GITREV, CTRL_GCCVER, CTRL_LIBCVER];
        let mut debug_info = DeviceDebugInfo::default();
        let mut info_buf = [0u8; XUM_DEVINFO_SIZE];

        // Process each command independently, ignoring failures
        for cmd in ver_cmds {
            if let Ok(debug_str) = self.read_debug_command(cmd, &mut info_buf) {
                self.update_debug_info(&mut debug_info, cmd, debug_str);
            }
        }

        Ok(debug_info)
    }

    /// Read a single debug command
    fn read_debug_command(&mut self, cmd: u8, info_buf: &mut [u8]) -> Result<String> {
        match self.control_read(cmd, 0, info_buf) {
            Ok(len) if len > 0 => from_utf8(info_buf)
                .map(|s| s.trim_matches(char::from(0)).to_string())
                .map_err(|_| Error::InitError {
                    message: "Invalid UTF-8 in debug info".into(),
                }),
            Ok(_) => {
                debug!("No debug data for command {:?}", cmd);
                Err(Error::InitError {
                    message: "No debug data available".into(),
                })
            }
            Err(e) => {
                debug!("Failed to read debug info for command {:?}: {:?}", cmd, e);
                Err(Error::InitError {
                    message: format!("Failed to read debug info: {}", e),
                })
            }
        }
    }

    /// Update debug info structure with command results
    fn update_debug_info(&self, debug_info: &mut DeviceDebugInfo, cmd: u8, debug_str: String) {
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
        trace!("Entered drop");
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
            Err(Error::DeviceNotFound {
                vid: XUM1541_VID,
                pid: XUM1541_PID
            })
        ));
    }
}
