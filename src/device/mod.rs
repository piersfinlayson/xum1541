//! The [`Device`] module provides the low-level interface to the XUM1541 USB adapter.  Prefer [`crate::Bus`] for most use cases.
//!
//! It is unlikely you need to use this interface directly unless you are
//! re-implementing [`crate::Bus`] or adding to it.

pub mod remoteusb;
pub mod usb;

use serde::{Deserialize, Serialize};

pub use remoteusb::RemoteUsbDevice;
pub use usb::{UsbDevice, UsbInfo};

use crate::{constants::*, RemoteUsbInfo};
use crate::{Error, Ioctl};
use crate::{RemoteUsbDeviceConfig, UsbDeviceConfig};

#[derive(Debug, Clone)]
pub enum DeviceType {
    Usb(UsbDevice),
    RemoteUsb(RemoteUsbDevice),
}

#[derive(Debug, Clone)]
pub enum DeviceConfig {
    Usb(UsbDeviceConfig),
    RemoteUsb(RemoteUsbDeviceConfig),
}

impl DeviceType {
    pub fn new(config: DeviceConfig) -> Result<Self, Error> {
        match config {
            DeviceConfig::Usb(config) => UsbDevice::new(Some(config)).map(Self::Usb),
            DeviceConfig::RemoteUsb(config) => {
                RemoteUsbDevice::new(Some(config)).map(Self::RemoteUsb)
            }
        }
    }

    pub fn init(&mut self) -> Result<(), Error> {
        match self {
            Self::Usb(device) => device.init(),
            Self::RemoteUsb(device) => device.init(),
        }
    }

    pub fn info(&mut self) -> Option<DeviceInfo> {
        match self {
            Self::Usb(device) => device.info(),
            Self::RemoteUsb(device) => device.info(),
        }
    }

    pub fn usb_info(&mut self) -> Option<UsbInfo> {
        match self {
            Self::Usb(device) => device.specific_info(),
            Self::RemoteUsb(device) => device.specific_info().and_then(|i| i.usb_info),
        }
    }

    pub fn remote_usb_info(&mut self) -> Option<RemoteUsbInfo> {
        match self {
            Self::Usb(_) => None,
            Self::RemoteUsb(device) => device.specific_info(),
        }
    }

    pub fn sw_debug_info(&mut self) -> SwDebugInfo {
        match self {
            Self::Usb(device) => device.sw_debug_info(),
            Self::RemoteUsb(device) => device.sw_debug_info(),
        }
    }

    pub fn hard_reset_and_re_init(&mut self) -> Result<(), Error> {
        match self {
            Self::Usb(device) => device.hard_reset_and_re_init(),
            Self::RemoteUsb(device) => device.hard_reset_and_re_init(),
        }
    }

    pub fn read_control(
        &mut self,
        request: u8,
        value: u16,
        buffer: &mut [u8],
    ) -> Result<usize, Error> {
        match self {
            Self::Usb(device) => device.read_control(request, value, buffer),
            Self::RemoteUsb(device) => device.read_control(request, value, buffer),
        }
    }

    pub fn write_control(&mut self, request: u8, value: u16, buffer: &[u8]) -> Result<(), Error> {
        match self {
            Self::Usb(device) => device.write_control(request, value, buffer),
            Self::RemoteUsb(device) => device.write_control(request, value, buffer),
        }
    }

    pub fn write_data(&mut self, mode: u8, data: &[u8]) -> Result<usize, Error> {
        match self {
            Self::Usb(device) => device.write_data(mode, data),
            Self::RemoteUsb(device) => device.write_data(mode, data),
        }
    }

    pub fn read_data(&mut self, mode: u8, buffer: &mut [u8]) -> Result<usize, Error> {
        match self {
            Self::Usb(device) => device.read_data(mode, buffer),
            Self::RemoteUsb(device) => device.read_data(mode, buffer),
        }
    }

    pub fn wait_for_status(&mut self) -> Result<u16, Error> {
        match self {
            Self::Usb(device) => device.wait_for_status(),
            Self::RemoteUsb(device) => device.wait_for_status(),
        }
    }

    pub fn ioctl(
        &mut self,
        ioctl: Ioctl,
        address: u8,
        secondary_address: u8,
    ) -> Result<Option<u16>, Error> {
        match self {
            Self::Usb(device) => device.ioctl(ioctl, address, secondary_address),
            Self::RemoteUsb(device) => device.ioctl(ioctl, address, secondary_address),
        }
    }
}

pub trait Config: std::fmt::Debug {}

/// The core Device trait, which allows Device to be mocked out for testing
pub trait Device: std::fmt::Debug + Send + Clone {
    type Config;
    type SpecificDeviceInfo;

    /// Creates a new Device using the provided config, which can be
    /// omitted in order to create a Device using default configuration.
    ///
    /// # Arguments
    /// * `config` - Configuration for the device, or None for default
    /// * `serial_num` - Serial number to match, or None to use first available device
    ///
    /// # Returns
    /// * `Ok(Device)` - Successfully created device instance
    /// * `Err(Error)` - If device creation fails (no device found or USB error)
    ///
    /// Note that the Device returned has not been initialized.  It must be
    /// initialized before use with [`Device::init`].
    ///
    /// # Example
    ///
    /// ```rust,no_run
    /// use xum1541::{Device, UsbDevice};
    ///
    /// // Create device using first available (serial_num = None)
    /// let device = UsbDevice::new(None).unwrap();
    ///
    /// // Now do device.init().unwrap(); etc
    /// ```
    fn new(config: Option<Self::Config>) -> Result<Self, Error>
    where
        Self: Sized;

    /// Initialize the Device.
    ///
    /// This function
    /// * reads physical device information
    /// * resets the device if considered necessary
    ///
    /// # Returns
    /// * `Ok(())` - Successfully initialized device
    /// * `Err(Error)` - If device initialization fails
    fn init(&mut self) -> Result<(), Error>;

    /// Returns the [`DeviceInfo`] for this device as an
    /// [`Option<&DeviceInfo>`].
    ///
    /// May be None, in which case the Device has not been initialized
    fn info(&mut self) -> Option<DeviceInfo>;

    /// Returns [`SpecificDeviceInfo`] for this device, as an
    /// ['Option<SpecificDeviceInfo']
    ///
    /// Will be None if the device hasn't been initialized
    fn specific_info(&mut self) -> Option<Self::SpecificDeviceInfo>;

    /// Returns [`SwDebugInfo`] for this device
    fn sw_debug_info(&mut self) -> SwDebugInfo;

    /// Do a hard reset of the device, and reinitialize it afterwards.
    ///
    /// Use with caution, as this function can fail, in which case you may be
    /// in a worse state than before, as if it fails you will be left with an
    /// un-initialized device.
    ///
    /// The [`DeviceInfo`] may change during this process.
    ///
    /// # Returns
    /// * `Ok(())` - If successful
    /// * `Err(Error)` - On error
    fn hard_reset_and_re_init(&mut self) -> Result<(), Error>;

    /// Sends a control message and reads the response
    ///
    /// # Arguments
    /// * `request` - The request_type byte
    /// * `value` - The 2 byte request value
    /// * `buffer` - A buffer which will be filled in with the read data
    ///
    /// # Returns
    /// * `Ok(usize)` - On success, with the number of bytes read
    /// * `Err(Error)` - On failure
    ///
    /// Note that the calling function must provide a large enough buffer for
    /// the response, or some bytes may not be read.
    fn read_control(&mut self, request: u8, value: u16, buffer: &mut [u8]) -> Result<usize, Error>;

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
    /// * `Ok()` - On success
    /// * `Err(Error)` - On failure
    fn write_control(&mut self, request: u8, value: u16, buffer: &[u8]) -> Result<(), Error>;

    /// Writes data to the device
    ///
    /// Will attempt to write all data, but will return the amount written on
    /// success, even if all the data could not be written.
    ///
    /// # Argumemts
    /// * `mode` - Mode, one of xum1541::PROTO_*, see [`crate::constants`]
    /// * `data` - Data to write
    ///
    /// # Returns
    /// * `Ok(usize)` - On success, number of bytes written
    /// * `Err(Error>` - On failure
    fn write_data(&mut self, mode: u8, data: &[u8]) -> Result<usize, Error>;

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
    /// * `Err(Error>` - On failure
    fn read_data(&mut self, mode: u8, buffer: &mut [u8]) -> Result<usize, Error>;

    /// Waits for status from the XUM1541 device, after certain commands and
    /// ioctls.
    ///
    /// This will block, so should be used with care.  Consider spawning a
    /// thread for this call, especially if waiting for status after an
    /// asyncronous ioctl.
    ///
    /// # Returns
    /// `Ok(u16)` - the 2 byte status value from the device
    /// `Err(Error)` - the error on failure
    fn wait_for_status(&mut self) -> Result<u16, Error>;

    /// Sends an ioctl command to the device.  Reads status from the device
    /// after sending, except where the Ioctl is asyncronous
    ///
    /// # Arguments
    /// * `ioctl` - The request_type of type [`crate::constants::Ioctl`]
    /// * `address` - The address to target with this ioctl.  May be 0.
    /// * `secondary_address` - The secondary to target with this ioctl. May be 0.
    ///
    /// # Returns
    /// * `Ok(Option<u16>)` - 2 byte status from the device, or None for an asyncronous ioctl, as this function does not wait after async ioctl
    /// * `Err(Error)` - On failure
    fn ioctl(
        &mut self,
        ioctl: Ioctl,
        address: u8,
        secondary_address: u8,
    ) -> Result<Option<u16>, Error>;
}

/// Debug information about this Device software object
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SwDebugInfo {
    /// Number of API calls made to this object that led to this object
    /// querying the device
    pub api_calls: u64,
}

impl Default for SwDebugInfo {
    fn default() -> Self {
        SwDebugInfo { api_calls: 0 }
    }
}

impl SwDebugInfo {
    pub(crate) fn increment_api_call(&mut self) {
        self.api_calls += 1;
    }
}

/// DeviceInfo contains information read from the XUM1541 device.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeviceInfo {
    /// Product [`String`] from the USB device
    pub product: String,
    /// Manufacturer [`String`] from the USB device
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

impl Default for DeviceInfo {
    fn default() -> Self {
        DeviceInfo {
            product: String::new(),
            manufacturer: None,
            serial_number: None,
            firmware_version: 0,
            capabilities: 0,
            status: 0,
            debug_info: None,
        }
    }
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

/// DeviceDebugInfo contains some additional data from XUM1541 supporting
/// firmware version 8 and onwards.
#[derive(Debug, Clone, Serialize, Deserialize)]
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

/// A trait to allow for specific device information to be accessed
pub trait SpecificDeviceInfo {
    type Info;

    fn print(&self);
}
