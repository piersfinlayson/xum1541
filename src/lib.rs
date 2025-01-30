//! # xum1541
//!
//! This crate provides a Rust interface for communicating with Commodore disk drives via the XUM1541
//! USB adapter (also known as the ZoomFloppy). It enables modern computers to interact with vintage
//! Commodore hardware through a clean, safe, and idiomatic Rust API.
//!
//! ## Overview
//!
//! The xum1541 crate is structured around two main components:
//!
//! - A high-level [`Bus`] interface for IEC/IEEE-488 bus operations
//! - A lower-level [`Device`] interface for direct USB communication
//!
//! ### Key Features
//!
//! - Safe Rust interface for XUM1541 USB adapter communication
//! - Support for both IEC and IEEE-488 protocols
//! - Comprehensive error handling
//! - Built-in device discovery and initialization
//! - Automatic USB resource management
//! - Support for multiple XUM1541 devices through serial number selection
//! - Configurable USB timeouts and debug logging
//!
//! ## Architecture
//!
//! ### Bus Layer
//!
//! The [`Bus`] struct is the primary interface most users should interact with. It provides:
//!
//! - High-level IEC/IEEE-488 bus operations (talk, listen, open, close)
//! - Data transfer primitives (read, write)
//! - Bus state management
//! - Pattern-based reading capabilities
//! - Safe state transitions between bus modes
//!
//! The Bus layer enforces correct protocol usage by tracking the current bus state and validating
//! operations against that state.
//!
//! ### Device Layer
//!
//! The `Device` struct provides low-level USB communication with the XUM1541 hardware:
//!
//! - Direct USB control transfers
//! - Bulk data transfers
//! - Device initialization and configuration
//! - Firmware version verification
//! - Device capability detection
//! - Debug information access (for firmware version 8+)
//!
//! ## Getting Started
//!
//! The recommended way to create a new xum1541 interface is either through
//! [`BusBuilder`] (or a device type specific one such as [`UsbBusBuilder`]),
//! Or using the default() function on the device specific [`Bus`] type, such
//! ass [`UsbBus`].
//!
//! ```rust,no_run
//! use xum1541::{UsbBus, BusBuilder, UsbBusBuilder};
//!
//! let mut bus = UsbBusBuilder::new()
//!     .device_serial_number(0)  // Use first available device
//!     .build().unwrap();
//!     
//! bus.initialize().unwrap();
//!
//! // Or
//!
//! let mut bus = UsbBus::default().unwrap();
//! bus.initialize().unwrap();
//! ```
//!
//! ## Error Handling
//!
//! The library uses a custom `Error` type that covers:
//!
//! - Device access errors (not found, firmware version mismatch, serial number mismatch)
//! - Communication failures (USB errors, timeouts)
//! - Protocol violations (invalid state transitions)
//! - Resource exhaustion (buffer overflows)
//! - Invalid parameters (zero-length transfers, invalid patterns)
//!
//! ## Key Types
//!
//! ### DeviceChannel
//!
//! Represents a combination of device number and channel (secondary address) on the Commodore bus.
//!
//! ### BusMode
//!
//! Tracks the current state of the bus:
//! - Idle
//! - Talking (a specific device talking using a specific channel)
//! - Listening (a specific device listening using a specific channel)
//!
//! ### DeviceInfo
//!
//! Contains information about the XUM1541 device:
//! - Firmware version
//! - Hardware capabilities
//! - Device status
//! - Debug information (firmware v8+)
//!
//! ## Advanced Usage
//!
//! ### Custom USB Context
//!
//! For applications needing fine-grained control over USB behavior:
//!
//! ```rust,no_run
//! use rusb::{Context, UsbContext};
//! use xum1541::{BusBuilder, UsbBusBuilder};
//!
//! let mut context = Context::new().unwrap();
//! context.set_log_level(rusb::LogLevel::Debug);
//!
//! let bus = UsbBusBuilder::new()
//!     .context(context)
//!     .build()
//!     .unwrap();
//! ```
//!
//! ### Reading with Patterns
//!
//! The library provides sophisticated pattern-based reading capabilities:
//!
//! ```rust,ignore
//! // Read until specific byte pattern, including the pattern itself
//! let mut buffer = vec![0u8; 256];
//! // Will read until 0x0D is found, and include the 0x0D in the buffer
//! let bytes_read = bus.read_until(&mut buffer, &[0x0D])?;
//!
//! // Read until multiple byte pattern
//! let mut buffer = vec![0u8; 256];
//! // Will read until the sequence 0x31, 0x32 is found, including both bytes
//! let bytes_read = bus.read_until(&mut buffer, &[0x31, 0x32])?;
//!
//! // Read until any byte from set is found (inclusive)
//! let mut buffer = vec![0u8; 256];
//! // Will read until either 0x0D or 0x20 is found, including the matching byte
//! let bytes_read = bus.read_until_any(&mut buffer, &[0x0D, 0x20])?;
//! ```
//!
//! ## Examples
//!
//! See
//! * [`examples/readme.rs`](examples/readme.rs) - A very simple app which queries a drive's status.  This is also included below.
//! * [`examples/basic.rs`](examples/basic.rs) - A more complex example which enables logging (run with `RUST_LOG=info`` for example) queries the device's capabilities and drive status.
//!
//! ```rust,no_run
//! use xum1541::{BusBuilder, UsbBusBuilder, DeviceChannel, Error};
//!
//! fn main() -> Result<(), Error> {
//!     // Connect to the XUM1541 device
//!     let mut bus = UsbBusBuilder::new().build()?;
//!
//!     // Initialize the bus
//!     bus.initialize()?;
//!
//!     // Reset the IEC
//!     bus.reset()?;
//!
//!     // Instuct device 8 to talk using channel 15
//!     bus.talk(DeviceChannel::new(8, 15)?)?;
//!
//!     // Read up to 256 bytes of data from the drive
//!     let mut data = vec![0u8; 256];
//!     bus.read(&mut data)?;
//!
//!     // Print it out (this should be the drive status)
//!     println!(
//!         "Retrieved data from drive: {}",
//!         std::str::from_utf8(&data).unwrap()
//!     );
//!
//!     // Tell the drive to stop talking
//!     bus.untalk()?;
//!
//!     // No need to close the XUM1541 device, it will be closed when bus goes
//!     // out of scope
//!
//!     Ok(())
//! }
//! ```
//!
//! ## Thread Safety
//!
//! The library is not inherently thread-safe but can be accessed from multiple
//! threads, and from futures/async code, with some considerations:
//!
//! 1. Wrap the [`Bus`] instance in a mutex
//! 2. Consider how to allocate/deallocate channels for device communication
//! 3. Be aware of USB timeout implications in threaded contexts
//!
//! ## Logging
//!
//! The library uses the `log` crate for diagnostic output:
//!
//! - Error: Critical failures requiring immediate attention
//! - Warn: Potential issues or deprecated usage
//! - Info: Important state changes
//! - Debug: Detailed operation information
//! - Trace: Function entry/exit and protocol-level details
//!
//! To enable logging, use [`env_logger::init`](https://docs.rs/env_logger/latest/env_logger/index.html) and set the `RUST_LOG` environment variable:
//!
//! ## Technical Details
//!
//! ### USB Protocol
//!
//! - Vendor ID: 0x16D0
//! - Product ID: 0x0504
//! - Interface: Bulk transfers
//! - Endpoints: IN and OUT for data/status
//! - Control transfers for device management
//!
//! ### XUM1541 Firmware Compatibility
//!
//! - Minimum supported firmware: Version 7
//! - Enhanced debugging: Version 8+
//!
//! ## Contributing
//!
//! When extending the library:
//!
//! 1. Maintain the state machine model for bus operations
//! 2. Follow the error handling patterns
//! 3. Add appropriate logging at each level
//! 4. Document public interfaces thoroughly
//! 5. Add tests for new functionality
//!
//! ## License
//!
//! This library is licensed under the GNU General Public License Version 3 (GPLv3).
//! ## Acknowledgments
//! - Original [OpenCBM](https://github.com/OpenCbm/OpenCbm) project and xum1541 plugin developers.  OpenCbm is licensed under the GPLv2
//! - ZoomFloppy hardware developers
//! - Commodore community
#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};

pub mod bus;
pub mod constants;
pub mod device;
pub mod error;

pub use crate::error::Communication as CommunicationError;
pub use crate::error::DeviceAccess as DeviceAccessError;
/// Error types
pub use crate::error::Error;
pub use crate::error::Internal as InternalError;

/// Constants
pub use crate::constants::{
    Ioctl, DEVICE_MAX_NUM, DEVICE_MIN_NUM, DRIVE_MAX_CHANNEL, DRIVE_MIN_CHANNEL,
};

pub use crate::bus::remoteusb::{RemoteUsbBus, RemoteUsbBusBuilder};
pub use crate::bus::usb::{UsbBus, UsbBusBuilder};
pub use crate::bus::DEFAULT_TIMEOUT as BUS_DEFAULT_TIMEOUT;
/// Bus types
pub use crate::bus::{Bus, BusBuilder};

pub use crate::device::remoteusb::{
    RemoteUsbDevice, RemoteUsbDeviceConfig, RemoteUsbInfo, UsbDeviceServer,
};
pub use crate::device::usb::{UsbDevice, UsbDeviceConfig, UsbInfo};
pub use crate::device::Config as DeviceConfig;
/// Device types
pub use crate::device::{Config, Device, DeviceDebugInfo, DeviceInfo, SpecificDeviceInfo};

/// Struct holding device and channel numbers.  Used by [`crate::Bus`] functions
/// which require the device and channel to be specified.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DeviceChannel {
    device: u8,
    channel: u8,
}

impl DeviceChannel {
    /// Create a new DeviceChannel object
    pub fn new(device: u8, channel: u8) -> Result<Self, Error> {
        match Self::validate(device, channel) {
            Ok(()) => Ok(Self { device, channel }),
            Err(e) => Err(e),
        }
    }

    // Returns the device number
    pub const fn device(&self) -> u8 {
        self.device
    }

    // Returns the channel number
    pub const fn channel(&self) -> u8 {
        self.channel
    }

    fn validate(device: u8, channel: u8) -> Result<(), Error> {
        trace!("DeviceChannel::validate: device {device} and channel {channel}");

        if device < DEVICE_MIN_NUM {
            trace!("Device {device} below minimum {DEVICE_MIN_NUM}");
            Err(Error::Args {
                message: format!("Device number {device} is less than minimum {DEVICE_MIN_NUM}"),
            })
        } else if device > DEVICE_MAX_NUM {
            trace!("Device {device} above maximum {DEVICE_MAX_NUM}");
            Err(Error::Args {
                message: format!("Device number {device} is greater than maximum {DEVICE_MAX_NUM}"),
            })
        } else if channel < DRIVE_MIN_CHANNEL {
            trace!("Channel {channel} below minimum {DRIVE_MIN_CHANNEL}");
            Err(Error::Args {
                message: format!(
                    "Channel number {channel} is less than minimum {DRIVE_MIN_CHANNEL}"
                ),
            })
        } else if channel > DRIVE_MAX_CHANNEL {
            trace!("Channel {channel} above maximum {DRIVE_MAX_CHANNEL}");
            Err(Error::Args {
                message: format!(
                    "Channel number {channel} is greater than maximum {DRIVE_MAX_CHANNEL}"
                ),
            })
        } else {
            trace!("Validation successful for device {device} channel {channel}");
            Ok(())
        }
    }

    fn as_talk_bytes(&self) -> Vec<u8> {
        vec![0x40 | self.device, 0x60 | self.channel]
    }

    fn as_listen_bytes(&self) -> Vec<u8> {
        vec![0x20 | self.device, 0x60 | self.channel]
    }

    fn as_open_bytes(&self) -> Vec<u8> {
        vec![0x20 | self.device, 0xf0 | self.channel]
    }

    fn as_close_bytes(&self) -> Vec<u8> {
        vec![0x20 | self.device, 0xe0 | self.channel]
    }
}

impl std::fmt::Display for DeviceChannel {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Device: {} Channel: {}", self.device, self.channel)
    }
}
