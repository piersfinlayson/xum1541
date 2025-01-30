//! Error objects for the xum1541 crate
use libc::{EACCES, EINVAL, EIO, ENODEV, ENOENT, ETIMEDOUT};
use serde::{Deserialize, Serialize};
use thiserror::Error;

/// Error type for the xum1541 crate
#[derive(Debug, Error, PartialEq, Serialize, Deserialize)]
pub enum Error {
    /// Errors accessing the USB device
    /// Note that permission problems are explicitly handled in the DeviceAccess
    #[error("USB error while attempting to commuicate with the XUM1541: {0}")]
    Usb(SerializableUsbError),

    /// Failure in initializing the XUM1541 device
    #[error("XUM1541 device initialization failed: {message}")]
    Init { message: String },

    /// Failure in communicating with the XUM1541 device - may be transient
    #[error("XUM1541 device communication error: {kind}")]
    Communication { kind: Communication },

    /// A USB operation timed out
    #[error("XUM1541 operation timed out after {dur:?}")]
    Timeout { dur: std::time::Duration },

    /// DeviceAccess holds a variety errors relating to accessing the XUM1541
    #[error("{kind}")]
    DeviceAccess { kind: DeviceAccess },

    /// Invalid arguments passed to the xum1541 library
    #[error("xum1541 library called with invalid arguments: {message}")]
    Args { message: String },
}

/// Used to differentiate between different types of problems accessing the
/// XUM1541 device
#[derive(Debug, Error, PartialEq, Serialize, Deserialize)]
pub enum DeviceAccess {
    #[error("The library is not connected to an XUM1541")]
    NoDevice,

    #[error("XUM1541 device {vid:04x}/{pid:04x} not found - is it connected and do you have permissions to access it?")]
    NotFound { vid: u16, pid: u16 },

    #[error("XUM1541 device found, but non-matching serial numbers. Found {actual:?}, was looking for {expected}")]
    SerialMismatch {
        vid: u16,
        pid: u16,
        actual: Vec<u8>,
        expected: u8,
    },

    #[error(
        "XUM1541 device has unsupported firmware version {actual}, expected minimum {expected}"
    )]
    FirmwareVersion { actual: u8, expected: u8 },

    #[error("Hit USB permissions error while attempting to access XUM1541 device.  Are you sure you have suitable permissions?  You may need to reconfigure udev rules in /etc/udev/rules.d/.")]
    Permission,

    #[error("Failed to resolve network address: {message}")]
    AddressResolution { message: String, errno: i32 },

    #[error("Failed to connect to remote device: {message}")]
    NetworkConnection { message: String, errno: i32 },
}

#[derive(Debug, Error, PartialEq, Serialize, Deserialize)]
pub enum Communication {
    /// Hit a failure issuing an ioctl to the device
    #[error("Failed to issue ioctl to device")]
    IoctlFailed,

    /// Device failed to provide a status response of the proper format
    #[error("Device returned an invalid status response format")]
    StatusFormat,

    /// Device returned an IO error status response
    #[error("Device returned an IO error status response")]
    StatusIo,

    /// Devicereturned an overall status response with an invalid value
    #[error("Device returned an invalid status response value 0x{value:02x}")]
    StatusResponse { value: u8 },

    /// Device returned an error status value
    #[error("Device returned an error status value 0x{value:04x}")]
    StatusValue { value: u16 },

    /// Timed out waiting for a status response from the device
    #[error("Timed out waiting for a status response from the device")]
    StatusTimeout { dur: std::time::Duration },

    /// The remote end of the connection disconnected
    #[error("Remote disconnected: {message} {errno}")]
    RemoteDisconnected { message: String, errno: i32 },

    /// Hit an error talking to a remote
    #[error("Hit an error communicating with a remote device: {message}, {errno}")]
    Remote { message: String, errno: i32 },

    /// Hit an unknown communication error
    #[error("Hit an unknown error during communication:  {message}, {errno}")]
    Unknown { message: String, errno: i32 },
}

#[derive(Debug, Error, PartialEq, Serialize, Deserialize)]
pub enum SerializableUsbError {
    #[error("{message}")]
    UsbError { message: String },
}

impl Error {
    pub fn to_errno(&self) -> i32 {
        match self {
            Error::Usb { .. } => EIO,
            Error::Init { .. } => EIO,
            Error::Communication { kind } => match kind {
                Communication::RemoteDisconnected { errno, .. } => *errno,
                Communication::Remote { errno, .. } => *errno,
                Communication::Unknown { errno, .. } => *errno,
                _ => EIO,
            },
            Error::Timeout { .. } => ETIMEDOUT,
            Error::DeviceAccess { kind } => match kind {
                DeviceAccess::NoDevice { .. } => ENODEV,
                DeviceAccess::NotFound { .. } => ENOENT,
                DeviceAccess::SerialMismatch { .. } => ENOENT,
                DeviceAccess::FirmwareVersion { .. } => ENODEV,
                DeviceAccess::Permission { .. } => EACCES,
                DeviceAccess::AddressResolution { errno, .. } => *errno,
                DeviceAccess::NetworkConnection { errno, .. } => *errno,
            },
            Error::Args { .. } => EINVAL,
        }
    }
}

/// Used for errors which must remain internal to the xum1541 crate
/// This is not "pub use"d by lib.rs
/// We do not use this error for all internal functions - but will if we have
/// specific, handled, internal errors to deal with and we want to prevent them
/// from escaping
#[derive(Debug, Error, Serialize, Deserialize)]
pub enum Internal {
    #[error("Invalid device information: {message}")]
    DeviceInfo { message: String },

    #[error("{error}")]
    PublicError { error: Error },
}

/// Map Error to an Internal
impl From<Error> for Internal {
    fn from(error: Error) -> Self {
        Internal::PublicError { error }
    }
}

/// Map rusb:Error to an Error, wrapped in an Internal::PublicError type
impl From<rusb::Error> for Internal {
    fn from(error: rusb::Error) -> Self {
        Internal::PublicError {
            error: error.into(),
        }
    }
}

// Map rusb::Error to Error
impl From<rusb::Error> for Error {
    fn from(err: rusb::Error) -> Self {
        Self::Usb(SerializableUsbError::UsbError {
            message: err.to_string(),
        })
    }
}

// Map Communication to Error
impl From<Communication> for Error {
    fn from(kind: Communication) -> Self {
        Self::Communication { kind }
    }
}
