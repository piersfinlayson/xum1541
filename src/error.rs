//! Error objects for the xum1541 crate
use libc::{EACCES, EINVAL, EIO, ENODEV, ENOENT, ETIMEDOUT};
use serde::{Deserialize, Serialize};
use thiserror::Error;

/// Error type for the xum1541 crate
#[derive(Debug, Error, PartialEq, Serialize, Deserialize)]
pub enum Error {
    /// Errors accessing the USB device
    /// Note that permission problems are explicitly handled in the DeviceAccessKind
    #[error("USB error while attempting to commuicate with the XUM1541: {0}")]
    Usb(SerializableUsbError),

    /// Failure in initializing the XUM1541 device
    #[error("XUM1541 device initialization failed: {message}")]
    Init { message: String },

    /// Failure in communicating with the XUM1541 device - may be transient
    #[error("XUM1541 device communication error: {kind}")]
    Communication { kind: CommunicationKind },

    /// A USB operation timed out
    #[error("XUM1541 operation timed out after {dur:?}")]
    Timeout { dur: std::time::Duration },

    /// DeviceAccess holds a variety errors relating to accessing the XUM1541
    #[error("{kind}")]
    DeviceAccess { kind: DeviceAccessKind },

    /// Invalid arguments passed to the xum1541 library
    #[error("xum1541 library called with invalid arguments: {message}")]
    Args { message: String },
}

/// Used to differentiate between different types of problems accessing the
/// XUM1541 device
#[derive(Debug, Error, PartialEq, Serialize, Deserialize)]
pub enum DeviceAccessKind {
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
}

#[derive(Debug, Error, PartialEq, Serialize, Deserialize)]
pub enum CommunicationKind {
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

    /// Hit an error talking to a remote
    #[error("Hit an error communicating with a remote device: {0}")]
    Remote(String),
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
            Error::Communication { .. } => EIO,
            Error::Timeout { .. } => ETIMEDOUT,
            Error::DeviceAccess { kind } => match kind {
                DeviceAccessKind::NoDevice { .. } => ENODEV,
                DeviceAccessKind::NotFound { .. } => ENOENT,
                DeviceAccessKind::SerialMismatch { .. } => ENOENT,
                DeviceAccessKind::FirmwareVersion { .. } => ENODEV,
                DeviceAccessKind::Permission { .. } => EACCES,
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
pub enum InternalError {
    #[error("Invalid device information: {message}")]
    DeviceInfo { message: String },

    #[error("{error}")]
    PublicError { error: Error },
}

/// Map Error to an InternalError
impl From<Error> for InternalError {
    fn from(error: Error) -> Self {
        InternalError::PublicError { error }
    }
}

/// Map rusb:Error to an Error, wrapped in an InternalError::PublicError type
impl From<rusb::Error> for InternalError {
    fn from(error: rusb::Error) -> Self {
        InternalError::PublicError {
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

// Map CommunicationKind to Error
impl From<CommunicationKind> for Error {
    fn from(kind: CommunicationKind) -> Self {
        Self::Communication { kind }
    }
}
