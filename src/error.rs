use thiserror::Error;

/// Error type for the xum1541 crate
#[derive(Debug, Error)]
pub enum Xum1541Error {
    /// Errors accessing the USB device
    /// Note that permission problems are explicitly handled in the DeviceAccessKind
    #[error("USB error while attempting to commuicate with the XUM1541: {error}")]
    Usb { error: rusb::Error },

    /// Failure in initializing the XUM1541 device
    #[error("XUM1541 device initialization failed: {message}")]
    Init { message: String },

    /// Failure in communicating with the XUM1541 device - may be transient
    #[error("XUM1541 device communication error: {message}")]
    Communication { message: String },

    /// A USB operation timed out
    #[error("XUM1541 operation timed out after {dur:?}")]
    Timeout { dur: std::time::Duration },

    /// DeviceAccess holds a variety errors relating to accessing the XUM1541
    #[error("{kind}")]
    DeviceAccess {
        kind: DeviceAccessKind,
    },

    /// Invalid arguments passed to the xum1541 library
    #[error("xum1541 library called with invalid arguments: {message}")]
    Args { message: String },
}

/// Used to differentiate between different types of problems accessing the
/// XUM1541 device
#[derive(Debug, Error)]
pub enum DeviceAccessKind {
    #[error("XUM1541 device {vid:04x}/{pid:04x} not found - is it connected and do you have permissions to access it?")]
    NotFound {
        vid: u16,
        pid: u16,
    },
    
    #[error("XUM1541 device found, but non-matching serial numbers. Found {actual:?}, was looking for {expected}")]
    SerialMismatch {
        vid: u16,
        pid: u16,
        actual: Vec<u8>,
        expected: u8,
    },

    #[error("XUM1541 device has unsupported firmware version {actual}, expected minimum {expected}")]
    FirmwareVersion {
        actual: u8,
        expected: u8,
    },

    #[error("Hit USB permissions error while attempting to access XUM1541 device.  Are you sure you have suitable permissions?  You may need to reconfigure udev rules in /etc/udev/rules.d/.")]
    Permission,
}

/// Map rusb:Error to an Xum1541Error
/// We explicitly map a permission problem to a specific error - all others
/// are mapped to the generic Usb error
impl From<rusb::Error> for Xum1541Error {
    fn from(error: rusb::Error) -> Self {
        match error {
            rusb::Error::Access => Xum1541Error::DeviceAccess { kind: DeviceAccessKind::Permission },
            other => Xum1541Error::Usb { error: other },
        }
    }
}

/// Used for errors which must remain internal to the xum1541 crate
/// This is not "pub use"d by lib.rs
/// We do not use this error for all internal functions - but will if we have
/// specific, handled, internal errors to deal with and we want to prevent them
/// from escaping
#[derive(Debug, Error)]
pub enum InternalError {
    #[error("Invalid device information: {message}")]
    DeviceInfo { message: String },

    #[error("{error}")]
    PublicError { error: Xum1541Error },
}

/// Map Xum1541Error to an InternalError
impl From<Xum1541Error> for InternalError {
    fn from(error: Xum1541Error) -> Self {
        InternalError::PublicError { error }
    }
}

/// Map rusb:Error to an Xum1541Error, wrapped in an InternalError::PublicError type
impl From<rusb::Error> for InternalError {
    fn from(error: rusb::Error) -> Self {
        InternalError::PublicError { error: error.into() }
    }
}