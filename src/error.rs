use thiserror::Error;

#[derive(Debug, Error)]
pub enum Error {
    #[error("USB error while attempting to commuicate with the XUM1541: {error}")]
    UsbError { error: rusb::Error },

    #[error("USB permissions error while attempting to access the XUM1541.  Are you sure you have permissions to access it?  You may need to reconfigure udev rules in /etc/udev/rules.d/.")]
    PermissionError,

    #[error("XUM1541 device initialization failed: {message}")]
    InitError { message: String },

    #[error("Invalid XUM1541 firmware version {actual}, expected minimum {expected}")]
    FirmwareVersion { actual: u8, expected: u8 },

    #[error("XUM1541 device communication error: {message}")]
    CommunicationError { message: String },

    #[error("XUM1541 operation timed out after {dur:?}")]
    Timeout { dur: std::time::Duration },

    #[error("Invalid XUM1541 device state: {message}")]
    InvalidState { message: String },

    #[error("No XUM1541 Device {vid:04x}/{pid:04x} found - is it connected and do you have permissions to access it?")]
    DeviceNotFound { vid: u16, pid: u16 },

    #[error("XUM1541 device {vid:04x}/{pid:04x} found, but non-matching serial numbers. Found {actual:?}, was looking for {expected}")]
    SerialMismatch {
        vid: u16,
        pid: u16,
        actual: Vec<u8>,
        expected: u8,
    },

    #[error("Calling code attempted to read or write too many bytes {attempt} vs max {max}")]
    SizeTooLarge { attempt: usize, max: usize },

    #[error("Calling code attempted to read or write too few bytes {attempt} vs min {min}")]
    SizeTooSmall { attempt: usize, min: usize },

    #[error("XUM1541 internal error {message}")]
    InternalError { message: String },
}

impl From<rusb::Error> for Error {
    fn from(error: rusb::Error) -> Self {
        match error {
            rusb::Error::Access => Error::PermissionError,
            other => Error::UsbError { error: other },
        }
    }
}

pub type Result<T> = std::result::Result<T, Error>;
