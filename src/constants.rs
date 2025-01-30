//! Constants used in the XUM1541 implementation
use rusb::constants::{LIBUSB_ENDPOINT_IN, LIBUSB_ENDPOINT_OUT};
use serde::{Deserialize, Serialize};
use std::time::Duration;

/// XUM1541 device info

/// XUM1541 USB vendor ID
pub const XUM1541_VID: u16 = 0x16d0;
/// XUM1541 USB product ID
pub const XUM1541_PID: u16 = 0x0504;
/// Latest known XUM1541 firmware version
pub const CUR_FW_VERSION: u8 = 8;
/// Minimum version of XUM1541 firmware supported by this crate
pub const MIN_FW_VERSION: u8 = 7;

/// Device protocol commands

/// Used with [`crate::constants::PROTO_WRITE_ATN`] and [`crate::constants::PROTO_CBM`] to enter a device into Talk mode
pub const PROTO_WRITE_TALK: u8 = 1 << 0;
/// Used with [`crate::constants::PROTO_CBM`] to enter a device into Untalk, Listen, Unlisten, Open and Close modes
pub const PROTO_WRITE_ATN: u8 = 1 << 1;
/// Mask to retrieve the protocol from the XUM1541 status byte
pub const PROTO_MASK: u8 = 0xf0;
/// CBM rotocol
pub const PROTO_CBM: u8 = 1 << 4;
/// Serial 1 protocol
pub const PROTO_S1: u8 = 2 << 4;
/// Serial 2 protocol
pub const PROTO_S2: u8 = 3 << 4;
/// Parallel protocol
pub const PROTO_PP: u8 = 4 << 4;
/// 2 byte parallel protocol
pub const PROTO_P2: u8 = 5 << 4;
/// Burst nibbler parallel protocol
pub const PROTO_NIB: u8 = 6 << 4;
/// Burst nibbler parallel commands
pub const PROTO_NIB_COMMAND: u8 = 7 << 4;
/// 1570/1571 serial nibbler protocol
pub const PROTO_NIB_SRQ: u8 = 8 << 4;
/// 1570/1571 serial nibbler commands
pub const PROTO_NIB_SRQ_COMMAND: u8 = 9 << 4;

/// Timeouts

/// Used to wait for the USB device to respond to a read message
pub const DEFAULT_READ_TIMEOUT: Duration = Duration::from_secs(5);
/// Used to wait for the USB device to respond to a write message
pub const DEFAULT_WRITE_TIMEOUT: Duration = Duration::from_secs(5);
/// Used to wait for the USB device to respond to a control message
pub const DEFAULT_CONTROL_TIMEOUT: Duration = Duration::from_secs(5);
/// Used to wait before retrying the status read of the USB device
pub const DEFAULT_USB_LOOP_SLEEP: Duration = Duration::from_millis(10);
/// Used to wait for the USB device to reset.  The USB spec requires a minimum of 100ms
pub const DEFAULT_USB_RESET_SLEEP: Duration = Duration::from_millis(100);

/// The USB device's bulk in endpoint is used to read data
pub const BULK_IN_ENDPOINT: u8 = 3 | LIBUSB_ENDPOINT_IN;
/// The USB device's bulk out endpoint is used to write data
pub const BULK_OUT_ENDPOINT: u8 = 4 | LIBUSB_ENDPOINT_OUT;

/// Size of XUM1541 status response
pub const STATUS_BUF_SIZE: usize = 3;
/// Size of XUM1541's devinfo response - used for each of the debug info strings
pub const DEV_INFO_SIZE: usize = 8;
/// Maximum data transfer size supported by the XUM1541
pub const MAX_XFER_SIZE: usize = 32768;

/// XUM1541 status busy - will retry
pub const IO_BUSY: u8 = 1;
/// XUM1541 status ready
pub const IO_READY: u8 = 2;
/// XUM1541 status reported an error
pub const IO_ERROR: u8 = 3;

/// Command used to read data from device
pub const READ: u8 = 8;
/// Command used to write data to device
pub const WRITE: u8 = 9;

/// USB echo command
pub const CTRL_ECHO: u8 = 0;
/// Device initialize command (usd to retrieve device info)
pub const CTRL_INIT: u8 = 1;
/// Bus reset command
pub const CTRL_RESET: u8 = 2;
/// Device shutdown command
pub const CTRL_SHUTDOWN: u8 = 3;
/// Enter device bootloader command
pub const CTRL_ENTER_BOOTLOADER: u8 = 4;
/// Tape operatons abort command
pub const CTRL_TAP_BREAK: u8 = 5;
/// Retrieve git revision command (firmware >= 8)
pub const CTRL_GITREV: u8 = 6;
/// Retrieve GCC version command (firmware >= 8)
pub const CTRL_GCCVER: u8 = 7;
/// Retrieve libc version command (firmware >= 8)
pub const CTRL_LIBCVER: u8 = 8;

/// Device status flags - retrieve at Device initialization time

/// Device was set down uncleeanly and should be reset
pub const STATUS_DOING_RESET: u8 = 0x01;
/// An IEEE-488 device is present on the bus
pub const STATUS_IEEE488_PRESENT: u8 = 0x10;
/// A 153x tape device is connected
pub const STATUS_TAPE_PRESENT: u8 = 0x20;

/// Device capability flags

/// Device supports CBM commands
pub const CAP_CBM: u8 = 0x01;
/// Device supports parallel nibbler commands
pub const CAP_NIB: u8 = 0x02;
/// Device supports 1570/1571 serial nibbler commands
pub const CAP_NIB_SRQ: u8 = 0x04;
/// Device supports IEEE-488
pub const CAP_IEEE488: u8 = 0x08;
/// Device supports 153x tape devices
pub const CAP_TAP: u8 = 0x10; // 153x tape support

/// Minimum and maximum disk drive numbers

/// Minimum (and default) Commodore disk drive device number.  Devices lower
/// tha 8 are reserved for other device types, such as printers and tape
/// drives, and  built in devices like keyboard and screen
pub const DEVICE_MIN_NUM: u8 = 8;

/// Maximum Commodore disk drive device number - derived from the maximum
/// number which can be set, with software, on the 1571
pub const DEVICE_MAX_NUM: u8 = 30;

/// Minimum and maximum disk drive channel numbers

/// Minimum disk drive channel number
pub const DRIVE_MIN_CHANNEL: u8 = 0;

/// Maximum disk drive channel number
pub const DRIVE_MAX_CHANNEL: u8 = 15;

/// Channel used for sending LOAD commands - such as $ for directory lising
pub const DRIVE_LOAD_CHANNEL: u8 = 0;

/// Channel used for sending SAVE comamnds
pub const DRIVE_SAVE_CHANNEL: u8 = 1;

/// First channel available for general read/write operations
pub const DRIVE_MIN_FREE_CHANNEL: u8 = 2;

/// Last channel available for general read/write operations
pub const DRIVE_MAX_FREE_CHANNEL: u8 = 14;

/// Drive's command channel, used for commands like `M-W` and `M-R`
pub const DRIVE_COMMAND_CHANNEL: u8 = 15;

/// Ioctls supported by [`crate::Device::ioctl`]
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum Ioctl {
    GetEoi = 23,
    ClearEoi = 24,
    PpRead = 25,
    PpWrite = 26,
    IecPoll = 27,
    IecWait = 28,
    IecSetRelease = 29,
    ParburstRead = 30,
    ParburstWrite = 31,
    SrqburstRead = 32,
    SrqburstWrite = 33,
    TapMotorOn = 66,
    TapGetVer = 67,
    TapPrepareCapture = 68,
    TapPrepareWrite = 69,
    TapGetSense = 70,
    TapWaitForStopSense = 71,
    TapWaitForPlaySense = 72,
    TapMotorOff = 73,
    Unknown = 255,
}

impl Ioctl {
    /// Whether the Ioctl is an asyncronous one
    pub fn is_async(&self) -> bool {
        *self == Self::IecWait
    }

    /// Whether the Ioctl is a syncronous one
    pub fn is_sync(&self) -> bool {
        !self.is_async()
    }
}

impl From<u8> for Ioctl {
    fn from(value: u8) -> Self {
        match value {
            23 => Self::GetEoi,
            24 => Self::ClearEoi,
            25 => Self::PpRead,
            26 => Self::PpWrite,
            27 => Self::IecPoll,
            28 => Self::IecWait,
            29 => Self::IecSetRelease,
            30 => Self::ParburstRead,
            31 => Self::ParburstWrite,
            32 => Self::SrqburstRead,
            33 => Self::SrqburstWrite,
            66 => Self::TapMotorOn,
            67 => Self::TapGetVer,
            68 => Self::TapPrepareCapture,
            69 => Self::TapPrepareWrite,
            70 => Self::TapGetSense,
            71 => Self::TapWaitForStopSense,
            72 => Self::TapWaitForPlaySense,
            73 => Self::TapMotorOff,
            _ => Self::Unknown,
        }
    }
}

impl std::fmt::Display for Ioctl {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::GetEoi => write!(f, "GetEoi"),
            Self::ClearEoi => write!(f, "ClearEoi"),
            Self::PpRead => write!(f, "PpRead"),
            Self::PpWrite => write!(f, "PpWrite"),
            Self::IecPoll => write!(f, "IecPoll"),
            Self::IecWait => write!(f, "IecWait"),
            Self::IecSetRelease => write!(f, "IecSetRelease"),
            Self::ParburstRead => write!(f, "ParburstRead"),
            Self::ParburstWrite => write!(f, "ParburstWrite"),
            Self::SrqburstRead => write!(f, "SrqburstRead"),
            Self::SrqburstWrite => write!(f, "SrqburstWrite"),
            Self::TapMotorOn => write!(f, "TapMotorOn"),
            Self::TapGetVer => write!(f, "TapGetVer"),
            Self::TapPrepareCapture => write!(f, "TapPrepareCapture"),
            Self::TapPrepareWrite => write!(f, "TapPrepareWrite"),
            Self::TapGetSense => write!(f, "TapGetSense"),
            Self::TapWaitForStopSense => write!(f, "TapWaitForStopSense"),
            Self::TapWaitForPlaySense => write!(f, "TapWaitForPlaySense"),
            Self::TapMotorOff => write!(f, "TapMotorOff"),
            Self::Unknown => write!(f, "Unknown"),
        }
    }
}
