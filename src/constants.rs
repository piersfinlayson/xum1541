use rusb::constants::{LIBUSB_ENDPOINT_IN, LIBUSB_ENDPOINT_OUT};
use std::time::Duration;

// Constants from original implementation
pub const XUM1541_VID: u16 = 0x16d0;
pub const XUM1541_PID: u16 = 0x0504;
pub const CUR_FW_VERSION: u8 = 8;
pub const MIN_FW_VERSION: u8 = 7;

// Protocols
pub const PROTO_MASK: u8 = 0xf0;
pub const PROTO_CBM: u8 = 1 << 4;
pub const PROTO_S1: u8 = 2 << 4;
pub const PROTO_S2: u8 = 3 << 4;
pub const PROTO_PP: u8 = 4 << 4;
pub const PROTO_P2: u8 = 5 << 4;
pub const PROTO_NIB: u8 = 6 << 4;
pub const PROTO_NIB_COMMAND: u8 = 7 << 4;
pub const PROTO_NIB_SRQ: u8 = 8 << 4;
pub const PROTO_NIB_SRQ_COMMAND: u8 = 9 << 4;

// Timeouts
pub const DEFAULT_READ_TIMEOUT: Duration = Duration::from_secs(5);
pub const DEFAULT_WRITE_TIMEOUT: Duration = Duration::from_secs(5);
pub const DEFAULT_CONTROL_TIMEOUT: Duration = Duration::from_secs(5);
pub const DEFAULT_USB_LOOP_SLEEP: Duration = Duration::from_millis(10);
pub const DEFAULT_USB_RESET_SLEEP: Duration = Duration::from_millis(100); // Full USB spec settling time

// Control protocol values
pub const BULK_IN_ENDPOINT: u8 = 3 | LIBUSB_ENDPOINT_IN;
pub const BULK_OUT_ENDPOINT: u8 = 4 | LIBUSB_ENDPOINT_OUT;

pub const XUM_STATUSBUF_SIZE: usize = 3;
pub const XUM_DEVINFO_SIZE: usize = 8;
pub const XUM_MAX_XFER_SIZE: usize = 32768;

pub const IO_BUSY: u8 = 1;
pub const IO_READY: u8 = 2;
pub const IO_ERROR: u8 = 3;

pub const READ: u8 = 8;
pub const WRITE: u8 = 9;

// Control messages
pub const CTRL_ECHO: u8 = 0;
pub const CTRL_INIT: u8 = 1;
pub const CTRL_RESET: u8 = 2;
pub const CTRL_SHUTDOWN: u8 = 3;
pub const CTRL_ENTER_BOOTLOADER: u8 = 4;
pub const CTRL_TAP_BREAK: u8 = 5;
pub const CTRL_GITREV: u8 = 6;
pub const CTRL_GCCVER: u8 = 7;
pub const CTRL_LIBCVER: u8 = 8;

pub const PROTO_WRITE_TALK: u8 = 1 << 0;
pub const PROTO_WRITE_ATN: u8 = 1 << 1;

// Device status flags
pub const STATUS_DOING_RESET: u8 = 0x01; // no clean shutdown, will reset now
pub const STATUS_IEEE488_PRESENT: u8 = 0x10; // IEEE-488 device connected
pub const STATUS_TAPE_PRESENT: u8 = 0x20; // 153x tape device connected

// Device capability flags
pub const CAP_CBM: u8 = 0x01; // supports CBM commands
pub const CAP_NIB: u8 = 0x02; // parallel nibbler
pub const CAP_NIB_SRQ: u8 = 0x04; // 1571 serial nibbler
pub const CAP_IEEE488: u8 = 0x08; // GPIB (PET) parallel bus
pub const CAP_TAP: u8 = 0x10; // 153x tape support
