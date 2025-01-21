use crate::constants::{
    MAX_CHANNEL_NUM, MAX_DEVICE_NUM, MIN_CHANNEL_NUM, MIN_DEVICE_NUM, PROTO_CBM, PROTO_WRITE_ATN,
    PROTO_WRITE_TALK,
};

use crate::Xum1541Error;
#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};
use std::fmt;

/// Struct holding Device and Channel numbers
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DeviceChannel {
    device: u8, // Now private
    channel: u8,
}

impl DeviceChannel {
    /// Create a new DeviceChannel with compile-time range validation
    pub fn new(device: u8, channel: u8) -> Result<Self, Xum1541Error> {
        match Self::validate(device, channel) {
            Ok(()) => Ok(Self { device, channel }),
            Err(e) => Err(e),
        }
    }

    // Getters since fields are now private
    pub const fn device(&self) -> u8 {
        self.device
    }
    pub const fn channel(&self) -> u8 {
        self.channel
    }

    pub fn validate(device: u8, channel: u8) -> Result<(), Xum1541Error> {
        trace!("DeviceChannel::validate: device {device} and channel {channel}");

        if device < MIN_DEVICE_NUM {
            trace!("Device {device} below minimum {MIN_DEVICE_NUM}");
            Err(Xum1541Error::Args {
                message: format!("Device number {device} is less than minimum {MIN_DEVICE_NUM}"),
            })
        } else if device > MAX_DEVICE_NUM {
            trace!("Device {device} above maximum {MAX_DEVICE_NUM}");
            Err(Xum1541Error::Args {
                message: format!("Device number {device} is greater than maximum {MAX_DEVICE_NUM}"),
            })
        } else if channel < MIN_CHANNEL_NUM {
            trace!("Channel {channel} below minimum {MIN_CHANNEL_NUM}");
            Err(Xum1541Error::Args {
                message: format!("Channel number {channel} is less than minimum {MIN_CHANNEL_NUM}"),
            })
        } else if channel > MAX_CHANNEL_NUM {
            trace!("Channel {channel} above maximum {MAX_CHANNEL_NUM}");
            Err(Xum1541Error::Args {
                message: format!(
                    "Channel number {channel} is greater than maximum {MAX_CHANNEL_NUM}"
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

impl fmt::Display for DeviceChannel {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Device: {} Channel: {}", self.device, self.channel)
    }
}

/// We support three modes on the bus:
/// * Talking - Drive with DeviceChannel has been told to talk
/// * Listening - Drive with DeviceChannel has been told to listen, or Open
///               has been sent to the drive
/// * Idle - Talking/Listening has been cancelled.  Note Bus is also in Idle
///          after a complete Open sequence (which is Open, Write, Unlisten)
#[derive(Debug, PartialEq)]
pub enum BusMode {
    Talking(DeviceChannel),
    Listening(DeviceChannel),
    Idle,
}

impl Default for BusMode {
    fn default() -> Self {
        BusMode::Idle
    }
}

impl fmt::Display for BusMode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            BusMode::Talking(dc) => write!(f, "Talking: {}", dc),
            BusMode::Listening(dc) => write!(f, "Listening: {}", dc),
            BusMode::Idle => write!(f, "Idle"),
        }
    }
}

/// Supported sequences of commands:
///
/// Open file:
/// - Open (execute command, on DeviceChannel)
/// - Write filename
/// - Unlisten
///
/// Close file:
/// - Close (execute command, on DeviceChannel)
///
/// Reading data from drive:
/// - Talk (DeviceChannel)
/// - Read data
/// - Untalk
///
/// Sending data to drive:
/// - Listen (DeviceChannel)
/// - Write data
/// - Unlisten
#[derive(Debug)]
pub enum BusCommand {
    Talk(DeviceChannel),
    Listen(DeviceChannel),
    Untalk,
    Unlisten,
    Open(DeviceChannel),
    Close(DeviceChannel),
}

impl fmt::Display for BusCommand {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            BusCommand::Talk(dc) => write!(f, "Talk: {}", dc),
            BusCommand::Listen(dc) => write!(f, "Listen: {}", dc),
            BusCommand::Untalk => write!(f, "Untalk"),
            BusCommand::Unlisten => write!(f, "Unlisten"),
            BusCommand::Open(dc) => write!(f, "Open: {}", dc),
            BusCommand::Close(dc) => write!(f, "Close: {}", dc),
        }
    }
}

impl BusCommand {
    pub fn protocol(&self) -> u8 {
        match self {
            BusCommand::Talk(_) => PROTO_CBM | PROTO_WRITE_ATN | PROTO_WRITE_TALK,
            _ => PROTO_CBM | PROTO_WRITE_ATN,
        }
    }

    pub fn command_bytes(&self) -> Vec<u8> {
        match self {
            BusCommand::Talk(dc) => dc.as_talk_bytes(),
            BusCommand::Listen(dc) => dc.as_listen_bytes(),
            BusCommand::Untalk => vec![0x5f],
            BusCommand::Unlisten => vec![0x3f],
            BusCommand::Open(dc) => dc.as_open_bytes(),
            BusCommand::Close(dc) => dc.as_close_bytes(),
        }
    }

    pub fn trace_message(&self) -> &'static str {
        match self {
            BusCommand::Talk(_) => "Entered Bus::talk",
            BusCommand::Listen(_) => "Entered Bus::listen",
            BusCommand::Untalk => "Entered Bus::untalk",
            BusCommand::Unlisten => "Entered Bus::unlisten",
            BusCommand::Open(_) => "Entered Bus::open",
            BusCommand::Close(_) => "Entered Bus::close",
        }
    }
}
