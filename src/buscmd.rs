use crate::constants::{MAX_CHANNEL_NUM, MIN_CHANNEL_NUM, MAX_DEVICE_NUM, MIN_DEVICE_NUM, PROTO_CBM, PROTO_WRITE_ATN, PROTO_WRITE_TALK};

#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};
use crate::Xum1541Error;
use std::fmt;

/// Struct holding Device and Channel numbers
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DeviceChannel {
    device: u8,  // Now private
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
    pub const fn device(&self) -> u8 { self.device }
    pub const fn channel(&self) -> u8 { self.channel }

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
                message: format!("Channel number {channel} is greater than maximum {MAX_CHANNEL_NUM}"),
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

#[derive(Debug)]
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
    pub fn new_talk(device: u8, channel: u8) -> Result<Self, Xum1541Error> {
        Ok(Self::Talk(DeviceChannel::new(device, channel)?))
    }

    pub fn new_listen(device: u8, channel: u8) -> Result<Self, Xum1541Error> {
        Ok(Self::Listen(DeviceChannel::new(device, channel)?))
    }

    pub fn new_open(device: u8, channel: u8) -> Result<Self, Xum1541Error> {
        Ok(Self::Open(DeviceChannel::new(device, channel)?))
    }

    pub fn new_close(device: u8, channel: u8) -> Result<Self, Xum1541Error> {
        Ok(Self::Close(DeviceChannel::new(device, channel)?))
    }

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

    pub fn conflicts_with(&self, current_mode: &BusMode) -> Option<&'static str> {
        match (self, current_mode) {
            (BusCommand::Talk(_), BusMode::Talking(_)) => {
                Some("Setting new talker while device is still talking")
            }
            (BusCommand::Talk(_), BusMode::Listening(_)) => {
                Some("Setting talker while device is listening")
            }
            (BusCommand::Listen(_), BusMode::Talking(_)) => {
                Some("Setting listener while device is talking")
            }
            (BusCommand::Listen(_), BusMode::Listening(_)) => {
                Some("Setting new listener while device is still listening")
            }
            (BusCommand::Untalk, BusMode::Listening(_)) => {
                Some("Clearing talker while device is listening")
            }
            (BusCommand::Untalk, BusMode::Idle) => Some("Clearing talker when bus is idle"),
            (BusCommand::Unlisten, BusMode::Talking(_)) => {
                Some("Clearing listener while device is talking")
            }
            (BusCommand::Unlisten, BusMode::Idle) => Some("Clearing listener when bus is idle"),
            (BusCommand::Open(_) | BusCommand::Close(_), BusMode::Talking(_)) => {
                Some("Cannot open/close while device is talking")
            }
            (BusCommand::Open(_) | BusCommand::Close(_), BusMode::Idle) => {
                Some("Cannot open/close when no device is listening")
            }
            _ => None,
        }
    }

    pub fn requires_listen_first(&self) -> bool {
        matches!(self, BusCommand::Open(_) | BusCommand::Close(_))
    }

    pub fn next_mode(&self) -> BusMode {
        match self {
            BusCommand::Talk(dc) => BusMode::Talking(dc.clone()),
            BusCommand::Listen(dc) => BusMode::Listening(dc.clone()),
            BusCommand::Untalk | BusCommand::Unlisten => BusMode::Idle,
            BusCommand::Open(dc) | BusCommand::Close(dc) => BusMode::Listening(dc.clone()),
        }
    }

    pub fn device_channel(&self) -> Option<DeviceChannel> {
        match self {
            BusCommand::Talk(dc) | BusCommand::Listen(dc) 
            | BusCommand::Open(dc) | BusCommand::Close(dc) => Some(dc.clone()),
            _ => None,
        }
    }
}