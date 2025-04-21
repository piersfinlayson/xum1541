//! Bus state management objects and implementation

use crate::constants::{PROTO_CBM, PROTO_WRITE_ATN, PROTO_WRITE_TALK};
use crate::DeviceChannel;

#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};
use std::fmt;

/// The [`crate::Bus`] supports three modes:
/// * [`BusMode::Talking`] - Drive with `DeviceChannel` has been told to talk
/// * [`BusMode::Listening`] - Drive with `DeviceChannel` has been told to
///   listen, or Open has been sent to the drive
/// * [`BusMode::Idle`] - Talking/Listening has been cancelled.  Note Bus is also in Idle
///          after a complete Open sequence (which is Open, Write, Unlisten)
#[derive(Debug, PartialEq, Default)]
pub enum BusMode {
    /// Device has been told to Talk using a specific channel
    Talking(DeviceChannel),
    /// Device has been told to Listen using a specific channel
    Listening(DeviceChannel),
    /// No device is in Talk or Listen mode (nor does it have any files Open)
    #[default]
    Idle,
}

impl fmt::Display for BusMode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            BusMode::Talking(dc) => write!(f, "Talking: {dc}"),
            BusMode::Listening(dc) => write!(f, "Listening: {dc}"),
            BusMode::Idle => write!(f, "Idle"),
        }
    }
}

/// The `BusCommand` enum is used to store information about various Bus
/// commands which can be performed.  This is used to generate the bytes which
/// are sent to the XUM1541 device to execute the commands.
///
/// Supported sequences of commands:
///
/// Open file:
/// - Open (execute command, using `DeviceChannel`)
/// - Write filename
/// - Unlisten (Open serves as the Listen in this instance)
///
/// Close file:
/// - Close (execute command, using `DeviceChannel`)
///
/// Reading data from drive:
/// - Talk (`DeviceChannel`)
/// - Read data
/// - Untalk
///
/// Sending data to drive:
/// - Listen (`DeviceChannel`)
/// - Write data
/// - Unlisten
#[derive(Debug)]
pub enum BusCommand {
    /// Instruct a device to talk using a specific channel
    Talk(DeviceChannel),
    /// Instruct a device to listen using a specific channel
    Listen(DeviceChannel),
    /// Instruct a device to listen using no channel
    ListenNoChannel(u8),
    /// Instruct a device to stop talking
    Untalk,
    /// Instruct a device to stop listening
    Unlisten,
    /// Instruct a device to open a file using a specific channel.
    /// Must be followed by a write (of the filename) and then and unlisten.
    Open(DeviceChannel),
    /// Instruct a device to close a file using a specific channel
    Close(DeviceChannel),
}

impl fmt::Display for BusCommand {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            BusCommand::Talk(dc) => write!(f, "Talk: {dc}"),
            BusCommand::Listen(dc) => write!(f, "Listen: {dc}"),
            BusCommand::ListenNoChannel(device) => write!(f, "Listen: {device}"),
            BusCommand::Untalk => write!(f, "Untalk"),
            BusCommand::Unlisten => write!(f, "Unlisten"),
            BusCommand::Open(dc) => write!(f, "Open: {dc}"),
            BusCommand::Close(dc) => write!(f, "Close: {dc}"),
        }
    }
}

impl BusCommand {
    /// Returns the protocol to use for this command - also know as the request
    #[must_use]
    pub fn protocol(&self) -> u8 {
        match self {
            BusCommand::Talk(_) => PROTO_CBM | PROTO_WRITE_ATN | PROTO_WRITE_TALK,
            _ => PROTO_CBM | PROTO_WRITE_ATN,
        }
    }

    /// Returns the specific command which will be sent to the device, also
    /// known as the request type
    #[must_use]
    pub fn command_bytes(&self) -> Vec<u8> {
        match self {
            BusCommand::Talk(dc) => dc.as_talk_bytes(),
            BusCommand::Listen(dc) => dc.as_listen_bytes(),
            BusCommand::ListenNoChannel(device) => vec![0x20 | device],
            BusCommand::Untalk => vec![0x5f],
            BusCommand::Unlisten => vec![0x3f],
            BusCommand::Open(dc) => dc.as_open_bytes(),
            BusCommand::Close(dc) => dc.as_close_bytes(),
        }
    }
}
