/// talk.rs
///
/// This is as basic talk example to retrieve whatever data from channel 15
/// the drive wants to send us.
use xum1541::{BusBuilder, DeviceChannel, Error};
use std::env;

fn main() -> Result<(), Error> {
    // Parse command line arguments for the channel number, default to 15 if not provided
    let channel = env::args().nth(1)
        .and_then(|arg| arg.parse::<u8>().ok())
        .unwrap_or(15);

    // Connect to the XUM1541 device via USB
    let mut bus = BusBuilder::new().build()?;

    // Initialize the bus
    bus.initialize()?;

    // Instuct device 8 to talk using the specified channel
    bus.talk(DeviceChannel::new(8, channel)?)?;

    // Read up to 256 bytes of data from the drive
    let mut data = vec![0u8; 256];
    bus.read(&mut data)?;

    // Print it out (this should be the drive status)
    println!(
        "Retrieved data from drive on channel {}:\n\n{}",
        channel,
        std::str::from_utf8(&data).unwrap()
    );

    // Tell the drive to stop talking
    bus.untalk()?;

    // No need to close the XUM1541 device, it will be closed when bus goes
    // out of scope

    Ok(())
}
