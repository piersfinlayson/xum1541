/// readme.rs
///
/// This is the example from the README.md file
///
/// basic.rs may be a better example to start with, as it is provides better
/// error handling.
use xum1541::{UsbBus, DeviceChannel, Error};

fn main() -> Result<(), Error> {
    // Connect to the XUM1541 device via USB
    let mut bus = UsbBus::default()?;

    // Initialize the bus
    bus.initialize()?;

    // Reset the IEC
    bus.reset()?;

    // Instuct device 8 to talk using channel 15
    bus.talk(DeviceChannel::new(8, 15)?)?;

    // Read up to 256 bytes of data from the drive
    let mut data = vec![0u8; 256];
    bus.read(&mut data)?;

    // Print it out (this should be the drive status)
    println!(
        "Retrieved data from drive: {}",
        std::str::from_utf8(&data).unwrap()
    );

    // Tell the drive to stop talking
    bus.untalk()?;

    // No need to close the XUM1541 device, it will be closed when bus goes
    // out of scope

    Ok(())
}
