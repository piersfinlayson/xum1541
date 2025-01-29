use env_logger;
#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};
use xum1541::{DeviceChannel, Error, RemoteUsbBus};

pub fn main() -> Result<(), Error> {
    env_logger::builder()
        .filter_level(log::LevelFilter::Info)
        .init();

    // Create the Remote USB device
    info!("Client: Create the Remote USB Device");
    let mut bus = RemoteUsbBus::default()?;

    // Iniitialize the bus
    info!("Client: Initialize the bus");
    bus.initialize()?;

    // Reset the bus
    info!("Client: Reset the bus");
    bus.reset()?;

    // Tell the drive on device 8 to talk using channel 15
    info!("Client: Talk - device 8, channel 15");
    bus.talk(DeviceChannel::new(8, 15)?)?;

    // Read up to 256 bytes of data from the drive
    let mut data = vec![0u8; 256];
    info!("Client: Read data from drive");
    bus.read(&mut data)?;

    // Print it out (this should be the drive status)
    println!(
        "Client: Retrieved data from drive: {}",
        std::str::from_utf8(&data).unwrap()
    );

    // Tell the drive to stop talking
    info!("Client: Untalk");
    bus.untalk()?;

    info!("Client: Exit");
    Ok(())
}
