#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};
use rusb::UsbContext;
use xum1541::{BusBuilder, DeviceChannel};
mod common;
use common::AppError;

fn code() -> Result<(), AppError> {
    info!("Create rusb context and logging");
    let mut context = rusb::Context::new().unwrap();
    context.set_log_level(rusb::LogLevel::Info);

    // Create bus - this opens the USB device and creates and initializations
    // Device as well under the covers
    info!("Create and open device");
    let mut builder = BusBuilder::new().context(context);
    let mut bus = builder.build()?;

    // Initialize the bus
    bus.initialize()?;

    // Print out the device info
    let info = bus.device_info();
    println!("XUM1541 information:");
    if let Some(info) = info {
        println!("  Product: {}", info.product);
        if let Some(manufacturer) = &info.manufacturer {
            println!("  Manufacturer: {}", manufacturer);
        }
        if let Some(serial) = &info.serial_number {
            println!("  Serial Number: {}", serial);
        }
        println!("  Firmware version: {}", info.firmware_version);
        println!("  Capabilities: {:#010b}", info.capabilities);
        info.print_capabilities();
        println!("  Status: {:#010b}", info.status);
        info.print_status();
        println!("  Debug information:");
        info.print_debug();
    } else {
        println!("  No device info available");
    }

    // Rese the bus.
    // Note it won't actually reset the bus if it just happened as part of the
    // initialization
    bus.reset()?;

    // Tell the drive on device 8 to talk using channel 15
    bus.talk(DeviceChannel::new(8, 15)?)?;

    // Read up to 256 bytes of data (this will read drive status)
    let mut data = vec![0u8; 256];
    bus.read(&mut data)?;
    let data_str = std::str::from_utf8(&data)
        .inspect_err(|e| warn!("Failed to convert data into string {}", e))
        .map_err(|e| AppError::App {
            message: format!("Failed to convert data into string {}", e),
        })?;
    println!("Retrieved data from drive: {}", data_str);

    // Tell the drive to stop talking
    bus.untalk()?;

    info!("All done - exiting");
    Ok(())
}

fn main() -> Result<(), AppError> {
    env_logger::init();

    // Use our own eprintln to print errors so our Error display is used instead
    // of Rust's default one
    if let Err(e) = code() {
        eprintln!("{}", e);
        std::process::exit(1);
    }
    Ok(())
}
