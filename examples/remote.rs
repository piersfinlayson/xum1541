use env_logger;
#[allow(unused_imports)]
use xum1541::{BusBuilder, DeviceChannel, Error};

pub fn main() -> Result<(), Error> {
    env_logger::builder()
        .filter_level(log::LevelFilter::Info)
        .init();

    // Create the Remote USB device
    println!("Create the Remote USB Device");
    let mut bus = BusBuilder::new().remote_default()?.build()?;

    // Initialize the bus
    println!("Initialize the bus");
    bus.initialize()?;

    // Get the info about this device:
    println!("Remote XUM1541 information:");
    let info = bus.device_info();
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
    println!("Remote USB information:");
    if let Some(remote_usb_info) = bus.remote_usb_device_info() {
        println!("  USB information:");
        if let Some(usb_info) = remote_usb_info.usb_info {
            println!(
                "    Vendor ID/Product ID: {:04x}:{:04x}",
                usb_info.vendor_id, usb_info.product_id
            );
            println!("    Bus: {:03}", usb_info.bus_number);
            println!("    Device: {:03}", usb_info.device_address);
        } else {
            println!("    No USB info available");
        }
        println!("  Local bind address: {}", remote_usb_info.bind_addr);
    } else {
        println!("  No Remote USB info available");
    }

    // Tell the drive on device 8 to talk using channel 15
    println!("Talk - device 8, channel 15");
    bus.talk(DeviceChannel::new(8, 15)?)?;

    // Read up to 256 bytes of data from the drive
    let mut data = vec![0u8; 256];
    println!("Read data from drive");
    bus.read(&mut data)?;

    // Print it out (this should be the drive status)
    println!(
        "Retrieved data from drive: {}",
        std::str::from_utf8(&data).unwrap()
    );

    // Tell the drive to stop talking
    println!("Untalk");
    bus.untalk()?;

    let sw_debug_info = bus.device_sw_debug_info();
    println!("Made {} device API calls", sw_debug_info.api_calls);

    println!("Exit");
    Ok(())
}
