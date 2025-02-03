/// serial.rs
///
/// Connects to a specific xum1541 serial number
use xum1541::{BusBuilder, Error};
use clap::Parser;

#[derive(Parser)]
#[command(author, version, about)]
struct Args {
    serial: u8,
}

fn main() -> Result<(), Error> {
    let args = Args::parse();

    // Connect to the XUM1541 device via USB
    let mut bus = BusBuilder::new().serial(args.serial).build()?;

    // Initialize the bus
    bus.initialize()?;

    println!("Successfully connected to device with serial number {}", args.serial);

    Ok(())
}