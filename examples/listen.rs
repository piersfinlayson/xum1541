/// listen.rs
///
/// This is as basic listen example to send a char to channel 15.
use xum1541::{BusBuilder, DeviceChannel, Error};
use std::thread;
use std::time::Duration;

fn main() -> Result<(), Error> {
    // Connect to the XUM1541 device via USB
    let mut bus = BusBuilder::new().build()?;

    // Initialize the bus
    bus.initialize()?;
    
    // Parse first command line argument (character to send)
    let arg = std::env::args().nth(1).unwrap_or_else(|| {
        eprintln!("Usage: {} <char> [iterations]", std::env::args().next().unwrap_or_default());
        std::process::exit(1);
    });
    
    // Get the first character
    let char_to_send = arg.chars().next().unwrap_or_else(|| {
        eprintln!("Error: Empty character argument");
        std::process::exit(1);
    });
    
    // Parse second command line argument (iterations)
    let iterations = std::env::args().nth(2)
        .and_then(|s| s.parse::<u32>().ok())
        .unwrap_or(1); // Default to 1 if not specified
    
    for ii in 0..iterations {
        // Instruct device 8 to talk using the specified channel
        bus.listen(DeviceChannel::new(8, 15)?)?;

        // Write a byte
        let data = [char_to_send as u8];
        bus.write(&data)?;

        // Print it out (this should be the drive status)
        println!("Iteration {}: Char sent {}", ii + 1, char_to_send);

        // Tell the drive to stop talking
        bus.unlisten()?;
        
        // Add 50ms pause between iterations (skip on last iteration)
        if ii < iterations - 1 {
            thread::sleep(Duration::from_millis(100));
        }
    }

    Ok(())
}
