/// drivecheck.rs
///
/// Check whether drives between 8-11 are present on the bus.  The way
/// this is done is by instructing each dive in turn to listen or talk on
/// the bus.
///
/// If Device::write_data(), which is called by Bus::execute_command()
/// (which in turns is being called by Bus::talk() and Bus::listen())
/// receives a 0x0000 status response from the xum1541, this indicates
/// an error, which is most likely to be because there is no device present.
///
/// Note however that there are other reasons why the xum1541 may return
/// a status value of 0x0000.  See OpenCbm/xum1541/iec.c, function
/// iec_raw_write() for these cases.
use xum1541::{BusBuilder, UsbBusBuilder, CommunicationKind, DeviceChannel, Error};
mod common;
use common::AppError;
use env_logger;

#[derive(PartialEq)]
enum Mode {
    Listen,
    Talk,
}
impl std::fmt::Display for Mode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Mode::Listen => write!(f, "Listen"),
            Mode::Talk => write!(f, "Talk"),
        }
    }
}

fn main() -> Result<(), AppError> {
    env_logger::init();

    // Connect to the XUM1541 device
    let mut bus = UsbBusBuilder::new().build()?;

    // Initialize the bus
    bus.initialize()?;

    // Reset the IEC
    bus.reset()?;

    // Try to detect the drives
    let mut drive = 8;
    let max_drive = 11;
    let channel = 15;
    let mut mode = Mode::Listen;
    println!("Test for drives {drive} to {max_drive}");
    loop {
        let dc = DeviceChannel::new(drive, channel)?;

        // Run the command to check for presence, and time how long it takes
        let start = std::time::Instant::now();
        let result = match mode {
            Mode::Listen => bus.listen(dc),
            Mode::Talk => bus.talk(dc),
        };
        let elapsed = start.elapsed();

        match result {
            Err(Error::Communication {
                kind: CommunicationKind::StatusValue { value: _ },
            }) => {
                println!(
                    "Drive {drive} not detected via {mode} - check took {}ms",
                    elapsed.as_millis()
                );
            }
            Err(e) => {
                println!("Hit unexpected error trying to detect drive {drive} via {mode}: {e} - check took {}ms", elapsed.as_millis());
            }
            Ok(_) => {
                let result = match mode {
                    Mode::Listen => bus.unlisten(),
                    Mode::Talk => bus.untalk(),
                };
                match result {
                    Err(e) => {
                        println!("Hit unexpected error trying to detect drive {drive} via {mode}: {e} - check took {}ms", elapsed.as_millis());
                    }
                    Ok(_) => (),
                }
                println!(
                    "Drive {drive} detected via {mode} - check took {}ms",
                    elapsed.as_millis()
                );
            }
        }

        // Move on the next drive, and use the other mechanism to check
        // for its presence.
        drive += 1;
        if mode == Mode::Listen {
            mode = Mode::Talk;
        } else {
            mode = Mode::Listen;
        }
        if drive > max_drive {
            break;
        } else {
            continue;
        }
    }

    Ok(())
}
