#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};
use xum1541::{CommunicationKind, Device, Error, UsbDevice, UsbDeviceServer};

fn main() -> Result<(), Error> {
    env_logger::builder()
        .filter_level(log::LevelFilter::Info)
        .init();

    info!("Server: Create USB Device");
    let device = UsbDevice::new(None)?;

    info!("Server: Create USB Device Server");
    let mut server = UsbDeviceServer::new(device, "127.0.0.1:9999")?;

    info!("Server: Serve");
    loop {
        match server.serve() {
            Ok(_) => continue,
            Err(e) => match e {
                Error::Communication { kind } => match kind {
                    CommunicationKind::Remote(message) => {
                        info!("Server: Remote message: {}", message);
                        info!("Client probably disconnected - we'll continue");
                        continue;
                    }
                    _ => {
                        error!("Server: Communication error: {:?}", kind);
                        break;
                    }
                },
                _ => {
                    error!("Server: Error: {}", e);
                    break;
                }
            },
        }
    }

    info!("Server: Exit");
    Ok(())
}
