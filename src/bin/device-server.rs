//! This is a complete implementation of an xum1541 network server, exposing
//! the xum1541 primitives over IP.
//!
//! It creates the [`UsbDevice`] locally, and accepts and serves up to one
//! connection from remote peers.  Theoretically it would be possible to serve
//! multiple peers, but this would probably require some kind of locking to
//! prevent the peers from interfering with each other's operations.
//!
//! The server listens on the default address and port, which can be
//! overridden.
//!
//! Once a remote peer has connected, no other connections will be accepted
//! until the remote peer disconnects.
//!
//! If any unexpected errors - such as pipes breaking, network errors, etc,
//! this server will exit.
//!
//! All logging is done to stdout and can be controlled via the `RUST_LOG`
//! environment variable.
//!
//! It is expected that in a production environment, this server would be
//! run as a service, and would be managed by a process manager such as
//! systemd, upstart, or supervisord.
use xum1541::device::remoteusb::DEFAULT_PORT;
use xum1541::{Device, Error, UsbDevice, UsbDeviceServer};

use env_logger;
#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};
use std::net::{IpAddr, SocketAddr};
use std::str::FromStr;

fn execute() -> Result<(), Error> {
    info!("Create USB Device");
    let device = UsbDevice::new(None)?;

    let addr = "0.0.0.0";
    info!(
        "Create Remote USB Device to listen on {}:{}",
        addr, DEFAULT_PORT
    );
    let ip_addr = IpAddr::from_str(addr).map_err(|e| Error::Args {
        message: format!("Failed to parse address: {}", e),
    })?;
    let bind_addr = SocketAddr::new(ip_addr, DEFAULT_PORT);
    let mut device = UsbDeviceServer::new(device, bind_addr)?;

    info!("Server: Listen for remote connection");
    let result = device.serve();

    info!("Exiting server");

    result
}

fn main() {
    env_logger::builder().init();

    match execute() {
        Ok(_) => std::process::exit(0),
        Err(e) => {
            error!("Error: {}", e);
            std::process::exit(1);
        }
    }
}
