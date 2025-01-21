# xum1541

A native Rust implementation of the xum1541 (ZoomFloppy) protocol for communicating with Commodore disk drives.

## Overview

This project reimplements OpenCBM's xum1541 plugin natively in Rust, offering several key improvements:

- Native Rust implementation for better integration with Rust applications
- Consistent, ergonomic API design with proper error handling
- Enhanced stability compared to OpenCBM's C implementation, particularly on Linux systems
- Type-safe interfaces and modern Rust idioms

## Features

- Direct communication with Commodore disk drives via xum1541/ZoomFloppy devices
- Native Rust error handling and type safety
- Clean API design following Rust best practices
- Improved stability and reliability over C FFI implementations
- Safe resource management through RAII - USB device resources are automatically closed when objects go out of scope
- Thread-safe design supporting concurrent access and async/futures through standard Rust concurrency primitives (Arc<Mutex<>>)

Note, this is not a drop-in replacement for the OpenCBM plugin - it will not work with the OpenCBM library and applications.

## Installation

```bash
cargo add xum1541
```

## Usage

```rust
use xum1541::{BusBuilder, Xum1541Error};

fn main() -> Result<(), Xum1541Error> {
    // Connect to the XUM1541 device 
    let mut bus = BusBuilder::new().build()?;

    // Initialize the bus
    bus.initialize()?;

    // Reset the IEC
    bus.reset()?;

    // Instuct device 8 to talk using channel 15
    bus.talk(8, 15)?;

    // Read up to 256 bytes of data from the drive
    let mut data = vec![0u8; 256];
    bus.read(&mut data, 256)?;

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
```

## Requirements

- Rust 1.70 or higher
- USB device access permissions (Linux systems may need udev rules)
- Compatible xum1541/ZoomFloppy hardware

## Building from Source

1. Clone the repository:
```bash
git clone https://github.com/piersfinlayson/xum1541.git
cd xum1541
```

2. Build the project:
```bash
cargo build --release
```

## Testing

Run the test suite:
```bash
cargo test
```

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

GPLv3

## Acknowledgments

- Original OpenCBM project and xum1541 plugin developers
- ZoomFloppy hardware developers
- Commodore community

## Project Status

This project is under active development. Please report any issues or feature requests through GitHub issues.