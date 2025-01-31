use crate::device::remoteusb::{DEFAULT_ADDR, DEFAULT_PORT};
use crate::{Bus, BUS_DEFAULT_TIMEOUT};
use crate::{DeviceAccessError, Error};
use crate::{DeviceConfig, DeviceType, RemoteUsbDeviceConfig, UsbDeviceConfig};

#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};
use rusb::{Context, UsbContext};
use std::net::{IpAddr, Ipv4Addr, Ipv6Addr, SocketAddr, ToSocketAddrs};
use std::str::FromStr;
use std::time::Duration;

/// A builder pattern for creating [`Bus`] instances using a UsbDevice and
/// custom configuration.
///
/// Allows setting optional parameters like serial number, timeout, and USB context
/// before creating the final [`Bus`] instance.
///
/// # Examples
///
/// ## A simple example
///
/// ```no_run
/// use xum1541::BusBuilder;
///
/// // Builds a directly conneted USB device
/// let bus = BusBuilder::new()
///     .build()
///     .unwrap();
/// ```
///
/// ## A more complex example
///
/// ```no_run
/// use xum1541::{BusBuilder, BUS_DEFAULT_TIMEOUT};
/// use std::time::Duration;
///
/// let bus = BusBuilder::new()
///     .serial(1)
///     .timeout(BUS_DEFAULT_TIMEOUT)
///     .build()
///     .unwrap();
/// ```
///
///  ## A Remote USB exaple
///
/// ```no_run
/// use xum1541::BusBuilder;
///
/// // Builds a remotely attached device
/// let bus = BusBuilder::new()
///     .remote_str("127.0.0.1:6767").unwrap()
///     .build().unwrap();
/// ```
pub struct BusBuilder {
    usb_context: Option<rusb::Context>,
    serial_num: Option<u8>,
    remote_addr: Option<SocketAddr>,
    timeout: Option<Duration>,
}

#[allow(dead_code)]
impl BusBuilder {
    pub fn new() -> Self {
        Self::default()
    }

    /// Creates a new [`BusBuilder`] instance with default values.
    ///
    /// All fields are initialized to None and can be set using the builder methods.
    ///
    /// # Returns
    /// * [`BusBuilder`] - new builder instance with default values
    ///
    /// # Example
    /// See [`BusBuilder`] documentation
    pub fn default() -> Self {
        BusBuilder {
            usb_context: None,
            serial_num: None,
            remote_addr: None,
            timeout: None,
        }
    }

    /// Builds and returns a new [`Bus`] instance using the configured parameters.
    ///
    /// # Returns
    /// * `Ok(Bus)` - the constructed Bus instance if successful
    /// * `Err(Error)` - if an error occurred during construction
    ///
    /// # Example
    /// See [`BusBuilder`]
    ///
    /// # Notes:
    /// Uses default values for any parameters that weren't set
    pub fn build(&mut self) -> Result<Bus, Error> {
        let device = match self.usb_context {
            Some(_) => {
                trace!("Creating local USB device");
                self.create_usb_device()
            }
            None => {
                trace!("Creating remote USB device");
                self.create_remote_device()
            }
        }?;

        // Create Bus
        let timeout = self.timeout.unwrap_or(BUS_DEFAULT_TIMEOUT);
        Ok(Bus::new(device, timeout))
    }

    /// Sets the XUM-1541 device serial number to use.
    ///
    /// # Args:
    /// * serial - the serial number to match when finding the device
    ///
    /// # Returns
    /// * `&mut Self` - builder instance for method chaining
    ///
    /// # Note:
    /// If not set, serial defaults to 0 when building
    pub fn serial(&mut self, serial: u8) -> &mut Self {
        self.serial_num = Some(serial);
        self
    }

    /// Sets the timeout duration for bus operations.
    ///
    /// # Args:
    /// * duration - the period to use for the Bus timeout
    ///
    /// # Returns
    /// * `&mut Self` - builder instance for method chaining
    ///
    /// # Note:
    /// If not set, defaults to [`BUS_DEFAULT_TIMEOUT`] when building
    pub fn timeout(&mut self, duration: Duration) -> &mut Self {
        self.timeout = Some(duration);
        self
    }
}

/// Local USB specific build functions
impl BusBuilder {
    /// Sets a custom USB context for device communication.
    ///
    /// This allows setting the USB debug log level via context.set_log_level()
    /// using rusb::LogLevel.
    ///
    /// # Args:
    /// * context - the [`rusb::Context`] to use
    ///
    /// # Returns
    /// * `Self` - builder instance for method chaining
    ///
    /// ```rust,no_run
    /// use rusb::{Context, UsbContext};
    /// use xum1541::BusBuilder;
    ///
    /// let mut context = Context::new().unwrap();
    /// context.set_log_level(rusb::LogLevel::Debug);
    ///
    /// let bus = BusBuilder::new()
    ///     .context(context)
    ///     .build()
    ///     .unwrap();
    /// ```
    ///
    /// # Note:
    /// If not set, a new default [`rusb::Context`] will be created with LogLevel::Info
    pub fn context(mut self, context: Context) -> Self {
        self.usb_context = Some(context);
        self
    }

    /// Create the Device instance using the configured parameters
    fn create_usb_device(&mut self) -> Result<DeviceType, Error> {
        // Get or create the context
        if self.usb_context.is_none() {
            let mut context = rusb::Context::new()?;
            context.set_log_level(rusb::LogLevel::Info);
            self.usb_context = Some(context);
        };

        // Create the Device - this involves consuming the stored context
        let device_config = UsbDeviceConfig {
            context: self.usb_context.take(),
            serial_num: self.serial_num,
        };
        DeviceType::new(DeviceConfig::Usb(device_config))
    }
}

/// Remote USB build functions
impl BusBuilder {
    /// Sets a remote address to default
    ///
    ///
    /// # Returns
    /// * `Result<&mut Self, Error>` - builder instance for method chaining or error
    pub fn remote_default(&mut self) -> Result<&mut Self, Error> {
        self.remote_str(&format!("{}:{}", DEFAULT_ADDR, DEFAULT_PORT))
    }

    /// Sets the remote device address using an already parsed SocketAddr.
    ///
    /// # Args:
    /// * addr - the socket address (IPv4 or IPv6) to connect to
    ///
    /// # Returns
    /// * `&mut Self` - builder instance for method chaining
    pub fn remote(&mut self, addr: SocketAddr) -> Result<&mut Self, Error> {
        if self.usb_context.is_none() {
            self.remote_addr = Some(addr);
            Ok(self)
        } else {
            Err(Error::Init {
                message: "Cannot set remote address when (local) USB context is already configured"
                    .to_string(),
            })
        }
    }

    /// Sets the remote device address using a string representation.
    ///
    /// # Args:
    /// * addr - String in format `HOST:PORT` where:
    ///   - IPv4 addresses: `192.168.1.1:9999`
    ///   - IPv6 addresses: must use square brackets `\[2001:db8::1\]:9999``
    ///   - Hostnames: `example.com:9999`
    ///
    /// # Returns
    /// * `Result<&mut Self, Error>` - builder instance for method chaining or error
    pub fn remote_str(&mut self, addr: &str) -> Result<&mut Self, Error> {
        let addr = addr
            .to_socket_addrs()
            .map_err(|e| Error::DeviceAccess {
                kind: DeviceAccessError::AddressResolution {
                    message: format!("Failed to resolve address: {}", e),
                    errno: e.raw_os_error().unwrap_or(libc::EINVAL),
                },
            })?
            .next()
            .ok_or_else(|| Error::DeviceAccess {
                kind: DeviceAccessError::AddressResolution {
                    message: "Could not resolve address".to_string(),
                    errno: libc::EAI_NONAME as i32,
                },
            })?;
        self.remote(addr)
    }

    /// Sets the remote device IPv4 address using a string representation.
    ///
    /// # Args:
    /// * addr - String in IPv4 format (e.g. "192.168.1.100")
    /// * port - Port number
    ///
    /// # Returns
    /// * `Result<&mut Self, Error>` - builder instance for method chaining or error
    pub fn remote_ipv4_str(&mut self, addr: &str, port: u16) -> Result<&mut Self, Error> {
        let ip = Ipv4Addr::from_str(addr).map_err(|e| Error::DeviceAccess {
            kind: DeviceAccessError::AddressResolution {
                message: format!("Invalid IPv4 address: {}", e),
                errno: libc::EINVAL,
            },
        })?;
        self.remote(SocketAddr::new(IpAddr::V4(ip), port))
    }

    /// Sets the remote device IPv6 address using a string representation.
    ///
    /// # Args:
    /// * addr - String in IPv6 format (e.g. "::1" or "2001:db8::1")
    /// * port - Port number
    ///
    /// # Returns
    /// * `Result<&mut Self, Error>` - builder instance for method chaining or error
    pub fn remote_ipv6_str(&mut self, addr: &str, port: u16) -> Result<&mut Self, Error> {
        let ip = Ipv6Addr::from_str(addr).map_err(|e| Error::DeviceAccess {
            kind: DeviceAccessError::AddressResolution {
                message: format!("Invalid IPv6 address: {}", e),
                errno: libc::EINVAL,
            },
        })?;
        self.remote(SocketAddr::new(IpAddr::V6(ip), port))
    }

    /// Create the Device instance using the configured parameters
    fn create_remote_device(&mut self) -> Result<DeviceType, Error> {
        let device_config = RemoteUsbDeviceConfig {
            serial_num: self.serial_num,
            remote_addr: self.remote_addr,
        };
        DeviceType::new(DeviceConfig::RemoteUsb(device_config))
    }
}
