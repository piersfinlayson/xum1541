use std::net::{IpAddr, Ipv4Addr, Ipv6Addr, SocketAddr, ToSocketAddrs};
use std::str::FromStr;
use std::time::Duration;

use crate::{Bus, BusBuilder, BUS_DEFAULT_TIMEOUT};
use crate::{Device, RemoteUsbDevice, RemoteUsbDeviceConfig, RemoteUsbInfo, SpecificDeviceInfo};
use crate::{DeviceAccessError, Error};

pub type RemoteUsbBus = Bus<RemoteUsbDevice>;

impl RemoteUsbBus {
    pub fn default() -> Result<Self, Error> {
        RemoteUsbBusBuilder::new().build()
    }

    pub fn device_specific_info(&self) -> Option<&RemoteUsbInfo> {
        self.device.specific_info()
    }
}
pub struct RemoteUsbBusBuilder {
    device_config: RemoteUsbDeviceConfig,
    address: Option<SocketAddr>,
    timeout: Option<Duration>,
    device: Option<RemoteUsbDevice>,
}

impl BusBuilder for RemoteUsbBusBuilder {
    type D = RemoteUsbDevice;

    fn default() -> Self {
        RemoteUsbBusBuilder {
            device_config: RemoteUsbDeviceConfig::default(),
            address: None,
            timeout: None,
            device: None,
        }
    }

    fn build(&mut self) -> Result<Bus<RemoteUsbDevice>, Error> {
        let device = if self.device.is_none() {
            self.create_device()?
        } else {
            self.device.take().unwrap()
        };

        let timeout = self.timeout.unwrap_or(BUS_DEFAULT_TIMEOUT);
        Ok(Bus::new(device, timeout))
    }

    fn device_config(&mut self, device_config: RemoteUsbDeviceConfig) -> &mut Self {
        self.device_config = device_config;
        self
    }
}

impl RemoteUsbBusBuilder {
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
    pub fn address_str(&mut self, addr: &str) -> Result<&mut Self, Error> {
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
        self.address = Some(addr);
        Ok(self)
    }

    /// Sets the remote device IPv4 address using a string representation.
    ///
    /// # Args:
    /// * addr - String in IPv4 format (e.g. "192.168.1.100")
    /// * port - Port number
    ///
    /// # Returns
    /// * `Result<&mut Self, Error>` - builder instance for method chaining or error
    pub fn ipv4_str(&mut self, addr: &str, port: u16) -> Result<&mut Self, Error> {
        let ip = Ipv4Addr::from_str(addr).map_err(|e| Error::DeviceAccess {
            kind: DeviceAccessError::AddressResolution {
                message: format!("Invalid IPv4 address: {}", e),
                errno: libc::EINVAL,
            },
        })?;
        self.address = Some(SocketAddr::new(IpAddr::V4(ip), port));
        Ok(self)
    }

    /// Sets the remote device IPv6 address using a string representation.
    ///
    /// # Args:
    /// * addr - String in IPv6 format (e.g. "::1" or "2001:db8::1")
    /// * port - Port number
    ///
    /// # Returns
    /// * `Result<&mut Self, Error>` - builder instance for method chaining or error
    pub fn ipv6_str(&mut self, addr: &str, port: u16) -> Result<&mut Self, Error> {
        let ip = Ipv6Addr::from_str(addr).map_err(|e| Error::DeviceAccess {
            kind: DeviceAccessError::AddressResolution {
                message: format!("Invalid IPv6 address: {}", e),
                errno: libc::EINVAL,
            },
        })?;
        self.address = Some(SocketAddr::new(IpAddr::V6(ip), port));
        Ok(self)
    }

    /// Sets the remote device address using an already parsed SocketAddr.
    ///
    /// # Args:
    /// * addr - the socket address (IPv4 or IPv6) to connect to
    ///
    /// # Returns
    /// * `&mut Self` - builder instance for method chaining
    pub fn address(&mut self, addr: SocketAddr) -> &mut Self {
        self.address = Some(addr);
        self
    }

    /// Sets the timeout duration for bus operations.
    ///
    /// # Args:
    /// * duration - the period to use for the Bus timeout
    ///
    /// # Returns
    /// * `&mut Self` - builder instance for method chaining
    pub fn timeout(&mut self, duration: Duration) -> &mut Self {
        self.timeout = Some(duration);
        self
    }

    /// Create the Device instance using the configured parameters
    fn create_device(&mut self) -> Result<RemoteUsbDevice, Error> {
        RemoteUsbDevice::new(Some(self.device_config.clone()))
    }
}
