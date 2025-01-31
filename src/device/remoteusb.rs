//! [`RemoteUsbDevice`] provides a network interface on top of [`UsbDevice`]
//! allowing clients to access the xum1541 over the network.
//!
//! # Example
//!
//! [remote example](examples/remote.rs) provides an example of connecting to the
//! server and driving the [`UsbDevice`] over the work.
//!
//! [`device-server`](crate::bin::device-server) uses [`RemoteUsbDevice`] to provide a complete
//! server implementation

use crate::constants::{Ioctl, CTRL_SHUTDOWN};
use crate::device::SwDebugInfo;
use crate::{CommunicationError, DeviceAccessError, Error};
use crate::{Device, DeviceInfo, SpecificDeviceInfo};
use crate::{UsbDevice, UsbDeviceConfig, UsbInfo};

use bincode::{deserialize, serialize};
#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};
use parking_lot::Mutex;
use serde::{Deserialize, Serialize};
use std::io::{Read, Write};
use std::net::{IpAddr, SocketAddr, TcpListener, TcpStream};
use std::str::FromStr;
use std::sync::Arc;

pub const DEFAULT_PORT: u16 = 1541;
pub const DEFAULT_ADDR: &str = "127.0.0.1";
fn default_socket_addr() -> SocketAddr {
    SocketAddr::new(IpAddr::from_str(DEFAULT_ADDR).unwrap(), DEFAULT_PORT)
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RemoteUsbDeviceConfig {
    pub serial_num: Option<u8>,
    pub remote_addr: Option<SocketAddr>,
}

impl Default for RemoteUsbDeviceConfig {
    fn default() -> Self {
        Self {
            serial_num: None,
            remote_addr: None,
        }
    }
}

impl From<UsbDeviceConfig> for RemoteUsbDeviceConfig {
    fn from(config: UsbDeviceConfig) -> Self {
        RemoteUsbDeviceConfig {
            serial_num: config.serial_num,
            remote_addr: None,
        }
    }
}

impl From<RemoteUsbDeviceConfig> for UsbDeviceConfig {
    fn from(config: RemoteUsbDeviceConfig) -> Self {
        UsbDeviceConfig {
            context: None,
            serial_num: config.serial_num,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RemoteUsbInfo {
    /// Standard UsbInfo retrieved over the network from UsbDevice
    pub usb_info: Option<UsbInfo>,

    /// Remote specific info, containing data about the connection to
    /// UsbDevice
    pub bind_addr: SocketAddr,
}

impl SpecificDeviceInfo for RemoteUsbInfo {
    type Info = RemoteUsbInfo;

    fn print(&self) {
        if self.usb_info.is_some() {
            self.usb_info.clone().unwrap().print();
        }
        println!("  - bind address: {}", self.bind_addr);
    }
}

struct UsbConfigWrapper(Option<UsbDeviceConfig>);
impl From<UsbConfigWrapper> for Option<RemoteUsbDeviceConfig> {
    fn from(wrapper: UsbConfigWrapper) -> Self {
        wrapper.0.map(|c| c.into())
    }
}

struct RemoteUsbConfigWrapper(Option<RemoteUsbDeviceConfig>);
impl From<RemoteUsbConfigWrapper> for Option<UsbDeviceConfig> {
    fn from(config: RemoteUsbConfigWrapper) -> Self {
        config.0.map(|c| c.into())
    }
}

// Shared protocol definitions
#[derive(Debug, Serialize, Deserialize)]
enum RemoteDeviceRequest {
    New(Option<RemoteUsbDeviceConfig>),
    Init,
    HardResetAndReInit,
    ReadControl {
        request: u8,
        value: u16,
        buffer_size: usize,
    },
    WriteControl {
        request: u8,
        value: u16,
        buffer: Vec<u8>,
    },
    WriteData {
        mode: u8,
        data: Vec<u8>,
    },
    ReadData {
        mode: u8,
        buffer_size: usize,
    },
    WaitForStatus,
    Ioctl {
        ioctl: Ioctl,
        address: u8,
        secondary_address: u8,
    },
}

impl std::fmt::Display for RemoteDeviceRequest {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RemoteDeviceRequest::New(config) => write!(f, "New({:?})", config),
            RemoteDeviceRequest::Init => write!(f, "Init"),
            RemoteDeviceRequest::HardResetAndReInit => write!(f, "HardResetAndReInit"),
            RemoteDeviceRequest::ReadControl {
                request,
                value,
                buffer_size,
            } => {
                write!(
                    f,
                    "ReadControl {{ request: {}, value: {}, buffer_size: {} }}",
                    request, value, buffer_size
                )
            }
            RemoteDeviceRequest::WriteControl {
                request,
                value,
                buffer,
            } => {
                write!(
                    f,
                    "WriteControl {{ request: {}, value: {}, buffer: {:?} }}",
                    request, value, buffer
                )
            }
            RemoteDeviceRequest::WriteData { mode, data } => {
                write!(f, "WriteData {{ mode: {}, data: {:?} }}", mode, data)
            }
            RemoteDeviceRequest::ReadData { mode, buffer_size } => {
                write!(
                    f,
                    "ReadData {{ mode: {}, buffer_size: {} }}",
                    mode, buffer_size
                )
            }
            RemoteDeviceRequest::WaitForStatus => write!(f, "WaitForStatus"),
            RemoteDeviceRequest::Ioctl {
                ioctl,
                address,
                secondary_address,
            } => {
                write!(
                    f,
                    "Ioctl {{ ioctl: {:?}, address: {}, secondary_address: {} }}",
                    ioctl, address, secondary_address
                )
            }
        }
    }
}

#[derive(Debug, Serialize, Deserialize)]
enum RemoteDeviceResponse {
    New(Result<(), Error>),
    Init(Result<(DeviceInfo, UsbInfo), Error>),
    HardResetAndReInit(Result<(), Error>),
    ReadControl(Result<(Vec<u8>, usize), Error>),
    WriteControl(Result<(), Error>),
    WriteData(Result<usize, Error>),
    ReadData(Result<(Vec<u8>, usize), Error>),
    WaitForStatus(Result<u16, Error>),
    Ioctl(Result<Option<u16>, Error>),
}

impl std::fmt::Display for RemoteDeviceResponse {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RemoteDeviceResponse::New(result) => write!(f, "New({:?})", result),
            RemoteDeviceResponse::Init(result) => write!(f, "Init({:?})", result),
            RemoteDeviceResponse::HardResetAndReInit(result) => {
                write!(f, "HardResetAndReInit({:?})", result)
            }
            RemoteDeviceResponse::ReadControl(result) => write!(f, "ReadControl({:?})", result),
            RemoteDeviceResponse::WriteControl(result) => write!(f, "WriteControl({:?})", result),
            RemoteDeviceResponse::WriteData(result) => write!(f, "WriteData({:?})", result),
            RemoteDeviceResponse::ReadData(result) => write!(f, "ReadData({:?})", result),
            RemoteDeviceResponse::WaitForStatus(result) => write!(f, "WaitForStatus({:?})", result),
            RemoteDeviceResponse::Ioctl(result) => write!(f, "Ioctl({:?})", result),
        }
    }
}

// Client implementation
#[derive(Debug, Clone)]
pub struct DeviceClient {
    stream: Option<Arc<Mutex<TcpStream>>>,
    device_config: RemoteUsbDeviceConfig,
    device_info: Option<DeviceInfo>,
    remote_usb_info: RemoteUsbInfo,
    sw_debug_info: SwDebugInfo,
}
pub use DeviceClient as RemoteUsbDevice;

impl DeviceClient {
    fn connect(config: Option<RemoteUsbDeviceConfig>) -> Result<Self, Error> {
        trace!("DeviceClient::connect");
        let remote_addr = match &config {
            Some(config) => config.remote_addr.unwrap_or(default_socket_addr()),
            None => default_socket_addr(),
        };

        let stream = TcpStream::connect(remote_addr).map_err(|e| Error::DeviceAccess {
            kind: DeviceAccessError::NetworkConnection {
                message: format!("Failed to connect: {}", e),
                errno: e.raw_os_error().unwrap_or(libc::EIO),
            },
        })?;

        let local_addr = stream.local_addr().map_err(|e| Error::DeviceAccess {
            kind: DeviceAccessError::NetworkConnection {
                message: format!("Failed to get local address: {}", e),
                errno: e.raw_os_error().unwrap_or(libc::EIO),
            },
        })?;
        let remote_usb_info = RemoteUsbInfo {
            usb_info: None,
            bind_addr: local_addr,
        };

        Ok(Self {
            stream: Some(Arc::new(Mutex::new(stream))),
            device_config: RemoteUsbDeviceConfig {
                serial_num: config.and_then(|c| c.serial_num),
                remote_addr: Some(remote_addr),
            },
            device_info: None,
            remote_usb_info: remote_usb_info,
            sw_debug_info: SwDebugInfo::default(),
        })
    }

    fn stream_or_err(&mut self) -> Result<Arc<Mutex<TcpStream>>, Error> {
        trace!("DeviceClient::stream_or_err");
        match &self.stream {
            Some(stream) => Ok(stream.clone()),
            None => Err(Error::DeviceAccess {
                kind: DeviceAccessError::NetworkConnection {
                    message: "Stream not initialized".to_string(),
                    errno: libc::EIO,
                },
            }),
        }
    }

    fn send_request(
        &mut self,
        request: RemoteDeviceRequest,
    ) -> Result<RemoteDeviceResponse, Error> {
        trace!("DeviceClient::send_request");
        let stream = self.stream_or_err()?;

        let data = serialize(&request).map_err(|e| Error::DeviceAccess {
            kind: DeviceAccessError::NetworkConnection {
                message: format!("Serialization error: {}", e),
                errno: libc::EIO, // Serialization errors don't have OS error codes
            },
        })?;

        let response_data = {
            let mut guard = stream.lock();

            // Send length prefix
            guard
                .write_all(&(data.len() as u64).to_le_bytes())
                .map_err(|e| Error::DeviceAccess {
                    kind: DeviceAccessError::NetworkConnection {
                        message: format!("Failed to send message length: {}", e),
                        errno: e.raw_os_error().unwrap_or(libc::EIO),
                    },
                })?;

            // Send data
            guard.write_all(&data).map_err(|e| Error::DeviceAccess {
                kind: DeviceAccessError::NetworkConnection {
                    message: format!("Failed to send message: {}", e),
                    errno: e.raw_os_error().unwrap_or(libc::EIO),
                },
            })?;

            // Read response length
            let mut len_buf = [0u8; 8];
            guard
                .read_exact(&mut len_buf)
                .map_err(|e| Error::DeviceAccess {
                    kind: DeviceAccessError::NetworkConnection {
                        message: format!("Failed to read response length: {}", e),
                        errno: e.raw_os_error().unwrap_or(libc::EIO),
                    },
                })?;
            let len = u64::from_le_bytes(len_buf) as usize;

            // Read response data
            let mut response_data = vec![0u8; len];
            guard
                .read_exact(&mut response_data)
                .map_err(|e| Error::DeviceAccess {
                    kind: DeviceAccessError::NetworkConnection {
                        message: format!("Failed to read response: {}", e),
                        errno: e.raw_os_error().unwrap_or(libc::EIO),
                    },
                })?;

            response_data
        };

        deserialize(&response_data).map_err(|e| Error::DeviceAccess {
            kind: DeviceAccessError::NetworkConnection {
                message: format!("Failed to deserialize response: {}", e),
                errno: libc::EIO, // Deserialization errors don't have OS error codes
            },
        })
    }

    fn unexpected_response(request: &str, rsp: RemoteDeviceResponse) -> Error {
        trace!("DeviceClient::unexpected_response");
        Error::Communication {
            kind: CommunicationError::Remote {
                message: format!("Unexpected response for {}: {}", request, rsp),
                errno: libc::EIO,
            },
        }
    }
}

impl Device for DeviceClient {
    type Config = RemoteUsbDeviceConfig;
    type SpecificDeviceInfo = RemoteUsbInfo;

    fn new(config: Option<Self::Config>) -> Result<Self, Error> {
        trace!("RemoteUsbDevice::new");
        let mut client = DeviceClient::connect(config)?;

        let usb_config = UsbDeviceConfig {
            context: None,
            serial_num: client.device_config.serial_num,
        };
        match client.send_request(RemoteDeviceRequest::New(
            UsbConfigWrapper(Some(usb_config)).into(),
        ))? {
            RemoteDeviceResponse::New(result) => match result {
                Ok(()) => {
                    result?;
                    Ok(client)
                }
                Err(e) => Err(e),
            },
            rsp => Err(Self::unexpected_response("New", rsp)),
        }
    }

    fn init(&mut self) -> Result<(), Error> {
        trace!("RemoteUsbDevice::init");
        self.sw_debug_info.increment_api_call();
        match self.send_request(RemoteDeviceRequest::Init)? {
            RemoteDeviceResponse::Init(result) => match result {
                Ok((device_info, usb_info)) => {
                    self.device_info = Some(device_info);
                    self.remote_usb_info.usb_info = Some(usb_info);
                    Ok(())
                }
                Err(e) => Err(e),
            },
            rsp => Err(Self::unexpected_response("Init", rsp)),
        }
    }

    fn info(&mut self) -> Option<DeviceInfo> {
        trace!("RemoteUsbDevice::info");
        self.device_info.clone()
    }

    fn specific_info(&mut self) -> Option<Self::SpecificDeviceInfo> {
        trace!("RemoteUsbDevice::specific_info");
        Some(self.remote_usb_info.clone())
    }

    fn sw_debug_info(&mut self) -> SwDebugInfo {
        trace!("RemoteUsbDevice::sw_debug_info");
        self.sw_debug_info.clone()
    }

    fn hard_reset_and_re_init(&mut self) -> Result<(), Error> {
        trace!("RemoteUsbDevice::hard_reset_and_re_init");
        self.sw_debug_info.increment_api_call();
        match self.send_request(RemoteDeviceRequest::HardResetAndReInit)? {
            RemoteDeviceResponse::HardResetAndReInit(result) => result,
            rsp => Err(Self::unexpected_response("HardResetAndReInit", rsp)),
        }
    }

    fn read_control(&mut self, request: u8, value: u16, buffer: &mut [u8]) -> Result<usize, Error> {
        trace!("RemoteUsbDevice::read_control");
        self.sw_debug_info.increment_api_call();
        match self.send_request(RemoteDeviceRequest::ReadControl {
            request,
            value,
            buffer_size: buffer.len(),
        })? {
            RemoteDeviceResponse::ReadControl(Ok((data, len))) => {
                buffer[..data.len()].copy_from_slice(&data);
                Ok(len)
            }
            RemoteDeviceResponse::ReadControl(Err(e)) => Err(e),
            rsp => Err(Self::unexpected_response("ReadControl", rsp)),
        }
    }

    fn write_control(&mut self, request: u8, value: u16, buffer: &[u8]) -> Result<(), Error> {
        trace!("RemoteUsbDevice::write_control");
        self.sw_debug_info.increment_api_call();
        match self.send_request(RemoteDeviceRequest::WriteControl {
            request,
            value,
            buffer: buffer.to_vec(),
        })? {
            RemoteDeviceResponse::WriteControl(result) => result,
            rsp => Err(Self::unexpected_response("WriteControl", rsp)),
        }
    }

    fn write_data(&mut self, mode: u8, data: &[u8]) -> Result<usize, Error> {
        trace!("RemoteUsbDevice::write_data");
        self.sw_debug_info.increment_api_call();
        match self.send_request(RemoteDeviceRequest::WriteData {
            mode,
            data: data.to_vec(),
        })? {
            RemoteDeviceResponse::WriteData(result) => result,
            rsp => Err(Self::unexpected_response("WriteData", rsp)),
        }
    }

    fn read_data(&mut self, mode: u8, buffer: &mut [u8]) -> Result<usize, Error> {
        trace!("RemoteUsbDevice::read_data");
        self.sw_debug_info.increment_api_call();
        match self.send_request(RemoteDeviceRequest::ReadData {
            mode,
            buffer_size: buffer.len(),
        })? {
            RemoteDeviceResponse::ReadData(Ok((data, len))) => {
                buffer[..data.len()].copy_from_slice(&data);
                Ok(len)
            }
            RemoteDeviceResponse::ReadData(Err(e)) => Err(e),
            rsp => Err(Self::unexpected_response("ReadData", rsp)),
        }
    }

    fn wait_for_status(&mut self) -> Result<u16, Error> {
        trace!("RemoteUsbDevice::wait_for_status");
        self.sw_debug_info.increment_api_call();
        match self.send_request(RemoteDeviceRequest::WaitForStatus)? {
            RemoteDeviceResponse::WaitForStatus(result) => result,
            rsp => Err(Self::unexpected_response("WaitForStatus", rsp)),
        }
    }

    fn ioctl(
        &mut self,
        ioctl: Ioctl,
        address: u8,
        secondary_address: u8,
    ) -> Result<Option<u16>, Error> {
        trace!("RemoteUsbDevice::ioctl");
        self.sw_debug_info.increment_api_call();
        match self.send_request(RemoteDeviceRequest::Ioctl {
            ioctl,
            address,
            secondary_address,
        })? {
            RemoteDeviceResponse::Ioctl(result) => result,
            rsp => Err(Self::unexpected_response("Ioctl", rsp)),
        }
    }
}

// Server implementation
pub struct UsbDeviceServer {
    device: UsbDevice,
    _bind_addr: SocketAddr,
    listener: TcpListener,
}

impl UsbDeviceServer {
    pub fn new(device: UsbDevice, bind_addr: SocketAddr) -> Result<Self, Error> {
        trace!("UsbDeviceServer::new");
        let listener = TcpListener::bind(bind_addr).map_err(|e| Error::Communication {
            kind: CommunicationError::Remote {
                message: format!("Failed to bind server: {}", e),
                errno: libc::EIO,
            },
        })?;
        Ok(UsbDeviceServer {
            device,
            _bind_addr: bind_addr,
            listener,
        })
    }

    /// Runs the USB Device server until a fatal error is hit, then returns
    ///
    // Note there is no need here to Drop the UsbDevice.  We sent a shutdown
    // to it within serve_one_connection, and when this object gets dropped,
    // so will the [`UsbDevice`], which will trigger a release of the USB
    // interface
    pub fn serve(&mut self) -> Result<(), Error> {
        trace!("UsbDeviceServer::serve");
        loop {
            match self.serve_one_connection() {
                Ok(_) => continue,
                Err(e) => match e {
                    Error::Communication { kind } => match kind {
                        CommunicationError::RemoteDisconnected { .. } => {
                            continue;
                        }
                        _ => {
                            error!("Hit unexpected error: {:?}", kind);
                            break Err(Error::Communication { kind });
                        }
                    },
                    _ => {
                        error!("Hi unexpedcted error: {}", e);
                        break Err(e);
                    }
                },
            }
        }
    }

    /// Runs the USB Device serving a single connection - then returns
    fn serve_one_connection(&mut self) -> Result<(), Error> {
        trace!("UsbDeviceServer::serve_one_connection");
        // Since this is 1:1, we only accept one connection
        let (mut stream, _) = self.listener.accept().map_err(|e| Error::Communication {
            kind: CommunicationError::Remote {
                message: format!("Failed to accept connection: {}", e),
                errno: libc::EIO,
            },
        })?;

        let peer_addr = stream
            .peer_addr()
            .inspect_err(|e| warn!("Failed to get remote peer address {}", e));
        let peer_addr_str = match peer_addr {
            Ok(peer_addr) => format!("{}", peer_addr),
            _ => "Unknown".to_string(),
        };
        info!("Remote client {} connected", peer_addr_str);

        loop {
            // TO DO - also when they disconnect we need to drop the connection that we made to the XUm1541
            // Read request length
            let mut len_buf = [0u8; 8];
            match stream.read_exact(&mut len_buf) {
                Ok(_) => {}
                Err(e) => match e.kind() {
                    std::io::ErrorKind::ConnectionReset
                    | std::io::ErrorKind::ConnectionAborted
                    | std::io::ErrorKind::UnexpectedEof => {
                        info!("Remote client {} disconnected", peer_addr_str);
                        trace!("Disconnect error information: {e}");
                        // Send a shutdown message to the xum1541 to cleanly
                        // shut it down
                        if let Err(e) = self.device.write_control(CTRL_SHUTDOWN, 0, &[]) {
                            warn!("Failed to cleanly shutodwn trace on Disconne error information:: {e}");
                        }
                        return Err(Error::Communication {
                            kind: CommunicationError::RemoteDisconnected {
                                message: format!(
                                    "Client disconnected: error: {} kind: {}",
                                    e,
                                    e.kind()
                                ),
                                errno: e.raw_os_error().unwrap_or_default(),
                            },
                        });
                    }
                    _ => {
                        return Err(Error::Communication {
                            kind: CommunicationError::Remote {
                                message: format!(
                                    "Hit error reading data error: {} kind: {}",
                                    e,
                                    e.kind()
                                ),
                                errno: e.raw_os_error().unwrap_or_default(),
                            },
                        })
                    }
                },
            };
            let len = u64::from_le_bytes(len_buf) as usize;

            // Read request data
            let mut request_data = vec![0u8; len];
            stream
                .read_exact(&mut request_data)
                .map_err(|e| Error::Communication {
                    kind: CommunicationError::Remote {
                        message: format!("Failed to read message: {}", e),
                        errno: libc::EIO,
                    },
                })?;

            let request: RemoteDeviceRequest =
                deserialize(&request_data).map_err(|e| Error::Communication {
                    kind: CommunicationError::Remote {
                        message: format!("Failed to deserialize request: {}", e),
                        errno: libc::EIO,
                    },
                })?;

            let response = match request {
                RemoteDeviceRequest::New(_config) => RemoteDeviceResponse::New(Ok(())),
                RemoteDeviceRequest::Init => {
                    let rsp = self.device.init();
                    // The network API version of Init returns DeviceInfo
                    // and UsbInfo
                    // device.init() (if it succeeds) always sets these both
                    // to Some, so we're OK to unwrap here
                    let new_rsp = match rsp {
                        Ok(_) => Ok((
                            self.device.info().unwrap(),
                            self.device.specific_info().unwrap(),
                        )),
                        Err(e) => Err(e),
                    };
                    RemoteDeviceResponse::Init(new_rsp)
                }
                RemoteDeviceRequest::HardResetAndReInit => {
                    RemoteDeviceResponse::HardResetAndReInit(self.device.hard_reset_and_re_init())
                }
                RemoteDeviceRequest::ReadControl {
                    request,
                    value,
                    buffer_size,
                } => {
                    let mut buffer = vec![0u8; buffer_size];
                    match self.device.read_control(request, value, &mut buffer) {
                        Ok(len) => {
                            RemoteDeviceResponse::ReadControl(Ok((buffer[..len].to_vec(), len)))
                        }
                        Err(e) => RemoteDeviceResponse::ReadControl(Err(e)),
                    }
                }
                RemoteDeviceRequest::WriteControl {
                    request,
                    value,
                    buffer,
                } => RemoteDeviceResponse::WriteControl(
                    self.device.write_control(request, value, &buffer),
                ),
                RemoteDeviceRequest::WriteData { mode, data } => {
                    RemoteDeviceResponse::WriteData(self.device.write_data(mode, &data))
                }
                RemoteDeviceRequest::ReadData { mode, buffer_size } => {
                    let mut buffer = vec![0u8; buffer_size];
                    match self.device.read_data(mode, &mut buffer) {
                        Ok(len) => {
                            RemoteDeviceResponse::ReadData(Ok((buffer[..len].to_vec(), len)))
                        }
                        Err(e) => RemoteDeviceResponse::ReadData(Err(e)),
                    }
                }
                RemoteDeviceRequest::WaitForStatus => {
                    RemoteDeviceResponse::WaitForStatus(self.device.wait_for_status())
                }
                RemoteDeviceRequest::Ioctl {
                    ioctl,
                    address,
                    secondary_address,
                } => RemoteDeviceResponse::Ioctl(self.device.ioctl(
                    ioctl,
                    address,
                    secondary_address,
                )),
            };

            let response_data = serialize(&response).map_err(|e| Error::Communication {
                kind: CommunicationError::Remote {
                    message: format!("Failed to serialize response: {}", e),
                    errno: libc::EIO,
                },
            })?;

            stream
                .write_all(&(response_data.len() as u64).to_le_bytes())
                .map_err(|e| Error::Communication {
                    kind: CommunicationError::Remote {
                        message: format!("Failed to send response length: {}", e),
                        errno: libc::EIO,
                    },
                })?;
            stream
                .write_all(&response_data)
                .map_err(|e| Error::Communication {
                    kind: CommunicationError::Remote {
                        message: format!("Failed to send response: {}", e),
                        errno: libc::EIO,
                    },
                })?;
        }
    }
}
