use bincode::{deserialize, serialize};
use serde::{Deserialize, Serialize};
use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};

use crate::constants::Ioctl;
use crate::CommunicationKind;
use crate::Error;
use crate::{Device, DeviceInfo};
use crate::{UsbDevice, UsbDeviceConfig};
use parking_lot::Mutex;

// TODO - should have RemoteUsbInfo or UsbInfo

#[derive(Debug, Serialize, Deserialize)]
pub struct RemoteUsbDeviceConfig {
    pub serial_num: Option<u8>,
}

impl From<UsbDeviceConfig> for RemoteUsbDeviceConfig {
    fn from(config: UsbDeviceConfig) -> Self {
        RemoteUsbDeviceConfig {
            serial_num: config.serial_num,
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
#[derive(Serialize, Deserialize)]
enum RemoteDeviceRequest {
    New(Option<RemoteUsbDeviceConfig>),
    Init,
    CurrentConfig,
    Info,
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

#[derive(Serialize, Deserialize)]
enum RemoteDeviceResponse {
    New(Result<(), Error>),
    Init(Result<(), Error>),
    CurrentConfig(Option<RemoteUsbDeviceConfig>),
    Info(Option<DeviceInfo>),
    HardResetAndReInit(Result<(), Error>),
    ReadControl(Result<(Vec<u8>, usize), Error>),
    WriteControl(Result<(), Error>),
    WriteData(Result<usize, Error>),
    ReadData(Result<(Vec<u8>, usize), Error>),
    WaitForStatus(Result<u16, Error>),
    Ioctl(Result<Option<u16>, Error>),
}

// Client implementation
pub struct DeviceClient {
    stream: Mutex<TcpStream>,
}
pub use DeviceClient as RemoteUsbDevice;

impl DeviceClient {
    pub(crate) fn connect(addr: &str) -> Result<Self, Error> {
        let stream = TcpStream::connect(addr).map_err(|e| Error::Communication {
            kind: CommunicationKind::Remote(format!("Failed to connect: {}", e)),
        })?;
        Ok(DeviceClient {
            stream: Mutex::new(stream),
        })
    }

    fn send_request(&self, request: RemoteDeviceRequest) -> Result<RemoteDeviceResponse, Error> {
        let mut stream = self.stream.lock();

        let data = serialize(&request).map_err(|e| Error::Communication {
            kind: CommunicationKind::Remote(format!("Serialization error: {}", e)),
        })?;

        // Send length prefix
        stream
            .write_all(&(data.len() as u64).to_le_bytes())
            .map_err(|e| Error::Communication {
                kind: CommunicationKind::Remote(format!("Failed to send message length: {}", e)),
            })?;
        // Send data
        stream.write_all(&data).map_err(|e| Error::Communication {
            kind: CommunicationKind::Remote(format!("Failed to send message: {}", e)),
        })?;

        // Read response length
        let mut len_buf = [0u8; 8];
        stream
            .read_exact(&mut len_buf)
            .map_err(|e| Error::Communication {
                kind: CommunicationKind::Remote(format!("Failed to read response length: {}", e)),
            })?;
        let len = u64::from_le_bytes(len_buf) as usize;

        // Read response data
        let mut response_data = vec![0u8; len];
        stream
            .read_exact(&mut response_data)
            .map_err(|e| Error::Communication {
                kind: CommunicationKind::Remote(format!("Failed to read response: {}", e)),
            })?;

        deserialize(&response_data).map_err(|e| Error::Communication {
            kind: CommunicationKind::Remote(format!("Failed to deserialize response: {}", e)),
        })
    }
}

impl Device for DeviceClient {
    type Config = UsbDeviceConfig;

    fn new(config: Option<Self::Config>) -> Result<Self, Error> {
        let client = DeviceClient::connect("127.0.0.1:9999")?;

        match client.send_request(RemoteDeviceRequest::New(UsbConfigWrapper(config).into()))? {
            RemoteDeviceResponse::New(result) => match result {
                Ok(()) => {
                    result?;
                    Ok(client)
                }
                Err(e) => Err(e),
            },
            _ => Err(Error::Communication {
                kind: CommunicationKind::Remote(
                    "Unexpected response type for New request".to_string(),
                ),
            }),
        }
    }

    fn current_config(&self) -> Option<&Self::Config> {
        match self.send_request(RemoteDeviceRequest::CurrentConfig) {
            Ok(RemoteDeviceResponse::CurrentConfig(config)) => {
                let serial_num = config.as_ref().and_then(|c| c.serial_num);

                // Create the config first
                let usb_device_config = UsbDeviceConfig {
                    context: None,
                    serial_num,
                };
                let boxed_config = Box::new(usb_device_config);

                // TODO fix memory leak - somehow?!
                let config_ref: &'static UsbDeviceConfig = Box::leak(boxed_config);

                Some(config_ref)
            }
            _ => None,
        }
    }

    fn init(&mut self) -> Result<(), Error> {
        match self.send_request(RemoteDeviceRequest::Init)? {
            RemoteDeviceResponse::Init(result) => result,
            _ => Err(Error::Communication {
                kind: CommunicationKind::Remote(
                    "Unexpected response type for Init request".to_string(),
                ),
            }),
        }
    }

    fn info(&self) -> Option<&DeviceInfo> {
        match self.send_request(RemoteDeviceRequest::Info) {
            Ok(RemoteDeviceResponse::Info(info)) => {
                match info {
                    Some(info) => {
                        // Create the info first
                        let boxed_info = Box::new(info);

                        // TODO fix memory leak - somehow?!
                        let info_ref: &'static DeviceInfo = Box::leak(boxed_info);

                        Some(info_ref)
                    }
                    None => None,
                }
            }
            _ => None,
        }
    }

    fn hard_reset_and_re_init(&mut self) -> Result<(), Error> {
        match self.send_request(RemoteDeviceRequest::HardResetAndReInit)? {
            RemoteDeviceResponse::HardResetAndReInit(result) => result,
            _ => Err(Error::Communication {
                kind: CommunicationKind::Remote(
                    "Unexpected response type for HardResetAndReInit request".to_string(),
                ),
            }),
        }
    }

    fn read_control(&self, request: u8, value: u16, buffer: &mut [u8]) -> Result<usize, Error> {
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
            _ => Err(Error::Communication {
                kind: CommunicationKind::Remote(
                    "Unexpected response type for ReadControl request".to_string(),
                ),
            }),
        }
    }

    fn write_control(&self, request: u8, value: u16, buffer: &[u8]) -> Result<(), Error> {
        match self.send_request(RemoteDeviceRequest::WriteControl {
            request,
            value,
            buffer: buffer.to_vec(),
        })? {
            RemoteDeviceResponse::WriteControl(result) => result,
            _ => Err(Error::Communication {
                kind: CommunicationKind::Remote(
                    "Unexpected response type for WriteControl request".to_string(),
                ),
            }),
        }
    }

    fn write_data(&self, mode: u8, data: &[u8]) -> Result<usize, Error> {
        match self.send_request(RemoteDeviceRequest::WriteData {
            mode,
            data: data.to_vec(),
        })? {
            RemoteDeviceResponse::WriteData(result) => result,
            _ => Err(Error::Communication {
                kind: CommunicationKind::Remote(
                    "Unexpected response type for WriteData request".to_string(),
                ),
            }),
        }
    }

    fn read_data(&self, mode: u8, buffer: &mut [u8]) -> Result<usize, Error> {
        match self.send_request(RemoteDeviceRequest::ReadData {
            mode,
            buffer_size: buffer.len(),
        })? {
            RemoteDeviceResponse::ReadData(Ok((data, len))) => {
                buffer[..data.len()].copy_from_slice(&data);
                Ok(len)
            }
            RemoteDeviceResponse::ReadData(Err(e)) => Err(e),
            _ => Err(Error::Communication {
                kind: CommunicationKind::Remote(
                    "Unexpected response type for ReadData request".to_string(),
                ),
            }),
        }
    }

    fn wait_for_status(&self) -> Result<u16, Error> {
        match self.send_request(RemoteDeviceRequest::WaitForStatus)? {
            RemoteDeviceResponse::WaitForStatus(result) => result,
            _ => Err(Error::Communication {
                kind: CommunicationKind::Remote(
                    "Unexpected response type for WaitForStatus request".to_string(),
                ),
            }),
        }
    }

    fn ioctl(
        &self,
        ioctl: Ioctl,
        address: u8,
        secondary_address: u8,
    ) -> Result<Option<u16>, Error> {
        match self.send_request(RemoteDeviceRequest::Ioctl {
            ioctl,
            address,
            secondary_address,
        })? {
            RemoteDeviceResponse::Ioctl(result) => result,
            _ => Err(Error::Communication {
                kind: CommunicationKind::Remote(
                    "Unexpected response type for Ioctl request".to_string(),
                ),
            }),
        }
    }
}

// Server implementation
pub struct UsbDeviceServer {
    device: UsbDevice,
    listener: TcpListener,
}

impl UsbDeviceServer {
    pub fn new(device: UsbDevice, addr: &str) -> Result<Self, Error> {
        let listener = TcpListener::bind(addr).map_err(|e| Error::Communication {
            kind: CommunicationKind::Remote(format!("Failed to bind server: {}", e)),
        })?;
        Ok(UsbDeviceServer { device, listener })
    }

    pub fn serve(&mut self) -> Result<(), Error> {
        // Since this is 1:1, we only accept one connection
        let (mut stream, _) = self.listener.accept().map_err(|e| Error::Communication {
            kind: CommunicationKind::Remote(format!("Failed to accept connection: {}", e)),
        })?;

        loop {
            // TO DO - when they disconnect... start again.
            // TO DO - also when they disconnect we need to drop the connection that we made to the XUm1541
            // Read request length
            let mut len_buf = [0u8; 8];
            stream
                .read_exact(&mut len_buf)
                .map_err(|e| Error::Communication {
                    kind: CommunicationKind::Remote(format!(
                        "Failed to read message length: {}",
                        e
                    )),
                })?;
            let len = u64::from_le_bytes(len_buf) as usize;

            // Read request data
            let mut request_data = vec![0u8; len];
            stream
                .read_exact(&mut request_data)
                .map_err(|e| Error::Communication {
                    kind: CommunicationKind::Remote(format!("Failed to read message: {}", e)),
                })?;

            let request: RemoteDeviceRequest =
                deserialize(&request_data).map_err(|e| Error::Communication {
                    kind: CommunicationKind::Remote(format!(
                        "Failed to deserialize request: {}",
                        e
                    )),
                })?;

            let response = match request {
                RemoteDeviceRequest::New(_config) => RemoteDeviceResponse::New(Ok(())),
                RemoteDeviceRequest::Init => RemoteDeviceResponse::Init(self.device.init()),
                RemoteDeviceRequest::CurrentConfig => {
                    let config = self.device.current_config();
                    let mapped_config = match config {
                        Some(config) => Some(RemoteUsbDeviceConfig {
                            serial_num: config.serial_num,
                        }),
                        None => None,
                    };
                    RemoteDeviceResponse::CurrentConfig(mapped_config)
                }
                RemoteDeviceRequest::Info => {
                    RemoteDeviceResponse::Info(self.device.info().cloned())
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
                kind: CommunicationKind::Remote(format!("Failed to serialize response: {}", e)),
            })?;

            stream
                .write_all(&(response_data.len() as u64).to_le_bytes())
                .map_err(|e| Error::Communication {
                    kind: CommunicationKind::Remote(format!(
                        "Failed to send response length: {}",
                        e
                    )),
                })?;
            stream
                .write_all(&response_data)
                .map_err(|e| Error::Communication {
                    kind: CommunicationKind::Remote(format!("Failed to send response: {}", e)),
                })?;
        }
    }
}
