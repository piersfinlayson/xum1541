use crate::Error;
use crate::{Bus, BusBuilder, BUS_DEFAULT_TIMEOUT};
use crate::{Device, SpecificDeviceInfo, UsbDevice, UsbDeviceConfig, UsbInfo};

use rusb::{Context, UsbContext};
use std::time::Duration;

pub type UsbBus = Bus<UsbDevice>;

impl UsbBus {
    pub fn default() -> Result<Self, Error> {
        UsbBusBuilder::new().build()
    }

    pub fn device_specific_info(&self) -> Option<&UsbInfo> {
        self.device.specific_info()
    }
}

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
/// use xum1541::{BusBuilder, UsbBusBuilder};
///
/// let bus = UsbBusBuilder::new()
///     .build()
///     .unwrap();
/// ```
///
/// ## A more complex example
///
/// ```no_run
/// use xum1541::{BusBuilder, UsbBusBuilder, BUS_DEFAULT_TIMEOUT};
/// use std::time::Duration;
///
/// let bus = UsbBusBuilder::new()
///     .device_serial_number(1)
///     .timeout(BUS_DEFAULT_TIMEOUT)
///     .build()
///     .unwrap();
/// ```
pub struct UsbBusBuilder {
    device_config: UsbDeviceConfig,
    timeout: Option<Duration>,
    device: Option<UsbDevice>,
}

impl BusBuilder for UsbBusBuilder {
    type D = UsbDevice;

    /// Creates a new [`BusBuilder`] instance with default values.
    ///
    /// All fields are initialized to None and can be set using the builder methods.
    ///
    /// # Returns
    /// * [`BusBuilder`] - new builder instance with default values
    ///
    /// # Example
    /// See [`BusBuilder`] documentation
    fn default() -> Self {
        UsbBusBuilder {
            device_config: UsbDeviceConfig::default(),
            timeout: None,
            device: None,
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
    /// * Uses default values for any parameters that weren't set:
    ///   * serial_number: 0 (take first device found)
    ///   * timeout: [`BUS_DEFAULT_TIMEOUT`]
    ///   * context: new Context with LogLevel::Info
    /// * Will create a new [`Device`] unless one was explicitly provided
    fn build(&mut self) -> Result<Bus<UsbDevice>, Error> {
        // Create the Device if necessary
        let device = if self.device.is_none() {
            self.create_device()?
        } else {
            self.device.take().unwrap()
        };

        // Create Bus
        let timeout = self.timeout.unwrap_or(BUS_DEFAULT_TIMEOUT);
        Ok(Bus::new(device, timeout))
    }

    fn device_config(&mut self, device_config: UsbDeviceConfig) -> &mut Self {
        self.device_config = device_config;
        self
    }
}

impl UsbBusBuilder {
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
    pub fn device_serial_number(&mut self, serial: u8) -> &mut Self {
        self.device_config.serial_num = Some(serial);
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
    /// use xum1541::{BusBuilder, UsbBusBuilder};
    ///
    /// let mut context = Context::new().unwrap();
    /// context.set_log_level(rusb::LogLevel::Debug);
    ///
    /// let bus = UsbBusBuilder::new()
    ///     .context(context)
    ///     .build()
    ///     .unwrap();
    /// ```
    ///
    /// # Note:
    /// If not set, a new default [`rusb::Context`] will be created with LogLevel::Info
    pub fn context(mut self, context: Context) -> Self {
        self.device_config.context = Some(context);
        self
    }

    /// Create the Device instance using the configured parameters
    fn create_device(&mut self) -> Result<UsbDevice, Error> {
        // Get or create the context
        if self.device_config.context.is_none() {
            let mut context = rusb::Context::new()?;
            context.set_log_level(rusb::LogLevel::Info);
            self.device_config.context = Some(context);
        };

        // Create the Device - this involves consuming the stored context
        let device_config = UsbDeviceConfig {
            context: self.device_config.context.take(),
            serial_num: self.device_config.serial_num,
        };
        UsbDevice::new(Some(device_config))
    }
}
