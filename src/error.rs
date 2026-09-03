use thiserror_no_std::Error;

#[derive(Error, Debug)]
#[allow(dead_code)]
pub(crate) enum MyError {
    Infallible(#[from] core::convert::Infallible),
    SpawnError(#[from] embassy_executor::SpawnError),
    Utf8Error(#[from] core::str::Utf8Error),
    SpiDeviceError(
        #[from] embedded_hal_bus::spi::DeviceError<esp_hal::spi::Error, core::convert::Infallible>,
    ),
    I2CMasterConfigError(#[from] esp_hal::i2c::master::ConfigError),
    I2CMasterError(#[from] esp_hal::i2c::master::Error),
    SpiConfigError(#[from] esp_hal::spi::master::ConfigError),
    WifiError(#[from] esp_radio::wifi::WifiError),
    ReqwlessError(#[from] reqwless::Error),
    ChronoParseError(#[from] chrono::ParseError),
    SerdeJsonError(#[from] serde_json::Error),
    #[error(transparent)]
    Other(#[from] anyhow::Error),
}

pub(crate) type Result<T> = core::result::Result<T, MyError>;
