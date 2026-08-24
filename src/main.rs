#![no_std]
#![no_main]

extern crate alloc;

use alloc::vec::Vec;
use embassy_executor::Spawner;
use embassy_net::tcp::client::TcpClientState;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embedded_graphics::{
    draw_target::DrawTarget,
    mono_font::{ascii::FONT_9X15, MonoTextStyleBuilder},
    prelude::*,
    text::Text,
};
use epd_waveshare::{
    color::*,
    epd2in13_v2::{Display2in13, Epd2in13},
    prelude::*,
};
use esp_backtrace as _;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{
    delay::Delay,
    gpio,
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    spi,
    spi::{master::Spi, Mode},
    time::Rate,
    Blocking,
};
mod display;
mod error;
mod transit_api;
mod wifi;

pub(crate) use error::Result;

static DISPLAY_CHANNEL: Channel<
    CriticalSectionRawMutex,
    // TODO: New struct
    Vec<(&'static str, Vec<u64>)>,
    2,
> = Channel::new();

#[derive(Debug)]
#[toml_cfg::toml_config]
pub struct Config {
    #[default("")]
    wifi_ssid: &'static str,
    #[default("")]
    wifi_password: &'static str,
    #[default("")]
    api_key: &'static str,
}

// TODO: Move to display.rs
type SpiT = embedded_hal_bus::spi::ExclusiveDevice<
    Spi<'static, Blocking>,
    gpio::Output<'static>,
    embedded_hal_bus::spi::NoDelay,
>;
type Epd =
    Epd2in13<SpiT, gpio::Input<'static>, gpio::Output<'static>, gpio::Output<'static>, Delay>;
struct Context {
    delay: Delay,
    spi: SpiT,
    epd: Epd,
    display: Display2in13,
}

async fn init(spawner: Spawner) -> Result<(Context, embassy_net::Stack<'static>)> {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    let clk = peripherals.GPIO0;
    let din = peripherals.GPIO4;
    let cs = Output::new(peripherals.GPIO5, Level::High, OutputConfig::default());
    let busy = Input::new(
        peripherals.GPIO6,
        InputConfig::default().with_pull(Pull::None),
    );
    let dc = Output::new(peripherals.GPIO23, Level::High, OutputConfig::default());
    let rst = Output::new(peripherals.GPIO22, Level::High, OutputConfig::default());

    let spi = Spi::new(
        peripherals.SPI2,
        spi::master::Config::default()
            .with_frequency(Rate::from_khz(100))
            .with_mode(Mode::_0),
    )?
    .with_sck(clk)
    .with_mosi(din);

    let mut delay = Delay::new();

    let mut spi = embedded_hal_bus::spi::ExclusiveDevice::new_no_delay(spi, cs)?;

    let mut epd: Epd = Epd2in13::new(&mut spi, busy, dc, rst, &mut delay, None)?;
    epd.set_refresh(&mut spi, &mut delay, RefreshLut::Full)?;

    let mut display = Display2in13::default();
    display.clear(Color::White)?;
    display.set_rotation(DisplayRotation::Rotate90);

    let style = MonoTextStyleBuilder::new()
        .font(&FONT_9X15)
        .text_color(Color::Black)
        .build();
    Text::new("Connecting...", Point::new(5, 15), style).draw(&mut display)?;
    epd.update_and_display_frame(&mut spi, display.buffer(), &mut delay)?;

    let stack = wifi::connect(
        spawner,
        peripherals.WIFI,
        CONFIG.wifi_ssid,
        CONFIG.wifi_password,
    )
    .await?;

    Ok((
        Context {
            delay,
            spi,
            epd,
            display,
        },
        stack,
    ))
}

async fn run(spawner: Spawner) -> Result<()> {
    esp_alloc::heap_allocator!(size: 128 * 1024);

    let (mut ctx, stack) = init(spawner).await?;
    // TODO: Move to init
    let tcp_client_state = transit_api::TCP_CLIENT_STATE.init(TcpClientState::new());

    ctx.epd
        .set_refresh(&mut ctx.spi, &mut ctx.delay, RefreshLut::Quick)?;

    spawner.spawn(display::display_task(ctx, DISPLAY_CHANNEL.receiver())?);
    spawner.spawn(transit_api::update_arrivals_task(
        stack,
        CONFIG.api_key,
        tcp_client_state,
        DISPLAY_CHANNEL.sender(),
    )?);

    Ok(())
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    run(spawner).await.unwrap();
    core::future::pending::<()>().await;
    unreachable!();
}
