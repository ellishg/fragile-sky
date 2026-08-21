#![no_std]
#![no_main]

extern crate alloc;

use alloc::format;
use alloc::string::ToString;
use alloc::vec::Vec;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embedded_graphics::{
    draw_target::DrawTarget,
    mono_font::{
        ascii::{FONT_9X15, FONT_9X15_BOLD},
        MonoTextStyleBuilder,
    },
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
use esp_println::println;

mod error;
mod transit_api;
mod wifi;

pub(crate) use error::Result;

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
    next_arrivals: Vec<(&'static str, Vec<u64>)>,
}

async fn init(spawner: Spawner) -> Result<Context> {
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

    let next_arrivals = transit_api::fetch_arrivals(stack, CONFIG.api_key).await?;

    Ok(Context {
        delay,
        spi,
        epd,
        display,
        next_arrivals,
    })
}

fn draw_next_arrivals(ctx: &mut Context) -> Result<()> {
    ctx.display.clear(Color::White)?;

    let name_style = MonoTextStyleBuilder::new()
        .font(&FONT_9X15_BOLD)
        .text_color(Color::Black)
        .build();
    let style = MonoTextStyleBuilder::new()
        .font(&FONT_9X15)
        .text_color(Color::Black)
        .build();

    // This display is 122x250 px
    for (i, (name, next_arrivals)) in ctx.next_arrivals.iter().enumerate() {
        let i = i as i32;
        let (x, y) = (i % 2, i / 2);
        Text::new(name, Point::new(5 + x * 125, 15 + y * 45), name_style).draw(&mut ctx.display)?;

        if !next_arrivals.is_empty() {
            let joined_next_arrivals = next_arrivals
                .iter()
                .take(3)
                .map(|t| t.to_string())
                .collect::<Vec<_>>()
                .join(",");
            Text::new(
                format!("{} mins", joined_next_arrivals).as_str(),
                Point::new(5 + x * 125, 35 + y * 45),
                style,
            )
            .draw(&mut ctx.display)?;
        }
    }
    Ok(())
}

async fn run(spawner: Spawner) -> Result<()> {
    esp_alloc::heap_allocator!(size: 128 * 1024);

    let mut ctx = init(spawner).await?;

    ctx.epd
        .set_refresh(&mut ctx.spi, &mut ctx.delay, RefreshLut::Quick)?;
    for _ in 0..10 {
        // TODO: Move draw frames to spawner tasks
        println!("draw frame");
        draw_next_arrivals(&mut ctx)?;
        ctx.epd
            .update_and_display_frame(&mut ctx.spi, ctx.display.buffer(), &mut ctx.delay)?;

        // Delay for a minute and update the arrival times
        // TODO: Periodically fetch new arrival times
        Timer::after(Duration::from_secs(60)).await;
        for i in 0..ctx.next_arrivals.len() {
            ctx.next_arrivals[i].1 = ctx.next_arrivals[i]
                .1
                .iter()
                .filter_map(|time| time.checked_sub(1))
                .collect();
        }
        ctx.epd
            .set_refresh(&mut ctx.spi, &mut ctx.delay, RefreshLut::Quick)?;
    }

    ctx.display.clear(Color::White)?;
    ctx.epd
        .set_refresh(&mut ctx.spi, &mut ctx.delay, RefreshLut::Full)?;
    ctx.epd
        .update_and_display_frame(&mut ctx.spi, ctx.display.buffer(), &mut ctx.delay)?;
    Ok(())
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    run(spawner).await.unwrap();
    #[allow(clippy::empty_loop)]
    loop {}
}
