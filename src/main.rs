#![no_std]
#![no_main]

extern crate alloc;

use alloc::format;
use alloc::string::ToString;
use alloc::vec::Vec;
use anyhow::anyhow;
use chrono::NaiveDateTime;
use embassy_executor::Spawner;
use embassy_net::{
    dns::DnsSocket,
    tcp::client::{TcpClient, TcpClientState},
};
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
use reqwless::{
    client::HttpClient,
    request::{Method, RequestBuilder},
};
use thiserror_no_std::Error;

mod wifi;

#[derive(Error, Debug)]
#[allow(dead_code)]
enum MyError {
    Infallible(#[from] core::convert::Infallible),
    Utf8Error(#[from] core::str::Utf8Error),
    SpiDeviceError(
        #[from] embedded_hal_bus::spi::DeviceError<esp_hal::spi::Error, core::convert::Infallible>,
    ),
    SpiConfigError(#[from] esp_hal::spi::master::ConfigError),
    WifiError(#[from] esp_radio::wifi::WifiError),
    ReqwlessError(#[from] reqwless::Error),
    ChronoParseError(#[from] chrono::ParseError),
    SerdeJsonError(#[from] serde_json::Error),
    #[error(transparent)]
    Other(#[from] anyhow::Error),
}
type Result<T> = core::result::Result<T, MyError>;

static TCP_CLIENT_STATE: static_cell::StaticCell<TcpClientState<1, 1500, 1500>> =
    static_cell::StaticCell::new();

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
type BusyPin = gpio::Input<'static>;
type DCPin = gpio::Output<'static>;
type RSTPin = gpio::Output<'static>;
type Epd = Epd2in13<SpiT, BusyPin, DCPin, RSTPin, Delay>;
struct Context {
    delay: Delay,
    spi: SpiT,
    epd: Epd,
    display: Display2in13,
    next_arrivals: Vec<(&'static str, Vec<u64>)>,
}

// TODO: Move to cfg.toml
struct StopConfig {
    stop_code: u16,
    name: &'static str,
}

static STOP_CONFIGS: [StopConfig; 4] = [
    StopConfig {
        stop_code: 13911,
        name: "N Eastbound",
    },
    StopConfig {
        stop_code: 13909,
        name: "N Westbound",
    },
    StopConfig {
        stop_code: 14946,
        name: "Hght&Clytn W",
    },
    StopConfig {
        stop_code: 14947,
        name: "Hght&Clytn E",
    },
];

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

    // Init HTTP client
    let tcp_client = TcpClient::new(
        stack,
        TCP_CLIENT_STATE.uninit().write(TcpClientState::new()),
    );
    let dns_client = DnsSocket::new(stack);

    let mut client = HttpClient::new(&tcp_client, &dns_client);
    let mut rx_buf = [0u8; 16 * 1024];

    let mut next_arrivals: Vec<(&'static str, Vec<u64>)> = Vec::new();
    for stop_config in STOP_CONFIGS.iter() {
        let url = format!("http://api.511.org/transit/StopMonitoring?api_key={}&agency=SF&format=json&stopcode={}", CONFIG.api_key, stop_config.stop_code);
        let builder = client.request(Method::GET, &url).await?;
        let mut builder = builder.headers(&[
            ("Host", "api.511.org"),
            ("Accept", "application/json"),
            ("Accept-Encoding", "identity"),
            ("Connection", "close"),
        ]);
        let response = builder.send(&mut rx_buf).await?;
        match response.body().read_to_end().await {
            Ok(data) => {
                let body = core::str::from_utf8(data)?;
                next_arrivals.push((stop_config.name, get_minutes_until_next_arrivals(body)?));
            }
            Err(e) => println!("Body error: {:?}", e),
        }
        println!("done");
    }

    // let next_arrivals = vec![
    //     ("N Eastbound", vec![5, 15, 25]),
    //     ("N Westbound", vec![3, 13, 23]),
    //     ("Hght&Clytn W", vec![7, 17, 27]),
    //     ("Hght&Clytn E", vec![2, 12, 22]),
    // ];

    Ok(Context {
        delay,
        spi,
        epd,
        display,
        next_arrivals,
    })
}

fn get_minutes_until_next_arrivals(content: &str) -> Result<Vec<u64>> {
    let v: serde_json::Value = serde_json::from_str(content)?;

    // TODO: Need to lookup and track walltime
    let now_str = v["ServiceDelivery"]["ResponseTimestamp"]
        .as_str()
        .ok_or(anyhow!("ResponseTimestamp not found in response"))?;
    let now = NaiveDateTime::parse_from_str(now_str, "%+")?;

    v["ServiceDelivery"]["StopMonitoringDelivery"]["MonitoredStopVisit"]
        .as_array()
        .ok_or(anyhow!("MonitoredStopVisit expected to be an array"))?
        .iter()
        .map(|v| {
            let arrival_time = v["MonitoredVehicleJourney"]["MonitoredCall"]["ExpectedArrivalTime"]
                .as_str()
                .ok_or(anyhow!("ExpectedArrivalTime expected to be a string"))?;
            let parsed_arrival_time = NaiveDateTime::parse_from_str(arrival_time, "%+")?
                .signed_duration_since(now)
                .num_minutes() as u64;
            Ok(parsed_arrival_time)
        })
        .collect()
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
        println!("draw frame");
        draw_next_arrivals(&mut ctx)?;
        ctx.epd
            .update_and_display_frame(&mut ctx.spi, ctx.display.buffer(), &mut ctx.delay)?;

        // Delay for a minute and update the arrival times
        // TODO: Periodically fetch new arrival times
        ctx.delay.delay_millis(60_000u32);
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
