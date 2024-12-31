// TODO: We may be able to use esp-idf now
// https://mabez.dev/blog/posts/esp-rust-30-06-2023/
#![no_std]
#![no_main]

extern crate alloc;

use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec;
use alloc::vec::Vec;
use anyhow::anyhow;
use chrono::NaiveDateTime;
use core::ptr::addr_of;
use embedded_graphics::{
    draw_target::DrawTarget,
    mono_font::{
        ascii::{FONT_9X15, FONT_9X15_BOLD},
        MonoTextStyleBuilder,
    },
    prelude::*,
    text::Text,
};
use embedded_svc::io::{Read, Write};
use epd_waveshare::{
    color::*,
    epd2in13_v2::{Display2in13, Epd2in13},
    prelude::*,
};
use esp_hal::clock::ClockControl;
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    gpio,
    peripherals::{Peripherals, SPI2},
    prelude::*,
    rng,
    spi::{master::Spi, FullDuplexMode, SpiMode},
    system::SystemControl,
    timer,
};
use esp_println::println;
use esp_wifi::wifi::{
    utils::create_network_interface, ClientConfiguration, Configuration, WifiStaDevice,
};
use esp_wifi::wifi_interface::WifiStack;
use esp_wifi::{current_millis, EspWifiInitFor};
use smoltcp::iface::SocketStorage;
use smoltcp::wire::{DnsQueryType, IpAddress, Ipv4Address};
use thiserror_no_std::Error;

#[derive(Error, Debug)]
#[allow(dead_code)]
enum MyError {
    Infallible(#[from] core::convert::Infallible),
    Utf8Error(#[from] core::str::Utf8Error),
    SpiDeviceError(
        #[from] embedded_hal_bus::spi::DeviceError<esp_hal::spi::Error, core::convert::Infallible>,
    ),
    EspWifiInitializationError(#[from] esp_wifi::InitializationError),
    EspWifiIoError(#[from] esp_wifi::wifi_interface::IoError),
    EspWifiWifiError(#[from] esp_wifi::wifi::WifiError),
    EspWifiWifiStackError(#[from] esp_wifi::wifi_interface::WifiStackError),
    ChronoParseError(#[from] chrono::ParseError),
    SerdeJsonError(#[from] serde_json::Error),
    #[error(transparent)]
    Other(#[from] anyhow::Error),
}
type Result<T> = core::result::Result<T, MyError>;

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

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();
// TODO: It seems that the symbol "_heap_start" is defined. Maybe that can help us.
const HEAP_SIZE: usize = 128 * 1024;
static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

type SpiT =
    embedded_hal_bus::spi::ExclusiveDevice<Spi<'static, SPI2, FullDuplexMode>, CSPin, Delay>;
type CSPin = gpio::Output<'static, gpio::GpioPin<5>>;
type BusyPin = gpio::Input<'static, gpio::GpioPin<6>>;
type DCPin = gpio::Output<'static, gpio::GpioPin<23>>;
type RSTPin = gpio::Output<'static, gpio::GpioPin<22>>;
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

fn request_stop_code_info(
    socket: &mut esp_wifi::wifi_interface::Socket<'_, '_, WifiStaDevice>,
    api_ip_address: IpAddress,
    stop_code: u16,
) -> Result<String> {
    // curl -X GET -v -I "http://api.511.org/transit/StopMonitoring?api_key=<API_KEY>&agency=SF&format=json&stopcode=<STOP_CODE>"
    println!("Making HTTP request");
    socket.work();

    socket.open(api_ip_address, 80)?;

    let get_request = format!(
        concat!(
            "GET /transit/StopMonitoring?api_key={}&agency=SF&format=json&stopcode={} HTTP/1.0\r\n",
            "Host: api.511.org\r\n",
            "Accept: application/json\r\n",
            "Accept-Encoding: identity\r\n\r\n",
        ),
        CONFIG.api_key, stop_code
    );
    socket.write(get_request.as_bytes())?;
    socket.flush()?;

    let content = read_content(socket)?;

    socket.disconnect();
    socket.work();

    Ok(content)
}

fn read_content(
    socket: &mut esp_wifi::wifi_interface::Socket<'_, '_, WifiStaDevice>,
) -> Result<String> {
    let mut buffer: Vec<u8> = vec![];
    loop {
        let mut chunk = [0u8; 1024];
        if let Ok(bytes_read) = socket.read(&mut chunk) {
            buffer.extend_from_slice(&chunk[..bytes_read]);
        } else if !buffer.is_empty() {
            let mut headers = [httparse::EMPTY_HEADER; 64];
            let mut response = httparse::Response::new(&mut headers);
            if let Ok(httparse::Status::Complete(content_start)) = response.parse(&buffer) {
                let content = &buffer[content_start..];
                // TODO: I'm not sure if this is needed to guarantee the content is complete
                // for header in response.headers.iter() {
                //     if header.name == "Content-Length" {
                //         let value = core::str::from_utf8(header.value)?;
                //         let expected_content_length: usize = value.parse()?;
                //         if content.len() >= expected_content_length {
                //             return core::str::from_utf8(content)?.to_string();
                //         }
                //     }
                // }
                return Ok(core::str::from_utf8(content)?
                    // TODO: I'm not sure why, but the content is prefixed with "ï»¿"
                    .replace(|c: char| !c.is_ascii(), "")
                    .to_string());
            }
        }
    }
}

fn init() -> Result<Context> {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::max(system.clock_control).freeze();

    let timer = timer::systimer::SystemTimer::new(peripherals.SYSTIMER).alarm0;
    let init = esp_wifi::initialize(
        EspWifiInitFor::Wifi,
        timer,
        rng::Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
        &clocks,
    )?;

    let io = gpio::Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let clk = io.pins.gpio0;
    let din = io.pins.gpio4;
    let cs = gpio::Output::new(io.pins.gpio5, gpio::Level::High);
    let busy = gpio::Input::new(io.pins.gpio6, gpio::Pull::None);
    let dc = gpio::Output::new(io.pins.gpio23, gpio::Level::High);
    let rst = gpio::Output::new(io.pins.gpio22, gpio::Level::High);

    let spi = Spi::new(peripherals.SPI2, 4u32.MHz(), SpiMode::Mode0, &clocks).with_pins(
        Some(clk),
        Some(din),
        gpio::NO_PIN,
        gpio::NO_PIN,
    );
    let mut delay = Delay::new(&clocks);

    let mut spi = embedded_hal_bus::spi::ExclusiveDevice::new(spi, cs, delay)?;

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

    let wifi = peripherals.WIFI;
    let mut socket_set_entries: [SocketStorage; 3] = Default::default();
    let (iface, device, mut controller, sockets) =
        create_network_interface(&init, wifi, WifiStaDevice, &mut socket_set_entries)?;
    let wifi_stack = WifiStack::new(iface, device, sockets, current_millis);
    let mut query_storage: [_; 1] = Default::default();
    wifi_stack.configure_dns(&[Ipv4Address::new(8, 8, 8, 8).into()], &mut query_storage);

    let client_config = Configuration::Client(ClientConfiguration {
        ssid: CONFIG.wifi_ssid.try_into().unwrap(),
        password: CONFIG.wifi_password.try_into().unwrap(),
        ..Default::default()
    });

    let res = controller.set_configuration(&client_config);
    println!("wifi_set_configuration returned {:?}", res);

    controller.start()?;
    assert!(controller.is_started()?);

    println!("{:?}", controller.get_capabilities());
    controller.connect()?;

    // wait to get connected
    println!("Wait to get connected");
    loop {
        let res = controller.is_connected();
        match res {
            Ok(connected) => {
                if connected {
                    break;
                }
            }
            Err(err) => {
                // TODO: This sometimes fails, retry
                panic!("Did not connect: {:?}", err);
            }
        }
    }
    assert!(controller.is_connected()?);

    // wait for getting an ip address
    println!("Wait to get an ip address");
    loop {
        wifi_stack.work();

        if wifi_stack.is_iface_up() {
            println!("{:?}", wifi_stack.get_ip_info()?);
            break;
        }
    }

    // TODO: Add wifi_stack to Context
    let mut rx_buffer = [0u8; 1536];
    let mut tx_buffer = [0u8; 1536];
    let mut socket = wifi_stack.get_socket(&mut rx_buffer, &mut tx_buffer);

    let api_ip_address = wifi_stack.dns_query("api.511.org", DnsQueryType::A)?[0];
    println!("Found ip address for api.511.org: {}", api_ip_address);

    let next_arrivals = STOP_CONFIGS
        .iter()
        .map(|stop_config| {
            let response =
                request_stop_code_info(&mut socket, api_ip_address, stop_config.stop_code)?;
            let next_arrivals = get_minutes_until_next_arrivals(&response)?;
            Ok((stop_config.name, next_arrivals))
        })
        .collect::<Result<_>>()?;

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

fn run() -> Result<()> {
    // Initialize the heap
    unsafe {
        let heap_start = addr_of!(HEAP) as *const _ as usize;
        ALLOCATOR.init(heap_start as *mut u8, HEAP_SIZE);
    }

    let mut ctx = init()?;

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

#[entry]
fn main() -> ! {
    run().unwrap();
    #[allow(clippy::empty_loop)]
    loop {}
}
