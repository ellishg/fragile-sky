// TODO: We may be able to use esp-idf now
// https://mabez.dev/blog/posts/esp-rust-30-06-2023/
#![no_std]
#![no_main]

extern crate alloc;

use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec;
use alloc::vec::Vec;
use chrono::NaiveDateTime;
use embedded_graphics::{
    mono_font::{
        ascii::{FONT_9X15, FONT_9X15_BOLD},
        MonoTextStyle,
    },
    prelude::*,
    text::Text,
};
use embedded_io::blocking::*;
use embedded_svc::ipv4::Interface;
use embedded_svc::wifi::{ClientConfiguration, Configuration, Wifi};
use epd_waveshare::{
    epd2in13b_v4::{Display2in13b, Epd2in13b},
    prelude::*,
};
use esp_backtrace as _;
use esp_hal_smartled::SmartLedsAdapter;
use esp_println::println;
use esp_wifi::wifi::{utils::create_network_interface, WifiMode};
use esp_wifi::wifi_interface::WifiStack;
use esp_wifi::{current_millis, EspWifiInitFor};
use hal::{
    clock, gpio,
    peripherals::{Peripherals, SPI2},
    prelude::*,
    pulse_control, spi,
    systimer::SystemTimer,
    timer, Delay, PulseControl, Rtc, IO,
};
use smart_leds::{
    hsv::{hsv2rgb, Hsv},
    SmartLedsWrite,
};
use smoltcp::iface::SocketStorage;
use smoltcp::wire::IpAddress;
use smoltcp::wire::Ipv4Address;

// TODO: use env!()
const SSID: Option<&str> = option_env!("SSID");
const PASSWORD: Option<&str> = option_env!("PASSWORD");
const API_KEY: Option<&str> = option_env!("API_KEY");

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();
// TODO: It seems that the symbol "_heap_start" is defined. Maybe that can help us.
const HEAP_SIZE: usize = 1024 * 64;
static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

type Spi<'a> = spi::Spi<'a, SPI2, spi::FullDuplexMode>;
type CSPin = gpio::GpioPin<gpio::Output<gpio::PushPull>, 5>;
type BusyPin = gpio::GpioPin<hal::gpio::Input<hal::gpio::Floating>, 4>;
type DCPin = gpio::GpioPin<gpio::Output<gpio::PushPull>, 17>;
type RSTPin = gpio::GpioPin<gpio::Output<gpio::PushPull>, 16>;
type Epd<'a> = Epd2in13b<Spi<'a>, CSPin, BusyPin, DCPin, RSTPin, Delay>;
type EPaperDisplay = Display<122, 250, false, 8000, TriColor>;
type LEDPin = gpio::GpioPin<hal::gpio::Unknown, 8>;
type Led<'a> = SmartLedsAdapter<pulse_control::ConfiguredChannel0<'a, LEDPin>, 25>;

struct Context<'a> {
    delay: Delay,
    spi: Spi<'a>,
    epd: Epd<'a>,
    display: EPaperDisplay,
    led: Led<'a>,
    next_arrivals: Vec<(&'static str, Vec<i64>)>,
}

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
    socket: &mut esp_wifi::wifi_interface::Socket<'_, '_>,
    stop_code: u16,
) -> String {
    // curl -X GET -v -I "http://api.511.org/transit/StopMonitoring?api_key=<API_KEY>&agency=SF&format=json&stopcode=<STOP_CODE>"
    println!("Making HTTP request");
    socket.work();

    // TODO: DNS lookup
    socket
        .open(IpAddress::Ipv4(Ipv4Address::new(52, 8, 155, 117)), 80)
        .unwrap();

    let get_request = format!(
        concat!(
            "GET /transit/StopMonitoring?api_key={}&agency=SF&format=json&stopcode={} HTTP/1.0\r\n",
            "Host: api.511.org\r\n",
            "Accept: application/json\r\n",
            "Accept-Encoding: identity\r\n\r\n",
        ),
        API_KEY.unwrap(),
        stop_code
    );
    socket.write(get_request.as_bytes()).unwrap();
    socket.flush().unwrap();

    let content = read_content(socket);

    socket.disconnect();
    socket.work();

    content
}

fn read_content(socket: &mut esp_wifi::wifi_interface::Socket<'_, '_>) -> String {
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
                //         let value = core::str::from_utf8(header.value).unwrap();
                //         let expected_content_length: usize = value.parse().unwrap();
                //         if content.len() >= expected_content_length {
                //             return core::str::from_utf8(content).unwrap().to_string();
                //         }
                //     }
                // }
                return core::str::from_utf8(content)
                    .unwrap()
                    // TODO: I'm not sure why, but the content is prefixed with "ï»¿"
                    .replace(|c: char| !c.is_ascii(), "")
                    .to_string();
            }
        }
    }
}

// TODO: Return result type
fn init<'a>() -> Context<'a> {
    let peripherals = Peripherals::take();
    let mut system = peripherals.PCR.split();
    let clocks =
        clock::ClockControl::configure(system.clock_control, clock::CpuClock::Clock160MHz).freeze();

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.LP_CLKRST);
    let timer_group0 = timer::TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = timer::TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt1 = timer_group1.wdt;
    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    let timer = SystemTimer::new(peripherals.SYSTIMER).alarm0;
    let init = esp_wifi::initialize(
        EspWifiInitFor::Wifi,
        timer,
        hal::Rng::new(peripherals.RNG),
        system.radio_clock_control,
        &clocks,
    )
    .unwrap();

    let (wifi, _, _) = peripherals.RADIO.split();
    let mut socket_set_entries: [SocketStorage; 3] = Default::default();
    let (iface, device, mut controller, sockets) =
        create_network_interface(&init, wifi, WifiMode::Sta, &mut socket_set_entries);
    let wifi_stack = WifiStack::new(iface, device, sockets, current_millis);

    let client_config = Configuration::Client(ClientConfiguration {
        ssid: SSID.unwrap().into(),
        password: PASSWORD.unwrap().into(),
        ..Default::default()
    });

    let res = controller.set_configuration(&client_config);
    println!("wifi_set_configuration returned {:?}", res);

    controller.start().unwrap();
    println!("is wifi started: {:?}", controller.is_started());

    println!("{:?}", controller.get_capabilities());
    println!("wifi_connect {:?}", controller.connect());

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
    println!("connected: {:?}", controller.is_connected());

    // wait for getting an ip address
    println!("Wait to get an ip address");
    loop {
        wifi_stack.work();

        if wifi_stack.is_iface_up() {
            println!("got ip {:?}", wifi_stack.get_ip_info());
            break;
        }
    }

    // TODO: Add wifi_stack to Context
    let mut rx_buffer = [0u8; 1536];
    let mut tx_buffer = [0u8; 1536];
    let mut socket = wifi_stack.get_socket(&mut rx_buffer, &mut tx_buffer);

    let next_arrivals = STOP_CONFIGS
        .iter()
        .map(|stop_config| {
            let response = request_stop_code_info(&mut socket, stop_config.stop_code);
            let next_arrivals = get_minutes_until_next_arrivals(&response);
            (stop_config.name, next_arrivals)
        })
        .collect();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut delay = Delay::new(&clocks);

    let clk = io.pins.gpio18;
    let din = io.pins.gpio23;

    // TODO: I believe printing breaks after this
    let mut spi = spi::Spi::new_no_cs_no_miso(
        peripherals.SPI2,
        clk,
        din,
        4u32.MHz(),
        spi::SpiMode::Mode0,
        &mut system.peripheral_clock_control,
        &clocks,
    );

    let cs = io.pins.gpio5.into_push_pull_output();
    let busy = io.pins.gpio4.into_floating_input();
    let dc = io.pins.gpio17.into_push_pull_output();
    let rst = io.pins.gpio16.into_push_pull_output();

    let epd = Epd2in13b::new(&mut spi, cs, busy, dc, rst, &mut delay, None).unwrap();

    let mut display = Display2in13b::default();
    display.set_rotation(DisplayRotation::Rotate90);

    // Configure RMT peripheral globally
    let pulse = PulseControl::new(
        peripherals.RMT,
        &mut system.peripheral_clock_control,
        pulse_control::ClockSource::APB,
        0,
        0,
        0,
    )
    .unwrap();

    // We use one of the RMT channels to instantiate a `SmartLedsAdapter` which can
    // be used directly with all `smart_led` implementations
    // FIXME: We could use <smartLedAdapter!(1)>::new(...) but this confuses rust-analyzer
    let led = SmartLedsAdapter::<_, 25>::new(pulse.channel0, io.pins.gpio8);

    Context {
        delay,
        spi,
        epd,
        display,
        led,
        next_arrivals,
    }
}

fn get_minutes_until_next_arrivals(content: &str) -> Vec<i64> {
    let v: serde_json::Value = serde_json::from_str(content).unwrap();
    let expected_arrival_times = v["ServiceDelivery"]["StopMonitoringDelivery"]
        ["MonitoredStopVisit"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| {
            v["MonitoredVehicleJourney"]["MonitoredCall"]["ExpectedArrivalTime"]
                .as_str()
                .unwrap()
        });

    // TODO: Need to lookup and track walltime
    let now_str = v["ServiceDelivery"]["ResponseTimestamp"].as_str().unwrap();
    let now = NaiveDateTime::parse_from_str(now_str, "%+").unwrap();

    expected_arrival_times
        .map(|t| {
            NaiveDateTime::parse_from_str(t, "%+")
                .unwrap()
                .signed_duration_since(now)
                .num_minutes()
        })
        .collect()
}

#[entry]
fn main() -> ! {
    // Initialize the heap
    unsafe {
        let heap_start = &HEAP as *const _ as usize;
        ALLOCATOR.init(heap_start as *mut u8, HEAP_SIZE);
    }

    let mut ctx = init();

    ctx.display.clear(TriColor::White).unwrap();

    let name_style = MonoTextStyle::new(&FONT_9X15_BOLD, TriColor::Black);
    let style = MonoTextStyle::new(&FONT_9X15, TriColor::Chromatic);

    // This display is 122x250 px
    for (i, (name, next_arrivals)) in ctx.next_arrivals.iter().enumerate() {
        let i = i as i32;
        let (x, y) = (i % 2, i / 2);
        Text::new(name, Point::new(5 + x * 125, 15 + y * 45), name_style)
            .draw(&mut ctx.display)
            .unwrap();

        // TODO: We should join bus names with times
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
        .draw(&mut ctx.display)
        .unwrap();
    }

    // TODO: Improve the speed of updating frames
    ctx.epd
        .update_color_frame(
            &mut ctx.spi,
            &mut ctx.delay,
            ctx.display.bw_buffer(),
            ctx.display.chromatic_buffer(),
        )
        .unwrap();
    ctx.epd.display_frame(&mut ctx.spi, &mut ctx.delay).unwrap();

    // TODO: Look into embassy_executor
    for hue in 0..=255 {
        let data = [hsv2rgb(Hsv {
            hue,
            sat: 255,
            val: 255,
        })];
        // When sending to the LED, we do a gamma correction first (see smart_leds
        // documentation for details) and then limit the brightness to 10 out of 255 so
        // that the output it's not too bright.
        ctx.led
            .write(smart_leds::brightness(
                smart_leds::gamma(data.iter().cloned()),
                5,
            ))
            .unwrap();
        ctx.delay.delay_ms(80u8);
    }

    ctx.display.clear(TriColor::White).unwrap();
    ctx.epd
        .update_color_frame(
            &mut ctx.spi,
            &mut ctx.delay,
            ctx.display.bw_buffer(),
            ctx.display.chromatic_buffer(),
        )
        .unwrap();
    ctx.epd.display_frame(&mut ctx.spi, &mut ctx.delay).unwrap();

    loop {
        ctx.delay.delay_ms(0xFFFFFFFFu32);
    }
}
