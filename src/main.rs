#![no_std]
#![no_main]

extern crate alloc;

use alloc::alloc::{GlobalAlloc, Layout};
use alloc::format;
use chrono::NaiveDateTime;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X13, MonoTextStyle},
    prelude::*,
    text::Text,
};
use epd_waveshare::{
    epd2in13b_v4::{Display2in13b, Epd2in13b},
    prelude::*,
};
use esp_backtrace as _;
use esp_hal_smartled::SmartLedsAdapter;
// use esp_println::println;
use hal::{
    clock, gpio,
    peripherals::{Peripherals, SPI2},
    prelude::*,
    pulse_control, spi, timer, Delay, PulseControl, Rtc, IO,
};
use serde_json;
use smart_leds::{
    hsv::{hsv2rgb, Hsv},
    SmartLedsWrite,
};
use spin::Mutex;

// Implement a very bad bump allocator so we allocate memory
struct BadAllocator {
    next: Mutex<usize>,
}
impl BadAllocator {
    const fn new() -> Self {
        BadAllocator {
            next: Mutex::new(0),
        }
    }
}
#[global_allocator]
static ALLOCATOR: BadAllocator = BadAllocator::new();

const BUFFER_SIZE: usize = 1024 * 64;
static mut BUFFER: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE];
unsafe impl GlobalAlloc for BadAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let mut next = self.next.lock();
        let start = *next;
        assert!(
            start + layout.size() < BUFFER.len(),
            "Overflow at {} bytes",
            start + layout.size()
        );
        *next += layout.size();
        BUFFER.as_mut_ptr().add(start)
    }
    // No we don't ever release memory :)
    unsafe fn dealloc(&self, _ptr: *mut u8, _layout: Layout) {}
}

type Spi<'a> = spi::Spi<'a, SPI2, spi::FullDuplexMode>;
type CSPin = gpio::GpioPin<
    gpio::Output<gpio::PushPull>,
    gpio::Bank0GpioRegisterAccess,
    gpio::SingleCoreInteruptStatusRegisterAccessBank0,
    gpio::InputOutputAnalogPinType,
    gpio::Gpio5Signals,
    5,
>;
type BusyPin = gpio::GpioPin<
    hal::gpio::Input<hal::gpio::Floating>,
    gpio::Bank0GpioRegisterAccess,
    gpio::SingleCoreInteruptStatusRegisterAccessBank0,
    gpio::InputOutputAnalogPinType,
    gpio::Gpio4Signals,
    4,
>;
type DCPin = gpio::GpioPin<
    gpio::Output<gpio::PushPull>,
    gpio::Bank0GpioRegisterAccess,
    gpio::SingleCoreInteruptStatusRegisterAccessBank0,
    gpio::InputOutputPinType,
    gpio::Gpio17Signals,
    17,
>;
type RSTPin = gpio::GpioPin<
    gpio::Output<gpio::PushPull>,
    gpio::Bank0GpioRegisterAccess,
    gpio::SingleCoreInteruptStatusRegisterAccessBank0,
    gpio::InputOutputPinType,
    gpio::Gpio16Signals,
    16,
>;
type Epd<'a> = Epd2in13b<Spi<'a>, CSPin, BusyPin, DCPin, RSTPin, Delay>;
type EPaperDisplay = Display<122, 250, false, 8000, TriColor>;
type LEDPin = gpio::GpioPin<
    hal::gpio::Unknown,
    gpio::Bank0GpioRegisterAccess,
    gpio::SingleCoreInteruptStatusRegisterAccessBank0,
    gpio::InputOutputPinType,
    gpio::Gpio8Signals,
    8,
>;
type Led<'a> = SmartLedsAdapter<pulse_control::ConfiguredChannel0<'a, LEDPin>, 25>;

struct Context<'a> {
    delay: Delay,
    spi: Spi<'a>,
    epd: Epd<'a>,
    display: EPaperDisplay,
    led: Led<'a>,
}

// TODO: Return result type
fn init<'a>() -> Context<'a> {
    let peripherals = Peripherals::take();
    let mut system = peripherals.PCR.split();
    let clocks = clock::ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.LP_CLKRST);
    let timer_group0 = timer::TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = timer::TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;
    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut delay = Delay::new(&clocks);

    let clk = io.pins.gpio18;
    let din = io.pins.gpio23;

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

    let display = Display2in13b::default();

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
    }
}

fn get_minutes_until_next_arrival() -> i64 {
    // https://api.511.org/transit/StopMonitoring?api_key=<API_KEY>&agency=SF&format=json&stopcode=13911
    // https://www.sfmta.com/stops/carl-st-cole-st-13911
    // Inline an example response until I get WiFi working
    let raw_str = r###"
{
  "ServiceDelivery": {
    "ResponseTimestamp": "2023-05-28T16:46:32Z",
    "ProducerRef": "SF",
    "Status": true,
    "StopMonitoringDelivery": {
      "version": "1.4",
      "ResponseTimestamp": "2023-05-28T16:46:32Z",
      "Status": true,
      "MonitoredStopVisit": [
        {
          "RecordedAtTime": "2023-05-28T16:46:26Z",
          "MonitoringRef": "13911",
          "MonitoredVehicleJourney": {
            "LineRef": "N",
            "DirectionRef": "IB",
            "FramedVehicleJourneyRef": {
              "DataFrameRef": "2023-05-28",
              "DatedVehicleJourneyRef": "11300630"
            },
            "PublishedLineName": "JUDAH",
            "OperatorRef": "SF",
            "OriginRef": "15223",
            "OriginName": "Judah/La Playa/Ocean Beach",
            "DestinationRef": "15239",
            "DestinationName": "Caltrain/Ballpark",
            "Monitored": true,
            "InCongestion": null,
            "VehicleLocation": {
              "Longitude": "-122.466629",
              "Latitude": "37.762146"
            },
            "Bearing": "75.0000000000",
            "Occupancy": "seatsAvailable",
            "VehicleRef": "2014",
            "MonitoredCall": {
              "StopPointRef": "13911",
              "StopPointName": "Carl St & Cole St",
              "VehicleLocationAtStop": "",
              "VehicleAtStop": "false",
              "DestinationDisplay": "Caltrain/Ballpark",
              "AimedArrivalTime": "2023-05-28T16:52:06Z",
              "ExpectedArrivalTime": "2023-05-28T16:54:32Z",
              "AimedDepartureTime": "2023-05-28T16:52:06Z",
              "ExpectedDepartureTime": null,
              "Distances": ""
            }
          }
        },
        {
          "RecordedAtTime": "2023-05-28T16:46:26Z",
          "MonitoringRef": "13911",
          "MonitoredVehicleJourney": {
            "LineRef": "N",
            "DirectionRef": "IB",
            "FramedVehicleJourneyRef": {
              "DataFrameRef": "2023-05-28",
              "DatedVehicleJourneyRef": "11300631"
            },
            "PublishedLineName": "JUDAH",
            "OperatorRef": "SF",
            "OriginRef": "15223",
            "OriginName": "Judah/La Playa/Ocean Beach",
            "DestinationRef": "15239",
            "DestinationName": "Caltrain/Ballpark",
            "Monitored": true,
            "InCongestion": null,
            "VehicleLocation": {
              "Longitude": "-122.496506",
              "Latitude": "37.7608376"
            },
            "Bearing": "75.0000000000",
            "Occupancy": "seatsAvailable",
            "VehicleRef": "2043",
            "MonitoredCall": {
              "StopPointRef": "13911",
              "StopPointName": "Carl St & Cole St",
              "VehicleLocationAtStop": "",
              "VehicleAtStop": "false",
              "DestinationDisplay": "Caltrain/Ballpark",
              "AimedArrivalTime": "2023-05-28T17:06:06Z",
              "ExpectedArrivalTime": "2023-05-28T17:24:16Z",
              "AimedDepartureTime": "2023-05-28T17:06:06Z",
              "ExpectedDepartureTime": null,
              "Distances": ""
            }
          }
        },
        {
          "RecordedAtTime": "1970-01-01T00:00:00Z",
          "MonitoringRef": "13911",
          "MonitoredVehicleJourney": {
            "LineRef": "N",
            "DirectionRef": "IB",
            "FramedVehicleJourneyRef": {
              "DataFrameRef": "2023-05-28",
              "DatedVehicleJourneyRef": "11300633"
            },
            "PublishedLineName": "JUDAH",
            "OperatorRef": "SF",
            "OriginRef": "15223",
            "OriginName": "Judah/La Playa/Ocean Beach",
            "DestinationRef": "15239",
            "DestinationName": "Caltrain/Ballpark",
            "Monitored": true,
            "InCongestion": null,
            "VehicleLocation": {
              "Longitude": "",
              "Latitude": ""
            },
            "Bearing": null,
            "Occupancy": null,
            "VehicleRef": null,
            "MonitoredCall": {
              "StopPointRef": "13911",
              "StopPointName": "Carl St & Cole St",
              "VehicleLocationAtStop": "",
              "VehicleAtStop": "",
              "DestinationDisplay": "Caltrain/Ballpark",
              "AimedArrivalTime": "2023-05-28T17:29:20Z",
              "ExpectedArrivalTime": "2023-05-28T17:40:24Z",
              "AimedDepartureTime": "2023-05-28T17:29:20Z",
              "ExpectedDepartureTime": null,
              "Distances": ""
            }
          }
        }
      ]
    }
  }
}"###;
    let v = serde_json::from_str::<serde_json::Value>(raw_str).unwrap();
    let expected_arrival_time = v["ServiceDelivery"]["StopMonitoringDelivery"]
        ["MonitoredStopVisit"][0]["MonitoredVehicleJourney"]["MonitoredCall"]
        ["ExpectedArrivalTime"]
        .as_str()
        .unwrap();

    // TODO: Need to lookup and track walltime
    let now = NaiveDateTime::parse_from_str("2023-05-28T16:30:00Z", "%+").unwrap();

    NaiveDateTime::parse_from_str(expected_arrival_time, "%+")
        .unwrap()
        .signed_duration_since(now)
        .num_minutes()
}

#[entry]
fn main() -> ! {
    let next_arrival = get_minutes_until_next_arrival();

    let mut ctx = init();

    ctx.display.clear(TriColor::White).unwrap();

    let style = MonoTextStyle::new(&FONT_6X13, TriColor::Chromatic);

    Text::new("N Eastbound", Point::new(5, 30), style)
        .draw(&mut ctx.display)
        .unwrap();

    Text::new(
        format!("{} mins", next_arrival).as_str(),
        Point::new(5, 50),
        style,
    )
    .draw(&mut ctx.display)
    .unwrap();

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

    // TODO: Look into embassy_executor
    loop {
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
            ctx.delay.delay_ms(20u8);
        }
    }
}
