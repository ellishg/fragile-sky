#![no_std]
#![no_main]

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
use esp_println::println;
use hal::{
    clock, gpio,
    peripherals::{Peripherals, SPI2},
    prelude::*,
    pulse_control, spi, timer, Delay, PulseControl, Rtc, IO,
};
use smart_leds::{
    hsv::{hsv2rgb, Hsv},
    SmartLedsWrite,
};

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

#[entry]
fn main() -> ! {
    println!("Initializing...");
    let mut ctx = init();
    println!("done!");

    ctx.display.clear(TriColor::White).unwrap();

    let style = MonoTextStyle::new(&FONT_6X13, TriColor::Chromatic);

    Text::new("Hello world", Point::new(0, 30), style)
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
