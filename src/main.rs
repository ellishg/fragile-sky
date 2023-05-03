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
use esp_hal_smartled::{smartLedAdapter, SmartLedsAdapter};
use esp_println::println;
use hal::{
    clock::ClockControl,
    peripherals::Peripherals,
    prelude::*,
    pulse_control::ClockSource,
    spi::{Spi, SpiMode},
    timer::TimerGroup,
    Delay, PulseControl, Rtc, IO,
};
use smart_leds::{
    brightness, gamma,
    hsv::{hsv2rgb, Hsv},
    SmartLedsWrite,
};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.PCR.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.LP_CLKRST);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;
    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();
    // TODO: Figure out why printing is messed up sometimes
    println!("hello world");

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut delay = Delay::new(&clocks);

    let clk = io.pins.gpio18;
    let din = io.pins.gpio23;

    let mut spi = Spi::new_no_cs_no_miso(
        peripherals.SPI2,
        clk,
        din,
        4u32.MHz(),
        SpiMode::Mode0,
        &mut system.peripheral_clock_control,
        &clocks,
    );

    let cs = io.pins.gpio5.into_push_pull_output();
    let busy = io.pins.gpio4.into_floating_input();
    let dc = io.pins.gpio17.into_push_pull_output();
    let rst = io.pins.gpio16.into_push_pull_output();

    let mut epd = Epd2in13b::new(&mut spi, cs, busy, dc, rst, &mut delay, None).unwrap();

    let mut display = Display2in13b::default();

    display.clear(TriColor::White).unwrap();

    let style = MonoTextStyle::new(&FONT_6X13, TriColor::Chromatic);

    Text::new("Hello world", Point::new(0, 30), style)
        .draw(&mut display)
        .unwrap();

    // TODO: Improve the speed of updating frames
    epd.update_color_frame(
        &mut spi,
        &mut delay,
        display.bw_buffer(),
        display.chromatic_buffer(),
    )
    .unwrap();
    epd.display_frame(&mut spi, &mut delay).unwrap();

    display.clear(TriColor::White).unwrap();
    epd.update_color_frame(
        &mut spi,
        &mut delay,
        display.bw_buffer(),
        display.chromatic_buffer(),
    )
    .unwrap();
    epd.display_frame(&mut spi, &mut delay).unwrap();

    // Configure RMT peripheral globally
    let pulse = PulseControl::new(
        peripherals.RMT,
        &mut system.peripheral_clock_control,
        ClockSource::APB,
        0,
        0,
        0,
    )
    .unwrap();

    // We use one of the RMT channels to instantiate a `SmartLedsAdapter` which can
    // be used directly with all `smart_led` implementations
    let mut led = <smartLedAdapter!(1)>::new(pulse.channel0, io.pins.gpio8);

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
            led.write(brightness(gamma(data.iter().cloned()), 5))
                .unwrap();
            delay.delay_ms(20u8);
        }
    }
}
