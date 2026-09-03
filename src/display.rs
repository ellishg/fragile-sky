pub(crate) use super::error::Result;
use alloc::format;
use alloc::string::ToString;
use alloc::vec::Vec;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::{Channel, Receiver},
    signal::Signal,
};
use embedded_graphics::{
    draw_target::DrawTarget,
    mono_font::{
        ascii::{FONT_10X20, FONT_9X15},
        MonoTextStyleBuilder,
    },
    prelude::*,
    text::Text,
};
use epd_waveshare::{
    color::*,
    epd1in54_v2::{Display1in54, Epd1in54},
    prelude::*,
};
use esp_backtrace as _;
use esp_hal::{delay::Delay, gpio, spi::master::Spi, Blocking};
use esp_println::println;

pub enum FrameState {
    WifiConnecting,
    Arrivals(Vec<(&'static str, Vec<u64>)>),
    PowerOff(&'static Signal<CriticalSectionRawMutex, ()>),
}

type SpiT = embedded_hal_bus::spi::ExclusiveDevice<
    Spi<'static, Blocking>,
    gpio::Output<'static>,
    embedded_hal_bus::spi::NoDelay,
>;
type Epd =
    Epd1in54<SpiT, gpio::Input<'static>, gpio::Output<'static>, gpio::Output<'static>, Delay>;

static DISPLAY_CHANNEL: Channel<CriticalSectionRawMutex, FrameState, 3> = Channel::new();

pub async fn send_display_frame(frame_state: FrameState) {
    DISPLAY_CHANNEL.sender().send(frame_state).await;
}

pub fn spawn_display_task(
    spawner: embassy_executor::Spawner,
    epd: Epd,
    spi: SpiT,
    delay: Delay,
) -> Result<()> {
    let display = Display1in54::default();
    spawner.spawn(display_task(
        epd,
        spi,
        display,
        delay,
        DISPLAY_CHANNEL.receiver(),
    )?);
    Ok(())
}

#[embassy_executor::task]
async fn display_task(
    mut epd: Epd,
    mut spi: SpiT,
    mut display: Display1in54,
    mut delay: super::Delay,
    receiver: Receiver<'static, CriticalSectionRawMutex, FrameState, 3>,
) {
    loop {
        let frame_state = receiver.receive().await;
        println!("draw frame");
        draw_frame(&mut epd, &mut spi, &mut display, &mut delay, frame_state).unwrap();
    }
}

fn draw_frame(
    epd: &mut Epd,
    spi: &mut SpiT,
    display: &mut Display1in54,
    delay: &mut esp_hal::delay::Delay,
    frame_state: FrameState,
) -> Result<()> {
    display.clear(Color::White)?;
    match &frame_state {
        FrameState::WifiConnecting => draw_wifi_connecting(display)?,
        FrameState::Arrivals(next_arrivals) => draw_next_arrivals(display, next_arrivals)?,
        FrameState::PowerOff(_) => draw_power_off(display)?,
    };
    epd.update_and_display_frame(spi, display.buffer(), delay)?;
    if let FrameState::PowerOff(done) = frame_state {
        println!("Power off done, signaling");
        // TODO: Need an async wait so we don't busy wait the CPU.
        epd.wait_until_idle(spi, delay)?;
        done.signal(());
    }
    Ok(())
}

fn draw_wifi_connecting(display: &mut Display1in54) -> Result<()> {
    let style = MonoTextStyleBuilder::new()
        .font(&FONT_10X20)
        .text_color(Color::Black)
        .build();
    Text::new("Connecting to wifi...", Point::new(5, 15), style).draw(display)?;
    Ok(())
}

fn draw_next_arrivals(
    display: &mut Display1in54,
    next_arrivals: &[(&'static str, Vec<u64>)],
) -> Result<()> {
    let name_style = MonoTextStyleBuilder::new()
        .font(&FONT_10X20)
        .text_color(Color::Black)
        .build();
    let style = MonoTextStyleBuilder::new()
        .font(&FONT_9X15)
        .text_color(Color::Black)
        .build();

    // This display is 200x200 px
    for (i, (name, next_arrivals)) in next_arrivals.iter().enumerate() {
        let i = i as i32;
        Text::new(name, Point::new(5, 15 + i * 45), name_style).draw(display)?;

        if !next_arrivals.is_empty() {
            // TODO: Display as:
            // Cole & Carl
            // E: N: 5, 15, 20 mins
            // W: N: 2, 10 mins
            // Haight & Clayton W
            // 37: 7, 8 mins; 33: 10, 40 mins
            //
            // If the next arrival is less than 5 minutes away, also display the one after.
            let joined_next_arrivals = next_arrivals
                .iter()
                .take(5)
                .map(|t| t.to_string())
                .collect::<Vec<_>>()
                .join(",");
            Text::new(
                format!("{} mins", joined_next_arrivals).as_str(),
                Point::new(5, 35 + i * 45),
                style,
            )
            .draw(display)?;
        }
    }
    Ok(())
}

fn draw_power_off(display: &mut Display1in54) -> Result<()> {
    let style = MonoTextStyleBuilder::new()
        .font(&FONT_10X20)
        .text_color(Color::Black)
        .build();
    Text::new("Powering off...", Point::new(5, 15), style).draw(display)?;
    Ok(())
}
