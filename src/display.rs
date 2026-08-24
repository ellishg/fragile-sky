use alloc::format;
use alloc::string::ToString;
use alloc::vec::Vec;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Receiver;
use embedded_graphics::{
    draw_target::DrawTarget,
    mono_font::{
        ascii::{FONT_9X15, FONT_9X15_BOLD},
        MonoTextStyleBuilder,
    },
    prelude::*,
    text::Text,
};
use epd_waveshare::{color::*, prelude::*};
use esp_backtrace as _;
use esp_println::println;

use super::{error, Context};

pub(crate) use error::Result;

#[embassy_executor::task]
pub async fn display_task(
    mut ctx: Context,
    receiver: Receiver<'static, CriticalSectionRawMutex, Vec<(&'static str, Vec<u64>)>, 2>,
) {
    loop {
        let next_arrivals = receiver.receive().await;
        println!("draw frame");
        draw_frame(&mut ctx, next_arrivals).unwrap();
    }
}

fn draw_frame(ctx: &mut Context, next_arrivals: Vec<(&'static str, Vec<u64>)>) -> Result<()> {
    draw_next_arrivals(ctx, next_arrivals)?;
    ctx.epd
        .update_and_display_frame(&mut ctx.spi, ctx.display.buffer(), &mut ctx.delay)?;
    Ok(())
}

fn draw_next_arrivals(
    ctx: &mut Context,
    next_arrivals: Vec<(&'static str, Vec<u64>)>,
) -> Result<()> {
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
    for (i, (name, next_arrivals)) in next_arrivals.iter().enumerate() {
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
