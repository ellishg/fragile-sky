pub(crate) use super::error::Result;
use esp_hal::i2c::master::Config as I2cConfig;
use esp_hal::{
    gpio::{Input, InputConfig, Pull},
    i2c::master::I2c,
    peripherals::GPIO2,
    Async,
};
use esp_println::println;

// TODO: Document these
const TCA9554_ADDR: u8 = 0x20;
const REG_CONFIG: u8 = 0x03;
const REG_OUTPUT: u8 = 0x01;
const POWER_LATCH_PIN: u8 = 1 << 5;
const POWER_LATCH_CONFIG: u8 = !POWER_LATCH_PIN;
const POWER_UNLATCHED: u8 = 0x00;

pub fn spawn_power_task(
    spawner: embassy_executor::Spawner,
    i2c: esp_hal::peripherals::I2C0<'static>,
    sda: esp_hal::peripherals::GPIO18<'static>,
    scl: esp_hal::peripherals::GPIO8<'static>,
    button: GPIO2<'static>,
) -> Result<()> {
    let i2c = I2c::new(i2c, I2cConfig::default())?
        .with_sda(sda)
        .with_scl(scl)
        .into_async();
    let button = Input::new(button, InputConfig::default().with_pull(Pull::Up));
    spawner.spawn(power_task(i2c, button)?);
    Ok(())
}

#[embassy_executor::task]
async fn power_task(i2c: I2c<'static, Async>, button: Input<'static>) {
    power_task_impl(i2c, button).await.unwrap();
}

async fn power_task_impl(mut i2c: I2c<'static, Async>, mut button: Input<'static>) -> Result<()> {
    // Latch power so the battery continues to power the board after the power button is released.
    i2c.write_async(TCA9554_ADDR, &[REG_CONFIG, POWER_LATCH_CONFIG])
        .await?;
    i2c.write_async(TCA9554_ADDR, &[REG_OUTPUT, POWER_LATCH_PIN])
        .await?;

    // Low means the button is pressed, so the falling edge is when the button was just pressed.
    button.wait_for_falling_edge().await;

    println!("Power button pressed, unlatching power");
    // TODO: Draw to screen to indicate device is off.
    i2c.write_async(TCA9554_ADDR, &[REG_OUTPUT, POWER_UNLATCHED])
        .await?;
    Ok(())
}
