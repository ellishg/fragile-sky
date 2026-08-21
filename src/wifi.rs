use embassy_executor::Spawner;
use embassy_net::{Runner, Stack, StackResources};
use embassy_time::{Duration, Timer};
use esp_hal::peripherals::WIFI;
use esp_println::println;
use esp_radio::wifi;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

pub async fn connect(
    spawner: Spawner,
    device: WIFI<'static>,
    ssid: &'static str,
    password: &'static str,
) -> Result<Stack<'static>, wifi::WifiError> {
    let client_config = wifi::Config::Station(
        wifi::sta::StationConfig::default()
            .with_ssid(ssid)
            .with_password(password.into()),
    );
    let wifi_interface = wifi::Interface::station();
    let mut wifi_controller = wifi::WifiController::new(
        device,
        wifi::ControllerConfig::default().with_initial_config(client_config),
    )?;

    let config = embassy_net::Config::dhcpv4(Default::default());
    let rng = esp_hal::rng::Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    let (stack, runner) = embassy_net::new(
        wifi_interface,
        config,
        mk_static!(StackResources<3>, StackResources::<3>::new()),
        seed,
    );

    let scan_config = wifi::scan::ScanConfig::default().with_max(10);
    wifi_controller.scan_async(&scan_config).await?;

    spawner.spawn(connection(wifi_controller).unwrap());
    spawner.spawn(net_task(runner).unwrap());

    stack.wait_config_up().await;

    if let Some(config) = stack.config_v4() {
        println!("Got IP: {}", config.address);
    }

    Ok(stack)
}

#[embassy_executor::task]
async fn connection(mut controller: wifi::WifiController<'static>) {
    loop {
        match controller.connect_async().await {
            Ok(info) => {
                println!("Wifi connected to {:?}", info);

                let info = controller.wait_for_disconnect_async().await.ok();
                println!("Disconnected: {:?}", info);
            }
            Err(e) => {
                println!("Failed to connect to wifi: {e:?}");
            }
        }

        Timer::after(Duration::from_millis(5000)).await
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, wifi::Interface>) {
    runner.run().await
}
