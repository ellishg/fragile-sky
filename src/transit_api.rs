pub(crate) use crate::error::Result;
use alloc::format;
use alloc::vec;
use alloc::vec::Vec;
use anyhow::anyhow;
use chrono::NaiveDateTime;
use embassy_net::{
    dns::DnsSocket,
    tcp::client::{TcpClient, TcpClientState},
    Stack,
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Sender;
use embassy_time::Timer;
use esp_println::println;
use reqwless::{
    client::HttpClient,
    request::{Method, RequestBuilder},
};

// TODO: Find good sizes
pub static TCP_CLIENT_STATE: static_cell::StaticCell<TcpClientState<1, 1500, 1500>> =
    static_cell::StaticCell::new();

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

pub async fn fetch_arrivals(
    client: &mut TcpClient<'static, 1, 1500, 1500>,
    stack: Stack<'static>,
    api_key: &'static str,
) -> crate::Result<Vec<(&'static str, Vec<u64>)>> {
    let dns_client = DnsSocket::new(stack);

    let mut client = HttpClient::new(client, &dns_client);
    let mut rx_buf = [0u8; 16 * 1024];

    let mut next_arrivals = vec![];
    for stop_config in STOP_CONFIGS.iter() {
        let url = format!(
            "http://api.511.org/transit/StopMonitoring?api_key={}&agency=SF&format=json&stopcode={}",
            api_key, stop_config.stop_code
        );
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
    }

    Ok(next_arrivals)
}

#[embassy_executor::task]
pub async fn update_arrivals_task(
    stack: Stack<'static>,
    api_key: &'static str,
    tcp_client_state: &'static mut TcpClientState<1, 1500, 1500>,
    sender: Sender<'static, CriticalSectionRawMutex, Vec<(&'static str, Vec<u64>)>, 2>,
) {
    let mut tcp_client = TcpClient::new(stack, tcp_client_state);
    update_arrivals(&mut tcp_client, stack, api_key, sender)
        .await
        .unwrap();
}

async fn update_arrivals(
    tcp_client: &mut TcpClient<'static, 1, 1500, 1500>,
    stack: Stack<'static>,
    api_key: &'static str,
    sender: Sender<'static, CriticalSectionRawMutex, Vec<(&'static str, Vec<u64>)>, 2>,
) -> Result<()> {
    loop {
        // TODO: Configure how often to fetch times from network
        let mut next_arrivals = fetch_arrivals(tcp_client, stack, api_key).await?;
        sender.send(next_arrivals.clone()).await;
        for _ in 0..5 {
            Timer::after_secs(60).await;
            // TODO:
            for arrivals in &mut next_arrivals {
                arrivals.1 = arrivals
                    .1
                    .iter()
                    .filter_map(|time| time.checked_sub(1))
                    .collect();
            }
            sender.send(next_arrivals.clone()).await;
        }
    }
}

fn get_minutes_until_next_arrivals(content: &str) -> crate::Result<Vec<u64>> {
    let v: serde_json::Value = serde_json::from_str(content)?;

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
