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
use esp_println::println;
use reqwless::{
    client::HttpClient,
    request::{Method, RequestBuilder},
};

static TCP_CLIENT_STATE: static_cell::StaticCell<TcpClientState<1, 1500, 1500>> =
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
    stack: Stack<'static>,
    api_key: &'static str,
) -> crate::Result<Vec<(&'static str, Vec<u64>)>> {
    let tcp_client = TcpClient::new(
        stack,
        TCP_CLIENT_STATE.uninit().write(TcpClientState::new()),
    );
    let dns_client = DnsSocket::new(stack);

    let mut client = HttpClient::new(&tcp_client, &dns_client);
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
        println!("done");
    }

    Ok(next_arrivals)
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
