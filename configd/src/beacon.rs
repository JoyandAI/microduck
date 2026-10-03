//! LAN discovery beacon.
//!
//! A phone that is already on the same Wi-Fi learns the robot's address from this, without a
//! Bluetooth scan. The advertisement remains the path that works when the access point drops
//! broadcasts; this is the fast path when it does not. `docs/design/app-connect.md` §3.2.
//!
//! One datagram a second, to the subnet broadcast of the address NetworkManager reports. No
//! query, no answer, no credential. The phone listens on [`PORT`].

use std::net::{Ipv4Addr, SocketAddrV4};
use std::sync::Arc;
use std::time::Duration;

use serde::Serialize;
use tokio::net::UdpSocket;

use crate::net::{LanAddress, Net};

/// Where a phone listens. Not in the payload: the payload names the robot, and the port is
/// this constant on both sides.
pub const PORT: u16 = 47090;

/// First bytes of every datagram, so a listener can drop everything else without parsing it.
pub const MAGIC: &[u8] = b"MDK1\n";

/// A host address worth announcing, and the broadcast to send it to.
///
/// `None` for loopback, link-local, a prefix that has no subnet broadcast, and an address that
/// is the network or the broadcast itself. `0.0.0.0` is the "no address" case the BLE
/// advertisement already reports; sending it here would tell a phone nothing.
pub fn subnet_broadcast(address: Ipv4Addr, prefix: u8) -> Option<Ipv4Addr> {
    if !(1..=30).contains(&prefix) || !is_lan_host(address) {
        return None;
    }
    let mask = u32::MAX << (32 - prefix);
    let ip = u32::from(address);
    let network = ip & mask;
    let broadcast = network | !mask;
    if ip == network || ip == broadcast {
        return None;
    }
    Some(Ipv4Addr::from(broadcast))
}

/// The first candidate that [`subnet_broadcast`] accepts.
pub fn select(candidates: &[(Ipv4Addr, u8)]) -> Option<LanAddress> {
    candidates.iter().find_map(|&(address, prefix)| {
        subnet_broadcast(address, prefix).map(|_| LanAddress { address, prefix })
    })
}

fn is_lan_host(address: Ipv4Addr) -> bool {
    let octets = address.octets();
    let link_local = octets[0] == 169 && octets[1] == 254;
    !address.is_unspecified()
        && !address.is_loopback()
        && !address.is_multicast()
        && !address.is_broadcast()
        && !link_local
}

/// `MDK1\n` plus one JSON object. `serial` is JSON `null` when the board has none.
pub fn datagram(name: &str, serial: Option<&str>, api: u32, ip4: Ipv4Addr) -> Vec<u8> {
    #[derive(Serialize)]
    struct Body<'a> {
        v: u32,
        name: &'a str,
        serial: Option<&'a str>,
        api: u32,
        ip4: String,
    }
    let body = Body {
        v: 1,
        name,
        serial,
        api,
        ip4: ip4.to_string(),
    };
    let mut bytes = Vec::with_capacity(128);
    bytes.extend_from_slice(MAGIC);
    serde_json::to_writer(&mut bytes, &body).expect("beacon fields are plain strings and numbers");
    bytes.push(b'\n');
    bytes
}

/// Send until the task is dropped. A send failure is logged when it changes and retried next
/// second; it does not stop `configd`, because the socket this process exists to serve is the
/// unix one.
pub async fn run(net: Arc<dyn Net>, name: impl Fn() -> String + Send + 'static, serial: Option<String>) {
    let mut ticker = tokio::time::interval(Duration::from_secs(1));
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    let mut bound: Option<(Ipv4Addr, UdpSocket)> = None;
    let mut last_error: Option<String> = None;

    loop {
        ticker.tick().await;
        let announcement = match net.lan().await {
            Ok(Some(lan)) => subnet_broadcast(lan.address, lan.prefix).map(|broadcast| (lan, broadcast)),
            Ok(None) => None,
            Err(error) => {
                note(&mut last_error, error);
                bound = None;
                continue;
            }
        };
        let Some((lan, broadcast)) = announcement else {
            bound = None;
            last_error = None;
            continue;
        };

        if bound.as_ref().is_none_or(|(address, _)| *address != lan.address) {
            match UdpSocket::bind((lan.address, 0)).await {
                Ok(socket) => match socket.set_broadcast(true) {
                    Ok(()) => bound = Some((lan.address, socket)),
                    Err(error) => {
                        note(&mut last_error, error.to_string());
                        bound = None;
                        continue;
                    }
                },
                Err(error) => {
                    note(&mut last_error, error.to_string());
                    bound = None;
                    continue;
                }
            }
        }

        let Some((_, socket)) = bound.as_ref() else {
            continue;
        };
        let packet = datagram(&name(), serial.as_deref(), duck_ipc_proto::API_VERSION, lan.address);
        let destination = SocketAddrV4::new(broadcast, PORT);
        match socket.send_to(&packet, destination).await {
            Ok(_) => last_error = None,
            Err(error) => {
                note(&mut last_error, error.to_string());
                bound = None;
            }
        }
    }
}

fn note(last: &mut Option<String>, error: String) {
    if last.as_deref() != Some(error.as_str()) {
        tracing::warn!(error = %error, "lan beacon");
        *last = Some(error);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_lan_address_broadcasts_to_its_subnet() {
        let address = Ipv4Addr::new(192, 168, 1, 42);
        assert_eq!(
            subnet_broadcast(address, 24),
            Some(Ipv4Addr::new(192, 168, 1, 255))
        );
        assert_eq!(
            subnet_broadcast(Ipv4Addr::new(10, 1, 2, 3), 8),
            Some(Ipv4Addr::new(10, 255, 255, 255))
        );
    }

    #[test]
    fn addresses_a_phone_cannot_use_are_not_announced() {
        assert!(subnet_broadcast(Ipv4Addr::UNSPECIFIED, 24).is_none());
        assert!(subnet_broadcast(Ipv4Addr::LOCALHOST, 8).is_none());
        assert!(subnet_broadcast(Ipv4Addr::new(169, 254, 1, 1), 16).is_none());
        assert!(subnet_broadcast(Ipv4Addr::new(192, 168, 1, 42), 32).is_none());
        assert!(subnet_broadcast(Ipv4Addr::new(192, 168, 1, 0), 24).is_none());
        assert!(subnet_broadcast(Ipv4Addr::new(192, 168, 1, 255), 24).is_none());
    }

    #[test]
    fn the_first_usable_address_wins() {
        let chosen = select(&[
            (Ipv4Addr::new(169, 254, 3, 4), 16),
            (Ipv4Addr::new(192, 168, 50, 63), 24),
        ])
        .expect("the private address is usable");
        assert_eq!(chosen.address, Ipv4Addr::new(192, 168, 50, 63));
        assert_eq!(chosen.prefix, 24);
    }

    #[test]
    fn the_datagram_is_the_magic_and_one_json_object() {
        let bytes = datagram("duck-c51b", Some("bb7b734a7717ac41"), 17, Ipv4Addr::new(192, 168, 1, 42));
        assert!(bytes.len() < 256, "a discovery packet stays in one frame");
        let text = std::str::from_utf8(&bytes).unwrap();
        let (magic, rest) = text.split_once('\n').unwrap();
        assert_eq!(magic, "MDK1");
        let body: serde_json::Value = serde_json::from_str(rest.trim_end()).unwrap();
        assert_eq!(body["v"], 1);
        assert_eq!(body["name"], "duck-c51b");
        assert_eq!(body["serial"], "bb7b734a7717ac41");
        assert_eq!(body["api"], 17);
        assert_eq!(body["ip4"], "192.168.1.42");
        assert!(body.get("psk").is_none());
    }

    #[test]
    fn a_board_with_no_serial_says_so_as_null() {
        let bytes = datagram("robot", None, 17, Ipv4Addr::new(192, 168, 1, 42));
        let text = std::str::from_utf8(&bytes).unwrap();
        let body: serde_json::Value = serde_json::from_str(text.split_once('\n').unwrap().1.trim_end()).unwrap();
        assert!(body["serial"].is_null());
    }
}
