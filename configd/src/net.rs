//! Wifi, as a trait plus a fake.
//!
//! **NetworkManager owns the credentials** (`architecture.md` §3). `configd` never stores a
//! PSK: it hands one to NM, which persists the profile root-only and reconnects on its own.
//! That is less code, better security, and one less thing to migrate — and it survives
//! `configd` being restarted, updated or rolled back.
//!
//! The trait exists for the same reason `duck-control` has `RobotIo`: the suite runs on a
//! laptop with no hardware, no network and no D-Bus, and the logic worth testing is the
//! dispatch and authorisation around this, not NM itself.

use async_trait::async_trait;
use duck_ipc_proto as proto;

/// What went wrong, in terms a caller can act on.
pub type NetResult<T> = Result<T, String>;

#[async_trait]
pub trait Net: Send + Sync {
    async fn status(&self) -> NetResult<proto::NetStatusResult>;
    async fn scan(&self) -> NetResult<proto::NetScanResult>;
    /// Join `ssid`, storing it so NM reconnects by itself next time.
    async fn connect(&self, ssid: &str, psk: Option<&str>) -> NetResult<proto::ConnectResult>;
    async fn forget(&self, ssid: &str) -> NetResult<proto::ForgetResult>;
}

/// A wifi stack that exists only in memory.
///
/// Used by every test and by `--fake`, which is how the whole `net.*` surface can be exercised
/// end to end from a laptop — including the failures that are awkward to provoke against a real
/// access point, like a wrong passphrase.
pub struct FakeNet {
    inner: tokio::sync::Mutex<FakeState>,
}

struct FakeState {
    /// What the radio can see, and the key each one actually wants.
    visible: Vec<(proto::Network, Option<String>)>,
    saved: Vec<String>,
    connected: Option<String>,
}

impl FakeNet {
    /// Two networks in range: one WPA2 with a known key, one open.
    pub fn new() -> Self {
        Self::with_visible(vec![
            (
                proto::Network {
                    ssid: "Pollen".into(),
                    signal: 82,
                    security: proto::Security::WpaPsk,
                    saved: false,
                },
                Some("correct-key".to_owned()),
            ),
            (
                proto::Network {
                    ssid: "Cafe".into(),
                    signal: 41,
                    security: proto::Security::Open,
                    saved: false,
                },
                None,
            ),
        ])
    }

    pub fn with_visible(visible: Vec<(proto::Network, Option<String>)>) -> Self {
        Self {
            inner: tokio::sync::Mutex::new(FakeState { visible, saved: Vec::new(), connected: None }),
        }
    }
}

impl Default for FakeNet {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl Net for FakeNet {
    async fn status(&self) -> NetResult<proto::NetStatusResult> {
        let state = self.inner.lock().await;
        Ok(match &state.connected {
            Some(ssid) => proto::NetStatusResult {
                state: proto::NetState::Connected,
                ssid: Some(ssid.clone()),
                signal: state
                    .visible
                    .iter()
                    .find(|(n, _)| &n.ssid == ssid)
                    .map(|(n, _)| n.signal),
                ip4: Some("192.168.50.63".into()),
                ip6: None,
                mac: Some("50:37:cd:16:1b:92".into()),
                iface: Some("wlan0".into()),
            },
            None => proto::NetStatusResult {
                state: proto::NetState::Disconnected,
                ssid: None,
                signal: None,
                ip4: None,
                ip6: None,
                mac: Some("50:37:cd:16:1b:92".into()),
                iface: Some("wlan0".into()),
            },
        })
    }

    async fn scan(&self) -> NetResult<proto::NetScanResult> {
        let state = self.inner.lock().await;
        let mut networks: Vec<proto::Network> = state
            .visible
            .iter()
            .map(|(n, _)| proto::Network { saved: state.saved.contains(&n.ssid), ..n.clone() })
            .collect();
        networks.sort_by_key(|n| std::cmp::Reverse(n.signal));
        Ok(proto::NetScanResult { networks })
    }

    async fn connect(&self, ssid: &str, psk: Option<&str>) -> NetResult<proto::ConnectResult> {
        let mut state = self.inner.lock().await;

        let Some((network, wanted)) = state.visible.iter().find(|(n, _)| n.ssid == ssid).cloned()
        else {
            return Ok(proto::ConnectResult::Failed {
                reason: proto::ConnectFailure::NotFound,
                detail: None,
            });
        };

        if network.security == proto::Security::Enterprise {
            return Ok(proto::ConnectResult::Failed {
                reason: proto::ConnectFailure::Unsupported,
                detail: Some("802.1X needs a certificate flow this API does not have".into()),
            });
        }
        if wanted.is_some() && psk.is_none() {
            return Ok(proto::ConnectResult::Failed {
                reason: proto::ConnectFailure::Unsupported,
                detail: Some("this network needs a passphrase".into()),
            });
        }
        if wanted.is_some() && wanted.as_deref() != psk {
            return Ok(proto::ConnectResult::Failed {
                reason: proto::ConnectFailure::BadKey,
                detail: None,
            });
        }

        state.connected = Some(ssid.to_owned());
        if !state.saved.iter().any(|s| s == ssid) {
            state.saved.push(ssid.to_owned());
        }
        Ok(proto::ConnectResult::Connected {
            ssid: ssid.to_owned(),
            ip4: Some("192.168.50.63".into()),
        })
    }

    async fn forget(&self, ssid: &str) -> NetResult<proto::ForgetResult> {
        let mut state = self.inner.lock().await;
        let before = state.saved.len();
        state.saved.retain(|s| s != ssid);
        if state.connected.as_deref() == Some(ssid) {
            state.connected = None;
        }
        Ok(proto::ForgetResult { removed: state.saved.len() != before })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn a_wrong_key_is_reported_as_a_wrong_key() {
        let net = FakeNet::new();
        let result = net.connect("Pollen", Some("wrong")).await.unwrap();
        assert!(matches!(
            result,
            proto::ConnectResult::Failed { reason: proto::ConnectFailure::BadKey, .. }
        ));
    }

    /// A secured network with no passphrase is refused before trying, and distinguishably from a
    /// wrong one — a client should ask for a password rather than say "that was wrong".
    #[tokio::test]
    async fn a_missing_key_is_not_a_wrong_key() {
        let net = FakeNet::new();
        let result = net.connect("Pollen", None).await.unwrap();
        assert!(matches!(
            result,
            proto::ConnectResult::Failed { reason: proto::ConnectFailure::Unsupported, .. }
        ));
    }

    #[tokio::test]
    async fn an_unknown_ssid_is_not_found() {
        let net = FakeNet::new();
        assert!(matches!(
            net.connect("Nowhere", Some("k")).await.unwrap(),
            proto::ConnectResult::Failed { reason: proto::ConnectFailure::NotFound, .. }
        ));
    }

    /// The whole provisioning arc: scan, join, see it stored and connected, forget it.
    #[tokio::test]
    async fn connecting_stores_the_network_and_forgetting_removes_it() {
        let net = FakeNet::new();
        assert!(net.scan().await.unwrap().networks.iter().all(|n| !n.saved));

        assert!(matches!(
            net.connect("Pollen", Some("correct-key")).await.unwrap(),
            proto::ConnectResult::Connected { .. }
        ));

        let status = net.status().await.unwrap();
        assert_eq!(status.state, proto::NetState::Connected);
        assert_eq!(status.ssid.as_deref(), Some("Pollen"));
        assert!(status.ip4.is_some(), "connected with no address");

        let saved = net.scan().await.unwrap();
        assert!(saved.networks.iter().find(|n| n.ssid == "Pollen").unwrap().saved);

        assert!(net.forget("Pollen").await.unwrap().removed);
        assert_eq!(net.status().await.unwrap().state, proto::NetState::Disconnected);
        // Forgetting again is not an error — a client must not present it as one.
        assert!(!net.forget("Pollen").await.unwrap().removed);
    }

    /// An open network needs no key, and asking with one is not an error either.
    #[tokio::test]
    async fn an_open_network_joins_without_a_key() {
        let net = FakeNet::new();
        assert!(matches!(
            net.connect("Cafe", None).await.unwrap(),
            proto::ConnectResult::Connected { .. }
        ));
    }

    /// Strongest first, because that is the order a phone shows them in.
    #[tokio::test]
    async fn scan_results_are_sorted_by_signal() {
        let net = FakeNet::new();
        let networks = net.scan().await.unwrap().networks;
        assert!(networks.windows(2).all(|w| w[0].signal >= w[1].signal), "{networks:?}");
    }
}
