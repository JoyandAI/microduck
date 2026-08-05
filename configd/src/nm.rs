//! NetworkManager over D-Bus. Linux only.
//!
//! `zbus` rather than the `dbus` crate: pure Rust, so no vendored C, and NM's settings are a
//! nested `a{sa{sv}}` that `zvariant` expresses without ceremony. The cost is that the shipped
//! artifact now carries two D-Bus stacks — `btd` links libdbus through `bluer` — which is real
//! and worth revisiting if `bluer` ever grows a `zbus` backend, or if the BlueZ calls turn out
//! small enough to hand-roll.
//!
//! **Untested against a real NetworkManager.** It type-checks for aarch64; every claim here is
//! intent until it runs on the board.

use std::collections::HashMap;
use std::time::Duration;

use async_trait::async_trait;
use duck_ipc_proto as proto;
use zbus::zvariant::{OwnedObjectPath, OwnedValue, Value};

use crate::net::{Net, NetResult};

/// Values from NetworkManager's own `nm-dbus-interface.h`, read from upstream rather than
/// remembered. These are the numbers that make "you typed the password wrong" reportable, which
/// is the entire reason NM was chosen over netplan — so they are named, not inlined.
///
/// Named `ids` rather than `nm`: a module repeating its own file's name is what clippy's
/// `module_inception` objects to, and `ids::ids::REASON_NO_SECRETS` read no better than this does.
mod ids {
    pub const DEVICE_TYPE_WIFI: u32 = 2;

    pub const STATE_UNAVAILABLE: u32 = 20;
    pub const STATE_DISCONNECTED: u32 = 30;
    pub const STATE_ACTIVATED: u32 = 100;
    pub const STATE_FAILED: u32 = 120;

    pub const REASON_NO_SECRETS: u32 = 7;
    pub const REASON_SUPPLICANT_DISCONNECT: u32 = 8;
    pub const REASON_SUPPLICANT_TIMEOUT: u32 = 11;
    pub const REASON_SSID_NOT_FOUND: u32 = 53;
    pub const REASON_IP_CONFIG_UNAVAILABLE: u32 = 5;

    /// `NM_802_11_AP_SEC_KEY_MGMT_PSK`
    pub const SEC_KEY_MGMT_PSK: u32 = 0x100;
    /// `NM_802_11_AP_SEC_KEY_MGMT_802_1X`
    pub const SEC_KEY_MGMT_802_1X: u32 = 0x200;
    /// `NM_802_11_AP_SEC_KEY_MGMT_SAE`
    pub const SEC_KEY_MGMT_SAE: u32 = 0x400;
    /// `NM_802_11_AP_FLAGS_PRIVACY`
    pub const AP_FLAGS_PRIVACY: u32 = 0x1;
}

/// How long to wait for a join to resolve one way or the other.
///
/// Association is quick; DHCP is what takes time on a busy network. Long enough to succeed on a
/// slow one, short enough that a phone gets an answer instead of a spinner — and a `Timeout` is
/// a reportable outcome rather than a hang, which is the point.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(45);

#[zbus::proxy(
    interface = "org.freedesktop.NetworkManager",
    default_service = "org.freedesktop.NetworkManager",
    default_path = "/org/freedesktop/NetworkManager"
)]
trait Manager {
    fn get_devices(&self) -> zbus::Result<Vec<OwnedObjectPath>>;

    #[zbus(name = "AddAndActivateConnection")]
    fn add_and_activate_connection(
        &self,
        connection: HashMap<&str, HashMap<&str, Value<'_>>>,
        device: &zbus::zvariant::ObjectPath<'_>,
        specific_object: &zbus::zvariant::ObjectPath<'_>,
    ) -> zbus::Result<(OwnedObjectPath, OwnedObjectPath)>;
}

#[zbus::proxy(
    interface = "org.freedesktop.NetworkManager.Device",
    default_service = "org.freedesktop.NetworkManager"
)]
trait Device {
    #[zbus(property)]
    fn device_type(&self) -> zbus::Result<u32>;
    #[zbus(property)]
    fn interface(&self) -> zbus::Result<String>;
    #[zbus(property)]
    fn state(&self) -> zbus::Result<u32>;
    #[zbus(property)]
    fn state_reason(&self) -> zbus::Result<(u32, u32)>;
    #[zbus(property, name = "Ip4Config")]
    fn ip4_config(&self) -> zbus::Result<OwnedObjectPath>;
    #[zbus(property, name = "Ip6Config")]
    fn ip6_config(&self) -> zbus::Result<OwnedObjectPath>;
}

#[zbus::proxy(
    interface = "org.freedesktop.NetworkManager.Device.Wireless",
    default_service = "org.freedesktop.NetworkManager"
)]
trait Wireless {
    fn request_scan(&self, options: HashMap<&str, Value<'_>>) -> zbus::Result<()>;
    fn get_all_access_points(&self) -> zbus::Result<Vec<OwnedObjectPath>>;
    #[zbus(property)]
    fn hw_address(&self) -> zbus::Result<String>;
    #[zbus(property)]
    fn active_access_point(&self) -> zbus::Result<OwnedObjectPath>;
}

#[zbus::proxy(
    interface = "org.freedesktop.NetworkManager.AccessPoint",
    default_service = "org.freedesktop.NetworkManager"
)]
trait AccessPoint {
    #[zbus(property)]
    fn ssid(&self) -> zbus::Result<Vec<u8>>;
    #[zbus(property)]
    fn strength(&self) -> zbus::Result<u8>;
    #[zbus(property)]
    fn flags(&self) -> zbus::Result<u32>;
    #[zbus(property)]
    fn rsn_flags(&self) -> zbus::Result<u32>;
    #[zbus(property)]
    fn wpa_flags(&self) -> zbus::Result<u32>;
}

#[zbus::proxy(
    interface = "org.freedesktop.NetworkManager.IP4Config",
    default_service = "org.freedesktop.NetworkManager"
)]
trait Ip4Config {
    #[zbus(property)]
    fn address_data(&self) -> zbus::Result<Vec<HashMap<String, OwnedValue>>>;
}

#[zbus::proxy(
    interface = "org.freedesktop.NetworkManager.IP6Config",
    default_service = "org.freedesktop.NetworkManager"
)]
trait Ip6Config {
    #[zbus(property)]
    fn address_data(&self) -> zbus::Result<Vec<HashMap<String, OwnedValue>>>;
}

#[zbus::proxy(
    interface = "org.freedesktop.NetworkManager.Settings",
    default_service = "org.freedesktop.NetworkManager",
    default_path = "/org/freedesktop/NetworkManager/Settings"
)]
trait Settings {
    fn list_connections(&self) -> zbus::Result<Vec<OwnedObjectPath>>;
}

#[zbus::proxy(
    interface = "org.freedesktop.NetworkManager.Settings.Connection",
    default_service = "org.freedesktop.NetworkManager"
)]
trait Connection {
    fn get_settings(&self) -> zbus::Result<HashMap<String, HashMap<String, OwnedValue>>>;
    fn delete(&self) -> zbus::Result<()>;
}

pub struct NetworkManager {
    bus: zbus::Connection,
}

impl NetworkManager {
    pub async fn new() -> zbus::Result<Self> {
        Ok(Self {
            bus: zbus::Connection::system().await?,
        })
    }

    /// The first wifi device NM knows about.
    ///
    /// `None` is a real answer, not an error: on a board still running netplan, NM manages no
    /// wifi device at all, and that reports as `Unavailable` rather than a failure — which is a
    /// provisioning problem the caller can be told about (`scripts/migrate-network.sh`).
    async fn wifi_device(&self) -> NetResult<Option<OwnedObjectPath>> {
        let manager = ManagerProxy::new(&self.bus).await.map_err(bus_err)?;
        for path in manager.get_devices().await.map_err(bus_err)? {
            let device = DeviceProxy::builder(&self.bus)
                .path(&path)
                .map_err(bus_err)?
                .build()
                .await
                .map_err(bus_err)?;
            if device.device_type().await.unwrap_or(0) == ids::DEVICE_TYPE_WIFI {
                return Ok(Some(path));
            }
        }
        Ok(None)
    }

    async fn first_address(&self, addresses: Vec<HashMap<String, OwnedValue>>) -> Option<String> {
        addresses
            .first()
            .and_then(|entry| entry.get("address").cloned())
            .and_then(|value| String::try_from(value).ok())
    }

    /// Which stored profile, if any, is for this SSID.
    /// Every saved profile for `ssid` — plural, deliberately.
    ///
    /// NetworkManager permits duplicate connection ids, so "the profile for this SSID" is not a
    /// thing that exists. Returning `Option` was wrong in a way that mattered: `net.forget` removed
    /// one of two profiles and reported success, leaving a stale one with an outdated key that NM
    /// would happily autoconnect with later.
    async fn saved_connections(&self, ssid: &str) -> NetResult<Vec<OwnedObjectPath>> {
        let settings = SettingsProxy::new(&self.bus).await.map_err(bus_err)?;
        let mut matches = Vec::new();
        for path in settings.list_connections().await.map_err(bus_err)? {
            let connection = ConnectionProxy::builder(&self.bus)
                .path(&path)
                .map_err(bus_err)?
                .build()
                .await
                .map_err(bus_err)?;
            let Ok(config) = connection.get_settings().await else {
                continue;
            };

            // The SSID lives as raw bytes under the wifi section, because an SSID is not
            // required to be UTF-8. Ours are compared as text, which is what a user typed.
            let stored = config
                .get("802-11-wireless")
                .and_then(|section| section.get("ssid"))
                .and_then(|value| Vec::<u8>::try_from(value.clone()).ok())
                .and_then(|bytes| String::from_utf8(bytes).ok());

            if stored.as_deref() == Some(ssid) {
                matches.push(path);
            }
        }
        Ok(matches)
    }

    /// Delete every saved profile for `ssid`. Returns how many went.
    async fn delete_saved(&self, ssid: &str) -> NetResult<usize> {
        let paths = self.saved_connections(ssid).await?;
        let mut deleted = 0;
        for path in &paths {
            let connection = ConnectionProxy::builder(&self.bus)
                .path(path)
                .map_err(bus_err)?
                .build()
                .await
                .map_err(bus_err)?;
            connection.delete().await.map_err(bus_err)?;
            deleted += 1;
        }
        Ok(deleted)
    }
}

fn bus_err(e: impl std::fmt::Display) -> String {
    format!("NetworkManager D-Bus call failed: {e}")
}

/// AP security flags → what a client needs to know to ask for the right thing.
fn security_of(flags: u32, rsn: u32, wpa: u32) -> proto::Security {
    let key_mgmt = rsn | wpa;
    if key_mgmt & ids::SEC_KEY_MGMT_802_1X != 0 {
        proto::Security::Enterprise
    } else if key_mgmt & ids::SEC_KEY_MGMT_SAE != 0 {
        proto::Security::Wpa3Sae
    } else if key_mgmt & ids::SEC_KEY_MGMT_PSK != 0 {
        proto::Security::WpaPsk
    } else if flags & ids::AP_FLAGS_PRIVACY != 0 {
        // Privacy bit set but no WPA/RSN key management: WEP, which nothing should still be
        // using. Reported so a client can say "too old to join" rather than failing obscurely.
        proto::Security::Wep
    } else {
        proto::Security::Open
    }
}

/// A device state and reason → why the join failed.
///
/// This mapping is the payoff for choosing NetworkManager: `BadKey` is a distinct, actionable
/// answer, and no other stack on the board reports it.
fn failure_of(state: u32, reason: u32) -> (proto::ConnectFailure, Option<String>) {
    let detail = Some(format!("NetworkManager state {state}, reason {reason}"));
    let failure = match reason {
        ids::REASON_NO_SECRETS => proto::ConnectFailure::BadKey,
        ids::REASON_SSID_NOT_FOUND => proto::ConnectFailure::NotFound,
        ids::REASON_SUPPLICANT_TIMEOUT
        | ids::REASON_SUPPLICANT_DISCONNECT
        | ids::REASON_IP_CONFIG_UNAVAILABLE => proto::ConnectFailure::Timeout,
        _ => proto::ConnectFailure::Other,
    };
    (failure, detail)
}

#[async_trait]
impl Net for NetworkManager {
    async fn status(&self) -> NetResult<proto::NetStatusResult> {
        let Some(path) = self.wifi_device().await? else {
            return Ok(proto::NetStatusResult {
                state: proto::NetState::Unavailable,
                ssid: None,
                signal: None,
                ip4: None,
                ip6: None,
                mac: None,
                iface: None,
            });
        };

        let device = DeviceProxy::builder(&self.bus)
            .path(&path)
            .map_err(bus_err)?
            .build()
            .await
            .map_err(bus_err)?;
        let wireless = WirelessProxy::builder(&self.bus)
            .path(&path)
            .map_err(bus_err)?
            .build()
            .await
            .map_err(bus_err)?;

        let raw_state = device.state().await.unwrap_or(ids::STATE_UNAVAILABLE);
        let state = match raw_state {
            ids::STATE_ACTIVATED => proto::NetState::Connected,
            ids::STATE_UNAVAILABLE => proto::NetState::Unavailable,
            ids::STATE_DISCONNECTED | ids::STATE_FAILED => proto::NetState::Disconnected,
            // Everything between disconnected and activated is "still trying", and a client
            // should poll rather than conclude anything.
            _ => proto::NetState::Connecting,
        };

        let (mut ssid, mut signal) = (None, None);
        if let Ok(ap_path) = wireless.active_access_point().await
            && ap_path.as_str() != "/"
            && let Ok(ap) = AccessPointProxy::builder(&self.bus)
                .path(&ap_path)
                .map_err(bus_err)?
                .build()
                .await
        {
            ssid = ap
                .ssid()
                .await
                .ok()
                .and_then(|bytes| String::from_utf8(bytes).ok());
            signal = ap.strength().await.ok();
        }

        let mut ip4 = None;
        if let Ok(config_path) = device.ip4_config().await
            && config_path.as_str() != "/"
            && let Ok(config) = Ip4ConfigProxy::builder(&self.bus)
                .path(&config_path)
                .map_err(bus_err)?
                .build()
                .await
            && let Ok(addresses) = config.address_data().await
        {
            ip4 = self.first_address(addresses).await;
        }

        let mut ip6 = None;
        if let Ok(config_path) = device.ip6_config().await
            && config_path.as_str() != "/"
            && let Ok(config) = Ip6ConfigProxy::builder(&self.bus)
                .path(&config_path)
                .map_err(bus_err)?
                .build()
                .await
            && let Ok(addresses) = config.address_data().await
        {
            ip6 = self.first_address(addresses).await;
        }

        Ok(proto::NetStatusResult {
            state,
            ssid,
            signal,
            ip4,
            ip6,
            mac: wireless.hw_address().await.ok(),
            iface: device.interface().await.ok(),
        })
    }

    async fn scan(&self) -> NetResult<proto::NetScanResult> {
        let Some(path) = self.wifi_device().await? else {
            return Ok(proto::NetScanResult {
                networks: Vec::new(),
            });
        };
        let wireless = WirelessProxy::builder(&self.bus)
            .path(&path)
            .map_err(bus_err)?
            .build()
            .await
            .map_err(bus_err)?;

        // A scan request is a hint, not a command: NM rate-limits it and refuses while one is in
        // flight. Either way the cached access-point list below is what we return, which is why
        // the error is dropped rather than reported — a client asking twice in a second should
        // get the previous results, not a failure.
        let _ = wireless.request_scan(HashMap::new()).await;

        let mut networks: Vec<proto::Network> = Vec::new();
        for ap_path in wireless.get_all_access_points().await.map_err(bus_err)? {
            let Ok(ap) = AccessPointProxy::builder(&self.bus)
                .path(&ap_path)
                .map_err(bus_err)?
                .build()
                .await
            else {
                continue;
            };
            let Some(ssid) = ap.ssid().await.ok().and_then(|b| String::from_utf8(b).ok()) else {
                // A hidden network advertises an empty SSID, and one that is not UTF-8 cannot be
                // named in JSON. Skipped rather than shown as a blank row.
                continue;
            };
            if ssid.is_empty() {
                continue;
            }

            let security = security_of(
                ap.flags().await.unwrap_or(0),
                ap.rsn_flags().await.unwrap_or(0),
                ap.wpa_flags().await.unwrap_or(0),
            );
            let signal = ap.strength().await.unwrap_or(0);
            let saved = !self.saved_connections(&ssid).await?.is_empty();

            // One entry per SSID, strongest wins. A mesh or a dual-band router presents several
            // access points for one network, and a list with "Pollen" five times is a worse
            // answer than the truth.
            match networks.iter_mut().find(|n| n.ssid == ssid) {
                Some(existing) if existing.signal < signal => existing.signal = signal,
                Some(_) => {}
                None => networks.push(proto::Network {
                    ssid,
                    signal,
                    security,
                    saved,
                }),
            }
        }

        networks.sort_by_key(|n| std::cmp::Reverse(n.signal));
        Ok(proto::NetScanResult { networks })
    }

    async fn connect(&self, ssid: &str, psk: Option<&str>) -> NetResult<proto::ConnectResult> {
        let Some(device_path) = self.wifi_device().await? else {
            return Ok(proto::ConnectResult::Failed {
                reason: proto::ConnectFailure::Unsupported,
                detail: Some(
                    "NetworkManager manages no wifi device; this board may still be on netplan"
                        .into(),
                ),
            });
        };

        // Refuse what we cannot do, before changing anything. An enterprise network needs a
        // username and certificate flow this API has no shape for, and half-attempting it would
        // leave a broken profile behind.
        if let Some(found) = self
            .scan()
            .await?
            .networks
            .iter()
            .find(|n| n.ssid == ssid)
            .cloned()
        {
            if found.security == proto::Security::Enterprise {
                return Ok(proto::ConnectResult::Failed {
                    reason: proto::ConnectFailure::Unsupported,
                    detail: Some("802.1X networks need a certificate flow this API lacks".into()),
                });
            }
            if found.security != proto::Security::Open && psk.is_none() {
                return Ok(proto::ConnectResult::Failed {
                    reason: proto::ConnectFailure::Unsupported,
                    detail: Some("this network needs a passphrase".into()),
                });
            }
        }

        // Replace any profile this SSID already has, rather than adding a second one.
        //
        // `AddAndActivateConnection` always *adds*, and NM allows two profiles with the same id. The
        // path that makes this matter is the ordinary one: a passphrase mistyped on a phone fails
        // with `BadKey`, the user re-sends the right one, and the robot is left carrying both — with
        // no guarantee about which NM autoconnects with after the next reboot. Re-provisioning is
        // how this API is *expected* to be used, so it has to be idempotent in the profile it
        // leaves behind.
        //
        // Deleted before adding, not after: the alternative leaves duplicates behind whenever the
        // add succeeds and the cleanup does not. If the add then fails, the SSID is left with no
        // profile — which is the correct outcome for "this configuration is being replaced" and is
        // reported to the client, rather than a silent half-state.
        //
        // This disconnects the robot if that profile is the active one. Unavoidable: changing a key
        // means re-associating, and a client on BLE is unaffected by design.
        match self.delete_saved(ssid).await {
            Ok(0) => {}
            Ok(n) => tracing::info!(ssid, replaced = n, "replacing saved profile(s)"),
            // Not fatal. A stale profile is a problem for the *next* boot; refusing to connect at
            // all is a problem right now, and a robot in a new place needs to get online.
            Err(e) => tracing::warn!(ssid, error = %e, "could not remove the saved profile(s)"),
        }

        let manager = ManagerProxy::new(&self.bus).await.map_err(bus_err)?;

        // The settings dictionary NM wants. Only `802-11-wireless.ssid` and the key are ours to
        // state; `autoconnect` defaults on, which is what makes the robot rejoin by itself after
        // a reboot — the property that keeps `configd` out of the reconnect business entirely.
        let mut wireless: HashMap<&str, Value<'_>> = HashMap::new();
        wireless.insert("ssid", Value::from(ssid.as_bytes().to_vec()));
        wireless.insert("mode", Value::from("infrastructure"));

        let mut connection: HashMap<&str, Value<'_>> = HashMap::new();
        connection.insert("id", Value::from(ssid));
        connection.insert("type", Value::from("802-11-wireless"));

        let mut settings: HashMap<&str, HashMap<&str, Value<'_>>> = HashMap::new();
        settings.insert("connection", connection);
        settings.insert("802-11-wireless", wireless);

        if let Some(psk) = psk {
            let mut security: HashMap<&str, Value<'_>> = HashMap::new();
            // `wpa-psk` covers WPA2 and WPA2/WPA3-transition. A WPA3-only network wants `sae`,
            // and NM is lenient enough to negotiate in practice — if a board proves otherwise,
            // this is where the scan's `Security` should choose.
            security.insert("key-mgmt", Value::from("wpa-psk"));
            security.insert("psk", Value::from(psk));
            settings.insert("802-11-wireless-security", security);
        }

        let root = zbus::zvariant::ObjectPath::try_from("/").map_err(bus_err)?;
        let device = zbus::zvariant::ObjectPath::try_from(device_path.as_str()).map_err(bus_err)?;

        if let Err(e) = manager
            .add_and_activate_connection(settings, &device, &root)
            .await
        {
            return Ok(proto::ConnectResult::Failed {
                reason: proto::ConnectFailure::Other,
                detail: Some(format!("{e}")),
            });
        }

        // NM returns as soon as activation *starts*, so the outcome has to be waited for. This
        // poll is what turns "config applied" into "associated, addressed, and here is the IP" —
        // the difference that made netplan unusable for provisioning.
        let deadline = tokio::time::Instant::now() + CONNECT_TIMEOUT;
        let device_proxy = DeviceProxy::builder(&self.bus)
            .path(&device_path)
            .map_err(bus_err)?
            .build()
            .await
            .map_err(bus_err)?;

        loop {
            match device_proxy.state().await.unwrap_or(ids::STATE_UNAVAILABLE) {
                ids::STATE_ACTIVATED => {
                    let status = self.status().await?;
                    return Ok(proto::ConnectResult::Connected {
                        ssid: status.ssid.unwrap_or_else(|| ssid.to_owned()),
                        ip4: status.ip4,
                    });
                }
                ids::STATE_FAILED => {
                    let (_, reason) = device_proxy.state_reason().await.unwrap_or((0, 0));
                    let (failure, detail) = failure_of(ids::STATE_FAILED, reason);
                    return Ok(proto::ConnectResult::Failed {
                        reason: failure,
                        detail,
                    });
                }
                _ => {}
            }

            if tokio::time::Instant::now() >= deadline {
                let (_, reason) = device_proxy.state_reason().await.unwrap_or((0, 0));
                // A timeout still carries NM's reason: "still authenticating after 45s" and
                // "waiting for DHCP" are different problems.
                let (_, detail) = failure_of(ids::STATE_FAILED, reason);
                return Ok(proto::ConnectResult::Failed {
                    reason: proto::ConnectFailure::Timeout,
                    detail,
                });
            }
            tokio::time::sleep(Duration::from_millis(500)).await;
        }
    }

    async fn forget(&self, ssid: &str) -> NetResult<proto::ForgetResult> {
        let deleted = self.delete_saved(ssid).await?;
        Ok(proto::ForgetResult {
            removed: deleted > 0,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The security mapping decides whether a client asks for a password, and which kind. Pure
    /// arithmetic on NM's flags, so it is testable without a bus — and worth testing, because
    /// getting `Open` wrong for a secured network means silently attempting an unauthenticated
    /// join.
    #[test]
    fn security_flags_map_to_what_a_client_must_ask_for() {
        assert_eq!(security_of(0, 0, 0), proto::Security::Open);
        assert_eq!(
            security_of(ids::AP_FLAGS_PRIVACY, 0, 0),
            proto::Security::Wep
        );
        assert_eq!(
            security_of(ids::AP_FLAGS_PRIVACY, ids::SEC_KEY_MGMT_PSK, 0),
            proto::Security::WpaPsk
        );
        assert_eq!(
            security_of(ids::AP_FLAGS_PRIVACY, 0, ids::SEC_KEY_MGMT_PSK),
            proto::Security::WpaPsk
        );
        assert_eq!(
            security_of(ids::AP_FLAGS_PRIVACY, ids::SEC_KEY_MGMT_SAE, 0),
            proto::Security::Wpa3Sae
        );
        assert_eq!(
            security_of(ids::AP_FLAGS_PRIVACY, ids::SEC_KEY_MGMT_802_1X, 0),
            proto::Security::Enterprise
        );

        // A WPA2/WPA3 transition AP advertises both; SAE wins, because a client offering SAE
        // gets the better of the two and PSK still works underneath.
        assert_eq!(
            security_of(
                ids::AP_FLAGS_PRIVACY,
                ids::SEC_KEY_MGMT_PSK | ids::SEC_KEY_MGMT_SAE,
                0
            ),
            proto::Security::Wpa3Sae
        );
        // Enterprise outranks everything: attempting a PSK join there cannot work.
        assert_eq!(
            security_of(
                ids::AP_FLAGS_PRIVACY,
                ids::SEC_KEY_MGMT_PSK | ids::SEC_KEY_MGMT_802_1X,
                0
            ),
            proto::Security::Enterprise
        );
    }

    /// The reason mapping is why NetworkManager was chosen over netplan, so the one that matters
    /// most is pinned: a rejected key must be `BadKey` and nothing else.
    #[test]
    fn a_rejected_key_is_reported_as_bad_key() {
        let (failure, detail) = failure_of(ids::STATE_FAILED, ids::REASON_NO_SECRETS);
        assert_eq!(failure, proto::ConnectFailure::BadKey);
        // The detail carries NM's raw numbers for a support ticket, but is never the primary
        // message a user sees.
        assert!(detail.unwrap().contains("reason 7"));

        assert_eq!(
            failure_of(ids::STATE_FAILED, ids::REASON_SSID_NOT_FOUND).0,
            proto::ConnectFailure::NotFound
        );
        assert_eq!(
            failure_of(ids::STATE_FAILED, ids::REASON_SUPPLICANT_TIMEOUT).0,
            proto::ConnectFailure::Timeout
        );
        // An unmapped reason must not masquerade as a wrong password — that would send a user
        // round a loop retyping a key that was already correct.
        assert_eq!(
            failure_of(ids::STATE_FAILED, 999).0,
            proto::ConnectFailure::Other
        );
    }
}
