//! The duck chorale's radio: a beacon out, and other ducks' beacons in.
//!
//! **Still a transport adapter.** `btd` owns no chorale state and makes no chorale decisions — it
//! broadcasts bytes it was handed and reports bytes it heard, which is the same job it does for
//! JSON-RPC. Who conducts, who sings what, and when a piece starts are all `robotd`'s, because
//! they are behaviour.
//!
//! It lives here rather than in a daemon of its own because everything it needs is already here:
//! the adapter, `bluer`, the D-Bus permissions, and a connection to `robotd`. A second process
//! contending for the same adapter would be more moving parts for no separation that matters.
//!
//! ## Out: a second advertising instance
//!
//! The board reports five advertising instances with one in use, so the beacon gets its own and
//! **the existing advertisement is not touched.** That is not tidiness. `crate::adv` documents a
//! 31-byte budget, and the controller here reports a 251-byte one — so BlueZ would happily accept
//! a bigger payload on the existing instance and, because it picks legacy against extended PDUs by
//! size, would switch it to extended and make the robot invisible to a legacy-only scanner. Phone
//! discovery would regress and it would look like a Bluetooth fault rather than a chorale one.
//!
//! ## Registered on demand, and this one is load-bearing
//!
//! A controller interleaves its advertising instances, so registering a second one **halves the
//! rate of the first**. `crate::bluez`'s interval was tuned against measurements — the default
//! 1.28 s left a robot absent for up to 31 s at a time, and 100–150 ms fixed it — so permanently
//! halving it would spend that hard-won margin on a feature nobody has asked for yet. The beacon
//! is therefore registered only while a chorale is wanted, and dropped when it is not.
//!
//! ## In: a hardware-offloaded passive scan
//!
//! BlueZ's advertisement monitor ([`bluer::monitor`]) filters on a byte pattern in the controller,
//! so the host is woken only for advertisements that are already chorale beacons. Passive: the
//! duck transmits nothing to scan, which matters because one antenna carries this, the gamepad's
//! link and wifi.
//!
//! The pattern matches the manufacturer-data field on company id `0xFFFF` followed by
//! [`ChoraleBeacon::TAG`]. That tag is why the *other* instance's four bytes of IPv4 address are
//! not delivered here as a beat.
//!
//! ## What the arrival time means
//!
//! A sighting is stamped when this process sees it, which is after the controller, the kernel and
//! a D-Bus property change. That path adds a few milliseconds and some jitter — and the design
//! upstream ([`sounds::chorale::beat`], in the `sounds` crate) is built for exactly that: it
//! averages the phase over many beats, so anything random averages out, and anything *constant* is
//! common to every duck on identical hardware and so inaudible. What it cannot absorb is a
//! systematic difference between the conductor's idea of when its beat went out and the
//! followers' — which is why that constant wants measuring on hardware rather than assuming.

use std::collections::HashMap;

use duck_ipc_proto::ChoraleBeacon;

/// The company id chorale beacons ride under — the same one [`crate::adv`] uses, for the same
/// reason: `0xFFFF` is the id the SIG reserves for testing and is the correct choice for a project
/// that has not been assigned one.
pub const COMPANY_ID: u16 = crate::adv::COMPANY_ID;

/// The advertising interval for the beacon.
///
/// Faster than [`crate::bluez`]'s 100–150 ms, and deliberately: this one carries a beat, and how
/// promptly a payload change reaches the air *is* the sync error. It is affordable because the
/// instance only exists while a chorale is running — see the module docs.
pub const BEACON_INTERVAL_MIN: std::time::Duration = std::time::Duration::from_millis(20);
pub const BEACON_INTERVAL_MAX: std::time::Duration = std::time::Duration::from_millis(40);

/// The bytes a scan filter has to match, from the start of the manufacturer-data AD field.
///
/// Little-endian company id, then the beacon's tag. Matching in the controller rather than in the
/// host is what makes the scan cost nothing while nothing is singing.
pub fn scan_pattern() -> Vec<u8> {
    let id = COMPANY_ID.to_le_bytes();
    vec![id[0], id[1], ChoraleBeacon::TAG]
}

/// A beacon as the advertisement's manufacturer-data payload.
///
/// The payload rather than the map, mirroring [`crate::adv::address_data`] — and because the two
/// halves want different containers: `bluer` advertises from a `BTreeMap` and reports a scanned
/// device's data as a `HashMap`.
pub fn beacon_data(beacon: &ChoraleBeacon) -> Vec<u8> {
    beacon.to_bytes()
}

/// The beacon in a device's manufacturer data, if there is one.
///
/// `None` for anything else on the same company id — the address field the other instance
/// broadcasts, or another vendor using `0xFFFF`, which anyone may. The tag is the discriminator;
/// see [`ChoraleBeacon::from_bytes`].
pub fn beacon_in(manufacturer_data: &HashMap<u16, Vec<u8>>) -> Option<ChoraleBeacon> {
    ChoraleBeacon::from_bytes(manufacturer_data.get(&COMPANY_ID)?)
}

#[cfg(target_os = "linux")]
pub use radio::{Sighting, broadcast, watch};

#[cfg(target_os = "linux")]
mod radio {
    use std::time::Instant;

    use bluer::adv::Advertisement;
    use bluer::monitor::{Monitor, MonitorEvent, Pattern, RssiSamplingPeriod};
    use duck_ipc_proto::ChoraleBeacon;
    use futures::StreamExt;
    use tokio::sync::mpsc;

    use super::{BEACON_INTERVAL_MAX, BEACON_INTERVAL_MIN, beacon_data, beacon_in, scan_pattern};

    /// AD type for manufacturer-specific data. What the monitor pattern is matched against.
    const AD_TYPE_MANUFACTURER_DATA: u8 = 0xFF;

    /// One beacon heard from another duck.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct Sighting {
        pub beacon: ChoraleBeacon,
        /// Which duck. Only an identity for de-duplicating — a beacon says nothing about who is
        /// broadcasting it beyond its register and tie-break byte.
        pub from: bluer::Address,
        /// When this process saw it. See the module docs for what that is and is not.
        pub at: Instant,
    }

    /// Put a beacon on the air, on its own advertising instance.
    ///
    /// Dropping the returned handle stops it, which is how the instance is released the moment a
    /// chorale ends — the whole reason it is registered on demand.
    ///
    /// Carries neither the service UUID nor a local name: the scan filters on the manufacturer
    /// data, so an 18-byte UUID would buy nothing and a name would only make the payload big
    /// enough to change PDU type.
    pub async fn broadcast(
        adapter: &bluer::Adapter,
        beacon: &ChoraleBeacon,
    ) -> bluer::Result<bluer::adv::AdvertisementHandle> {
        adapter
            .advertise(Advertisement {
                manufacturer_data: [(super::COMPANY_ID, beacon_data(beacon))]
                    .into_iter()
                    .collect(),
                // Not discoverable: this instance is a beacon, not a way in. The robot's front
                // door is the other advertisement, and it is unchanged.
                discoverable: Some(false),
                min_interval: Some(BEACON_INTERVAL_MIN),
                max_interval: Some(BEACON_INTERVAL_MAX),
                ..Default::default()
            })
            .await
    }

    /// Watch for other ducks' beacons, forever, sending each sighting to `tx`.
    ///
    /// Returns when the channel closes or the adapter goes away, so the caller can restart it the
    /// same way `bluez::serve` restarts on losing an adapter.
    pub async fn watch(adapter: &bluer::Adapter, tx: mpsc::Sender<Sighting>) -> bluer::Result<()> {
        let manager = adapter.monitor().await?;
        let mut monitor = manager
            .register(Monitor {
                monitor_type: bluer::monitor::Type::OrPatterns,
                patterns: Some(vec![Pattern {
                    data_type: AD_TYPE_MANUFACTURER_DATA,
                    start_position: 0,
                    content: scan_pattern(),
                }]),
                // Every packet, not just the first sighting of a device: the payload is what
                // changes, and a beat we are told about once is not a beat.
                rssi_sampling_period: Some(RssiSamplingPeriod::All),
                ..Default::default()
            })
            .await?;
        tracing::warn!(
            pattern = ?scan_pattern(),
            "chorale: listening for other ducks"
        );

        while let Some(event) = monitor.next().await {
            let MonitorEvent::DeviceFound(found) = event else {
                continue;
            };
            let Ok(device) = adapter.device(found.device) else {
                continue;
            };
            let tx = tx.clone();
            // One task per duck heard, watching its advertisement for changes. The monitor
            // reports a device once; the *payload* arriving repeatedly is a property change on it.
            tokio::spawn(async move {
                let address = device.address();
                // Whatever it was already advertising when the monitor first matched it — the
                // first beat is otherwise missed while waiting for a change that already happened.
                if let Ok(Some(data)) = device.manufacturer_data().await
                    && let Some(beacon) = beacon_in(&data)
                {
                    let sighting = Sighting {
                        beacon,
                        from: address,
                        at: Instant::now(),
                    };
                    if tx.send(sighting).await.is_err() {
                        return;
                    }
                }
                let Ok(mut events) = device.events().await else {
                    return;
                };
                while let Some(event) = events.next().await {
                    let bluer::DeviceEvent::PropertyChanged(
                        bluer::DeviceProperty::ManufacturerData(data),
                    ) = event
                    else {
                        continue;
                    };
                    // Stamped here, as early as this process can: everything after is jitter the
                    // phase average has to absorb.
                    let at = Instant::now();
                    let Some(beacon) = beacon_in(&data) else {
                        continue;
                    };
                    if tx
                        .send(Sighting {
                            beacon,
                            from: address,
                            at,
                        })
                        .await
                        .is_err()
                    {
                        return;
                    }
                }
            });
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn beacon() -> ChoraleBeacon {
        ChoraleBeacon {
            piece: 2,
            beat: 91,
            register: 56,
            id: 0x3D,
        }
    }

    fn advertised(beacon: &ChoraleBeacon) -> HashMap<u16, Vec<u8>> {
        HashMap::from([(COMPANY_ID, beacon_data(beacon))])
    }

    /// The round trip the broadcasting half and the scanning half both depend on — the same
    /// property `crate::adv` pins for the address field, and for the same reason.
    #[test]
    fn a_beacon_survives_the_advertisement() {
        assert_eq!(beacon_in(&advertised(&beacon())), Some(beacon()));
    }

    /// The trap the tag exists to close: the *other* advertising instance broadcasts four bytes of
    /// IPv4 under the same company id, and a scanner that read those as a beacon would hear a beat
    /// in an address.
    #[test]
    fn the_address_instance_is_not_heard_as_a_beat() {
        let address = HashMap::from([(
            COMPANY_ID,
            crate::adv::address_data(Some(std::net::Ipv4Addr::new(192, 168, 1, 42))),
        )]);
        assert_eq!(beacon_in(&address), None);
        // And the reverse: a beacon is not read as an address, so a scanning `duck-btctl` does not
        // report a robot at some nonsense IP.
        let as_advertised = advertised(&beacon());
        assert_eq!(crate::adv::address_in(&as_advertised), None);
        assert!(
            !crate::adv::has_address_field(&as_advertised),
            "five bytes is not an address field"
        );
    }

    /// The scan filter has to match what the beacon actually broadcasts, byte for byte, or the
    /// controller drops every beat and the failure looks like a radio problem.
    #[test]
    fn the_scan_pattern_matches_what_is_broadcast() {
        let pattern = scan_pattern();
        // The AD field is the little-endian company id followed by the payload.
        let mut field = COMPANY_ID.to_le_bytes().to_vec();
        field.extend(beacon_data(&beacon()));
        assert!(
            field.starts_with(&pattern),
            "pattern {pattern:?} does not prefix the broadcast field {field:?}"
        );
        // Company id first, little-endian, then the tag — the order a controller matches in.
        assert_eq!(pattern, vec![0xFF, 0xFF, ChoraleBeacon::TAG]);
        // An address advertisement must *not* match, or the filter buys nothing.
        let mut address_field = COMPANY_ID.to_le_bytes().to_vec();
        address_field.extend(crate::adv::address_data(None));
        assert!(!address_field.starts_with(&pattern));
    }

    /// Another vendor on the testing company id, or a future beacon this build does not know, is
    /// not a beat. `0xFFFF` is unassigned and anyone may use it.
    #[test]
    fn somebody_elses_payload_is_not_a_beacon() {
        for payload in [vec![], vec![0x01], vec![0xC0], vec![0xC0, 1, 2, 3, 4, 5]] {
            let data = HashMap::from([(COMPANY_ID, payload.clone())]);
            assert_eq!(beacon_in(&data), None, "{payload:?}");
        }
        // A different company id is not ours however well-formed it looks.
        let elsewhere = HashMap::from([(0x004C, beacon().to_bytes())]);
        assert_eq!(beacon_in(&elsewhere), None);
    }

    /// The beacon advertises faster than the front door, and that is only affordable because the
    /// instance is transient. If someone makes it permanent, these numbers are the argument.
    #[test]
    fn the_beacon_is_faster_than_the_front_door() {
        assert!(BEACON_INTERVAL_MIN < BEACON_INTERVAL_MAX);
        // The spec's floor is 20 ms; going under it would be refused by the controller.
        assert!(BEACON_INTERVAL_MIN >= std::time::Duration::from_millis(20));
        // And genuinely faster than the always-on advertisement, or a beat would reach the air no
        // sooner than the robot's name does.
        assert!(BEACON_INTERVAL_MAX < std::time::Duration::from_millis(100));
    }
}
