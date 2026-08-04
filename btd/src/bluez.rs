//! The radio. BlueZ via `bluetoothd`'s D-Bus API, Linux only.
//!
//! Everything here is plumbing between BlueZ and [`crate::session`]'s two channels. No decision
//! about the robot is taken in this file, which is the point: the logic that could be wrong is
//! the logic that is tested, and this is the part that needs a radio.
//!
//! It uses `bluer`'s **IO model** rather than its callback model. The callback model hands you a
//! notifier with no way to tell which central subscribed, so pairing a subscription to the
//! session that should feed it means guessing — one global slot, and a second phone breaks it.
//! The IO model reports `device_address()` on both halves, so a connection is a real duplex
//! stream and several centrals cost nothing extra.
//!
//! One characteristic carries both directions, for the reason in [`crate::gatt`]: BlueZ reports a
//! write and a subscription as separate events, and with two characteristics they would have to be
//! matched across them by address rather than belonging to the same characteristic by
//! construction.
//!
//! **Untested against hardware.** It type-checks for aarch64 and has never met a real central.
//! Treat what follows as intent until someone connects a phone.

use std::collections::HashMap;
use std::time::Duration;

use bluer::adv::Advertisement;
use bluer::agent::{Agent, ReqError};
// The reader and writer live on `gatt` rather than `gatt::local`: they are the same socket
// halves the client side uses, so bluer shares them between both roles.
use bluer::gatt::local::{
    Application, Characteristic, CharacteristicControlEvent, CharacteristicNotify,
    CharacteristicNotifyMethod, CharacteristicWrite, CharacteristicWriteMethod, Service,
    characteristic_control,
};
use bluer::gatt::{CharacteristicReader, CharacteristicWriter};
use futures::StreamExt;
use tokio::io::{AsyncReadExt, AsyncWriteExt};

use crate::gatt::{RPC_UUID, SERVICE_UUID};
use crate::link::{Link, QUEUE};
use crate::pairing;
use crate::session;
use crate::upstream::Sockets;

/// How long to wait between attempts to find a usable adapter.
///
/// Measured on the board: `hci0` does not exist until roughly 73 seconds after power-on —
/// `aic-bluetooth.service` attaches the AIC8800's UART late, and `bluetooth.service` itself
/// spends 26s blocked behind `dbus`. A daemon that exited on "no adapter" would be restarted by
/// systemd into the same emptiness for over a minute, so it waits. Same lesson as `robotd`
/// waiting for the motor bus rather than giving up on it.
const ADAPTER_RETRY: Duration = Duration::from_secs(5);

/// Halves of a connection seen so far. BlueZ delivers the write and notify sides as separate
/// events, in either order, so a session starts when the second one arrives.
#[derive(Default)]
struct Half {
    reader: Option<CharacteristicReader>,
    writer: Option<CharacteristicWriter>,
}

/// Wait for an adapter, then advertise and serve until cancelled.
///
/// `require_pairing` controls whether writing a request needs an authenticated, encrypted link.
/// It defaults on, because §7 requires it for anything carrying wifi credentials and
/// `net.connect` now does. The opt-out exists for bench work against a client that cannot pair.
pub async fn serve(sockets: Sockets, name: String, require_pairing: bool) -> bluer::Result<()> {
    let bt = bluer::Session::new().await?;

    let adapter = loop {
        match bt.default_adapter().await {
            Ok(adapter) => break adapter,
            Err(e) => {
                tracing::warn!(error = %e, retry_in = ?ADAPTER_RETRY, "no Bluetooth adapter yet");
                tokio::time::sleep(ADAPTER_RETRY).await;
            }
        }
    };
    adapter.set_powered(true).await?;

    // Pairable only matters while we advertise, and the board reports `Pairable: no` by default.
    // A pairing *window* — opened by a physical button rather than left open — is the obvious
    // next step and needs a button that does not exist yet.
    if require_pairing {
        adapter.set_pairable(true).await?;
    }

    // The agent answers BlueZ's passkey request with the PIN configd holds. Registered before
    // advertising, or a phone quick off the mark could reach pairing before there is anything to
    // answer it.
    let _agent = if require_pairing {
        let config_socket = sockets.config.clone();
        Some(
            bt.register_agent(Agent {
                request_default: true,
                request_passkey: Some(Box::new(move |request| {
                    let config_socket = config_socket.clone();
                    Box::pin(async move {
                        tracing::info!(peer = %request.device, "pairing requested");
                        match pairing::pin(&config_socket).await {
                            Ok(passkey) => Ok(passkey),
                            Err(e) => {
                                // Refusing is the only safe answer. Falling back to a default
                                // would let anyone pair whenever configd was briefly unreachable.
                                tracing::warn!(error = %e, "cannot read the pairing PIN; refusing to pair");
                                Err(ReqError::Rejected)
                            }
                        }
                    })
                })),
                ..Default::default()
            })
            .await?,
        )
    } else {
        tracing::warn!(
            "pairing NOT required: any device in range can write requests, including wifi \
             credentials. Bench use only."
        );
        None
    };

    tracing::warn!(
        adapter = adapter.name(),
        address = %adapter.address().await?,
        service = %SERVICE_UUID,
        pairing = require_pairing,
        "serving BLE"
    );

    // The advertised name is what someone sees in a phone's Bluetooth list, so it is the robot's
    // name rather than the service's. `system.setName` will rewrite it once `configd` exists;
    // until then it is the hostname, which is at least unique per board.
    let advertisement = Advertisement {
        service_uuids: [SERVICE_UUID].into_iter().collect(),
        discoverable: Some(true),
        local_name: Some(name),
        ..Default::default()
    };
    let _adv = adapter.advertise(advertisement).await?;

    let (control, control_handle) = characteristic_control();
    let app = Application {
        services: vec![Service {
            uuid: SERVICE_UUID,
            primary: true,
            characteristics: vec![Characteristic {
                uuid: RPC_UUID,
                write: Some(CharacteristicWrite {
                    write: true,
                    // Write-without-response too: a chunked request does not need an ATT
                    // acknowledgement per chunk, and requiring one roughly halves throughput on
                    // the slowest link in the system.
                    write_without_response: true,
                    // §7: anything carrying wifi credentials must be paired and encrypted, and
                    // `net.connect` does. `encrypt_authenticated_write` demands a bond made with
                    // passkey entry rather than just-works, which is what makes the PIN mean
                    // something — see `crate::pairing` for what it does and does not prove.
                    encrypt_authenticated_write: require_pairing,
                    method: CharacteristicWriteMethod::Io,
                    ..Default::default()
                }),
                notify: Some(CharacteristicNotify {
                    notify: true,
                    method: CharacteristicNotifyMethod::Io,
                    ..Default::default()
                }),
                control_handle,
                ..Default::default()
            }],
            ..Default::default()
        }],
        ..Default::default()
    };
    let _app = adapter.serve_gatt_application(app).await?;

    // Half-open connections, keyed by central. Entries are short-lived: BlueZ sends both events
    // for a connecting client in quick succession, and an entry that never completes is one
    // half of a client that gave up.
    let mut halves: HashMap<bluer::Address, Half> = HashMap::new();

    futures::pin_mut!(control);
    while let Some(event) = control.next().await {
        let address = match &event {
            CharacteristicControlEvent::Write(req) => req.device_address(),
            CharacteristicControlEvent::Notify(notifier) => notifier.device_address(),
        };

        let half = halves.entry(address).or_default();
        let mtu = match event {
            CharacteristicControlEvent::Write(req) => {
                let mtu = req.mtu();
                match req.accept() {
                    Ok(reader) => half.reader = Some(reader),
                    Err(e) => {
                        tracing::warn!(peer = %address, error = %e, "could not accept a write");
                        halves.remove(&address);
                        continue;
                    }
                }
                mtu
            }
            CharacteristicControlEvent::Notify(notifier) => {
                let mtu = notifier.mtu();
                half.writer = Some(notifier);
                mtu
            }
        };

        // Both halves present: this is a whole connection, so it becomes a session.
        if half.reader.is_some() && half.writer.is_some() {
            let Half { reader, writer } = halves.remove(&address).expect("entry present");
            let (reader, writer) = (reader.expect("reader"), writer.expect("writer"));
            tracing::info!(peer = %address, mtu, "central connected");
            spawn_session(address, mtu, reader, writer, sockets.clone());
        }
    }

    tracing::warn!("BlueZ closed the characteristic control stream");
    Ok(())
}

/// Bridge one connection's reader and writer onto a [`Link`], and run a session over it.
fn spawn_session(
    address: bluer::Address,
    mtu: usize,
    mut reader: CharacteristicReader,
    mut writer: CharacteristicWriter,
    sockets: Sockets,
) {
    let (link, to_session, mut from_session) = Link::pair(mtu, address.to_string());
    tokio::spawn(session::run(link, sockets));

    // Radio → session.
    tokio::spawn(async move {
        // One MTU per read: BlueZ delivers a characteristic write as a datagram, so a larger
        // buffer would not coalesce chunks and a smaller one would truncate them.
        let mut buf = vec![0u8; mtu.max(QUEUE)];
        loop {
            match reader.read(&mut buf).await {
                Ok(0) => break,
                Ok(n) => {
                    if to_session.send(buf[..n].to_vec()).await.is_err() {
                        break;
                    }
                }
                Err(e) => {
                    tracing::debug!(peer = %address, error = %e, "read failed");
                    break;
                }
            }
        }
        tracing::debug!(peer = %address, "radio reader closed");
    });

    // Session → radio. Chunks arrive already sized for this MTU, so each one is a single
    // notification.
    tokio::spawn(async move {
        while let Some(chunk) = from_session.recv().await {
            if let Err(e) = writer.write_all(&chunk).await {
                tracing::debug!(peer = %address, error = %e, "notify failed; central gone");
                break;
            }
        }
        let _ = writer.shutdown().await;
        tracing::debug!(peer = %address, "radio writer closed");
    });
}

