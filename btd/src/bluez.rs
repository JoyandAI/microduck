//! The radio. BlueZ via `bluetoothd`'s D-Bus API, Linux only.
//!
//! Everything here is plumbing between BlueZ and [`crate::session`]'s two channels. No decision
//! about the robot is taken in this file, which is the point: the logic that could be wrong is
//! the logic that is tested, and this is the part that needs a radio.
//!
//! It uses `bluer`'s **callback model**, and the alternative was tried on hardware and does not
//! work. `bluer`'s IO model answers BlueZ's `WriteValue` and `StartNotify` with `NotSupported` —
//! it serves only the `AcquireWrite`/`AcquireNotify` fd paths — and a CoreBluetooth central drove
//! the ordinary methods. The result was a robot that advertised, accepted a connection, accepted a
//! subscription, accepted a write, and delivered none of it to this file: no `central connected`
//! line, no pairing prompt, and a client timing out against a service that was working.
//!
//! The IO model was chosen for a benefit that turns out not to exist. It reports
//! `device_address()` on both halves, which looked necessary for pairing a subscription to the
//! session that should feed it — but `bluer` holds **one** `CharacteristicNotifyState` per
//! characteristic, so there is only ever one notification session to pair with. One central at a
//! time is a property of the stack, not a shortcut taken here.
//!
//! So: one session for the service's lifetime, one notify pump, and a write callback that pushes
//! bytes into it.
//!
//! **Untested against hardware.** It type-checks for aarch64 and has never met a real central.
//! Treat what follows as intent until someone connects a phone.

use std::sync::Arc;
use std::time::Duration;

use bluer::adv::Advertisement;
use bluer::agent::{Agent, ReqError as AgentError};
// Two distinct error types with the same name: one for the pairing agent, one for a
// characteristic. Aliased so a mix-up is a name error rather than a puzzling type error.
use bluer::gatt::local::ReqError as GattError;
use bluer::gatt::local::{
    Application, Characteristic, CharacteristicNotify, CharacteristicNotifyMethod,
    CharacteristicRead, CharacteristicWrite, CharacteristicWriteMethod, Service,
};
use futures::FutureExt;
use tokio::sync::{Mutex, mpsc};

use crate::gatt::{RPC_UUID, SERVICE_UUID};
use crate::link::Link;
use crate::pairing;
use crate::session;
use crate::upstream::Sockets;

/// Notification payload assumed for outbound chunks.
///
/// The write side learns the negotiated MTU (BlueZ reports it per request); the notify side has no
/// way to ask. So chunks are sized for 20 bytes — the payload every BLE link is required to
/// support — which is slower than necessary on a good link and correct on every link.
const FLOOR_MTU: usize = 20;

/// How long to wait between attempts to find a usable adapter.
///
/// Measured on the board: `hci0` does not exist until roughly 73 seconds after power-on —
/// `aic-bluetooth.service` attaches the AIC8800's UART late, and `bluetooth.service` itself
/// spends 26s blocked behind `dbus`. A daemon that exited on "no adapter" would be restarted by
/// systemd into the same emptiness for over a minute, so it waits. Same lesson as `robotd`
/// waiting for the motor bus rather than giving up on it.
const ADAPTER_RETRY: Duration = Duration::from_secs(5);

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
    // Left open rather than gated behind a window: the PIN carries what a window would add, as
    // long as it is per-robot. See `crate::pairing` for why that was chosen over a button.
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
                                Err(AgentError::Rejected)
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

    // One session for the service's lifetime, and one set of channels feeding it. Created before
    // the application is registered so a central quick off the mark cannot arrive first.
    //
    // Centrals subscribe *before* they write — that is the order every client uses, including
    // `btctl` — so the notify callback must have somewhere to read from before any write has
    // happened. Pre-creating the channels is what makes the order irrelevant.
    let (link, inbound, from_session) = Link::pair(FLOOR_MTU, "central");
    tokio::spawn(session::run(link, sockets));

    // Handed to the notify callback when a central subscribes, and handed back when it goes away,
    // so a reconnecting central can subscribe again.
    let outbound = Arc::new(Mutex::new(Some(from_session)));

    let app = Application {
        services: vec![Service {
            uuid: SERVICE_UUID,
            primary: true,
            characteristics: vec![Characteristic {
                uuid: RPC_UUID,
                // A read whose only job is to force a bond before anything is written.
                //
                // §7 requires the characteristic carrying wifi credentials to be paired and
                // encrypted, and `encrypt_authenticated_write` below does require that — but
                // nothing *triggers* the pairing. A central subscribes (which needs no
                // encryption, because bluer 0.17 has no flag for it), then writes, and the write
                // is refused on an unpaired link. On macOS the refusal produced no prompt and no
                // error: the client simply waited out its timeout against a working robot.
                //
                // A read is acknowledged, so an unpaired central gets "insufficient
                // authentication" and CoreBluetooth starts pairing there and then. Requiring it
                // on the read is the only encryption trigger bluer exposes for a subscribe-then-
                // write flow: `CharacteristicNotify` carries no encryption flags.
                //
                // The value returned matters far less than the fact that reading it needs a bond;
                // the API version is simply the most useful byte available, and a client that
                // finds a version it does not know can say so before writing anything.
                read: Some(CharacteristicRead {
                    read: true,
                    encrypt_authenticated_read: require_pairing,
                    fun: Box::new(|_req| {
                        async move { Ok(vec![duck_ipc_proto::API_VERSION as u8]) }.boxed()
                    }),
                    ..Default::default()
                }),
                write: Some(CharacteristicWrite {
                    write: true,
                    // Write-without-response as well: a chunked request needs no ATT
                    // acknowledgement per chunk. A client that wants a *refusal* to be visible —
                    // "insufficient authentication" on an unpaired link — must use the
                    // acknowledged form, which is why `btctl` does.
                    write_without_response: true,
                    // §7: anything carrying wifi credentials must travel over a paired, encrypted
                    // link, and `net.connect` does. `encrypt_authenticated_write` demands a bond
                    // made with passkey entry rather than just-works, which is what makes the PIN
                    // mean something — see `crate::pairing`.
                    encrypt_authenticated_write: require_pairing,
                    // **No `.await` between receiving a chunk and enqueueing it.** BlueZ
                    // dispatches each `WriteValue` as its own task, so a yield point here lets
                    // two chunks swap places — and a reordered chunk corrupts a request silently.
                    // Chunk 2 of 3 arriving last produced
                    // `{"id":1,"jsonrpc":"2.info","params":{}}`: valid JSON, missing a field, and
                    // a parse error that blamed the client. `try_send` is synchronous, so arrival
                    // order is preserved.
                    method: CharacteristicWriteMethod::Fun(Box::new(move |value, req| {
                        let inbound = inbound.clone();
                        let bytes = value.len();
                        let result = match inbound.try_send(value) {
                            Ok(()) => Ok(()),
                            Err(mpsc::error::TrySendError::Full(_)) => {
                                // Refusing the write is recoverable — the client can send the
                                // whole request again. Dropping the chunk would not be: the line
                                // would reassemble into something that parses as the wrong thing.
                                tracing::warn!(
                                    peer = %req.device_address,
                                    "inbound queue full; refusing the write"
                                );
                                Err(GattError::Failed)
                            }
                            Err(mpsc::error::TrySendError::Closed(_)) => {
                                tracing::error!("the session task has ended; refusing writes");
                                Err(GattError::Failed)
                            }
                        };
                        async move {
                            tracing::debug!(
                                peer = %req.device_address,
                                mtu = req.mtu,
                                bytes,
                                ok = result.is_ok(),
                                "write"
                            );
                            result
                        }
                        .boxed()
                    })),
                    ..Default::default()
                }),
                notify: Some(CharacteristicNotify {
                    notify: true,
                    method: CharacteristicNotifyMethod::Fun(Box::new(move |mut notifier| {
                        let outbound = outbound.clone();
                        async move {
                            tokio::spawn(async move {
                                let Some(mut chunks) = outbound.lock().await.take() else {
                                    // bluer keeps one notify state per characteristic, so this is
                                    // a second central while the first still holds the session.
                                    // Refusing is the honest answer: sharing one reassembly buffer
                                    // between two clients would interleave their requests.
                                    tracing::warn!(
                                        "another central is already subscribed; only one at a \
                                         time is supported"
                                    );
                                    return;
                                };
                                tracing::info!("central subscribed");

                                while let Some(chunk) = chunks.recv().await {
                                    if let Err(e) = notifier.notify(chunk).await {
                                        tracing::debug!(error = %e, "notify failed; central gone");
                                        break;
                                    }
                                }

                                // Give the receiver back, or a central that reconnects would find
                                // the slot empty and be refused for this daemon's whole life.
                                *outbound.lock().await = Some(chunks);
                                tracing::info!("central unsubscribed");
                            });
                        }
                        .boxed()
                    })),
                    ..Default::default()
                }),
                ..Default::default()
            }],
            ..Default::default()
        }],
        ..Default::default()
    };
    let _app = adapter.serve_gatt_application(app).await?;

    tracing::info!("GATT application registered; waiting for a central");

    // The advertisement and application handles deregister on drop, so this task must outlive
    // the service.
    std::future::pending::<()>().await;
    Ok(())
}
