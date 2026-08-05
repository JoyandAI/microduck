//! `btctl` — talk to a robot over BLE from a laptop.
//!
//! The phone app's stand-in, and the only way to test `btd` against a real radio.
//!
//! An **example, not a binary**, so `btleplug` never reaches the robot: examples' dependencies
//! are dev-dependencies, and nothing here is in the shipped artifact. `robotctl` is the tool
//! that ships, and it speaks unix sockets on the robot itself.
//!
//! `btleplug` rather than `bluer`, because this runs on a developer's machine: CoreBluetooth on
//! macOS, BlueZ on Linux, WinRT on Windows. `bluer` would restrict the client to Linux, which
//! defeats the point.
//!
//! It reuses `btd::framing` deliberately. The chunking here is the *client* half of the same
//! module the robot uses, so if the framing were asymmetric this would not work — which makes
//! it a real test of the protocol rather than a reimplementation that could agree with itself.
//!
//! ```text
//! cargo run -p btd --example btctl -- scan
//! cargo run -p btd --example btctl -- status
//! cargo run -p btd --example btctl -- wifi scan
//! cargo run -p btd --example btctl -- wifi connect "Pollen" --psk secret
//! cargo run -p btd --example btctl -- name "Ducky"
//! cargo run -p btd --example btctl -- call robot.health
//! ```

use std::time::Duration;

use btd::framing::{self, Reassembler};
use btd::gatt::{RPC_UUID, SERVICE_UUID};
use btleplug::api::{
    Central, CharPropFlags, Characteristic, Manager as _, Peripheral as _, ScanFilter, WriteType,
};
use btleplug::platform::{Manager, Peripheral};
use clap::{Parser, Subcommand};
use futures::StreamExt;

/// How long to look for a robot before giving up.
///
/// Generous, because BLE discovery is genuinely slow and a robot advertises at whatever interval
/// BlueZ chose. Shorter than this and a laptop that was simply unlucky reports "no robot".
const SCAN_TIME: Duration = Duration::from_secs(8);

/// How long to wait for a reply once the request is written.
///
/// Longer than any single call except `net.connect`, which polls NetworkManager for up to 45s
/// and so gets its own budget below.
const REPLY_TIMEOUT: Duration = Duration::from_secs(15);
const SLOW_REPLY_TIMEOUT: Duration = Duration::from_secs(60);

/// Every step before the first reply gets its own budget and its own message.
///
/// btleplug bounds none of these, so without them a stall anywhere between "found the robot" and
/// "sent a request" prints `connecting to …` and then nothing at all — which says only that
/// something is wrong, not what. Each of connect, discovery and the first read fails differently
/// and wants a different next move, so each says which one it was.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(20);
const DISCOVER_TIMEOUT: Duration = Duration::from_secs(20);
const READ_TIMEOUT: Duration = Duration::from_secs(15);

/// Run one step with a budget, naming it if the budget runs out.
async fn step<T>(
    what: &str,
    hint: &str,
    budget: Duration,
    f: impl std::future::Future<Output = Result<T, btleplug::Error>>,
) -> Result<T, Box<dyn std::error::Error>> {
    match tokio::time::timeout(budget, f).await {
        Ok(Ok(value)) => Ok(value),
        Ok(Err(e)) => Err(format!("{what} failed: {e}\n{hint}").into()),
        Err(_) => Err(format!("{what} timed out after {budget:?}\n{hint}").into()),
    }
}

#[derive(Parser)]
#[command(
    version,
    about = "Talk to a robot over BLE — the phone app's stand-in",
    long_about = "Finds a robot advertising the duck GATT service and speaks the same JSON-RPC \
                  lines every other transport uses. This is a development tool: it is an example \
                  rather than a binary, so it never ships to a robot."
)]
struct Cli {
    /// Connect to this robot by advertised name. Without it, the first one found wins.
    #[arg(long, global = true)]
    name: Option<String>,

    /// Print every line sent and received.
    #[arg(long, global = true)]
    verbose: bool,

    /// The robot's pairing PIN.
    ///
    /// Six digits, shown by `robotctl system pin` on the robot. The factory default is `000000`
    /// and authenticates anyone who has read this repository, which is why a shipped robot needs a
    /// per-robot one.
    #[arg(long, global = true, default_value = "000000")]
    pin: String,

    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// List robots in range, and stop.
    Scan,
    /// Version handshake plus update status.
    Status,
    /// Name, serial and uptime.
    Info,
    /// Is the control loop healthy?
    Health,
    /// Wifi.
    #[command(subcommand)]
    Wifi(Wifi),
    /// Rename the robot.
    Name { name: String },
    /// Reboot it.
    Reboot,
    /// Send any method, for whatever is not wrapped above.
    Call {
        method: String,
        /// Parameters as JSON. Defaults to `{}`.
        params: Option<String>,
    },
}

#[derive(Subcommand)]
enum Wifi {
    /// What the wifi is doing — SSID, signal, addresses.
    Status,
    /// Networks the robot can see.
    Scan,
    /// Join a network.
    Connect {
        ssid: String,
        /// Omit for an open network.
        #[arg(long)]
        psk: Option<String>,
    },
    /// Forget a stored network.
    Forget { ssid: String },
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cli = Cli::parse();

    let manager = Manager::new().await?;
    let adapter = manager
        .adapters()
        .await?
        .into_iter()
        .next()
        .ok_or("no Bluetooth adapter on this machine")?;

    eprintln!("scanning for {SCAN_TIME:?}…");
    // Filter by our service UUID so a busy office does not drown the robot in headphones. Note
    // some platforms ignore the filter and return everything, which is why the name check below
    // still happens.
    adapter
        .start_scan(ScanFilter {
            services: vec![SERVICE_UUID],
        })
        .await?;
    tokio::time::sleep(SCAN_TIME).await;

    // Candidates, most likely first.
    //
    // The advertised service UUID is an *optimisation*, not the identity check — and treating it as
    // the latter broke as soon as the Mac bonded with the robot. CoreBluetooth does not reliably
    // return a paired peripheral from a scan, and for a cached one the advertised service list is
    // often empty, so a robot that had been found every time became invisible. The authoritative
    // test is whether it serves our characteristic, which is only knowable after connecting.
    let mut found: Vec<(Peripheral, String)> = Vec::new();
    let mut fallback: Vec<(Peripheral, String)> = Vec::new();

    for peripheral in adapter.peripherals().await? {
        let Some(properties) = peripheral.properties().await? else {
            continue;
        };
        let name = properties
            .local_name
            .clone()
            .unwrap_or_else(|| properties.address.to_string());

        if properties.services.contains(&SERVICE_UUID) {
            found.push((peripheral, name));
        } else if cli.name.as_deref() == Some(name.as_str()) || peripheral.is_connected().await? {
            // Named explicitly, or already connected — both are strong enough hints to be worth a
            // connection attempt, which will reject it soon enough if it is something else.
            fallback.push((peripheral, name));
        }
    }
    let _ = adapter.stop_scan().await;

    if found.is_empty() && !fallback.is_empty() {
        if cli.verbose {
            eprintln!(
                "no robot advertised the service; trying {} known peripheral(s) — a bonded robot \
                 often stops advertising it to a Mac that has already paired",
                fallback.len()
            );
        }
        found = fallback;
    }

    if found.is_empty() {
        return Err(
            "no robot found. Is btd running, and is the robot in range?\n\
                    If the Mac has paired with it before, it may not appear in a scan: pass \
                    `--name <robot hostname>` to try it by name anyway."
                .into(),
        );
    }

    let (peripheral, name) = match &cli.name {
        Some(wanted) => found
            .into_iter()
            .find(|(_, name)| name == wanted)
            .ok_or_else(|| format!("no robot named {wanted:?} in range"))?,
        None => found.into_iter().next().expect("non-empty"),
    };
    eprintln!("connecting to {name}…");

    step(
        "connecting",
        "The robot advertised but would not accept a connection. If macOS shows it as paired, \
         forget it there and retry; `sudo pkill bluetoothd` also clears a half-finished bond.",
        CONNECT_TIMEOUT,
        peripheral.connect(),
    )
    .await?;
    if cli.verbose {
        eprintln!("connected; discovering services…");
    }

    step(
        "service discovery",
        "Connected, but the robot never described its services. Check `journalctl -u btd -b` on \
         the robot for whether the GATT application is registered.",
        DISCOVER_TIMEOUT,
        peripheral.discover_services(),
    )
    .await?;

    let (request, response) = characteristics(&peripheral)?;

    // Read first, and this is load-bearing rather than a courtesy.
    //
    // The robot requires an authenticated encrypted link to *write*, but a subscribe needs no
    // encryption — so without this a central subscribes happily, has its first write refused, and
    // on macOS sees neither a prompt nor an error. A read is acknowledged, so an unpaired link
    // fails here instead, which is what makes CoreBluetooth start pairing.
    //
    // The value is the robot's API version. Worth checking before sending anything: a client one
    // version ahead can say so rather than have every call refused.
    let read = step(
        "reading the API version",
        "This read requires an encrypted link, so it is what triggers pairing. A hang here usually \
         means the bond did not complete: forget the robot in macOS Bluetooth settings, or run \
         `sudo pkill bluetoothd`, and retry.",
        READ_TIMEOUT,
        peripheral.read(&response),
    )
    .await;

    match read {
        Ok(value) => {
            let theirs = value.first().copied().unwrap_or(0);
            if cli.verbose {
                eprintln!("robot speaks API v{theirs}");
            }
            if u32::from(theirs) != duck_ipc_proto::API_VERSION {
                return Err(format!(
                    "the robot speaks API v{theirs} and this client speaks v{}; \
                     install matching versions",
                    duck_ipc_proto::API_VERSION
                )
                .into());
            }
        }
        Err(e) => return Err(e),
    }

    // Subscribe *before* writing, or a reply can arrive before there is anywhere to put it.
    // btd's session begins on the first write, so the order here is not merely defensive: the
    // notify half has to exist for the session to have somewhere to answer.
    peripheral.subscribe(&response).await?;
    let mut notifications = peripheral.notifications().await?;

    // Prove the PIN before anything else. The bond is just-works, so it encrypts the link and
    // authenticates nobody; the robot serves nothing until this succeeds. See
    // `btd/src/pairing.rs` for why the check is here rather than in the pairing.
    let auth = serde_json::json!({
        "jsonrpc": "2.0",
        "id": 0,
        "method": "system.authenticate",
        "params": { "pin": cli.pin },
    });
    let auth = serde_json::to_string(&auth)?;
    if cli.verbose {
        // The PIN is deliberately not printed, even here: a terminal is a log too.
        eprintln!("→ system.authenticate (pin redacted)");
    }
    write_line(&peripheral, &request, &auth).await?;

    let reply = read_line(&mut notifications, REPLY_TIMEOUT).await?;
    if cli.verbose {
        eprintln!("← {reply}");
    }
    let parsed: serde_json::Value = serde_json::from_str(&reply)?;
    if parsed["result"]["authenticated"] != serde_json::json!(true) {
        let left = parsed["result"]["attempts_remaining"].as_u64();
        return Err(match left {
            Some(0) => "wrong PIN, and no attempts left — the robot closed the session. \
                        Check it with `robotctl system pin` on the robot."
                .to_owned()
                .into(),
            Some(n) => format!(
                "wrong PIN ({n} attempt(s) left). Check it with `robotctl system pin` on the robot."
            )
            .into(),
            None => format!("authentication failed: {reply}").into(),
        });
    }

    let (line, timeout) = request_line(&cli.command)?;
    if cli.verbose {
        eprintln!("→ {line}");
    }

    // Chunked by the same code the robot uses. btleplug does not expose the negotiated MTU, so
    // 20 bytes — the floor every BLE link guarantees — is the safe assumption. Slower than
    // necessary on a good link, and correct on every link.
    for chunk in framing::chunks(&line, 20) {
        peripheral
            .write(&request, &chunk, WriteType::WithoutResponse)
            .await?;
    }

    let mut reassembler = Reassembler::new();
    let deadline = tokio::time::Instant::now() + timeout;

    loop {
        let remaining = deadline.saturating_duration_since(tokio::time::Instant::now());
        if remaining.is_zero() {
            return Err(format!("no reply within {timeout:?}").into());
        }

        let Ok(Some(notification)) = tokio::time::timeout(remaining, notifications.next()).await
        else {
            return Err(format!("no reply within {timeout:?}").into());
        };

        for line in reassembler.push(&notification.value)? {
            if cli.verbose {
                eprintln!("← {line}");
            }
            // Notifications with no `id` are a progress stream, not an answer; print them and
            // keep waiting for the response that closes the call.
            let value: serde_json::Value = serde_json::from_str(&line)?;
            let is_answer = value.get("id").is_some_and(|id| !id.is_null());

            println!("{}", serde_json::to_string_pretty(&value)?);
            if is_answer {
                let _ = peripheral.disconnect().await;
                // A JSON-RPC error is the robot answering, not this tool failing — so it is
                // printed above and reported through the exit status rather than as a panic.
                return if value.get("error").is_some() {
                    Err("the robot returned an error".into())
                } else {
                    Ok(())
                };
            }
        }
    }
}

/// Write one NDJSON line, chunked.
///
/// Chunked by the same code the robot uses. btleplug does not expose the negotiated MTU, so 20
/// bytes — the floor every BLE link guarantees — is the safe assumption: slower than necessary on a
/// good link, correct on every link.
///
/// **Acknowledged writes**, and that is not a detail. An ATT Write *Command* (`WithoutResponse`)
/// carries no reply, so a refusal — for insufficient encryption, say — is invisible: the request
/// silently never arrives and the client waits out its timeout with no idea why. That is exactly
/// how this first behaved, against a robot that was working perfectly.
async fn write_line(
    peripheral: &Peripheral,
    characteristic: &Characteristic,
    line: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    for chunk in framing::chunks(line, 20) {
        peripheral
            .write(characteristic, &chunk, WriteType::WithResponse)
            .await?;
    }
    Ok(())
}

/// Read one complete NDJSON line from the notification stream.
async fn read_line(
    notifications: &mut (impl futures::Stream<Item = btleplug::api::ValueNotification> + Unpin),
    timeout: Duration,
) -> Result<String, Box<dyn std::error::Error>> {
    let mut reassembler = Reassembler::new();
    let deadline = tokio::time::Instant::now() + timeout;

    loop {
        let remaining = deadline.saturating_duration_since(tokio::time::Instant::now());
        if remaining.is_zero() {
            return Err(format!("no reply within {timeout:?}").into());
        }
        let Ok(Some(notification)) = tokio::time::timeout(remaining, notifications.next()).await
        else {
            return Err(format!("no reply within {timeout:?}").into());
        };
        if let Some(line) = reassembler.push(&notification.value)?.into_iter().next() {
            return Ok(line);
        }
    }
}

/// Find the two characteristics, and check they can do what we need.
///
/// Checking the properties rather than assuming them turns a confusing silence — a write that
/// lands nowhere — into a clear message naming which half is wrong.
fn characteristics(
    peripheral: &Peripheral,
) -> Result<(Characteristic, Characteristic), Box<dyn std::error::Error>> {
    let all = peripheral.characteristics();
    let find = |uuid| all.iter().find(|c| c.uuid == uuid).cloned();

    let rpc =
        find(RPC_UUID).ok_or("the robot has no RPC characteristic; is this the right service?")?;

    // One characteristic carries both directions, so it must be able to do both. Checking rather
    // than assuming turns a confusing silence — a write that lands nowhere — into a message
    // naming which half is missing.
    if !rpc
        .properties
        .intersects(CharPropFlags::WRITE | CharPropFlags::WRITE_WITHOUT_RESPONSE)
    {
        return Err("the RPC characteristic is not writable".into());
    }
    if !rpc.properties.contains(CharPropFlags::NOTIFY) {
        return Err("the RPC characteristic cannot notify".into());
    }
    // Cloned rather than borrowed twice: btleplug takes a &Characteristic for each operation.
    Ok((rpc.clone(), rpc))
}

/// One command becomes one JSON-RPC line, plus how long to wait for it.
fn request_line(command: &Command) -> Result<(String, Duration), Box<dyn std::error::Error>> {
    let (method, params, timeout) = match command {
        Command::Scan => unreachable!("handled before connecting"),
        Command::Status => ("update.status", serde_json::json!({}), REPLY_TIMEOUT),
        Command::Info => ("system.info", serde_json::json!({}), REPLY_TIMEOUT),
        Command::Health => ("robot.health", serde_json::json!({}), REPLY_TIMEOUT),
        Command::Name { name } => (
            "system.setName",
            serde_json::json!({ "name": name }),
            REPLY_TIMEOUT,
        ),
        Command::Reboot => ("system.reboot", serde_json::json!({}), REPLY_TIMEOUT),
        Command::Wifi(Wifi::Status) => ("net.status", serde_json::json!({}), REPLY_TIMEOUT),
        // A scan asks NetworkManager to re-scan, which takes seconds on a quiet radio.
        Command::Wifi(Wifi::Scan) => ("net.scan", serde_json::json!({}), SLOW_REPLY_TIMEOUT),
        Command::Wifi(Wifi::Connect { ssid, psk }) => {
            let mut params = serde_json::json!({ "ssid": ssid });
            if let Some(psk) = psk {
                params["psk"] = serde_json::Value::String(psk.clone());
            }
            // configd polls NM for up to 45s before calling a join timed out, so this must wait
            // longer than that or the tool gives up before the robot has decided.
            ("net.connect", params, SLOW_REPLY_TIMEOUT)
        }
        Command::Wifi(Wifi::Forget { ssid }) => (
            "net.forget",
            serde_json::json!({ "ssid": ssid }),
            REPLY_TIMEOUT,
        ),
        Command::Call { method, params } => {
            let params = match params {
                Some(text) => {
                    serde_json::from_str(text).map_err(|e| format!("params must be JSON: {e}"))?
                }
                None => serde_json::json!({}),
            };
            (method.as_str(), params, SLOW_REPLY_TIMEOUT)
        }
    };

    let request = serde_json::json!({
        "jsonrpc": "2.0",
        "id": 1,
        "method": method,
        "params": params,
    });
    Ok((serde_json::to_string(&request)?, timeout))
}
