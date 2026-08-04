//! `btd` — the BLE front door onto the robot's API.
//!
//! Runs only on the robot. See the crate docs in `lib.rs` for what it is and why it owns
//! nothing; this file is argument parsing, logging and startup.

use std::path::PathBuf;
use std::process::ExitCode;

use btd::upstream::Sockets;
use clap::Parser;

#[derive(Parser, Debug)]
#[command(
    version,
    about = "BLE transport adapter for the robot API",
    long_about = "Serves a GATT service that carries the same JSON-RPC lines as every other \
                  transport, forwarding each request to the service that owns it. Exposes a \
                  subset: status, update trigger and progress. Never motor control."
)]
struct Args {
    /// `updaterd`'s socket.
    #[arg(long, default_value = duck_ipc_proto::socket::UPDATER)]
    update_socket: PathBuf,

    /// `robotd`'s socket.
    #[arg(long, default_value = duck_ipc_proto::socket::ROBOT)]
    robot_socket: PathBuf,

    /// `configd`'s socket — wifi and the robot's identity.
    #[arg(long, default_value = duck_ipc_proto::socket::CONFIG)]
    config_socket: PathBuf,

    /// Serve without requiring a paired, encrypted link.
    ///
    /// Bench use only. Without pairing, anyone in radio range can write requests — including
    /// `net.connect`, which carries a wifi passphrase.
    #[arg(long)]
    insecure_no_pairing: bool,

    /// Name to advertise. Defaults to the hostname.
    ///
    /// This is what someone sees in a phone's Bluetooth list. It becomes `system.setName`'s
    /// business once `configd` exists; until then the hostname is at least unique per board.
    #[arg(long)]
    name: Option<String>,
}

/// The first line this daemon writes is its own identity, at `warn` so it survives
/// `RUST_LOG=warn` on a long-running board (`architecture.md` §8.1).
///
/// `exe` earns its place: it says which release directory the process was actually launched
/// from, which is the difference between "the update worked" and "the symlink moved but systemd
/// is still running the old path".
fn log_startup_identity(service: &str) {
    let exe = std::env::current_exe()
        .map(|p| p.display().to_string())
        .unwrap_or_else(|_| "unknown".to_owned());

    tracing::warn!(
        service,
        build = %duck_ipc_proto::build_info!(),
        exe,
        pid = std::process::id(),
        "starting"
    );
}

fn hostname() -> String {
    // /etc/hostname rather than the `hostname` crate or a libc call: one file read, no
    // dependency, and it is what the board is actually configured with.
    std::fs::read_to_string("/etc/hostname")
        .map(|s| s.trim().to_owned())
        .ok()
        .filter(|s| !s.is_empty())
        .unwrap_or_else(|| "robot".to_owned())
}

#[tokio::main]
async fn main() -> ExitCode {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .with_writer(std::io::stderr)
        .init();

    let args = Args::parse();
    log_startup_identity("btd");

    let sockets = Sockets {
        updater: args.update_socket,
        robot: args.robot_socket,
        config: args.config_socket,
    };
    let name = args.name.unwrap_or_else(hostname);

    run(sockets, name, !args.insecure_no_pairing).await
}

#[cfg(target_os = "linux")]
async fn run(sockets: Sockets, name: String, require_pairing: bool) -> ExitCode {
    tokio::select! {
        result = btd::bluez::serve(sockets, name, require_pairing) => match result {
            // `serve` only returns when BlueZ closes the control stream, which means the
            // adapter went away. Exiting non-zero lets systemd restart us into the retry loop
            // rather than leaving a daemon that is advertising nothing.
            Ok(()) => {
                tracing::error!("BLE service ended unexpectedly");
                ExitCode::FAILURE
            }
            Err(e) => {
                tracing::error!(error = %e, "BLE service failed");
                ExitCode::FAILURE
            }
        },
        () = shutdown() => {
            tracing::info!("shutting down");
            ExitCode::SUCCESS
        }
    }
}

/// Off-Linux this daemon has nothing to serve, and says so rather than pretending.
///
/// The crate still builds and tests here, which is the point: `cargo test` on a laptop is the
/// onboarding path, and only the radio is Linux-only.
#[cfg(not(target_os = "linux"))]
async fn run(_sockets: Sockets, _name: String, _require_pairing: bool) -> ExitCode {
    tracing::error!(
        "btd needs BlueZ, which is Linux-only. This binary exists here so the crate builds \
         and its tests run; it cannot serve BLE on this platform."
    );
    ExitCode::FAILURE
}

/// Resolve on SIGTERM (systemd stop) or SIGINT (Ctrl-C).
#[cfg(target_os = "linux")]
async fn shutdown() {
    use tokio::signal::unix::{SignalKind, signal};

    let mut term = match signal(SignalKind::terminate()) {
        Ok(s) => s,
        Err(e) => {
            tracing::warn!(error = %e, "cannot listen for SIGTERM");
            return std::future::pending().await;
        }
    };
    tokio::select! {
        _ = term.recv() => {}
        _ = tokio::signal::ctrl_c() => {}
    }
}
