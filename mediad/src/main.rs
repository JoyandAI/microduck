//! `mediad` — camera, mic, WebRTC, and the remote gateway.
//!
//! Runs the signalling server in this process, streams video to whoever connects, and gives each
//! peer a `control` datachannel that is a pipe to the robot API. `docs/design/remote-webrtc.md` is
//! the design.
//!
//! ## What it does not do
//!
//! **It does not authenticate.** Anyone who reaches the signalling port can drive the robot and
//! see its camera. That is a decision, not an omission — §4 has the reasoning, and the short
//! version is that the pairing PIN is a shared `000000`, so a gate would add a step to every
//! connection and prove nothing. The bridge that makes a robot reachable from outside the LAN
//! authenticates on both sides before a session arrives.
//!
//! **It is not on the recovery path.** If `mediad` will not start, the robot still walks, still
//! takes an update, and is still reachable over Bluetooth. That is why it may depend on a plugin
//! from a release asset and a device node's group while `updaterd` may not.

use std::process::ExitCode;

use clap::Parser;

#[derive(Parser, Debug)]
#[command(about = "Camera, mic, WebRTC — and the remote gateway", version)]
struct Args {
    /// Where to bind the signalling server.
    ///
    /// All interfaces by default, and that is the point: loopback-only would mean a peer on the
    /// LAN cannot reach it at all and every session would have to go through a bridge, which
    /// defeats having a local mode. See `remote-webrtc.md` §3.
    #[arg(long, default_value = "0.0.0.0")]
    host: String,

    /// The signalling server's port. 8443 is what `webrtcsink`'s own signaller defaults to, so a
    /// client built against it needs no argument.
    #[arg(long, default_value_t = 8443)]
    port: u32,

    /// Target video bitrate, bits per second.
    ///
    /// Explicit rather than the encoder's "auto calculate": `rc-mode` is already constant-bitrate,
    /// which is what a lossy link wants, and leaving the rate unset is how a stream comes out
    /// fifty times under what anyone expected.
    #[arg(long, default_value_t = 2_000_000)]
    bitrate: u32,

    /// Stream the head camera instead of a test pattern. Not implemented yet.
    #[arg(long)]
    camera: bool,

    /// Frame size and rate, pinned rather than negotiated.
    ///
    /// Both branches of the tee depend on the answer — the encoder and whatever reads raw NV12 —
    /// so a consumer that had to guess would get it wrong the first time the source changed.
    /// 1280x720 at 30 is what the hardware encoder was measured at.
    #[arg(long, default_value_t = 1280)]
    width: u32,

    #[arg(long, default_value_t = 720)]
    height: u32,

    #[arg(long, default_value_t = 30)]
    fps: u32,
}

#[cfg(target_os = "linux")]
fn main() -> ExitCode {
    let args = Args::parse();
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .with_writer(std::io::stderr)
        .init();

    // Before anything that can fail, so a journal that reports a startup failure also reports
    // which build failed. Every other daemon does this for the same reason.
    duck_ipc_proto::log_startup_identity!("mediad");

    let runtime = match tokio::runtime::Runtime::new() {
        Ok(runtime) => runtime,
        Err(e) => {
            tracing::error!(error = %e, "no tokio runtime");
            return ExitCode::FAILURE;
        }
    };

    runtime.block_on(async move {
        let source = if args.camera {
            mediad::pipeline::Source::Camera
        } else {
            mediad::pipeline::Source::Test
        };

        // `_frames` is the raw NV12 tap off the tee. Nothing reads it yet — perception and the
        // `get_frame` surface in `architecture.md` §5.3 are what it is for — but the branch runs
        // from the start rather than being added later, because a tee inserted into a live
        // pipeline is a different and much harder problem than a tee that was always there.
        let (_pipeline, mut channels, _frames) = match mediad::pipeline::start(
            source,
            &args.host,
            args.port,
            args.bitrate,
            args.width,
            args.height,
            args.fps,
        ) {
            Ok(started) => started,
            Err(e) => {
                // The message names which step failed and what usually causes it — a missing
                // plugin, a missing library, or a device node nobody can open. Those look
                // identical from a log line that only says "failed".
                tracing::error!(error = %format!("{e:#}"), "mediad cannot start");
                return ExitCode::FAILURE;
            }
        };

        // One session per peer, each with its own connections to the services it talks to. Per
        // peer rather than shared, so one peer's minutes-long update cannot silence another's
        // telemetry — which is the same reason a session keeps one connection per lane.
        while let Some(channel) = channels.recv().await {
            let (replies_tx, mut replies_rx) = tokio::sync::mpsc::channel::<String>(256);
            let pool = mediad::upstream::Pool::new(Default::default(), replies_tx);

            let to_peer = channel.outbound.clone();
            tokio::spawn(async move {
                while let Some(line) = replies_rx.recv().await {
                    if to_peer.send(line).await.is_err() {
                        break;
                    }
                }
            });
            tokio::spawn(mediad::session::run(
                channel.inbound,
                channel.outbound,
                pool,
            ));
        }

        // The pipeline outlived its consumers, which means `webrtcsink` stopped producing them.
        tracing::warn!("no longer accepting peers");
        ExitCode::FAILURE
    })
}

/// `mediad` is a Linux daemon: it drives GStreamer against a Rockchip VPU and a V4L2 capture path.
/// The rest of the crate is portable and its tests run anywhere, which is why this is a stub rather
/// than a `cfg` on the whole crate.
#[cfg(not(target_os = "linux"))]
fn main() -> ExitCode {
    let _ = Args::parse();
    eprintln!("mediad runs on the robot; this host is not Linux");
    ExitCode::FAILURE
}
