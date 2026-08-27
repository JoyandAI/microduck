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

    /// Where the console is served. `http://<robot>:8080/`, and nothing else to run.
    ///
    /// **Two ports, and only this one is ever typed.** `webrtcsink` owns the listener on `--port`
    /// and takes only a host and a port about it, so the page cannot be a route on it — see
    /// `webrtc-console.md` §1.3, which also says where this ends up: one port, our own signalling
    /// server, and a certificate, on the day a microphone or a browser gamepad is wanted.
    #[arg(long, default_value_t = 8080)]
    web_port: u16,

    /// Target video bitrate, bits per second.
    ///
    /// Explicit rather than the encoder's "auto calculate": `rc-mode` is already constant-bitrate,
    /// which is what a lossy link wants, and leaving the rate unset is how a stream comes out
    /// fifty times under what anyone expected.
    #[arg(long, default_value_t = 2_000_000)]
    bitrate: u32,

    /// Stream the head camera instead of a test pattern.
    #[arg(long)]
    camera: bool,

    /// Which capture node. rkisp exposes several; `video0` is the main path.
    #[arg(long, default_value = "/dev/video0")]
    camera_device: String,

    /// Sensor exposure in lines (~19 µs each) and analogue gain, where 256 is 1x.
    ///
    /// The starting values only: with the driver's boot values the picture is black rather than
    /// merely dark, so something must write the sensor before the first frame. On a board where
    /// `scripts/setup-rkaiq.sh` installed the 3A engine, it converges exposure from here; on one
    /// where it did not, these are what the camera keeps. The defaults are the prototype's.
    #[arg(long, default_value_t = 600)]
    exposure: u32,

    #[arg(long, default_value_t = 1024)]
    analogue_gain: u32,

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

    /// How far the camera is mounted from upright, clockwise: 0, 90, 180 or 270.
    ///
    /// **90, because the head camera is mounted a quarter turn off**, and this is the one place that
    /// fact is written down. It no longer means "rotate the pixels": it is told to whoever displays
    /// the video, and they rotate for free — the console with a CSS transform on the GPU. Rotating
    /// here cost 145% of a core and 22 fps; `pipeline::Rotation` has the numbers.
    #[arg(long, default_value_t = 90)]
    rotate: u32,

    /// Leave the exposure where `--exposure` and `--analogue-gain` put it, instead of metering.
    ///
    /// **The software loop is on by default because the board's 3A engine only does this once.**
    /// `rkaiq_3A_server` owns white balance, gamma and noise reduction, and its AE converges the
    /// sensor at stream start and then stops responding — and skips even that on a boot where it
    /// missed the stream-start event, which is the "3A stopped working" shape. `mediad::exposure`
    /// is the loop. Turn it off for a fixed exposure: a calibration capture, or a board whose
    /// engine really does keep converging.
    #[arg(long)]
    no_auto_exposure: bool,

    /// Rotate in the pipeline as well, so the *encoded stream* comes out upright.
    ///
    /// **Off by default because it is expensive in a way that does not look like rotation.** It
    /// breaks `mpph264enc`'s zero-copy path to the SoC's 2D engine, so MPP converts every frame in
    /// software: measured at 97 °C, the CPU throttled to 408 MHz and 8 fps out of a 30 fps camera.
    /// Worth it only for a consumer that cannot rotate for itself.
    #[arg(long)]
    flip_in_pipeline: bool,
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

    // Refused before anything starts: a bad angle is a typo on a command line, and the daemon
    // should say so rather than opening a camera first.
    // Validated even when the pipeline will not use it, because it is still what every consumer is
    // told about the mount — a typo should not reach the console as a rotation nobody can apply.
    let mount = match mediad::pipeline::Rotation::from_degrees(args.rotate) {
        Ok(rotation) => rotation,
        Err(e) => {
            tracing::error!(error = %e, "mediad cannot start");
            return ExitCode::FAILURE;
        }
    };
    let rotation = if args.flip_in_pipeline {
        tracing::warn!(
            degrees = args.rotate,
            "--flip-in-pipeline: rotating in the pipeline costs the encoder its zero-copy path"
        );
        mount
    } else {
        mediad::pipeline::Rotation::None
    };

    runtime.block_on(async move {
        // The console, before the pipeline: it is the page that says a robot's pipeline would not
        // start, so it should be up first — and it needs nothing from GStreamer.
        //
        // **A page that cannot be served does not cost the video.** A refused bind is almost always
        // a port already in use, which `Restart=always` cannot fix by trying again; a robot that
        // streams and answers control calls with no console is much better than one that does
        // neither. So this is logged at error and the daemon carries on.
        let page = mediad::web::page(args.port);
        let (web_host, web_port) = (args.host.clone(), args.web_port);
        tokio::spawn(async move {
            if let Err(e) = mediad::web::serve(&web_host, web_port, page).await {
                tracing::error!(
                    error = %format!("{e:#}"),
                    "the console is not being served; video and control are unaffected"
                );
            }
        });

        // Before the pipeline, because `webrtcsink`'s `meta` is set as the element is built and a
        // producer that registered without a name would keep it until this daemon restarts. Costs a
        // unix-socket round trip on a boot where `configd` may not be up yet, which is why it is
        // bounded and why a failure is a warning rather than an exit.
        let producer =
            mediad::producer::Producer::learn(Default::default(), duck_ipc_proto::build_info!())
                .await;
        tracing::info!(
            name = producer.name.as_deref().unwrap_or("unknown"),
            release = %producer.release,
            api_version = producer.api_version,
            "producing as"
        );

        let source = if args.camera {
            mediad::pipeline::Source::Camera(mediad::pipeline::Camera {
                device: args.camera_device.clone(),
                exposure: args.exposure,
                analogue_gain: args.analogue_gain,
            })
        } else {
            mediad::pipeline::Source::Test
        };

        let settings = mediad::pipeline::Settings {
            host: args.host.clone(),
            port: args.port,
            bitrate: args.bitrate,
            width: args.width,
            height: args.height,
            fps: args.fps,
            rotation,
        };

        // `frames` is the raw tap off the tee: the auto-exposure loop meters it, and the
        // `get_frame` surface in `architecture.md` §5.3 is what the rest of it is for. The branch
        // runs from the start rather than being added later, because a tee inserted into a live
        // pipeline is a different and much harder problem than a tee that was always there.
        let (_pipeline, mut channels, frames) =
            match mediad::pipeline::start(source.clone(), &producer, &settings) {
                Ok(started) => started,
                Err(e) => {
                    // The message names which step failed and what usually causes it — a missing
                    // plugin, a missing library, or a device node nobody can open. Those look
                    // identical from a log line that only says "failed".
                    tracing::error!(error = %format!("{e:#}"), "mediad cannot start");
                    return ExitCode::FAILURE;
                }
            };

        // After the pipeline, because it meters the pipeline's own frames — and only with a real
        // camera, since a test pattern has no sensor to write and the loop would spend the daemon's
        // life reporting that it cannot.
        //
        // `_exposure` is the handle that stops the thread; it lives as long as this scope, which is
        // as long as the daemon.
        let _exposure = match (&source, args.no_auto_exposure) {
            (mediad::pipeline::Source::Camera(camera), false) => Some(mediad::exposure::spawn(
                camera.device.clone(),
                frames.clone(),
                camera.exposure,
                camera.analogue_gain,
            )),
            (mediad::pipeline::Source::Camera(_), true) => {
                tracing::info!(
                    exposure = args.exposure,
                    analogue_gain = args.analogue_gain,
                    "--no-auto-exposure: the picture stays at the starting exposure"
                );
                None
            }
            (mediad::pipeline::Source::Test, _) => None,
        };

        // What every peer is told about the picture. The geometry is the *encoded* frame — the
        // pipeline does not rotate, so it is the capture geometry — and the rotation is the mount.
        let video = mediad::session::Video {
            width: args.width,
            height: args.height,
            rotate: args.rotate,
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
            // Pushed as a courtesy for a client that only listens — and it may well arrive before
            // the peer's datachannel is open, in which case it is dropped. The console *asks*
            // (`media.video`), which is why that path exists and this one is best-effort.
            {
                let to_peer = channel.outbound.clone();
                let line = mediad::session::video_notification(video);
                tokio::spawn(async move {
                    let _ = to_peer.send(line).await;
                });
            }

            tokio::spawn(mediad::session::run(
                channel.inbound,
                channel.outbound,
                pool,
                video,
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
