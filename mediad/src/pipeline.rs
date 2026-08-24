//! The GStreamer pipeline, and the datachannel the control channel rides on.
//!
//! Linux only, the way `padd`'s evdev tap is: the daemon runs on the robot, and everything else in
//! this crate — [`crate::route`], [`crate::session`], [`crate::upstream`] — is portable and stays
//! testable on a laptop. Gating here rather than behind a feature keeps `cargo test` honest on both.
//!
//! ## Shape
//!
//! ```text
//! videotestsrc | camera  →  mpph264enc  →  h264parse  →  webrtcsink
//!                                                            │
//!                                          run-signalling-server=true
//!                                                            │
//!                                          consumer-added → "control" datachannel
//! ```
//!
//! **`webrtcsink` runs the signalling server in this process** (`run-signalling-server`, with
//! `signalling-server-host` and `-port`), so the separate `gst-webrtc-signalling-server` binary
//! never has to be built or shipped — what we ship from that upstream is a `.so`.
//! `remote-webrtc.md` §3.
//!
//! **The encoder is fed pre-encoded H.264**, which `webrtcsink` accepts on its sink pad, so the
//! encoder never reaches negotiation. The four properties that are decisions rather than defaults
//! are set here and explained in `media-bringup.md`: `profile=baseline` — which produces a stream
//! `h264parse` reports as `constrained-baseline`, WebRTC's interoperable floor — and
//! `header-mode=each-idr`, without which SPS/PPS appear in the first frame only and a peer that
//! joins late decodes nothing.
//!
//! ## A test pattern before a camera
//!
//! The default source is `videotestsrc`. That is not a placeholder for want of a better idea: it is
//! the source that works on a board with no camera attached, which is most of them, and it makes
//! the whole session — signalling, negotiation, the datachannel, the control API — exercisable
//! without the capture path existing. The camera arrives as a different source element behind the
//! same encoder, and `media-bringup.md` records why capture cannot simply be `v4l2src`.
//!
//! ## What is not verified
//!
//! The signal wiring below has never run. It is written from `webrtcsink`'s properties and from
//! `reachy_mini`'s working equivalent, and the first board run is its first test — so every step
//! that can fail says which one it was, and the daemon refuses to start rather than running half a
//! pipeline.

use std::sync::Arc;

use anyhow::{Context, Result, anyhow};
use gstreamer as gst;
use gstreamer::prelude::*;
use gstreamer_webrtc as gst_webrtc;
use tokio::sync::mpsc;

/// Where the video comes from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Source {
    /// A test pattern. Works with no camera attached, which is what makes a session testable
    /// before the capture path exists.
    Test,
    /// The head camera, through the rkisp capture path.
    ///
    /// Not implemented yet, and deliberately an error rather than a silent fallback to `Test`: a
    /// robot streaming a test pattern when someone asked for its camera is worse than one that
    /// says it cannot.
    Camera,
}

/// What a peer's control channel needs to talk to [`crate::session::run`].
pub struct Channel {
    /// Lines the peer sent.
    pub inbound: mpsc::Receiver<String>,
    /// Lines to send the peer.
    pub outbound: mpsc::Sender<String>,
}

/// Build and start the pipeline. Returns it, plus a stream of control channels — one per peer.
///
/// The pipeline is returned rather than kept here so the caller owns its lifetime: dropping it
/// stops the session, which is what a shutdown should do.
pub fn start(
    source: Source,
    host: &str,
    port: u32,
    bitrate: u32,
) -> Result<(gst::Pipeline, mpsc::Receiver<Channel>)> {
    gst::init().context("gstreamer would not initialise")?;

    let pipeline = gst::Pipeline::new();

    let src = match source {
        Source::Test => make("videotestsrc")?,
        Source::Camera => {
            return Err(anyhow!(
                "the camera source is not implemented yet — see docs/project/media-bringup.md \
                 for the capture path, which cannot simply be v4l2src on this driver"
            ));
        }
    };
    // `is-live` so the pipeline behaves like a camera does rather than racing ahead of the clock.
    src.set_property("is-live", true);

    // The one element that is not in Debian. If it is missing the message has to say why, because
    // "no element mpph264enc" has three separate causes and only one of them is a missing package:
    // see `media-bringup.md` on the plugin, its two libraries, and /dev/mpp_service's group.
    let enc = gst::ElementFactory::make("mpph264enc")
        .build()
        .map_err(|_| {
            anyhow!(
                "no mpph264enc. Either the plugin is absent (run setup-gstreamer.sh), or its \
             libraries are (librockchip-mpp1, librga2 — ldd the plugin), or /dev/mpp_service is \
             root-only and the encoders silently did not register. GST_PLUGIN_PATH must also \
             include /usr/local/lib/gstreamer-1.0."
            )
        })?;
    // WebRTC's interoperable floor, and the SPS/PPS repetition without which a late peer decodes
    // nothing. `media-bringup.md` has the measurements behind both.
    enc.set_property_from_str("profile", "baseline");
    enc.set_property_from_str("header-mode", "each-idr");
    enc.set_property("bps", bitrate);

    let parse = make("h264parse")?;

    let sink = gst::ElementFactory::make("webrtcsink")
        .build()
        .map_err(|_| {
            anyhow!(
                "no webrtcsink. It comes from gst-plugins-rs, which Debian packages in no suite — \
             setup-gstreamer.sh installs it from the microduck-gst-plugins release, and \
             GST_PLUGIN_PATH must include /usr/local/lib/gstreamer-1.0."
            )
        })?;
    sink.set_property("run-signalling-server", true);
    sink.set_property("signalling-server-host", host);
    sink.set_property("signalling-server-port", port);

    let (channels_tx, channels_rx) = mpsc::channel::<Channel>(4);
    wire_consumers(&sink, channels_tx)?;

    pipeline
        .add_many([&src, &enc, &parse, &sink])
        .context("could not add elements to the pipeline")?;
    gst::Element::link_many([&src, &enc, &parse, &sink]).context(
        "could not link src → mpph264enc → h264parse → webrtcsink. A caps failure here is \
         usually the encoder's sink pad: it takes NV12 and friends, not whatever the source \
         negotiated.",
    )?;

    pipeline
        .set_state(gst::State::Playing)
        .context("the pipeline would not start")?;

    tracing::info!(host, port, ?source, "signalling server listening");
    Ok((pipeline, channels_rx))
}

fn make(name: &str) -> Result<gst::Element> {
    gst::ElementFactory::make(name)
        .build()
        .map_err(|_| anyhow!("no {name} element; a GStreamer package is missing"))
}

/// Give every consumer a `control` datachannel, and hand its ends to the caller.
///
/// The robot creates the channel rather than waiting for the peer to, which is what
/// `reachy_mini`'s working equivalent does. It means a peer that connects and creates nothing
/// still gets a control surface.
fn wire_consumers(sink: &gst::Element, channels: mpsc::Sender<Channel>) -> Result<()> {
    let channels = Arc::new(channels);
    sink.connect("consumer-added", false, move |values| {
        // (webrtcsink, peer_id, webrtcbin). A signature change upstream shows up here as a
        // warning naming what arrived, rather than a panic in a signal handler.
        let Some(webrtcbin) = values.get(2).and_then(|v| v.get::<gst::Element>().ok()) else {
            tracing::warn!(
                arity = values.len(),
                "consumer-added did not carry a webrtcbin; cannot open a control channel"
            );
            return None;
        };
        let peer = values
            .get(1)
            .and_then(|v| v.get::<String>().ok())
            .unwrap_or_else(|| "?".into());

        match open_control_channel(&webrtcbin, &peer) {
            Ok(channel) => {
                // A full queue means nobody is accepting sessions, which is a bug rather than
                // backpressure — say so instead of blocking a GStreamer signal handler.
                if channels.try_send(channel).is_err() {
                    tracing::error!(peer, "no room for another control channel");
                }
            }
            Err(e) => tracing::error!(peer, error = %e, "could not open a control channel"),
        }
        None
    });
    Ok(())
}

/// Create the `control` datachannel on one peer's `webrtcbin` and bridge it to channels.
fn open_control_channel(webrtcbin: &gst::Element, peer: &str) -> Result<Channel> {
    // Reliable and ordered, which is the default and is what §2 wants for `control` —
    // `remote-webrtc.md` §6 covers why the first version opens only this one.
    // Typed as `WebRTCDataChannel` rather than `glib::Object`, and that is load-bearing rather
    // than tidy: a `GstObject` is `Send`, a bare `glib::Object` is not, so the writer task below
    // does not compile against the untyped form.
    let channel = webrtcbin
        .emit_by_name::<Option<gst_webrtc::WebRTCDataChannel>>(
            "create-data-channel",
            &[&"control", &None::<gst::Structure>],
        )
        .ok_or_else(|| anyhow!("webrtcbin returned no data channel"))?;

    let (inbound_tx, inbound) = mpsc::channel::<String>(64);
    let (outbound, mut outbound_rx) = mpsc::channel::<String>(64);

    let peer_label = peer.to_owned();
    channel.connect("on-message-string", false, move |values| {
        if let Some(line) = values.get(1).and_then(|v| v.get::<String>().ok()) {
            // Dropping a control frame is bad, but blocking a GStreamer signal handler is worse:
            // it would stall the whole pipeline, media included.
            if inbound_tx.try_send(line).is_err() {
                tracing::warn!(peer = %peer_label, "dropped a control frame; the session is behind");
            }
        }
        None
    });

    // The writer half. `send-string` is called from this task rather than from the session, so
    // nothing in the session has to know about GStreamer.
    let writer = channel.clone();
    let peer_label = peer.to_owned();
    tokio::spawn(async move {
        while let Some(line) = outbound_rx.recv().await {
            writer.emit_by_name::<()>("send-string", &[&line]);
        }
        tracing::debug!(peer = %peer_label, "control channel writer ended");
    });

    tracing::info!(peer, "control channel open");
    Ok(Channel { inbound, outbound })
}
