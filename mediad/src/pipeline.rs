//! The GStreamer pipeline, and the datachannel the control channel rides on.
//!
//! Linux only, the way `padd`'s evdev tap is: the daemon runs on the robot, and everything else in
//! this crate — [`crate::route`], [`crate::session`], [`crate::upstream`] — is portable and stays
//! testable on a laptop. Gating here rather than behind a feature keeps `cargo test` honest on both.
//!
//! ## Shape
//!
//! ```text
//!                                    ┌─ queue ─ mpph264enc ─ h264parse ─ webrtcsink
//! videotestsrc | camera ─ NV12 ─ tee ┤                                        │
//!                                    └─ queue ─ appsink        run-signalling-server=true
//!                                       (leaky)                               │
//!                                          │              consumer-added → "control" channel
//!                                     latest frame
//! ```
//!
//! **The tee is on raw NV12, before the encoder**, and that placement is the point of it.
//! `architecture.md` §5.3 wants a frame on demand for a server-side program — "it wants a frame
//! every second or two plus a state blob", not a 30 fps H.264 track to decode — and §2 wants
//! perception next to the sensor, deriving features rather than shipping pixels to `robotd`. Both
//! need pixels, and taking them off the encoded branch would mean decoding what we just encoded.
//!
//! NV12 because that is what the rkisp capture path emits and what `mpph264enc` takes, so nothing
//! converts anywhere: no `videoconvert`, and no RGA pass, between capture and either consumer.
//!
//! **Each branch has its own `queue`, and the raw one is leaky.** A `tee` without queues runs its
//! branches on one thread, so a slow consumer stalls the others — here that would mean a
//! perception consumer pausing the video track. The raw branch drops old frames rather than
//! applying backpressure, which is the semantics `architecture.md` §2 asks for: the *latest*
//! snapshot, non-blocking, last-value-wins. A stalled reader costs frames, never the encoder.
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
//! **Nothing in a signal handler here may panic.** These closures are invoked from C, so a panic
//! does not unwind — it aborts the process, and the journal shows `thread caused non-unwinding
//! panic` with a backtrace through `g_closure_invoke` and nothing about what was actually wrong.
//! The first board run died exactly that way, from `tokio::spawn` on a GStreamer thread that has
//! no runtime. So: the runtime handle is captured where one exists and spawned onto explicitly,
//! and every signal is checked to exist before it is connected or emitted — `emit_by_name` and
//! `connect` both panic on an absent name.
//!
//! What is left of that risk is a signature that exists but differs, which shows up as a warning
//! naming the arity rather than as an abort.

use std::sync::{Arc, Mutex};

use anyhow::{Context, Result, anyhow};
use gstreamer as gst;
use gstreamer::prelude::*;
use gstreamer_app as gst_app;
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

/// One raw frame off the tee, as the last one seen.
#[derive(Debug, Clone)]
pub struct Frame {
    pub width: u32,
    pub height: u32,
    /// NV12, tightly packed as the caps describe it.
    pub data: Vec<u8>,
}

/// The most recent raw frame, or none yet.
///
/// Last-value-wins by construction: the appsink callback replaces whatever was here. A reader that
/// is slow sees a newer frame next time rather than a queue of stale ones, which is what a
/// perception consumer and a `get_frame` both want — and neither can slow the encoder down by
/// being slow itself.
#[derive(Clone, Default)]
pub struct Frames(Arc<Mutex<Option<Frame>>>);

impl Frames {
    /// The latest frame, cloned. `None` until the first one arrives.
    pub fn latest(&self) -> Option<Frame> {
        self.0.lock().expect("frame lock").clone()
    }
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
    width: u32,
    height: u32,
    fps: u32,
) -> Result<(gst::Pipeline, mpsc::Receiver<Channel>, Frames)> {
    gst::init().context("gstreamer would not initialise")?;

    // **A GStreamer signal handler runs on a GStreamer thread, which is not inside the tokio
    // runtime.** `tokio::spawn` there panics with "there is no reactor running", and a panic
    // crossing the C closure boundary is a non-unwinding abort — the whole daemon dies with
    // SIGABRT from inside `g_closure_invoke`, which is exactly what the first board run did. So
    // the handle is captured here, where there *is* a runtime, and the handler spawns onto it.
    let runtime = tokio::runtime::Handle::try_current().context(
        "pipeline::start must be called from inside a tokio runtime: the datachannel writer is \
         spawned onto it from a GStreamer signal thread, which has no runtime of its own",
    )?;

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

    // Pinned rather than negotiated, because both branches of the tee depend on the answer. NV12
    // is what the rkisp capture path emits and what `mpph264enc` takes, so this costs no
    // conversion — and a raw consumer that has to guess the format is a consumer that gets it
    // wrong the first time the source changes.
    let caps = gst::Caps::builder("video/x-raw")
        .field("format", "NV12")
        .field("width", width as i32)
        .field("height", height as i32)
        .field("framerate", gst::Fraction::new(fps as i32, 1))
        .build();
    let capsfilter = gst::ElementFactory::make("capsfilter")
        .property("caps", &caps)
        .build()
        .map_err(|_| anyhow!("no capsfilter element; gstreamer core is incomplete"))?;

    let tee = make("tee")?;

    // ── the encoded branch ──────────────────────────────────────────────────
    //
    // Its own queue, so this branch runs on its own thread. Without one, `tee` pushes to both
    // branches from a single thread and whichever is slower holds up the other.
    let enc_queue = make("queue")?;

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
    wire_consumers(&sink, channels_tx, runtime)?;

    // ── the raw branch ──────────────────────────────────────────────────────
    //
    // Leaky downstream and one buffer deep: when the reader is behind, the *oldest* frame is
    // dropped and the newest kept. That is last-value-wins, and it is what keeps a slow perception
    // consumer from ever becoming the video track's problem.
    let raw_queue = gst::ElementFactory::make("queue")
        .property("max-size-buffers", 1u32)
        .property("max-size-bytes", 0u32)
        .property("max-size-time", 0u64)
        .property_from_str("leaky", "downstream")
        .build()
        .map_err(|_| anyhow!("no queue element; gstreamer core is incomplete"))?;

    let frames = Frames::default();
    let appsink = gst_app::AppSink::builder()
        .caps(&caps)
        // `sync=false` so this branch never waits on the clock: a snapshot wants the newest frame
        // as soon as it exists, and pacing it would only add latency to a consumer that is not
        // rendering anything.
        .sync(false)
        .max_buffers(1)
        .drop(true)
        .build();
    wire_frames(&appsink, frames.clone(), width, height);

    pipeline
        .add_many([
            &src,
            &capsfilter,
            &tee,
            &enc_queue,
            &enc,
            &parse,
            &sink,
            &raw_queue,
            appsink.upcast_ref(),
        ])
        .context("could not add elements to the pipeline")?;

    gst::Element::link_many([&src, &capsfilter, &tee]).context(
        "could not link the source to the tee. A caps failure here means the source cannot \
         produce NV12 at the requested size and rate.",
    )?;
    gst::Element::link_many([&enc_queue, &enc, &parse, &sink]).context(
        "could not link queue → mpph264enc → h264parse → webrtcsink. A caps failure here is \
         usually the encoder's sink pad, which takes NV12 and friends.",
    )?;
    gst::Element::link_many([&raw_queue, appsink.upcast_ref()])
        .context("could not link the raw branch to its appsink")?;

    // `tee`'s source pads are request pads: they do not exist until asked for, which is why these
    // two links are separate from the `link_many` chains above.
    link_tee_branch(&tee, &enc_queue).context("could not attach the encoded branch to the tee")?;
    link_tee_branch(&tee, &raw_queue).context("could not attach the raw branch to the tee")?;

    pipeline
        .set_state(gst::State::Playing)
        .context("the pipeline would not start")?;

    tracing::info!(
        host,
        port,
        ?source,
        width,
        height,
        fps,
        "signalling server listening"
    );
    Ok((pipeline, channels_rx, frames))
}

/// Request a source pad from the tee and link it to a branch's sink pad.
fn link_tee_branch(tee: &gst::Element, branch: &gst::Element) -> Result<()> {
    let src_pad = tee
        .request_pad_simple("src_%u")
        .ok_or_else(|| anyhow!("the tee would not give a source pad"))?;
    let sink_pad = branch
        .static_pad("sink")
        .ok_or_else(|| anyhow!("the branch has no sink pad"))?;
    src_pad
        .link(&sink_pad)
        .map_err(|e| anyhow!("linking a tee branch failed: {e:?}"))?;
    Ok(())
}

/// Keep [`Frames`] pointing at the most recent buffer off the raw branch.
fn wire_frames(appsink: &gst_app::AppSink, frames: Frames, width: u32, height: u32) {
    appsink.set_callbacks(
        gst_app::AppSinkCallbacks::builder()
            .new_sample(move |sink| {
                let sample = sink.pull_sample().map_err(|_| gst::FlowError::Eos)?;
                let Some(buffer) = sample.buffer() else {
                    return Ok(gst::FlowSuccess::Ok);
                };
                let Ok(map) = buffer.map_readable() else {
                    // A buffer that will not map is not worth failing the pipeline over — the next
                    // one is a frame away, and this branch is advisory by design.
                    tracing::debug!("a raw frame would not map");
                    return Ok(gst::FlowSuccess::Ok);
                };
                let frame = Frame {
                    width,
                    height,
                    data: map.as_slice().to_vec(),
                };
                // Replaced, not queued: last-value-wins is the contract.
                *frames.0.lock().expect("frame lock") = Some(frame);
                Ok(gst::FlowSuccess::Ok)
            })
            .build(),
    );
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
fn wire_consumers(
    sink: &gst::Element,
    channels: mpsc::Sender<Channel>,
    runtime: tokio::runtime::Handle,
) -> Result<()> {
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

        match open_control_channel(&webrtcbin, &peer, &runtime) {
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
fn open_control_channel(
    webrtcbin: &gst::Element,
    peer: &str,
    runtime: &tokio::runtime::Handle,
) -> Result<Channel> {
    // `emit_by_name` panics when the signal is absent or its signature differs — and a panic here
    // aborts the process rather than unwinding, because this runs inside a C closure. Checked
    // first so an upstream change becomes a logged refusal to open a control channel, with the
    // video track still working.
    for signal in ["create-data-channel"] {
        if glib::subclass::signal::SignalId::lookup(signal, webrtcbin.type_()).is_none() {
            return Err(anyhow!(
                "webrtcbin has no {signal} signal; gst-plugins-rs may have changed it"
            ));
        }
    }
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

    // Same reasoning for the channel's own signals: `connect` and `emit_by_name` both panic when a
    // name is absent, and both run where a panic aborts. Checked together so the failure is one
    // clear message rather than whichever fires first.
    for signal in ["on-message-string", "send-string"] {
        if glib::subclass::signal::SignalId::lookup(signal, channel.type_()).is_none() {
            return Err(anyhow!(
                "the data channel has no {signal} signal; gst-plugins-rs may have changed it"
            ));
        }
    }

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
    runtime.spawn(async move {
        while let Some(line) = outbound_rx.recv().await {
            writer.emit_by_name::<()>("send-string", &[&line]);
        }
        tracing::debug!(peer = %peer_label, "control channel writer ended");
    });

    tracing::info!(peer, "control channel open");
    Ok(Channel { inbound, outbound })
}
