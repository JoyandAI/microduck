//! The GStreamer pipeline, and the datachannel the control channel rides on.
//!
//! Linux only, the way `padd`'s evdev tap is: the daemon runs on the robot, and everything else in
//! this crate — [`crate::route`], [`crate::session`], [`crate::upstream`] — is portable and stays
//! testable on a laptop. Gating here rather than behind a feature keeps `cargo test` honest on both.
//!
//! ## Shape
//!
//! ```text
//!                                    ┌─ queue ─ webrtcsink ── encoder-setup → mpph264enc
//! videotestsrc | camera ─ NV12 ─ tee ┤              │
//!                                    └─ queue ─     │  run-signalling-server=true
//!                                       (leaky)     │
//!                                          │        └─ consumer-added → "control" channel
//!                                      appsink
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
//! **`webrtcsink` owns the encoder, and is handed raw video.** It briefly did not — the pipeline
//! was `mpph264enc ! h264parse ! webrtcsink`, which worked and quietly gave up two things: with
//! pre-encoded input `webrtcsink` cannot reach the encoder, so its congestion control cannot adapt
//! the bitrate to the link, and a peer's PLI cannot produce a keyframe, leaving a viewer that lost
//! one broken until the next periodic GOP.
//!
//! That costs a software `videoconvert ! videoscale` in front of any encoder `webrtcsink` does not
//! recognise, and it does not recognise `mpph264enc` — so **the plugin we ship carries a patch**
//! adding that arm. Without the patch this arrangement is slower than pre-encoding rather than
//! faster; the two belong together. See `patches/` in `pollen-robotics/microduck-gst-plugins`.
//!
//! The encoder settings survive through `encoder-setup` — see [`wire_encoder_setup`], which is
//! also where a fallback to software encoding gets noticed.
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

use anyhow::{Context, Result, anyhow, bail};
use gstreamer as gst;
use gstreamer::prelude::*;
use gstreamer_app as gst_app;
use gstreamer_video as gst_video;
use gstreamer_webrtc as gst_webrtc;
use tokio::sync::mpsc;

/// Where the video comes from.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Source {
    /// A test pattern. Works with no camera attached, which is what makes a session testable
    /// before the capture path exists.
    Test,
    /// The head camera, through the rkisp capture path.
    Camera(Camera),
}

/// The head camera, and the two things it will not work without.
///
/// **There is no 3A daemon on this platform.** Nothing converges exposure or gain, so a capture
/// with the driver's boot defaults comes out black — this is not tuning, it is the difference
/// between a picture and no picture. Values are in the sensor's own units: exposure in lines
/// (~19 µs each) and analogue gain where 256 is 1x, up to 2816 for 11x.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Camera {
    pub device: String,
    pub exposure: u32,
    pub analogue_gain: u32,
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
    // GStreamer's own log has to be bridged before `init`, or the first thing it says is lost.
    bridge_gstreamer_log();

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

    let src = match &source {
        Source::Test => {
            let src = make("videotestsrc")?;
            // `is-live` so the pipeline behaves like a camera does rather than racing ahead of
            // the clock. A camera is live by construction and needs no such property.
            src.set_property("is-live", true);
            src
        }
        Source::Camera(camera) => camera_source(camera, fps)?,
    };

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

    // ── the video branch ────────────────────────────────────────────────────
    //
    // Its own queue, so this branch runs on its own thread. Without one, `tee` pushes to both
    // branches from a single thread and whichever is slower holds up the other.
    let video_queue = make("queue")?;

    // **Raw video in, and `webrtcsink` owns the encoder.** This used to be
    // `mpph264enc ! h264parse ! webrtcsink`, which worked and gave up two things quietly: with
    // pre-encoded input `webrtcsink` cannot reach the encoder, so its congestion control cannot
    // adapt the bitrate to the link, and a peer's PLI cannot produce a keyframe — a viewer that
    // loses one stays broken until the next periodic GOP.
    //
    // Handing it raw video costs a software `videoconvert ! videoscale` in front of whatever
    // encoder it picks, unless it knows the encoder. It does not know `mpph264enc`, so the
    // plugin we ship carries a patch adding that arm — see `patches/` in
    // pollen-robotics/microduck-gst-plugins. Without it this is *slower* than pre-encoding, not
    // faster, so the two changes belong together.
    //
    // Which encoder it picks is by rank: `mpph264enc` registers at primary+1 (257), above
    // `x264enc`. Worth confirming with `GST_DEBUG=webrtcsink:4` rather than trusting, because the
    // failure mode is a robot quietly encoding in software.
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

    // Offer H.264 and nothing else. Left alone `webrtcsink` proposes everything it can encode:
    // `mppvp8enc`, `mpph265enc` and `mpph264enc` on the VPU, but `vp9enc` and `av1enc` in
    // *software*. A browser preferring AV1 would have this robot software-encoding AV1 on four
    // Cortex-A55s, which is not a degraded stream but a dead control loop.
    //
    // **No `profile` field here, deliberately.** `webrtcsink` reads one off these caps and does
    // `H264_PROFILES_COMPAT.iter().position(..).expect("Unsupported H264 profile")` — a panic, in
    // a plugin, for a value it does not know. Omitting the field skips that path, and the profile
    // is set on the encoder itself in `wire_encoder_setup` where it belongs.
    //
    // This restriction was held back for a while because H.264 was missing from the offer, and
    // restricting to a codec that fails discovery leaves *no* codecs. The cause was
    // `mpph264enc`'s pad template omitting `constrained-baseline`, which is the one profile
    // `webrtcsink`'s discovery pass demands; the plugins release carries a patch for it from `v3`.
    // If a robot on older plugins reaches here it now fails loudly — no producer at all — rather
    // than quietly serving VP8.
    sink.set_property("video-caps", gst::Caps::builder("video/x-h264").build());

    // The starting bitrate. `webrtcsink` moves it from here as congestion control learns the
    // link — which is the whole point of letting it own the encoder, so this is a starting
    // point rather than the setting it was when we encoded ourselves.
    sink.set_property("start-bitrate", bitrate);

    // The encoder settings, applied through the hook that exists for it.
    //
    // Handing `webrtcsink` the encoder would otherwise *lose* them, which would make this change a
    // regression rather than an improvement: `profile` defaults to High and `header-mode` to
    // first-frame, and both matter — see `wire_encoder_setup`.
    wire_encoder_setup(&sink)?;

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
            &video_queue,
            &sink,
            &raw_queue,
            appsink.upcast_ref(),
        ])
        .context("could not add elements to the pipeline")?;

    gst::Element::link_many([&src, &capsfilter, &tee]).context(
        "could not link the source to the tee. A caps failure here means the source cannot \
         produce NV12 at the requested size and rate.",
    )?;
    gst::Element::link_many([&video_queue, &sink])
        .context("could not link the video queue to webrtcsink")?;
    gst::Element::link_many([&raw_queue, appsink.upcast_ref()])
        .context("could not link the raw branch to its appsink")?;

    // `tee`'s source pads are request pads: they do not exist until asked for, which is why these
    // two links are separate from the `link_many` chains above.
    link_tee_branch(&tee, &video_queue).context("could not attach the video branch to the tee")?;
    link_tee_branch(&tee, &raw_queue).context("could not attach the raw branch to the tee")?;

    // **Watch the bus, or every media failure is silent.**
    //
    // This was learned the hard way. `webrtcsink` drops a codec whose discovery pipeline fails
    // with nothing more than `gst::warning!` — "We don't consider this fatal, as long as we end up
    // with one potential codec" — and a consumer pipeline that dies posts an ERROR to the bus.
    // Neither reaches `tracing`, so the journal showed a session starting, a session ending, and
    // no reason for either. Two rounds of guessing went into diagnosing something GStreamer was
    // already saying out loud.
    watch_bus(&pipeline);

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
                // **Mapping a camera buffer merges its planes.** rkisp hands out `NM12` — two
                // non-contiguous planes — so this copies rather than borrowing. That is correct
                // here because neither plane is padded (`bytesperline` is 1280 for both, and
                // 921600 + 460800 is exactly tight NV12), so the merged block is what `Frame`
                // promises. A driver that padded a stride would need `VideoFrameRef` and a
                // per-row copy instead; the `GstVideoMeta` this branch now receives is where that
                // stride would be read from.
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

/// Send GStreamer's own log into `tracing`, so the journal shows what it says.
///
/// **The bus is not enough.** `webrtcsink` drops a codec whose discovery pipeline fails with a
/// `gst::warning!` and nothing else — "We don't consider this fatal, as long as we end up with one
/// potential codec for each input stream" — and that goes to GStreamer's debug log, not the bus. So
/// a robot offering VP8 instead of H.264 said nothing at all about why, and it had been saying it
/// the whole time to a log nobody was reading.
///
/// `GST_DEBUG` is honoured if set, so raising a category still works the usual way. Unset, the
/// threshold is `WARNING`: enough to catch a codec being dropped or an element refusing, quiet
/// enough for a journal.
fn bridge_gstreamer_log() {
    if std::env::var_os("GST_DEBUG").is_none() {
        // SAFETY: single-threaded here — this runs before `gst::init` and before any task is
        // spawned, which is the only point at which setting an env var is sound.
        unsafe { std::env::set_var("GST_DEBUG", "*:WARNING") };
    }

    // Otherwise every message is printed to stderr by GStreamer *and* logged by us, which in a
    // journal is the same line twice with different formatting.
    gst::log::remove_default_log_function();

    // This is called from arbitrary GStreamer threads and from C, so — as everywhere in this file
    // — it must not panic. It formats and forwards, and nothing else.
    gst::log::add_log_function(|category, level, file, _function, line, object, message| {
        let text = message.get().unwrap_or_default();
        let src = object
            .map(|o| o.to_string())
            .unwrap_or_else(|| "-".to_string());
        let cat = category.name();
        match level {
            gst::DebugLevel::Error => {
                tracing::error!(target: "gst", %cat, %src, %file, line, "{text}")
            }
            gst::DebugLevel::Warning => {
                tracing::warn!(target: "gst", %cat, %src, %file, line, "{text}")
            }
            gst::DebugLevel::Fixme | gst::DebugLevel::Info => {
                tracing::info!(target: "gst", %cat, %src, "{text}")
            }
            _ => tracing::debug!(target: "gst", %cat, %src, "{text}"),
        }
    });
}

/// Forward what the pipeline says about itself into the journal.
///
/// A dedicated thread rather than `bus.add_watch`, which needs a GLib main loop this daemon does
/// not run, and rather than a tokio task, because `timed_pop` blocks.
fn watch_bus(pipeline: &gst::Pipeline) {
    let Some(bus) = pipeline.bus() else {
        tracing::warn!("the pipeline has no bus; media failures will be silent");
        return;
    };
    std::thread::Builder::new()
        .name("gst-bus".into())
        .spawn(move || {
            // `None` blocks until a message arrives; the loop ends when the bus is flushed on
            // teardown, which is the daemon exiting.
            while let Some(msg) = bus.timed_pop(gst::ClockTime::NONE) {
                let src = msg
                    .src()
                    .map(|s| s.path_string().to_string())
                    .unwrap_or_else(|| "?".into());
                match msg.view() {
                    gst::MessageView::Error(e) => {
                        // `debug` carries the element's own detail, which is usually the part that
                        // names the actual cause — a caps mismatch, a device that would not open.
                        tracing::error!(
                            %src,
                            error = %e.error(),
                            detail = e.debug().unwrap_or_default().as_str(),
                            "pipeline error"
                        );
                    }
                    gst::MessageView::Warning(w) => {
                        tracing::warn!(
                            %src,
                            warning = %w.error(),
                            detail = w.debug().unwrap_or_default().as_str(),
                            "pipeline warning"
                        );
                    }
                    // Everything else is state changes and stream status at a rate nobody wants in
                    // a journal — visible with GST_DEBUG when it is wanted.
                    _ => {}
                }
            }
            tracing::debug!("bus watch ended");
        })
        .map(|_| ())
        .unwrap_or_else(
            |e| tracing::warn!(error = %e, "no bus watch thread; failures will be silent"),
        );
}

fn make(name: &str) -> Result<gst::Element> {
    gst::ElementFactory::make(name)
        .build()
        .map_err(|_| anyhow!("no {name} element; a GStreamer package is missing"))
}

/// The head camera as a `v4l2src`, with the one adjustment this driver needs.
///
/// `v4l2src` rather than a hand-written V4L2 loop. The case for our own capture was that this
/// driver drops every third frame, and that raw bytes through `fdsrc` need
/// `rawvideoparse blocksize=…`, which is silently wrong the moment stride padding appears. Both
/// belong to the *subprocess* shape: `v4l2src` attaches a `GstVideoMeta` describing the real
/// layout, and the frame loss has a cause with a small fix — see [`raise_capture_buffers`].
fn camera_source(camera: &Camera, fps: u32) -> Result<gst::Element> {
    pin_sensor_mode(fps)?;

    // Exposure and gain go through `extra-controls` rather than a `v4l2-ctl` call, so they are
    // applied by whoever opens the device — including after a re-open we did not initiate.
    let controls = gst::Structure::builder("c")
        .field("exposure", camera.exposure as i32)
        .field("analogue_gain", camera.analogue_gain as i32)
        .build();

    let src = gst::ElementFactory::make("v4l2src")
        .property("device", &camera.device)
        .property("extra-controls", &controls)
        .build()
        .map_err(|_| {
            anyhow!(
                "no v4l2src element; it comes from gstreamer1.0-plugins-good, which \
                 setup-gstreamer.sh installs"
            )
        })?;

    raise_capture_buffers(&src)?;

    tracing::info!(
        device = %camera.device,
        exposure = camera.exposure,
        analogue_gain = camera.analogue_gain,
        "head camera"
    );
    Ok(src)
}

/// How many capture buffers to ask for. Three is the cliff; four leaves one spare.
///
/// Measured with `v4l2-ctl --stream-mmap=N`, 300 frames of 1280x720 NV12 off rkisp:
///
/// | buffers | 2 | 3 | 4 | 6 |
/// |---|---|---|---|---|
/// | seconds | 15.2 | 10.3 | 10.3 | 10.3 |
///
/// 19.7 fps against 29.2 from a 30 fps sensor, and `v4l2src` lands on two.
const CAPTURE_BUFFERS: u32 = 4;

/// Get `v4l2src` off two capture buffers, which costs a third of the frames.
///
/// **Two things are needed and neither works alone**, which is what made this take three
/// attempts. `gst_v4l2_object_decide_allocation` has two branches:
///
/// ```text
/// can_share_own_pool = (has_video_meta || !obj->need_video_meta);
/// ...
/// if (pushing_from_our_pool)
///     own_min = min + obj->min_buffers + 2;      // honours the query's `min`
/// else
///     own_min = MAX (obj->min_buffers + 1, GST_V4L2_MIN_BUFFERS (obj));   // ignores it
/// ```
///
/// rkisp offers a two-plane, non-contiguous `NM12` alongside single-plane `NV12` — both map to
/// GStreamer's `NV12`, and `v4l2src` picks `NM12`. Only a `GstVideoMeta` can describe that, so
/// `need_video_meta` is true, and with nothing downstream advertising the meta
/// `can_share_own_pool` is false. That takes the **else** branch, where the query's `min` is
/// discarded and the driver's own `min_buffers` is used — and rkisp does not implement
/// `V4L2_CID_MIN_BUFFERS_FOR_CAPTURE`, so that is **0**. `MAX(0 + 1, 2)` is 2, which the pool
/// reports as "increasing minimum buffers to 2", and the debug log then shows it cycling `ix=0`,
/// `ix=1` forever.
///
/// So: the meta unlocks the branch that reads `min`, and the pool is what puts a useful `min`
/// there. Asking for one without the other changes nothing measurable — verified both ways.
///
/// The cost of `update = TRUE` (which a pool in the query sets) is that copy-at-threshold is
/// switched off. That mechanism exists to stop a shallow pool starving, and at six buffers
/// (`4 + 0 + 2`) there is nothing for it to rescue.
fn raise_capture_buffers(src: &gst::Element) -> Result<()> {
    let pad = src
        .static_pad("src")
        .context("v4l2src has no src pad, which cannot happen")?;

    // Logged once, at INFO, because three versions of this probe changed nothing measurable and
    // "the probe never fired" and "GStreamer ignored what it added" want completely different
    // fixes. Whichever it is, the next run says so instead of being inferred from a frame rate.
    let reported = std::sync::atomic::AtomicBool::new(false);

    pad.add_probe(gst::PadProbeType::QUERY_DOWNSTREAM, move |_, info| {
        if let Some(gst::PadProbeData::Query(query)) = info.data.as_mut()
            && let gst::QueryViewMut::Allocation(allocation) = query.view_mut()
        {
            let pools_before = allocation.allocation_pools().len();
            let had_meta = allocation
                .find_allocation_meta::<gst_video::VideoMeta>()
                .is_some();

            if !had_meta {
                allocation.add_allocation_meta::<gst_video::VideoMeta>(None);
            }
            // Size 0 because `decide_allocation` overwrites it with the driver's own frame size
            // for every io-mode we can end up in; max 0 means unlimited.
            if pools_before == 0 {
                allocation.add_allocation_pool(
                    None::<&gst::BufferPool>,
                    0,
                    CAPTURE_BUFFERS,
                    0,
                );
            }

            if !reported.swap(true, std::sync::atomic::Ordering::Relaxed) {
                tracing::info!(
                    pools_before,
                    had_meta,
                    pools_after = allocation.allocation_pools().len(),
                    asked_for = CAPTURE_BUFFERS,
                    "allocation query seen; capture buffers requested"
                );
            }
        }
        gst::PadProbeReturn::Ok
    })
    .context("could not add the allocation probe to v4l2src")?;
    Ok(())
}

/// Switch the IMX219 out of its boot mode, which caps capture at 21 fps.
///
/// The sensor boots in 3280x2464 and the rkisp scaler will happily give us 1280x720 from it — at
/// the full-res frame rate. 1920x1080 is the mode that runs at 30, and the ISP scales down from
/// there, so nothing else in the pipeline changes with it.
///
/// This shells out to `media-ctl` once at startup, because the switch is a subdev ioctl on an
/// entity whose name embeds its I2C bus and address (`m00_b_imx219 2-0010`) and therefore has to
/// be discovered from the topology rather than named. Doing it here rather than in the unit means
/// a `--camera`-less run needs no camera at all.
fn pin_sensor_mode(fps: u32) -> Result<()> {
    let (media, entity) = find_sensor()?;

    let format = format!("\"{entity}\":0[fmt:SRGGB10_1X10/1920x1080]");
    let output = std::process::Command::new("media-ctl")
        .args(["-d", &media, "--set-v4l2", &format])
        .output()
        .context("could not run media-ctl; it comes from v4l-utils")?;

    if !output.status.success() {
        // Not fatal: capture still works, just slower. Said loudly because a third of the frames
        // going missing looks like a network problem from the far end.
        tracing::warn!(
            %media, %entity,
            why = %String::from_utf8_lossy(&output.stderr).trim(),
            "media-ctl would not set the 1920x1080 sensor mode — capture stays in the boot \
             mode, which caps it at 21 fps"
        );
    } else {
        tracing::info!(%media, %entity, target_fps = fps, "sensor mode 1920x1080");
    }
    Ok(())
}

/// The media device and entity name of the IMX219, from the topology.
///
/// Matched on a substring rather than a fixed name: the entity is `m00_b_imx219 2-0010`, which
/// embeds the I2C bus and address, and those move with the overlay.
///
/// **Every way this fails says which one it was.** An earlier version returned `Option` and
/// reported "no imx219 entity" for all of them, which sent the first real run chasing the
/// overlay when the actual cause was `media-ctl` being denied `/dev/media0`. The three cases want
/// three different fixes and look identical from the outside.
fn find_sensor() -> Result<(String, String)> {
    let mut nodes = 0;
    let mut failures = Vec::new();

    for index in 0..8 {
        let media = format!("/dev/media{index}");
        if !std::path::Path::new(&media).exists() {
            continue;
        }
        nodes += 1;

        let output = match std::process::Command::new("media-ctl")
            .args(["-d", &media, "-p"])
            .output()
        {
            Ok(output) => output,
            Err(err) => {
                failures.push(format!("{media}: cannot run media-ctl ({err})"));
                continue;
            }
        };
        if !output.status.success() {
            let why = String::from_utf8_lossy(&output.stderr);
            failures.push(format!("{media}: {}", why.trim()));
            continue;
        }

        for line in String::from_utf8_lossy(&output.stdout).lines() {
            // "- entity 76: m00_b_imx219 2-0010 (1 pad, 1 link, 0 routes)"
            let line = line.trim_start();
            if !line.starts_with("- entity") || !line.contains("imx219") {
                continue;
            }
            let Some((_, rest)) = line.split_once(": ") else {
                continue;
            };
            let name = rest.split(" (").next().unwrap_or(rest).trim();
            if !name.is_empty() {
                return Ok((media, name.to_string()));
            }
        }
    }

    if nodes == 0 {
        bail!(
            "no /dev/media* at all, so no camera is attached as far as the kernel is concerned.\n  \
             The overlay is enabled by setup-board.sh's configure_camera and needs a reboot; \
             Armbian ships it unprefixed while the board sets overlay_prefix=rk3568, so a boot \
             with no camera and no complaint is the expected shape of that bug."
        );
    }
    if !failures.is_empty() {
        bail!(
            "found {nodes} media device(s) and could not read the topology of any:\n  {}\n  \
             /dev/media* is root:video, so this is what running outside the `video` group looks \
             like. The unit grants it with SupplementaryGroups=, which `sudo -u` does not apply — \
             use `systemctl` or `systemd-run -p SupplementaryGroups=video`.",
            failures.join("\n  ")
        );
    }
    bail!(
        "read {nodes} media device(s) and none has an imx219 entity. The overlay loaded something, \
         so DUCK_CAMERA_OVERLAY may name the wrong module for this camera."
    )
}

/// Configure each encoder `webrtcsink` builds, before it runs.
///
/// `webrtcsink` emits `encoder-setup` once per encoder — per consumer, plus one for the discovery
/// pass it uses to work out caps — with the element in hand. It is the only place these can be set
/// now that it owns the encoder rather than us.
///
/// Both settings are measured, and both fail in ways that do not look like encoder settings:
///
/// - **`profile=baseline`** produces a stream `h264parse` reports as `constrained-baseline`, which
///   is WebRTC's interoperable floor (`profile-level-id 42e01f`). The default is High: current
///   browsers negotiate it, older peers do not.
/// - **`header-mode=each-idr`** repeats SPS/PPS on every IDR. The default puts them in the first
///   frame only, so a peer that joins late — or loses that one packet — never decodes anything.
///
/// Returns `false`, so `webrtcsink` still applies its own configuration on top: it owns the
/// bitrate now, and congestion control moving it is the reason for this whole arrangement.
fn wire_encoder_setup(sink: &gst::Element) -> Result<()> {
    if glib::subclass::signal::SignalId::lookup("encoder-setup", sink.type_()).is_none() {
        return Err(anyhow!(
            "webrtcsink has no encoder-setup signal; without it the encoder cannot be configured \
             and the stream would be High profile with SPS/PPS only in its first frame"
        ));
    }

    sink.connect("encoder-setup", false, move |values| {
        // (webrtcsink, consumer_id, stream_name, encoder).
        let Some(encoder) = values.get(3).and_then(|v| v.get::<gst::Element>().ok()) else {
            tracing::warn!(
                arity = values.len(),
                "encoder-setup did not carry an encoder; it will run unconfigured"
            );
            return Some(false.to_value());
        };
        let name = encoder
            .factory()
            .map(|f| f.name().to_string())
            .unwrap_or_default();

        // `"discovery"` for the startup pass in which `webrtcsink` builds one encoder per codec it
        // could offer, purely to learn its caps. A real peer id otherwise.
        let consumer = values
            .get(1)
            .and_then(|v| v.get::<String>().ok())
            .unwrap_or_default();
        let discovering = consumer == "discovery";

        // Only `mpph264enc` has these properties, and setting a property an element lacks panics —
        // which in a signal handler aborts. So this is keyed on the factory rather than attempted
        // hopefully.
        if name == "mpph264enc" {
            encoder.set_property_from_str("profile", "baseline");
            encoder.set_property_from_str("header-mode", "each-idr");
            if !discovering {
                tracing::info!(encoder = %name, %consumer, "hardware H.264, configured for WebRTC");
            }
        } else if !discovering {
            // Only meaningful for a real consumer. During discovery this fires once per codec —
            // including `mppvp8enc` and `mpph265enc`, which are *hardware* — so warning there
            // called two VPU encoders software on every startup, crying wolf about the one thing
            // it exists to catch.
            //
            // For a real peer it is worth saying loudly: `video-caps` restricts the offer to
            // H.264, so anything else arriving here means that restriction stopped working, and
            // something is encoding on the cores `robotd`'s control loop shares.
            tracing::warn!(
                encoder = %name, %consumer,
                "a consumer negotiated something other than hardware H.264"
            );
        }
        Some(false.to_value())
    });
    Ok(())
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
