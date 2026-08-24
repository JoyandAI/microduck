//! `mediad` — camera, mic, WebRTC, and the remote gateway.
//!
//! What exists so far is the control channel, end to end but for the channel itself:
//!
//! - [`route`] — which calls a WebRTC peer may make. `remote-webrtc.md` §5.
//! - [`upstream`] — connections to the five services that own the answers, one per (service, lane).
//! - [`session`] — the pipe. Lines in, lines out, replies never parsed.
//!
//! [`session::run`] is transport-agnostic on purpose: it takes lines and gives lines, so it is
//! testable without a WebRTC peer and would serve a WebSocket surface (§11) unchanged.
//!
//! [`pipeline`] is the rest, and the only part that is not portable: `webrtcsink` with the
//! signalling server in this process, `mpph264enc` in front of it, and a `control` datachannel per
//! peer wired to [`session::run`].

pub mod route;
pub mod session;
pub mod upstream;

/// The GStreamer pipeline and the datachannel. Linux only — see the crate manifest for why the
/// gate is by target rather than by feature.
#[cfg(target_os = "linux")]
pub mod pipeline;
