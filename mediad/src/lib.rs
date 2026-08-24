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
//! There is no binary yet, and no GStreamer. Both arrive together, because the moment this crate
//! links GStreamer, CI's `board` job needs `scripts/cross-sysroot.sh` rather than the multiarch
//! libudev in `ci-cross-deps.sh` — which is its own change and belongs beside the code that needs
//! it.

pub mod route;
pub mod session;
pub mod upstream;
