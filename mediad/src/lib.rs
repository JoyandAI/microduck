//! `mediad` — camera, mic, WebRTC, and the remote gateway.
//!
//! What exists so far is the transport policy: which calls a WebRTC peer may make
//! ([`route`]). `docs/design/remote-webrtc.md` is the design; §5 is this module.
//!
//! There is no binary yet, and no GStreamer. Both arrive together, because the moment this crate
//! links GStreamer, CI's `board` job needs `scripts/cross-sysroot.sh` rather than the multiarch
//! libudev in `ci-cross-deps.sh` — which is its own change and belongs beside the code that needs
//! it.

pub mod route;
