//! `configd` — wifi and the robot's identity.
//!
//! It exists because **config must be reachable when `robotd` is dead** (`architecture.md`
//! §3.1): provisioning wifi is exactly what a client needs when things are broken, so it cannot
//! live in the control daemon. And because `btd` **owns nothing** (§4.1) — if this lived in the
//! BLE service, an SDK would absurdly have to go through Bluetooth to set a robot's name.
//!
//! So it is a fifth service, deliberately: one socket, four `net.*` methods and three
//! `system.*` ones, and every transport — BLE, `robotctl`, later `mediad`'s remote gateway —
//! is a client of it rather than a reimplementation.
//!
//! **It stores no credentials.** NetworkManager owns those, persists them root-only and
//! reconnects on its own; `configd` hands a passphrase over and forgets it. What it does own is
//! a small [`store`] file: the robot's name, and whatever identity follows.
//!
//! It runs as root, which is not the default this repo prefers. The reason is narrow: logind's
//! `Reboot` is polkit-gated, there is no polkit on the board, and a session-less non-root caller
//! is therefore denied. The trust boundary is still in the right place — `btd` is the process
//! parsing bytes from anyone in radio range, and it is unprivileged; `configd` only ever sees
//! typed JSON from a peer-credentialled local socket. See `systemd/configd.service` for the
//! sandbox that goes with it.

pub mod net;
#[cfg(target_os = "linux")]
pub mod nm;
pub mod power;
pub mod store;
