//! `btd` — the BLE front door onto the robot's API.
//!
//! **A transport adapter and nothing else** (`architecture.md` §4.1). `btd` owns no state, and
//! that is load-bearing rather than tidy: if provisioning or config lived here, every other
//! service would depend on `btd`, and an SDK would absurdly have to go through Bluetooth to
//! set a robot's name.
//!
//! So the design is a pipe. A GATT service with two characteristics — write `request`, notify
//! `response` — carries **the same NDJSON JSON-RPC lines as every other transport**, and `btd`
//! reassembles them, checks the method against [`route`]'s table, forwards them verbatim to
//! the owning service's unix socket, and chunks the replies back. Adding a method to the
//! protocol needs no change here beyond one line in that table.
//!
//! It is also the process that parses bytes from anyone in radio range, which is why it runs
//! unprivileged while `configd` — which only ever sees typed JSON from a peer-credentialled
//! local socket — is the one running as root. Putting the parser on the safe side of that
//! boundary matters more than hardening the dispatcher.
//!
//! ## What exists today
//!
//! The transport-independent half: [`framing`] and [`route`], both tested without a radio.
//! The GATT backend and the daemon binary follow — and BLE on this board is not usable until
//! roughly 73s after power-on (`aic-bluetooth.service` brings `hci0` up late), so whatever
//! drives the adapter must wait and retry for one rather than assume it exists at startup.
//!
//! `net.*` and `system.*` — wifi, name, reboot — are not in [`route`] yet because `configd`
//! does not exist yet. When it does they are one arm each, and nothing else here changes.

pub mod framing;
pub mod route;
