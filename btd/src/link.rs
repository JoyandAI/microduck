//! The seam between the radio and everything worth testing.
//!
//! Deliberately **not a trait**. A `GattLink` trait would need an async `recv` and an async
//! `send`, and the session loop has to wait on both at once — which means either splitting the
//! link into halves with associated types, or fighting the borrow checker inside a `select!`.
//! Two channels and a plain struct express the same thing with none of that, and the test
//! constructs one by hand rather than implementing anything.
//!
//! So a backend's whole job is: accept a connection, feed inbound chunks into `inbound`, write
//! whatever appears on `outbound` back to the central, and drop the channels when the central
//! goes away. What happens between those two channels is [`crate::session`], and it never
//! learns whether a radio is involved.

use tokio::sync::mpsc;

/// How many chunks may queue in either direction before a backend has to slow down.
///
/// Small on purpose. BLE is slow, so a deep buffer would mostly serve to hide a stalled peer
/// and delay noticing it; the update progress stream is advisory anyway — `updaterd` already
/// drops a subscriber that cannot keep up rather than applying backpressure to an update.
pub const QUEUE: usize = 32;

/// One connected central.
pub struct Link {
    /// Chunks written by the central to the `request` characteristic, in arrival order.
    /// Closed when it disconnects.
    pub inbound: mpsc::Receiver<Vec<u8>>,
    /// Chunks to notify on the `response` characteristic.
    pub outbound: mpsc::Sender<Vec<u8>>,
    /// Usable notification payload — `ATT_MTU - 3` — as negotiated for this connection.
    ///
    /// Read once per session rather than per message: a central may renegotiate, but not
    /// mid-line, and re-reading it would let a line be chunked two different ways.
    pub mtu: usize,
    /// The central's address, for the log line. Never used for authorization — a BLE address
    /// is trivially spoofed, and pairing is what authorizes (`architecture.md` §4.2).
    pub peer: String,
}

impl Link {
    /// A link wired to channels the caller drives. Used by tests and by `--fake`.
    pub fn pair(
        mtu: usize,
        peer: impl Into<String>,
    ) -> (Self, mpsc::Sender<Vec<u8>>, mpsc::Receiver<Vec<u8>>) {
        let (to_robot, inbound) = mpsc::channel(QUEUE);
        let (outbound, from_robot) = mpsc::channel(QUEUE);
        (
            Self {
                inbound,
                outbound,
                mtu,
                peer: peer.into(),
            },
            to_robot,
            from_robot,
        )
    }
}
