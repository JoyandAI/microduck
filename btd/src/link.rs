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

/// How many chunks may queue in either direction.
///
/// Sized so that a **maximal inbound line never has to block**, which is a correctness
/// requirement rather than a tuning choice. The radio backend must hand chunks over without
/// awaiting: BlueZ dispatches each write as its own task, so any yield point between receiving a
/// chunk and enqueueing it is a chance for two chunks to swap places — and a reordered chunk
/// silently corrupts a request. Chunk 2 of 3 arriving last once produced
/// `{"id":1,"jsonrpc":"2.info","params":{}}`, which is valid JSON missing a field, and a parse
/// error blaming the client.
///
/// So the queue has to be deep enough that a synchronous `try_send` cannot fail on legitimate
/// traffic: `QUEUE * 20 >= framing::MAX_LINE`, where 20 is the smallest payload BLE guarantees.
/// A test asserts that relationship. Beyond it, a flood gets a clean ATT error rather than a
/// dropped chunk, because failing a write is recoverable and corrupting one is not.
pub const QUEUE: usize = 512;

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

#[cfg(test)]
mod tests {
    use super::*;

    /// The queue must be deep enough that a maximal line never needs a blocking send.
    ///
    /// This is the invariant behind the radio backend using a synchronous `try_send`: it may not
    /// await, because a yield point between receiving a chunk and enqueueing it lets two chunks
    /// swap places, and a reordered chunk corrupts a request rather than failing it. If `MAX_LINE`
    /// grows or `QUEUE` shrinks, this fails here rather than as an occasional mangled request on a
    /// robot.
    #[test]
    fn the_queue_holds_a_maximal_line_at_the_ble_floor() {
        // 20 bytes is the payload every BLE link is required to support, and therefore the
        // smallest chunk size a client may use.
        const FLOOR: usize = 20;
        assert!(
            QUEUE * FLOOR >= crate::framing::MAX_LINE,
            "QUEUE ({QUEUE}) * {FLOOR} must be at least MAX_LINE ({}), or a full-length request \
             can fill the queue and be refused",
            crate::framing::MAX_LINE
        );
    }
}
