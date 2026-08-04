//! The GATT contract: the UUIDs a client must know.
//!
//! Platform-independent on purpose. These live here rather than in [`crate::bluez`] because they
//! are part of the wire contract, exactly like the method names in `duck-ipc-proto` — the robot
//! serves them and every client must look for them, so a client that cannot compile for Linux
//! still needs them. `examples/btctl.rs`, which runs on a laptop, is the case that proves it.
//!
//! Random v4 UUIDs rather than anything derived: they are ours, and they must not change once an
//! app has shipped against them. Written out in full so that grepping for a value finds this
//! comment.

/// The robot's service. What a client scans for.
pub const SERVICE_UUID: uuid::Uuid = uuid::uuid!("6f5d2a10-3b47-4c8e-9a1f-2d7e8c4b6019");

/// Central → robot. NDJSON request bytes, chunked, written here.
pub const REQUEST_UUID: uuid::Uuid = uuid::uuid!("6f5d2a11-3b47-4c8e-9a1f-2d7e8c4b6019");

/// Robot → central. NDJSON response and notification bytes, chunked, notified here.
pub const RESPONSE_UUID: uuid::Uuid = uuid::uuid!("6f5d2a12-3b47-4c8e-9a1f-2d7e8c4b6019");

#[cfg(test)]
mod tests {
    use super::*;

    /// Distinct, and stable forever once an app ships against them. A copy-paste making two of
    /// them equal would present one characteristic and hang a client waiting for the other.
    #[test]
    fn the_uuids_are_distinct() {
        let all = [SERVICE_UUID, REQUEST_UUID, RESPONSE_UUID];
        for (i, a) in all.iter().enumerate() {
            for b in &all[i + 1..] {
                assert_ne!(a, b);
            }
        }
    }
}
