//! Pairing: who is allowed to talk to this robot over BLE.
//!
//! §4.2 says BLE authorisation is "physical presence + pairing", and §7 requires the
//! characteristic carrying wifi credentials to be paired and encrypted. This is the mechanism
//! for both — and it is deliberately a **process rather than a security guarantee** at this
//! stage.
//!
//! The robot answers BlueZ's passkey request with a six-digit PIN held by `configd`. A phone is
//! prompted for it, and BlueZ refuses the bond if it does not match. Because the pairing is
//! *authenticated* (passkey entry, not just-works), the link is then encrypted and MITM-resistant
//! — which is what makes `encrypt_authenticated_write` on the request characteristic mean
//! something.
//!
//! **The factory PIN is `000000` and everyone can read it in this repository.** So out of the box
//! this proves physical presence and nothing more, which is the same guarantee just-works pairing
//! gives — the difference being that the mechanism, the storage and the six-digit contract are
//! all in place, so making it a real secret is a provisioning change rather than a redesign. A
//! per-robot PIN printed under the robot is what turns this into security, and that is
//! `updater-design.md` §5.7's per-device state.
//!
//! ## No pairing window, and that is decided rather than deferred
//!
//! The robot is pairable whenever it advertises. A physical button-held window is the usual
//! answer, and it was considered and rejected: a **per-robot PIN already carries the property a
//! window would add.** If the PIN is unique and printed under the robot, knowing it requires
//! physical access — and anyone who can read the sticker can also pick the robot up. A window
//! would defend only against someone in range while the factory default is still in place, and
//! the answer to that is a real PIN, not a button.
//!
//! What a button would buy beyond this: a visible consent moment, a recovery path when the PIN is
//! lost, and defence in depth if a sticker is photographed. None is needed for v1, and each is
//! additive later — an enclosure with a button can gate `set_pairable` without changing anything
//! here.
//!
//! So the security of this rests entirely on the PIN being per-robot, which makes it a
//! provisioning obligation rather than a software one: something has to generate it, print it and
//! record what was printed. That is `updater-design.md` §5.7's per-device state — the same slot
//! that owes us a serial number, which is a reason to define it once.
//!
//! Still open, and smaller: **no bond management.** Every paired phone stays paired and nothing
//! revokes one; `bluetoothctl untrust` is the manual escape until there is an API for it.

use std::time::Duration;

use duck_ipc_proto as proto;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::UnixStream;

/// How long to wait for `configd` to answer with the PIN.
///
/// Short: BlueZ is holding a pairing exchange open, and a phone shows a spinner while we decide.
/// If `configd` cannot answer in this long it is not going to.
const PIN_TIMEOUT: Duration = Duration::from_secs(3);

/// Ask `configd` for the pairing PIN.
///
/// Fetched **per pairing request** rather than cached at startup, so `robotctl system set-pin`
/// takes effect on the next pairing rather than the next reboot. One socket round-trip during an
/// exchange that already takes a human several seconds.
///
/// The PIN is never logged. It is barely a secret today, but a per-robot one is meant to be, and
/// the journal is the wrong place for it.
pub async fn pin(config_socket: &std::path::Path) -> Result<u32, String> {
    let result = tokio::time::timeout(PIN_TIMEOUT, fetch(config_socket))
        .await
        .map_err(|_| "configd did not answer in time".to_owned())??;

    // BlueZ wants the passkey as a number. Parsing loses the leading zeros, which is correct
    // here and exactly why the stored form is a string: `000000` is passkey 0, and the phone
    // displays six digits because the *spec* says six, not because we sent them.
    result.pin.parse::<u32>().map_err(|_| {
        format!(
            "configd returned a PIN that is not a number: {} chars",
            result.pin.len()
        )
    })
}

async fn fetch(config_socket: &std::path::Path) -> Result<proto::PairingPinResult, String> {
    let stream = UnixStream::connect(config_socket)
        .await
        .map_err(|e| format!("cannot reach configd at {}: {e}", config_socket.display()))?;
    let (read, mut write) = stream.into_split();
    let mut lines = BufReader::new(read).lines();

    let request = proto::Request::call(proto::Id::Number(1), &proto::Call::SystemPairingPin);
    let mut line = serde_json::to_vec(&request).map_err(|e| e.to_string())?;
    line.push(b'\n');
    write.write_all(&line).await.map_err(|e| e.to_string())?;
    write.flush().await.map_err(|e| e.to_string())?;

    let reply = lines
        .next_line()
        .await
        .map_err(|e| e.to_string())?
        .ok_or("configd closed the connection without answering")?;

    let response: proto::Response = serde_json::from_str(&reply).map_err(|e| e.to_string())?;
    if let Some(error) = response.error {
        return Err(format!("configd refused: {error}"));
    }
    response.result_as().map_err(|e| e.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A PIN with leading zeros must reach BlueZ as the right *number*, since that is the only
    /// form a passkey has on the wire.
    #[test]
    fn a_pin_with_leading_zeros_is_the_right_passkey() {
        for (pin, expected) in [
            ("000000", 0u32),
            ("000042", 42),
            ("123456", 123456),
            ("999999", 999999),
        ] {
            assert_eq!(pin.parse::<u32>().unwrap(), expected, "{pin}");
        }
    }

    /// A missing `configd` must be a reported error rather than a hang: BlueZ is holding a
    /// pairing exchange open, and a phone waiting forever is worse than a refused bond.
    #[tokio::test]
    async fn an_absent_configd_fails_rather_than_hanging() {
        let dir = tempfile::tempdir().unwrap();
        let err = pin(&dir.path().join("absent.sock")).await.unwrap_err();
        assert!(err.contains("cannot reach configd"), "{err}");
    }

    /// The whole path, over a real socket: a fake configd answers and the PIN becomes a passkey.
    #[tokio::test]
    async fn the_pin_is_fetched_over_the_socket() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("configd.sock");
        let listener = tokio::net::UnixListener::bind(&path).unwrap();

        tokio::spawn(async move {
            let (stream, _) = listener.accept().await.unwrap();
            let (read, mut write) = stream.into_split();
            let mut lines = BufReader::new(read).lines();
            let request = lines.next_line().await.unwrap().unwrap();
            // The request must be the PIN method and nothing else.
            assert!(
                request.contains(proto::method::SYSTEM_PAIRING_PIN),
                "{request}"
            );

            let response = proto::Response::ok(
                Some(proto::Id::Number(1)),
                &proto::PairingPinResult {
                    pin: "000042".into(),
                    is_default: false,
                },
            );
            let mut line = serde_json::to_vec(&response).unwrap();
            line.push(b'\n');
            write.write_all(&line).await.unwrap();
            write.flush().await.unwrap();
        });

        assert_eq!(pin(&path).await.unwrap(), 42);
    }

    /// A refusal from `configd` is reported, not swallowed into a default passkey — which would
    /// silently let anyone pair with `000000`.
    #[tokio::test]
    async fn a_refusal_is_not_treated_as_a_default_pin() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("configd.sock");
        let listener = tokio::net::UnixListener::bind(&path).unwrap();

        tokio::spawn(async move {
            let (stream, _) = listener.accept().await.unwrap();
            let (read, mut write) = stream.into_split();
            let mut lines = BufReader::new(read).lines();
            let _ = lines.next_line().await;
            let response = proto::Response::err(
                Some(proto::Id::Number(1)),
                proto::Error::new(proto::code::PERMISSION_DENIED, "nope"),
            );
            let mut line = serde_json::to_vec(&response).unwrap();
            line.push(b'\n');
            write.write_all(&line).await.unwrap();
            write.flush().await.unwrap();
        });

        let err = pin(&path).await.unwrap_err();
        assert!(err.contains("refused"), "{err}");
    }
}
