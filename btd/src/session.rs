//! One connected central, from `hello` to disconnect.
//!
//! This is the whole of `btd`'s behaviour, and it holds no state about the robot — only about
//! the conversation: a reassembly buffer and whichever upstream sockets this session has had
//! reason to open.
//!
//! A request is forwarded **verbatim**. `btd` parses each line only far enough to answer two
//! questions — is this method allowed here, and which socket owns it — and then passes the
//! original bytes on. It never rewrites `id`, never re-serialises params, and never invents a
//! result. That is what keeps it a transport rather than a second implementation of the API,
//! and it is why adding a protocol method costs one line in [`crate::route`] and nothing here.

use duck_ipc_proto as proto;
use tokio::sync::mpsc;

use crate::framing::{self, Reassembler};
use crate::link::{Link, QUEUE};
use crate::route;
use crate::upstream::{Pool, Sockets};

/// Serve one central until it disconnects or breaks framing.
pub async fn run(mut link: Link, sockets: Sockets) {
    let peer = link.peer.clone();
    tracing::info!(peer = %peer, mtu = link.mtu, "session opened");

    let (replies_tx, mut replies) = mpsc::channel::<String>(QUEUE);
    let mut pool = Pool::new(sockets, replies_tx);
    let mut inbound = Reassembler::new();

    loop {
        tokio::select! {
            // Bytes from the radio.
            chunk = link.inbound.recv() => {
                let Some(chunk) = chunk else { break };

                let lines = match inbound.push(&chunk) {
                    Ok(lines) => lines,
                    Err(e) => {
                        // Framing failures end the session rather than being answered. There
                        // is no id to answer *to* — we never saw a complete request — and a
                        // peer that cannot frame will not be helped by a JSON error it also
                        // cannot parse.
                        tracing::warn!(peer = %peer, error = ?e, "framing failed; closing session");
                        break;
                    }
                };

                for line in lines {
                    if let Some(response) = dispatch(&mut pool, &line).await
                        && send_line(&link, &response).await.is_err()
                    {
                        return;
                    }
                }
            }

            // A reply or a notification from a service.
            line = replies.recv() => {
                // The channel is held by `pool`, which lives as long as this loop, so `None`
                // is unreachable — but treat it as end-of-session rather than panicking.
                let Some(line) = line else { break };
                if send_line(&link, &line).await.is_err() {
                    return;
                }
            }
        }
    }

    if inbound.pending() > 0 {
        // Worth a line: it distinguishes "client finished and left" from "client vanished
        // mid-message", which is the difference between a normal disconnect and a bug.
        tracing::debug!(peer = %peer, pending = inbound.pending(), "session ended mid-line");
    }
    tracing::info!(peer = %peer, "session closed");
}

/// Handle one complete line. Returns a response `btd` must answer itself, if any.
///
/// `None` means the line was forwarded and the upstream will answer — the ordinary path.
async fn dispatch(pool: &mut Pool, line: &str) -> Option<String> {
    let request: proto::Request = match serde_json::from_str(line) {
        Ok(request) => request,
        Err(e) => {
            // No id is recoverable from an unparseable line, so the response carries `null` —
            // which the spec requires and every client already handles.
            return Some(encode(&proto::Response::err(
                None,
                proto::Error::new(proto::code::PARSE_ERROR, e.to_string()),
            )));
        }
    };

    // A notification — no id — expects no reply, so a refused one is dropped silently. It is
    // still not forwarded: the allowlist is not advisory.
    let id = request.id.clone();

    let call = match request.as_call() {
        Ok(call) => call,
        Err(e) => return id.map(|id| encode(&proto::Response::err(Some(id), e))),
    };

    let Some(upstream) = route::upstream_for(&call) else {
        tracing::info!(method = call.method(), "refused over BLE");
        return id.map(|id| encode(&proto::Response::err(Some(id), route::refusal(&call))));
    };

    if let Err(e) = pool.send(upstream, line).await {
        tracing::warn!(method = call.method(), upstream = ?upstream, error = %e, "upstream unreachable");
        // Naming the service is what makes this diagnosable from a phone screenshot: "robotd
        // is not answering" is a different problem from "the robot refused".
        return id.map(|id| {
            encode(&proto::Response::err(
                Some(id),
                proto::Error::new(
                    proto::code::INTERNAL_ERROR,
                    format!("{upstream:?} is not answering: {e}"),
                ),
            ))
        });
    }
    None
}

/// Chunk one line out to the central.
async fn send_line(link: &Link, line: &str) -> Result<(), ()> {
    for chunk in framing::chunks(line, link.mtu) {
        if link.outbound.send(chunk).await.is_err() {
            // The backend dropped its half: the central is gone.
            return Err(());
        }
    }
    Ok(())
}

fn encode(response: &proto::Response) -> String {
    // A Response is plain strings, ints and enums; this cannot fail. If it somehow did,
    // sending nothing would hang the client, so send something it can parse as an error.
    serde_json::to_string(response).unwrap_or_else(|_| {
        r#"{"jsonrpc":"2.0","id":null,"error":{"code":-32603,"message":"internal error"}}"#.to_owned()
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;
    use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
    use tokio::net::UnixListener;
    use tokio::sync::mpsc::{Receiver, Sender};

    /// A stand-in for one daemon: accepts a connection, records the lines it receives, and
    /// replies with whatever the test queued.
    ///
    /// A real unix socket rather than a mock, because the framing between `btd` and a daemon is
    /// part of what is under test — the same reason `robotd`'s own tests speak over a socket.
    struct FakeDaemon {
        path: PathBuf,
        seen: Sender<String>,
        replies: Vec<String>,
    }

    impl FakeDaemon {
        fn spawn(dir: &std::path::Path, name: &str, replies: Vec<String>) -> (PathBuf, Receiver<String>) {
            let path = dir.join(name);
            let (seen, seen_rx) = mpsc::channel(16);
            let daemon = FakeDaemon { path: path.clone(), seen, replies };

            let listener = UnixListener::bind(&daemon.path).expect("bind");
            tokio::spawn(async move {
                while let Ok((stream, _)) = listener.accept().await {
                    let seen = daemon.seen.clone();
                    let replies = daemon.replies.clone();
                    tokio::spawn(async move {
                        let (read, mut write) = stream.into_split();
                        let mut lines = BufReader::new(read).lines();
                        while let Ok(Some(line)) = lines.next_line().await {
                            let _ = seen.send(line).await;
                            for reply in &replies {
                                let _ = write.write_all(format!("{reply}\n").as_bytes()).await;
                            }
                            let _ = write.flush().await;
                        }
                    });
                }
            });
            (path, seen_rx)
        }
    }

    /// Collect notified chunks and reassemble them the way a client would.
    async fn read_reply(from_robot: &mut Receiver<Vec<u8>>) -> String {
        let mut r = Reassembler::new();
        loop {
            let chunk = tokio::time::timeout(std::time::Duration::from_secs(2), from_robot.recv())
                .await
                .expect("client saw no reply")
                .expect("link closed");
            if let Some(line) = r.push(&chunk).expect("framing").into_iter().next() {
                return line;
            }
        }
    }

    fn sockets(dir: &std::path::Path, updater: &str, robot: &str) -> Sockets {
        Sockets { updater: dir.join(updater), robot: dir.join(robot) }
    }

    /// The ordinary path: an allowed call reaches the right daemon **byte for byte**, and its
    /// reply comes back. Verbatim forwarding is the property that keeps btd a transport.
    #[tokio::test]
    async fn an_allowed_call_is_forwarded_verbatim_and_answered() {
        let dir = tempdir();
        let (_, mut seen) = FakeDaemon::spawn(dir.path(), "updaterd.sock",
            vec![r#"{"jsonrpc":"2.0","id":1,"result":{"api_version":2,"daemon_version":"0.1.4","revision":null}}"#.into()]);
        let (_, _) = FakeDaemon::spawn(dir.path(), "robotd.sock", vec![]);

        let (link, to_robot, mut from_robot) = Link::pair(23, "AA:BB");
        tokio::spawn(run(link, sockets(dir.path(), "updaterd.sock", "robotd.sock")));

        let request = r#"{"jsonrpc":"2.0","id":1,"method":"hello","params":{"api_version":2}}"#;
        to_robot.send(request.as_bytes().to_vec()).await.unwrap();
        to_robot.send(b"\n".to_vec()).await.unwrap();

        assert_eq!(seen.recv().await.unwrap(), request, "not forwarded byte for byte");
        assert!(read_reply(&mut from_robot).await.contains(r#""api_version":2"#));
    }

    /// A refused call must never touch the upstream. Answering correctly is not enough — the
    /// point of the allowlist is that the daemon never sees it.
    #[tokio::test]
    async fn a_refused_call_never_reaches_the_daemon() {
        let dir = tempdir();
        let (_, mut seen) = FakeDaemon::spawn(dir.path(), "updaterd.sock", vec![]);
        let (_, _) = FakeDaemon::spawn(dir.path(), "robotd.sock", vec![]);

        let (link, to_robot, mut from_robot) = Link::pair(23, "AA:BB");
        tokio::spawn(run(link, sockets(dir.path(), "updaterd.sock", "robotd.sock")));

        to_robot.send(
            format!("{}\n", r#"{"jsonrpc":"2.0","id":9,"method":"update.resetToGolden","params":{"component":"daemon"}}"#)
                .into_bytes(),
        ).await.unwrap();

        let reply = read_reply(&mut from_robot).await;
        assert!(reply.contains(&proto::code::PERMISSION_DENIED.to_string()), "{reply}");

        // Nothing arrived at the daemon, and "nothing" needs a moment to be provable.
        tokio::time::sleep(std::time::Duration::from_millis(150)).await;
        assert!(seen.try_recv().is_err(), "a refused call was forwarded");
    }

    /// `robot.*` goes to `robotd` and not to `updaterd`. One table drives routing and
    /// permission, so a mistake here would send an update trigger to the control daemon.
    #[tokio::test]
    async fn robot_calls_go_to_robotd() {
        let dir = tempdir();
        let (_, mut updater_seen) = FakeDaemon::spawn(dir.path(), "updaterd.sock", vec![]);
        let (_, mut robot_seen) = FakeDaemon::spawn(dir.path(), "robotd.sock",
            vec![r#"{"jsonrpc":"2.0","id":2,"result":{"healthy":true}}"#.into()]);

        let (link, to_robot, mut from_robot) = Link::pair(185, "AA:BB");
        tokio::spawn(run(link, sockets(dir.path(), "updaterd.sock", "robotd.sock")));

        to_robot.send(b"{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"robot.health\"}\n".to_vec()).await.unwrap();

        assert!(robot_seen.recv().await.unwrap().contains("robot.health"));
        assert!(read_reply(&mut from_robot).await.contains(r#""healthy":true"#));
        assert!(updater_seen.try_recv().is_err(), "robotd's call went to updaterd");
    }

    /// A subscription is a stream of notifications on an open connection, and every one has to
    /// reach the central. This is the case that would break if replies were correlated to
    /// requests rather than forwarded as they arrive.
    #[tokio::test]
    async fn every_notification_in_a_stream_reaches_the_client() {
        let dir = tempdir();
        let progress: Vec<String> = (0..3)
            .map(|i| format!(
                r#"{{"jsonrpc":"2.0","method":"update.progress","params":{{"component":"daemon","phase":"downloading","percent":{},"detail":null}}}}"#,
                i * 50
            ))
            .collect();
        let (_, _) = FakeDaemon::spawn(dir.path(), "updaterd.sock", progress);
        let (_, _) = FakeDaemon::spawn(dir.path(), "robotd.sock", vec![]);

        let (link, to_robot, mut from_robot) = Link::pair(23, "AA:BB");
        tokio::spawn(run(link, sockets(dir.path(), "updaterd.sock", "robotd.sock")));

        to_robot.send(b"{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"update.subscribe\"}\n".to_vec()).await.unwrap();

        // A 23-byte MTU means each of these arrives in several chunks, so this also proves
        // reassembly survives back-to-back messages.
        for expected in [0, 50, 100] {
            let line = read_reply(&mut from_robot).await;
            assert!(line.contains(&format!(r#""percent":{expected}"#)), "{line}");
        }
    }

    /// Garbage gets an error with a null id, not a dropped session: a client that sent one bad
    /// line should be able to carry on.
    #[tokio::test]
    async fn an_unparseable_line_is_answered_and_the_session_survives() {
        let dir = tempdir();
        let (_, _) = FakeDaemon::spawn(dir.path(), "updaterd.sock",
            vec![r#"{"jsonrpc":"2.0","id":1,"result":{"api_version":2,"daemon_version":null,"revision":null}}"#.into()]);
        let (_, _) = FakeDaemon::spawn(dir.path(), "robotd.sock", vec![]);

        let (link, to_robot, mut from_robot) = Link::pair(185, "AA:BB");
        tokio::spawn(run(link, sockets(dir.path(), "updaterd.sock", "robotd.sock")));

        to_robot.send(b"not json at all\n".to_vec()).await.unwrap();
        let reply = read_reply(&mut from_robot).await;
        assert!(reply.contains(&proto::code::PARSE_ERROR.to_string()), "{reply}");
        assert!(reply.contains(r#""id":null"#), "{reply}");

        // Still usable.
        to_robot.send(b"{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"hello\",\"params\":{\"api_version\":2}}\n".to_vec()).await.unwrap();
        assert!(read_reply(&mut from_robot).await.contains(r#""api_version":2"#));
    }

    /// A daemon that is not running must produce a diagnosable error naming it, rather than a
    /// hang. `robotd` is missing precisely when an update has just restarted it, which is when
    /// someone is most likely to be looking at a phone.
    #[tokio::test]
    async fn a_dead_daemon_is_reported_rather_than_hanging() {
        let dir = tempdir();
        let (_, _) = FakeDaemon::spawn(dir.path(), "updaterd.sock", vec![]);
        // No robotd socket at all.

        let (link, to_robot, mut from_robot) = Link::pair(185, "AA:BB");
        tokio::spawn(run(link, sockets(dir.path(), "updaterd.sock", "absent.sock")));

        to_robot.send(b"{\"jsonrpc\":\"2.0\",\"id\":4,\"method\":\"robot.health\"}\n".to_vec()).await.unwrap();

        let reply = read_reply(&mut from_robot).await;
        assert!(reply.contains("Robot is not answering"), "{reply}");
    }

    /// A notification (no id) gets no reply even when refused — the spec says so, and a client
    /// waiting for one would wait forever.
    #[tokio::test]
    async fn a_refused_notification_is_answered_with_silence() {
        let dir = tempdir();
        let (_, mut seen) = FakeDaemon::spawn(dir.path(), "updaterd.sock", vec![]);
        let (_, _) = FakeDaemon::spawn(dir.path(), "robotd.sock", vec![]);

        let (link, to_robot, mut from_robot) = Link::pair(185, "AA:BB");
        tokio::spawn(run(link, sockets(dir.path(), "updaterd.sock", "robotd.sock")));

        to_robot.send(
            format!("{}\n", r#"{"jsonrpc":"2.0","method":"update.resetToGolden","params":{"component":"daemon"}}"#)
                .into_bytes(),
        ).await.unwrap();

        tokio::time::sleep(std::time::Duration::from_millis(150)).await;
        assert!(from_robot.try_recv().is_err(), "a notification was answered");
        assert!(seen.try_recv().is_err(), "a refused notification was forwarded");
    }

    /// Sockets live in a temp directory, and unix socket paths are short by necessity — a
    /// long temp path would exceed `sun_path` and fail to bind for reasons unrelated to btd.
    fn tempdir() -> tempfile::TempDir {
        tempfile::Builder::new().prefix("btd").tempdir().expect("tempdir")
    }
}
