//! Connections to the services that actually own the answers.
//!
//! One socket per service, connected directly — with four services there is no case for a
//! broker, and a bus would be another component that can fail (`architecture.md` §2.2).
//!
//! Every operation here is timeout-bounded, without exception. Any peer may be dead, and a
//! closed or silent socket is a normal answer rather than an error worth retrying forever —
//! `robotd` in particular is the service most likely to be missing, since it is the one an
//! update restarts.

use std::collections::HashMap;
use std::io;
use std::path::{Path, PathBuf};
use std::time::Duration;

use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::UnixStream;
use tokio::sync::mpsc;

use crate::route::Upstream;

/// Long enough for a loaded board, short enough that a phone gets an answer rather than a
/// spinner. A unix socket connect either succeeds immediately or the daemon is not there.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(3);

/// Cap on a single write. A blocked write means the daemon has stopped reading, which is a
/// dead peer rather than a slow one.
const WRITE_TIMEOUT: Duration = Duration::from_secs(5);

/// Where each service listens.
#[derive(Debug, Clone)]
pub struct Sockets {
    pub updater: PathBuf,
    pub robot: PathBuf,
}

impl Sockets {
    pub fn path(&self, upstream: Upstream) -> &Path {
        match upstream {
            Upstream::Updater => &self.updater,
            Upstream::Robot => &self.robot,
        }
    }
}

/// The write half of a live connection. The read half lives in a spawned task.
struct Conn {
    write: tokio::net::unix::OwnedWriteHalf,
}

/// Connections opened so far in one BLE session, made on demand.
///
/// Lazy rather than eager because most sessions touch one service: a phone asking for the
/// version has no reason to make `robotd` accept a connection it will never use. And because
/// connecting eagerly would mean a dead `robotd` delayed or failed a session that did not need
/// it.
pub struct Pool {
    sockets: Sockets,
    conns: HashMap<Upstream, Conn>,
    /// Every reply and notification from every upstream, merged. Merging is safe because
    /// JSON-RPC correlates by `id`, which is the client's business — `btd` forwards lines
    /// without reading them.
    replies: mpsc::Sender<String>,
}

impl Pool {
    pub fn new(sockets: Sockets, replies: mpsc::Sender<String>) -> Self {
        Self { sockets, conns: HashMap::new(), replies }
    }

    /// Send one line to `upstream`, connecting first if needed.
    pub async fn send(&mut self, upstream: Upstream, line: &str) -> io::Result<()> {
        if !self.conns.contains_key(&upstream) {
            let conn = self.open(upstream).await?;
            self.conns.insert(upstream, conn);
        }

        // Unwrap is sound: just inserted, or the contains_key above held.
        let conn = self.conns.get_mut(&upstream).expect("connection present");

        let mut bytes = line.as_bytes().to_vec();
        bytes.push(b'\n');

        let write = async {
            conn.write.write_all(&bytes).await?;
            conn.write.flush().await
        };
        match tokio::time::timeout(WRITE_TIMEOUT, write).await {
            Ok(Ok(())) => Ok(()),
            Ok(Err(e)) => {
                // A broken pipe here is ordinary — the daemon restarted. Drop the connection
                // so the next call reconnects rather than writing into a dead socket forever.
                self.conns.remove(&upstream);
                Err(e)
            }
            Err(_) => {
                self.conns.remove(&upstream);
                Err(io::Error::new(io::ErrorKind::TimedOut, "upstream write timed out"))
            }
        }
    }

    async fn open(&self, upstream: Upstream) -> io::Result<Conn> {
        let path = self.sockets.path(upstream);
        let stream = tokio::time::timeout(CONNECT_TIMEOUT, UnixStream::connect(path))
            .await
            .map_err(|_| io::Error::new(io::ErrorKind::TimedOut, "connect timed out"))??;

        let (read, write) = stream.into_split();
        let replies = self.replies.clone();
        let label = format!("{upstream:?}");

        // The read half is pumped for the session's lifetime. Responses and notifications are
        // the same thing to us: a line to forward. That is what makes `update.subscribe`'s
        // progress stream work without any special case.
        tokio::spawn(async move {
            let mut lines = BufReader::new(read).lines();
            loop {
                match lines.next_line().await {
                    Ok(Some(line)) => {
                        // A full queue means the central cannot keep up. Give up on the line
                        // rather than the session: progress is advisory, and blocking here
                        // would stall every other upstream too.
                        if replies.try_send(line).is_err() {
                            tracing::debug!(upstream = %label, "dropped a line; client is behind");
                        }
                    }
                    Ok(None) => break,
                    Err(e) => {
                        tracing::debug!(upstream = %label, error = %e, "upstream read failed");
                        break;
                    }
                }
            }
            tracing::debug!(upstream = %label, "upstream closed");
        });

        tracing::debug!(upstream = ?upstream, path = %path.display(), "connected");
        Ok(Conn { write })
    }
}
