//! Daemon stdout/stderr as journald captured them, plus this process's own tracing.

use std::collections::VecDeque;
use std::io::{self, Write};
use std::sync::{Arc, Mutex};
use std::time::{SystemTime, UNIX_EPOCH};

use serde::Serialize;
use serde_json::{Value, json};
use tracing_subscriber::fmt::MakeWriter;

pub const DAEMONS: &[&str] = &[
    "robotd", "padd", "webd", "mediad", "btd", "configd", "updaterd", "tofd",
];

const RING: usize = 1500;
const MAX_LINES: u32 = 2000;

#[derive(Clone)]
pub struct Capture {
    inner: Arc<Inner>,
}

struct Inner {
    lines: Mutex<VecDeque<Line>>,
    live: tokio::sync::broadcast::Sender<Line>,
    partial: Mutex<Vec<u8>>,
}

#[derive(Clone, Serialize)]
pub struct Line {
    pub ts: u64,
    pub unit: String,
    pub stream: &'static str,
    pub priority: u8,
    pub message: String,
}

impl Capture {
    pub fn new() -> Self {
        let (live, _) = tokio::sync::broadcast::channel(256);
        Self {
            inner: Arc::new(Inner {
                lines: Mutex::new(VecDeque::new()),
                live,
                partial: Mutex::new(Vec::new()),
            }),
        }
    }

    pub fn snapshot(&self, n: usize) -> Vec<Line> {
        let guard = self.inner.lines.lock().unwrap_or_else(|p| p.into_inner());
        guard.iter().rev().take(n).rev().cloned().collect()
    }

    pub fn subscribe(&self) -> tokio::sync::broadcast::Receiver<Line> {
        self.inner.live.subscribe()
    }

    fn push_text(&self, text: &str) {
        let line = Line {
            ts: now_us(),
            unit: "webd".into(),
            stream: "stderr",
            priority: 6,
            message: text.trim_end().to_owned(),
        };
        {
            let mut guard = self.inner.lines.lock().unwrap_or_else(|p| p.into_inner());
            if guard.len() >= RING {
                guard.pop_front();
            }
            guard.push_back(line.clone());
        }
        let _ = self.inner.live.send(line);
    }
}

impl<'a> MakeWriter<'a> for Capture {
    type Writer = CaptureWriter;
    fn make_writer(&'a self) -> Self::Writer {
        CaptureWriter {
            inner: self.inner.clone(),
        }
    }
}

pub struct CaptureWriter {
    inner: Arc<Inner>,
}

impl Write for CaptureWriter {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        let _ = io::stderr().write_all(buf);
        let mut partial = self.inner.partial.lock().unwrap_or_else(|p| p.into_inner());
        partial.extend_from_slice(buf);
        while let Some(at) = partial.iter().position(|b| *b == b'\n') {
            let raw: Vec<u8> = partial.drain(..=at).collect();
            let text = String::from_utf8_lossy(&raw);
            drop(partial);
            Capture {
                inner: self.inner.clone(),
            }
            .push_text(&text);
            partial = self.inner.partial.lock().unwrap_or_else(|p| p.into_inner());
        }
        Ok(buf.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        io::stderr().flush()
    }
}

pub fn allowed(unit: &str) -> bool {
    unit == "all" || DAEMONS.contains(&unit)
}

pub fn clamp_lines(n: Option<u32>) -> usize {
    n.unwrap_or(300).clamp(1, MAX_LINES) as usize
}

pub fn stream_name(raw: Option<&str>) -> &'static str {
    match raw.unwrap_or("all") {
        "stdout" => "stdout",
        "stderr" => "stderr",
        _ => "all",
    }
}

fn now_us() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_micros() as u64)
        .unwrap_or(0)
}

pub fn keep_stream(stream: &str, priority: u8) -> bool {
    match stream {
        "stdout" => priority >= 5,
        "stderr" => priority <= 4,
        _ => true,
    }
}

#[cfg(unix)]
fn line_from_journal(value: &Value) -> Option<Line> {
    let message = match value.get("MESSAGE") {
        Some(Value::String(s)) => s.clone(),
        Some(Value::Array(bytes)) => bytes
            .iter()
            .filter_map(|b| b.as_u64().map(|n| n as u8))
            .map(|b| b as char)
            .collect(),
        _ => return None,
    };
    if message.starts_with("-- No entries --") {
        return None;
    }
    let priority = value
        .get("PRIORITY")
        .and_then(|p| p.as_str())
        .and_then(|s| s.parse().ok())
        .unwrap_or(6);
    let ts = value
        .get("__REALTIME_TIMESTAMP")
        .and_then(|p| p.as_str())
        .and_then(|s| s.parse().ok())
        .unwrap_or_else(now_us);
    let unit = value
        .get("SYSLOG_IDENTIFIER")
        .and_then(|p| p.as_str())
        .or_else(|| value.get("_SYSTEMD_UNIT").and_then(|p| p.as_str()))
        .or_else(|| value.get("_COMM").and_then(|p| p.as_str()))
        .unwrap_or("unknown")
        .trim_end_matches(".service")
        .to_owned();
    Some(Line {
        ts,
        unit,
        stream: if priority <= 4 { "stderr" } else { "stdout" },
        priority,
        message,
    })
}

#[cfg(unix)]
pub async fn daemon_status() -> Vec<Value> {
    let mut out = Vec::new();
    for &id in DAEMONS {
        out.push(unit_status(id).await);
    }
    out
}

#[cfg(not(unix))]
pub async fn daemon_status() -> Vec<Value> {
    DAEMONS
        .iter()
        .map(|id| json!({ "id": id, "active": "unknown", "pid": 0, "hint": "日志只在机器人（Linux）上可读" }))
        .collect()
}

#[cfg(unix)]
async fn unit_status(id: &str) -> Value {
    let shown = tokio::process::Command::new("systemctl")
        .args(["show", "-p", "ActiveState", "-p", "SubState", "-p", "MainPID", "--", id])
        .output()
        .await;
    let mut active = "unknown".to_owned();
    let mut sub = String::new();
    let mut pid: u32 = 0;
    if let Ok(done) = shown {
        let text = String::from_utf8_lossy(&done.stdout);
        for line in text.lines() {
            if let Some(v) = line.strip_prefix("ActiveState=") {
                active = v.to_owned();
            } else if let Some(v) = line.strip_prefix("SubState=") {
                sub = v.to_owned();
            } else if let Some(v) = line.strip_prefix("MainPID=") {
                pid = v.parse().unwrap_or(0);
            }
        }
    }
    if id == "webd" && pid == 0 {
        pid = std::process::id();
        if active == "unknown" || active == "inactive" {
            active = "self".into();
        }
    }
    json!({ "id": id, "active": active, "sub": sub, "pid": pid })
}

pub struct Fetch {
    pub lines: Vec<Line>,
    pub hint: Option<String>,
}

#[cfg(unix)]
pub async fn fetch(unit: &str, stream: &str, n: usize, capture: &Capture) -> Fetch {
    let mut lines = Vec::new();
    let mut hint = None;
    if unit == "webd" || unit == "all" {
        lines.extend(capture.snapshot(n));
    }
    match journalctl(unit, stream, n).await {
        Ok(JournalOut::Lines(more)) => {
            if unit == "webd" || unit == "all" {
                lines.extend(more);
                lines.sort_by_key(|l| l.ts);
                let extra = lines.len().saturating_sub(n);
                if extra > 0 {
                    lines.drain(0..extra);
                }
            } else {
                lines = more;
            }
        }
        Ok(JournalOut::Empty) => {
            if lines.is_empty() {
                hint = Some(empty_hint(unit));
            }
        }
        Err(e) => hint = Some(e),
    }
    lines.retain(|l| keep_stream(stream, l.priority));
    Fetch { lines, hint }
}

#[cfg(not(unix))]
pub async fn fetch(unit: &str, _stream: &str, n: usize, capture: &Capture) -> Fetch {
    let mut lines = Vec::new();
    if unit == "webd" || unit == "all" {
        lines = capture.snapshot(n);
    }
    Fetch {
        lines,
        hint: Some("系统日志只在机器人（Linux）上可读。webd 自己的输出仍显示在下面。".into()),
    }
}

#[cfg(unix)]
fn empty_hint(unit: &str) -> String {
    format!(
        "journalctl 没有读到 {unit} 的条目。常见原因：运行 webd 的用户不在 systemd-journal 组，\
         或这个进程是从终端拉起的、stdout/stderr 还在那个终端上。把用户加入该组后重新登录，\
         或用 sudo 启动 webd。手动启动的 webd 看本页「webd」即可，不经过 journal。"
    )
}

#[cfg(unix)]
enum JournalOut {
    Lines(Vec<Line>),
    Empty,
}

#[cfg(unix)]
async fn journalctl(unit: &str, stream: &str, n: usize) -> Result<JournalOut, String> {
    let mut cmd = tokio::process::Command::new("journalctl");
    cmd.args([
        "--no-pager",
        "-o",
        "json",
        "--output-fields=MESSAGE,PRIORITY,__REALTIME_TIMESTAMP,SYSLOG_IDENTIFIER,_COMM,_SYSTEMD_UNIT",
        "-n",
        &n.to_string(),
    ]);
    match stream {
        "stdout" => {
            cmd.arg("-p").arg("5..7");
        }
        "stderr" => {
            cmd.arg("-p").arg("0..4");
        }
        _ => {}
    }
    snapshot_args(&mut cmd, unit);
    cmd.stdout(std::process::Stdio::piped());
    cmd.stderr(std::process::Stdio::piped());
    let done = cmd.output().await.map_err(|e| format!("无法运行 journalctl：{e}"))?;
    let stderr = String::from_utf8_lossy(&done.stderr).trim().to_owned();
    let stdout = String::from_utf8_lossy(&done.stdout);
    if !done.status.success() && stdout.is_empty() {
        return Err(if stderr.is_empty() {
            format!("journalctl 退出 {}", done.status)
        } else {
            stderr
        });
    }
    let mut lines = Vec::new();
    for raw in stdout.lines() {
        let Ok(value) = serde_json::from_str::<Value>(raw) else {
            continue;
        };
        if let Some(line) = line_from_journal(&value) {
            lines.push(line);
        }
    }
    if lines.is_empty() {
        if stderr.contains("Permission") || stderr.contains("denied") {
            return Err(stderr);
        }
        return Ok(JournalOut::Empty);
    }
    Ok(JournalOut::Lines(lines))
}

#[cfg(unix)]
fn snapshot_args(cmd: &mut tokio::process::Command, unit: &str) {
    if unit == "all" {
        for id in DAEMONS {
            cmd.arg("-u").arg(*id);
        }
    } else {
        cmd.arg("-u").arg(unit);
    }
}

#[cfg(unix)]
pub fn follow_command(unit: &str, stream: &str) -> tokio::process::Command {
    let mut cmd = tokio::process::Command::new("journalctl");
    cmd.args([
        "--no-pager",
        "-o",
        "json",
        "--output-fields=MESSAGE,PRIORITY,__REALTIME_TIMESTAMP,SYSLOG_IDENTIFIER,_COMM,_SYSTEMD_UNIT",
        "-n",
        "0",
        "-f",
    ]);
    match stream {
        "stdout" => {
            cmd.arg("-p").arg("5..7");
        }
        "stderr" => {
            cmd.arg("-p").arg("0..4");
        }
        _ => {}
    }
    snapshot_args(&mut cmd, unit);
    cmd.stdout(std::process::Stdio::piped());
    cmd.stderr(std::process::Stdio::piped());
    cmd.kill_on_drop(true);
    cmd
}

#[cfg(unix)]
pub fn parse_journal_line(raw: &str) -> Option<Line> {
    serde_json::from_str::<Value>(raw)
        .ok()
        .and_then(|v| line_from_journal(&v))
}

pub fn as_json(line: &Line) -> Value {
    json!({
        "ts": line.ts,
        "unit": line.unit,
        "stream": line.stream,
        "priority": line.priority,
        "message": line.message,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_known_daemons() {
        assert!(allowed("robotd"));
        assert!(allowed("padd"));
        assert!(allowed("all"));
        assert!(!allowed("robotd;id"));
        assert!(!allowed("/etc/passwd"));
        assert!(!allowed("sshd"));
    }

    #[test]
    fn stdout_is_info_and_above() {
        assert!(keep_stream("stdout", 6));
        assert!(!keep_stream("stdout", 3));
        assert!(keep_stream("stderr", 3));
        assert!(!keep_stream("stderr", 6));
        assert!(keep_stream("all", 3));
    }
}
