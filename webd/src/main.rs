//! LAN console for OpenMicroDuck.
//!
//! Two jobs, and they do not share the serial port:
//!
//! - **控制** talks to `robotd` over its unix socket, the same JSON-RPC `robotctl` uses.
//!   Stand, walk, roulade and the joint/attitude view all go through that. The browser
//!   never opens the bus, so it cannot race the control loop.
//! - **舵机** opens the bus itself, and only while `robotd` is stopped. One writer.
//! - **日志** reads each `*d` unit's stdout/stderr from journald, and this
//!   process's own tracing when webd was started from a terminal.

use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use axum::Json;
use axum::Router;
use axum::extract::ws::{Message, WebSocket, WebSocketUpgrade};
use axum::extract::{Path, Query, State};
use axum::http::{StatusCode, header};
use axum::response::IntoResponse;
use axum::routing::{get, post};
use clap::Parser;
use duck_control::ftbus::FeetechIo;
use duck_control::{JOINT_IDS, JOINT_NAMES};
use duck_ipc_proto as proto;
use futures_util::{SinkExt, StreamExt};
use serde::Deserialize;
use serde_json::{Value, json};
#[cfg(unix)]
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};

const INDEX: &str = include_str!("../web/index.html");
const SERVO: &str = include_str!("../web/servo.html");
const CALIBRATE: &str = include_str!("../web/calibrate.html");
const LOGS: &str = include_str!("../web/logs.html");
const CSS: &str = include_str!("../web/style.css");
const APP_JS: &str = include_str!("../web/app.js");
const SERVO_JS: &str = include_str!("../web/servo.js");
const CALIBRATE_JS: &str = include_str!("../web/calibrate.js");
const LOGS_JS: &str = include_str!("../web/logs.js");

/// Stiff position-loop gains while calibrating, so each horn holds a definite
/// encoder count. Same HD1910 factory numbers restored on exit. SRAM only.
const CAL_KP: u8 = 32;
const CAL_KD: u8 = 40;
/// HD1910 factory position-loop gains, restored when calibration exits.
const RUN_KP: u8 = 32;
const RUN_KD: u8 = 40;

const LABELS: [&str; 15] = [
    "左髋偏航",
    "左髋滚转",
    "左髋俯仰",
    "左膝",
    "左踝",
    "颈俯仰",
    "头俯仰",
    "头偏航",
    "头滚转",
    "嘴",
    "右髋偏航",
    "右髋滚转",
    "右髋俯仰",
    "右膝",
    "右踝",
];

#[derive(Parser, Debug)]
#[command(name = "webd", about = "OpenMicroDuck web console", version)]
struct Args {
    /// Address to listen on. `0.0.0.0` is what a phone on the same LAN needs.
    #[arg(long, default_value = "0.0.0.0")]
    host: String,
    /// HTTP port. mediad already uses 8080 for the camera console.
    #[arg(long, default_value_t = 8090)]
    port: u16,
    /// robotd's JSON-RPC socket.
    #[arg(long, default_value = "/run/robotd.sock")]
    robot_socket: PathBuf,
    /// Default serial port shown on the servo page.
    #[arg(long, default_value = "/dev/ttyS2")]
    servo_port: String,
    /// `robotd.toml`. The calibrate page reads and writes `policy.home` here.
    /// A missing file means the built-in standing pose.
    #[arg(long, default_value = robotd_params::DEFAULT_PATH)]
    config: PathBuf,
}

mod logs;

struct App {
    robot_socket: PathBuf,
    servo_port: String,
    config: PathBuf,
    bus: Mutex<Option<FeetechIo>>,
    capture: logs::Capture,
}

#[tokio::main]
async fn main() {
    let capture = logs::Capture::new();
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "webd=info".into()),
        )
        .with_writer(capture.clone())
        .init();

    let args = Args::parse();
    let app = Arc::new(App {
        robot_socket: args.robot_socket,
        servo_port: args.servo_port,
        config: args.config,
        bus: Mutex::new(None),
        capture,
    });
    let address: SocketAddr = match format!("{}:{}", args.host, args.port).parse() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("webd: {e}");
            std::process::exit(1);
        }
    };
    let listener = match tokio::net::TcpListener::bind(address).await {
        Ok(l) => l,
        Err(e) => {
            eprintln!("webd: could not listen on {address}: {e}");
            std::process::exit(1);
        }
    };
    tracing::info!(%address, "OpenMicroDuck console");
    axum::serve(listener, router(app))
        .with_graceful_shutdown(shutdown())
        .await
        .ok();
}

fn router(app: Arc<App>) -> Router {
    Router::new()
        .route("/", get(index))
        .route("/servo", get(servo_page))
        .route("/calibrate", get(calibrate_page))
        .route("/logs", get(logs_page))
        .route("/style.css", get(css))
        .route("/app.js", get(app_js))
        .route("/servo.js", get(servo_js))
        .route("/calibrate.js", get(calibrate_js))
        .route("/logs.js", get(logs_js))
        .route("/ws", get(ws))
        .route("/ws/logs", get(ws_logs))
        .route("/api/meta", get(meta))
        .route("/api/logs", get(api_logs))
        .route("/api/logs/daemons", get(api_daemons))
        .route("/api/servo/connect", post(servo_connect))
        .route("/api/servo/disconnect", post(servo_disconnect))
        .route("/api/servo/scan", get(servo_scan))
        .route("/api/servo/read/{id}", get(servo_read))
        .route("/api/servo/goto", post(servo_goto))
        .route("/api/servo/gains", post(servo_gains))
        .route("/api/servo/torque", post(servo_torque))
        .route("/api/servo/id", post(servo_id))
        .route("/api/cal/positions", get(cal_positions))
        .route("/api/cal/enter", post(cal_enter))
        .route("/api/cal/exit", post(cal_exit))
        .route("/api/cal/zero", post(cal_zero))
        .route("/api/cal/default", post(cal_default))
        .route("/api/cal/home", post(cal_save_home))
        .with_state(app)
}

async fn index() -> impl IntoResponse {
    page(INDEX, "text/html; charset=utf-8")
}
async fn servo_page() -> impl IntoResponse {
    page(SERVO, "text/html; charset=utf-8")
}
async fn calibrate_page() -> impl IntoResponse {
    page(CALIBRATE, "text/html; charset=utf-8")
}
async fn logs_page() -> impl IntoResponse {
    page(LOGS, "text/html; charset=utf-8")
}
async fn css() -> impl IntoResponse {
    page(CSS, "text/css; charset=utf-8")
}
async fn app_js() -> impl IntoResponse {
    page(APP_JS, "text/javascript; charset=utf-8")
}
async fn servo_js() -> impl IntoResponse {
    page(SERVO_JS, "text/javascript; charset=utf-8")
}
async fn calibrate_js() -> impl IntoResponse {
    page(CALIBRATE_JS, "text/javascript; charset=utf-8")
}
async fn logs_js() -> impl IntoResponse {
    page(LOGS_JS, "text/javascript; charset=utf-8")
}

fn page(body: &'static str, content_type: &'static str) -> impl IntoResponse {
    (
        [
            (header::CONTENT_TYPE, content_type),
            (header::CACHE_CONTROL, "no-cache"),
        ],
        body,
    )
}

async fn meta(State(app): State<Arc<App>>) -> Json<Value> {
    let homes = home_counts(&app.config);
    let joints: Vec<Value> = JOINT_NAMES
        .iter()
        .zip(JOINT_IDS)
        .zip(LABELS)
        .enumerate()
        .map(|(index, ((name, id), label))| {
            json!({
                "index": index,
                "name": name,
                "id": id,
                "label": label,
                "home": homes[index],
            })
        })
        .collect();
    Json(json!({
        "api_version": proto::API_VERSION,
        "servo_port": app.servo_port,
        "joints": joints,
    }))
}

async fn ws(State(app): State<Arc<App>>, upgrade: WebSocketUpgrade) -> impl IntoResponse {
    let path = app.robot_socket.clone();
    upgrade.on_upgrade(move |socket| bridge(socket, path))
}

#[derive(Deserialize)]
struct LogQuery {
    unit: Option<String>,
    stream: Option<String>,
    lines: Option<u32>,
}

fn log_params(q: &LogQuery) -> Result<(String, &'static str, usize), Api> {
    let unit = q.unit.clone().unwrap_or_else(|| "robotd".into());
    if !logs::allowed(&unit) {
        return Err(err(StatusCode::BAD_REQUEST, "不是已知的 *d 进程"));
    }
    Ok((
        unit,
        logs::stream_name(q.stream.as_deref()),
        logs::clamp_lines(q.lines),
    ))
}

async fn api_daemons() -> Json<Value> {
    Json(json!({ "daemons": logs::daemon_status().await }))
}

async fn api_logs(State(app): State<Arc<App>>, Query(q): Query<LogQuery>) -> Api {
    let (unit, stream, n) = match log_params(&q) {
        Ok(v) => v,
        Err(api) => return api,
    };
    let fetched = logs::fetch(&unit, stream, n, &app.capture).await;
    ok(json!({
        "ok": true,
        "unit": unit,
        "stream": stream,
        "lines": fetched.lines.iter().map(logs::as_json).collect::<Vec<_>>(),
        "hint": fetched.hint,
    }))
}

async fn ws_logs(
    State(app): State<Arc<App>>,
    Query(q): Query<LogQuery>,
    upgrade: WebSocketUpgrade,
) -> impl IntoResponse {
    match log_params(&q) {
        Err(api) => api.into_response(),
        Ok((unit, stream, n)) => {
            let capture = app.capture.clone();
            upgrade
                .on_upgrade(move |socket| follow_logs(socket, capture, unit, stream, n))
                .into_response()
        }
    }
}

async fn follow_logs(
    mut socket: WebSocket,
    capture: logs::Capture,
    unit: String,
    stream: &'static str,
    n: usize,
) {
    let fetched = logs::fetch(&unit, stream, n, &capture).await;
    let payload = json!({
        "lines": fetched.lines.iter().map(logs::as_json).collect::<Vec<_>>(),
        "hint": fetched.hint,
    });
    if socket
        .send(Message::Text(payload.to_string().into()))
        .await
        .is_err()
    {
        return;
    }
    #[cfg(unix)]
    follow_unix(socket, capture, unit, stream).await;
    #[cfg(not(unix))]
    follow_capture(socket, capture, unit, stream).await;
}

async fn follow_capture(
    socket: WebSocket,
    capture: logs::Capture,
    unit: String,
    stream: &'static str,
) {
    if unit != "webd" && unit != "all" {
        return;
    }
    let (mut ws_tx, mut ws_rx) = socket.split();
    let mut rx = capture.subscribe();
    loop {
        tokio::select! {
            msg = ws_rx.next() => {
                if msg.is_none() { break; }
            }
            cap = rx.recv() => {
                match cap {
                    Ok(line) if logs::keep_stream(stream, line.priority) => {
                        if ws_tx.send(Message::Text(logs::as_json(&line).to_string().into())).await.is_err() {
                            break;
                        }
                    }
                    Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => {}
                    Err(_) => break,
                    Ok(_) => {}
                }
            }
        }
    }
}

#[cfg(unix)]
async fn follow_unix(
    socket: WebSocket,
    capture: logs::Capture,
    unit: String,
    stream: &'static str,
) {
    let (mut ws_tx, mut ws_rx) = socket.split();
    let mut rx = if unit == "webd" || unit == "all" {
        Some(capture.subscribe())
    } else {
        None
    };
    let mut child = logs::follow_command(&unit, stream).spawn().ok();
    let mut journal_lines = match child.as_mut().and_then(|c| c.stdout.take()) {
        Some(out) => Some(BufReader::new(out).lines()),
        None => None,
    };
    if child.is_none() {
        let note = json!({
            "hint": "无法启动 journalctl 跟随。若正在看 webd，本进程的输出仍会推送。"
        });
        if ws_tx
            .send(Message::Text(note.to_string().into()))
            .await
            .is_err()
        {
            return;
        }
    }
    loop {
        tokio::select! {
            msg = ws_rx.next() => {
                if msg.is_none() { break; }
            }
            journal = next_journal_line(&mut journal_lines) => {
                if let Some(raw) = journal
                    && let Some(parsed) = logs::parse_journal_line(&raw)
                    && logs::keep_stream(stream, parsed.priority)
                    && ws_tx.send(Message::Text(logs::as_json(&parsed).to_string().into())).await.is_err()
                {
                    break;
                }
            }
            cap = async {
                match rx.as_mut() {
                    Some(r) => r.recv().await,
                    None => std::future::pending().await,
                }
            } => {
                match cap {
                    Ok(line) if logs::keep_stream(stream, line.priority) => {
                        if ws_tx.send(Message::Text(logs::as_json(&line).to_string().into())).await.is_err() {
                            break;
                        }
                    }
                    Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => {}
                    Err(_) => rx = None,
                    Ok(_) => {}
                }
            }
        }
    }
    if let Some(mut c) = child {
        let _ = c.kill().await;
    }
}

#[cfg(unix)]
async fn next_journal_line(
    lines: &mut Option<tokio::io::Lines<BufReader<tokio::process::ChildStdout>>>,
) -> Option<String> {
    loop {
        match lines.as_mut() {
            Some(src) => match src.next_line().await {
                Ok(Some(line)) => return Some(line),
                _ => *lines = None,
            },
            None => std::future::pending::<()>().await,
        }
    }
}

async fn bridge(socket: WebSocket, path: PathBuf) {
    #[cfg(unix)]
    unix_bridge(socket, path).await;
    #[cfg(not(unix))]
    not_unix(socket, path).await;
}

#[cfg(not(unix))]
async fn not_unix(mut socket: WebSocket, path: PathBuf) {
    let _ = path;
    let _ = socket
        .send(Message::Text(
            "{\"jsonrpc\":\"2.0\",\"error\":{\"code\":1,\"message\":\"控制通道只在机器人（Linux）上可用\"}}"
                .into(),
        ))
        .await;
}

#[cfg(unix)]
async fn unix_bridge(socket: WebSocket, path: PathBuf) {
    let stream = match tokio::net::UnixStream::connect(&path).await {
        Ok(s) => s,
        Err(e) => {
            let note = json!({
                "jsonrpc": "2.0",
                "error": {
                    "code": 1,
                    "message": format!(
                        "连不上 robotd（{}）：{e}。确认 robotd 在运行。",
                        path.display()
                    ),
                }
            });
            let mut socket = socket;
            let _ = socket.send(Message::Text(note.to_string().into())).await;
            return;
        }
    };
    let (read, mut write) = stream.into_split();
    let (mut ws_tx, mut ws_rx) = socket.split();

    let reader = tokio::spawn(async move {
        let mut lines = BufReader::new(read).lines();
        while let Ok(Some(line)) = lines.next_line().await {
            if ws_tx.send(Message::Text(line.into())).await.is_err() {
                break;
            }
        }
    });

    while let Some(Ok(msg)) = ws_rx.next().await {
        let Message::Text(text) = msg else { continue };
        let mut line = text.to_string();
        if !line.ends_with('\n') {
            line.push('\n');
        }
        if write.write_all(line.as_bytes()).await.is_err() {
            break;
        }
    }
    // The page went away. Zero the velocity so a dropped tab is not a stuck walk.
    // Torque stays on — unloading is an explicit button.
    let _ = write
        .write_all(b"{\"jsonrpc\":\"2.0\",\"method\":\"robot.stop\",\"params\":{}}\n")
        .await;
    reader.abort();
}

type Api = (StatusCode, Json<Value>);

fn ok(value: Value) -> Api {
    (StatusCode::OK, Json(value))
}

fn err(status: StatusCode, error: impl Into<String>) -> Api {
    (status, Json(json!({ "ok": false, "error": error.into() })))
}

fn explain_port(error: String) -> String {
    let lower = error.to_lowercase();
    if lower.contains("busy") || lower.contains("resource") {
        format!("{error}。串口被占用，先执行 sudo systemctl stop robotd")
    } else if lower.contains("permission") || lower.contains("denied") || lower.contains("access") {
        format!("{error}。当前用户没有这个串口的权限")
    } else {
        error
    }
}

#[derive(Deserialize)]
struct ConnectBody {
    port: Option<String>,
}

async fn servo_connect(State(app): State<Arc<App>>, Json(body): Json<ConnectBody>) -> Api {
    let port = body
        .port
        .filter(|p| !p.trim().is_empty())
        .unwrap_or_else(|| app.servo_port.clone());
    let opened =
        tokio::task::spawn_blocking(move || FeetechIo::open_bench(&port).map(|io| (io, port)))
            .await;
    match opened {
        Ok(Ok((io, port))) => {
            *app.bus.lock().unwrap_or_else(|p| p.into_inner()) = Some(io);
            ok(json!({ "ok": true, "port": port }))
        }
        Ok(Err(e)) => err(StatusCode::CONFLICT, explain_port(e.to_string())),
        Err(e) => err(StatusCode::INTERNAL_SERVER_ERROR, e.to_string()),
    }
}

async fn servo_disconnect(State(app): State<Arc<App>>) -> Api {
    *app.bus.lock().unwrap_or_else(|p| p.into_inner()) = None;
    ok(json!({ "ok": true }))
}

async fn servo_scan(State(app): State<Arc<App>>) -> Api {
    let joined = tokio::task::spawn_blocking(move || {
        let mut guard = app.bus.lock().unwrap_or_else(|p| p.into_inner());
        let Some(io) = guard.as_mut() else {
            return Err("还没有连接串口".to_owned());
        };
        Ok(io.bench_scan(&JOINT_IDS))
    })
    .await;
    match joined {
        Ok(Ok(online)) => ok(json!({ "ok": true, "online": online })),
        Ok(Err(e)) => err(StatusCode::BAD_REQUEST, e),
        Err(e) => err(StatusCode::INTERNAL_SERVER_ERROR, e.to_string()),
    }
}

async fn servo_read(State(app): State<Arc<App>>, Path(id): Path<u8>) -> Api {
    let joined = tokio::task::spawn_blocking(move || {
        let mut guard = app.bus.lock().unwrap_or_else(|p| p.into_inner());
        let Some(io) = guard.as_mut() else {
            return Err("还没有连接串口".to_owned());
        };
        io.bench_read(id).map_err(|e| e.to_string())
    })
    .await;
    match joined {
        Ok(Ok(sample)) => ok(serde_json::to_value(sample).unwrap_or(json!({ "ok": true }))),
        Ok(Err(e)) => err(StatusCode::BAD_REQUEST, e),
        Err(e) => err(StatusCode::INTERNAL_SERVER_ERROR, e.to_string()),
    }
}

#[derive(Deserialize)]
struct GotoBody {
    id: u8,
    position: u16,
}

async fn servo_goto(State(app): State<Arc<App>>, Json(body): Json<GotoBody>) -> Api {
    run_bus(app, move |io| {
        io.bench_goto(body.id, body.position)
            .map_err(|e| e.to_string())
    })
    .await
}

#[derive(Deserialize)]
struct GainsBody {
    id: u8,
    kp: u16,
    kd: u16,
}

async fn servo_gains(State(app): State<Arc<App>>, Json(body): Json<GainsBody>) -> Api {
    if body.kp > 254 || body.kd > 254 {
        return err(StatusCode::BAD_REQUEST, "KP 和 KD 的范围是 0–254");
    }
    let kp = body.kp as u8;
    let kd = body.kd as u8;
    run_bus(app, move |io| {
        io.bench_gains(body.id, kp, kd).map_err(|e| e.to_string())
    })
    .await
}

#[derive(Deserialize)]
struct TorqueBody {
    id: u8,
    on: bool,
}

async fn servo_torque(State(app): State<Arc<App>>, Json(body): Json<TorqueBody>) -> Api {
    run_bus(app, move |io| {
        io.bench_torque(body.id, body.on).map_err(|e| e.to_string())
    })
    .await
}

#[derive(Deserialize)]
struct IdBody {
    id: u8,
    new_id: u16,
}

async fn servo_id(State(app): State<Arc<App>>, Json(body): Json<IdBody>) -> Api {
    if !(1..=253).contains(&body.new_id) || body.new_id == duck_control::IMU_DXL_ID as u16 {
        return err(StatusCode::BAD_REQUEST, "新 ID 要在 1–253，并且不能是 200");
    }
    let new_id = body.new_id as u8;
    run_bus(app, move |io| {
        io.bench_set_id(body.id, new_id).map_err(|e| e.to_string())
    })
    .await
}

async fn run_bus<F>(app: Arc<App>, f: F) -> Api
where
    F: FnOnce(&mut FeetechIo) -> Result<(), String> + Send + 'static,
{
    let joined = tokio::task::spawn_blocking(move || {
        let mut guard = app.bus.lock().unwrap_or_else(|p| p.into_inner());
        let Some(io) = guard.as_mut() else {
            return Err("还没有连接串口".to_owned());
        };
        f(io)
    })
    .await;
    match joined {
        Ok(Ok(())) => ok(json!({ "ok": true })),
        Ok(Err(e)) => err(StatusCode::BAD_REQUEST, e),
        Err(e) => err(StatusCode::INTERNAL_SERVER_ERROR, e.to_string()),
    }
}

fn home_counts(config: &std::path::Path) -> [u16; 15] {
    let pose = match robotd_params::Params::load(config, false) {
        Ok(params) => duck_control::standing_pose(params.policy.home.as_deref()),
        Err(error) => {
            tracing::warn!(
                %error,
                path = %config.display(),
                "standing pose unreadable; using the built-in"
            );
            duck_control::DEFAULT_POSITION
        }
    };
    let mut counts = [0u16; 15];
    for (i, rad) in pose.iter().enumerate() {
        counts[i] = duck_control::ftbus::rad_to_position_count(*rad);
    }
    counts
}

/// Write `policy.home` in degrees and leave every other key and comment alone.
///
/// `pose` is radians, the same unit as the control loop. The file stores degrees;
/// `robotd-params` converts back on the next load.
fn write_home(path: &std::path::Path, pose: &[f64; 15]) -> Result<(), String> {
    let text = match std::fs::read_to_string(path) {
        Ok(text) => text,
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => String::new(),
        Err(error) => return Err(format!("读不到配置：{error}")),
    };
    let mut doc: toml_edit::DocumentMut = text
        .parse()
        .map_err(|error| format!("配置不是合法的 toml：{error}"))?;
    let mut array = toml_edit::Array::new();
    for angle in pose {
        array.push(angle.to_degrees());
    }
    doc["policy"]["home"] = toml_edit::Item::Value(array.into());
    if let Some(parent) = path.parent()
        && !parent.as_os_str().is_empty()
    {
        std::fs::create_dir_all(parent).map_err(|error| format!("建不了配置目录：{error}"))?;
    }
    let tmp = path.with_extension("toml.tmp");
    std::fs::write(&tmp, doc.to_string()).map_err(|error| format!("写不了配置：{error}"))?;
    if let Err(error) = std::fs::rename(&tmp, path) {
        if path.exists() {
            std::fs::remove_file(path).map_err(|err| format!("替换不了配置：{err}"))?;
            std::fs::rename(&tmp, path).map_err(|err| format!("替换不了配置：{err}"))?;
        } else {
            let _ = std::fs::remove_file(&tmp);
            return Err(format!("替换不了配置：{error}"));
        }
    }
    Ok(())
}

fn joint_slot(id: u8) -> Option<usize> {
    JOINT_IDS.iter().position(|joint| *joint == id)
}

async fn with_bus<T, F>(app: Arc<App>, f: F) -> Result<T, Api>
where
    T: Send + 'static,
    F: FnOnce(&mut FeetechIo) -> Result<T, String> + Send + 'static,
{
    let joined = tokio::task::spawn_blocking(move || {
        let mut guard = app.bus.lock().unwrap_or_else(|p| p.into_inner());
        let Some(io) = guard.as_mut() else {
            return Err("还没有连接串口".to_owned());
        };
        f(io)
    })
    .await;
    match joined {
        Ok(Ok(value)) => Ok(value),
        Ok(Err(e)) => Err(err(StatusCode::BAD_REQUEST, e)),
        Err(e) => Err(err(StatusCode::INTERNAL_SERVER_ERROR, e.to_string())),
    }
}

fn each_joint(
    io: &mut FeetechIo,
    mut step: impl FnMut(&mut FeetechIo, u8) -> Result<(), String>,
) -> Result<(), String> {
    let mut errors = Vec::new();
    for &id in &JOINT_IDS {
        if let Err(e) = step(io, id) {
            errors.push(format!("{id}: {e}"));
        }
    }
    if errors.is_empty() {
        Ok(())
    } else {
        Err(errors.join("；"))
    }
}

async fn cal_positions(State(app): State<Arc<App>>) -> Api {
    match with_bus(app, |io| {
        let positions: Vec<Value> = JOINT_IDS
            .iter()
            .map(|&id| match io.bench_position(id) {
                Ok(position) => json!({ "id": id, "position": position }),
                Err(_) => json!({ "id": id, "position": null }),
            })
            .collect();
        Ok(positions)
    })
    .await
    {
        Ok(positions) => ok(json!({ "ok": true, "positions": positions })),
        Err(api) => api,
    }
}

async fn cal_enter(State(app): State<Arc<App>>) -> Api {
    match with_bus(app, |io| {
        each_joint(io, |io, id| {
            io.bench_gains(id, CAL_KP, CAL_KD)
                .map_err(|e| e.to_string())?;
            io.bench_goto(id, 2048).map_err(|e| e.to_string())
        })
    })
    .await
    {
        Ok(()) => ok(json!({ "ok": true })),
        Err(api) => api,
    }
}

async fn cal_exit(State(app): State<Arc<App>>) -> Api {
    match with_bus(app, |io| {
        each_joint(io, |io, id| {
            io.bench_torque(id, false).map_err(|e| e.to_string())?;
            io.bench_gains(id, RUN_KP, RUN_KD)
                .map_err(|e| e.to_string())
        })
    })
    .await
    {
        Ok(()) => ok(json!({ "ok": true })),
        Err(api) => api,
    }
}

#[derive(Deserialize)]
struct CalId {
    id: u8,
}

async fn cal_zero(State(app): State<Arc<App>>, Json(body): Json<CalId>) -> Api {
    if joint_slot(body.id).is_none() {
        return err(StatusCode::BAD_REQUEST, "不是这只鸭子的关节");
    }
    match with_bus(app, move |io| {
        io.bench_set_mid(body.id).map_err(|e| e.to_string())
    })
    .await
    {
        Ok(position) => ok(json!({ "ok": true, "id": body.id, "position": position })),
        Err(api) => api,
    }
}

#[derive(Deserialize)]
struct CalDefault {
    id: Option<u8>,
}

async fn cal_default(State(app): State<Arc<App>>, Json(body): Json<CalDefault>) -> Api {
    let homes = home_counts(&app.config);
    let targets: Vec<(u8, u16)> = match body.id {
        Some(id) => match joint_slot(id) {
            Some(index) => vec![(id, homes[index])],
            None => return err(StatusCode::BAD_REQUEST, "不是这只鸭子的关节"),
        },
        None => JOINT_IDS.iter().copied().zip(homes).collect(),
    };
    match with_bus(app, move |io| each_joint_targets(io, &targets)).await {
        Ok(()) => ok(json!({ "ok": true })),
        Err(api) => api,
    }
}

fn each_joint_targets(io: &mut FeetechIo, targets: &[(u8, u16)]) -> Result<(), String> {
    let mut errors = Vec::new();
    for &(id, position) in targets {
        if let Err(e) = io.bench_goto(id, position) {
            errors.push(format!("{id}: {e}"));
        }
    }
    if errors.is_empty() {
        Ok(())
    } else {
        Err(errors.join("；"))
    }
}

async fn cal_save_home(State(app): State<Arc<App>>) -> Api {
    let counts = match with_bus(app.clone(), |io| {
        let mut counts = [0u16; 15];
        for (index, &id) in JOINT_IDS.iter().enumerate() {
            counts[index] = io
                .bench_position(id)
                .map_err(|error| format!("ID {id}：{error}"))?;
        }
        Ok(counts)
    })
    .await
    {
        Ok(counts) => counts,
        Err(api) => return api,
    };
    let mut pose = [0.0; 15];
    for (index, count) in counts.iter().enumerate() {
        let Some(radians) = duck_control::ftbus::position_to_rad(*count) else {
            return err(
                StatusCode::BAD_REQUEST,
                format!("{} 的编码器 {count} 不是单圈位置", LABELS[index]),
            );
        };
        pose[index] = radians;
    }
    if let Err(error) = write_home(&app.config, &pose) {
        return err(StatusCode::INTERNAL_SERVER_ERROR, error);
    }
    ok(json!({
        "ok": true,
        "path": app.config.display().to_string(),
        "home": home_counts(&app.config),
    }))
}

async fn shutdown() {
    let ctrl_c = tokio::signal::ctrl_c();
    #[cfg(unix)]
    {
        use tokio::signal::unix::{SignalKind, signal};
        let mut term = match signal(SignalKind::terminate()) {
            Ok(s) => s,
            Err(_) => {
                let _ = ctrl_c.await;
                return;
            }
        };
        tokio::select! {
            _ = ctrl_c => {}
            _ = term.recv() => {}
        }
    }
    #[cfg(not(unix))]
    {
        let _ = ctrl_c.await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pages_are_titled_openmicroduck() {
        assert!(INDEX.contains("<title>OpenMicroDuck</title>"));
        assert!(SERVO.contains("<title>OpenMicroDuck</title>"));
        assert!(CALIBRATE.contains("<title>OpenMicroDuck</title>"));
        assert!(LOGS.contains("<title>OpenMicroDuck</title>"));
        assert!(LOGS_JS.contains("/ws/logs"));
        assert!(INDEX.contains("href=\"/logs\""));
        assert!(CALIBRATE.contains("设 0 位"));
        assert!(CALIBRATE.contains("运行到默认位置"));
        assert!(CALIBRATE.contains("保存为默认站姿"));
        assert_eq!(LABELS.len(), JOINT_NAMES.len());
        assert_eq!(JOINT_IDS.len(), JOINT_NAMES.len());
        let homes = home_counts(std::path::Path::new("/no/such/microduck-robotd.toml"));
        assert_eq!(homes[0], 2048, "a 0 rad joint is the encoder midpoint");
        assert_eq!(homes.len(), 15);
    }

    #[test]
    fn saving_the_standing_pose_round_trips() {
        let dir = std::env::temp_dir().join(format!("microduck-home-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("robotd.toml");
        std::fs::write(&path, "[policy]\nenabled = true\n# keep me\n").unwrap();
        let mut pose = duck_control::DEFAULT_POSITION;
        pose[5] = 0.2;
        write_home(&path, &pose).unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        assert!(text.contains("# keep me"), "comments survive the edit");
        let raw: toml_edit::DocumentMut = text.parse().unwrap();
        let stored = raw["policy"]["home"][5].as_float().unwrap();
        assert!(
            (stored - 0.2_f64.to_degrees()).abs() < 1e-6,
            "the file stores degrees, got {stored}"
        );
        let loaded = robotd_params::Params::load(&path, true).unwrap();
        let got = duck_control::standing_pose(loaded.policy.home.as_deref());
        assert!((got[5] - 0.2).abs() < 1e-9);
        assert!((got[0] - pose[0]).abs() < 1e-9);
        let _ = std::fs::remove_dir_all(&dir);
    }
}
