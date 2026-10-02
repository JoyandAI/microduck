//! Live LSM6DSV16X sample, from I²C or from `imu_to_servo` on the Feetech bus.
//!
//! The production alias is `/dev/i2c-imu` (i2c4 @ 0x6A). Some boards put the
//! chip on another controller — this example scans `/dev/i2c-*` for WHO_AM_I
//! `0x70` if you do not pass a bus. A serial port instead reads ID 200 (five
//! pings, model `0x4D44`) and decodes the same 12-byte block `robotd` uses.
//!
//! ```text
//! cargo run -p duck-control --example imu_probe
//! cargo run -p duck-control --example imu_probe -- /dev/i2c-5 0x6A
//! cargo run -p duck-control --example imu_probe -- /dev/i2c-5 0x6A --hz 50
//! cargo run -p duck-control --example imu_probe -- --scan
//! cargo run -p duck-control --example imu_probe -- --port COM5
//! cargo run -p duck-control --example imu_probe -- /dev/ttyACM0 --hz 50
//! ```
//!
//! Cross-compile for the board (Windows host: WSL + `aarch64-linux-gnu-gcc`):
//! `docs/robot/bringup-examples.md`.
//!
//! ```text
//! cargo zigbuild --target aarch64-unknown-linux-gnu.2.31 -p duck-control --example imu_probe
//! ```
//!
//! A TTY gets a frame that repaints in place. Anything else (a pipe, a file)
//! prints one snapshot and exits.

use std::io::{self, IsTerminal, Write};
use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use duck_control::IMU_DXL_ID;
use duck_control::ftbus::{IMU_MODEL_ADDR, IMU_PROBE_ATTEMPTS, IMU_TO_SERVO_MODEL, reg};
use duck_control::imu::{IMU_BLOCK_LEN, ImuData, SflpDecoder};
use duck_control::imui2c::{
    ImuBus, ImuWho, LSM6DSV16X_ID, pick_lsm6dsv16x, probe_who_am_i, scan_lsm6dsv16x,
};
use duck_control::io::ImuIo;
use duck_control::model::BAUD_RATE;
use rustypot::servo::feetech::sts3215::Sts3215Controller;

/// Display / sample rate if `--hz` is omitted. Comfortable on a TTY; the control
/// loop's 50 Hz is opt-in (`--hz 50`).
const DEFAULT_HZ: f64 = 10.0;

/// Fixed frame height so cursor-up always lands on the first row.
const ROWS: usize = 10;

/// Production default: i2c4 alias, SA0 pulled to the 0x6A pad.
const DEFAULT_BUS: &str = "/dev/i2c-imu";
const DEFAULT_ADDR: u8 = 0x6A;

static RUNNING: AtomicBool = AtomicBool::new(true);

fn main() {
    if let Err(e) = run() {
        eprintln!("{e}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.iter().any(|a| a == "-h" || a == "--help") {
        eprint!("{USAGE}");
        std::process::exit(2);
    }

    let flags = parse_flags(&args)?;
    let scan_only = flags.scan;
    let target_hz = flags.hz;
    let tick_period = Duration::from_secs_f64(1.0 / target_hz);
    if let Some(port) = &flags.port {
        if scan_only {
            return Err("--scan reads I²C; it does not take a serial port".into());
        }
        return run_bus(port, target_hz, tick_period);
    }
    let positional: Vec<&str> = flags.positional.iter().map(String::as_str).collect();

    let (bus, address) = match positional.as_slice() {
        [] if scan_only => {
            let hits = probe_who_am_i();
            print_scan(&hits);
            let n = hits
                .iter()
                .filter(|h| matches!(h.who, Ok(id) if id == LSM6DSV16X_ID))
                .count();
            if n != 1 {
                std::process::exit(if n == 0 { 1 } else { 2 });
            }
            return Ok(());
        }
        [] => match pick_bus()? {
            Some(pair) => pair,
            None => {
                eprint!("{USAGE}");
                std::process::exit(2);
            }
        },
        [bus] => (bus.to_string(), DEFAULT_ADDR),
        [bus, addr] => (bus.to_string(), parse_addr(addr)?),
        _ => {
            eprint!("{USAGE}");
            std::process::exit(2);
        }
    };

    if scan_only {
        print_scan(&probe_who_am_i());
        return Ok(());
    }

    install_sigint();

    let who = ImuBus::who_am_i(Path::new(&bus), address);
    match &who {
        Ok(id) if *id == LSM6DSV16X_ID => {}
        Ok(id) => {
            return Err(format!(
                "{bus} addr {address:#04x}: WHO_AM_I={id:#04x}, expected {LSM6DSV16X_ID:#04x} — not an LSM6DSV16X"
            )
            .into());
        }
        Err(e) => {
            return Err(format!("probe {bus} addr {address:#04x}: {e}").into());
        }
    }

    let mut imu = ImuBus::open(Path::new(&bus), address)
        .map_err(|e| format!("open {bus} addr {address:#04x}: {e}"))?;

    let tty = io::stdout().is_terminal();
    if tty {
        print!("\x1b[?25l");
        io::stdout().flush()?;
    }

    let mut first = true;
    let mut errors: u64 = 0;
    let mut reads: u64 = 0;
    let mut hz = 0.0;
    let mut last = Instant::now();
    let mut sample = ImuData::default();

    while RUNNING.load(Ordering::Relaxed) {
        let t0 = Instant::now();
        let last_err = match imu.read() {
            Ok(data) => {
                sample = data;
                reads += 1;
                let dt = last.elapsed().as_secs_f64();
                if dt > 0.0 {
                    hz = 1.0 / dt;
                }
                last = Instant::now();
                None
            }
            Err(e) => {
                errors += 1;
                if !tty {
                    return Err(e.into());
                }
                Some(e.to_string())
            }
        };

        let stale = imu.stale();
        let lines = render(
            &format!("imu_probe  {bus}  addr {address:#04x}  WHO_AM_I={LSM6DSV16X_ID:#04x}"),
            &sample,
            imu.ready(),
            stale.total,
            stale.run,
            reads,
            errors,
            hz,
            target_hz,
            last_err.as_deref(),
        );
        if tty {
            paint(&lines, first)?;
            first = false;
        } else {
            for line in &lines {
                println!("{line}");
            }
            break;
        }

        let spent = t0.elapsed();
        if let Some(rest) = tick_period.checked_sub(spent) {
            thread::sleep(rest);
        }
    }

    if tty {
        print!("\x1b[?25h\n");
        io::stdout().flush()?;
    }
    Ok(())
}

/// Stream ID 200. Five pings, then the model must be `0x4D44`. The sample is the
/// trunk-frame [`SflpDecoder`] output, the same numbers the control loop stores.
fn run_bus(
    port: &str,
    target_hz: f64,
    tick_period: Duration,
) -> Result<(), Box<dyn std::error::Error>> {
    install_sigint();
    let serial = serialport::new(port, BAUD_RATE)
        .timeout(Duration::from_millis(30))
        .open()
        .map_err(|e| {
            format!(
                "open {port}: {e}\n\
                 on the robot bus, stop robotd first (`sudo systemctl stop robotd`)"
            )
        })?;
    let mut controller = Sts3215Controller::new()
        .with_protocol_v1()
        .with_serial_port(serial);
    if !probe_bus_imu(&mut controller)? {
        return Err(format!(
            "no imu_to_servo at id {IMU_DXL_ID} on {port} after {IMU_PROBE_ATTEMPTS} pings"
        )
        .into());
    }

    let tty = io::stdout().is_terminal();
    if tty {
        print!("\x1b[?25l");
        io::stdout().flush()?;
    }

    let mut decoder = SflpDecoder::default();
    let mut last_raw: Option<[u8; IMU_BLOCK_LEN]> = None;
    let mut stale_total = 0u64;
    let mut stale_run = 0u64;
    let mut first = true;
    let mut errors: u64 = 0;
    let mut reads: u64 = 0;
    let mut hz = 0.0;
    let mut last = Instant::now();
    let mut sample = ImuData::default();
    let header =
        format!("imu_probe  {port}  feetech id {IMU_DXL_ID}  model {IMU_TO_SERVO_MODEL:#06x}");

    while RUNNING.load(Ordering::Relaxed) {
        let t0 = Instant::now();
        let last_err = match read_bus_block(&mut controller) {
            Ok(raw) => {
                if last_raw.replace(raw) == Some(raw) {
                    stale_total = stale_total.saturating_add(1);
                    stale_run = stale_run.saturating_add(1);
                } else {
                    stale_run = 0;
                }
                sample = decoder.decode(&raw);
                reads += 1;
                let dt = last.elapsed().as_secs_f64();
                if dt > 0.0 {
                    hz = 1.0 / dt;
                }
                last = Instant::now();
                None
            }
            Err(e) => {
                errors += 1;
                if !tty {
                    return Err(e);
                }
                Some(e.to_string())
            }
        };

        let lines = render(
            &header,
            &sample,
            decoder.ready(),
            stale_total,
            stale_run,
            reads,
            errors,
            hz,
            target_hz,
            last_err.as_deref(),
        );
        if tty {
            paint(&lines, first)?;
            first = false;
        } else {
            for line in &lines {
                println!("{line}");
            }
            break;
        }

        let spent = t0.elapsed();
        if let Some(rest) = tick_period.checked_sub(spent) {
            thread::sleep(rest);
        }
    }

    if tty {
        print!("\x1b[?25h\n");
        io::stdout().flush()?;
    }
    Ok(())
}

fn probe_bus_imu(controller: &mut Sts3215Controller) -> Result<bool, Box<dyn std::error::Error>> {
    for attempt in 1..=IMU_PROBE_ATTEMPTS {
        let answered = match controller.ping(IMU_DXL_ID) {
            Ok(alive) => alive,
            Err(e) => {
                eprintln!("ping {IMU_DXL_ID} attempt {attempt}: {e}");
                false
            }
        };
        if !answered {
            continue;
        }
        match controller.read_raw_data(IMU_DXL_ID, IMU_MODEL_ADDR, 2) {
            Ok(bytes) if bytes.len() == 2 => {
                let model = u16::from_le_bytes([bytes[0], bytes[1]]);
                if model == IMU_TO_SERVO_MODEL {
                    eprintln!(
                        "imu_to_servo id {IMU_DXL_ID} model {model:#06x} (attempt {attempt})"
                    );
                    return Ok(true);
                }
                return Err(format!(
                    "id {IMU_DXL_ID} answered model {model:#06x}, not an imu_to_servo ({IMU_TO_SERVO_MODEL:#06x})"
                )
                .into());
            }
            Ok(bytes) => eprintln!(
                "id {IMU_DXL_ID} model read was {} bytes (attempt {attempt})",
                bytes.len()
            ),
            Err(e) => eprintln!("id {IMU_DXL_ID} model read attempt {attempt}: {e}"),
        }
    }
    Ok(false)
}

fn read_bus_block(
    controller: &mut Sts3215Controller,
) -> Result<[u8; IMU_BLOCK_LEN], Box<dyn std::error::Error>> {
    let block = controller
        .read_raw_data(IMU_DXL_ID, reg::PRESENT_POSITION, IMU_BLOCK_LEN as u8)
        .map_err(|e| format!("read id {IMU_DXL_ID}: {e}"))?;
    if block.len() < IMU_BLOCK_LEN {
        return Err(format!("short imu block: {} bytes", block.len()).into());
    }
    let mut raw = [0u8; IMU_BLOCK_LEN];
    raw.copy_from_slice(&block[..IMU_BLOCK_LEN]);
    Ok(raw)
}

fn print_scan(hits: &[ImuWho]) {
    if hits.is_empty() {
        eprintln!("no 0x6A/0x6B answers on /dev/i2c-*");
        return;
    }
    println!("bus            addr   WHO_AM_I");
    for h in hits {
        let who = match &h.who {
            Ok(id) if *id == LSM6DSV16X_ID => format!("{id:#04x}  LSM6DSV16X"),
            Ok(id) => format!("{id:#04x}"),
            Err(e) => e.clone(),
        };
        println!(
            "{:<14} {addr:#04x}  {who}",
            h.bus.display(),
            addr = h.address
        );
    }
}

fn pick_bus() -> Result<Option<(String, u8)>, Box<dyn std::error::Error>> {
    match pick_lsm6dsv16x() {
        Some(hit) => {
            eprintln!(
                "using {} addr {:#04x} (WHO_AM_I={LSM6DSV16X_ID:#04x})",
                hit.bus.display(),
                hit.address
            );
            Ok(Some((hit.bus.display().to_string(), hit.address)))
        }
        None => {
            print_scan(&probe_who_am_i());
            if scan_lsm6dsv16x().len() > 1 {
                eprintln!("found several LSM6DSV16X chips; pass one bus explicitly");
            } else {
                eprintln!(
                    "no LSM6DSV16X (WHO_AM_I={LSM6DSV16X_ID:#04x}) at 0x6A/0x6B on /dev/i2c-*\n\
                     pass a bus explicitly, e.g. {DEFAULT_BUS} or /dev/i2c-5"
                );
            }
            Ok(None)
        }
    }
}

fn parse_flags(args: &[String]) -> Result<Flags, Box<dyn std::error::Error>> {
    let mut scan = false;
    let mut hz = DEFAULT_HZ;
    let mut port = None;
    let mut positional = Vec::new();
    let mut i = 0;
    while i < args.len() {
        let a = args[i].as_str();
        if a == "--port" {
            i += 1;
            let path = args
                .get(i)
                .cloned()
                .ok_or("--port needs a serial device, e.g. COM5 or /dev/ttyACM0")?;
            port = Some(path);
            i += 1;
            continue;
        }
        if a == "--scan" {
            scan = true;
            i += 1;
            continue;
        }
        let hz_arg = a.eq_ignore_ascii_case("--hz")
            || a.eq_ignore_ascii_case("-hz")
            || a.to_ascii_lowercase().starts_with("--hz=");
        if hz_arg {
            let value = if let Some((_, v)) = a.split_once('=') {
                v.to_string()
            } else {
                i += 1;
                args.get(i)
                    .cloned()
                    .ok_or("--hz needs a value, e.g. --hz 50")?
            };
            hz = parse_hz(&value)?;
            i += 1;
            continue;
        }
        if looks_like_uart(&args[i]) {
            if port.is_some() {
                return Err("pass the serial port once".into());
            }
            port = Some(args[i].clone());
            i += 1;
            continue;
        }
        positional.push(args[i].clone());
        i += 1;
    }
    Ok(Flags {
        scan,
        hz,
        port,
        positional,
    })
}

fn looks_like_uart(path: &str) -> bool {
    let lower = path.to_ascii_lowercase();
    let com = lower
        .strip_prefix("com")
        .is_some_and(|rest| !rest.is_empty() && rest.chars().all(|c| c.is_ascii_digit()));
    com || lower.contains("tty")
}

struct Flags {
    scan: bool,
    hz: f64,
    port: Option<String>,
    positional: Vec<String>,
}

fn parse_hz(s: &str) -> Result<f64, Box<dyn std::error::Error>> {
    let hz: f64 = s.parse().map_err(|_| format!("bad --hz {s:?}"))?;
    if !(hz.is_finite() && hz > 0.0 && hz <= 120.0) {
        return Err(format!("--hz {s} is out of range (0, 120]").into());
    }
    Ok(hz)
}

fn parse_addr(s: &str) -> Result<u8, Box<dyn std::error::Error>> {
    let t = s.trim();
    let addr = if let Some(hex) = t.strip_prefix("0x").or_else(|| t.strip_prefix("0X")) {
        u8::from_str_radix(hex, 16)?
    } else if t.chars().any(|c| matches!(c, 'a'..='f' | 'A'..='F')) {
        u8::from_str_radix(t, 16)?
    } else {
        t.parse::<u8>()?
    };
    Ok(addr)
}

fn render(
    header: &str,
    imu: &ImuData,
    ready: bool,
    stale_total: u64,
    stale_run: u64,
    reads: u64,
    errors: u64,
    hz: f64,
    target_hz: f64,
    last_err: Option<&str>,
) -> [String; ROWS] {
    let [gx, gy, gz] = imu.gyro;
    let [grx, gry, grz] = imu.gravity;
    let [qw, qx, qy, qz] = imu.quat;
    let to_dps = |v: f64| v * 180.0 / std::f64::consts::PI;
    let sample_kind = if ready {
        "live SFLP block"
    } else {
        "default (FIFO has not packed gyro+quat yet)"
    };
    [
        header.to_string(),
        format!("ready      {ready}  {sample_kind}"),
        format!("gyro rad/s {:>8.4} {:>8.4} {:>8.4}", gx, gy, gz),
        format!(
            "gyro dps   {:>8.2} {:>8.2} {:>8.2}",
            to_dps(gx),
            to_dps(gy),
            to_dps(gz)
        ),
        format!("gravity    {:>8.4} {:>8.4} {:>8.4}", grx, gry, grz),
        format!("quat wxyz  {:>8.4} {:>8.4} {:>8.4} {:>8.4}", qw, qx, qy, qz),
        format!("stale      total {stale_total}  run {stale_run}"),
        format!("reads      {reads}  errors {errors}"),
        format!("{hz:5.1} Hz  target {target_hz:.0}  Ctrl-C quit"),
        last_err
            .map(|e| format!("last err   {e}"))
            .unwrap_or_else(|| "last err   —".to_string()),
    ]
}

fn paint(lines: &[String; ROWS], first: bool) -> io::Result<()> {
    let mut out = io::stdout().lock();
    if !first {
        write!(out, "\x1b[{ROWS}A")?;
    }
    for line in lines {
        write!(out, "\r\x1b[2K{line}\n")?;
    }
    out.flush()
}

#[cfg(unix)]
fn install_sigint() {
    // SAFETY: the handler only stores to an AtomicBool.
    unsafe {
        libc::signal(
            libc::SIGINT,
            handle_sigint as *const () as libc::sighandler_t,
        );
        libc::signal(
            libc::SIGTERM,
            handle_sigint as *const () as libc::sighandler_t,
        );
    }
}

#[cfg(unix)]
extern "C" fn handle_sigint(_: libc::c_int) {
    RUNNING.store(false, Ordering::Relaxed);
}

#[cfg(windows)]
fn install_sigint() {
    // SAFETY: the handler only stores to an AtomicBool.
    unsafe {
        unsafe extern "system" {
            fn SetConsoleCtrlHandler(
                handler: Option<unsafe extern "system" fn(u32) -> i32>,
                add: i32,
            ) -> i32;
        }
        unsafe extern "system" fn handler(_: u32) -> i32 {
            RUNNING.store(false, Ordering::Relaxed);
            1
        }
        let _ = SetConsoleCtrlHandler(Some(handler), 1);
    }
}

#[cfg(not(any(unix, windows)))]
fn install_sigint() {}

const USAGE: &str = "\
imu_probe — live LSM6DSV16X sample (I²C chip, or imu_to_servo on the Feetech bus)

Usage:
  imu_probe [--hz N]                 # scan /dev/i2c-* for WHO_AM_I=0x70, then stream
  imu_probe --scan                   # print WHO_AM_I on 0x6A/0x6B, do not stream
  imu_probe [--hz N] <bus> [address] # open that I²C bus (default address 0x6A)
  imu_probe [--hz N] --port <uart>   # ping Feetech ID 200 five times, then stream

A COM port or a path containing \"tty\" is the bus, not an I²C device.
--hz defaults to 10. robotd samples the IMU at 50 Hz (--hz 50).
Stop robotd before opening the servo UART.

Examples:
  imu_probe /dev/i2c-5 0x6A
  imu_probe /dev/i2c-imu 0x6A --hz 50
  imu_probe --port COM5
  imu_probe /dev/ttyACM0 --hz 50

ready=false with gravity [0,0,-1] is the decoder default, not a dead chip:
the control path waits until SFLP has produced a quaternion.
";
