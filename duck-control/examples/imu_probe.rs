//! Live LSM6DSV16X sample from the duck's I²C IMU bus.
//!
//! The production alias is `/dev/i2c-imu` (i2c4 @ 0x6A). Some boards put the
//! chip on another controller — this example scans `/dev/i2c-*` for WHO_AM_I
//! `0x70` if you do not pass a bus.
//!
//! ```text
//! cargo run -p duck-control --example imu_probe
//! cargo run -p duck-control --example imu_probe -- /dev/i2c-5 0x6A
//! cargo run -p duck-control --example imu_probe -- /dev/i2c-5 0x6A --hz 50
//! cargo run -p duck-control --example imu_probe -- --scan
//! ```
//!
//! Cross-compile for the board:
//!
//! ```text
//! cargo zigbuild --target aarch64-unknown-linux-gnu.2.31 -p duck-control --example imu_probe
//! ```
//!
//! A TTY gets a frame that repaints in place. Anything else (a pipe, a file)
//! prints one snapshot and exits.

use std::io::{self, IsTerminal, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use duck_control::ImuData;
use duck_control::imui2c::ImuBus;
use duck_control::io::ImuIo;

/// Display / sample rate if `--hz` is omitted. Comfortable on a TTY; the control
/// loop's 50 Hz is opt-in (`--hz 50`).
const DEFAULT_HZ: f64 = 10.0;

/// Fixed frame height so cursor-up always lands on the first row.
const ROWS: usize = 10;

/// Production default: i2c4 alias, SA0 pulled to the 0x6A pad.
const DEFAULT_BUS: &str = "/dev/i2c-imu";
const DEFAULT_ADDR: u8 = 0x6A;
const WHO_AM_I: u8 = 0x70;
const CANDIDATE_ADDRS: [u8; 2] = [0x6A, 0x6B];

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
    let positional: Vec<&str> = flags.positional.iter().map(String::as_str).collect();

    let (bus, address) = match positional.as_slice() {
        [] if scan_only => {
            let hits = scan_buses()?;
            print_scan(&hits);
            if hits.iter().filter(|h| h.who == Some(WHO_AM_I)).count() != 1 {
                std::process::exit(if hits.iter().any(|h| h.who == Some(WHO_AM_I)) {
                    2
                } else {
                    1
                });
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
        let hits = scan_buses()?;
        print_scan(&hits);
        return Ok(());
    }

    install_sigint();

    let who = ImuBus::who_am_i(Path::new(&bus), address);
    match &who {
        Ok(id) if *id == WHO_AM_I => {}
        Ok(id) => {
            return Err(format!(
                "{bus} addr {address:#04x}: WHO_AM_I={id:#04x}, expected {WHO_AM_I:#04x} — not an LSM6DSV16X"
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
            &bus,
            address,
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

struct ScanHit {
    bus: PathBuf,
    addr: u8,
    who: Option<u8>,
    err: Option<String>,
}

fn scan_buses() -> Result<Vec<ScanHit>, Box<dyn std::error::Error>> {
    let mut buses: Vec<PathBuf> = Vec::new();
    if let Ok(dir) = std::fs::read_dir("/dev") {
        for ent in dir.flatten() {
            let name = ent.file_name();
            let name = name.to_string_lossy();
            if let Some(rest) = name.strip_prefix("i2c-")
                && rest.chars().all(|c| c.is_ascii_digit())
            {
                buses.push(ent.path());
            }
        }
    }
    buses.sort_by(|a, b| bus_num(a).cmp(&bus_num(b)));

    let mut hits = Vec::new();
    for bus in buses {
        for addr in CANDIDATE_ADDRS {
            match ImuBus::who_am_i(&bus, addr) {
                Ok(who) => hits.push(ScanHit {
                    bus: bus.clone(),
                    addr,
                    who: Some(who),
                    err: None,
                }),
                Err(e) => {
                    let msg = e.to_string();
                    // Skip buses that simply have no device at this address.
                    if msg.contains("ENXIO")
                        || msg.contains("No such device")
                        || msg.contains("Remote I/O")
                        || msg.contains("Input/output error")
                    {
                        continue;
                    }
                    hits.push(ScanHit {
                        bus: bus.clone(),
                        addr,
                        who: None,
                        err: Some(msg),
                    });
                }
            }
        }
    }
    Ok(hits)
}

fn bus_num(path: &Path) -> u32 {
    path.file_name()
        .and_then(|n| n.to_str())
        .and_then(|n| n.strip_prefix("i2c-"))
        .and_then(|n| n.parse().ok())
        .unwrap_or(u32::MAX)
}

fn print_scan(hits: &[ScanHit]) {
    if hits.is_empty() {
        eprintln!("no 0x6A/0x6B answers on /dev/i2c-*");
        return;
    }
    println!("bus            addr   WHO_AM_I");
    for h in hits {
        let who = h
            .who
            .map(|id| {
                if id == WHO_AM_I {
                    format!("{id:#04x}  LSM6DSV16X")
                } else {
                    format!("{id:#04x}")
                }
            })
            .unwrap_or_else(|| h.err.clone().unwrap_or_else(|| "err".into()));
        println!("{:<14} {addr:#04x}  {who}", h.bus.display(), addr = h.addr);
    }
}

fn pick_bus() -> Result<Option<(String, u8)>, Box<dyn std::error::Error>> {
    let hits = scan_buses()?;
    let found: Vec<&ScanHit> = hits.iter().filter(|h| h.who == Some(WHO_AM_I)).collect();
    match found.as_slice() {
        [one] => {
            eprintln!(
                "using {} addr {:#04x} (WHO_AM_I={WHO_AM_I:#04x})",
                one.bus.display(),
                one.addr
            );
            Ok(Some((one.bus.display().to_string(), one.addr)))
        }
        [] => {
            print_scan(&hits);
            eprintln!(
                "no LSM6DSV16X (WHO_AM_I={WHO_AM_I:#04x}) at 0x6A/0x6B on /dev/i2c-*\n\
                 pass a bus explicitly, e.g. {DEFAULT_BUS} or /dev/i2c-5"
            );
            Ok(None)
        }
        many => {
            print_scan(&hits);
            eprintln!(
                "found {} LSM6DSV16X chips; pass one bus explicitly",
                many.len()
            );
            Ok(None)
        }
    }
}

fn parse_flags(args: &[String]) -> Result<Flags, Box<dyn std::error::Error>> {
    let mut scan = false;
    let mut hz = DEFAULT_HZ;
    let mut positional = Vec::new();
    let mut i = 0;
    while i < args.len() {
        let a = args[i].as_str();
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
        positional.push(args[i].clone());
        i += 1;
    }
    Ok(Flags {
        scan,
        hz,
        positional,
    })
}

struct Flags {
    scan: bool,
    hz: f64,
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
    bus: &str,
    address: u8,
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
        format!("imu_probe  {bus}  addr {address:#04x}  WHO_AM_I={WHO_AM_I:#04x}"),
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
imu_probe — live LSM6DSV16X sample (SFLP FIFO, not raw OUT registers)

Usage:
  imu_probe [--hz N]                 # scan /dev/i2c-* for WHO_AM_I=0x70, then stream
  imu_probe --scan                   # print WHO_AM_I on 0x6A/0x6B, do not stream
  imu_probe [--hz N] <bus> [address] # open that bus (default address 0x6A)

--hz defaults to 10. robotd samples the IMU at 50 Hz (--hz 50).

Examples:
  imu_probe /dev/i2c-5 0x6A
  imu_probe /dev/i2c-5 0x6A --hz 50
  imu_probe /dev/i2c-imu 0x6A

ready=false with gravity [0,0,-1] is the decoder default, not a dead chip:
the control path waits for a FIFO block that has both gyro and SFLP quat.
";
