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

use duck_control::ImuData;
use duck_control::imui2c::{
    ImuBus, ImuWho, LSM6DSV16X_ID, pick_lsm6dsv16x, probe_who_am_i, scan_lsm6dsv16x,
};
use duck_control::io::ImuIo;

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
        format!("imu_probe  {bus}  addr {address:#04x}  WHO_AM_I={LSM6DSV16X_ID:#04x}"),
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
