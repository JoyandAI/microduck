//! Live FT-HLS servo status for the duck's fifteen joints.
//!
//! Compile on a development machine (not on the board):
//!
//! ```text
//! cargo run -p duck-control --example hls_read -- COM5
//! cargo run -p duck-control --example hls_read -- /dev/ttyACM0
//! ```
//!
//! For the Radxa (aarch64), see `docs/robot/bringup-examples.md`.
//!
//! The UART is exclusive (`TIOCEXCL` on Linux): if this talks to the robot's
//! onboard bus, stop `robotd` first. After the fifteen joints, ID 200 is pinged
//! five times. A reply whose model is `0x4D44` is `imu_to_servo`: this example still
//! places it first in one `sync_read` with the joints. The control loop does not —
//! `FeetechIo` reads the fifteen servos with `sync_read` and ID 200 with a unicast
//! read. Opening here is read-only: unlike
//! [`duck_control::ftbus::FeetechIo::open`], this does not rewrite SRAM registers
//! or drop torque.
//!
//! A TTY gets a 14-row × 15-column frame that repaints in place. Anything else
//! (a pipe, a file) gets one snapshot and exits, so the escape codes never land
//! in a log.

use std::f64::consts::PI;
use std::io::{self, IsTerminal, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use duck_control::ftbus::{self, IMU_MODEL_ADDR, IMU_PROBE_ATTEMPTS, IMU_TO_SERVO_MODEL, reg};
use duck_control::imu::{IMU_BLOCK_LEN, ImuData, SflpDecoder};
use duck_control::model::BAUD_RATE;
use duck_control::{IMU_DXL_ID, JOINT_IDS, JOINT_NAMES, NUM_JOINTS};
use rustypot::servo::feetech::sts3215::Sts3215Controller;

/// Same bound as the control loop: a missing device costs a hiccup, not a stall.
const READ_TIMEOUT: Duration = Duration::from_millis(30);

/// The tick block: present_position (56) through present_current (69), 15 bytes.
const TICK_ADDR: u8 = reg::PRESENT_POSITION;
const TICK_LEN: u8 = 15;

/// Display cadence. The bus can go faster; the terminal does not need to.
const FRAME_PERIOD: Duration = Duration::from_millis(100);

/// Fixed frame height so cursor-up always lands on the first row.
/// Thirteen joint rows, one IMU row, one footer.
const ROWS: usize = 15;

/// Per-servo column width, not counting the space between columns.
const COL: usize = 6;

/// Left gutter for the row label.
const LABEL: usize = 8;

static RUNNING: AtomicBool = AtomicBool::new(true);

/// Short names that fit [`COL`]. The wire names are the long [`JOINT_NAMES`].
/// Indexed as [`JOINT_IDS`], not as the TTY columns — see [`COL_ORDER`].
const SHORT_NAMES: [&str; NUM_JOINTS] = [
    "l_yaw", "l_rol", "l_pit", "l_kne", "l_ank", // left leg
    "n_pit", "h_pit", "h_yaw", "h_rol", "mouth", // neck, head, mouth
    "r_yaw", "r_rol", "r_pit", "r_kne", "r_ank", // right leg
];

/// TTY columns: IDs 10–14, then 20–24, then 30–34. Each entry is an index
/// into [`JOINT_IDS`] / [`SHORT_NAMES`] / the sample array.
const COL_ORDER: [usize; NUM_JOINTS] = [10, 11, 12, 13, 14, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9];

const _: () = {
    let mut seen = [false; NUM_JOINTS];
    let mut i = 0;
    while i < NUM_JOINTS {
        let idx = COL_ORDER[i];
        assert!(idx < NUM_JOINTS);
        assert!(!seen[idx]);
        seen[idx] = true;
        i += 1;
    }
};

#[derive(Clone, Copy)]
struct Sample {
    pos_deg: f64,
    pos_rad: Option<f64>,
    rpm: f64,
    load_pct: f64,
    volts: f64,
    current_ma: f64,
    temp_c: f64,
    moving: bool,
    goal_deg: f64,
    status: u8,
    async_flag: u8,
}

fn main() {
    if let Err(e) = run() {
        eprintln!("{e}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let port = match args.next() {
        Some(p) if p != "-h" && p != "--help" => p,
        _ => {
            eprint!("{USAGE}");
            std::process::exit(2);
        }
    };

    install_sigint();

    let serial = serialport::new(&port, BAUD_RATE)
        .timeout(READ_TIMEOUT)
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

    let mut alive = [false; NUM_JOINTS];
    let mut n_alive = 0usize;
    for (i, &id) in JOINT_IDS.iter().enumerate() {
        match controller.ping(id) {
            Ok(true) => {
                alive[i] = true;
                n_alive += 1;
            }
            Ok(false) => eprintln!("servo {id} ({}) did not answer ping", JOINT_NAMES[i]),
            Err(e) => eprintln!("ping {id} ({}): {e}", JOINT_NAMES[i]),
        }
    }
    if n_alive == 0 {
        return Err("no duck servo answered (IDs 10-14 / 20-24 / 30-34)".into());
    }
    let imu_on_bus = probe_bus_imu(&mut controller);

    let tty = io::stdout().is_terminal();
    if tty {
        print!("\x1b[?25l");
        io::stdout().flush()?;
    }

    let mut first = true;
    let mut misses: u64 = 0;
    let mut hz = 0.0;
    let mut last = Instant::now();
    let mut samples: [Option<Sample>; NUM_JOINTS] = [const { None }; NUM_JOINTS];
    let mut imu = SflpDecoder::default();
    let mut imu_sample = ImuData::default();

    while RUNNING.load(Ordering::Relaxed) {
        let t0 = Instant::now();
        match read_tick(
            &mut controller,
            &alive,
            &mut samples,
            imu_on_bus,
            &mut imu,
            &mut imu_sample,
        ) {
            Ok(()) => {
                let dt = last.elapsed().as_secs_f64();
                if dt > 0.0 {
                    hz = 1.0 / dt;
                }
                last = Instant::now();
            }
            Err(e) => {
                misses += 1;
                if !tty {
                    return Err(e);
                }
            }
        }

        let lines = render(
            &port,
            &samples,
            n_alive,
            hz,
            misses,
            imu_on_bus,
            &imu_sample,
            imu.ready(),
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
        if let Some(rest) = FRAME_PERIOD.checked_sub(spent) {
            thread::sleep(rest);
        }
    }

    if tty {
        print!("\x1b[?25h\n");
        io::stdout().flush()?;
    }
    Ok(())
}

/// Unicast ping of ID 200, up to [`IMU_PROBE_ATTEMPTS`] times. The first reply whose
/// model is [`IMU_TO_SERVO_MODEL`] is the bus IMU. A miss leaves the joint table alone.
fn probe_bus_imu(controller: &mut Sts3215Controller) -> bool {
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
                    return true;
                }
                eprintln!(
                    "id {IMU_DXL_ID} answered model {model:#06x}, not an imu_to_servo ({IMU_TO_SERVO_MODEL:#06x})"
                );
                return false;
            }
            Ok(bytes) => eprintln!(
                "id {IMU_DXL_ID} model read was {} bytes (attempt {attempt})",
                bytes.len()
            ),
            Err(e) => eprintln!("id {IMU_DXL_ID} model read attempt {attempt}: {e}"),
        }
    }
    eprintln!("no imu_to_servo at id {IMU_DXL_ID} after {IMU_PROBE_ATTEMPTS} pings");
    false
}

fn read_tick(
    controller: &mut Sts3215Controller,
    alive: &[bool; NUM_JOINTS],
    samples: &mut [Option<Sample>; NUM_JOINTS],
    imu_on_bus: bool,
    imu: &mut SflpDecoder,
    imu_sample: &mut ImuData,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut ids = Vec::with_capacity(NUM_JOINTS + usize::from(imu_on_bus));
    if imu_on_bus {
        ids.push(IMU_DXL_ID);
    }
    ids.extend(
        JOINT_IDS
            .iter()
            .zip(alive.iter())
            .filter_map(|(&id, &ok)| ok.then_some(id)),
    );
    let blocks = controller
        .sync_read_raw_data(&ids, TICK_ADDR, TICK_LEN)
        .map_err(|e| format!("sync_read: {e}"))?;
    if blocks.len() != ids.len() {
        return Err(format!(
            "sync_read: expected {} blocks, got {}",
            ids.len(),
            blocks.len()
        )
        .into());
    }

    let joint_blocks = if imu_on_bus {
        *imu_sample = decode_imu_block(&blocks[0], imu)?;
        &blocks[1..]
    } else {
        &blocks[..]
    };

    let mut b = 0usize;
    for (i, &ok) in alive.iter().enumerate() {
        if !ok {
            samples[i] = None;
            continue;
        }
        samples[i] = Some(decode_block(&joint_blocks[b])?);
        b += 1;
    }
    Ok(())
}

fn decode_imu_block(
    block: &[u8],
    imu: &mut SflpDecoder,
) -> Result<ImuData, Box<dyn std::error::Error>> {
    if block.len() < IMU_BLOCK_LEN {
        return Err(format!("short imu block: {} bytes", block.len()).into());
    }
    let mut raw = [0u8; IMU_BLOCK_LEN];
    raw.copy_from_slice(&block[..IMU_BLOCK_LEN]);
    Ok(imu.decode(&raw))
}

fn decode_block(block: &[u8]) -> Result<Sample, Box<dyn std::error::Error>> {
    if block.len() < TICK_LEN as usize {
        return Err(format!("short block: {} bytes", block.len()).into());
    }
    let position = u16::from_le_bytes([block[0], block[1]]);
    let speed = u16::from_le_bytes([block[2], block[3]]);
    let load = u16::from_le_bytes([block[4], block[5]]);
    let goal = u16::from_le_bytes([block[11], block[12]]);
    let current = u16::from_le_bytes([block[13], block[14]]);

    let pos_rad = ftbus::position_to_rad(position);
    let goal_rad = ftbus::position_to_rad(goal);
    let rad_s = ftbus::speed_to_rad_s(speed);

    Ok(Sample {
        pos_deg: pos_rad.map(|r| r.to_degrees()).unwrap_or(f64::NAN),
        pos_rad,
        rpm: rad_s * 60.0 / (2.0 * PI),
        load_pct: ftbus::load_percent(load),
        volts: ftbus::voltage_to_volts(block[6]),
        current_ma: ftbus::current_to_ma(current),
        temp_c: ftbus::temperature_to_c(block[7]),
        moving: block[10] != 0,
        goal_deg: goal_rad.map(|r| r.to_degrees()).unwrap_or(f64::NAN),
        status: block[9],
        async_flag: block[8],
    })
}

fn render(
    port: &str,
    samples: &[Option<Sample>; NUM_JOINTS],
    n_alive: usize,
    hz: f64,
    misses: u64,
    imu_on_bus: bool,
    imu: &ImuData,
    imu_ready: bool,
) -> [String; ROWS] {
    let names = row_cells("name", cells_in_col_order(|i| pad(SHORT_NAMES[i])));
    let ids = row_cells("id", cells_in_col_order(|i| pad(&JOINT_IDS[i].to_string())));
    let pos_deg = metric(samples, "pos deg", |s| fmt_f(s.pos_deg, 1));
    let pos_rad = metric(samples, "pos rad", |s| match s.pos_rad {
        Some(r) => fmt_f(r, 2),
        None => pad("MT"),
    });
    let rpm = metric(samples, "spd rpm", |s| fmt_f(s.rpm, 1));
    let load = metric(samples, "load %", |s| fmt_f(s.load_pct, 1));
    let volt = metric(samples, "volt V", |s| fmt_f(s.volts, 1));
    let curr = metric(samples, "curr mA", |s| fmt_f(s.current_ma, 0));
    let temp = metric(samples, "temp C", |s| fmt_f(s.temp_c, 0));
    let moving = metric(samples, "moving", |s| pad(if s.moving { "*" } else { "." }));
    let goal = metric(samples, "goal deg", |s| fmt_f(s.goal_deg, 1));
    let status = metric(samples, "status", |s| {
        if s.status == 0 {
            pad("ok")
        } else {
            pad(&format!("0x{:02x}", s.status))
        }
    });
    let async_flag = metric(samples, "async", |s| pad(&s.async_flag.to_string()));

    let imu_note = if imu_on_bus {
        "IMU id 200"
    } else {
        "no bus IMU"
    };
    let footer = format!(
        "{hz:5.1} Hz  {n_alive}/{n} alive  {misses} miss  {port}  {imu_note}  Ctrl-C quit",
        n = NUM_JOINTS,
    );

    [
        names,
        ids,
        pos_deg,
        pos_rad,
        rpm,
        load,
        volt,
        curr,
        temp,
        moving,
        goal,
        status,
        async_flag,
        imu_row(imu_on_bus, imu, imu_ready),
        footer,
    ]
}

fn imu_row(imu_on_bus: bool, imu: &ImuData, ready: bool) -> String {
    if !imu_on_bus {
        return format!(
            "{:<LABEL$}no id {IMU_DXL_ID} after {IMU_PROBE_ATTEMPTS} pings",
            "imu"
        );
    }
    let [gx, gy, gz] = imu.gyro.map(|v| v * 180.0 / PI);
    let [px, py, pz] = imu.gravity;
    let [qw, qx, qy, qz] = imu.quat;
    format!(
        "{:<LABEL$}id {IMU_DXL_ID}  g {gx:+6.2} {gy:+6.2} {gz:+6.2} dps  \
         grav {px:+.3} {py:+.3} {pz:+.3}  q {qw:+.3} {qx:+.3} {qy:+.3} {qz:+.3}  ready {ready}",
        "imu"
    )
}

fn metric(
    samples: &[Option<Sample>; NUM_JOINTS],
    label: &str,
    cell: impl Fn(&Sample) -> String,
) -> String {
    row_cells(
        label,
        cells_in_col_order(|i| samples[i].as_ref().map(&cell).unwrap_or_else(|| pad("--"))),
    )
}

fn cells_in_col_order<T>(cell: impl Fn(usize) -> T) -> [T; NUM_JOINTS] {
    std::array::from_fn(|col| cell(COL_ORDER[col]))
}

fn row_cells(label: &str, cells: [String; NUM_JOINTS]) -> String {
    let mut line = format!("{label:<LABEL$}");
    for c in cells {
        line.push_str(&c);
        line.push(' ');
    }
    line
}

fn pad(s: &str) -> String {
    format!("{s:>COL$}")
}

fn fmt_f(v: f64, digits: usize) -> String {
    if !v.is_finite() {
        return pad("--");
    }
    pad(&format!("{v:.digits$}"))
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
    // SAFETY: the handler only stores to an AtomicBool. An interrupt before this
    // point simply kills the process — nothing has hidden the cursor yet.
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

#[cfg(not(unix))]
fn install_sigint() {}

#[cfg(unix)]
extern "C" fn handle_sigint(_: libc::c_int) {
    RUNNING.store(false, Ordering::Relaxed);
}

const USAGE: &str = "\
hls_read — live Feetech HD1910 status on the duck's servo bus

Usage:
  cargo run -p duck-control --example hls_read -- <port>

Windows:
  cargo run -p duck-control --example hls_read -- COM5

Linux / the robot:
  sudo systemctl stop robotd
  cargo run -p duck-control --example hls_read -- /dev/ttyACM0

Reads every duck servo ID (10-14, 20-24, 30-34). Then pings ID 200 five times.
Model 0x4D44 is imu_to_servo and is read in the same sync_read, shown on the
imu row in the trunk frame. Does not enable torque or write registers.
";
