//! Every Feetech device on one bus: HD1910 feedback and `imu_to_servo`.
//!
//! Same job as `E:\dev\imu_to_servo\src\Tools\imu_servo_read.py`. A broadcast
//! PING is confirmed with a unicast PING, because on a full duck bus the
//! broadcast replies overlap. Known joint IDs are then pinged too, and ID 200
//! is tried five times so a late `imu_to_servo` is not missed. The IMU is the
//! device whose model number is `0x4D44`; its address 56 is gyro and quaternion,
//! and acceleration is the six bytes at address 136. Servos are the 15-byte
//! block at address 56. Voltage is byte 6 of that block, 0.1 V per count.
//!
//! Read-only. Stop `robotd` before opening the robot's UART.
//!
//! ```text
//! cargo run -p duck-control --example imu_servo_read -- --port COM13
//! cargo run -p duck-control --example imu_servo_read -- /dev/ttyACM0 --hz 10
//! ```

use std::io::{self, IsTerminal, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use duck_control::ftbus::{
    IMU_MODEL_ADDR, IMU_PROBE_ATTEMPTS, IMU_TO_SERVO_MODEL, voltage_to_volts,
};
use duck_control::model::BAUD_RATE;
use duck_control::{IMU_DXL_ID, JOINT_IDS};
use serialport::SerialPort;

const SERVO_ADDR: u8 = 56;
const SERVO_LEN: u8 = 15;
/// Address 56 through the 6-byte accelerometer at 136.
const IMU_LEN: u8 = 86;
const ACCEL_OFFSET: usize = 80;
const GYRO_DPS_PER_LSB: f64 = 17.5e-3;
const ACCEL_G_PER_LSB: f64 = 0.122e-3;
const RPM_PER_SPEED: f64 = 0.732;
const MA_PER_CURRENT: f64 = 6.5;
const BROADCAST_ID: u8 = 0xFE;
const PING: u8 = 0x01;
const READ: u8 = 0x02;

/// How long a broadcast PING is left open. Replies from several servos are spread
/// over tens of milliseconds.
const BROADCAST_WINDOW: Duration = Duration::from_millis(80);
/// One unicast status. Matches the control loop's read cap, not the Python tool's
/// 12 ms, so a late HD1910 reply still counts.
const READ_DEADLINE: Duration = Duration::from_millis(30);
const DEFAULT_HZ: f64 = 10.0;

static RUNNING: AtomicBool = AtomicBool::new(true);

fn main() {
    if let Err(e) = run() {
        eprintln!("{e}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse()?;
    install_sigint();

    let mut port = serialport::new(&args.port, args.baud)
        .timeout(Duration::from_millis(2))
        .open()
        .map_err(|e| {
            format!(
                "open {}: {e}\n\
                 on the robot bus, stop robotd first (`sudo systemctl stop robotd`)",
                args.port
            )
        })?;

    let ids = discover(&mut *port);
    if ids.is_empty() {
        return Err("no Feetech device answered".into());
    }
    let mut kinds = Vec::with_capacity(ids.len());
    for &id in &ids {
        kinds.push(is_imu(&mut *port, id));
    }
    let names = ids
        .iter()
        .zip(kinds.iter())
        .map(|(id, imu)| {
            if *imu {
                format!("{id} IMU")
            } else {
                id.to_string()
            }
        })
        .collect::<Vec<_>>()
        .join(", ");
    println!("{} {} 8N1  {names}", args.port, args.baud);
    println!(
        "pos=counts  spd=rpm (clockwise negative)  cur=mA  tq=load%  volt=V  g=dps a=g  Ctrl-C quit"
    );

    let tty = io::stdout().is_terminal();
    if tty {
        print!("\x1b[?25l");
        io::stdout().flush()?;
    }
    let period = Duration::from_secs_f64(1.0 / args.hz);
    let mut shown = 0u32;
    let limit = if args.count == 0 && !tty {
        1
    } else {
        args.count
    };

    while RUNNING.load(Ordering::Relaxed) && (limit == 0 || shown < limit) {
        let started = Instant::now();
        let mut lines = Vec::with_capacity(ids.len());
        for (&id, &imu) in ids.iter().zip(kinds.iter()) {
            let text = if imu {
                match read_mem(&mut *port, id, SERVO_ADDR, IMU_LEN) {
                    Ok(block) => format!("id={id:<3} IMU {}", decode_imu(&block)),
                    Err(e) => e,
                }
            } else {
                match read_mem(&mut *port, id, SERVO_ADDR, SERVO_LEN) {
                    Ok(block) => format!("id={id:<3} {}", decode_servo(&block)),
                    Err(e) => e,
                }
            };
            lines.push(text);
        }
        paint(&lines, tty, shown == 0)?;
        shown += 1;
        if let Some(rest) = period.checked_sub(started.elapsed()) {
            thread::sleep(rest);
        }
    }

    if tty {
        print!("\x1b[?25h\n");
        io::stdout().flush()?;
    }
    Ok(())
}

struct Args {
    port: String,
    baud: u32,
    hz: f64,
    count: u32,
}

impl Args {
    fn parse() -> Result<Self, Box<dyn std::error::Error>> {
        let argv: Vec<String> = std::env::args().skip(1).collect();
        if argv.iter().any(|a| a == "-h" || a == "--help") || argv.is_empty() {
            eprint!("{USAGE}");
            std::process::exit(2);
        }
        let mut port = None;
        let mut baud = BAUD_RATE;
        let mut hz = DEFAULT_HZ;
        let mut count = 0u32;
        let mut i = 0;
        while i < argv.len() {
            let a = argv[i].as_str();
            let take = |i: &mut usize| -> Result<String, Box<dyn std::error::Error>> {
                *i += 1;
                argv.get(*i)
                    .cloned()
                    .ok_or_else(|| format!("{a} needs a value").into())
            };
            match a {
                "--port" => port = Some(take(&mut i)?),
                "--baud" => baud = take(&mut i)?.parse().map_err(|_| "bad --baud")?,
                "--hz" => hz = take(&mut i)?.parse().map_err(|_| "bad --hz")?,
                "--count" => count = take(&mut i)?.parse().map_err(|_| "bad --count")?,
                other if other.starts_with('-') => {
                    return Err(format!("unknown argument {other}").into());
                }
                other if port.is_none() => port = Some(other.to_string()),
                other => return Err(format!("unexpected argument {other}").into()),
            }
            i += 1;
        }
        let port = port.ok_or("pass a serial port, e.g. --port COM13")?;
        if !(hz.is_finite() && hz > 0.0) || count > 1_000_000 {
            return Err("hz must be positive and count must be >= 0".into());
        }
        Ok(Self {
            port,
            baud,
            hz,
            count,
        })
    }
}

fn discover(port: &mut dyn SerialPort) -> Vec<u8> {
    let mut ids = Vec::new();
    let packet = frame(BROADCAST_ID, PING, &[]);
    let raw = collect(port, &packet, BROADCAST_WINDOW);
    let payload = strip_echo(&raw, &packet);
    for item in parse_frames(&payload) {
        if item.error == 0
            && item.params.is_empty()
            && !ids.contains(&item.id)
            && ping(port, item.id)
        {
            ids.push(item.id);
        }
    }
    for &id in &JOINT_IDS {
        if !ids.contains(&id) && ping(port, id) {
            ids.push(id);
        }
    }
    if !ids.contains(&IMU_DXL_ID) {
        for _ in 0..IMU_PROBE_ATTEMPTS {
            if ping(port, IMU_DXL_ID) {
                ids.push(IMU_DXL_ID);
                break;
            }
        }
    }
    ids.sort_unstable();
    ids
}

fn ping(port: &mut dyn SerialPort, id: u8) -> bool {
    let packet = frame(id, PING, &[]);
    let raw = collect(port, &packet, READ_DEADLINE);
    parse_frames(&strip_echo(&raw, &packet))
        .into_iter()
        .any(|item| item.id == id && item.error == 0 && item.params.is_empty())
}

fn is_imu(port: &mut dyn SerialPort, id: u8) -> bool {
    let Ok(regs) = read_mem(port, id, IMU_MODEL_ADDR, 2) else {
        return false;
    };
    regs.len() == 2 && u16::from_le_bytes([regs[0], regs[1]]) == IMU_TO_SERVO_MODEL
}

fn read_mem(port: &mut dyn SerialPort, id: u8, address: u8, length: u8) -> Result<Vec<u8>, String> {
    let packet = frame(id, READ, &[address, length]);
    let _ = port.clear(serialport::ClearBuffer::Input);
    let _ = port.write_all(&packet);
    let _ = port.flush();
    let mut raw = Vec::new();
    let start = Instant::now();
    let mut buf = [0u8; 256];
    while start.elapsed() < READ_DEADLINE {
        match port.read(&mut buf) {
            Ok(0) | Err(_) => {}
            Ok(n) => raw.extend_from_slice(&buf[..n]),
        }
        for item in parse_frames(&strip_echo(&raw, &packet)) {
            if item.id == id && item.error == 0 && item.params.len() == length as usize {
                return Ok(item.params);
            }
        }
    }
    Err(format!("id {id} no reply"))
}

fn collect(port: &mut dyn SerialPort, packet: &[u8], window: Duration) -> Vec<u8> {
    let _ = port.clear(serialport::ClearBuffer::Input);
    let _ = port.write_all(packet);
    let _ = port.flush();
    let mut raw = Vec::new();
    let start = Instant::now();
    let mut buf = [0u8; 256];
    while start.elapsed() < window {
        match port.read(&mut buf) {
            Ok(0) | Err(_) => {}
            Ok(n) => raw.extend_from_slice(&buf[..n]),
        }
    }
    raw
}

fn strip_echo<'a>(raw: &'a [u8], packet: &[u8]) -> &'a [u8] {
    raw.strip_prefix(packet).unwrap_or(raw)
}

struct Status {
    id: u8,
    error: u8,
    params: Vec<u8>,
}

fn parse_frames(data: &[u8]) -> Vec<Status> {
    let mut frames = Vec::new();
    let mut i = 0;
    while i + 6 <= data.len() {
        if data[i] != 0xFF || data[i + 1] != 0xFF || data[i + 2] == 0xFF {
            i += 1;
            continue;
        }
        let length = data[i + 3] as usize;
        let total = 4 + length;
        if length < 2 || i + total > data.len() {
            i += 1;
            continue;
        }
        let body = &data[i + 2..i + total - 1];
        if checksum(body) != data[i + total - 1] {
            i += 1;
            continue;
        }
        frames.push(Status {
            id: data[i + 2],
            error: data[i + 4],
            params: data[i + 5..i + total - 1].to_vec(),
        });
        i += total;
    }
    frames
}

fn frame(id: u8, instruction: u8, params: &[u8]) -> Vec<u8> {
    let mut body = Vec::with_capacity(3 + params.len());
    body.push(id);
    body.push((params.len() + 2) as u8);
    body.push(instruction);
    body.extend_from_slice(params);
    let sum = checksum(&body);
    let mut packet = vec![0xFF, 0xFF];
    packet.extend(body);
    packet.push(sum);
    packet
}

fn checksum(body: &[u8]) -> u8 {
    !body.iter().fold(0u8, |sum, byte| sum.wrapping_add(*byte))
}

fn signed_mag(raw: u16, bit: u32) -> (i32, u16) {
    let magnitude = raw & ((1u16 << bit) - 1);
    let sign = if raw & (1u16 << bit) == 0 { 1 } else { -1 };
    (sign, magnitude)
}

fn decode_servo(block: &[u8]) -> String {
    if block.len() < SERVO_LEN as usize {
        return format!("short block ({} bytes)", block.len());
    }
    let position = u16::from_le_bytes([block[0], block[1]]);
    let speed = u16::from_le_bytes([block[2], block[3]]);
    let load = u16::from_le_bytes([block[4], block[5]]);
    let volts = voltage_to_volts(block[6]);
    let temp = block[7];
    let current = u16::from_le_bytes([block[13], block[14]]);
    let (speed_sign, speed_mag) = signed_mag(speed, 15);
    let rpm = -(speed_sign as f64) * f64::from(speed_mag) * RPM_PER_SPEED;
    let (load_sign, load_mag) = signed_mag(load, 10);
    let torque = f64::from(load_sign) * f64::from(load_mag) * 0.1;
    let current_ma = f64::from(current & 0x7FFF) * MA_PER_CURRENT;
    let pos = if position <= 4095 {
        position.to_string()
    } else {
        format!("{position:#06x}")
    };
    format!(
        "pos={pos} temp={temp}C volt={volts:.1}V spd={rpm:+.1}rpm cur={current_ma:.1}mA tq={torque:+.1}%"
    )
}

fn decode_imu(block: &[u8]) -> String {
    if block.len() < ACCEL_OFFSET + 6 {
        return format!("short imu block ({} bytes)", block.len());
    }
    let gyro = [
        i16::from_le_bytes([block[0], block[1]]),
        i16::from_le_bytes([block[2], block[3]]),
        i16::from_le_bytes([block[4], block[5]]),
    ];
    let packed = [
        u16::from_le_bytes([block[6], block[7]]),
        u16::from_le_bytes([block[8], block[9]]),
        u16::from_le_bytes([block[10], block[11]]),
    ];
    let accel = [
        i16::from_le_bytes([block[ACCEL_OFFSET], block[ACCEL_OFFSET + 1]]),
        i16::from_le_bytes([block[ACCEL_OFFSET + 2], block[ACCEL_OFFSET + 3]]),
        i16::from_le_bytes([block[ACCEL_OFFSET + 4], block[ACCEL_OFFSET + 5]]),
    ];
    let quat = if packed == [0, 0, 0] {
        "q ---".to_string()
    } else {
        let (x, y, z) = (half(packed[0]), half(packed[1]), half(packed[2]));
        let sumsq = x * x + y * y + z * z;
        let w = if sumsq <= 1.02 {
            (1.0 - sumsq.min(1.0)).sqrt()
        } else {
            f64::NAN
        };
        format!("q {x:+.3} {y:+.3} {z:+.3} {w:+.3}")
    };
    format!(
        "g {:+6.2} {:+6.2} {:+6.2} a {:+6.3} {:+6.3} {:+6.3} {quat}",
        f64::from(gyro[0]) * GYRO_DPS_PER_LSB,
        f64::from(gyro[1]) * GYRO_DPS_PER_LSB,
        f64::from(gyro[2]) * GYRO_DPS_PER_LSB,
        f64::from(accel[0]) * ACCEL_G_PER_LSB,
        f64::from(accel[1]) * ACCEL_G_PER_LSB,
        f64::from(accel[2]) * ACCEL_G_PER_LSB,
    )
}

/// IEEE 754 binary16 → f64. Same cases `imu.rs` pins.
fn half(bits: u16) -> f64 {
    let sign = if bits & 0x8000 != 0 { -1.0 } else { 1.0 };
    let exp = ((bits >> 10) & 0x1F) as i32;
    let frac = f64::from(bits & 0x3FF);
    match exp {
        0 => sign * frac * 2.0_f64.powi(-24),
        0x1F if frac == 0.0 => sign * f64::INFINITY,
        0x1F => f64::NAN,
        _ => sign * (1.0 + frac / 1024.0) * 2.0_f64.powi(exp - 15),
    }
}

fn paint(lines: &[String], tty: bool, first: bool) -> io::Result<()> {
    let mut out = io::stdout().lock();
    if tty && !first {
        write!(out, "\x1b[{}A", lines.len())?;
    }
    for line in lines {
        if tty {
            write!(out, "\r\x1b[2K{line}\n")?;
        } else {
            writeln!(out, "{line}")?;
        }
    }
    out.flush()
}

#[cfg(unix)]
fn install_sigint() {
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
    unsafe {
        let _ = SetConsoleCtrlHandler(Some(handler), 1);
    }
}

#[cfg(not(any(unix, windows)))]
fn install_sigint() {}

const USAGE: &str = "\
imu_servo_read — Feetech servos and imu_to_servo on one bus

Usage:
  imu_servo_read --port <uart> [--baud N] [--hz N] [--count N]
  imu_servo_read <uart>

--baud defaults to 1000000. --hz defaults to 10. --count 0 refreshes until Ctrl-C
on a terminal, and prints one snapshot when stdout is not a terminal.
ID 200 is pinged five times. Model 0x4D44 is the IMU; anything else is a servo.
Does not enable torque or write registers. Stop robotd before using the robot UART.

Windows:
  cargo run -p duck-control --example imu_servo_read -- --port COM13

Linux / the robot:
  sudo systemctl stop robotd
  cargo run -p duck-control --example imu_servo_read -- /dev/ttyACM0
";

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ping_frame_matches_the_manual() {
        assert_eq!(
            frame(0x01, PING, &[]),
            vec![0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB]
        );
    }

    #[test]
    fn read_frame_matches_the_manual() {
        assert_eq!(
            frame(0x01, READ, &[0x38, 0x02]),
            vec![0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x02, 0xBE]
        );
    }

    #[test]
    fn a_status_frame_round_trips() {
        let params = [0x11u8, 0x22];
        let mut body = vec![0x0A, (params.len() + 2) as u8, 0x00];
        body.extend_from_slice(&params);
        let mut packet = vec![0xFF, 0xFF];
        packet.extend_from_slice(&body);
        packet.push(checksum(&body));
        let frames = parse_frames(&packet);
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].id, 0x0A);
        assert_eq!(frames[0].error, 0);
        assert_eq!(frames[0].params, params);
    }

    #[test]
    fn clockwise_speed_is_negative_rpm() {
        let mut block = [0u8; SERVO_LEN as usize];
        block[2..4].copy_from_slice(&1000u16.to_le_bytes());
        let text = decode_servo(&block);
        assert!(text.contains("spd=-732.0rpm"), "{text}");
        block[2..4].copy_from_slice(&(0x8000u16 | 1000).to_le_bytes());
        let text = decode_servo(&block);
        assert!(text.contains("spd=+732.0rpm"), "{text}");
    }

    #[test]
    fn voltage_byte_is_tenths_of_a_volt() {
        let mut block = [0u8; SERVO_LEN as usize];
        block[6] = 74;
        let text = decode_servo(&block);
        assert!(text.contains("volt=7.4V"), "{text}");
    }

    #[test]
    fn a_zero_quaternion_is_not_printed_as_upright() {
        let mut block = vec![0u8; IMU_LEN as usize];
        block[0..2].copy_from_slice(&1000i16.to_le_bytes());
        let text = decode_imu(&block);
        assert!(text.contains("q ---"), "{text}");
        assert!(text.contains("g "), "{text}");
    }

    #[test]
    fn half_precision_one_is_one() {
        assert_eq!(half(0x3C00), 1.0);
    }

    #[test]
    fn model_constant_is_the_firmware_value() {
        assert_eq!(IMU_TO_SERVO_MODEL, 0x4D44);
        assert_eq!(IMU_PROBE_ATTEMPTS, 5);
        assert_eq!(IMU_DXL_ID, 200);
    }
}
