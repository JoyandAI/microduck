//! The Feetech bus driver (FT-HLS protocol): HD1910 servos, and `imu_to_servo` when it is present.
//!
//! The `HD1910` speaks the vendor's `FT-HLS` framing, a Dynamixel-v1-style layout:
//! `FF FF ID LEN INST PARAMS CHK` with `CHK = ~(ID + LEN + INST + PARAMS) & 0xFF` and
//! multi-byte values little-endian. We reuse `rustypot`'s `Sts3215Controller` **purely as a
//! transport layer** (`with_protocol_v1()`); its named register table differs from the
//! HLS at 34/35/36, so **every address is a constant defined here** and only the raw-data
//! methods (`sync_read_raw_data` / `sync_write_raw_data` / `read_raw_data` /
//! `write_raw_data` / `ping`) are used. rustypot's proto-1 `sync_read` is exactly "send the
//! 0x82 packet, then collect each id's reply frame in order", its `StatusPacketV1` checksum
//! is `~(ID+LEN+ERR+DATA) == CHK` and its params slice is `data[5..3+LEN]` — all aligned
//! with the FT-HLS spec, and a non-zero status byte is surfaced via `errors()` rather than
//! treated as a frame failure.
//!
//! The register addresses, units and conversion factors below are frozen against the
//! vendor memory table (`feetech_hls_memtable.md`) and the RL-side analysis
//! (`feetech_hls2915_servo_swap.md`). Values flagged as "pending calibration" (O3/O5) are
//! documented in place and must be backfilled on hardware before shipping.
//!
//! **This type is Feetech-only.** [`crate::bus::DynamixelIo`] is the XL330 path and is not
//! extended here. [`FeetechIo`] talks to the fifteen joints. At startup it pings
//! [`crate::model::IMU_DXL_ID`] (200) five times. An answer whose model number is `0x4D44`
//! is `imu_to_servo`. The tick then does two transactions: one `sync_read` of the fifteen
//! servos (address 56, 15 bytes), and a unicast read of the same block on ID 200. The
//! first 12 bytes of that block are the SFLP sample [`crate::imu::SflpDecoder`] already
//! decodes. Putting ID 200 inside the servo `sync_read` checksum-fails on this bus from
//! the first tick; the split is what `imu_servo_read` verified on `duck-57ed`
//! (2026-10-03). See `docs/robot/ttl_bus_communication.md`. The choice is fixed for the
//! life of this object. If ID 200 never answers, the tick read stays the fifteen joints
//! and the caller fills [`Sensors::imu`] from I²C [`crate::io::ImuIo`].

use std::f64::consts::PI;
use std::io::{Read, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use rustypot::servo::feetech::sts3215::Sts3215Controller;
use serialport::SerialPort;

use crate::bus::{STALE_RUN_WARN, StaleImuTracker};
use crate::hardware::HD1910_FIRMWARE;
use crate::imu::{IMU_BLOCK_LEN, ImuData, SflpDecoder};
use crate::io::{ImuIo, ImuStale, IoError, JointTargets, Result, RobotIo, Sensors, SlowSensors};
use crate::model::{BAUD_RATE, IMU_DXL_ID, JOINT_IDS, NUM_JOINTS};

/// HLS register addresses (DEC), from the vendor memory table. The HLS's *named* register
/// table differs from rustypot's STS3215 at 34/35/36, so these are the single source of
/// truth and only the raw-data methods touch the wire.
pub mod reg {
    pub const ID: u8 = 5; // 主 ID
    pub const BAUD: u8 = 6; // 波特率 (0 = 1 Mbps)
    pub const RESPONSE_LEVEL: u8 = 8; // 应答状态级别
    pub const ANGLE_LIMIT_MIN: u8 = 9; // 最小角度限制 (2B)
    pub const ANGLE_LIMIT_MAX: u8 = 11; // 最大角度限制 (2B)
    pub const MAX_TEMPERATURE: u8 = 13; // 最高温度上限
    pub const MAX_VOLTAGE: u8 = 14; // 最高输入电压 (0.1 V)
    pub const MIN_VOLTAGE: u8 = 15; // 最低输入电压 (0.1 V)
    pub const MAX_TORQUE: u8 = 16; // 最大扭矩 (0.1%)
    pub const DEADZONE_CW: u8 = 26; // 正向不灵敏区
    pub const DEADZONE_CCW: u8 = 27; // 负向不灵敏区
    pub const PROTECTION_CURRENT: u8 = 28; // 保护电流 (6.5 mA)
    pub const ANGULAR_RESOLUTION: u8 = 30; // 角度分辨率
    pub const POSITION_OFFSET: u8 = 31; // 位置偏移 (2B)
    pub const MODE: u8 = 33; // 运行模式 (0 位置伺服)
    pub const TORQUE_ENABLE: u8 = 40; // 扭矩开关
    pub const ACCEL: u8 = 41; // 加速度 (0 = 最大)
    pub const GOAL_POSITION: u8 = 42; // 目标位置 (2B)
    pub const GOAL_CURRENT: u8 = 44; // 目标电流 (2B, 6.5 mA)
    pub const GOAL_SPEED: u8 = 46; // 运行速度 (2B, 0.732 RPM)
    pub const TORQUE_LIMIT: u8 = 48; // 转矩限制 (2B, 0.1%)
    pub const KP: u8 = 50; // 位置环 P 系数
    pub const KD: u8 = 51; // 位置环 D 系数
    pub const KI: u8 = 52; // 位置环 I 系数 (位置模式无效)
    pub const LOCK: u8 = 55; // 锁标志 (EPROM 写保护)
    pub const PRESENT_POSITION: u8 = 56; // 当前位置 (2B)
    pub const PRESENT_SPEED: u8 = 58; // 当前速度 (2B)
    pub const PRESENT_LOAD: u8 = 60; // 当前负载 (2B, 0.1%)
    pub const PRESENT_VOLTAGE: u8 = 62; // 当前电压 (0.1 V)
    pub const PRESENT_TEMPERATURE: u8 = 63; // 当前温度 (°C)
    pub const MOVING: u8 = 66; // 移动标志
    pub const PRESENT_CURRENT: u8 = 69; // 当前电流 (2B, 6.5 mA)
    pub const CURRENT_BIAS: u8 = 73; // 电流偏置 (2B)
}

/// Start of the contiguous block read every tick: present_position (56) through
/// present_current (69), 15 bytes. This is the whole per-servo feedback the runtime needs.
const TICK_READ_ADDR: u8 = reg::PRESENT_POSITION; // 0x38
const TICK_READ_LEN: u8 = 15;

/// Status-packet `LEN` for a tick read: error byte + [`TICK_READ_LEN`] data + checksum.
/// A resync candidate has to carry this and no other length. A ping is 2, and a shifted
/// window often shows 32 or 255; accepting those would swallow the real frames that follow.
const TICK_STATUS_LEN: u8 = TICK_READ_LEN + 2;

/// `FF FF | ID | LEN | payload[LEN]`, with `payload` ending in the checksum.
const TICK_STATUS_FRAME: usize = 4 + TICK_STATUS_LEN as usize;

/// A healthy read completes well inside this. Capping it means a missing device costs a
/// bounded hiccup rather than stalling the loop on the serial driver's default.
const READ_TIMEOUT: Duration = Duration::from_millis(30);

/// How many times one tick's `sync_read` is attempted before the tick is declared failed.
///
/// **Measured on the K1 bench, 2026-09-20.** A `sync_read` is one 0x82 broadcast and fifteen
/// replies collected as a single byte stream; a byte lost to motor-current noise on the TTL
/// link shifts every following reply, so one dropped byte failed the whole transaction. The
/// control loop then had no retry, and `COAST_TICKS` (3) meant the robot was frozen for the
/// fourth tick onwards — with bursts measured at 40–90 consecutive failures, the loop spent
/// whole tenths of a second not driving at all: 50 Hz target, 22 Hz achieved, visible as a
/// stuttering gait.
///
/// A retry costs one more `READ_TIMEOUT` at worst. Before spending it, the failed attempt's
/// bytes are scanned for real frames ([`resync_tick_blocks`]): one lost byte makes rustypot
/// give up, but the other fourteen replies are usually still intact, and recovering them
/// keeps this tick. Only a scan that cannot assemble every joint arms [`RxFlushPort`] to
/// discard what is left before the next instruction.
const READ_ATTEMPTS: usize = 2;

/// Startup pings of ID 200. One ping can miss the board's reply; five separates "not on
/// this bus" from "the byte didn't land". After that the source does not change.
pub const IMU_PROBE_ATTEMPTS: usize = 5;

/// `imu_to_servo` model number, little-endian at address 3. Not an HD1910.
pub const IMU_TO_SERVO_MODEL: u16 = 0x4D44;
pub const IMU_MODEL_ADDR: u8 = 3;

/// How long a silent input means the late replies from a failed read have finished.
///
/// At 1 Mbps a byte is 10 µs, so 2 ms is two hundred byte-times of idle — longer than one
/// status frame, short enough to add to a retry without spending the tick.
const RX_DRAIN_QUIET: Duration = Duration::from_millis(2);

/// Cap on draining a failed read's leftovers. A line that never goes quiet must not hold
/// the control tick; past this the next instruction is sent anyway.
const RX_DRAIN_CAP: Duration = Duration::from_millis(8);

/// The plausible supply range for the servo rail, in volts.
///
/// The pack is 2S: `BATTERY_FULL_V` is 8.2 and `BATTERY_EMPTY_V` is 6.6, so anything outside
/// this band is not a battery, it is a misread. `present_voltage` is register 62 of a servo
/// whose reply shares the byte stream with fourteen others, and a shifted stream decodes to
/// plausible-looking nonsense — 4.0 V was observed on a pack sitting at 7.4 V. A reading this
/// far out is dropped rather than averaged in, because the average is what the daemon
/// compares against the empty threshold, and a wrong comparison there powers the board off.
pub const PLAUSIBLE_VOLTAGE_V: std::ops::RangeInclusive<f64> = 5.0..=9.0;

/// The fewest servo voltage readings that must be plausible for the average to be used.
///
/// Ten of fifteen: a majority with margin, so one or two dead or wedged servos cannot block
/// the battery estimate, while a bus that is broadly misreading cannot produce one either.
const MIN_PLAUSIBLE_VOLTAGE_READINGS: usize = 10;

/// HLSCL mode 4: pure position PD (HD1910 factory / Sim2Real). Register 33 sits in the
/// EEPROM window (5..=39); a locked EPROM silently drops the write — `hls_control` had to
/// unlock around this. Mode 0 is the HLS "angle servo with force limit"; BAM identified
/// the HD1910 in mode 4, so that is what we write.
const MODE_SERVO: u8 = HD1910_FIRMWARE.mode;

/// Production [`encode_write_pos_ex`] fields, from the HD1910 bring-up in `hls_control`.
///
/// Accel 0 = maximum. Goal speed 0 = maximum **steps/s** (not the 0.732 RPM present-speed
/// LSB). Torque is 0.1% LSB; 980 is the same cap as `max_torque` / `TORQUE_LIMIT`.
const WRITE_POS_EX_ACCEL: u8 = 0;
const WRITE_POS_EX_SPEED: u16 = 0;
const WRITE_POS_EX_TORQUE: u16 = 980;
const TORQUE_LIMIT_LSB: u16 = 980;
const PROTECTION_CURRENT_BYTES: [u8; 2] = HD1910_FIRMWARE.protection_current_lsb.to_le_bytes();

/// Position counts per revolution (4096), single-turn.
const POSITION_COUNTS_PER_REV: f64 = 4096.0;
const RAD_PER_POSITION_COUNT: f64 = 2.0 * PI / POSITION_COUNTS_PER_REV;

/// RPM per speed LSB, from the datasheet.
const RPM_PER_SPEED_COUNT: f64 = 0.732;
const RAD_PER_SEC_PER_RPM: f64 = 2.0 * PI / 60.0;

/// Milliamps per current LSB, from the datasheet.
const MA_PER_CURRENT_COUNT: f64 = 6.5;

/// Volts per voltage LSB (the byte is 0.1 V per count).
const VOLTS_PER_VOLTAGE_COUNT: f64 = 0.1;

/// Single-turn position range, 0..=4095. A count above this means BIT15 (the direction bit)
/// is set and the servo is reporting a multi-turn value, which this build does not use.
const MAX_POSITION_COUNT: u16 = 4095;

/// BIT15 = direction bit for the signed (speed/current) registers; low 15 bits are magnitude.
const BIT15: u16 = 0x8000;
/// Load register: BIT10 is the direction bit and the low 10 bits are the magnitude (monitor-only).
const LOAD_MAGNITUDE_MASK: u16 = 0x03FF;

// ── conversions (frozen) ─────────────────────────────────────────────────────
//
// Feetech's horn: increasing count is clockwise (datasheet "Clockwise 0→4095").
// XL330 Drive Mode 0 and the MJCF / MicroDuck_rl joint axes: positive radian is
// CCW from the horn. We negate the count↔radian map so a +rad command on HD1910
// is the same physical turn as on the original duck. Mid-count 2048 stays 0 rad.
// Do not "correct" this back to the vendor formula — that would invert every
// policy action.

/// Present-position counts → radians in `(-π, π]`.
///
/// Single-turn only: the count must be 0..=4095, which is exactly the range a servo in
/// single-turn mode reports (BIT15 clear). A value with BIT15 or higher bits set is a
/// multi-turn reading and returns `None` — the caller must not feed it through this mapping.
///
/// Sign is flipped relative to the raw encoder: count 0 is `+π`, count 4095 is just
/// above `−π`. Matches [`rad_to_position_count`].
pub fn position_to_rad(count: u16) -> Option<f64> {
    if count > MAX_POSITION_COUNT {
        return None;
    }
    Some(PI - count as f64 * RAD_PER_POSITION_COUNT)
}

/// Radians → goal-position counts, the inverse of [`position_to_rad`].
///
/// Single-turn positions (`(-π, π]`) map to `0..=4095`. Out-of-range inputs are clamped so a
/// command never wraps into a multi-turn value. Matches `(π − rad) · 4096 / 2π`.
/// Positive radian decreases the count (CCW on the Feetech horn).
pub fn rad_to_position_count(rad: f64) -> u16 {
    let count = (PI - rad) / RAD_PER_POSITION_COUNT;
    count.clamp(0.0, MAX_POSITION_COUNT as f64).round() as u16
}

/// HLSCL `WritePosEx`: one 7-byte block at ACCEL (41) =
/// `acc | pos_l pos_h | torque_l torque_h | speed_l speed_h`.
///
/// Split writes that only poke [`reg::GOAL_POSITION`] leave goal-torque (44) at 0 and do
/// not arm a move the way the HD1910 firmware expects — that is what `hls_control` found
/// on the robot, and it is why [`FeetechIo::write`] sends this block rather than 2 bytes.
pub fn encode_write_pos_ex(pos: u16, speed: u16, acc: u8, torque: u16) -> [u8; 7] {
    let p = pos.to_le_bytes();
    let t = torque.to_le_bytes();
    let s = speed.to_le_bytes();
    [acc, p[0], p[1], t[0], t[1], s[0], s[1]]
}

/// Present-speed counts → rad/s.
///
/// Sign-magnitude: BIT15 is the direction bit and the low 15 bits are the magnitude, matching
/// the register's `±32767` (0.732 RPM/LSB) range. Feetech's "forward" bit (BIT15 clear) is
/// clockwise, which is **negative** in the XL330 / MJCF convention used by
/// [`position_to_rad`].
pub fn speed_to_rad_s(raw: u16) -> f64 {
    let magnitude = (raw & !BIT15) as f64;
    let sign = if raw & BIT15 != 0 { 1.0 } else { -1.0 };
    sign * magnitude * RPM_PER_SPEED_COUNT * RAD_PER_SEC_PER_RPM
}

/// Present-current counts → mA.
///
/// Sign is dropped (BIT15 is the direction bit) — every consumer wants load, not direction,
/// and `Sensors::currents_ma` is documented as a magnitude. Bias (reg 73, O4) is not yet
/// subtracted.
pub fn current_to_ma(raw: u16) -> f64 {
    (raw & !BIT15) as f64 * MA_PER_CURRENT_COUNT
}

/// Present-voltage byte → volts (0.1 V per count).
pub fn voltage_to_volts(count: u8) -> f64 {
    count as f64 * VOLTS_PER_VOLTAGE_COUNT
}

/// The mean of the servo voltage readings that are physically possible, or `None`.
///
/// **This guard decides whether the robot has a battery estimate at all.** A servo's supply
/// voltage rides in the same reply block as its position and temperature, so a misaligned byte
/// stream turns it into nonsense that still looks like a number. Measured on the K1 bench with
/// a pack sitting at 7.40 V: the reported value fell to 6.757 V, then 6.411 V, then 4.01 V as
/// the bus failed — and 6.411 V is below `BATTERY_EMPTY_V` (6.6 V), so the daemon concluded the
/// pack was flat and powered the board off mid-stride. Dropping implausible readings instead of
/// averaging them keeps that comparison meaningful: a reported voltage is now either a real
/// battery or nothing at all.
///
/// Returns `None` when fewer than [`MIN_PLAUSIBLE_VOLTAGE_READINGS`] readings are usable, so
/// the caller reports "no estimate" rather than a number derived from wreckage.
pub fn plausible_voltage_average(volts: &[f64; NUM_JOINTS]) -> Option<f64> {
    let mut sum = 0.0;
    let mut count = 0usize;
    for &v in volts {
        if PLAUSIBLE_VOLTAGE_V.contains(&v) {
            sum += v;
            count += 1;
        }
    }
    if count < MIN_PLAUSIBLE_VOLTAGE_READINGS {
        return None;
    }
    Some(sum / count as f64)
}

/// Present-temperature byte → °C (already whole degrees).
pub fn temperature_to_c(count: u8) -> f64 {
    count as f64
}

/// Present-load counts → percent of commanded duty. BIT10 is the direction bit and the low
/// 10 bits are the magnitude. Monitor-only: nothing downstream consumes load today.
pub fn load_percent(raw: u16) -> f64 {
    (raw & LOAD_MAGNITUDE_MASK) as f64 * 0.1
}

// ── per-joint tick block ─────────────────────────────────────────────────────

/// Decoded per-joint feedback from the 15-byte tick block.
#[derive(Debug, Clone, Copy, PartialEq)]
struct JointFeedback {
    position_rad: f64,
    velocity_rad_s: f64,
    current_ma: f64,
    load_percent: f64,
    volts: f64,
    temp_c: f64,
}

/// Decode one servo's 15-byte tick block (registers 56..=70) into [`JointFeedback`].
///
/// Byte layout, little-endian per the HLS memory table:
/// ```text
///  0..2   present_position   (2B, direction BIT15)
///  2..4   present_speed      (2B, direction BIT15)
///  4..6   present_load       (2B, direction BIT10)
///  6      present_voltage    (1B, 0.1 V)
///  7      present_temperature(1B, deg C)
///  8      async write flag   (1B, not consumed)
///  9      servo status       (1B, not consumed)
///  10     moving             (1B, not consumed)
///  11..13 target position    (2B, not consumed)
///  13..15 present_current    (2B, direction BIT15)
/// ```
fn decode_joint_block(block: &[u8]) -> Result<JointFeedback> {
    if block.len() < TICK_READ_LEN as usize {
        return Err(IoError::ShortRead {
            what: "hls tick block",
            expected: TICK_READ_LEN as usize,
            got: block.len(),
        });
    }
    let position = u16::from_le_bytes([block[0], block[1]]);
    let speed = u16::from_le_bytes([block[2], block[3]]);
    let load = u16::from_le_bytes([block[4], block[5]]);
    let current = u16::from_le_bytes([block[13], block[14]]);

    let position_rad = position_to_rad(position).ok_or(IoError::Bus(format!(
        "present position {position} out of single-turn range (multi-turn reported)"
    )))?;

    Ok(JointFeedback {
        position_rad,
        velocity_rad_s: speed_to_rad_s(speed),
        current_ma: current_to_ma(current),
        load_percent: load_percent(load),
        volts: voltage_to_volts(block[6]),
        temp_c: temperature_to_c(block[7]),
    })
}

// ── startup / EEPROM assertion table ────────────────────────────────────────

/// A register the bus asserts (and corrects) at startup.
#[derive(Debug)]
pub struct RegisterExpectation {
    pub name: &'static str,
    pub addr: u8,
    pub len: u8,
    pub want: &'static [u8],
}

/// EEPROM registers asserted (and corrected) by [`FeetechIo::check_registers`].
///
/// These live in the EEPROM range; correcting them requires opening the write lock
/// (`reg::LOCK` = 0) and re-locking afterward. The SRAM registers that must be re-applied on
/// every boot (mode/accel/goal_speed/torque_limit) are written by [`FeetechIo::apply_startup`]
/// and are not listed here.
///
/// **Voltage window is the HD1910 spec (4.0–8.4 V), not the 12 V HLS placeholders.**
/// A 2S pack at 7.4 V would trip a 10 V minimum.
pub const HLS_EEPROM_EXPECTED: &[RegisterExpectation] = &[
    RegisterExpectation {
        name: "baud",
        addr: reg::BAUD,
        len: 1,
        want: &[0],
    }, // 0 = 1 Mbps
    RegisterExpectation {
        name: "response_level",
        addr: reg::RESPONSE_LEVEL,
        len: 1,
        want: &[1],
    },
    RegisterExpectation {
        name: "angle_limit_min",
        addr: reg::ANGLE_LIMIT_MIN,
        len: 2,
        want: &[0, 0],
    },
    RegisterExpectation {
        name: "angle_limit_max",
        addr: reg::ANGLE_LIMIT_MAX,
        len: 2,
        want: &[0xFF, 0x0F],
    }, // 4095
    RegisterExpectation {
        name: "max_temperature",
        addr: reg::MAX_TEMPERATURE,
        len: 1,
        want: &[70],
    },
    RegisterExpectation {
        name: "max_voltage",
        addr: reg::MAX_VOLTAGE,
        len: 1,
        want: &[HD1910_FIRMWARE.max_voltage_lsb],
    }, // 8.4 V
    RegisterExpectation {
        name: "min_voltage",
        addr: reg::MIN_VOLTAGE,
        len: 1,
        want: &[HD1910_FIRMWARE.min_voltage_lsb],
    }, // 4.0 V
    RegisterExpectation {
        name: "protection_current",
        addr: reg::PROTECTION_CURRENT,
        len: 2,
        want: &PROTECTION_CURRENT_BYTES,
    }, // 500 LSB = 3.25 A
    RegisterExpectation {
        name: "max_torque",
        addr: reg::MAX_TORQUE,
        len: 2,
        want: &[0xD4, 0x03],
    }, // 980 (0.1%)
    RegisterExpectation {
        name: "mode",
        addr: reg::MODE,
        len: 1,
        want: &[MODE_SERVO],
    },
];

// ── the bus ──────────────────────────────────────────────────────────────────

/// What [`discard_stale_input`] needs from a port. Split out so the drain can be tested
/// without a real tty: `clear` drops what the driver already buffered, then a short read
/// waits out bytes still on the wire.
trait InputDrain {
    fn clear_input(&mut self);
    fn io_timeout(&self) -> Duration;
    fn set_io_timeout(&mut self, timeout: Duration) -> bool;
    fn read_some(&mut self, buf: &mut [u8]) -> std::io::Result<usize>;
}

struct PortDrain<'a>(&'a mut dyn SerialPort);

impl InputDrain for PortDrain<'_> {
    fn clear_input(&mut self) {
        let _ = self.0.clear(serialport::ClearBuffer::Input);
    }

    fn io_timeout(&self) -> Duration {
        self.0.timeout()
    }

    fn set_io_timeout(&mut self, timeout: Duration) -> bool {
        self.0.set_timeout(timeout).is_ok()
    }

    fn read_some(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        self.0.read(buf)
    }
}

/// `~(ID + LEN + ERR + DATA) & 0xFF`, the FT-HLS status checksum.
fn hls_checksum(body: &[u8]) -> u8 {
    !body.iter().fold(0u8, |sum, byte| sum.wrapping_add(*byte))
}

/// `FF FF ID LEN=2 INST=0x0B CHK`. No parameters means "current position is the midpoint".
fn calibrate_mid_packet(id: u8) -> [u8; 6] {
    let body = [id, 2, 0x0B];
    [0xFF, 0xFF, id, 2, 0x0B, hls_checksum(&body)]
}

/// A status frame with error 0, after skipping the half-duplex echo of `echo`.
fn calibrate_status_ok(buf: &[u8], id: u8, echo: &[u8]) -> bool {
    buf.windows(6).any(|frame| {
        frame != echo
            && frame[0] == 0xFF
            && frame[1] == 0xFF
            && frame[2] == id
            && frame[3] == 2
            && frame[4] == 0
            && hls_checksum(&frame[2..5]) == frame[5]
    })
}

fn read_calibrate_status(port: &mut dyn SerialPort, id: u8, echo: &[u8]) -> bool {
    let saved = port.timeout();
    if port.set_timeout(READ_TIMEOUT).is_err() {
        return false;
    }
    let mut buf = Vec::new();
    let mut tmp = [0u8; 32];
    let start = Instant::now();
    let mut ok = false;
    while start.elapsed() < Duration::from_millis(80) {
        match port.read(&mut tmp) {
            Ok(0) | Err(_) => break,
            Ok(n) => {
                buf.extend_from_slice(&tmp[..n]);
                if calibrate_status_ok(&buf, id, echo) {
                    ok = true;
                    break;
                }
            }
        }
    }
    let _ = port.set_timeout(saved);
    ok
}

/// Pull joint replies back out of a stream rustypot has already given up on.
///
/// See [`resync_status_blocks`]. This is the joints-only scan: an `imu_to_servo` frame,
/// if one is even on the wire, is not part of a tick that did not ask for it.
#[cfg(test)]
fn resync_tick_blocks(buf: &[u8]) -> Option<Vec<Vec<u8>>> {
    resync_status_blocks(buf, &JOINT_IDS)
}

/// Tick `sync_read` ids: the fifteen joints. ID 200 is a unicast read, not a member of
/// this list. See the module comment.
#[cfg(test)]
fn tick_read_ids() -> Vec<u8> {
    JOINT_IDS.to_vec()
}

/// The SFLP sample inside a 15-byte tick block. Bytes 12..15 are a counter and status
/// flags; counting them would hide a frozen orientation behind a changing tail.
fn imu_sample_prefix(block: &[u8]) -> Option<[u8; IMU_BLOCK_LEN]> {
    if block.len() < IMU_BLOCK_LEN {
        return None;
    }
    let mut raw = [0u8; IMU_BLOCK_LEN];
    raw.copy_from_slice(&block[..IMU_BLOCK_LEN]);
    Some(raw)
}

/// Pull tick-read replies back out of a stream rustypot has already given up on.
///
/// A candidate starts at `FF FF`, names one of `ids`, and has [`TICK_STATUS_LEN`].
/// The checksum is what makes that boundary real: a mismatch, or an id that is not in
/// `ids`, advances one byte and the scan continues, so one lost byte does not discard the
/// frames after it. Bytes before the first real header are skipped. Returns register
/// blocks in `ids` order when each id appears once; a duplicate id means two transactions
/// got mixed, and that is not a tick we can trust.
fn resync_status_blocks(buf: &[u8], ids: &[u8]) -> Option<Vec<Vec<u8>>> {
    let mut found = vec![None; ids.len()];
    let mut i = 0;
    while i + TICK_STATUS_FRAME <= buf.len() {
        let head = &buf[i..];
        let slot = ids.iter().position(|&id| id == head[2]);
        if head[0] != 0xFF || head[1] != 0xFF || head[3] != TICK_STATUS_LEN || slot.is_none() {
            i += 1;
            continue;
        }
        let frame = &head[..TICK_STATUS_FRAME];
        if hls_checksum(&frame[2..TICK_STATUS_FRAME - 1]) != frame[TICK_STATUS_FRAME - 1] {
            i += 1;
            continue;
        }
        let slot = slot.expect("id was just found");
        if found[slot].is_some() {
            return None;
        }
        found[slot] = Some(frame[5..5 + TICK_READ_LEN as usize].to_vec());
        i += TICK_STATUS_FRAME;
    }
    found.into_iter().collect()
}

/// Read whatever a failed `sync_read` has not consumed yet, until the line is quiet.
///
/// These bytes are the rest of the same reply burst. They have to be scanned, not
/// `clear`ed: `clear` throws away frames that are only misaligned.
fn append_until_quiet(port: &mut dyn SerialPort, buf: &mut Vec<u8>) {
    let saved = port.timeout();
    if port.set_timeout(RX_DRAIN_QUIET).is_err() {
        return;
    }
    let mut tmp = [0u8; 128];
    let start = Instant::now();
    while start.elapsed() < RX_DRAIN_CAP {
        match port.read(&mut tmp) {
            Ok(0) | Err(_) => break,
            Ok(n) => buf.extend_from_slice(&tmp[..n]),
        }
    }
    let _ = port.set_timeout(saved);
}

/// Drop bytes left in the receive buffer by a failed `sync_read`, then read until the line
/// is quiet or [`RX_DRAIN_CAP`] elapses.
///
/// `clear` alone is not enough: replies already in flight arrive after it and become the
/// first bytes of the next transaction. The port's real timeout is restored before return
/// so the following status read still waits [`READ_TIMEOUT`], not the 2 ms quiet gap.
fn discard_stale_input(port: &mut dyn InputDrain) {
    let saved = port.io_timeout();
    if !port.set_io_timeout(RX_DRAIN_QUIET) {
        port.clear_input();
        return;
    }
    port.clear_input();
    let mut buf = [0u8; 128];
    let start = Instant::now();
    while start.elapsed() < RX_DRAIN_CAP {
        match port.read_some(&mut buf) {
            Ok(0) | Err(_) => break,
            Ok(_) => continue,
        }
    }
    let _ = port.set_io_timeout(saved);
}

/// Serial port wrapper that records a `sync_read` and can discard a stale buffer on the next write.
///
/// rustypot owns the port inside `Sts3215Controller` and does not give it back. [`RxCapture`]
/// is the copy of every byte rustypot already pulled, so a checksum failure can be scanned
/// for `FF FF` frames instead of thrown away. [`Self::discard_rx`] is the fallback: a scan
/// that cannot assemble every joint still has to empty the line before the retry's
/// instruction, and that write happens before the new replies are on the wire.
struct RxFlushPort {
    inner: Box<dyn SerialPort>,
    discard_rx: Arc<AtomicBool>,
    capture: Arc<RxCapture>,
}

/// Bytes rustypot has read since [`RxCapture::start`], shared with [`FeetechIo`].
struct RxCapture {
    on: AtomicBool,
    bytes: Mutex<Vec<u8>>,
}

impl RxCapture {
    fn new() -> Self {
        Self {
            on: AtomicBool::new(false),
            bytes: Mutex::new(Vec::new()),
        }
    }

    fn start(&self) {
        self.bytes.lock().unwrap_or_else(|e| e.into_inner()).clear();
        self.on.store(true, Ordering::Release);
    }

    fn stop(&self) {
        self.on.store(false, Ordering::Release);
    }

    fn push(&self, data: &[u8]) {
        if self.on.load(Ordering::Acquire) && !data.is_empty() {
            self.bytes
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .extend_from_slice(data);
        }
    }

    fn take(&self) -> Vec<u8> {
        std::mem::take(&mut *self.bytes.lock().unwrap_or_else(|e| e.into_inner()))
    }
}

impl RxFlushPort {
    fn discard_if_armed(&mut self) {
        if self.discard_rx.swap(false, Ordering::AcqRel) {
            discard_stale_input(&mut PortDrain(&mut *self.inner));
        }
    }
}

impl Read for RxFlushPort {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        let n = self.inner.read(buf)?;
        self.capture.push(&buf[..n]);
        Ok(n)
    }
}

impl Write for RxFlushPort {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        self.discard_if_armed();
        self.inner.write(buf)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        self.inner.flush()
    }
}

impl SerialPort for RxFlushPort {
    fn name(&self) -> Option<String> {
        self.inner.name()
    }

    fn baud_rate(&self) -> serialport::Result<u32> {
        self.inner.baud_rate()
    }

    fn data_bits(&self) -> serialport::Result<serialport::DataBits> {
        self.inner.data_bits()
    }

    fn flow_control(&self) -> serialport::Result<serialport::FlowControl> {
        self.inner.flow_control()
    }

    fn parity(&self) -> serialport::Result<serialport::Parity> {
        self.inner.parity()
    }

    fn stop_bits(&self) -> serialport::Result<serialport::StopBits> {
        self.inner.stop_bits()
    }

    fn timeout(&self) -> Duration {
        self.inner.timeout()
    }

    fn set_baud_rate(&mut self, baud_rate: u32) -> serialport::Result<()> {
        self.inner.set_baud_rate(baud_rate)
    }

    fn set_data_bits(&mut self, data_bits: serialport::DataBits) -> serialport::Result<()> {
        self.inner.set_data_bits(data_bits)
    }

    fn set_flow_control(
        &mut self,
        flow_control: serialport::FlowControl,
    ) -> serialport::Result<()> {
        self.inner.set_flow_control(flow_control)
    }

    fn set_parity(&mut self, parity: serialport::Parity) -> serialport::Result<()> {
        self.inner.set_parity(parity)
    }

    fn set_stop_bits(&mut self, stop_bits: serialport::StopBits) -> serialport::Result<()> {
        self.inner.set_stop_bits(stop_bits)
    }

    fn set_timeout(&mut self, timeout: Duration) -> serialport::Result<()> {
        self.inner.set_timeout(timeout)
    }

    fn write_request_to_send(&mut self, level: bool) -> serialport::Result<()> {
        self.inner.write_request_to_send(level)
    }

    fn write_data_terminal_ready(&mut self, level: bool) -> serialport::Result<()> {
        self.inner.write_data_terminal_ready(level)
    }

    fn read_clear_to_send(&mut self) -> serialport::Result<bool> {
        self.inner.read_clear_to_send()
    }

    fn read_data_set_ready(&mut self) -> serialport::Result<bool> {
        self.inner.read_data_set_ready()
    }

    fn read_ring_indicator(&mut self) -> serialport::Result<bool> {
        self.inner.read_ring_indicator()
    }

    fn read_carrier_detect(&mut self) -> serialport::Result<bool> {
        self.inner.read_carrier_detect()
    }

    fn bytes_to_read(&self) -> serialport::Result<u32> {
        self.inner.bytes_to_read()
    }

    fn bytes_to_write(&self) -> serialport::Result<u32> {
        self.inner.bytes_to_write()
    }

    fn clear(&self, buffer_to_clear: serialport::ClearBuffer) -> serialport::Result<()> {
        self.inner.clear(buffer_to_clear)
    }

    fn try_clone(&self) -> serialport::Result<Box<dyn SerialPort>> {
        Ok(Box::new(RxFlushPort {
            inner: self.inner.try_clone()?,
            discard_rx: Arc::clone(&self.discard_rx),
            capture: Arc::clone(&self.capture),
        }))
    }

    fn set_break(&self) -> serialport::Result<()> {
        self.inner.set_break()
    }

    fn clear_break(&self) -> serialport::Result<()> {
        self.inner.clear_break()
    }
}

pub struct FeetechIo {
    controller: Sts3215Controller,
    /// Set after a failed read that resync could not save. [`RxFlushPort`] consumes it
    /// on the next write.
    discard_rx: Arc<AtomicBool>,
    /// Copy of the bytes rustypot already consumed. See [`RxCapture`].
    capture: Arc<RxCapture>,
    /// Second handle on the same tty, used only after rustypot has returned, to read
    /// the reply bytes it left behind and rescan them.
    tail: Box<dyn SerialPort>,
    /// Joint ids in [`JOINT_IDS`] order. The tick `sync_read`, writes, and the
    /// position-only read use this list, so `imu_to_servo` is never in those transactions.
    ids: Vec<u8>,
    /// Set once in [`Self::open`]. Does not flip if the board appears or disappears later.
    imu_on_bus: bool,
    imu: SflpDecoder,
    stale_imu: StaleImuTracker,
    /// Last decoded sample. [`RobotIo::read`] fills this; [`ImuIo::read`] returns it.
    last_imu: ImuData,
    /// Supply voltage per joint captured in the last tick's block. Zero until a read happens.
    last_volts: [f64; NUM_JOINTS],
    /// Case temperature per joint from the last tick's block.
    last_temps_c: [f64; NUM_JOINTS],
}

/// What one startup probe of ID 200 found. A model other than [`IMU_TO_SERVO_MODEL`] is
/// retried: a shifted frame can look like a number, and five tries are what separate that
/// from a device that is genuinely something else.
enum BusImuProbe {
    Found,
    Other { model: u16 },
    Miss,
}

/// One servo, as a bench tool reads it. Degrees use the same count map as the control loop.
#[derive(Debug, Clone, serde::Serialize)]
pub struct BenchSample {
    pub id: u8,
    pub position: u16,
    pub degrees: Option<f64>,
    pub volts: f64,
    pub temperature_c: u8,
    pub load_percent: f64,
    pub kp: u8,
    pub kd: u8,
    pub torque: bool,
}

impl FeetechIo {
    /// Open the port and nothing else: no ping of all fifteen, no EEPROM rewrite.
    ///
    /// [`Self::open`] is what the control loop uses. This is for a bench page that talks to
    /// one servo while `robotd` is stopped — calling [`Self::open`] would refuse a missing
    /// joint and rewrite mode on every id it did find.
    pub fn open_bench(port: &str) -> Result<Self> {
        Self::attach(port)
    }

    /// Open the bus and apply the per-boot startup register writes.
    ///
    /// Verifies every servo answers, then writes the SRAM/operational registers and leaves
    /// torque off — nothing enables torque just because a process started.
    pub fn open(port: &str) -> Result<Self> {
        let mut io = Self::attach(port)?;
        io.ping_all()?;
        io.imu_on_bus = io.probe_bus_imu();
        io.apply_startup()?;
        Ok(io)
    }

    fn attach(port: &str) -> Result<Self> {
        let serial = serialport::new(port, BAUD_RATE)
            .timeout(READ_TIMEOUT)
            .open()
            .map_err(|e| IoError::Port {
                path: port.to_owned(),
                source: std::io::Error::other(e),
            })?;

        let discard_rx = Arc::new(AtomicBool::new(false));
        let capture = Arc::new(RxCapture::new());
        let tail = serial.try_clone().map_err(|e| IoError::Port {
            path: port.to_owned(),
            source: std::io::Error::other(e),
        })?;
        let serial = Box::new(RxFlushPort {
            inner: serial,
            discard_rx: Arc::clone(&discard_rx),
            capture: Arc::clone(&capture),
        });
        let controller = Sts3215Controller::new()
            .with_protocol_v1()
            .with_serial_port(serial);

        Ok(Self {
            controller,
            discard_rx,
            capture,
            tail,
            ids: JOINT_IDS.to_vec(),
            imu_on_bus: false,
            imu: SflpDecoder::default(),
            stale_imu: StaleImuTracker::default(),
            last_imu: ImuData::default(),
            last_volts: [0.0; NUM_JOINTS],
            last_temps_c: [0.0; NUM_JOINTS],
        })
    }

    /// Which of `ids` answer a ping. A miss discards a late reply so it cannot shift the next one.
    pub fn bench_scan(&mut self, ids: &[u8]) -> Vec<u8> {
        let mut online = Vec::new();
        for &id in ids {
            match self.controller.ping(id) {
                Ok(true) => online.push(id),
                _ => self.discard_rx.store(true, Ordering::Release),
            }
        }
        online
    }

    /// Position, supply, temperature, load and the position-loop gains of one servo.
    pub fn bench_read(&mut self, id: u8) -> Result<BenchSample> {
        let pos = self.read_u16(id, reg::PRESENT_POSITION)?;
        let volts = voltage_to_volts(self.read_u8(id, reg::PRESENT_VOLTAGE)?);
        let temperature_c = self.read_u8(id, reg::PRESENT_TEMPERATURE)?;
        let load = self.read_u16(id, reg::PRESENT_LOAD)?;
        Ok(BenchSample {
            id,
            position: pos,
            degrees: position_to_rad(pos).map(|r| r.to_degrees()),
            volts,
            temperature_c,
            load_percent: load_percent(load),
            kp: self.read_u8(id, reg::KP)?,
            kd: self.read_u8(id, reg::KD)?,
            torque: self.read_u8(id, reg::TORQUE_ENABLE)? != 0,
        })
    }

    /// Torque on, then one `WritePosEx` to `counts` (0–4095). Speed is capped so a bench
    /// command is a move, not a slam: 800 steps/s, accel 20, torque cap 50%.
    pub fn bench_goto(&mut self, id: u8, counts: u16) -> Result<()> {
        if counts > MAX_POSITION_COUNT {
            return Err(IoError::Bus(format!(
                "position {counts} is outside 0–{MAX_POSITION_COUNT}"
            )));
        }
        self.write_register(id, reg::TORQUE_ENABLE, &[1])?;
        let block = encode_write_pos_ex(counts, 800, 20, 500);
        self.controller
            .sync_write_raw_data(&[id], reg::ACCEL, &[block.to_vec()])
            .map_err(|e| IoError::Bus(format!("WritePosEx on {id}: {e}")))
    }

    /// Position-loop P and D (registers 50 and 51). SRAM, so the lock stays shut.
    pub fn bench_gains(&mut self, id: u8, kp: u8, kd: u8) -> Result<()> {
        if kp > 254 {
            return Err(IoError::Bus(format!("kp {kp} is above 254")));
        }
        self.write_register(id, reg::KP, &[kp])?;
        self.write_register(id, reg::KD, &[kd])?;
        Ok(())
    }

    /// Torque on or off for one servo. Off is how a joint is freed before it is picked up.
    pub fn bench_torque(&mut self, id: u8, on: bool) -> Result<()> {
        self.write_register(id, reg::TORQUE_ENABLE, &[u8::from(on)])
    }

    /// Change one servo's ID. EEPROM: unlock, write, lock the new id.
    ///
    /// Only one servo of `id` may be on the bus. `200` is the IMU node and is refused.
    pub fn bench_set_id(&mut self, id: u8, new_id: u8) -> Result<()> {
        if new_id == 0 || new_id == IMU_DXL_ID {
            return Err(IoError::Bus(format!(
                "id {new_id} is reserved (1–253, not {IMU_DXL_ID})"
            )));
        }
        self.write_register(id, reg::TORQUE_ENABLE, &[0])?;
        self.write_register(id, reg::LOCK, &[0])?;
        self.write_register(id, reg::ID, &[new_id])?;
        self.write_register(new_id, reg::LOCK, &[1])?;
        Ok(())
    }

    /// Present-position counts of one servo. A miss is that servo, not the whole bus.
    pub fn bench_position(&mut self, id: u8) -> Result<u16> {
        self.read_u16(id, reg::PRESENT_POSITION)
    }

    /// Feetech position-calibrate `0x0B` with no parameter: the position the horn is at
    /// becomes the midpoint (2048), which this runtime treats as 0 rad.
    ///
    /// Torque is cut first. Leaving it on would make the servo chase the old goal after
    /// the origin moves. The write is permanent; call it only when the joint is already
    /// at its mechanical zero.
    pub fn bench_set_mid(&mut self, id: u8) -> Result<u16> {
        self.write_register(id, reg::TORQUE_ENABLE, &[0])?;
        let packet = calibrate_mid_packet(id);
        let _ = self.tail.clear(serialport::ClearBuffer::Input);
        self.tail
            .write_all(&packet)
            .map_err(|e| IoError::Bus(format!("calibrate {id}: {e}")))?;
        self.tail
            .flush()
            .map_err(|e| IoError::Bus(format!("calibrate {id}: {e}")))?;
        if !read_calibrate_status(&mut *self.tail, id, &packet) {
            return Err(IoError::Bus(format!(
                "servo {id} did not accept position calibrate"
            )));
        }
        self.read_u16(id, reg::PRESENT_POSITION)
    }

    fn read_u8(&mut self, id: u8, addr: u8) -> Result<u8> {
        let bytes = self.read_register(id, addr, 1)?;
        bytes
            .first()
            .copied()
            .ok_or_else(|| IoError::Bus(format!("empty read of reg {addr} on {id}")))
    }

    fn read_u16(&mut self, id: u8, addr: u8) -> Result<u16> {
        let bytes = self.read_register(id, addr, 2)?;
        if bytes.len() != 2 {
            return Err(IoError::ShortRead {
                what: "register",
                expected: 2,
                got: bytes.len(),
            });
        }
        Ok(u16::from_le_bytes([bytes[0], bytes[1]]))
    }

    /// Whether startup found `imu_to_servo` at ID 200. Fixed for this object's life.
    pub fn imu_on_bus(&self) -> bool {
        self.imu_on_bus
    }

    /// Confirm every servo is present. One missing servo means a `sync_read` below would fail
    /// the whole transaction anyway, so it is better to say so at open.
    ///
    /// Only [`JOINT_IDS`]. ID 200 is probed afterwards, and a miss there is not a failed open.
    fn ping_all(&mut self) -> Result<()> {
        for &id in &JOINT_IDS {
            let alive = self
                .controller
                .ping(id)
                .map_err(|e| IoError::Bus(format!("ping {id}: {e}")))?;
            if !alive {
                return Err(IoError::Bus(format!("servo {id} did not answer ping")));
            }
        }
        Ok(())
    }

    /// Ping ID 200 up to [`IMU_PROBE_ATTEMPTS`] times. The first reply whose model is
    /// [`IMU_TO_SERVO_MODEL`] locks the bus IMU in; anything else leaves it off.
    ///
    /// Unicast, not broadcast. A broadcast ping is answered by every servo at once, and the
    /// bytes collide. A failed attempt arms a receive discard so a late reply is not the
    /// next ping's status packet.
    fn probe_bus_imu(&mut self) -> bool {
        let mut other_model = None;
        for attempt in 1..=IMU_PROBE_ATTEMPTS {
            match self.classify_imu_probe() {
                BusImuProbe::Found => {
                    tracing::info!(
                        id = IMU_DXL_ID,
                        attempt,
                        model = format!("{IMU_TO_SERVO_MODEL:#06x}"),
                        "imu_to_servo answered on the Feetech bus"
                    );
                    return true;
                }
                BusImuProbe::Other { model } => {
                    other_model = Some(model);
                    tracing::debug!(
                        attempt,
                        model = format!("{model:#06x}"),
                        "id 200 answered with an unexpected model"
                    );
                }
                BusImuProbe::Miss => {
                    tracing::debug!(attempt, "id 200 did not answer");
                }
            }
            self.discard_rx.store(true, Ordering::Release);
        }
        if let Some(model) = other_model {
            tracing::warn!(
                id = IMU_DXL_ID,
                model = format!("{model:#06x}"),
                expected = format!("{IMU_TO_SERVO_MODEL:#06x}"),
                "id 200 is not an imu_to_servo; IMU stays on I2C"
            );
        } else {
            tracing::info!(
                id = IMU_DXL_ID,
                attempts = IMU_PROBE_ATTEMPTS,
                "no imu_to_servo at id 200; IMU stays on I2C"
            );
        }
        false
    }

    fn classify_imu_probe(&mut self) -> BusImuProbe {
        match self.controller.ping(IMU_DXL_ID) {
            Ok(true) => {}
            Ok(false) => return BusImuProbe::Miss,
            Err(e) => {
                tracing::debug!(error = %e, "ping id 200 failed");
                return BusImuProbe::Miss;
            }
        }
        match self.read_register(IMU_DXL_ID, IMU_MODEL_ADDR, 2) {
            Ok(bytes) if bytes.len() == 2 => {
                let model = u16::from_le_bytes([bytes[0], bytes[1]]);
                if model == IMU_TO_SERVO_MODEL {
                    BusImuProbe::Found
                } else {
                    BusImuProbe::Other { model }
                }
            }
            Ok(bytes) => {
                tracing::debug!(got = bytes.len(), "short model read from id 200");
                BusImuProbe::Miss
            }
            Err(e) => {
                tracing::debug!(error = %e, "model read from id 200 failed");
                BusImuProbe::Miss
            }
        }
    }

    /// Write the operational registers this session needs. EEPROM persistence (angle limits,
    /// voltages, baud, **mode**) is [`FeetechIo::check_registers`]; mode is also forced here
    /// because a servo left in electric mode by `hls_control` must be a position servo
    /// before the first tick, and register 33 is in the EEPROM window — writing it while
    /// locked is a silent no-op.
    fn apply_startup(&mut self) -> Result<()> {
        for &id in &JOINT_IDS {
            // Torque off before touching mode, matching `hls_control::set_mode`.
            self.write_register(id, reg::TORQUE_ENABLE, &[0])?;
            self.write_register(id, reg::LOCK, &[0])?;
            self.write_register(id, reg::MODE, &[MODE_SERVO])?;
            self.write_register(id, reg::LOCK, &[1])?;
            self.write_register(id, reg::TORQUE_LIMIT, &TORQUE_LIMIT_LSB.to_le_bytes())?;
        }
        Ok(())
    }

    /// Assert — and correct — the EEPROM registers the control loop depends on.
    ///
    /// Each corrected register requires the write lock to be open: unlock (`reg::LOCK` = 0),
    /// compare/correct, then re-lock (`reg::LOCK` = 1). Returns how many needed fixing.
    pub fn check_registers(&mut self) -> Result<usize> {
        let mut fixed = 0;
        for &id in &JOINT_IDS {
            self.write_register(id, reg::LOCK, &[0])?;
            for exp in HLS_EEPROM_EXPECTED {
                let got = self.read_register(id, exp.addr, exp.len)?;
                if got == exp.want {
                    continue;
                }
                tracing::warn!(
                    id,
                    register = exp.name,
                    got = ?got,
                    want = ?exp.want,
                    "correcting HLS EEPROM register"
                );
                self.write_register(id, exp.addr, exp.want)?;
                fixed += 1;
            }
            self.write_register(id, reg::LOCK, &[1])?;
        }
        Ok(fixed)
    }

    /// Present positions only — a lighter read than [`RobotIo::read`], used once at startup
    /// to adopt the pose the robot is already in.
    pub fn present_positions(&mut self) -> Result<[f64; NUM_JOINTS]> {
        let blocks = self
            .controller
            .sync_read_raw_data(&self.ids, reg::PRESENT_POSITION, 2)
            .map_err(|e| IoError::Bus(format!("read present positions: {e}")))?;
        if blocks.len() != NUM_JOINTS {
            return Err(IoError::ShortRead {
                what: "present positions",
                expected: NUM_JOINTS,
                got: blocks.len(),
            });
        }
        let mut out = [0.0; NUM_JOINTS];
        for (joint, block) in blocks.iter().enumerate() {
            if block.len() != 2 {
                return Err(IoError::ShortRead {
                    what: "present position block",
                    expected: 2,
                    got: block.len(),
                });
            }
            let count = u16::from_le_bytes([block[0], block[1]]);
            out[joint] = position_to_rad(count).ok_or(IoError::Bus(format!(
                "present position {count} out of single-turn range"
            )))?;
        }
        Ok(out)
    }

    /// Torque on every servo — one transaction per joint, so not something to call per tick.
    pub fn set_torque(&mut self, on: bool) -> Result<()> {
        for &id in &JOINT_IDS {
            self.write_register(id, reg::TORQUE_ENABLE, &[if on { 1 } else { 0 }])?;
        }
        Ok(())
    }

    /// Ramp every joint from where it is now to `target`, linearly. Blocking and deliberately
    /// so — nothing else should talk to the bus while this runs.
    pub fn interpolate_to(
        &mut self,
        target: &[f64; NUM_JOINTS],
        duration: Duration,
        step: Duration,
    ) -> Result<()> {
        let start = self.present_positions()?;
        let steps = (duration.as_secs_f64() / step.as_secs_f64())
            .ceil()
            .max(1.0) as u32;
        for i in 1..=steps {
            let t = i as f64 / steps as f64;
            let mut next = [0.0; NUM_JOINTS];
            for j in 0..NUM_JOINTS {
                next[j] = start[j] + (target[j] - start[j]) * t;
            }
            self.write(&JointTargets::new(next))?;
            std::thread::sleep(step);
        }
        Ok(())
    }

    fn read_register(&mut self, id: u8, addr: u8, len: u8) -> Result<Vec<u8>> {
        self.controller
            .read_raw_data(id, addr, len)
            .map_err(|e| IoError::Bus(format!("read reg {addr} on {id}: {e}")))
    }

    fn write_register(&mut self, id: u8, addr: u8, data: &[u8]) -> Result<()> {
        self.controller
            .write_raw_data(id, addr, data.to_vec())
            .map_err(|e| IoError::Bus(format!("write reg {addr} on {id}: {e}")))
    }

    /// One `sync_read` of the fifteen servos, then decode them.
    ///
    /// ID 200 is not in this transaction. [`ImuIo::read`] fetches it with a unicast read
    /// of the same address and length. rustypot stops at the first bad frame; the bytes
    /// it already read, plus whatever is still in the tty, are scanned for `FF FF` /
    /// [`TICK_STATUS_LEN`] before this attempt is declared failed.
    fn read_once(&mut self) -> Result<Sensors> {
        self.capture.start();
        let result =
            self.controller
                .sync_read_raw_data(&self.ids, TICK_READ_ADDR, TICK_READ_LEN);
        self.capture.stop();
        match result {
            Ok(blocks) => self.decode_tick_blocks(&blocks),
            Err(e) => match self.recover_tick() {
                Ok(sensors) => Ok(sensors),
                Err(_) => Err(IoError::Bus(format!("hls servo sync_read: {e}"))),
            },
        }
    }

    /// Rescan a failed `sync_read` and keep the tick when every joint frame is in there.
    fn recover_tick(&mut self) -> Result<Sensors> {
        let mut buf = self.capture.take();
        append_until_quiet(&mut *self.tail, &mut buf);
        let blocks = resync_status_blocks(&buf, &self.ids)
            .ok_or_else(|| IoError::Bus("hls servo sync_read: resync found no full tick".into()))?;
        self.decode_tick_blocks(&blocks)
    }

    fn decode_tick_blocks(&mut self, blocks: &[Vec<u8>]) -> Result<Sensors> {
        if blocks.len() != NUM_JOINTS {
            return Err(IoError::ShortRead {
                what: "hls sync_read blocks",
                expected: NUM_JOINTS,
                got: blocks.len(),
            });
        }

        let mut sensors = Sensors::default();
        for (joint, block) in blocks.iter().enumerate() {
            if block.len() != TICK_READ_LEN as usize {
                return Err(IoError::ShortRead {
                    what: "hls motor block",
                    expected: TICK_READ_LEN as usize,
                    got: block.len(),
                });
            }
            let fb = decode_joint_block(block)?;
            sensors.positions[joint] = fb.position_rad;
            sensors.velocities[joint] = fb.velocity_rad_s;
            sensors.currents_ma[joint] = fb.current_ma;
            self.last_volts[joint] = fb.volts;
            self.last_temps_c[joint] = fb.temp_c;
        }
        Ok(sensors)
    }

    /// First 12 bytes of the 15-byte block at address 56. The tail is the board's counter
    /// and status; [`SflpDecoder`] does not look at them.
    fn ingest_imu_block(&mut self, block: &[u8]) -> Result<ImuData> {
        if block.len() != TICK_READ_LEN as usize {
            return Err(IoError::ShortRead {
                what: "imu_to_servo block",
                expected: TICK_READ_LEN as usize,
                got: block.len(),
            });
        }
        let raw = imu_sample_prefix(block).expect("length checked");
        let run = self.stale_imu.observe(&raw);
        if run == STALE_RUN_WARN || (run > STALE_RUN_WARN && run.is_multiple_of(500)) {
            tracing::warn!(
                consecutive = run,
                total = self.stale_imu.stale().total,
                "imu board has returned the same sample {run} reads running — orientation is frozen"
            );
        }
        let imu = self.imu.decode(&raw);
        self.last_imu = imu;
        Ok(imu)
    }
}

impl RobotIo for FeetechIo {
    /// One bus read, retried once.
    ///
    /// See [`READ_ATTEMPTS`]. A failed attempt first tries to rebuild the tick from the
    /// bytes already on hand. Only when that scan is short a joint does it arm a
    /// receive-buffer discard, consumed by the next write — the retry's instruction when
    /// another attempt remains, otherwise the next tick's first write. The last attempt's
    /// error is the one reported.
    fn read(&mut self) -> Result<Sensors> {
        let mut last = None;
        for _ in 0..READ_ATTEMPTS {
            match self.read_once() {
                Ok(sensors) => return Ok(sensors),
                Err(e) => {
                    self.discard_rx.store(true, Ordering::Release);
                    last = Some(e);
                }
            }
        }
        Err(last.expect("READ_ATTEMPTS is non-zero"))
    }

    fn write(&mut self, targets: &JointTargets) -> Result<()> {
        // One `WritePosEx` block per joint (accel + pos + torque + speed at ACCEL), not a
        // 2-byte GOAL_POSITION poke — that leaves goal-torque at 0 and the HD1910 does not
        // take the step. See [`encode_write_pos_ex`].
        let data: Vec<Vec<u8>> = targets
            .positions
            .iter()
            .map(|&rad| {
                encode_write_pos_ex(
                    rad_to_position_count(rad),
                    WRITE_POS_EX_SPEED,
                    WRITE_POS_EX_ACCEL,
                    WRITE_POS_EX_TORQUE,
                )
                .to_vec()
            })
            .collect();
        self.controller
            .sync_write_raw_data(&self.ids, reg::ACCEL, &data)
            .map_err(|e| IoError::Bus(format!("sync_write WritePosEx: {e}")))
    }

    fn set_torque(&mut self, on: bool) -> Result<()> {
        // The inherent method, which is what `robotd init` uses on the way down.
        FeetechIo::set_torque(self, on)
    }

    fn set_gain(&mut self, kp: u16) -> Result<()> {
        // `kp` is the raw register value for reg 50 (0..254). Kd/Ki are the BAM-identified
        // firmware values (40/0): position-mode Ki is inert, and the HD1910 factory D is 40
        // not 32. See `crate::hardware::HD1910_FIRMWARE`.
        const KD: u8 = HD1910_FIRMWARE.kd;
        const KI: u8 = HD1910_FIRMWARE.ki;
        // Register 50 is 0..=254. BAM identified Kp=32; clamp so a leftover 1023 from an
        // XL330 config cannot wrap.
        let kp_byte = kp.min(254) as u8;
        for &id in &JOINT_IDS {
            self.write_register(id, reg::KP, &[kp_byte])?;
            self.write_register(id, reg::KD, &[KD])?;
            self.write_register(id, reg::KI, &[KI])?;
        }
        Ok(())
    }

    /// Supply voltage and case temperatures from the last tick's block — no extra transaction.
    ///
    /// `volts` is the mean of readings inside [`PLAUSIBLE_VOLTAGE_V`], and only when at least
    /// [`MIN_PLAUSIBLE_VOLTAGE_READINGS`] servos reported one. A bus that is misreading therefore
    /// produces no battery estimate — the caller keeps its last good sample, instead of treating
    /// garbage as an empty pack and powering the board off. `temps_c` is per-joint.
    fn slow_sensors(&mut self) -> Result<SlowSensors> {
        let volts = plausible_voltage_average(&self.last_volts).ok_or(IoError::Bus(format!(
            "input voltage: fewer than {MIN_PLAUSIBLE_VOLTAGE_READINGS} of {NUM_JOINTS} servo \
             readings were in {PLAUSIBLE_VOLTAGE_V:?} V — the bus is misreading, so there is no \
             battery estimate to report"
        )))?;
        Ok(SlowSensors {
            volts,
            temps_c: self.last_temps_c,
        })
    }
}

/// The IMU sample, fetched by its own unicast read. A checksum failure here must not
/// fail the servo tick — the caller keeps the previous orientation.
impl ImuIo for FeetechIo {
    fn read(&mut self) -> Result<ImuData> {
        if !self.imu_on_bus {
            return Err(IoError::Bus("imu_to_servo was not found at startup".into()));
        }
        match self.read_register(IMU_DXL_ID, TICK_READ_ADDR, TICK_READ_LEN) {
            Ok(block) => self.ingest_imu_block(&block),
            Err(e) => {
                self.discard_rx.store(true, Ordering::Release);
                Err(e)
            }
        }
    }

    fn ready(&self) -> bool {
        self.imu_on_bus && self.imu.ready()
    }

    fn stale(&self) -> ImuStale {
        self.stale_imu.stale()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    // ── FT-HLS frame bits, checked against the protocol manual examples ─────
    //
    // rustypot frames these for us; we still verify the byte layout and checksum are the
    // FT-HLS ones (framing is the part most likely to drift if someone swaps the transport),
    // purely as an offline checksum exercise — no serial port is faked.

    /// The FT-HLS checksum: `~(ID + LEN + INST + PARAMS) & 0xFF`, summing over bytes.
    fn checksum(parts: &[u8]) -> u8 {
        let mut sum: u8 = 0;
        for &p in parts {
            sum = sum.wrapping_add(p);
        }
        !sum
    }

    /// Build an instruction frame `FF FF ID LEN INST PARAMS CHK`.
    fn instruction_frame(id: u8, instruction: u8, params: &[u8]) -> Vec<u8> {
        let length = (params.len() + 2) as u8;
        let mut frame = vec![0xFF, 0xFF, id, length, instruction];
        frame.extend_from_slice(params);
        let chk = checksum(&frame[2..]);
        frame.push(chk);
        frame
    }

    #[test]
    fn ping_frame_matches_protocol_manual_example() {
        // 协议手册 例1：PING ID=1 → FF FF 01 02 01 FB
        assert_eq!(
            instruction_frame(0x01, 0x01, &[]),
            vec![0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB]
        );
    }

    #[test]
    fn read_frame_matches_protocol_manual_example() {
        // 协议手册 例2：READ ID=1, addr 0x38, len 2 → FF FF 01 04 02 38 02 BE
        assert_eq!(
            instruction_frame(0x01, 0x02, &[0x38, 0x02]),
            vec![0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x02, 0xBE]
        );
    }

    // ── position conversions ─────────────────────────────────────────────────

    #[test]
    fn position_conversion_handles_bit15_and_sign() {
        assert_eq!(position_to_rad(0), Some(PI));
        assert!((position_to_rad(2048).unwrap() - 0.0).abs() < 1e-12); // 中位
        let max = 4095u16;
        let expected = PI - max as f64 * RAD_PER_POSITION_COUNT;
        assert!((position_to_rad(max).unwrap() - expected).abs() < 1e-12);
        // BIT15 (direction) set, or any multi-turn count, is out of single-turn range.
        assert_eq!(position_to_rad(0x8000), None);
        assert_eq!(position_to_rad(4096), None);
        assert_eq!(position_to_rad(0xFFFF), None);
    }

    /// Positive radian must turn CCW on the horn (decreasing Feetech count), matching XL330.
    #[test]
    fn positive_radian_decreases_count() {
        assert!(rad_to_position_count(0.2) < 2048);
        assert!(rad_to_position_count(-0.2) > 2048);
        assert!(position_to_rad(2048 - 100).unwrap() > 0.0);
        assert!(position_to_rad(2048 + 100).unwrap() < 0.0);
    }

    #[test]
    fn position_goal_round_trips() {
        for count in [0u16, 1, 1024, 2048, 3072, 4095] {
            let rad = position_to_rad(count).unwrap();
            assert_eq!(
                rad_to_position_count(rad),
                count,
                "count {count} did not round-trip"
            );
        }
    }

    // ── velocity / current / voltage / temperature ───────────────────────────

    #[test]
    fn speed_conversion_handles_bit15_and_sign() {
        let per_count = 0.732 * RAD_PER_SEC_PER_RPM;
        // BIT15 clear = Feetech clockwise = negative in the XL330 / MJCF convention.
        assert!((speed_to_rad_s(1) + per_count).abs() < 1e-12);
        assert!((speed_to_rad_s(0x8001) - per_count).abs() < 1e-12);
        assert_eq!(speed_to_rad_s(0), 0.0);
    }

    #[test]
    fn current_conversion_drops_direction_and_sign() {
        assert!((current_to_ma(1000) - 1000.0 * 6.5).abs() < 1e-9);
        // BIT15 direction set gives the same magnitude.
        assert!((current_to_ma(0x8000 | 1000) - 1000.0 * 6.5).abs() < 1e-9);
        assert_eq!(current_to_ma(0), 0.0);
    }

    #[test]
    fn voltage_and_temperature_conversions() {
        assert!((voltage_to_volts(140) - 14.0).abs() < 1e-12);
        assert!((voltage_to_volts(100) - 10.0).abs() < 1e-12);
        assert_eq!(temperature_to_c(70), 70.0);
    }

    // ── tick block parsing ───────────────────────────────────────────────────

    #[test]
    fn tick_block_parses_joint_fields() {
        let mut block = [0u8; TICK_READ_LEN as usize];
        block[0..2].copy_from_slice(&1024u16.to_le_bytes()); // pos → +π/2 (count < mid)
        block[2..4].copy_from_slice(&1000u16.to_le_bytes()); // speed, BIT15 clear = clockwise
        block[4..6].copy_from_slice(&500u16.to_le_bytes()); // load
        block[6] = 120; // 12.0 V
        block[7] = 45; // 45 °C
        block[8] = 0; // async flag
        block[9] = 0; // status
        block[10] = 1; // moving
        block[11..13].copy_from_slice(&2048u16.to_le_bytes()); // target
        block[13..15].copy_from_slice(&200u16.to_le_bytes()); // current

        let fb = decode_joint_block(&block).unwrap();
        assert!((fb.position_rad - (PI / 2.0)).abs() < 1e-9);
        assert!((fb.velocity_rad_s + 1000.0 * 0.732 * RAD_PER_SEC_PER_RPM).abs() < 1e-9);
        assert!((fb.current_ma - 200.0 * 6.5).abs() < 1e-9);
        assert!((fb.load_percent - 50.0).abs() < 1e-9); // 500 * 0.1
        assert!((fb.volts - 12.0).abs() < 1e-12);
        assert!((fb.temp_c - 45.0).abs() < 1e-12);
    }

    #[test]
    fn tick_block_rejects_short_input() {
        assert!(decode_joint_block(&[0u8; 10]).is_err());
    }

    #[test]
    fn tick_block_rejects_multi_turn_position() {
        let mut block = [0u8; TICK_READ_LEN as usize];
        block[0..2].copy_from_slice(&0x8000u16.to_le_bytes()); // BIT15 set
        assert!(decode_joint_block(&block).is_err());
    }

    // ── EEPROM expected-register table ───────────────────────────────────────

    #[test]
    fn eeprom_table_is_complete_and_consistent() {
        assert!(!HLS_EEPROM_EXPECTED.is_empty());
        let mut addrs: Vec<u8> = Vec::with_capacity(HLS_EEPROM_EXPECTED.len());
        for r in HLS_EEPROM_EXPECTED {
            // Everything here is a persisted EEPROM configuration register (5..=39).
            assert!(
                r.addr >= 5 && r.addr <= 39,
                "{} at {} outside EPROM range",
                r.name,
                r.addr
            );
            assert_eq!(r.len as usize, r.want.len(), "{} length mismatch", r.name);
            addrs.push(r.addr);
        }
        addrs.sort_unstable();
        addrs
            .windows(2)
            .for_each(|w| assert_ne!(w[0], w[1], "duplicate register address {}", w[0]));
    }

    #[test]
    fn eeprom_values_match_the_hls_defaults() {
        let expect = |addr: u8, want: &[u8]| {
            HLS_EEPROM_EXPECTED
                .iter()
                .any(|r| r.addr == addr && r.want == want)
        };
        assert!(expect(reg::BAUD, &[0])); // 0 = 1 Mbps
        assert!(expect(reg::RESPONSE_LEVEL, &[1]));
        assert!(expect(reg::ANGLE_LIMIT_MIN, &[0, 0]));
        assert!(expect(reg::ANGLE_LIMIT_MAX, &[0xFF, 0x0F])); // 4095
        assert!(expect(reg::MAX_TEMPERATURE, &[70]));
        assert!(expect(reg::MAX_VOLTAGE, &[HD1910_FIRMWARE.max_voltage_lsb]));
        assert!(expect(reg::MIN_VOLTAGE, &[HD1910_FIRMWARE.min_voltage_lsb]));
        assert!(expect(
            reg::PROTECTION_CURRENT,
            &HD1910_FIRMWARE.protection_current_lsb.to_le_bytes()
        ));
        assert!(expect(reg::MAX_TORQUE, &[0xD4, 0x03])); // 980 (0.1%)
        assert!(expect(reg::MODE, &[MODE_SERVO]));
        assert_eq!(MODE_SERVO, 4);
    }

    /// Pin the `WritePosEx` layout `hls_control` uses: 7 bytes at ACCEL, little-endian.
    #[test]
    fn write_pos_ex_is_acc_pos_torque_speed() {
        let block = encode_write_pos_ex(2048, 800, 20, 100);
        assert_eq!(block.len(), 7);
        assert_eq!(block[0], 20); // acc
        assert_eq!(&block[1..3], &2048u16.to_le_bytes()); // pos
        assert_eq!(&block[3..5], &100u16.to_le_bytes()); // torque
        assert_eq!(&block[5..7], &800u16.to_le_bytes()); // speed
        // Mid-count 2048 is 0 rad, which is what the bring-up parks at.
        assert_eq!(rad_to_position_count(0.0), 2048);
        let production = encode_write_pos_ex(
            rad_to_position_count(0.0),
            WRITE_POS_EX_SPEED,
            WRITE_POS_EX_ACCEL,
            WRITE_POS_EX_TORQUE,
        );
        assert_eq!(production[0], 0);
        assert_eq!(&production[1..3], &2048u16.to_le_bytes());
        assert_eq!(&production[3..5], &980u16.to_le_bytes());
        assert_eq!(&production[5..7], &0u16.to_le_bytes());
    }

    // ── the battery guard ───────────────────────────────────────────────────
    //
    // These pin the behaviour that the K1 bench falsified: a bus that is misreading must not
    // be able to produce a battery estimate, because that estimate is compared against
    // `BATTERY_EMPTY_V` and a wrong comparison there powers the board off.

    /// A healthy rail averages to the real voltage.
    #[test]
    fn a_healthy_rail_averages_to_its_voltage() {
        let volts = [7.4f64; NUM_JOINTS];
        let avg = plausible_voltage_average(&volts).expect("a healthy rail has an estimate");
        assert!((avg - 7.4).abs() < 1e-9, "got {avg}");
    }

    /// One or two servos reading nonsense is tolerated — the average is still the pack.
    #[test]
    fn a_few_implausible_readings_do_not_spoil_the_estimate() {
        let mut volts = [7.4f64; NUM_JOINTS];
        volts[0] = 0.0; // a servo that did not answer
        volts[1] = 213.0; // a decoded byte-shift
        volts[2] = 4.01; // the observed mid-failure value, below the empty threshold
        let avg = plausible_voltage_average(&volts).expect("12 good readings are enough");
        assert!((avg - 7.4).abs() < 1e-9, "got {avg}");
    }

    /// **What this filter can and cannot do**, pinned so the limit is explicit rather than
    /// assumed. A *uniform* misread — every servo reporting 6.411 V because the whole reply
    /// stream shifted the same way — is inside the plausible band and looks exactly like a
    /// deeply discharged pack. No value test can separate those two, which is why the real
    /// defence is `Safety::slow_sensors` refusing to report a battery estimate for a tick
    /// whose read was not clean. This filter only rejects readings that are impossible.
    #[test]
    fn a_uniform_misread_is_inside_the_band_and_is_the_read_gates_job() {
        let volts = [6.411f64; NUM_JOINTS]; // below BATTERY_EMPTY_V, yet plausible-looking
        assert!(
            plausible_voltage_average(&volts).is_some(),
            "6.411 V is a possible pack voltage, so the value filter cannot reject it"
        );
    }

    /// Values that are physically impossible are rejected: a reading outside the band cannot
    /// be a 2S pack, whatever the bus did.
    #[test]
    fn impossible_readings_are_rejected() {
        for bad in [0.0, 4.9, 9.01, 213.0, -7.4] {
            let mut volts = [7.4f64; NUM_JOINTS];
            for v in volts
                .iter_mut()
                .take(NUM_JOINTS - MIN_PLAUSIBLE_VOLTAGE_READINGS + 1)
            {
                *v = bad;
            }
            assert_eq!(
                plausible_voltage_average(&volts),
                None,
                "{bad} V was accepted as a battery reading"
            );
        }
    }

    /// Below the quorum there is no estimate, even if every reading is individually sane.
    #[test]
    fn too_few_readings_is_no_estimate() {
        let mut volts = [0.0f64; NUM_JOINTS];
        for v in volts.iter_mut().take(MIN_PLAUSIBLE_VOLTAGE_READINGS - 1) {
            *v = 7.4;
        }
        assert_eq!(plausible_voltage_average(&volts), None);
    }

    /// The retry count is load-bearing: one attempt is the behaviour that left the robot
    /// frozen for four ticks per dropped byte, and zero attempts would not compile a read.
    #[test]
    fn the_read_is_retried() {
        assert!(
            READ_ATTEMPTS >= 2,
            "a single attempt is the bug this guards"
        );
    }

    fn tick_frame(id: u8, mark: u8) -> Vec<u8> {
        let mut data = [0u8; TICK_READ_LEN as usize];
        data[0] = mark;
        let mut frame = vec![0xFF, 0xFF, id, TICK_STATUS_LEN, 0];
        frame.extend_from_slice(&data);
        let chk = hls_checksum(&frame[2..]);
        frame.push(chk);
        frame
    }

    fn all_tick_frames() -> Vec<u8> {
        let mut buf = Vec::new();
        for (i, &id) in JOINT_IDS.iter().enumerate() {
            buf.extend(tick_frame(id, i as u8 + 1));
        }
        buf
    }

    /// A byte in front of an otherwise intact burst is not a lost tick: the scan locks
    /// onto `FF FF` and still returns every joint, in [`JOINT_IDS`] order.
    #[test]
    fn resync_skips_leading_garbage_and_keeps_the_tick() {
        let mut buf = vec![0x00, 0xFF, 0x13];
        buf.extend(all_tick_frames());
        let blocks = resync_tick_blocks(&buf).expect("fifteen frames are in this buffer");
        assert_eq!(blocks.len(), NUM_JOINTS);
        for (i, block) in blocks.iter().enumerate() {
            assert_eq!(block.len(), TICK_READ_LEN as usize);
            assert_eq!(block[0], i as u8 + 1);
        }
    }

    /// A header whose length is not the tick reply must not swallow the real frames after it.
    #[test]
    fn resync_rejects_an_illegal_length() {
        let mut buf = vec![0xFF, 0xFF, JOINT_IDS[0], 32, 0, 1, 2, 3];
        buf.extend(all_tick_frames());
        let blocks = resync_tick_blocks(&buf).expect("illegal LEN is not a frame");
        assert_eq!(blocks[0][0], 1);
    }

    /// Frames may arrive in any order; the blocks come back in joint order.
    #[test]
    fn resync_orders_blocks_by_joint_id() {
        let mut buf = Vec::new();
        for (i, &id) in JOINT_IDS.iter().enumerate().rev() {
            buf.extend(tick_frame(id, i as u8 + 1));
        }
        let blocks = resync_tick_blocks(&buf).unwrap();
        assert_eq!(blocks[0][0], 1);
        assert_eq!(blocks[NUM_JOINTS - 1][0], NUM_JOINTS as u8);
    }

    /// One damaged frame means this buffer is not a full tick. The caller retries
    /// rather than inventing the missing joint.
    #[test]
    fn resync_needs_every_joint() {
        let mut buf = all_tick_frames();
        buf.drain(..TICK_STATUS_FRAME);
        assert!(resync_tick_blocks(&buf).is_none());
    }

    /// Two checksum-valid frames for one id are two bursts mixed together.
    #[test]
    fn resync_rejects_a_duplicate_id() {
        let mut buf = all_tick_frames();
        buf.extend(tick_frame(JOINT_IDS[0], 0xAB));
        assert!(resync_tick_blocks(&buf).is_none());
    }

    /// Startup asks ID 200 five times. A hit does not put that id on the servo `sync_read`.
    #[test]
    fn bus_imu_is_probed_five_times_and_left_off_the_servo_sync_read() {
        assert_eq!(IMU_PROBE_ATTEMPTS, 5);
        assert_eq!(IMU_TO_SERVO_MODEL, 0x4D44);
        assert_eq!(tick_read_ids(), JOINT_IDS.to_vec());
        assert!(!JOINT_IDS.contains(&IMU_DXL_ID));
    }

    /// A recovered tick is the fifteen joints. An IMU frame sitting in the same buffer
    /// is not one of them, and a buffer of only those joints is a full tick.
    #[test]
    fn resync_ignores_an_imu_frame_on_a_joint_tick() {
        let mut buf = tick_frame(IMU_DXL_ID, 0xC8);
        buf.extend(all_tick_frames());

        let joints = resync_tick_blocks(&buf).unwrap();
        assert_eq!(joints.len(), NUM_JOINTS);
        assert_eq!(joints[0][0], 1);
        assert_eq!(joints[NUM_JOINTS - 1][0], NUM_JOINTS as u8);

        assert!(resync_tick_blocks(&all_tick_frames()).is_some());
    }

    /// The unicast IMU read is the same 15-byte block the servos use. The sample the
    /// decoder sees is the first 12; the counter and status in the tail are not.
    #[test]
    fn the_read_tail_is_not_part_of_the_imu_sample() {
        let mut block = [0u8; TICK_READ_LEN as usize];
        block[6..8].copy_from_slice(&0x3000u16.to_le_bytes());
        block[12] = 0xAB;
        block[13] = 0x0F;
        let raw = imu_sample_prefix(&block).unwrap();
        assert_eq!(raw.len(), IMU_BLOCK_LEN);
        assert_eq!(&raw[6..8], &0x3000u16.to_le_bytes());

        let mut from_prefix = SflpDecoder::default();
        let mut from_twelve = SflpDecoder::default();
        let mut only_twelve = [0u8; IMU_BLOCK_LEN];
        only_twelve[6..8].copy_from_slice(&0x3000u16.to_le_bytes());
        assert_eq!(
            from_prefix.decode(&raw).quat,
            from_twelve.decode(&only_twelve).quat
        );
    }

    /// Bytes already buffered are dropped, bytes still arriving are read off, then the
    /// caller's timeout is put back. A retry that skipped this would checksum the leftover.
    #[test]
    fn a_failed_read_drains_buffered_and_inflight_bytes() {
        let mut port = FakeDrain {
            buffered: vec![0xFF, 0xFF, 12],
            inflight: vec![17, 1, 255],
            ..FakeDrain::default()
        };
        discard_stale_input(&mut port);
        assert_eq!(port.cleared, vec![0xFF, 0xFF, 12]);
        assert!(port.inflight.is_empty());
        assert_eq!(port.timeout, Duration::from_millis(30));
    }

    /// A line that keeps delivering bytes has to give up, with the real timeout restored.
    #[test]
    fn drain_stops_at_the_cap_when_the_line_stays_noisy() {
        let mut port = FakeDrain {
            buffered: vec![1],
            noisy: true,
            ..FakeDrain::default()
        };
        let start = Instant::now();
        discard_stale_input(&mut port);
        assert!(start.elapsed() < Duration::from_millis(500));
        assert_eq!(port.timeout, Duration::from_millis(30));
        assert!(port.reads > 1);
    }

    /// The armed flush runs before the instruction bytes, and only on that one write.
    #[test]
    fn an_armed_flush_discards_input_before_the_next_write() {
        let flag = Arc::new(AtomicBool::new(true));
        let state = Arc::new(std::sync::Mutex::new(FakePortState {
            buffered: vec![0xFF, 0x0C],
            inflight: vec![0x11],
            timeout: READ_TIMEOUT,
            ..FakePortState::default()
        }));
        let mut port = RxFlushPort {
            inner: Box::new(FakePort {
                state: Arc::clone(&state),
            }),
            discard_rx: Arc::clone(&flag),
            capture: Arc::new(RxCapture::new()),
        };
        port.write_all(&[0x82, 0x01]).unwrap();
        {
            let inner = state.lock().unwrap();
            assert_eq!(inner.cleared, vec![0xFF, 0x0C]);
            assert!(inner.inflight.is_empty());
            assert_eq!(inner.written, vec![0x82, 0x01]);
            assert_eq!(
                inner.events,
                vec![PortEvent::Clear, PortEvent::Write],
                "stale bytes have to leave before the retry's instruction"
            );
            assert_eq!(inner.timeout, READ_TIMEOUT);
        }
        assert!(!flag.load(Ordering::Acquire));

        port.write_all(&[0x03]).unwrap();
        let inner = state.lock().unwrap();
        assert_eq!(inner.cleared, vec![0xFF, 0x0C]);
        assert_eq!(inner.written, vec![0x82, 0x01, 0x03]);
        assert_eq!(
            inner.events,
            vec![PortEvent::Clear, PortEvent::Write, PortEvent::Write]
        );
    }

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum PortEvent {
        Clear,
        Write,
    }

    struct FakePortState {
        buffered: Vec<u8>,
        inflight: Vec<u8>,
        cleared: Vec<u8>,
        written: Vec<u8>,
        events: Vec<PortEvent>,
        timeout: Duration,
    }

    impl Default for FakePortState {
        fn default() -> Self {
            Self {
                buffered: Vec::new(),
                inflight: Vec::new(),
                cleared: Vec::new(),
                written: Vec::new(),
                events: Vec::new(),
                timeout: READ_TIMEOUT,
            }
        }
    }

    struct FakePort {
        state: Arc<std::sync::Mutex<FakePortState>>,
    }

    fn unused_port() -> serialport::Error {
        serialport::Error::new(serialport::ErrorKind::Unknown, "unused in this test")
    }

    impl Read for FakePort {
        fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
            let mut state = self.state.lock().unwrap();
            if state.inflight.is_empty() {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::TimedOut,
                    "line quiet",
                ));
            }
            let n = state.inflight.len().min(buf.len());
            buf[..n].copy_from_slice(&state.inflight[..n]);
            state.inflight.drain(..n);
            Ok(n)
        }
    }

    impl Write for FakePort {
        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
            let mut state = self.state.lock().unwrap();
            state.events.push(PortEvent::Write);
            state.written.extend_from_slice(buf);
            Ok(buf.len())
        }

        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }

    impl SerialPort for FakePort {
        fn name(&self) -> Option<String> {
            None
        }

        fn baud_rate(&self) -> serialport::Result<u32> {
            Err(unused_port())
        }

        fn data_bits(&self) -> serialport::Result<serialport::DataBits> {
            Err(unused_port())
        }

        fn flow_control(&self) -> serialport::Result<serialport::FlowControl> {
            Err(unused_port())
        }

        fn parity(&self) -> serialport::Result<serialport::Parity> {
            Err(unused_port())
        }

        fn stop_bits(&self) -> serialport::Result<serialport::StopBits> {
            Err(unused_port())
        }

        fn timeout(&self) -> Duration {
            self.state.lock().unwrap().timeout
        }

        fn set_baud_rate(&mut self, _baud_rate: u32) -> serialport::Result<()> {
            Err(unused_port())
        }

        fn set_data_bits(&mut self, _data_bits: serialport::DataBits) -> serialport::Result<()> {
            Err(unused_port())
        }

        fn set_flow_control(
            &mut self,
            _flow_control: serialport::FlowControl,
        ) -> serialport::Result<()> {
            Err(unused_port())
        }

        fn set_parity(&mut self, _parity: serialport::Parity) -> serialport::Result<()> {
            Err(unused_port())
        }

        fn set_stop_bits(&mut self, _stop_bits: serialport::StopBits) -> serialport::Result<()> {
            Err(unused_port())
        }

        fn set_timeout(&mut self, timeout: Duration) -> serialport::Result<()> {
            self.state.lock().unwrap().timeout = timeout;
            Ok(())
        }

        fn write_request_to_send(&mut self, _level: bool) -> serialport::Result<()> {
            Err(unused_port())
        }

        fn write_data_terminal_ready(&mut self, _level: bool) -> serialport::Result<()> {
            Err(unused_port())
        }

        fn read_clear_to_send(&mut self) -> serialport::Result<bool> {
            Err(unused_port())
        }

        fn read_data_set_ready(&mut self) -> serialport::Result<bool> {
            Err(unused_port())
        }

        fn read_ring_indicator(&mut self) -> serialport::Result<bool> {
            Err(unused_port())
        }

        fn read_carrier_detect(&mut self) -> serialport::Result<bool> {
            Err(unused_port())
        }

        fn bytes_to_read(&self) -> serialport::Result<u32> {
            Err(unused_port())
        }

        fn bytes_to_write(&self) -> serialport::Result<u32> {
            Err(unused_port())
        }

        fn clear(&self, buffer_to_clear: serialport::ClearBuffer) -> serialport::Result<()> {
            if buffer_to_clear == serialport::ClearBuffer::Input
                || buffer_to_clear == serialport::ClearBuffer::All
            {
                let mut state = self.state.lock().unwrap();
                state.events.push(PortEvent::Clear);
                let buffered = std::mem::take(&mut state.buffered);
                state.cleared.extend(buffered);
            }
            Ok(())
        }

        fn try_clone(&self) -> serialport::Result<Box<dyn SerialPort>> {
            Ok(Box::new(FakePort {
                state: Arc::clone(&self.state),
            }))
        }

        fn set_break(&self) -> serialport::Result<()> {
            Err(unused_port())
        }

        fn clear_break(&self) -> serialport::Result<()> {
            Err(unused_port())
        }
    }

    /// In-memory stand-in for [`InputDrain`]. `buffered` is what `clear` drops; `inflight`
    /// is what a following read still pulls off the wire.
    struct FakeDrain {
        buffered: Vec<u8>,
        inflight: Vec<u8>,
        timeout: Duration,
        cleared: Vec<u8>,
        noisy: bool,
        reads: u32,
    }

    impl Default for FakeDrain {
        fn default() -> Self {
            Self {
                buffered: Vec::new(),
                inflight: Vec::new(),
                timeout: Duration::from_millis(30),
                cleared: Vec::new(),
                noisy: false,
                reads: 0,
            }
        }
    }

    impl InputDrain for FakeDrain {
        fn clear_input(&mut self) {
            self.cleared.append(&mut self.buffered);
        }

        fn io_timeout(&self) -> Duration {
            self.timeout
        }

        fn set_io_timeout(&mut self, timeout: Duration) -> bool {
            self.timeout = timeout;
            true
        }

        fn read_some(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
            self.reads += 1;
            if self.noisy {
                buf[0] = 0xFF;
                return Ok(1);
            }
            if self.inflight.is_empty() {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::TimedOut,
                    "line quiet",
                ));
            }
            let n = self.inflight.len().min(buf.len());
            buf[..n].copy_from_slice(&self.inflight[..n]);
            self.inflight.drain(..n);
            Ok(n)
        }
    }

    #[test]
    fn calibrate_mid_packet_is_instruction_0x0b() {
        let packet = calibrate_mid_packet(20);
        assert_eq!(&packet[..5], &[0xFF, 0xFF, 20, 2, 0x0B]);
        assert_eq!(packet[5], hls_checksum(&[20, 2, 0x0B]));
    }

    #[test]
    fn calibrate_status_ignores_the_echo_and_a_fault() {
        let echo = calibrate_mid_packet(20);
        let mut ok = vec![0xFF, 0xFF, 20, 2, 0, hls_checksum(&[20, 2, 0])];
        assert!(!calibrate_status_ok(&echo, 20, &echo));
        assert!(calibrate_status_ok(&ok, 20, &echo));
        let mut both = echo.to_vec();
        both.extend_from_slice(&ok);
        assert!(calibrate_status_ok(&both, 20, &echo));
        ok[4] = 1;
        ok[5] = hls_checksum(&[20, 2, 1]);
        assert!(!calibrate_status_ok(&ok, 20, &echo));
    }
}
