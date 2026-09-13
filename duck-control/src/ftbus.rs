//! The Feetech bus driver (FT-HLS protocol): HD1910 servos today, a bus IMU later.
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
//! extended here. Today [`FeetechIo`] talks to the fifteen joints; [`crate::model::IMU_DXL_ID`]
//! (200) is reserved for a later Feetech IMU node on this same bus — the same shape the
//! original MicroDuck used on Dynamixel. Until that node exists, [`RobotIo::read`] leaves
//! `Sensors::imu` at its default and the caller fills it from I²C [`crate::io::ImuIo`] on
//! the same tick.

use std::f64::consts::PI;
use std::time::Duration;

use rustypot::servo::feetech::sts3215::Sts3215Controller;

use crate::hardware::HD1910_FIRMWARE;
use crate::imu::ImuData;
use crate::io::{IoError, JointTargets, Result, RobotIo, Sensors, SlowSensors};
use crate::model::{BAUD_RATE, JOINT_IDS, NUM_JOINTS};

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

/// A healthy read completes well inside this. Capping it means a missing device costs a
/// bounded hiccup rather than stalling the loop on the serial driver's default.
const READ_TIMEOUT: Duration = Duration::from_millis(30);

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

pub struct FeetechIo {
    controller: Sts3215Controller,
    /// Servo ids in [`crate::model::JOINT_IDS`] order — the order blocks come back in.
    ids: Vec<u8>,
    /// Supply voltage per joint captured in the last tick's block. Zero until a read happens.
    last_volts: [f64; NUM_JOINTS],
    /// Case temperature per joint from the last tick's block.
    last_temps_c: [f64; NUM_JOINTS],
}

impl FeetechIo {
    /// Open the bus and apply the per-boot startup register writes.
    ///
    /// Verifies every servo answers, then writes the SRAM/operational registers and leaves
    /// torque off — nothing enables torque just because a process started.
    pub fn open(port: &str) -> Result<Self> {
        let serial = serialport::new(port, BAUD_RATE)
            .timeout(READ_TIMEOUT)
            .open()
            .map_err(|e| IoError::Port {
                path: port.to_owned(),
                source: std::io::Error::other(e),
            })?;

        let controller = Sts3215Controller::new()
            .with_protocol_v1()
            .with_serial_port(serial);

        let mut io = Self {
            controller,
            ids: JOINT_IDS.to_vec(),
            last_volts: [0.0; NUM_JOINTS],
            last_temps_c: [0.0; NUM_JOINTS],
        };
        io.ping_all()?;
        io.apply_startup()?;
        Ok(io)
    }

    /// Confirm every servo is present. One missing servo means a `sync_read` below would fail
    /// the whole transaction anyway, so it is better to say so at open.
    ///
    /// Only [`JOINT_IDS`]. [`crate::model::IMU_DXL_ID`] (200) is reserved for a later
    /// Feetech IMU node and is not pinged yet.
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
}

impl RobotIo for FeetechIo {
    fn read(&mut self) -> Result<Sensors> {
        let blocks = self
            .controller
            .sync_read_raw_data(&self.ids, TICK_READ_ADDR, TICK_READ_LEN)
            .map_err(|e| IoError::Bus(format!("hls servo sync_read: {e}")))?;

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

        // The IMU no longer rides this bus; the caller fuses the separate ImuIo sample.
        sensors.imu = ImuData::default();
        Ok(sensors)
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
    /// `volts` is averaged over the joints that reported a positive reading; `temps_c` is
    /// per-joint. Before the first tick no cache exists, so this fails until the loop has read
    /// once (mirroring how a bus with nothing on it reports no voltage).
    fn slow_sensors(&mut self) -> Result<SlowSensors> {
        let mut volts_sum = 0.0;
        let mut count = 0usize;
        for &v in &self.last_volts {
            if v > 0.0 {
                volts_sum += v;
                count += 1;
            }
        }
        if count == 0 {
            return Err(IoError::ShortRead {
                what: "input voltage",
                expected: NUM_JOINTS,
                got: 0,
            });
        }
        Ok(SlowSensors {
            volts: volts_sum / count as f64,
            temps_c: self.last_temps_c,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
}
