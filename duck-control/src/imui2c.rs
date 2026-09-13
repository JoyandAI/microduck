//! LSM6DSV16X IMU on its own I²C bus, driven directly from Linux i2c-dev.
//!
//! This is the linux-only half of the IMU move described in
//! `docs/design/hd1910-i2c-imu-migration.md` §2.3: instead of the IMU riding the Dynamixel
//! bus as a 12-byte block read at address 124, the chip is now reached over `/dev/i2c-imu`
//! (the i2c4 bus) at 0x6A, and the host drives it with ST's official `lsm6dsv16x-rs`
//! driver. The chip's on-sensor SFLP fusion still ships a game-rotation quaternion and its
//! own gyro bias; the host only reads the FIFO and repacks the bytes into the same 12-byte
//! block [`crate::imu::SflpDecoder`] already knows how to decode. That decoder stays
//! byte-identical — the whole point of the split is that neither `imu.rs` nor any consumer
//! of [`Sensors::imu`] changes.
//!
//! The block layout `SflpDecoder` consumes (unchanged from the v2 board):
//!
//! | bytes | contents |
//! |---|---|
//! | 0..6  | gyro x/y/z, `i16` LE raw counts, ±500 dps |
//! | 6..12 | SFLP quaternion x/y/z as IEEE half-precision; `w = √(1 − x² − y² − z²)` |
//!
//! The only pieces this file owns are the two embedded-hal shims the driver needs — an
//! [`I2cDev`] that talks to the kernel's `I2C_RDWR` ioctl and a [`StdDelay`] that sleeps —
//! plus the FIFO → 12-byte-block assembly and the stale counting that used to live in
//! `bus.rs`. Everything else (memory-bank switching, register access for the FIFO and the
//! SFLP bank) is the driver's job, because DSV16X's SFLP registers live behind an
//! embedded-function page switch that is hand-rolling bait.
//!
//! `ImuBus` is the concrete [`crate::io::ImuIo`]: open the bus, and each [`ImuIo::read`]
//! drains whatever the FIFO has, keeps the latest gyro and latest quaternion, packs them
//! into a block, and hands it to `SflpDecoder`. A failed [`ImuIo::read`] must degrade the
//! tick, not fail it — that is a property of the caller, not of this type.

use std::path::{Path, PathBuf};

use crate::imu::ImuData;
#[cfg(target_os = "linux")]
use crate::imu::{IMU_BLOCK_LEN, SflpDecoder};
use crate::io::{ImuIo, ImuStale, IoError, Result};
#[cfg(target_os = "linux")]
use lsm6dsv16x_rs::blocking::prelude::Tag;

#[cfg(target_os = "linux")]
mod bus {
    use super::*;
    use embedded_hal::delay::DelayNs;
    use embedded_hal::i2c::Operation;
    use lsm6dsv16x_rs::blocking::prelude::*;
    use st_mems_bus::blocking::i2c::I2cBus;

    /// The concrete sensor: an `I2cDev` behind the driver's I2C bus wrapper, a host
    /// `StdDelay`, and the chip's main memory bank.
    pub(super) type Sensor = Lsm6dsv16x<I2cBus<I2cDev>, StdDelay, MainBank>;

    /// A Linux i2c-dev device, opened by path and driven with the `I2C_RDWR` ioctl.
    ///
    /// `embedded_hal::i2c::I2c` in 1.0 is a single trait whose only required method is
    /// `transaction`; `read`/`write`/`write_read` route through it. Implementing it here
    /// means a whole register transaction becomes one `I2C_RDWR` call carrying one
    /// `i2c_msg` per `Operation` — which is exactly the repeated-start behaviour the
    /// driver wants for a read of a register address.
    pub struct I2cDev {
        fd: libc::c_int,
    }

    impl I2cDev {
        /// Open the bus device file. Address selection is per-message, so no `I2C_SLAVE`
        /// is needed — each transaction carries its own 7-bit address.
        pub fn open(path: &Path) -> std::io::Result<Self> {
            let cpath = std::ffi::CString::new(path.as_os_str().as_encoded_bytes())
                .map_err(|_| std::io::Error::from(std::io::ErrorKind::InvalidInput))?;
            let fd = unsafe {
                libc::open(
                    cpath.as_ptr(),
                    libc::O_RDWR | libc::O_NONBLOCK | libc::O_CLOEXEC,
                )
            };
            if fd < 0 {
                return Err(std::io::Error::last_os_error());
            }
            Ok(Self { fd })
        }

        fn transaction_once(
            &mut self,
            address: u8,
            operations: &mut [Operation<'_>],
        ) -> std::result::Result<(), I2cError> {
            let mut msgs: Vec<I2cMsg> = Vec::with_capacity(operations.len());
            for op in operations {
                match op {
                    Operation::Read(buf) => msgs.push(I2cMsg {
                        addr: address as u16,
                        flags: I2C_M_RD,
                        len: buf.len() as u16,
                        buf: buf.as_mut_ptr(),
                    }),
                    Operation::Write(buf) => msgs.push(I2cMsg {
                        addr: address as u16,
                        flags: 0,
                        len: buf.len() as u16,
                        buf: buf.as_ptr() as *mut u8,
                    }),
                }
            }
            let mut data = I2cRdwrData {
                msgs: msgs.as_mut_ptr(),
                nmsgs: msgs.len() as u32,
            };
            // SAFETY: `data` points to a live, correctly-laid-out `i2c_rdwr_ioctl_data`
            // whose `msgs` array outlives the call; the kernel only reads from write
            // buffers and writes into the read buffers the caller owns for the call.
            let rc = unsafe { libc::ioctl(self.fd, I2C_RDWR, &mut data as *mut I2cRdwrData) };
            if rc < 0 {
                Err(I2cError(std::io::Error::last_os_error()))
            } else {
                Ok(())
            }
        }
    }

    impl Drop for I2cDev {
        fn drop(&mut self) {
            unsafe {
                libc::close(self.fd);
            }
        }
    }

    /// The error the bus reports to the driver.
    #[derive(Debug)]
    pub struct I2cError(pub std::io::Error);

    impl core::fmt::Display for I2cError {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            write!(f, "i2c: {}", self.0)
        }
    }

    impl embedded_hal::i2c::Error for I2cError {
        fn kind(&self) -> embedded_hal::i2c::ErrorKind {
            embedded_hal::i2c::ErrorKind::Other
        }
    }

    impl embedded_hal::i2c::ErrorType for I2cDev {
        type Error = I2cError;
    }

    const I2C_RDWR: libc::c_ulong = 0x0707;
    const I2C_M_RD: u16 = 0x0001;

    /// Linux `struct i2c_msg`, exactly the uapi layout.
    #[repr(C)]
    struct I2cMsg {
        addr: u16,
        flags: u16,
        len: u16,
        buf: *mut u8,
    }

    /// Linux `struct i2c_rdwr_ioctl_data`.
    #[repr(C)]
    struct I2cRdwrData {
        msgs: *mut I2cMsg,
        nmsgs: u32,
    }

    fn is_transient_i2c(err: &I2cError) -> bool {
        matches!(
            err.0.raw_os_error(),
            Some(libc::EIO | libc::EREMOTEIO | libc::EAGAIN | libc::ETIMEDOUT)
        )
    }

    impl embedded_hal::i2c::I2c for I2cDev {
        fn transaction(
            &mut self,
            address: u8,
            operations: &mut [Operation<'_>],
        ) -> std::result::Result<(), Self::Error> {
            // Instant NACKs are ordinary on this bus (~4–15% on the Pi bench; rarer
            // here). Three tries, matching `docs/design/imu-env-setup.md`. ENXIO is
            // "nobody home" and is not retried — `scan_lsm6dsv16x` walks empty
            // addresses on purpose.
            const ATTEMPTS: u32 = 3;
            let mut attempt = 0u32;
            loop {
                match self.transaction_once(address, operations) {
                    Ok(()) => return Ok(()),
                    Err(e) => {
                        attempt += 1;
                        if attempt >= ATTEMPTS || !is_transient_i2c(&e) {
                            return Err(e);
                        }
                        std::thread::sleep(std::time::Duration::from_micros(200));
                    }
                }
            }
        }
    }

    /// A host [`DelayNs`] — the driver wants a timer for its reset handshake.
    pub struct StdDelay;

    impl DelayNs for StdDelay {
        fn delay_ns(&mut self, ns: u32) {
            std::thread::sleep(std::time::Duration::from_nanos(ns as u64));
        }
    }

    pub(super) fn drv_err<E: core::fmt::Debug>(e: E) -> IoError {
        IoError::Bus(format!("lsm6dsv16x: {e:?}"))
    }

    /// Map a config byte to the driver's 7-bit address. Only the two the chip straps.
    pub(super) fn i2c_address(addr: u8) -> Option<I2CAddress> {
        match addr {
            0x6A => Some(I2CAddress::I2cAddL),
            0x6B => Some(I2CAddress::I2cAddH),
            _ => None,
        }
    }

    /// Bring the chip up, in the order the migration contract fixes.
    pub(super) fn init(sensor: &mut Sensor) -> Result<()> {
        // The chip needs a moment after power-up before it answers the ID register.
        sensor.tim.delay_ms(5);

        let id = sensor.device_id_get().map_err(drv_err)?;
        if id != ID {
            return Err(IoError::Bus(format!(
                "who_am_i is {id:#04x}, expected {ID:#04x} — wrong IMU or bus"
            )));
        }

        // Restore the control registers, then wait for the reset to land.
        sensor.reset_set(Reset::RestoreCtrlRegs).map_err(drv_err)?;
        let mut rst = Reset::RestoreCtrlRegs;
        let mut attempts = 0;
        while rst != Reset::Ready {
            rst = sensor.reset_get().map_err(drv_err)?;
            attempts += 1;
            if attempts > 500 {
                return Err(IoError::Bus("reset never reached Ready".into()));
            }
            sensor.tim.delay_ms(2);
        }

        // Block Data Update so a read never straddles a register refresh.
        sensor.block_data_update_set(1).map_err(drv_err)?;

        // ±4 g, and ±500 dps so the raw counts match `imu.rs::GYRO_RAD_PER_LSB`.
        sensor
            .xl_full_scale_set(XlFullScale::_4g)
            .map_err(drv_err)?;
        sensor
            .gy_full_scale_set(GyFullScale::_500dps)
            .map_err(drv_err)?;

        // 120 Hz on XL+GY and on the SFLP output.
        sensor.xl_data_rate_set(Odr::_120hz).map_err(drv_err)?;
        sensor.gy_data_rate_set(Odr::_120hz).map_err(drv_err)?;
        sensor
            .sflp_data_rate_set(SflpDataRate::_120hz)
            .map_err(drv_err)?;

        // FIFO: a watermark below which the status won't flag, batch the game rotation
        // (gravity/gbias left off, or the block would carry more than the 12 bytes the
        // decoder expects), and stream to the FIFO continuously.
        sensor.fifo_watermark_set(8).map_err(drv_err)?;
        let fifo_sflp = FifoSflpRaw {
            game_rotation: 1,
            ..Default::default()
        };
        sensor.fifo_sflp_batch_set(fifo_sflp).map_err(drv_err)?;
        // SFLP tags alone are not a 12-byte block. Without a gyro batch rate the FIFO
        // never carries `GyNcTag`, `ImuIo::read` never packs, and `ready()` stays false
        // on a live chip — the decoder keeps returning the upright default.
        sensor
            .fifo_gy_batch_set(FifoBatch::_120hz)
            .map_err(drv_err)?;
        sensor
            .fifo_mode_set(FifoMode::StreamMode)
            .map_err(drv_err)?;

        // Turn the SFLP game-rotation output on, with a zero gyro-bias offset.
        sensor.sflp_game_rotation_set(1).map_err(drv_err)?;
        let gbias = SflpGbias {
            gbias_x: 0.0,
            gbias_y: 0.0,
            gbias_z: 0.0,
        };
        sensor.sflp_game_gbias_set(&gbias).map_err(drv_err)?;

        Ok(())
    }

    /// The 12-byte block [`SflpDecoder`] consumes: gyro (3×`i16` LE) then quaternion
    /// (3×`u16` LE half).
    pub(super) fn pack_12(gyro: [i16; 3], quat: [u16; 3]) -> [u8; IMU_BLOCK_LEN] {
        let mut block = [0u8; IMU_BLOCK_LEN];
        for (i, g) in gyro.iter().enumerate() {
            block[i * 2..i * 2 + 2].copy_from_slice(&g.to_le_bytes());
        }
        for (i, q) in quat.iter().enumerate() {
            block[6 + i * 2..6 + i * 2 + 2].copy_from_slice(&q.to_le_bytes());
        }
        block
    }

    /// Pull the *last* gyro and *last* SFLP quaternion out of a stream of FIFO entries and
    /// pack them into one block. Kept pure (and generic over `[u8; 6]` payloads) so the
    /// tag→block assembly is testable without a device.
    #[cfg(test)]
    pub(super) fn assemble_block(entries: &[(Tag, [u8; 6])]) -> Option<[u8; IMU_BLOCK_LEN]> {
        let mut gyro: Option<[i16; 3]> = None;
        let mut quat: Option<[u16; 3]> = None;
        for (tag, data) in entries {
            match tag {
                Tag::GyNcTag => {
                    gyro = Some([
                        i16::from_le_bytes([data[0], data[1]]),
                        i16::from_le_bytes([data[2], data[3]]),
                        i16::from_le_bytes([data[4], data[5]]),
                    ]);
                }
                Tag::SflpGameRotationVectorTag => {
                    quat = Some([
                        u16::from_le_bytes([data[0], data[1]]),
                        u16::from_le_bytes([data[2], data[3]]),
                        u16::from_le_bytes([data[4], data[5]]),
                    ]);
                }
                _ => {}
            }
        }
        Some(pack_12(gyro?, quat?))
    }

    /// Byte-identical repeat counting, exactly the semantics `ImuStale` documents: a run is
    /// the number of consecutive blocks equal to their predecessor, and a total is the
    /// cumulative count since startup. Resetting a block breaks the run but not the total.
    #[derive(Default)]
    pub(super) struct StaleImuTracker {
        total: u64,
        run: u64,
        last: Option<[u8; IMU_BLOCK_LEN]>,
    }

    impl StaleImuTracker {
        pub(super) fn observe(&mut self, block: &[u8; IMU_BLOCK_LEN]) {
            if self.last.as_ref() == Some(block) {
                self.total += 1;
                self.run += 1;
            } else {
                self.run = 0;
                self.last = Some(*block);
            }
        }

        pub(super) fn get(&self) -> ImuStale {
            ImuStale {
                total: self.total,
                run: self.run,
            }
        }
    }
}

/// The linux driver: an open sensor plus the decode state and stale counting.
#[cfg(target_os = "linux")]
pub struct ImuBus {
    sensor: bus::Sensor,
    decoder: SflpDecoder,
    stale: bus::StaleImuTracker,
    /// Latest gyro raw counts seen, so a block can be assembled even when this tick only
    /// delivered one of the two tag kinds.
    last_gyro: Option<[i16; 3]>,
    /// Latest quaternion halves seen.
    last_quat: Option<[u16; 3]>,
    /// The last decoded sample, returned unchanged on a tick the FIFO had nothing new.
    last_imu: ImuData,
}

#[cfg(target_os = "linux")]
impl ImuBus {
    /// Open and initialise the IMU at `address` (0x6A or 0x6B) on the `bus_path` device.
    pub fn open(bus_path: &Path, address: u8) -> Result<Self> {
        let dev = bus::I2cDev::open(bus_path).map_err(|source| IoError::Port {
            path: bus_path.display().to_string(),
            source,
        })?;
        let addr = bus::i2c_address(address).ok_or_else(|| {
            IoError::Bus(format!(
                "bad IMU address {address:#04x}, expected 0x6A/0x6B"
            ))
        })?;

        let mut sensor = lsm6dsv16x_rs::blocking::Lsm6dsv16x::new_i2c(dev, addr, bus::StdDelay);
        bus::init(&mut sensor)?;

        Ok(Self {
            sensor,
            decoder: SflpDecoder::default(),
            stale: bus::StaleImuTracker::default(),
            last_gyro: None,
            last_quat: None,
            last_imu: ImuData::default(),
        })
    }

    /// Read WHO_AM_I (0x0F) without resetting or configuring the chip.
    ///
    /// LSM6DSV16X is `0x70`. Use this to pick a bus: opening [`ImuBus::open`] on a
    /// different device at 0x6A would go on to reset it.
    pub fn who_am_i(bus_path: &Path, address: u8) -> Result<u8> {
        let dev = bus::I2cDev::open(bus_path).map_err(|source| IoError::Port {
            path: bus_path.display().to_string(),
            source,
        })?;
        let addr = bus::i2c_address(address).ok_or_else(|| {
            IoError::Bus(format!(
                "bad IMU address {address:#04x}, expected 0x6A/0x6B"
            ))
        })?;
        let mut sensor = lsm6dsv16x_rs::blocking::Lsm6dsv16x::new_i2c(dev, addr, bus::StdDelay);
        std::thread::sleep(std::time::Duration::from_millis(5));
        sensor.device_id_get().map_err(bus::drv_err)
    }
}

#[cfg(target_os = "linux")]
impl ImuIo for ImuBus {
    fn read(&mut self) -> Result<ImuData> {
        let status = self.sensor.fifo_status_get().map_err(bus::drv_err)?;

        let mut new_data = false;
        for _ in 0..status.fifo_level {
            let raw = self.sensor.fifo_out_raw_get().map_err(bus::drv_err)?;
            match raw.tag {
                Tag::GyNcTag => {
                    self.last_gyro = Some([
                        i16::from_le_bytes([raw.data[0], raw.data[1]]),
                        i16::from_le_bytes([raw.data[2], raw.data[3]]),
                        i16::from_le_bytes([raw.data[4], raw.data[5]]),
                    ]);
                    new_data = true;
                }
                Tag::SflpGameRotationVectorTag => {
                    self.last_quat = Some([
                        u16::from_le_bytes([raw.data[0], raw.data[1]]),
                        u16::from_le_bytes([raw.data[2], raw.data[3]]),
                        u16::from_le_bytes([raw.data[4], raw.data[5]]),
                    ]);
                    new_data = true;
                }
                _ => {}
            }
        }

        if new_data && let (Some(g), Some(q)) = (self.last_gyro, self.last_quat) {
            let block = bus::pack_12(g, q);
            self.stale.observe(&block);
            self.last_imu = self.decoder.decode(&block);
        }

        Ok(self.last_imu)
    }

    fn ready(&self) -> bool {
        self.decoder.ready()
    }

    fn stale(&self) -> ImuStale {
        self.stale.get()
    }
}

/// WHO_AM_I of an LSM6DSV16X.
pub const LSM6DSV16X_ID: u8 = 0x70;

/// Addresses the chip straps (SA0 low / high).
pub const LSM6DSV16X_ADDRS: [u8; 2] = [0x6A, 0x6B];

/// A bus + 7-bit address that answered [`LSM6DSV16X_ID`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ImuLocation {
    pub bus: PathBuf,
    pub address: u8,
}

/// One WHO_AM_I read at a strapped LSM6 address, without resetting the chip.
///
/// Empty pads (NACK / ENXIO) are omitted from [`probe_who_am_i`]. `who` is `Ok(id)` when a
/// device ACKed — including chips that are not an LSM6DSV16X — and `Err` for unexpected I/O.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ImuWho {
    pub bus: PathBuf,
    pub address: u8,
    pub who: std::result::Result<u8, String>,
}

/// Probe 0x6A/0x6B on every `/dev/i2c-*`. This is the verbose scan `imu_probe --scan` prints.
///
/// [`scan_lsm6dsv16x`] keeps only [`LSM6DSV16X_ID`]. Call this when you also need to see the
/// other occupant at 0x6A (this board's i2c6 answers `0x4E` — do not [`ImuBus::open`] it).
pub fn probe_who_am_i() -> Vec<ImuWho> {
    #[cfg(not(target_os = "linux"))]
    {
        Vec::new()
    }
    #[cfg(target_os = "linux")]
    {
        probe_who_am_i_linux()
    }
}

/// Scan `/dev/i2c-*` for an LSM6DSV16X, the same search `imu_probe` uses when no bus is given.
///
/// Production wants the udev alias `/dev/i2c-imu` (i2c4). Some boards put the chip on another
/// controller — this one answers on `/dev/i2c-5` (`fe5e0000.i2c`) — so [`crate`]-level callers
/// fall back to this when the configured path cannot be opened or is the wrong chip.
pub fn scan_lsm6dsv16x() -> Vec<ImuLocation> {
    probe_who_am_i()
        .into_iter()
        .filter_map(|hit| match hit.who {
            Ok(id) if id == LSM6DSV16X_ID => Some(ImuLocation {
                bus: hit.bus,
                address: hit.address,
            }),
            _ => None,
        })
        .collect()
}

/// The unique chip on the machine, if there is exactly one. `None` if none or several.
pub fn pick_lsm6dsv16x() -> Option<ImuLocation> {
    let mut hits = scan_lsm6dsv16x();
    (hits.len() == 1).then(|| hits.remove(0))
}

#[cfg(target_os = "linux")]
fn probe_who_am_i_linux() -> Vec<ImuWho> {
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
        for address in LSM6DSV16X_ADDRS {
            match ImuBus::who_am_i(&bus, address) {
                Ok(id) => hits.push(ImuWho {
                    bus: bus.clone(),
                    address,
                    who: Ok(id),
                }),
                Err(e) if nobody_home(&e.to_string()) => {}
                Err(e) => hits.push(ImuWho {
                    bus: bus.clone(),
                    address,
                    who: Err(e.to_string()),
                }),
            }
        }
    }
    hits
}

#[cfg(target_os = "linux")]
fn bus_num(path: &Path) -> u32 {
    path.file_name()
        .and_then(|n| n.to_str())
        .and_then(|n| n.strip_prefix("i2c-"))
        .and_then(|n| n.parse().ok())
        .unwrap_or(u32::MAX)
}

#[cfg(target_os = "linux")]
fn nobody_home(msg: &str) -> bool {
    msg.contains("ENXIO")
        || msg.contains("No such device")
        || msg.contains("Remote I/O")
        || msg.contains("Input/output error")
}

/// The same type on a platform with no i2c-dev, so the crate still builds there.
///
/// This is not a fake IMU and must never become one — `FakeImu` already exists for that and
/// says what it is in its name. The type exists only so `robotd`'s platform-gated wiring can
/// name it on a laptop build; the only constructor always fails.
#[cfg(not(target_os = "linux"))]
pub struct ImuBus(std::convert::Infallible);

#[cfg(not(target_os = "linux"))]
impl ImuBus {
    pub fn open(bus_path: &Path, _address: u8) -> Result<Self> {
        Err(IoError::Bus(format!(
            "no i2c-dev on this platform, so {} cannot be opened — run with a fake IMU",
            bus_path.display()
        )))
    }

    pub fn who_am_i(bus_path: &Path, _address: u8) -> Result<u8> {
        Err(IoError::Bus(format!(
            "no i2c-dev on this platform, so {} cannot be probed",
            bus_path.display()
        )))
    }
}

#[cfg(not(target_os = "linux"))]
impl ImuIo for ImuBus {
    fn read(&mut self) -> Result<ImuData> {
        match self.0 {}
    }

    fn ready(&self) -> bool {
        match self.0 {}
    }

    fn stale(&self) -> ImuStale {
        match self.0 {}
    }
}

#[cfg(target_os = "linux")]
#[cfg(test)]
mod tests {
    use super::bus::{StaleImuTracker, assemble_block, pack_12};
    use super::*;
    use lsm6dsv16x_rs::blocking::prelude::Tag;

    const GYRO_RAD_PER_LSB: f64 = 0.0175 * std::f64::consts::PI / 180.0;

    /// The whole assembly path: synthetic FIFO tags → a 12-byte block → decoding. Pinned
    /// values: half 0x3C00 is 1.0, and a gyro count of 1000 must come back scaled by
    /// `GYRO_RAD_PER_LSB`.
    #[test]
    fn fifo_tags_pack_into_a_decodable_block() {
        let entries = [
            (Tag::GyNcTag, [0xE8, 0x03, 0x18, 0xFC, 0x00, 0x00]), // 1000, -1000, 0
            (
                Tag::SflpGameRotationVectorTag,
                [0x00, 0x3C, 0x00, 0x00, 0x00, 0x00], // x = 0x3C00 = 1.0 half
            ),
        ];
        let block = assemble_block(&entries).expect("a gyro+quat pair assembles");
        // 1000i16 LE, -1000i16 LE, 0i16 LE, then quat halves LE.
        assert_eq!(
            block,
            [
                0xE8, 0x03, 0x18, 0xFC, 0x00, 0x00, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00
            ]
        );

        let mut decoder = SflpDecoder::default();
        // Three identical blocks so the median in `decode` settles on the sample.
        let _ = decoder.decode(&block);
        let _ = decoder.decode(&block);
        let out = decoder.decode(&block);

        let gyro_mag: f64 = out.gyro.iter().map(|v| v.abs()).sum();
        let expected = (1000.0 + 1000.0) * GYRO_RAD_PER_LSB;
        assert!(
            (gyro_mag - expected).abs() < 1e-9,
            "gyro {gyro_mag} != {expected}"
        );

        let gm = (out.gravity[0] * out.gravity[0]
            + out.gravity[1] * out.gravity[1]
            + out.gravity[2] * out.gravity[2])
            .sqrt();
        assert!((gm - 1.0).abs() < 1e-9, "gravity not unit: {gm}");
        // A real half-quaternion is not the identity the decoder starts at.
        assert_ne!(out.quat, [1.0, 0.0, 0.0, 0.0]);
    }

    /// `pack_12` is the byte contract the decoder relies on; pin it directly.
    #[test]
    fn pack_12_is_little_endian_gyro_then_quaternion() {
        let block = pack_12([0x0102, -3, 0x0405], [0x0607, 0x0809, 0x0A0B]);
        assert_eq!(block[0..6], [0x02, 0x01, 0xFD, 0xFF, 0x05, 0x04]);
        assert_eq!(block[6..12], [0x07, 0x06, 0x09, 0x08, 0x0B, 0x0A]);
    }

    /// Repeated identical blocks are stale: the run grows unbounded and the total climbs; a
    /// fresh block resets the run but leaves the total where it is.
    #[test]
    fn stale_counts_repeated_blocks_and_resets_on_a_fresh_one() {
        let mut tracker = StaleImuTracker::default();
        let a = [7u8; IMU_BLOCK_LEN];
        let b = [8u8; IMU_BLOCK_LEN];

        tracker.observe(&a);
        tracker.observe(&a);
        tracker.observe(&a);
        let s = tracker.get();
        assert_eq!(s.total, 2);
        assert_eq!(s.run, 2);

        tracker.observe(&b);
        let s = tracker.get();
        assert_eq!(s.total, 2, "a fresh block keeps the cumulative total");
        assert_eq!(s.run, 0);

        tracker.observe(&b);
        let s = tracker.get();
        assert_eq!(s.total, 3);
        assert_eq!(s.run, 1);
    }

    /// `ready()` gates fall detection in slice 2. It must not be true from the first live
    /// block — the robot would otherwise be judged on a default orientation for the first
    /// quarter second.
    #[test]
    fn ready_gates_on_twenty_five_live_blocks() {
        let mut decoder = SflpDecoder::default();
        let mut block = [0u8; IMU_BLOCK_LEN];
        block[6..8].copy_from_slice(&0x3000u16.to_le_bytes());
        for _ in 0..24 {
            decoder.decode(&block);
        }
        assert!(!decoder.ready());
        decoder.decode(&block);
        assert!(decoder.ready());
    }

    /// No device here, so only the error path: opening a bus that cannot exist must fail.
    /// There is no ioctl test — running one needs an i2c-dev bus and a real IMU.
    #[test]
    fn open_fails_without_a_device() {
        let nowhere = Path::new("/dev/definitely-not-an-i2c-bus");
        assert!(ImuBus::open(nowhere, 0x6A).is_err());
        assert!(ImuBus::open(nowhere, 0x6B).is_err());
    }

    /// A bad address is rejected before any I/O, rather than silently picking one of the two.
    #[test]
    fn open_rejects_a_non_dsv16x_address() {
        // Use a path that would open fine if we got there; the address check happens first.
        assert!(ImuBus::open(Path::new("/dev/null"), 0x29).is_err());
    }

    #[test]
    fn nobody_home_matches_the_probe_skip_strings() {
        assert!(nobody_home("ENXIO"));
        assert!(nobody_home("Remote I/O error"));
        assert!(nobody_home("Input/output error"));
        assert!(!nobody_home("permission denied"));
    }

    #[test]
    fn scan_does_not_panic() {
        let _ = probe_who_am_i();
        let _ = scan_lsm6dsv16x();
        let _ = pick_lsm6dsv16x();
    }
}
