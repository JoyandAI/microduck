//! Per-variant hardware numbers: the original MicroDuck (XL330 + bus IMU) and OpenMicroDuck
//! (HD1910 + I²C IMU).
//!
//! The BAM *model* (inertia, friction, back-EMF) lives in the identification repo and is
//! used at training time. This file keeps the snapshot the runtime must not drift from:
//! the six M1 identification parameters copied from `bam/params/hd1910/m1.json`, and the
//! firmware register values that model was identified against. `ftbus` writes the firmware
//! half; the six identification numbers are not sent to the servo.

/// The six M1 (Coulomb) identification parameters for the HD-1910-C001.
///
/// Copied from `E:\dev\bam\bam\params\hd1910\m1.json` (commit `b8b7721`, 2026-09-09).
/// Units: `kt` N·m/A, `R` Ω, `armature` kg·m², `q_offset` rad, `friction_base` N·m,
/// `friction_viscous` N·m/(rad/s).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Hd1910BamM1 {
    pub kt: f64,
    pub r: f64,
    pub armature: f64,
    pub q_offset: f64,
    pub friction_base: f64,
    pub friction_viscous: f64,
}

/// HD-1910-C001 M1 snapshot. These six are the identification set; firmware registers
/// that the same JSON also carries (`kd`, `error_gain`, `max_current`, …) live in
/// [`HD1910_FIRMWARE`].
pub const HD1910_BAM_M1: Hd1910BamM1 = Hd1910BamM1 {
    kt: 0.736,
    r: 3.75,
    armature: 0.0008,
    q_offset: 0.0,
    friction_base: 0.05,
    friction_viscous: 0.008,
};

/// Firmware values the HD1910 BAM model was identified against, and what `FeetechIo` writes.
///
/// True-hardware readback (FT-SCS, firmware 3.46): mode 4, Kp/Kd/Ki = 32/40/0, protection
/// current 500 LSB = 3.25 A, voltage window 4.0–8.4 V. The robot's pack is 2S **7.4 V**.
/// Mode 4 is pure position PD and does not rate-limit the target
/// (`use_rate_limiting = false` in the BAM actuator).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Hd1910Firmware {
    /// Register 33: 4 = pure position PD (factory / Sim2Real).
    pub mode: u8,
    /// Register 50 (and EEPROM 21).
    pub kp: u8,
    /// Register 51 (and EEPROM 22). Not 32 — the HD1910 factory D is 40.
    pub kd: u8,
    /// Register 52 (and EEPROM 23). Inactive in position mode.
    pub ki: u8,
    /// Register 28/44, 6.5 mA/LSB. 500 → 3.25 A.
    pub protection_current_lsb: u16,
    /// Register 14, 0.1 V/LSB. Spec max 8.4 V.
    pub max_voltage_lsb: u8,
    /// Register 15, 0.1 V/LSB. Spec min 4.0 V.
    pub min_voltage_lsb: u8,
}

pub const HD1910_FIRMWARE: Hd1910Firmware = Hd1910Firmware {
    mode: 4,
    kp: 32,
    kd: 40,
    ki: 0,
    protection_current_lsb: 500,
    max_voltage_lsb: 84,
    min_voltage_lsb: 40,
};

/// Pack the HD1910 runs from on the robot: 2S, 7.4 V. This is the unset default of
/// `policy.nominal_voltage`.
///
/// BAM identification used `vin=6.0` in `hd1910/m1.json`; that is a model condition,
/// not the runtime pack.
pub const HD1910_VIN: f64 = 7.4;

/// Milliamps per current LSB, from the Feetech memory table.
pub const HD1910_MA_PER_CURRENT_LSB: f64 = 6.5;

impl Hd1910Firmware {
    pub fn protection_current_a(self) -> f64 {
        self.protection_current_lsb as f64 * HD1910_MA_PER_CURRENT_LSB / 1000.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Pin the six M1 numbers to the BAM JSON. A silent drift here is a sim2real split.
    #[test]
    fn hd1910_m1_matches_the_bam_json() {
        assert_eq!(HD1910_BAM_M1.kt, 0.736);
        assert_eq!(HD1910_BAM_M1.r, 3.75);
        assert_eq!(HD1910_BAM_M1.armature, 0.0008);
        assert_eq!(HD1910_BAM_M1.q_offset, 0.0);
        assert_eq!(HD1910_BAM_M1.friction_base, 0.05);
        assert_eq!(HD1910_BAM_M1.friction_viscous, 0.008);
    }

    #[test]
    fn hd1910_firmware_matches_the_bench_readback() {
        assert_eq!(HD1910_FIRMWARE.mode, 4);
        assert_eq!(HD1910_FIRMWARE.kp, 32);
        assert_eq!(HD1910_FIRMWARE.kd, 40);
        assert_eq!(HD1910_FIRMWARE.ki, 0);
        assert_eq!(HD1910_FIRMWARE.protection_current_lsb, 500);
        assert!((HD1910_FIRMWARE.protection_current_a() - 3.25).abs() < 1e-12);
        assert_eq!(HD1910_FIRMWARE.max_voltage_lsb, 84);
        assert_eq!(HD1910_FIRMWARE.min_voltage_lsb, 40);
        assert_eq!(HD1910_VIN, 7.4);
    }
}
