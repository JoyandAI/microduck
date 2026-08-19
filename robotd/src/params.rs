//! Startup parameters.
//!
//! A file rather than a wall of CLI flags — the prototype grew 142 of them and most were
//! variants, dead skills and dead sensors, all of which are gone. **Read once at startup,
//! not watched**; live reload is deferred (`docs/design/robotd-design.md` §7.2).
//!
//! It lives outside `releases/<ver>/` so it survives an update *and* a rollback: this is
//! per-robot configuration, not shipped defaults (`architecture.md` §3).

use std::path::{Path, PathBuf};

use serde::Deserialize;

/// Where a release is mounted. Policy paths default under here, so an ordinary update
/// carries the policy with the binaries that were trained against it.
pub const RELEASE_DIR: &str = "/opt/robot/daemon/current";

/// Where a provisioned robot keeps it, alongside the updater's own config.
pub const DEFAULT_PATH: &str = "/etc/robot/robotd.toml";

#[derive(Debug, Clone, Default, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct Params {
    pub bus: Bus,
    pub control: Control,
    pub update_gate: UpdateGate,
    pub policy: PolicyParams,
    pub safety: SafetyParams,
}

/// Which drive configuration this robot runs. One robot, two personalities: legs, or the
/// roller. They differ in policies *and* tuning, so the mode is one switch here rather than
/// six paths an operator has to keep consistent — the prototype's launcher kept two whole
/// command lines for the same reason. Switching is an edit plus `systemctl restart robotd`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Mode {
    #[default]
    Walk,
    Roller,
}

impl Mode {
    pub fn as_str(self) -> &'static str {
        match self {
            Mode::Walk => "walk",
            Mode::Roller => "roller",
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct PolicyParams {
    /// Whether to load a policy at all.
    ///
    /// False means slice 1's behaviour: run the loop, hold the pose, stay healthy. That is a
    /// legitimate configuration — it is the safest thing to be doing while hammering
    /// install/rollback cycles at a bench — and it is distinct from a policy that was wanted
    /// and could not be loaded, which is unhealthy.
    pub enabled: bool,
    /// `walk` (default) or `roller`. Changes which policies load *and* the tuning defaults
    /// below — every unset field resolves per mode, so a roller robot needs one line.
    pub mode: Mode,
    /// Policy paths. Absent means the mode's default inside the release directory, so a
    /// normal update ships them; point one elsewhere to try a build without cutting a
    /// release. The literal `"none"` disables a slot outright — the prototype's convention.
    pub walk: Option<PathBuf>,
    /// Standing policy. Without one the walking policy runs at every velocity.
    pub stand: Option<PathBuf>,
    /// Commanded sit↔stand (posture flag in the twist `vx` slot). Sit toggle, shutdown sit
    /// and the seated-boot rise all need it.
    pub sitstand: Option<PathBuf>,
    /// Phase-scripted ground pick. In roller mode this slot holds the crouch.
    pub ground_pick: Option<PathBuf>,
    pub kick_left: Option<PathBuf>,
    pub kick_right: Option<PathBuf>,
    /// Episodic forward roll. Ships by default in both modes, as the prototype now does.
    pub roulade: Option<PathBuf>,
    /// Scales raw policy output into a joint offset. Absent resolves per mode: 0.9 walking
    /// (the prototype's alpha default), 0.8 roller.
    pub action_scale: Option<f64>,
    pub standing_action_scale: f64,
    /// Standing runs softer, at this fraction of `gain`.
    pub standing_gain_ratio: f64,
    /// Position P gain while running.
    pub gain: u16,
    /// First-order low-pass on the head joint targets, `1.0` = pass-through. Default 0.5
    /// in both modes — the value the alpha policies are *trained* with, so it must match
    /// or transfer degrades. (The roller preset used to ship it off; the prototype rebased
    /// its roller line on the alpha defaults, and this follows.)
    pub head_lowpass: Option<f64>,
    /// Same, for the ten leg joints. Walking default 0.7.
    pub legs_lowpass: Option<f64>,
    /// One ground-pick cycle, seconds. The move ends at 70% of the cycle, as the prototype
    /// does. Absent resolves per mode: 4.0 walking, 3.0 roller (the crouch).
    pub ground_pick_period: Option<f64>,
    /// Action scale while the ground pick runs. Absent: 1.0 walking, 0.8 roller.
    pub ground_pick_action_scale: Option<f64>,
    /// Gain multiplier while the ground pick runs.
    pub ground_pick_gain_ratio: f64,
    /// How long a kick window stays on the kick network, seconds.
    pub kick_duration: f64,
    /// One roulade — one forward roll, seconds. Holding the button chains rolls; this is
    /// the length of each. The prototype's measured single-roll time.
    pub roulade_duration: f64,
    /// Action scale while a roulade runs.
    pub roulade_action_scale: f64,
    /// Gain multiplier while a roulade runs.
    pub roulade_gain_ratio: f64,
    /// Scale actions with battery voltage: effective scale × (nominal / measured). The
    /// servos' effective kP tracks their supply, so this holds the robot's response steady
    /// as the pack sags. Off by default, as in the prototype.
    pub voltage_adapt: bool,
    /// Reference voltage for `voltage_adapt` — the supply the gains were identified at.
    pub nominal_voltage: f64,
}

/// The literal that disables an optional policy slot, per the prototype's `--x-policy None`.
fn is_none_sentinel(path: &std::path::Path) -> bool {
    path.as_os_str().eq_ignore_ascii_case("none")
}

/// `[policy]` with every absent field resolved against the mode's defaults.
///
/// This is what the rest of `robotd` consumes — nothing downstream should ever have to ask
/// "walk or roller?" to know the action scale.
#[derive(Debug, Clone, PartialEq)]
pub struct ResolvedPolicy {
    pub enabled: bool,
    pub mode: Mode,
    pub walk: PathBuf,
    pub stand: Option<PathBuf>,
    pub sitstand: Option<PathBuf>,
    pub ground_pick: Option<PathBuf>,
    pub kick_left: Option<PathBuf>,
    pub kick_right: Option<PathBuf>,
    pub roulade: Option<PathBuf>,
    pub action_scale: f64,
    pub standing_action_scale: f64,
    pub standing_gain_ratio: f64,
    pub gain: u16,
    pub head_lowpass: Option<f64>,
    pub legs_lowpass: Option<f64>,
    pub ground_pick_period: f64,
    pub ground_pick_action_scale: f64,
    pub ground_pick_gain_ratio: f64,
    pub kick_duration: f64,
    pub roulade_duration: f64,
    pub roulade_action_scale: f64,
    pub roulade_gain_ratio: f64,
    pub voltage_adapt: bool,
    pub nominal_voltage: f64,
}

impl PolicyParams {
    pub fn resolved(&self) -> ResolvedPolicy {
        let release = |name: &str| PathBuf::from(RELEASE_DIR).join("policies").join(name);
        let path = |field: &Option<PathBuf>, default: Option<&str>| -> Option<PathBuf> {
            match field {
                Some(p) if is_none_sentinel(p) => None,
                Some(p) => Some(p.clone()),
                None => default.map(release),
            }
        };

        let (walk_default, stand, sitstand, ground_pick, kick) = match self.mode {
            Mode::Walk => (
                "alpha_walking.onnx",
                Some("alpha_stand.onnx"),
                Some("alpha_sitstand.onnx"),
                Some("alpha_ground_pick.onnx"),
                true,
            ),
            // The prototype's roller preset, since rebased on the alpha defaults: roller
            // policy, crouch on the ground-pick trigger, and everything else — sit/stand,
            // kicks, the trained low-pass — as the walking mode has it. `stand` stays
            // unloaded, deliberately: the prototype loads the standing network in roller
            // mode and then skips every standing transition while `roller_mode` is set, so
            // it never runs — not loading it is the same robot without the dead session.
            Mode::Roller => (
                "roller.onnx",
                None,
                Some("alpha_sitstand.onnx"),
                Some("roller_crouch.onnx"),
                true,
            ),
        };

        ResolvedPolicy {
            enabled: self.enabled,
            mode: self.mode,
            walk: path(&self.walk, Some(walk_default)).expect("walk always has a default"),
            stand: path(&self.stand, stand),
            sitstand: path(&self.sitstand, sitstand),
            ground_pick: path(&self.ground_pick, ground_pick),
            kick_left: path(&self.kick_left, kick.then_some("ball_kick_left.onnx")),
            kick_right: path(&self.kick_right, kick.then_some("ball_kick_right.onnx")),
            roulade: path(&self.roulade, Some("roulade.onnx")),
            action_scale: self.action_scale.unwrap_or(match self.mode {
                Mode::Walk => 0.9,
                Mode::Roller => 0.8,
            }),
            standing_action_scale: self.standing_action_scale,
            standing_gain_ratio: self.standing_gain_ratio,
            gain: self.gain,
            head_lowpass: Some(self.head_lowpass.unwrap_or(0.5)).filter(|a| *a < 1.0),
            legs_lowpass: Some(self.legs_lowpass.unwrap_or(0.7)).filter(|a| *a < 1.0),
            ground_pick_period: self.ground_pick_period.unwrap_or(match self.mode {
                Mode::Walk => 4.0,
                Mode::Roller => 3.0,
            }),
            ground_pick_action_scale: self.ground_pick_action_scale.unwrap_or(match self.mode {
                Mode::Walk => 1.0,
                Mode::Roller => 0.8,
            }),
            ground_pick_gain_ratio: self.ground_pick_gain_ratio,
            kick_duration: self.kick_duration,
            roulade_duration: self.roulade_duration,
            roulade_action_scale: self.roulade_action_scale,
            roulade_gain_ratio: self.roulade_gain_ratio,
            voltage_adapt: self.voltage_adapt,
            nominal_voltage: self.nominal_voltage,
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct SafetyParams {
    /// Projected-gravity z above which the robot counts as going down. Upright is about
    /// -1.0; on its side is near 0.
    pub fall_gravity_z: f64,
    /// How long that has to hold. Debounced so a firm footfall is not a fall.
    pub fall_debounce_ms: u64,
    /// Intent age past which the velocity is zeroed. Stop, not limp.
    pub deadman_ms: u64,
    /// Gain once fallen — low enough to yield rather than fight the floor.
    pub gain_limp: u16,
    /// Whether a detected fall preempts the policy: hold at `gain_limp`, refuse
    /// `robot.init`/`robot.enable`/skills until the robot is upright again. Off by
    /// default, as the prototype is — its `--fall-detect` ships off, so a fallen robot
    /// keeps being driven and the humans stay in charge. The fall verdict is reported in
    /// the state stream either way. `fall_recover = true` implies this: recovery starts
    /// with the limp settle.
    pub fall_limp: bool,
    /// Stand back up after a fall, on its own: limp 0.3 s, then the standing network drives
    /// until the robot has been solidly upright for a second. Reserves the standing network
    /// for recovery, so command magnitude stops selecting it. Off by default, as the
    /// prototype ships `--fall-detect`.
    pub fall_recover: bool,
    /// Sit down and power the machine off when the battery EMA reaches the empty floor
    /// (6.6 V — `duck_control::model::BATTERY_EMPTY_V`). The EMA moves over ~10 s, so a
    /// load sag cannot trip it.
    pub battery_empty_shutdown: bool,
}

impl Default for PolicyParams {
    fn default() -> Self {
        Self {
            enabled: true,
            mode: Mode::Walk,
            walk: None,
            stand: None,
            sitstand: None,
            ground_pick: None,
            kick_left: None,
            kick_right: None,
            roulade: None,
            action_scale: None,
            standing_action_scale: 1.0,
            // The prototype's `--standing-kp-ratio`.
            standing_gain_ratio: 0.8,
            gain: 200,
            head_lowpass: None,
            legs_lowpass: None,
            ground_pick_period: None,
            ground_pick_action_scale: None,
            ground_pick_gain_ratio: 1.0,
            kick_duration: 0.5,
            roulade_duration: 1.0,
            roulade_action_scale: 1.0,
            roulade_gain_ratio: 1.0,
            voltage_adapt: false,
            nominal_voltage: 7.4,
        }
    }
}

impl Default for SafetyParams {
    fn default() -> Self {
        Self {
            fall_gravity_z: -0.5,
            fall_debounce_ms: 200,
            deadman_ms: 500,
            gain_limp: 50,
            fall_limp: false,
            fall_recover: false,
            battery_empty_shutdown: true,
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct Bus {
    /// Serial port the servos and the IMU board share. The Radxa Zero 3W wires them to
    /// `/dev/ttyS2`.
    pub port: String,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct Control {
    /// Control loop rate. 50 Hz is inherited from the prototype, where it was chosen on a
    /// Pi Zero 2W — re-derive it on the Radxa rather than trusting it.
    pub hz: u32,
    /// Per-tick EMA on the velocity command: `cmd += α × (target − cmd)`. The prototype's
    /// `--cmd-alpha` — what turns a stick snap into a ramp the gait can follow. `1.0` is
    /// pass-through.
    pub cmd_alpha: f64,
    /// Same, for head targets and the body pose.
    pub head_alpha: f64,
}

/// Thresholds that decide `healthy` — and therefore whether an update is kept.
///
/// **Not** the thresholds for everything `robot.health` reports. That answer also describes the
/// battery, the motor temperatures and the loop counters, and none of those may reach a verdict
/// (`docs/design/robotd-design.md` §4.5) — so none of them has a setting here. Naming this section
/// `[health]` invited exactly that mistake: it reads like "how the robot is doing", when what it
/// configures is the one question auto-rollback turns on.
///
/// Everything here is a property of the *software*. A future `[thermal]` section for a motor
/// temperature that should throttle the robot would be a different thing, and belongs under a
/// different name.
///
/// The section was called `[health]`. Renamed outright rather than aliased: a board carrying
/// the old name gets a parse error naming the section, which is a better outcome than a robot
/// quietly running on default thresholds nobody chose.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct UpdateGate {
    /// Below this achieved rate the robot reports unhealthy, which is what makes the
    /// updater's auto-rollback mean something. A loop running at 60% of target is alive,
    /// answers every request, and is badly broken.
    pub min_achieved_hz: f64,
    /// How many periods may pass with no tick before the loop counts as **wedged**.
    ///
    /// This detects a dead loop, not a slow one — `min_achieved_hz` owns degradation. Keep
    /// the two apart: set this near the period and it fires on ordinary scheduler jitter,
    /// which on a loaded board would report a perfectly good release unhealthy and roll it
    /// back. A loop that has not ticked in half a second is genuinely gone; one that
    /// ticked 80 ms late is just late.
    pub stall_periods: u32,
    /// Consecutive bus read failures tolerated before reporting unhealthy. One dropped
    /// transaction is ordinary; a run of them means the bus is gone.
    pub max_consecutive_errors: u32,
}

impl Default for Bus {
    fn default() -> Self {
        Self {
            port: "/dev/ttyS2".into(),
        }
    }
}

impl Default for Control {
    fn default() -> Self {
        Self {
            hz: 50,
            cmd_alpha: 0.2,
            head_alpha: 0.2,
        }
    }
}

impl Default for UpdateGate {
    fn default() -> Self {
        Self {
            // 90% of the default rate. Generous enough not to trip on a slow tick, tight
            // enough that a loop losing every tenth cycle is not called healthy.
            min_achieved_hz: 45.0,
            // 500 ms at the default rate. Deliberately far from the period: three periods
            // is 60 ms, which ordinary scheduler jitter exceeds on a busy machine, and a
            // health check that trips on jitter rolls back good releases.
            stall_periods: 25,
            max_consecutive_errors: 10,
        }
    }
}

#[derive(Debug, thiserror::Error)]
pub enum ParamsError {
    #[error("reading {path}: {source}")]
    Read {
        path: String,
        #[source]
        source: std::io::Error,
    },
    #[error("parsing {path}: {source}")]
    Parse {
        path: String,
        #[source]
        source: toml::de::Error,
    },
    #[error("{path}: control.hz must be between 1 and 1000, got {got}")]
    Rate { path: String, got: u32 },
}

impl Params {
    /// Load from `path`. A missing file at the *default* location is not an error — an
    /// unprovisioned board should still come up on defaults rather than refuse to start,
    /// and a daemon that will not start is much harder to diagnose remotely than one
    /// running on known defaults. A file explicitly named on the command line must exist.
    pub fn load(path: &Path, explicit: bool) -> Result<Self, ParamsError> {
        let text = match std::fs::read_to_string(path) {
            Ok(text) => text,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound && !explicit => {
                tracing::warn!(path = %path.display(), "no params file; using defaults");
                return Ok(Self::default());
            }
            Err(source) => {
                return Err(ParamsError::Read {
                    path: path.display().to_string(),
                    source,
                });
            }
        };

        let params: Params = toml::from_str(&text).map_err(|source| ParamsError::Parse {
            path: path.display().to_string(),
            source,
        })?;
        params.validate(path)?;
        Ok(params)
    }

    /// Reject values that would produce a loop that cannot work, at startup rather than as
    /// a division by zero three seconds later.
    fn validate(&self, path: &Path) -> Result<(), ParamsError> {
        if self.control.hz == 0 || self.control.hz > 1000 {
            return Err(ParamsError::Rate {
                path: path.display().to_string(),
                got: self.control.hz,
            });
        }
        Ok(())
    }

    pub fn period(&self) -> std::time::Duration {
        std::time::Duration::from_secs_f64(1.0 / self.control.hz as f64)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn write(dir: &std::path::Path, body: &str) -> std::path::PathBuf {
        let path = dir.join("robotd.toml");
        std::fs::write(&path, body).unwrap();
        path
    }

    /// An unprovisioned board must still come up. A daemon that refuses to start because a
    /// config file is absent is far harder to diagnose on a robot than one running on
    /// documented defaults.
    #[test]
    fn a_missing_default_file_falls_back_to_defaults() {
        let dir = tempfile::tempdir().unwrap();
        let p = Params::load(&dir.path().join("absent.toml"), false).unwrap();
        assert_eq!(p.control.hz, 50);
    }

    /// But a file named explicitly on the command line must exist — silently ignoring
    /// `--params /path/typo.toml` would run the robot on settings nobody chose.
    #[test]
    fn an_explicitly_named_missing_file_is_an_error() {
        let dir = tempfile::tempdir().unwrap();
        assert!(Params::load(&dir.path().join("absent.toml"), true).is_err());
    }

    /// Partial files are the normal case — a board overrides the port and nothing else.
    #[test]
    fn absent_sections_take_their_defaults() {
        let dir = tempfile::tempdir().unwrap();
        let path = write(dir.path(), "[bus]\nport = \"/dev/ttyUSB0\"\n");
        let p = Params::load(&path, true).unwrap();
        assert_eq!(p.bus.port, "/dev/ttyUSB0");
        assert_eq!(p.control.hz, 50);
        assert_eq!(p.update_gate.stall_periods, 25);
    }

    /// The shipped example must agree with the built-in defaults, or the file documents a
    /// robot that does not exist — and an operator reading it would draw wrong conclusions
    /// about what their board is actually doing.
    #[test]
    fn the_shipped_example_matches_the_defaults() {
        let shipped = include_str!("../../deploy/robotd.toml");
        let from_file: Params = toml::from_str(shipped).expect("deploy/robotd.toml must parse");
        let built_in = Params::default();

        assert_eq!(from_file.bus.port, built_in.bus.port);
        assert_eq!(from_file.control.hz, built_in.control.hz);
        assert_eq!(from_file.control.cmd_alpha, built_in.control.cmd_alpha);
        assert_eq!(from_file.control.head_alpha, built_in.control.head_alpha);
        assert_eq!(from_file.policy.resolved(), built_in.policy.resolved());
        assert_eq!(from_file.safety.fall_recover, built_in.safety.fall_recover);
        assert_eq!(
            from_file.safety.battery_empty_shutdown,
            built_in.safety.battery_empty_shutdown
        );
        assert_eq!(
            from_file.update_gate.min_achieved_hz,
            built_in.update_gate.min_achieved_hz
        );
        assert_eq!(
            from_file.update_gate.stall_periods,
            built_in.update_gate.stall_periods
        );
        assert_eq!(
            from_file.update_gate.max_consecutive_errors,
            built_in.update_gate.max_consecutive_errors
        );
    }

    /// The resolved walk-mode defaults are the prototype's **current alpha configuration**
    /// — the values `microduck_runtime` ships as built-in defaults, which its installer
    /// deliberately passes no flags to override. Changing any of these silently changes how
    /// the robot moves relative to the thing this daemon replaces.
    #[test]
    fn walk_mode_resolves_to_the_prototype_alpha_config() {
        let p = Params::default().policy.resolved();
        assert_eq!(p.mode, Mode::Walk);
        assert_eq!(p.action_scale, 0.9);
        assert_eq!(p.standing_action_scale, 1.0);
        assert_eq!(p.standing_gain_ratio, 0.8, "--standing-kp-ratio");
        assert_eq!(p.gain, 200);
        assert_eq!(
            p.head_lowpass,
            Some(0.5),
            "trained with the filter ON at 0.5"
        );
        assert_eq!(
            p.legs_lowpass,
            Some(0.7),
            "trained with the filter ON at 0.7"
        );
        assert_eq!(p.ground_pick_period, 4.0);
        assert_eq!(p.ground_pick_action_scale, 1.0);
        assert_eq!(p.ground_pick_gain_ratio, 1.0);
        assert_eq!(p.kick_duration, 0.5);
        assert_eq!(p.roulade_duration, 1.0, "one roll, the measured time");
        assert_eq!(p.roulade_action_scale, 1.0);
        assert_eq!(p.roulade_gain_ratio, 1.0);
        assert!(!p.voltage_adapt, "off by default in the prototype");
        assert_eq!(p.nominal_voltage, 7.4);

        let name = |p: &Option<std::path::PathBuf>| {
            p.as_ref()
                .and_then(|p| p.file_name())
                .map(|n| n.to_string_lossy().into_owned())
        };
        assert!(p.walk.ends_with("policies/alpha_walking.onnx"));
        assert_eq!(name(&p.stand).as_deref(), Some("alpha_stand.onnx"));
        assert_eq!(name(&p.sitstand).as_deref(), Some("alpha_sitstand.onnx"));
        assert_eq!(
            name(&p.ground_pick).as_deref(),
            Some("alpha_ground_pick.onnx")
        );
        assert_eq!(name(&p.kick_left).as_deref(), Some("ball_kick_left.onnx"));
        assert_eq!(name(&p.kick_right).as_deref(), Some("ball_kick_right.onnx"));
        assert_eq!(name(&p.roulade).as_deref(), Some("roulade.onnx"));
    }

    /// Command smoothing matches the prototype's `--cmd-alpha` / `--head-alpha`.
    #[test]
    fn command_smoothing_defaults_match_the_prototype() {
        let c = Control::default();
        assert_eq!(c.cmd_alpha, 0.2);
        assert_eq!(c.head_alpha, 0.2);
    }

    /// One line — `mode = "roller"` — must reproduce the prototype's whole roller preset,
    /// which its installer rebased on the alpha defaults: the roller policy and its tuning
    /// (kp 200, scale 0.8, the crouch on the ground-pick trigger at 3 s / 0.8), and
    /// everything else exactly as walking mode has it — sit/stand, kicks, roulade, the
    /// trained low-pass. Only the standing network stays out (the prototype loads it and
    /// then skips every standing transition in roller mode, so it never runs).
    #[test]
    fn roller_mode_resolves_to_the_prototype_roller_preset() {
        let dir = tempfile::tempdir().unwrap();
        let path = write(dir.path(), "[policy]\nmode = \"roller\"\n");
        let p = Params::load(&path, true).unwrap().policy.resolved();

        assert_eq!(p.mode, Mode::Roller);
        assert!(p.walk.ends_with("policies/roller.onnx"));
        assert_eq!(
            p.stand, None,
            "the prototype never runs standing in roller mode"
        );
        assert!(
            p.sitstand
                .as_ref()
                .unwrap()
                .ends_with("alpha_sitstand.onnx"),
            "the rebased roller line keeps the sit"
        );
        assert!(p.kick_left.as_ref().unwrap().ends_with("ball_kick_left.onnx"));
        assert!(
            p.kick_right
                .as_ref()
                .unwrap()
                .ends_with("ball_kick_right.onnx")
        );
        assert!(p.roulade.as_ref().unwrap().ends_with("roulade.onnx"));
        assert!(
            p.ground_pick
                .as_ref()
                .unwrap()
                .ends_with("roller_crouch.onnx")
        );
        assert_eq!(p.action_scale, 0.8);
        assert_eq!(p.ground_pick_period, 3.0);
        assert_eq!(p.ground_pick_action_scale, 0.8);
        assert_eq!(
            p.head_lowpass,
            Some(0.5),
            "the rebased roller line keeps the trained filters"
        );
        assert_eq!(p.legs_lowpass, Some(0.7));
        assert_eq!(p.gain, 200);
    }

    /// `"none"` disables an optional slot outright — the prototype's `--sitstand-policy None`
    /// convention — and `1.0` turns a low-pass into a pass-through, which is how its preset
    /// spells "off".
    #[test]
    fn none_and_unity_are_the_off_switches() {
        let dir = tempfile::tempdir().unwrap();
        let path = write(
            dir.path(),
            "[policy]\nsitstand = \"None\"\nhead_lowpass = 1.0\n",
        );
        let p = Params::load(&path, true).unwrap().policy.resolved();
        assert_eq!(p.sitstand, None);
        assert_eq!(
            p.head_lowpass, None,
            "alpha 1.0 is a pass-through, so store it as off"
        );
        assert_eq!(
            p.legs_lowpass,
            Some(0.7),
            "the other filter keeps its default"
        );
    }

    /// A typo in a key must fail loudly. Silently ignoring `min_acheived_hz` would leave
    /// the update gate at a threshold the operator believes they changed.
    #[test]
    fn an_unknown_key_is_rejected() {
        let dir = tempfile::tempdir().unwrap();
        let path = write(dir.path(), "[update_gate]\nmin_acheived_hz = 10.0\n");
        assert!(Params::load(&path, true).is_err());
    }

    /// The old section name must be *rejected*, not silently ignored.
    ///
    /// A board still carrying `[health]` gets a `robotd` that refuses to start and says why,
    /// which is the honest outcome: `deny_unknown_fields` means the operator hears about the
    /// file rather than running on defaults they did not choose while believing otherwise.
    /// `install.sh` never overwrites `robotd.toml`, so the fix is to edit the section name —
    /// and the parse error names it.
    #[test]
    fn the_old_health_section_name_is_rejected() {
        let dir = tempfile::tempdir().unwrap();
        let path = write(dir.path(), "[health]\nmin_achieved_hz = 40.0\n");
        assert!(Params::load(&path, true).is_err());
    }

    /// Zero would divide by zero when computing the period; absurdly high would spin.
    #[test]
    fn an_impossible_rate_is_rejected_at_startup() {
        let dir = tempfile::tempdir().unwrap();
        for hz in ["0", "5000"] {
            let path = write(dir.path(), &format!("[control]\nhz = {hz}\n"));
            assert!(Params::load(&path, true).is_err(), "hz = {hz} was accepted");
        }
    }
}
