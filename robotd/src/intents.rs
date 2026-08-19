//! What clients are asking the robot to do.
//!
//! Written by IPC tasks, read once per tick by the control loop. The loop must never wait
//! on a client, so each slot is an [`ArcSwap`]: the reader does one atomic load and the
//! writer one atomic store, and neither can hold up the other.
//!
//! **Twist and head are separate slots on purpose.** A single combined slot would need
//! read-modify-write to update one field, and two clients — a gamepad driving the body and
//! something else driving the head — would silently lose each other's updates. Separate
//! slots make each one single-writer in practice, so last-writer-wins means what it says.
//!
//! Every slot is stamped, because the loop's real question is never "what is the value" but
//! "how old is it". That is what the deadman reads.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};

/// Encodings for [`Intents::power`]. An `AtomicU8` rather than two bools, so "init" and "relax"
/// cannot both be pending — they are alternatives, and the last one asked for wins.
const POWER_NONE: u8 = 0;
const POWER_INIT: u8 = 1;
const POWER_RELAX: u8 = 2;
use std::time::{Duration, Instant};

use arc_swap::ArcSwap;
use duck_control::obs::{BodyPose, Command};

/// A value and when it arrived.
#[derive(Debug, Clone, Copy)]
struct Stamped<T> {
    value: T,
    /// Microseconds since the [`Intents`] epoch.
    at_us: u64,
}

/// The body-pose intent, as the loop consumes it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PoseIntent {
    /// z, roll, pitch — the order the observation's body block uses.
    pub body: [f64; 3],
    /// While true the loop glides toward `body`; false snaps it back to nominal at once,
    /// which is the prototype's B-button exit.
    pub active: bool,
}

impl Default for PoseIntent {
    fn default() -> Self {
        Self {
            body: [0.0; 3],
            active: false,
        }
    }
}

/// Pending one-shot skill requests, taken once per tick.
///
/// Booleans rather than a queue: within one 20 ms tick a second press of the same button
/// means nothing extra, and two *different* requests both deserve to be seen — which a
/// single last-writer slot would lose.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct SkillRequests {
    pub ground_pick: bool,
    pub kick_left: bool,
    pub kick_right: bool,
    pub sit_toggle: bool,
    /// Start a roll — or, arriving while one runs, chain another. Clients hold a button
    /// down by sending this every tick, so unlike the others it is a *level* in practice.
    pub roulade: bool,
}

impl SkillRequests {
    pub fn any(&self) -> bool {
        self.ground_pick
            || self.kick_left
            || self.kick_right
            || self.sit_toggle
            || self.roulade
    }
}

// Bit positions for the skill-request mask.
const SKILL_GROUND_PICK: u32 = 1 << 0;
const SKILL_KICK_LEFT: u32 = 1 << 1;
const SKILL_KICK_RIGHT: u32 = 1 << 2;
const SKILL_SIT_TOGGLE: u32 = 1 << 3;
const SKILL_ROULADE: u32 = 1 << 4;

pub struct Intents {
    /// Epoch for every stamp. `Instant` so the clock cannot run backwards under us.
    epoch: Instant,
    twist: ArcSwap<Stamped<[f64; 3]>>,
    head: ArcSwap<Stamped<[f64; 4]>>,
    /// Standing body pose. Unstamped: `active: false` is its own "nobody is posing".
    pose: ArcSwap<PoseIntent>,
    /// Mouth opening, 0..1, as `f64::to_bits`. Continuous like the twist, unstamped like
    /// the pose — a mouth left open by a dead client is exactly what the prototype does.
    mouth: std::sync::atomic::AtomicU64,
    /// Whether the policy should drive. Discrete, so a plain flag rather than a slot.
    enabled: AtomicBool,
    /// A pending `robot.init` / `robot.relax`, as [`PowerRequest`].
    ///
    /// A request rather than a flag, and taken rather than read: powering the joints is an *edge*,
    /// not a state the loop should keep re-applying. One `set_torque` is a bus transaction per
    /// joint, so a level here would put sixteen writes into every tick for as long as it stayed set.
    ///
    /// It lives with the intents because this is where the loop reads what clients asked for, and
    /// because the loop is the only thing that may touch the bus — the IPC task cannot do it itself.
    power: AtomicU8,
    /// Pending skill requests, a bitmask taken (swapped to zero) once per tick. A mask
    /// rather than one slot so two different buttons in the same tick both arrive.
    skills: std::sync::atomic::AtomicU32,
    /// A shutdown was requested. A level, not an edge: once asked, the sequence runs.
    shutdown: AtomicBool,
}

/// What a client asked for, once.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PowerRequest {
    /// Torque on, ramp to the home pose.
    Init,
    /// Torque off. The robot collapses if nothing holds it.
    Relax,
}

/// What the loop reads at the top of a tick.
#[derive(Debug, Clone, Copy)]
pub struct Snapshot {
    pub command: Command,
    /// Age of the most recent *twist*, which is what the deadman guards. A stale head pose
    /// is harmless; a stale velocity walks the robot into a wall.
    pub twist_age: Duration,
    pub enabled: bool,
    /// The body-pose intent. The loop smooths `body` into `command.body` itself, because
    /// smoothing is per-tick state the intent slots must not own.
    pub pose: PoseIntent,
    /// Mouth opening, 0..1.
    pub mouth: f64,
}

impl Default for Intents {
    fn default() -> Self {
        Self::new()
    }
}

impl Intents {
    pub fn new() -> Self {
        let epoch = Instant::now();
        Self {
            epoch,
            // Stamped at zero, so before any client connects the twist already reads as
            // maximally stale and the deadman holds the robot still. Starting "fresh" would
            // mean a robot that briefly believes it has a live driver.
            twist: ArcSwap::from_pointee(Stamped {
                value: [0.0; 3],
                at_us: 0,
            }),
            head: ArcSwap::from_pointee(Stamped {
                value: [0.0; 4],
                at_us: 0,
            }),
            pose: ArcSwap::from_pointee(PoseIntent::default()),
            mouth: std::sync::atomic::AtomicU64::new(0.0f64.to_bits()),
            enabled: AtomicBool::new(false),
            power: AtomicU8::new(POWER_NONE),
            skills: std::sync::atomic::AtomicU32::new(0),
            shutdown: AtomicBool::new(false),
        }
    }

    fn now_us(&self) -> u64 {
        self.epoch.elapsed().as_micros() as u64
    }

    pub fn set_twist(&self, twist: [f64; 3]) {
        self.twist.store(Arc::new(Stamped {
            value: twist,
            at_us: self.now_us(),
        }));
    }

    pub fn set_head(&self, head: [f64; 4]) {
        self.head.store(Arc::new(Stamped {
            value: head,
            at_us: self.now_us(),
        }));
    }

    /// Zero the velocity now. Distinct from the deadman only in that it is deliberate.
    pub fn stop(&self) {
        self.set_twist([0.0; 3]);
    }

    pub fn set_pose(&self, pose: PoseIntent) {
        self.pose.store(Arc::new(pose));
    }

    pub fn set_mouth(&self, open: f64) {
        self.mouth
            .store(open.to_bits(), std::sync::atomic::Ordering::Relaxed);
    }

    /// Queue a one-shot skill for the loop's next tick.
    pub fn request_skill(&self, skill: duck_ipc_proto::Skill) {
        let bit = match skill {
            duck_ipc_proto::Skill::GroundPick => SKILL_GROUND_PICK,
            duck_ipc_proto::Skill::KickLeft => SKILL_KICK_LEFT,
            duck_ipc_proto::Skill::KickRight => SKILL_KICK_RIGHT,
            duck_ipc_proto::Skill::SitToggle => SKILL_SIT_TOGGLE,
            duck_ipc_proto::Skill::Roulade => SKILL_ROULADE,
        };
        self.skills
            .fetch_or(bit, std::sync::atomic::Ordering::Relaxed);
    }

    /// Take the pending skill requests, leaving none. Once per tick, like the power request.
    pub fn take_skills(&self) -> SkillRequests {
        let bits = self.skills.swap(0, std::sync::atomic::Ordering::Relaxed);
        SkillRequests {
            ground_pick: bits & SKILL_GROUND_PICK != 0,
            kick_left: bits & SKILL_KICK_LEFT != 0,
            kick_right: bits & SKILL_KICK_RIGHT != 0,
            sit_toggle: bits & SKILL_SIT_TOGGLE != 0,
            roulade: bits & SKILL_ROULADE != 0,
        }
    }

    /// Ask for the sit-then-power-off sequence.
    pub fn request_shutdown(&self) {
        self.shutdown.store(true, Ordering::Relaxed);
    }

    /// Take a pending shutdown request. Taken rather than read so the sequence starts once.
    pub fn take_shutdown(&self) -> bool {
        self.shutdown.swap(false, Ordering::Relaxed)
    }

    pub fn set_enabled(&self, on: bool) {
        self.enabled.store(on, Ordering::Relaxed);
    }

    /// The current enable state — what `robot.enable`'s `toggle` flips. The loop reads its
    /// copy through [`Self::snapshot`]; this is for the IPC side, which owns the toggle.
    pub fn enabled(&self) -> bool {
        self.enabled.load(Ordering::Relaxed)
    }

    /// Ask the loop to power the joints and stand up.
    pub fn request_init(&self) {
        self.power.store(POWER_INIT, Ordering::Relaxed);
    }

    /// Ask the loop to cut power to the joints.
    ///
    /// Also clears `enabled`: a robot that has been asked to go limp is not one the policy should
    /// keep driving, and leaving that flag set would have the next tick bring it straight back up.
    pub fn request_relax(&self) {
        self.enabled.store(false, Ordering::Relaxed);
        self.power.store(POWER_RELAX, Ordering::Relaxed);
    }

    /// Take the pending request, leaving none.
    ///
    /// Called once per tick by the loop. A later request replaces an unread earlier one, which is
    /// the right resolution: if someone asked to stand up and then to relax within 20 ms, the
    /// second is what they meant.
    pub fn take_power_request(&self) -> Option<PowerRequest> {
        match self.power.swap(POWER_NONE, Ordering::Relaxed) {
            POWER_INIT => Some(PowerRequest::Init),
            POWER_RELAX => Some(PowerRequest::Relax),
            _ => None,
        }
    }

    pub fn snapshot(&self) -> Snapshot {
        let now = self.now_us();
        let twist = self.twist.load();
        let head = self.head.load();
        let pose = **self.pose.load();
        Snapshot {
            command: Command {
                twist: twist.value,
                head: head.value,
                // The loop owns the smoothing that turns the pose intent into this block;
                // the raw target travels in `pose` below. Nominal zero is the trained
                // encoding, not a placeholder.
                body: BodyPose::default(),
            },
            twist_age: Duration::from_micros(now.saturating_sub(twist.at_us)),
            enabled: self.enabled.load(Ordering::Relaxed),
            pose,
            mouth: f64::from_bits(self.mouth.load(std::sync::atomic::Ordering::Relaxed)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Before any client has spoken, the twist must already look stale. A robot that comes
    /// up believing it has a live driver would run its deadman timer down from `now`,
    /// giving a window where nothing is commanding it and nothing knows.
    #[test]
    fn the_twist_starts_stale() {
        let intents = Intents::new();
        let snap = intents.snapshot();
        assert_eq!(snap.command.twist, [0.0; 3]);
        assert!(
            snap.twist_age >= Duration::ZERO,
            "age must be measured from the epoch, not from first use"
        );
        assert!(!snap.enabled, "nothing drives until something asks");
    }

    /// Setting the head must not disturb the twist or its age, and vice versa. This is the
    /// whole reason they are separate slots: a combined one would need read-modify-write
    /// and two clients would clobber each other.
    #[test]
    fn the_slots_are_independent() {
        let intents = Intents::new();
        intents.set_twist([0.5, 0.0, 0.2]);
        std::thread::sleep(Duration::from_millis(5));
        intents.set_head([0.1, 0.2, 0.3, 0.4]);

        let snap = intents.snapshot();
        assert_eq!(
            snap.command.twist,
            [0.5, 0.0, 0.2],
            "head write clobbered twist"
        );
        assert_eq!(snap.command.head, [0.1, 0.2, 0.3, 0.4]);
        assert!(
            snap.twist_age >= Duration::from_millis(5),
            "a head write must not refresh the twist's deadman clock"
        );
    }

    /// The age is what the deadman reads, so a fresh write has to visibly reset it.
    #[test]
    fn writing_the_twist_refreshes_its_age() {
        let intents = Intents::new();
        std::thread::sleep(Duration::from_millis(10));
        let stale = intents.snapshot().twist_age;

        intents.set_twist([0.1, 0.0, 0.0]);
        let fresh = intents.snapshot().twist_age;

        assert!(
            fresh < stale,
            "expected {fresh:?} to be younger than {stale:?}"
        );
    }

    /// `stop` zeroes velocity without disabling the policy — the robot should stand, not
    /// go limp or stop being driven.
    #[test]
    fn stop_zeroes_the_twist_and_leaves_the_policy_enabled() {
        let intents = Intents::new();
        intents.set_enabled(true);
        intents.set_twist([1.0, 1.0, 1.0]);

        intents.stop();
        let snap = intents.snapshot();
        assert_eq!(snap.command.twist, [0.0; 3]);
        assert!(snap.enabled, "stop is not disable");
    }

    /// The body block has no intent behind it yet and must stay at the trained nominal.
    #[test]
    fn the_body_command_stays_nominal() {
        let intents = Intents::new();
        intents.set_twist([1.0, 0.0, 0.0]);
        assert_eq!(intents.snapshot().command.body, BodyPose::default());
    }
}
