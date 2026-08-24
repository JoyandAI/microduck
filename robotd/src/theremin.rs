//! The ToF theremin: a hand's distance in front of the beak becomes a note.
//!
//! Three things have to meet for that, and they run at three different rates. The depth
//! frames arrive from `tofd` at 15 Hz over a socket this daemon does not own. The control
//! loop runs at 50 Hz and is the only thing allowed to touch the mouth. The audio is
//! rendered at 48 kHz in a writer thread. This module is where the first two meet: a reader
//! thread parks on the depth socket and leaves the newest frame in a slot, and
//! [`Theremin::tick`] — called from the control loop, never blocking — turns whatever is in
//! that slot into a note, a mouth opening, and a line of state for clients to watch.
//!
//! **One gesture, three outputs.** Closeness (0 at the far end of the playable band, 1 at
//! the near end, from [`kinematics::hand`]) drives the pitch, the level, *and* how far the
//! mouth opens. Not three tunings of the same thing but literally one number, because a
//! duck whose mouth opens on a different curve from its pitch reads as a mouth animation
//! playing over a sound rather than as an animal making one.
//!
//! **Where the hand-versus-wall problem is solved: not here.** [`kinematics::hand`] owns
//! that, and its answer is a background captured when the theremin arms — see its module
//! docs. What this module owns is the *sequencing* that answer needs: an arming window that
//! spans several frames, a refusal that has to survive until a client reads it, and the fact
//! that a walking duck invalidates its own background and so is not allowed to hold the
//! instrument.
//!
//! **Rate mismatch is a fade, not a gate.** A depth frame that stops arriving — `tofd`
//! restarted, the sensor dropped off the bus — must not leave a note sounding forever, and
//! must not chop one off either. A frame older than [`FRAME_STALE`] takes the level to zero
//! and leaves everything else alone, so the instrument goes quiet and comes back when the
//! frames do.

use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant};

use arc_swap::ArcSwapOption;
use duck_ipc_proto as proto;
use kinematics::hand::{self, Arming, Background, Hand, Refusal};
use kinematics::tof::{COLS, Posture, ROWS, Reprojector};

const N_ZONES: usize = ROWS * COLS;

/// How long a depth frame stays playable. Three frames at 15 Hz: long enough to ride out a
/// dropped one, short enough that a dead sensor falls silent rather than holding a note.
const FRAME_STALE: Duration = Duration::from_millis(200);

/// How fast the reader thread retries a depth socket that is not there. `tofd` may be
/// restarting, or may not be running at all on a duck without the sensor — either way this
/// is a background thread and its failure is a log line, not an error anybody waits on.
const RECONNECT: Duration = Duration::from_secs(2);

/// Request id for the subscription. Any number; `tofd` echoes it.
const SUBSCRIBE_ID: u64 = 1;

/// One depth frame as the loop consumes it: slant ranges in metres, `None` where the sensor
/// had nothing usable to say.
///
/// Interpreted in the reader thread rather than the loop — the status-byte rules are the
/// `tof` crate's (`Frame::zone`), transcribed here because `robotd` must not link that crate
/// for its vendored C driver, and doing it once per frame off the loop is free.
struct Frame {
    ranges_m: [Option<f64>; N_ZONES],
    at: Instant,
}

/// What the instrument is doing.
enum State {
    /// Put down. Nothing is read, nothing sounds.
    Down,
    /// Collecting frames for a background. Half a second, at 15 Hz.
    Arming(Arming),
    /// Playable. The background is the zero every frame is measured against. Boxed because
    /// it is by far the largest thing a `State` can be — 64 ranges and a shape — and an
    /// instrument that is down should not carry the size of one that is up.
    Armed(Box<Background>),
    /// Arming came back with a reason, which is kept until the instrument is picked up
    /// again — a refusal that vanished before the client's next state frame would present
    /// as a theremin that silently does nothing.
    Refused(Refusal),
}

/// What one tick of the theremin produced.
///
/// Deliberately *not* a frequency: mapping closeness to a note needs this duck's register,
/// which lives with its voice (`crate::sound::Sound::theremin_hz_at`) and not with its
/// depth sensor. This module says how close the hand is; the voice says what that sounds
/// like.
#[derive(Debug, Clone, PartialEq)]
pub struct Note {
    /// Mouth opening to drive, 0..1. Always present while the instrument is up, because a
    /// silent theremin is a *closed* mouth and that has to be commanded too.
    pub mouth: f64,
    /// The state block for `robot.state`. `note_hz` is left for the caller to fill, for the
    /// reason above.
    pub state: proto::ThereminState,
    /// Where the hand is in the playable band, 0 far to 1 near. `None` is silence — no hand
    /// in the band, still arming, refused, or a sensor that stopped delivering.
    pub closeness: Option<f64>,
}

pub struct Theremin {
    /// Newest frame from the reader thread. `ArcSwapOption` for the reason every other
    /// intent slot is one: the loop does an atomic load and can never be held up by the
    /// thread writing the other side.
    latest: Arc<ArcSwapOption<Frame>>,
    reprojector: Reprojector,
    config: hand::Config,
    state: State,
    /// The frame the arming window last took, so a stalled sensor cannot fill a window with
    /// sixty copies of one frame and call it a background.
    last_armed_at: Option<Instant>,
}

impl Theremin {
    /// Start the depth reader and hold an instrument that is down.
    ///
    /// The reader runs whether or not a theremin is ever asked for: it is one parked read on
    /// a socket, and starting it lazily would mean the first arming window waited for a
    /// connection as well as for frames — a second of nothing, which reads as a broken
    /// feature.
    pub fn spawn(socket: PathBuf, config: hand::Config, reprojector: Reprojector) -> Self {
        let latest = Arc::new(ArcSwapOption::empty());
        let slot = latest.clone();
        let spawned = std::thread::Builder::new()
            .name("tof-reader".into())
            .spawn(move || read_frames(&socket, &slot));
        if spawned.is_err() {
            tracing::warn!("cannot spawn the depth reader; no theremin");
        }
        Self {
            latest,
            reprojector,
            config,
            state: State::Down,
            last_armed_at: None,
        }
    }

    /// True while the instrument is up — arming, armed, or refused.
    pub fn active(&self) -> bool {
        !matches!(self.state, State::Down)
    }

    /// Whether depth frames are arriving at all. What a refusal at the door is made of:
    /// accepting a theremin on a duck with no sensor would be a feature that silently does
    /// nothing.
    pub fn has_frames(&self) -> bool {
        self.latest
            .load()
            .as_ref()
            .is_some_and(|frame| frame.at.elapsed() < FRAME_STALE)
    }

    /// Pick the instrument up, or put it down. Idempotent: picking up an instrument already
    /// in hand does *not* restart the arming window, because that would drop the background
    /// a player is in the middle of using.
    pub fn set_active(&mut self, active: bool) {
        match (active, &self.state) {
            (true, State::Down | State::Refused(_)) => {
                self.state = State::Arming(Arming::new());
                self.last_armed_at = None;
            }
            (true, _) => {}
            (false, _) => self.state = State::Down,
        }
    }

    /// Put the instrument down because the robot is doing something incompatible with
    /// holding it — walking, or on its side.
    ///
    /// A background is a picture of what is in front of the duck, so a duck that moves has a
    /// background of somewhere it no longer is. Re-arming automatically when it stops would
    /// be worse than dropping it: the player would get an instrument whose zero silently
    /// changed under them.
    pub fn invalidate(&mut self, why: &str) {
        if self.active() {
            tracing::warn!(why, "theremin: put down");
            self.state = State::Down;
        }
    }

    /// One tick. Never blocks, never allocates a frame, and returns `None` when there is
    /// nothing for the loop to do.
    pub fn tick(&mut self, head_joints: [f64; 4], posture: &Posture, dt_s: f64) -> Option<Note> {
        if matches!(self.state, State::Down) {
            return None;
        }
        let frame = self.latest.load_full();
        let fresh = frame
            .as_ref()
            .filter(|frame| frame.at.elapsed() < FRAME_STALE);

        // A stale sensor is silence with everything else held: the note fades, the
        // background is kept, and the instrument plays again the moment frames return.
        let Some(frame) = fresh else {
            return Some(Note {
                mouth: 0.0,
                state: self.state_block(None, None),
                closeness: None,
            });
        };
        // The same frame twice is not two observations. Without this an arming window could
        // fill from one frozen frame, and the background would be a picture of a stall.
        let repeated = self.last_armed_at == Some(frame.at);

        match &mut self.state {
            State::Down => None,
            State::Arming(arming) => {
                if !repeated {
                    arming.observe(&frame.ranges_m);
                    self.last_armed_at = Some(frame.at);
                }
                if arming.ready(&self.config) {
                    let outcome = arming.finish(&self.config, self.reprojector.beams());
                    self.state = match outcome {
                        Ok(background) => {
                            tracing::warn!(
                                background = %describe(&background.shape),
                                "theremin: armed"
                            );
                            State::Armed(Box::new(background))
                        }
                        Err(refusal) => {
                            tracing::warn!(reason = refusal.as_str(), "theremin: refused to arm");
                            State::Refused(refusal)
                        }
                    };
                }
                // Silent while arming — and the mouth closed, which is what makes the
                // half-second read as the duck getting ready rather than as a lag.
                Some(Note {
                    mouth: 0.0,
                    state: self.state_block(None, None),
                    closeness: None,
                })
            }
            State::Refused(_) => Some(Note {
                mouth: 0.0,
                state: self.state_block(None, None),
                closeness: None,
            }),
            State::Armed(background) => {
                let zones = self
                    .reprojector
                    .project(&frame.ranges_m, head_joints, posture);
                let hand = background.detect(&frame.ranges_m, &zones, &self.config);
                if !repeated {
                    background.relax(&frame.ranges_m, hand.as_ref(), &self.config, dt_s);
                    self.last_armed_at = Some(frame.at);
                }
                let closeness = hand.as_ref().map(|h| h.closeness);
                Some(Note {
                    // A silent theremin closes the beak; a played one opens it as far as the
                    // note is high, which is the same number.
                    mouth: closeness.unwrap_or(0.0),
                    state: self.state_block(hand.as_ref(), closeness),
                    closeness,
                })
            }
        }
    }

    /// The block clients watch. Assembled here rather than at the call site because half of
    /// it is this module's private state.
    fn state_block(&self, hand: Option<&Hand>, closeness: Option<f64>) -> proto::ThereminState {
        proto::ThereminState {
            armed: matches!(self.state, State::Armed(_)),
            refused: match &self.state {
                State::Refused(refusal) => Some(refusal.as_str().to_owned()),
                _ => None,
            },
            background: match &self.state {
                State::Armed(background) => Some(describe(&background.shape)),
                _ => None,
            },
            hand_range_m: hand.map(|h| h.range_m),
            // Filled in by the caller, which is the only one that knows this duck's
            // register — see `Sound::theremin_hz_at`.
            note_hz: None,
            mouth: closeness.unwrap_or(0.0),
        }
    }
}

/// One line saying what the theremin is measuring against — the thing that explains a
/// theremin behaving differently in two corners of the same room.
fn describe(shape: &hand::Shape) -> String {
    match shape {
        hand::Shape::Open { zones } => format!("open space ({zones} zones)"),
        hand::Shape::Plane { distance_m, rms_m } => {
            format!("plane at {distance_m:.2} m ({:.1} cm rms)", rms_m * 100.0)
        }
        hand::Shape::Cluttered { rms_m } => format!("cluttered ({:.1} cm rms)", rms_m * 100.0),
    }
}

/// Park on `tofd`'s depth stream, forever, leaving the newest frame in `slot`.
fn read_frames(socket: &Path, slot: &ArcSwapOption<Frame>) {
    loop {
        if let Err(why) = stream_frames(socket, slot) {
            tracing::debug!(why, "theremin: the depth stream ended");
        }
        // Nothing is playable without frames, and `tick` already treats a stale frame as
        // silence — so clear the slot rather than leaving a note hanging on the last thing
        // the sensor said before it went away.
        slot.store(None);
        std::thread::sleep(RECONNECT);
    }
}

/// One connection, from its subscribe to whatever ended it.
fn stream_frames(socket: &Path, slot: &ArcSwapOption<Frame>) -> Result<(), String> {
    let stream = UnixStream::connect(socket).map_err(|e| format!("connect: {e}"))?;
    let mut writer = stream.try_clone().map_err(|e| format!("clone: {e}"))?;
    let request = proto::Request::call(proto::Id::Number(SUBSCRIBE_ID), &proto::Call::TofStream);
    let line = serde_json::to_string(&request).map_err(|e| format!("encode: {e}"))?;
    writer
        .write_all(format!("{line}\n").as_bytes())
        .map_err(|e| format!("subscribe: {e}"))?;

    let mut reader = BufReader::new(stream);
    let mut line = String::new();
    loop {
        line.clear();
        match reader.read_line(&mut line) {
            Err(e) => return Err(format!("read: {e}")),
            Ok(0) => return Err("tofd closed the stream".to_owned()),
            Ok(_) => {}
        }
        // The subscription's answer names the sensor; everything after it is a frame.
        // Anything else is skipped rather than treated as an end — a future `tofd` may say
        // more than this build knows how to read.
        if let Ok(response) = serde_json::from_str::<proto::Response>(&line)
            && let Ok(status) = response.result_as::<proto::TofStreamResult>()
        {
            tracing::warn!(
                sensor = status.sensor.as_deref().unwrap_or("none"),
                unavailable = status.unavailable.as_deref().unwrap_or(""),
                hz = status.hz,
                "theremin: subscribed to the depth stream"
            );
            continue;
        }
        let Some(wire) = serde_json::from_str::<proto::Request>(&line)
            .ok()
            .and_then(|r| r.as_tof_frame())
        else {
            continue;
        };
        slot.store(Some(Arc::new(Frame {
            ranges_m: interpret(&wire),
            at: Instant::now(),
        })));
    }
}

/// Turn a wire frame into ranges in metres.
///
/// The status byte is the whole point: "nothing is in range" and "the measurement failed"
/// look identical in a distance-only view and mean opposite things to a background model —
/// but *both* are `None` here, because neither is a range, and the difference between them
/// belongs to a consumer that maps rather than to one that plays a note. The codes are the
/// `tof` crate's `Frame::zone`, which `robotd` cannot call without linking that crate's
/// vendored C.
fn interpret(wire: &proto::TofFrame) -> [Option<f64>; N_ZONES] {
    /// Status codes ST documents as a usable range: valid, and valid with a large pulse.
    const VALID: [u8; 2] = [5, 9];
    let mut out = [None; N_ZONES];
    for (zone, slot) in out.iter_mut().enumerate() {
        let Some(&status) = wire.status.get(zone) else {
            continue;
        };
        let Some(&mm) = wire.distance_mm.get(zone) else {
            continue;
        };
        // A negative range under a valid status comes back on a failed convergence; it is
        // not a measurement whatever the status says.
        if VALID.contains(&status) && mm > 0 {
            *slot = Some(f64::from(mm) / 1000.0);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn theremin() -> Theremin {
        // A socket path nothing listens on: the reader thread retries in the background and
        // the state machine under test is driven by frames pushed in directly.
        Theremin::spawn(
            PathBuf::from("/nonexistent/tofd.sock"),
            hand::Config::default(),
            Reprojector::alpha(),
        )
    }

    /// A wall at `distance`, as the sensor would report it: every beam reaches it at
    /// `d / (beam · axis)`.
    fn wall(distance: f64) -> [Option<f64>; N_ZONES] {
        let mut out = [None; N_ZONES];
        for (slot, beam) in out.iter_mut().zip(Reprojector::alpha().beams()) {
            *slot = Some(distance / beam[0]);
        }
        out
    }

    fn push(t: &Theremin, ranges_m: [Option<f64>; N_ZONES]) {
        t.latest.store(Some(Arc::new(Frame {
            ranges_m,
            at: Instant::now(),
        })));
    }

    /// Drive `frames` ticks, each with a distinct arrival time so the repeat guard does not
    /// eat them, and hand back the last note.
    fn run(t: &mut Theremin, ranges_m: [Option<f64>; N_ZONES], frames: usize) -> Option<Note> {
        let mut last = None;
        for _ in 0..frames {
            push(t, ranges_m);
            last = t.tick([0.0; 4], &Posture::default(), 1.0 / 15.0);
        }
        last
    }

    /// A theremin that was never picked up produces nothing at all — not silence, nothing:
    /// the loop must not be commanding a mouth for a feature nobody asked for.
    #[test]
    fn a_theremin_that_is_down_says_nothing() {
        let mut t = theremin();
        assert!(!t.active());
        push(&t, wall(0.5));
        assert!(t.tick([0.0; 4], &Posture::default(), 0.02).is_none());
    }

    /// Arming spans frames and is silent while it does, then the background is named.
    #[test]
    fn arming_takes_frames_and_then_names_its_background() {
        let mut t = theremin();
        t.set_active(true);
        assert!(t.active());

        let far = wall(1.5);
        let note = run(&mut t, far, 1).expect("up");
        assert!(!note.state.armed, "one frame is not a background");
        assert_eq!(note.mouth, 0.0, "the beak stays shut while arming");

        let note = run(&mut t, far, hand::Config::default().min_arming_frames).expect("up");
        assert!(note.state.armed, "{:?}", note.state);
        let background = note
            .state
            .background
            .expect("armed states name their background");
        assert!(background.starts_with("plane at 1.5"), "{background}");
    }

    /// The scenario the feature has to survive: armed in front of a wall the duck walked up
    /// to, then a hand between duck and wall. The wall is silent, the hand plays, and the
    /// mouth opens with the pitch.
    #[test]
    fn a_hand_in_front_of_the_wall_plays_and_the_wall_does_not() {
        let mut t = theremin();
        t.set_active(true);
        let wall_m = 0.45;
        let note = run(&mut t, wall(wall_m), 8).expect("up");
        assert!(note.state.armed, "{:?}", note.state);
        assert_eq!(note.mouth, 0.0, "the wall it armed against must be silent");
        assert_eq!(note.state.hand_range_m, None);

        // A hand in the middle of the frame, 15 cm in front of that wall.
        let mut hand_frame = wall(wall_m);
        for row in 2..6 {
            for col in 2..6 {
                hand_frame[row * COLS + col] = Some(wall_m - 0.15);
            }
        }
        let note = run(&mut t, hand_frame, 1).expect("up");
        let range = note.state.hand_range_m.expect("a hand");
        assert!((range - 0.30).abs() < 0.03, "{range}");
        assert!(note.mouth > 0.0, "the beak opens on a note");
        assert_eq!(note.mouth, note.state.mouth);
    }

    /// Closer is higher *and* wider: the mouth opening and the pitch are one number, so the
    /// two can never drift apart into a mouth animation over a sound.
    #[test]
    fn the_mouth_opens_with_the_note() {
        let mut t = theremin();
        t.set_active(true);
        run(&mut t, wall(1.5), 8);

        let at = |distance: f64, t: &mut Theremin| {
            let mut frame = wall(1.5);
            for row in 3..5 {
                for col in 3..5 {
                    frame[row * COLS + col] = Some(distance);
                }
            }
            run(t, frame, 1).expect("up").mouth
        };
        let far = at(0.55, &mut t);
        let mid = at(0.35, &mut t);
        let near = at(0.18, &mut t);
        assert!(far < mid && mid < near, "{far} {mid} {near}");
        assert!(
            near > 0.8,
            "a hand at the near end opens the beak wide: {near}"
        );
    }

    /// Arming with a hand already in front of the beak is refused, the refusal is readable
    /// by a client, and it survives until the instrument is picked up again — a refusal that
    /// vanished would present as a theremin that silently does nothing.
    #[test]
    fn a_refusal_is_reported_and_kept() {
        let mut t = theremin();
        t.set_active(true);
        let mut in_the_way = wall(1.5);
        for row in 3..6 {
            for col in 3..6 {
                in_the_way[row * COLS + col] = Some(0.22);
            }
        }
        let note = run(&mut t, in_the_way, 8).expect("up");
        assert!(!note.state.armed);
        assert_eq!(
            note.state.refused.as_deref(),
            Some(Refusal::SomethingInTheWay.as_str())
        );
        // Still refused several frames later, and still silent.
        let note = run(&mut t, wall(1.5), 5).expect("up");
        assert!(note.state.refused.is_some(), "{:?}", note.state);
        assert_eq!(note.mouth, 0.0);

        // Asking again re-arms — and now that nothing is in the way, it takes.
        t.set_active(true);
        let note = run(&mut t, wall(1.5), 8).expect("up");
        assert!(note.state.armed, "{:?}", note.state);
    }

    /// A sensor that stops delivering falls silent with everything else held, and plays
    /// again when it comes back — rather than holding the last note forever, or dropping the
    /// background and needing a re-arm.
    #[test]
    fn a_stale_frame_is_silence_and_not_a_lost_background() {
        let mut t = theremin();
        t.set_active(true);
        let mut hand_frame = wall(1.0);
        for row in 3..6 {
            for col in 3..6 {
                hand_frame[row * COLS + col] = Some(0.25);
            }
        }
        run(&mut t, wall(1.0), 8);
        assert!(run(&mut t, hand_frame, 1).expect("up").mouth > 0.0);

        // Age the frame past the staleness window.
        t.latest.store(Some(Arc::new(Frame {
            ranges_m: hand_frame,
            at: Instant::now() - FRAME_STALE * 2,
        })));
        let note = t.tick([0.0; 4], &Posture::default(), 0.02).expect("up");
        assert_eq!(note.mouth, 0.0, "a dead sensor closes the beak");
        assert_eq!(note.closeness, None, "and plays nothing");

        // Frames return; the background was kept, so the hand plays again straight away.
        let note = run(&mut t, hand_frame, 1).expect("up");
        assert!(note.mouth > 0.0, "the background survived the gap");
        assert!(note.state.armed);
    }

    /// A frozen sensor must not be able to fill an arming window with copies of one frame:
    /// that background would be a picture of a stall.
    #[test]
    fn a_repeated_frame_does_not_fill_the_arming_window() {
        let mut t = theremin();
        t.set_active(true);
        push(&t, wall(1.0));
        for _ in 0..50 {
            let note = t.tick([0.0; 4], &Posture::default(), 0.02).expect("up");
            assert!(
                !note.state.armed,
                "one frame, however often read, is one frame"
            );
        }
    }

    /// Walking puts the instrument down rather than quietly changing its zero: a background
    /// is a picture of where the duck was standing.
    #[test]
    fn moving_puts_the_instrument_down() {
        let mut t = theremin();
        t.set_active(true);
        run(&mut t, wall(0.5), 8);
        assert!(t.active());
        t.invalidate("walking");
        assert!(!t.active());
        assert!(t.tick([0.0; 4], &Posture::default(), 0.02).is_none());
    }

    /// Picking up an instrument already in hand must not restart its arming window — that
    /// would drop the background out from under a player mid-note.
    #[test]
    fn picking_up_twice_does_not_re_arm() {
        let mut t = theremin();
        t.set_active(true);
        run(&mut t, wall(0.5), 8);
        assert!(
            t.tick([0.0; 4], &Posture::default(), 0.02)
                .expect("up")
                .state
                .armed
        );
        t.set_active(true);
        let note = run(&mut t, wall(0.5), 1).expect("up");
        assert!(note.state.armed, "still armed, not arming again");
    }

    /// The wire's status byte decides what a distance means. A frame of failed measurements
    /// is not a frame of near returns, however small the numbers in it are.
    #[test]
    fn only_a_valid_status_is_a_range() {
        let wire = proto::TofFrame {
            seq: 1,
            at_us: 0,
            rows: ROWS as u8,
            cols: COLS as u8,
            distance_mm: vec![250; N_ZONES],
            status: {
                let mut status = vec![4u8; N_ZONES];
                status[0] = 5;
                status[1] = 9;
                status[2] = 255;
                status
            },
        };
        let ranges = interpret(&wire);
        assert_eq!(ranges[0], Some(0.25), "status 5 is a range");
        assert_eq!(ranges[1], Some(0.25), "status 9 is a range");
        assert_eq!(ranges[2], None, "status 255 measured nothing");
        assert_eq!(ranges[3], None, "status 4 failed");

        // A valid status over a negative distance is a failed convergence, not a range.
        let negative = proto::TofFrame {
            distance_mm: vec![-3; N_ZONES],
            status: vec![5; N_ZONES],
            ..wire
        };
        assert_eq!(interpret(&negative)[0], None);

        // A short frame from a peer of another release must not panic or read past its end.
        let ragged = proto::TofFrame {
            distance_mm: vec![300; 3],
            status: vec![5; 2],
            ..Default::default()
        };
        let ranges = interpret(&ragged);
        assert_eq!(ranges[0], Some(0.3));
        assert_eq!(ranges[2], None, "no status for this zone");
        assert_eq!(ranges[N_ZONES - 1], None, "past the end");
    }
}
