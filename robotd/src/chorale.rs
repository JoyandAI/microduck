//! Several ducks singing one piece: who conducts, who sings what, and where in the score we are.
//!
//! The behaviour half of the duck chorale. `btd` owns the radio and does no thinking; this owns the
//! thinking and touches no radio. What passes between them is `chorale.*`: a beacon to advertise
//! going down, and beacons heard coming up.
//!
//! ## Nobody is in charge until somebody is
//!
//! A duck asked for a chorale starts *listening*: it advertises an idle beacon saying it is
//! willing, and watches for others. When two willing ducks see each other the lower id conducts —
//! deterministic, so there is no election to go wrong and no message to lose. A duck that hears a
//! beacon already carrying a piece does not argue about it; it joins.
//!
//! ## The conductor owns the seating, and that is not a convenience
//!
//! Seating depends on join order ([`sounds::chorale::seat`]), so a duck seating *itself* from
//! whatever it happened to hear will disagree with a duck that heard a different subset — and both
//! will sing alto. So the conductor keeps the roster, broadcasts it, and everyone replays
//! [`sounds::chorale::seat_all`] over it. One source of truth, which is what a conductor is.
//!
//! A duck that cannot find itself in the roster is not in the piece yet: it keeps listening, and
//! the conductor adds it on the next beat. That is why joining is free rather than negotiated.
//!
//! ## Where the score position comes from
//!
//! Not a start time. There is no clock to agree on, so the conductor's beat counter *is* the
//! timebase: [`sounds::chorale::beat::Conductor`] for the duck holding it and `Follower` for
//! everyone else, and both answer the same question — how many beats into the piece are we, right
//! now. The audio side reads that and renders the score at it, so a duck whose audio stalls
//! resumes in the right place rather than a bar behind.

use std::time::{Duration, Instant};

use duck_ipc_proto as proto;
use sounds::chorale::beat::{Conductor, Follower};
use sounds::chorale::{Part, Score, seat_all};

/// How long a heard beacon counts for.
///
/// A duck that has walked out of range stops being in the piece — otherwise a chorale would hold a
/// seat for a duck that left, and the roster it broadcasts would name someone who is not singing.
/// Generous against a missed advertisement: at the beacon's interval this is dozens of chances.
const PEER_STALE: Duration = Duration::from_secs(3);

/// How long to listen before starting a piece alone.
///
/// A duck that has heard nobody does not sing: a solo chorale is a duck quacking to itself. This is
/// only the settling time before *two* ducks that can both see each other agree who conducts —
/// long enough that they have both certainly heard the other, so they cannot both decide they are
/// alone.
const SETTLE: Duration = Duration::from_millis(1500);

/// Which piece. One for now; the field exists so a beacon can say what is being sung, and so a
/// duck joining knows what to load.
const PIECE_WISTFUL: u8 = 1;

/// Another duck, as last heard.
#[derive(Debug, Clone)]
struct Peer {
    beacon: proto::ChoraleBeacon,
    /// Local time the beacon was heard — `btd` reports an age, and this is that age subtracted
    /// from the clock on arrival. The two daemons share a machine but not an epoch.
    at: Instant,
}

/// What this duck is doing.
enum State {
    /// Not asked for, or not allowed. Nothing on the air.
    Off,
    /// Willing: an idle beacon out, and listening.
    Listening { since: Instant },
    /// Holding the beat for everyone.
    Conducting {
        conductor: Conductor,
        /// The roster, in seating order, as `(register, id)`. The conductor's own copy is the
        /// authority — everyone else reads it off the beacon.
        roster: Vec<(u8, u8)>,
    },
    /// Singing to somebody else's beat.
    Following {
        follower: Follower,
        /// The conductor, so a second conductor appearing does not pull this duck off its beat.
        /// Empty until the first singing beacon names one.
        conductor: String,
        roster: Vec<(u8, u8)>,
        /// The counter last taken, so a repeated beacon is not counted as another beat.
        last_beat: Option<u8>,
    },
}

/// What the loop should do about the chorale this tick.
#[derive(Debug, Clone, PartialEq)]
pub struct Tick {
    /// What to ask `btd` to advertise, when it has changed. `None` means "no change" — the beacon
    /// only needs resending when the beat turns over, not fifty times a second.
    pub advertise: Option<proto::ChoraleAdvertise>,
    /// Which part this duck is singing, and how far into the score, in beats. `None` when it is not
    /// singing — listening, alone, or off.
    pub singing: Option<(Part, f64)>,
    /// How many ducks are actually singing, this one included.
    ///
    /// Not the roster's length: a duck that walks out of range keeps its *seat* — pruning it would
    /// shift everyone below it onto a different part, which is the one thing the roster exists to
    /// prevent — so its line simply goes unsung, exactly as it would in a choir somebody left.
    pub voices: usize,
}

pub struct Chorale {
    /// This duck's own beacon identity: its register, quantised, and a tie-break byte.
    register: u8,
    id: u8,
    /// The epoch every time in this module is measured from.
    ///
    /// `sounds::chorale::beat` works in plain seconds and `robotd` works in `Instant`s, so one of
    /// them has to convert. Doing it here keeps the beat maths free of anything platform-shaped,
    /// which is what lets it be tested against simulated jitter on a laptop.
    started: Instant,
    score: Score,
    state: State,
    peers: Vec<Peer>,
    /// The beacon last handed to `btd`, so it is only resent when it changes.
    advertised: Option<proto::ChoraleBeacon>,
    listening: bool,
}

impl Chorale {
    /// `seed` is the robot's voice seed — the identity everything else is derived from, and which
    /// deliberately does not go on the air.
    pub fn new(pitch_center_hz: f64, seed: u32, score: Score) -> Self {
        Self {
            register: proto::ChoraleBeacon::quantise_register(pitch_center_hz),
            // A byte of the seed, mixed. Enough to break a tie between two ducks that rolled the
            // same register, and not enough to identify a robot.
            id: (seed.wrapping_mul(2_654_435_761) >> 24) as u8,
            started: Instant::now(),
            score,
            state: State::Off,
            peers: Vec::new(),
            advertised: None,
            listening: false,
        }
    }

    pub fn active(&self) -> bool {
        !matches!(self.state, State::Off)
    }

    /// Start listening for other ducks, or stop and fall silent.
    pub fn set_active(&mut self, active: bool, now: Instant) {
        match (active, &self.state) {
            (true, State::Off) => {
                tracing::warn!(register = self.register, id = self.id, "chorale: listening");
                self.state = State::Listening { since: now };
            }
            (true, _) => {}
            (false, _) => {
                if self.active() {
                    tracing::warn!("chorale: stopping");
                }
                self.state = State::Off;
                self.peers.clear();
            }
        }
    }

    /// A beacon `btd` heard.
    pub fn heard(&mut self, heard: &proto::ChoraleHeard, now: Instant) {
        if !self.active() {
            return;
        }
        // Our own beacon, reflected by something. Not a peer.
        if heard.beacon.register == self.register && heard.beacon.id == self.id {
            return;
        }
        // `btd` reports an age rather than a timestamp — the two daemons share a machine but not an
        // epoch — so the arrival time is that age subtracted from the clock here.
        let at = now
            .checked_sub(Duration::from_micros(heard.age_us))
            .unwrap_or(now);
        let peer = Peer {
            beacon: heard.beacon.clone(),
            at,
        };
        match self
            .peers
            .iter_mut()
            .find(|existing| existing.beacon.id == peer.beacon.id)
        {
            Some(existing) => *existing = peer,
            None => {
                tracing::warn!(
                    register = heard.beacon.register,
                    id = heard.beacon.id,
                    from = %heard.from,
                    "chorale: another duck"
                );
                self.peers.push(peer);
            }
        }

        // A beat from the duck we are following: hand it to the phase lock, stamped when it was
        // heard rather than when it was processed.
        let arrival = at.saturating_duration_since(self.started).as_secs_f64();
        if let State::Following {
            follower,
            conductor,
            roster,
            last_beat,
        } = &mut self.state
        {
            // The beacon carries no address, so the first singing beacon adopts its sender as *the*
            // conductor. After that another duck's beacon cannot pull this one off its beat — which
            // matters when two pieces briefly overlap in one room.
            if conductor.is_empty() && heard.beacon.singing() {
                *conductor = heard.from.clone();
                tracing::warn!(conductor = %heard.from, "chorale: following");
            }
            if *conductor != heard.from {
                return;
            }
            roster.clone_from(&heard.beacon.roster);
            // **Only on a change of counter.** A beacon repeats several times per beat, and
            // re-reading the same value is not another beat — it is the same beat, later, and
            // feeding those in would drag the phase late by half an advertising interval.
            if *last_beat != Some(heard.beacon.beat) {
                *last_beat = Some(heard.beacon.beat);
                follower.observe(heard.beacon.beat, arrival);
            }
        }
    }

    /// Seconds since this module's epoch — the clock `sounds::chorale::beat` speaks in.
    fn seconds(&self, at: Instant) -> f64 {
        at.saturating_duration_since(self.started).as_secs_f64()
    }

    /// One tick. Cheap enough for every one; only the beacon is rate-limited, by changing rarely.
    pub fn tick(&mut self, now: Instant) -> Tick {
        self.peers
            .retain(|peer| now.saturating_duration_since(peer.at) < PEER_STALE);

        match &mut self.state {
            State::Off => {
                let advertise = self.publish(None, false);
                Tick {
                    advertise,
                    singing: None,
                    voices: 0,
                }
            }
            State::Listening { since } => {
                let since = *since;
                // Somebody is already singing: join it rather than starting a second piece.
                if let Some(peer) = self
                    .peers
                    .iter()
                    .find(|peer| peer.beacon.singing())
                    .cloned()
                {
                    tracing::warn!(
                        piece = peer.beacon.piece,
                        conductor = peer.beacon.id,
                        "chorale: joining"
                    );
                    self.state = State::Following {
                        follower: Follower::new(self.score.bpm),
                        conductor: String::new(),
                        roster: peer.beacon.roster.clone(),
                        last_beat: None,
                    };
                    // The address is not in the beacon, so the first `heard` from this conductor
                    // adopts it — until then this duck listens without a lock, which is what
                    // `Follower` does anyway for its first few beats.
                    return self.tick(now);
                }
                // Nobody singing. If anyone is here and this duck has the lowest id, conduct.
                let alone = self.peers.is_empty();
                let lowest = self.peers.iter().all(|peer| self.id < peer.beacon.id);
                if !alone && lowest && now.saturating_duration_since(since) >= SETTLE {
                    let mut roster: Vec<(u8, u8)> = vec![(self.register, self.id)];
                    roster.extend(self.peers.iter().map(|p| (p.beacon.register, p.beacon.id)));
                    roster.truncate(proto::ChoraleBeacon::MAX_ROSTER);
                    tracing::warn!(voices = roster.len(), "chorale: conducting");
                    let roster = roster;
                    self.state = State::Conducting {
                        conductor: Conductor::new(self.score.bpm, self.seconds(now)),
                        roster,
                    };
                    return self.tick(now);
                }
                let idle = self.beacon(proto::ChoraleBeacon::IDLE, 0, Vec::new());
                let advertise = self.publish(Some(idle), true);
                Tick {
                    advertise,
                    singing: None,
                    voices: 0,
                }
            }
            State::Conducting { .. } => self.conduct(now),
            State::Following { .. } => self.follow(now),
        }
    }

    fn conduct(&mut self, now: Instant) -> Tick {
        // Everyone in range who is willing, in join order: the roster grows as ducks arrive and
        // never reorders, which is what keeps anyone already singing on their own part.
        let heard: Vec<(u8, u8)> = self
            .peers
            .iter()
            .map(|p| (p.beacon.register, p.beacon.id))
            .collect();
        let State::Conducting { conductor, roster } = &mut self.state else {
            unreachable!("checked by the caller");
        };
        for entry in heard {
            if roster.len() < proto::ChoraleBeacon::MAX_ROSTER && !roster.contains(&entry) {
                tracing::warn!(id = entry.1, "chorale: seating a new voice");
                roster.push(entry);
            }
        }
        let seconds = now.saturating_duration_since(self.started).as_secs_f64();
        conductor.due(seconds);
        let position = conductor.position_beats(seconds);
        let beat = conductor.wire_beat();
        let roster = roster.clone();

        let beacon = self.beacon(PIECE_WISTFUL, beat, roster.clone());
        let advertise = self.publish(Some(beacon), true);
        Tick {
            advertise,
            singing: self.my_part(&roster).map(|part| (part, position)),
            voices: self.voices(&roster),
        }
    }

    /// Roster entries that are still audible: this duck, plus peers heard recently enough.
    ///
    /// The roster keeps a departed duck's seat so nobody else is reseated; this is the count that
    /// says how many are actually singing.
    fn voices(&self, roster: &[(u8, u8)]) -> usize {
        roster
            .iter()
            .filter(|(register, id)| {
                (*register == self.register && *id == self.id)
                    || self.peers.iter().any(|peer| peer.beacon.id == *id)
            })
            .count()
    }

    fn follow(&mut self, now: Instant) -> Tick {
        let State::Following {
            follower, roster, ..
        } = &mut self.state
        else {
            unreachable!("checked by the caller");
        };
        let roster = roster.clone();
        let seconds = now.saturating_duration_since(self.started).as_secs_f64();
        let position = follower.position_beats(seconds);
        // A follower advertises an idle beacon: it is willing and findable, but it is not the one
        // holding the beat, and two beacons carrying a piece would be two conductors.
        let idle = self.beacon(proto::ChoraleBeacon::IDLE, 0, Vec::new());
        let advertise = self.publish(Some(idle), true);
        Tick {
            advertise,
            singing: position.and_then(|at| self.my_part(&roster).map(|part| (part, at))),
            voices: self.voices(&roster),
        }
    }

    /// This duck's part, from the roster the conductor published.
    ///
    /// `None` when it is not in the roster yet — which is not an error but the ordinary state of a
    /// duck that has just arrived. It keeps listening, and the conductor seats it on the next beat.
    fn my_part(&self, roster: &[(u8, u8)]) -> Option<Part> {
        let seat = roster
            .iter()
            .position(|(register, id)| *register == self.register && *id == self.id)?;
        let registers: Vec<f64> = roster
            .iter()
            .map(|(register, _)| {
                proto::ChoraleBeacon::REGISTER_LOW_HZ
                    + f64::from(*register) * proto::ChoraleBeacon::REGISTER_STEP_HZ
            })
            .collect();
        seat_all(&registers).get(seat).copied()
    }

    fn beacon(&self, piece: u8, beat: u8, roster: Vec<(u8, u8)>) -> proto::ChoraleBeacon {
        proto::ChoraleBeacon {
            piece,
            beat,
            register: self.register,
            id: self.id,
            roster,
        }
    }

    /// Hand `btd` a beacon, but only when it has actually changed.
    ///
    /// The loop runs at 50 Hz and the beat turns over about once a second, so this is the
    /// difference between two D-Bus round trips a second and a hundred.
    fn publish(
        &mut self,
        beacon: Option<proto::ChoraleBeacon>,
        listening: bool,
    ) -> Option<proto::ChoraleAdvertise> {
        if beacon == self.advertised && listening == self.listening {
            return None;
        }
        self.advertised = beacon.clone();
        self.listening = listening;
        Some(proto::ChoraleAdvertise { beacon, listening })
    }

    /// The score being sung, for the audio side to render.
    pub fn score(&self) -> &Score {
        &self.score
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn chorale() -> Chorale {
        Chorale::new(214.4, 7, Score::wistful())
    }

    /// A duck that hears nobody must not sing. A solo chorale is a duck quacking to itself.
    #[test]
    fn one_duck_alone_does_not_sing() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now);
        for step in 0..200 {
            let tick = c.tick(now + Duration::from_millis(50 * step));
            assert_eq!(tick.singing, None, "step {step}");
        }
        // But it is on the air, saying it is willing, and listening for company.
        let advertise = {
            let mut c = chorale();
            c.set_active(true, now);
            c.tick(now).advertise.expect("something to advertise")
        };
        assert!(advertise.listening);
        let beacon = advertise.beacon.expect("an idle beacon");
        assert!(!beacon.singing());
        assert!(beacon.roster.is_empty());
    }

    /// Off means off: nothing on the air, nothing heard, nothing sung. This is what
    /// `[chorale] accept = false` buys — invisible rather than visibly declining.
    #[test]
    fn a_duck_that_was_not_asked_is_silent_and_invisible() {
        let mut c = chorale();
        let now = Instant::now();
        let tick = c.tick(now);
        assert_eq!(tick.singing, None);
        // Nothing to *say*, because nothing has to change: `btd` starts out advertising nothing and
        // scanning for nothing, which is already what an unasked duck wants.
        assert_eq!(tick.advertise, None);

        // A beacon arriving is ignored rather than answered.
        c.heard(&heard_from(2, 120, 1, 5, vec![(120, 2)]), now);
        assert_eq!(c.tick(now).singing, None);
        assert!(!c.active());
    }

    fn heard_from(
        id: u8,
        register: u8,
        piece: u8,
        beat: u8,
        roster: Vec<(u8, u8)>,
    ) -> proto::ChoraleHeard {
        proto::ChoraleHeard {
            beacon: proto::ChoraleBeacon {
                piece,
                beat,
                register,
                id,
                roster,
            },
            from: format!("AA:BB:CC:DD:EE:{id:02X}"),
            age_us: 2_000,
        }
    }

    /// Two willing ducks agree who conducts with no election and no message: the lower id does. Both
    /// ducks reach the same answer from the same beacons, which is why there is nothing to lose.
    #[test]
    fn the_lower_id_conducts_and_the_other_follows() {
        let now = Instant::now();
        // This duck's id, from seed 7.
        let mine = chorale().id;
        let higher = mine.wrapping_add(1);
        let lower = mine.wrapping_sub(1);

        let mut leads = chorale();
        leads.set_active(true, now);
        leads.heard(
            &heard_from(higher, 200, proto::ChoraleBeacon::IDLE, 0, vec![]),
            now,
        );
        let tick = leads.tick(now + SETTLE);
        assert!(
            tick.advertise
                .as_ref()
                .and_then(|a| a.beacon.as_ref())
                .is_some_and(|b| b.singing()),
            "the lower id should be conducting: {tick:?}"
        );
        assert_eq!(tick.voices, 2);

        let mut defers = chorale();
        defers.set_active(true, now);
        defers.heard(
            &heard_from(lower, 200, proto::ChoraleBeacon::IDLE, 0, vec![]),
            now,
        );
        let tick = defers.tick(now + SETTLE);
        assert!(
            tick.advertise
                .as_ref()
                .and_then(|a| a.beacon.as_ref())
                .is_none_or(|b| !b.singing()),
            "the higher id must not also conduct: {tick:?}"
        );
    }

    /// A duck hearing a piece already under way joins it rather than starting a second one — which
    /// is the whole of "and other ducks can then join".
    #[test]
    fn a_duck_arriving_late_joins_what_it_finds() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now);
        // A conductor already singing, with this duck already seated in its roster.
        let mine = (c.register, c.id);
        let roster = vec![(120u8, 9u8), mine];
        c.heard(&heard_from(9, 120, PIECE_WISTFUL, 4, roster.clone()), now);
        let tick = c.tick(now);
        assert_eq!(tick.voices, 2, "{tick:?}");
        // It does not conduct — there is already a conductor — and its own beacon stays idle.
        let beacon = tick
            .advertise
            .as_ref()
            .and_then(|a| a.beacon.as_ref())
            .expect("still findable");
        assert!(!beacon.singing(), "two conductors: {beacon:?}");
    }

    /// The conductor seats a newcomer without moving anyone: the roster grows and never reorders.
    #[test]
    fn the_roster_grows_and_never_reorders() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now);
        let mine = c.id;
        // Two ducks with higher ids, so this one conducts.
        c.heard(
            &heard_from(
                mine.wrapping_add(1),
                250,
                proto::ChoraleBeacon::IDLE,
                0,
                vec![],
            ),
            now,
        );
        let first = c
            .tick(now + SETTLE)
            .advertise
            .and_then(|a| a.beacon)
            .expect("conducting");
        assert_eq!(first.roster.len(), 2);

        c.heard(
            &heard_from(
                mine.wrapping_add(2),
                60,
                proto::ChoraleBeacon::IDLE,
                0,
                vec![],
            ),
            now + SETTLE,
        );
        let second = c
            .tick(now + SETTLE + Duration::from_millis(20))
            .advertise
            .and_then(|a| a.beacon)
            .expect("still conducting");
        assert_eq!(second.roster.len(), 3);
        assert_eq!(
            &second.roster[..2],
            &first.roster[..],
            "the newcomer went on the end"
        );
    }

    /// A duck that walks out of range stops being in the piece, or the roster names someone who is
    /// not singing and a part goes missing.
    #[test]
    fn a_duck_that_leaves_stops_being_counted() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now);
        c.heard(
            &heard_from(
                c.id.wrapping_add(1),
                250,
                proto::ChoraleBeacon::IDLE,
                0,
                vec![],
            ),
            now,
        );
        assert_eq!(c.tick(now + SETTLE).voices, 2);
        // Long enough that its last beacon is stale.
        let later = now + SETTLE + PEER_STALE + Duration::from_millis(100);
        let tick = c.tick(later);
        assert_eq!(tick.voices, 1, "{tick:?}");
        // But its seat survives, so this duck is still singing the part it started on rather than
        // being reseated mid-piece.
        let beacon = tick
            .advertise
            .and_then(|a| a.beacon)
            .expect("still conducting");
        assert_eq!(beacon.roster.len(), 2, "the seat is kept: {beacon:?}");
        assert_eq!(tick.singing.expect("still singing").0, Part::Soprano);
    }

    /// The beacon is resent only when it changes. The loop runs at 50 Hz and the beat turns over
    /// about once a second; the difference is two D-Bus round trips a second against a hundred.
    #[test]
    fn the_beacon_is_only_published_when_it_changes() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now);
        assert!(
            c.tick(now).advertise.is_some(),
            "the first one has to go out"
        );
        for step in 1..20 {
            assert_eq!(
                c.tick(now + Duration::from_millis(10 * step)).advertise,
                None,
                "nothing changed at step {step}"
            );
        }
        // And a peer arriving *does* change it, eventually — the roster is in the beacon.
        c.heard(
            &heard_from(
                c.id.wrapping_add(1),
                250,
                proto::ChoraleBeacon::IDLE,
                0,
                vec![],
            ),
            now,
        );
        assert!(c.tick(now + SETTLE).advertise.is_some());
    }

    /// A duck not yet in the conductor's roster does not sing — it has no part, and guessing one is
    /// how two ducks end up on the same line.
    #[test]
    fn a_duck_not_yet_seated_waits_rather_than_guessing() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now);
        // A piece under way whose roster does not mention this duck.
        c.heard(
            &heard_from(9, 120, PIECE_WISTFUL, 4, vec![(120, 9), (250, 11)]),
            now,
        );
        let tick = c.tick(now);
        assert_eq!(tick.singing, None, "no seat, no part: {tick:?}");
    }
}
