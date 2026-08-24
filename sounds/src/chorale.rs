//! Several ducks singing one thing: the score, who sings what, and the mix.
//!
//! This module is the *musical* half of the duck chorale, and it is deliberately separate from
//! the half that will be hard on real hardware (finding each other, agreeing on a clock). It
//! renders an ensemble offline, on a laptop, so the arrangement can be judged before a single
//! packet is sent between two ducks — and so that when the sync work starts, "does it sound
//! good" is already answered and only "is it together" is in question.
//!
//! ## Identity is timbre, not pitch
//!
//! Every duck's voice is derived from its SoC serial, and the loudest thing that varies is
//! [`Personality::pitch_center_hz`] — a duck is high or low. Letting that shift the *notes*
//! would be the obvious way to keep each duck sounding like itself, and it would wreck the
//! piece: four ducks singing a chord each in their own tuning is four ducks out of tune with
//! each other. Beating, not harmony.
//!
//! So the note is absolute, from one shared reference ([`A4_HZ`], equal temperament), and what
//! each duck keeps is everything else: harmonic weights, formant, nasality, breath, and the
//! tamed remains of its vibrato ([`Stream::choral`]). Register is used instead for **casting**
//! — the lowest duck sings bass — and for choosing what key the piece lands in, so nobody is
//! asked to sing outside the range their own voice was rolled for.
//!
//! ## Why a perfectly synchronised chorus is the wrong target
//!
//! Four voices starting a note on the same sample and holding the same frequency do not sound
//! like a choir; they sound like one organ with a thick stop. What makes an ensemble is that
//! its members are *almost* together and *almost* in tune: a few cents of pitch spread and a
//! few tens of milliseconds of onset spread. Both are added here on purpose, derived from each
//! duck's seed so a given group always sounds like that group ([`Singer::detune_cents`],
//! [`Singer::onset_offset_s`]).
//!
//! That is also the answer to how tightly real ducks will have to agree on a clock: the target
//! is ±20 ms, not ±1 ms, because ±15 ms is what we are deliberately adding. A chord's *tuning*
//! is what has to be exact, and that needs no synchronisation at all — only a shared reference
//! pitch, which is a constant.

use crate::personality::Personality;
use crate::stream::Stream;
use crate::synth::SR;

/// The reference the whole ensemble tunes to. A constant, which is the point: tuning needs no
/// agreement at run time, only the same number compiled into every duck.
pub const A4_HZ: f64 = 440.0;

/// The four parts, low to high. `as usize` indexes a [`Chord`]'s voicing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Part {
    Bass,
    Tenor,
    Alto,
    Soprano,
}

impl Part {
    pub const ALL: [Part; 4] = [Part::Bass, Part::Tenor, Part::Alto, Part::Soprano];

    pub fn as_str(&self) -> &'static str {
        match self {
            Part::Bass => "bass",
            Part::Tenor => "tenor",
            Part::Alto => "alto",
            Part::Soprano => "soprano",
        }
    }

    /// Which parts an ensemble of `n` ducks sings.
    ///
    /// Not simply the lowest `n`: a chord needs its outer voices most, so a duet is bass and
    /// soprano — melody over a bass line — rather than bass and tenor, which would be two
    /// ducks muttering in the same octave. A trio drops the tenor, the voice whose notes are
    /// most often doubled elsewhere in the chord.
    pub fn ensemble(n: usize) -> Vec<Part> {
        match n {
            0 => Vec::new(),
            1 => vec![Part::Soprano],
            2 => vec![Part::Bass, Part::Soprano],
            3 => vec![Part::Bass, Part::Alto, Part::Soprano],
            _ => Part::ALL.to_vec(),
        }
    }
}

/// One part's note: which voice, when it enters, how long it holds.
///
/// The flat form everything downstream reads. [`Gesture`] is what a score is *written* in;
/// this is what it compiles to.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Note {
    pub part: Part,
    pub start_beat: f64,
    pub beats: f64,
    pub midi: u8,
}

impl Note {
    fn end_beat(&self) -> f64 {
        self.start_beat + self.beats
    }
}

/// A voicing with a tacet voice allowed: `None` is a part that does not sing here.
pub type Voicing = [Option<u8>; 4];

/// Four voices in unison rhythm — the plain block chord.
fn chord(bass: u8, tenor: u8, alto: u8, soprano: u8) -> Voicing {
    [Some(bass), Some(tenor), Some(alto), Some(soprano)]
}

/// What a score is written in.
///
/// Not a list of block chords, which is where this started and which can only write one kind
/// of music: a real chorale breathes, assembles chords from below, and drops to one voice.
/// Each variant here is a *gesture* an arranger actually thinks in, and it compiles down to
/// per-part [`Note`]s — so the source reads as the musical intent rather than as a table of
/// simultaneities.
#[derive(Debug, Clone, PartialEq)]
pub enum Gesture {
    /// Everyone together, one duration. The homophonic tread.
    Chord { voicing: Voicing, beats: f64 },
    /// Voices enter one at a time, `stagger` beats apart, and each holds to the end of the
    /// gesture — so the chord *assembles* and is then sustained whole.
    ///
    /// The single most effective thing a group of singers can do that a keyboard cannot, and
    /// on hardware it is also the most forgiving: a chord whose entries are deliberately
    /// 0.5 s apart does not care whether two ducks disagree by 20 ms.
    Build {
        voicing: Voicing,
        beats: f64,
        stagger: f64,
        /// Enter from the top voice down instead of the bass up.
        from_top: bool,
    },
    /// One voice moves while the others hold a chord under it.
    ///
    /// The gesture lasts as long as the solo line does. Parts left `None` in `under` are
    /// silent for it — a solo over nothing at all is `under: [None; 4]`, which is what a
    /// genuinely unaccompanied entry is.
    Solo {
        part: Part,
        /// `(midi, beats)`, in order.
        notes: Vec<(u8, f64)>,
        under: Voicing,
    },
    /// Silence for everyone. A breath, and the thing that makes the chord after it land.
    Rest { beats: f64 },
}

impl Gesture {
    /// How many beats this gesture occupies.
    pub fn beats(&self) -> f64 {
        match self {
            Gesture::Chord { beats, .. } | Gesture::Build { beats, .. } => *beats,
            Gesture::Solo { notes, .. } => notes.iter().map(|(_, b)| b).sum(),
            Gesture::Rest { beats } => *beats,
        }
    }
}

/// A four-part score.
#[derive(Debug, Clone, PartialEq)]
pub struct Score {
    pub name: &'static str,
    pub bpm: f64,
    pub gestures: Vec<Gesture>,
}

impl Score {
    /// The default piece: an original arrangement, ours to ship.
    ///
    /// Written for this rather than borrowed, and that is a licensing fact as much as a
    /// musical one — the close-harmony records this is meant to evoke are all in copyright, so
    /// the *idiom* is the reference and the notes are not. What it borrows is the grammar: a
    /// slow homophonic tread, chords in inversion so the bass line moves by step, common tones
    /// held across changes rather than re-attacked, suspensions that resolve late, and one
    /// borrowed minor chord (an F minor in C major) which is where the ache in this kind of
    /// writing always comes from.
    ///
    /// It opens and closes with the chord *assembling* from the bass up, has a soprano line
    /// over a held triad in the middle, and takes a breath before the last entry — the three
    /// things that make a group of singers sound like singers rather than like one instrument
    /// with four oscillators.
    ///
    /// Voicings stay inside ranges a duck can sing and a small speaker can reproduce: bass
    /// A2–A3, soprano D4–D5. A duck's voice bottoms out around 110 Hz, and a chorale whose
    /// bass is inaudible on the hardware is a trio.
    pub fn wistful() -> Self {
        Self {
            name: "wistful",
            bpm: 58.0,
            gestures: vec![
                // The chord assembles, low to high, and holds.
                Gesture::Build {
                    voicing: chord(48, 55, 60, 64),
                    beats: 6.0,
                    stagger: 0.75,
                    from_top: false,
                },
                // The tread. The alto's C4 is the thread these hang on; the bass walks under it.
                Gesture::Chord {
                    voicing: chord(45, 55, 60, 64),
                    beats: 2.0,
                }, // Am7
                Gesture::Chord {
                    voicing: chord(53, 57, 60, 65),
                    beats: 2.0,
                }, // F
                Gesture::Chord {
                    voicing: chord(53, 56, 60, 65),
                    beats: 3.0,
                }, // Fm — the ache
                Gesture::Chord {
                    voicing: chord(52, 55, 60, 64),
                    beats: 3.0,
                }, // C/E
                // One voice over a held triad. The alto keeps its C4 and the soprano moves.
                Gesture::Solo {
                    part: Part::Soprano,
                    notes: vec![(64, 1.0), (65, 1.0), (67, 2.0), (65, 1.0), (64, 3.0)],
                    under: [Some(48), Some(55), Some(60), None],
                },
                Gesture::Chord {
                    voicing: chord(50, 57, 60, 65),
                    beats: 2.0,
                }, // Dm7
                Gesture::Chord {
                    voicing: chord(55, 59, 62, 65),
                    beats: 2.0,
                }, // G7
                Gesture::Chord {
                    voicing: chord(48, 55, 60, 65),
                    beats: 2.0,
                }, // Csus4
                Gesture::Chord {
                    voicing: chord(48, 55, 60, 64),
                    beats: 4.0,
                }, // C, arrived at
                // The same trick from the other end: the soprano enters first, the bass last.
                Gesture::Build {
                    voicing: chord(45, 57, 60, 64),
                    beats: 5.0,
                    stagger: 0.5,
                    from_top: true,
                },
                Gesture::Chord {
                    voicing: chord(53, 57, 60, 65),
                    beats: 2.0,
                }, // F
                Gesture::Chord {
                    voicing: chord(53, 56, 60, 65),
                    beats: 3.0,
                }, // Fm again
                Gesture::Chord {
                    voicing: chord(55, 60, 62, 65),
                    beats: 2.0,
                }, // G7sus4
                Gesture::Chord {
                    voicing: chord(55, 59, 62, 65),
                    beats: 2.0,
                }, // G7
                // The breath. Short, and it is what makes the last chord land.
                Gesture::Rest { beats: 0.75 },
                Gesture::Build {
                    voicing: chord(48, 55, 60, 64),
                    beats: 8.0,
                    stagger: 0.6,
                    from_top: false,
                },
            ],
        }
    }

    /// Seconds per beat.
    pub fn beat_s(&self) -> f64 {
        60.0 / self.bpm.max(1.0)
    }

    /// How long the piece runs, seconds.
    pub fn duration_s(&self) -> f64 {
        self.gestures.iter().map(|g| g.beats()).sum::<f64>() * self.beat_s()
    }

    /// Compile the gestures into per-part notes, with ties merged.
    ///
    /// A voice holding the same pitch across a gesture boundary comes out as **one** note, not
    /// two — which is how part-writing actually works, and audibly the difference between a
    /// chorale and a sequence of chords. A [`Gesture::Rest`] breaks a tie by leaving a gap,
    /// which is the whole point of putting one there.
    pub fn notes(&self) -> Vec<Note> {
        let mut notes: Vec<Note> = Vec::new();
        let mut at = 0.0f64;
        for gesture in &self.gestures {
            match gesture {
                Gesture::Chord { voicing, beats } => {
                    for part in Part::ALL {
                        if let Some(midi) = voicing[part as usize] {
                            notes.push(Note {
                                part,
                                start_beat: at,
                                beats: *beats,
                                midi,
                            });
                        }
                    }
                }
                Gesture::Build {
                    voicing,
                    beats,
                    stagger,
                    from_top,
                } => {
                    let mut order: Vec<Part> = Part::ALL
                        .into_iter()
                        .filter(|p| voicing[*p as usize].is_some())
                        .collect();
                    if *from_top {
                        order.reverse();
                    }
                    for (seat, part) in order.into_iter().enumerate() {
                        let offset = seat as f64 * stagger;
                        // A stagger long enough to run past the gesture would give the last
                        // voice a negative-length note; it simply does not get in.
                        let held = beats - offset;
                        if held <= 0.0 {
                            continue;
                        }
                        notes.push(Note {
                            part,
                            start_beat: at + offset,
                            beats: held,
                            midi: voicing[part as usize].expect("filtered"),
                        });
                    }
                }
                Gesture::Solo {
                    part,
                    notes: line,
                    under,
                } => {
                    let total = gesture.beats();
                    for other in Part::ALL {
                        if other == *part {
                            continue;
                        }
                        if let Some(midi) = under[other as usize] {
                            notes.push(Note {
                                part: other,
                                start_beat: at,
                                beats: total,
                                midi,
                            });
                        }
                    }
                    let mut solo_at = at;
                    for (midi, beats) in line {
                        notes.push(Note {
                            part: *part,
                            start_beat: solo_at,
                            beats: *beats,
                            midi: *midi,
                        });
                        solo_at += beats;
                    }
                }
                Gesture::Rest { .. } => {}
            }
            at += gesture.beats();
        }

        notes.sort_by(|a, b| {
            a.part.cmp(&b.part).then(
                a.start_beat
                    .partial_cmp(&b.start_beat)
                    .expect("beats are finite"),
            )
        });
        let mut tied: Vec<Note> = Vec::new();
        for note in notes {
            match tied.last_mut() {
                Some(previous)
                    if previous.part == note.part
                        && previous.midi == note.midi
                        // Contiguous, within a rounding error of a beat boundary.
                        && (note.start_beat - previous.end_beat()).abs() < 1e-6 =>
                {
                    previous.beats += note.beats;
                }
                _ => tied.push(note),
            }
        }
        tied
    }

    /// One part's notes, in order.
    pub fn line(&self, part: Part) -> Vec<Note> {
        self.notes()
            .into_iter()
            .filter(|n| n.part == part)
            .collect()
    }

    /// The mean MIDI pitch of one part, weighted by how long each note is held — what "where
    /// does this part sit" has to mean when the notes are not equal length.
    pub fn mean_pitch(&self, part: Part) -> f64 {
        let line = self.line(part);
        let beats: f64 = line.iter().map(|n| n.beats).sum();
        if beats <= 0.0 {
            return 0.0;
        }
        line.iter()
            .map(|n| f64::from(n.midi) * n.beats)
            .sum::<f64>()
            / beats
    }
}

/// One duck in the ensemble.
#[derive(Debug, Clone)]
pub struct Singer {
    pub personality: Personality,
    pub part: Part,
    /// Pitch offset, cents. A few cents of spread is what makes four voices a choir rather
    /// than one thick organ stop. Derived from the seed, so a group sounds like that group.
    pub detune_cents: f64,
    /// How early or late this duck takes each note, seconds. Same reasoning as the detune, and
    /// the reason the eventual clock sync needs ±20 ms rather than ±1 ms.
    pub onset_offset_s: f64,
}

/// Cast an ensemble: lowest duck sings the lowest part.
///
/// Deterministic from the personalities alone, and that is worth more than it looks — it means
/// real ducks can agree on who sings what *without negotiating*, from a list of seeds they
/// already exchange. No leader has to assign parts.
pub fn cast(personalities: &[Personality]) -> Vec<Singer> {
    let parts = Part::ensemble(personalities.len());
    let mut order: Vec<usize> = (0..personalities.len()).collect();
    // By register, then by seed — the tie-break matters only for two ducks rolled to the same
    // pitch centre, and without it their parts could swap between runs.
    order.sort_by(|&a, &b| {
        personalities[a]
            .pitch_center_hz
            .partial_cmp(&personalities[b].pitch_center_hz)
            .expect("pitch centres are finite")
            .then(personalities[a].seed.cmp(&personalities[b].seed))
    });

    let mut singers: Vec<Option<Singer>> = vec![None; personalities.len()];
    for (rank, &index) in order.iter().enumerate() {
        let personality = personalities[index];
        let mut rng = personality.variant_rng("chorale-seat", rank as u32);
        singers[index] = Some(Singer {
            part: parts[rank.min(parts.len().saturating_sub(1))],
            detune_cents: rng.uniform(-5.0, 5.0),
            onset_offset_s: rng.uniform(-0.015, 0.015),
            personality,
        });
    }
    singers.into_iter().flatten().collect()
}

/// Equal-temperament frequency of a MIDI note, from [`A4_HZ`].
pub fn midi_hz(midi: f64) -> f64 {
    A4_HZ * 2.0f64.powf((midi - 69.0) / 12.0)
}

// **Why there is no automatic transposition.**
//
// The obvious idea, and it was tried: shift the whole piece so each duck's part sits near its
// own `pitch_center_hz`, since a duck rolled high should get high notes. Every ensemble came
// out pinned at the maximum shift, dragging the piece up and thinning the bass, and the
// heuristic turns out to be measuring the wrong thing. A duck's pitch centre is where its
// *quacks* sit; the synth's harmonic weights are relative to f0, so a duck singing well below
// its centre sounds like itself an octave down rather than like a duck out of its depth.
// Register already does its job in `cast` — the low duck gets the low part — and a second
// mechanism chasing the same goal only fought the voicings, which were written for what the
// hardware can reproduce. `Options::transpose` stays as a knob for taste.

/// What to render, and how.
#[derive(Debug, Clone, Copy)]
pub struct Options {
    /// Transposition in semitones. See the note above on why this is not chosen for you.
    pub transpose: i32,
    /// Wet mix of the room, 0..1.
    ///
    /// Real ducks get this for free by being four objects in a room, and the dry sum of four
    /// synths is harsher than what the hardware will actually produce — so a preview with no
    /// room in it misrepresents the arrangement in the pessimistic direction.
    pub room: f64,
    /// Peak of the finished mix, dBFS.
    pub peak_dbfs: f64,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            transpose: 0,
            room: 0.25,
            peak_dbfs: -3.0,
        }
    }
}

/// Render the ensemble to one mono buffer at [`SR`].
///
/// Each singer is rendered whole and then summed, rather than the parts being interleaved
/// block by block, because that is what will actually happen: on real hardware these are four
/// separate machines, and anything that needed them to share a buffer would be a preview of
/// something we cannot build.
pub fn render(score: &Score, singers: &[Singer], options: &Options) -> Vec<f32> {
    let shift = options.transpose;
    // Tail enough for the last release and the room to decay.
    let total = ((score.duration_s() + 2.5) * f64::from(SR)) as usize;
    let mut mix = vec![0.0f32; total];

    for singer in singers {
        for (sample, value) in sing(score, singer, shift, total).iter().enumerate() {
            mix[sample] += value;
        }
    }
    // Not 1/n: uncorrelated voices sum closer to sqrt(n), and dividing by n would make every
    // added duck quieter rather than fuller.
    let gain = 1.0 / (singers.len().max(1) as f32).sqrt();
    for sample in &mut mix {
        *sample *= gain;
    }

    if options.room > 0.0 {
        reverb(&mut mix, options.room as f32);
    }
    crate::synth::normalise(&mut mix, options.peak_dbfs);
    mix
}

/// One duck's part, as it would come out of that duck's speaker.
fn sing(score: &Score, singer: &Singer, shift: i32, total: usize) -> Vec<f32> {
    /// Control-rate block. Short enough that a note change lands within a couple of
    /// milliseconds; the stream's own slews do the shaping from there.
    const BLOCK: usize = 128;

    let mut stream = Stream::choral(&singer.personality, singer.part as u32);
    let detune = 2.0f64.powf(singer.detune_cents / 1200.0);
    let beat_s = score.beat_s();

    // This part's line, in seconds. Ties are already merged by `Score::notes`, so a common
    // tone held across a chord change arrives here as one note and is not re-attacked.
    let notes: Vec<(f64, f64, u8)> = score
        .line(singer.part)
        .into_iter()
        .map(|note| {
            (
                note.start_beat * beat_s,
                (note.start_beat + note.beats) * beat_s,
                note.midi,
            )
        })
        .collect();

    // Where in its own part this note sits, for the mouth-and-timbre shape below.
    let (low, high) = notes
        .iter()
        .fold((127.0f64, 0.0f64), |(lo, hi), &(_, _, m)| {
            (lo.min(f64::from(m)), hi.max(f64::from(m)))
        });

    let mut out = vec![0.0f32; total];
    let mut note_index = 0usize;
    // Held across rests: a silent voice keeps its last pitch so the next entry does not glide
    // up from nowhere. The level is what makes it silent.
    let mut last_hz = notes.first().map_or(220.0, |&(_, _, midi)| {
        midi_hz(f64::from(midi) + f64::from(shift))
    });

    for start in (0..total).step_by(BLOCK) {
        let now = start as f64 / f64::from(SR) - singer.onset_offset_s;
        while note_index < notes.len() && now >= notes[note_index].1 {
            note_index += 1;
        }
        let (level, open) = match notes.get(note_index) {
            Some(&(begin, end, midi)) if now >= begin => {
                last_hz = midi_hz(f64::from(midi) + f64::from(shift)) * detune;
                // A breath before the next entry: the note releases a little early, so a
                // change of chord re-articulates instead of sliding into the next one.
                let sustain = (end - begin) * 0.92;
                // Higher in your own part is a more open mouth and a brighter tone, which is
                // what a singer reaching up actually does — and on a duck it is the same
                // number that moves the beak.
                let reach = ((f64::from(midi) - low) / (high - low).max(1.0)).clamp(0.0, 1.0);
                let level = if now - begin < sustain { 1.0 } else { 0.0 };
                (level, 0.30 + 0.45 * reach)
            }
            // Before the first entry, or after the last release.
            _ => (0.0, 0.30),
        };
        stream.set(last_hz, level, open);
        let end = (start + BLOCK).min(total);
        stream.block(&mut out[start..end]);
    }
    out
}

/// A small room, so a preview is not drier than the hardware.
///
/// Schroeder's arrangement — four parallel combs into two series allpasses — which is the
/// cheapest thing that sounds like a space rather than like an echo. The delays are the
/// classic mutually-prime lengths, scaled from the 25 kHz they were published at to [`SR`] so
/// the room keeps its *size* rather than its sample counts.
fn reverb(buffer: &mut [f32], wet: f32) {
    const COMB_MS: [f64; 4] = [29.7, 37.1, 41.1, 43.7];
    const COMB_FEEDBACK: [f32; 4] = [0.78, 0.76, 0.74, 0.72];
    const ALLPASS_MS: [f64; 2] = [5.0, 1.7];
    const ALLPASS_GAIN: f32 = 0.7;

    let samples = |ms: f64| ((ms / 1000.0) * f64::from(SR)) as usize;
    let dry = buffer.to_vec();
    let mut wet_sum = vec![0.0f32; buffer.len()];

    for (delay_ms, feedback) in COMB_MS.iter().zip(COMB_FEEDBACK) {
        let delay = samples(*delay_ms).max(1);
        let mut line = vec![0.0f32; delay];
        for (i, &input) in dry.iter().enumerate() {
            let slot = i % delay;
            let delayed = line[slot];
            line[slot] = input + delayed * feedback;
            wet_sum[i] += delayed * 0.25;
        }
    }
    for delay_ms in ALLPASS_MS {
        let delay = samples(delay_ms).max(1);
        let mut line = vec![0.0f32; delay];
        for (i, sample) in wet_sum.iter_mut().enumerate() {
            let slot = i % delay;
            let delayed = line[slot];
            let input = *sample;
            line[slot] = input + delayed * ALLPASS_GAIN;
            *sample = delayed - input * ALLPASS_GAIN;
        }
    }
    let wet = wet.clamp(0.0, 1.0);
    for (out, w) in buffer.iter_mut().zip(&wet_sum) {
        *out = *out * (1.0 - 0.5 * wet) + w * wet;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cast_of(seeds: &[u32]) -> Vec<Singer> {
        let personalities: Vec<Personality> =
            seeds.iter().copied().map(Personality::from_seed).collect();
        cast(&personalities)
    }

    /// The lowest duck sings bass, and nobody has to be told — real ducks agree on the casting
    /// from the seeds they already know, with no leader and no negotiation.
    #[test]
    fn the_lowest_duck_sings_bass() {
        for seeds in [
            vec![100u32, 101, 102, 103],
            vec![7, 42, 313, 9001],
            vec![0, 1, 2, 3],
        ] {
            let singers = cast_of(&seeds);
            let mut by_part = singers.clone();
            by_part.sort_by_key(|s| s.part);
            for pair in by_part.windows(2) {
                assert!(
                    pair[0].personality.pitch_center_hz <= pair[1].personality.pitch_center_hz,
                    "{:?} sings under {:?} but is higher",
                    pair[0].part,
                    pair[1].part
                );
            }
            // And the casting is a function of the seeds alone, so two ducks reach it
            // independently.
            let again = cast_of(&seeds);
            let parts: Vec<Part> = singers.iter().map(|s| s.part).collect();
            let parts_again: Vec<Part> = again.iter().map(|s| s.part).collect();
            assert_eq!(parts, parts_again);
        }
    }

    /// The order the ducks are listed in must not change who sings what — on real hardware the
    /// list arrives in whatever order discovery produced.
    #[test]
    fn casting_does_not_depend_on_the_order_the_ducks_arrive_in() {
        let forward = cast_of(&[100, 101, 102, 103]);
        let backward = cast_of(&[103, 102, 101, 100]);
        for singer in &forward {
            let same = backward
                .iter()
                .find(|s| s.personality.seed == singer.personality.seed)
                .expect("same ducks");
            assert_eq!(singer.part, same.part, "seed {}", singer.personality.seed);
            assert_eq!(singer.detune_cents, same.detune_cents);
        }
    }

    /// A duet takes the outer voices. Two ducks muttering a bass and a tenor in the same octave
    /// is not a duet, it is a thick unison.
    #[test]
    fn small_ensembles_take_the_voices_a_chord_needs_most() {
        assert_eq!(Part::ensemble(2), vec![Part::Bass, Part::Soprano]);
        assert_eq!(
            Part::ensemble(3),
            vec![Part::Bass, Part::Alto, Part::Soprano]
        );
        assert_eq!(Part::ensemble(4), Part::ALL.to_vec());
        // More ducks than parts still produces a valid cast rather than a panic — the extras
        // double a part, which is what a real choir does.
        let singers = cast_of(&[1, 2, 3, 4, 5, 6]);
        assert_eq!(singers.len(), 6);
    }

    /// Tuning is absolute and shared. This is the property that makes a chord a chord, and the
    /// one a "keep each duck's own register" design would have broken.
    #[test]
    fn every_duck_sings_the_same_pitch_for_the_same_note() {
        assert!((midi_hz(69.0) - A4_HZ).abs() < 1e-9);
        assert!((midi_hz(81.0) - 2.0 * A4_HZ).abs() < 1e-9);
        assert!((midi_hz(60.0) - 261.6255653).abs() < 1e-6, "middle C");

        // The detune spread is small enough to be a chorus and not a wrong note: five cents is
        // a third of the way to the smallest interval anyone would call out of tune.
        for singer in cast_of(&[100, 101, 102, 103]) {
            assert!(
                singer.detune_cents.abs() <= 5.0,
                "{} cents is audible as flat",
                singer.detune_cents
            );
            assert!(singer.onset_offset_s.abs() <= 0.015);
        }
    }

    /// Common tones across a chord change are tied, not re-attacked — the audible difference
    /// between a chorale and a list of chords, and the reason `notes()` merges rather than
    /// emitting one note per gesture.
    #[test]
    fn common_tones_come_out_as_one_note() {
        let score = Score::wistful();
        let alto = score.line(Part::Alto);
        // The alto's C4 opens the piece and is held through the tread that follows: the
        // opening build plus four chords are one note, not five.
        let first = alto[0];
        assert_eq!(first.midi, 60);
        assert!(
            first.beats > 12.0,
            "the alto's opening C4 should be one long tie, got {} beats",
            first.beats
        );
        // Nothing anywhere is a zero-length or backwards note.
        for note in score.notes() {
            assert!(note.beats > 0.0, "{note:?}");
            assert!(note.start_beat >= 0.0, "{note:?}");
        }
        // And a part's notes never overlap themselves — one voice, one note at a time.
        for part in Part::ALL {
            for pair in score.line(part).windows(2) {
                assert!(
                    pair[1].start_beat + 1e-9 >= pair[0].start_beat + pair[0].beats,
                    "{part:?} overlaps itself: {:?} then {:?}",
                    pair[0],
                    pair[1]
                );
            }
        }
    }

    /// A `Build` assembles the chord: voices enter apart and end together. This is the gesture
    /// a keyboard cannot make, and the one that will be most forgiving of imperfect sync
    /// between real ducks.
    #[test]
    fn a_build_staggers_the_entries_and_lands_them_together() {
        let score = Score {
            name: "test",
            bpm: 60.0,
            gestures: vec![Gesture::Build {
                voicing: chord(48, 55, 60, 64),
                beats: 6.0,
                stagger: 0.75,
                from_top: false,
            }],
        };
        let notes = score.notes();
        assert_eq!(notes.len(), 4, "one note per voice");
        for part in Part::ALL {
            let note = notes.iter().find(|n| n.part == part).expect("sings");
            // Entries are ordered low to high...
            assert!(
                (note.start_beat - part as usize as f64 * 0.75).abs() < 1e-9,
                "{part:?} entered at {}",
                note.start_beat
            );
            // ...and everyone finishes at the end of the gesture.
            assert!((note.end_beat() - 6.0).abs() < 1e-9, "{part:?} {note:?}");
        }

        // From the top, the soprano is first in and the bass last.
        let flipped = Score {
            gestures: vec![Gesture::Build {
                voicing: chord(48, 55, 60, 64),
                beats: 6.0,
                stagger: 0.75,
                from_top: true,
            }],
            ..score.clone()
        };
        let soprano = flipped.line(Part::Soprano)[0];
        let bass = flipped.line(Part::Bass)[0];
        assert!(soprano.start_beat < bass.start_beat, "{soprano:?} {bass:?}");

        // A stagger too long for the gesture drops the late voices rather than emitting a
        // negative-length note.
        let crowded = Score {
            gestures: vec![Gesture::Build {
                voicing: chord(48, 55, 60, 64),
                beats: 1.0,
                stagger: 0.75,
                from_top: false,
            }],
            ..score
        };
        let notes = crowded.notes();
        assert_eq!(notes.len(), 2, "only two voices fit: {notes:?}");
        assert!(notes.iter().all(|n| n.beats > 0.0));
    }

    /// A solo is one voice moving over a held chord — and the voices left out of `under` are
    /// genuinely silent for it, which is what makes an unaccompanied entry possible.
    #[test]
    fn a_solo_moves_over_what_is_held_under_it() {
        let score = Score {
            name: "test",
            bpm: 60.0,
            gestures: vec![Gesture::Solo {
                part: Part::Soprano,
                notes: vec![(64, 1.0), (65, 1.0), (67, 2.0)],
                under: [Some(48), None, Some(60), None],
            }],
        };
        assert_eq!(
            score.gestures[0].beats(),
            4.0,
            "the solo line sets the length"
        );
        assert_eq!(score.line(Part::Soprano).len(), 3, "the soloist moves");
        assert_eq!(score.line(Part::Tenor).len(), 0, "tacet under this solo");
        let alto = score.line(Part::Alto);
        assert_eq!(alto.len(), 1, "the accompaniment is one held note");
        assert_eq!(alto[0].beats, 4.0);
        // A soloist's own pitch is theirs, not `under`'s.
        assert_eq!(score.line(Part::Bass)[0].midi, 48);
        assert_eq!(score.line(Part::Soprano)[0].midi, 64);
    }

    /// A rest breaks the tie. Without that, a breath written before a chord the voices were
    /// already holding would be silently swallowed by the tie merge.
    #[test]
    fn a_rest_is_a_real_gap() {
        let score = Score {
            name: "test",
            bpm: 60.0,
            gestures: vec![
                Gesture::Chord {
                    voicing: chord(48, 55, 60, 64),
                    beats: 2.0,
                },
                Gesture::Rest { beats: 1.0 },
                Gesture::Chord {
                    voicing: chord(48, 55, 60, 64),
                    beats: 2.0,
                },
            ],
        };
        let bass = score.line(Part::Bass);
        assert_eq!(
            bass.len(),
            2,
            "the same note either side of a breath is two notes"
        );
        assert_eq!(bass[0].end_beat(), 2.0);
        assert_eq!(bass[1].start_beat, 3.0);
        assert_eq!(score.duration_s(), 5.0, "the rest occupies time");

        // Whereas without the rest, it is one tied note.
        let tied = Score {
            gestures: vec![
                Gesture::Chord {
                    voicing: chord(48, 55, 60, 64),
                    beats: 2.0,
                },
                Gesture::Chord {
                    voicing: chord(48, 55, 60, 64),
                    beats: 2.0,
                },
            ],
            ..score
        };
        assert_eq!(tied.line(Part::Bass).len(), 1);
        assert_eq!(tied.line(Part::Bass)[0].beats, 4.0);
    }

    /// The shipped piece uses all of it — a chord assembling, a solo, and a breath. If someone
    /// flattens it back to block chords this is what notices.
    #[test]
    fn the_default_piece_is_more_than_block_chords() {
        let score = Score::wistful();
        let has = |f: fn(&Gesture) -> bool| score.gestures.iter().any(f);
        assert!(
            has(|g| matches!(g, Gesture::Build { .. })),
            "no chord assembles"
        );
        assert!(
            has(|g| matches!(g, Gesture::Solo { .. })),
            "nobody ever sings alone"
        );
        assert!(
            has(|g| matches!(g, Gesture::Rest { .. })),
            "nobody ever breathes"
        );
        // And the whole thing is a sensible length for a demo.
        assert!(
            (30.0..90.0).contains(&score.duration_s()),
            "{}s",
            score.duration_s()
        );
    }

    /// Every note has to be singable by a duck and audible on its speaker: a chorale whose bass
    /// is below what the hardware reproduces is a trio. Voices must not cross either — checked
    /// on the written voicings, where simultaneity is unambiguous.
    #[test]
    fn the_score_stays_inside_ranges_a_duck_can_sing() {
        let score = Score::wistful();
        // Bass A2..A3, tenor E3..E4, alto A3..A4, soprano D4..D5.
        let bounds = [(45u8, 57u8), (52, 64), (57, 69), (62, 74)];
        for note in score.notes() {
            let (low, high) = bounds[note.part as usize];
            assert!(
                (low..=high).contains(&note.midi),
                "{:?} sings {}, outside {low}..={high}",
                note.part,
                note.midi
            );
        }
        for gesture in &score.gestures {
            let voicing = match gesture {
                Gesture::Chord { voicing, .. } | Gesture::Build { voicing, .. } => *voicing,
                // A solo's `under` is checked with its soloist's line excluded, since the two
                // are not a chord in the voice-leading sense.
                Gesture::Solo { .. } | Gesture::Rest { .. } => continue,
            };
            let sung: Vec<u8> = voicing.iter().flatten().copied().collect();
            assert!(
                sung.windows(2).all(|w| w[0] <= w[1]),
                "voices cross in {sung:?}"
            );
        }
    }

    /// A transposition is a property of the performance, not of a singer: one shift moves
    /// everyone, so the ensemble stays in one key. The default is no shift at all — see the
    /// note in this module on the heuristic that was tried and removed.
    #[test]
    fn a_transposition_moves_the_whole_ensemble_or_nobody() {
        assert_eq!(Options::default().transpose, 0);
        let score = Score::wistful();
        let singers = cast_of(&[100, 101, 102, 103]);
        let plain = render(&score, &singers, &Options::default());
        let up = render(
            &score,
            &singers,
            &Options {
                transpose: 4,
                room: 0.0,
                ..Options::default()
            },
        );
        assert_eq!(plain.len(), up.len(), "a shift is not a different piece");
        assert!(up.iter().all(|v| v.is_finite()));
    }

    /// The whole ensemble must render to finite, audible, non-clipping audio for any number of
    /// ducks and any seeds — this is the thing a laptop preview exists to produce.
    #[test]
    fn an_ensemble_of_any_size_renders_sane_audio() {
        let score = Score::wistful();
        for count in 2..=4 {
            let seeds: Vec<u32> = (0..count).map(|i| 100 + i as u32).collect();
            let singers = cast_of(&seeds);
            let mix = render(&score, &singers, &Options::default());
            assert!(
                mix.len() as f64 / f64::from(SR) > score.duration_s(),
                "the tail must outlast the last note"
            );
            assert!(
                mix.iter().all(|v| v.is_finite()),
                "{count} ducks: not finite"
            );
            let peak = mix.iter().fold(0.0f32, |m, &v| m.max(v.abs()));
            assert!((0.5..=1.0).contains(&peak), "{count} ducks: peak {peak}");
            // And it is not silence in the middle of the piece.
            let middle = mix.len() / 2;
            let rms: f32 = mix[middle..middle + 4800]
                .iter()
                .map(|v| v * v)
                .sum::<f32>();
            assert!(rms > 1e-3, "{count} ducks: silent mid-piece");
        }
    }

    /// A dry render is still a render: the room is a preview convenience, not load-bearing, and
    /// turning it off must not change the level or blow up.
    #[test]
    fn the_room_is_optional() {
        let score = Score::wistful();
        let singers = cast_of(&[100, 101, 102, 103]);
        let dry = render(
            &score,
            &singers,
            &Options {
                room: 0.0,
                ..Options::default()
            },
        );
        assert!(dry.iter().all(|v| v.is_finite()));
        let peak = dry.iter().fold(0.0f32, |m, &v| m.max(v.abs()));
        assert!((0.5..=1.0).contains(&peak), "dry peak {peak}");
    }
}
