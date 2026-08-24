//! Telling a hand from a wall in an 8×8 depth frame.
//!
//! The ToF theremin needs one number — how far away the hand is — and the whole difficulty
//! is that the sensor does not report hands, it reports distances. A duck that walks up to
//! a wall and stops sees a frame that is near, filled, and stable, which is also what a
//! palm held in front of its beak looks like. Every cheap test for "is this a hand" fails
//! on one of those two:
//!
//!   - **"The near return is small, with free space around it."** The field of view is only
//!     `0.828 × d` wide, so at 25 cm it spans 20.7 cm and a hand really does leave the
//!     border zones looking past it. Below ~15 cm it does not, and under
//!     [`crate::tof::Reprojector::MIN_RANGE_M`] the returns are crosstalk anyway — so this
//!     works, but only over part of the band we want to play.
//!   - **"The duck is standing still, so anything that moves is a hand."** A standing duck
//!     is never still: the policy sways, and a wall 30 cm away swings a few centimetres in
//!     range with it. Worse, a theremin note should *hold* when the hand holds, so
//!     requiring motion breaks the instrument.
//!
//! So neither is the test. The test is a **background**, captured when the theremin arms:
//! whatever is in front of the duck at that moment — wall, table, sofa, nothing — becomes
//! the reference, per zone, and from then on only returns *nearer than the background by a
//! margin* are a hand. The duck that stopped in front of a wall has the wall as its zero,
//! at whatever distance it happens to be, and a hand between duck and wall is by
//! definition nearer. The margin ([`Config::margin_m`]) is set above the trunk's sway,
//! which is the real reason "the robot is not walking" is not the same as "the frame is not
//! moving".
//!
//! The one case a background cannot see is a hand that was already there when it was
//! captured. So arming runs the hand test against its own candidate background, with
//! nothing behind it: a background that already looks like a hand is refused
//! ([`Refusal::SomethingInTheWay`]) instead of being frozen in as the zero. The arming
//! check is the play check, which is why there is only one of them to be wrong.
//!
//! Which leaves one thing the play check cannot get right on its own: a wall inside the
//! playable band *is* one enormous near blob, and refusing to arm in front of a wall is
//! refusing the exact case this module was written for. So the candidate is fitted to a
//! plane, and a plane is believed as background when it is either beyond the band or
//! covering nearly the whole frame — which every real wall does, the field of view being
//! only `0.828 × d` across. A flat palm is planar too and fails both halves: it is near,
//! and it is partial.
//!
//! The fit is cheap and exact rather than iterative. For a plane at distance `d` with unit
//! normal `n`, every beam gives `1/r = beam · (n/d)`, which is *linear* in `n/d` — so a
//! 3×3 least squares over inverse ranges recovers the plane, and its residual (carried back
//! into metres) says how far off one the returns sit. That number is also the log line that
//! settles a field report about a theremin that would not arm: "background: plane at
//! 0.42 m, 0.9 cm rms", or "background: cluttered, 11 cm rms".
//!
//! This lives in `kinematics` for the reason [`crate::tof`] does: it is arithmetic over a
//! frame, and its callers must not have to link the vendored ST driver to get at it.

use crate::tof::{COLS, ROWS, Zone};

const N_ZONES: usize = ROWS * COLS;

/// What counts as a hand.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Config {
    /// Nearest playable range, metres. Above the band where a hand fills the whole field of
    /// view and the border test stops meaning anything (~15 cm), and well above the
    /// crosstalk floor.
    pub near_m: f64,
    /// Farthest playable range, metres. The sensor's usable reach at 15 Hz is around a
    /// metre; the top of the playable band is deliberately shorter than that, because the
    /// returns near the limit are the noisy ones and they would be the *low* notes.
    pub far_m: f64,
    /// How much nearer than the background a zone must read to count as foreground. Above
    /// the trunk sway of a standing policy — the point of the whole margin.
    pub margin_m: f64,
    /// How many foreground zones make a hand. Two is noise; a hand at 60 cm is ~4 zones
    /// across the diagonal even before the fingers.
    pub min_zones: usize,
    /// A foreground filling more than this fraction of the usable frame is not a hand — it
    /// is a wall that arrived after arming, i.e. the duck walked into something.
    pub max_fill: f64,
    /// Time constant for the background's drift, seconds. Long: the room may be
    /// rearranged, a hand may not become the floor.
    pub background_tau_s: f64,
    /// How many frames an arming window must hold before it can be reduced. At 15 Hz this
    /// is a third of a second — long enough for the per-zone median to step over the
    /// dropouts that are the reason it is a median.
    pub min_arming_frames: usize,
    /// Fraction of the frame a *plane* must cover to be believed as a wall rather than a
    /// palm. A wall inside the playable band always fills the whole field of view — it is
    /// only 0.828 × d across, smaller than any wall — so anything planar, near, and
    /// partial is a hand however flat it is.
    pub wall_fill: f64,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            near_m: 0.15,
            far_m: 0.60,
            margin_m: 0.08,
            min_zones: 3,
            max_fill: 0.75,
            background_tau_s: 120.0,
            min_arming_frames: 5,
            wall_fill: 0.9,
        }
    }
}

/// A hand, as the theremin needs it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Hand {
    /// Robust distance to it, metres — a low percentile of the foreground rather than the
    /// single nearest zone, which on this sensor is regularly a flier.
    pub range_m: f64,
    /// Where that sits in the playable band: 0 at [`Config::far_m`], 1 at
    /// [`Config::near_m`]. Closer is *higher*, which is the direction the pitch and the
    /// mouth both move.
    pub closeness: f64,
    /// How many zones the hand covers. The instrument's dynamics: a flat palm is louder
    /// than a fingertip.
    pub zones: usize,
    /// Its centroid in the trunk frame, metres — for a gaze that follows the hand, and for
    /// nothing the theremin itself needs.
    pub centroid: [f64; 3],
}

/// Why a background was not accepted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Refusal {
    /// The sensor produced almost nothing over the whole arming window: no background, and
    /// nothing to play against either.
    NoReturns,
    /// The candidate background already looks like a hand. Almost always literally that —
    /// arming happened with a hand in front of the beak — and freezing it in would make
    /// that hand the silent zero.
    SomethingInTheWay,
}

impl Refusal {
    /// What to tell whoever asked for the theremin.
    pub fn as_str(&self) -> &'static str {
        match self {
            Refusal::NoReturns => "the depth sensor is not seeing anything to play against",
            Refusal::SomethingInTheWay => {
                "something is right in front of the beak — move it away and arm again"
            }
        }
    }
}

/// The reference frame the hand test measures against, and what it turned out to be.
#[derive(Debug, Clone, PartialEq)]
pub struct Background {
    /// Per-zone reference range, metres. `None` where the sensor saw nothing — which for
    /// the hand test means "infinitely far", so anything appearing there is foreground.
    pub range_m: [Option<f64>; N_ZONES],
    /// What the background is, geometrically: the wall exemption in [`Arming::finish`]
    /// turns on it, and it is the log line when a theremin refuses to arm.
    pub shape: Shape,
}

/// A background's geometry, as the plane fit reads it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Shape {
    /// Too few returns to fit anything — mostly open space.
    Open { zones: usize },
    /// A plane: a wall, a table, a door. `rms_m` is how far the returns sit off it.
    Plane { distance_m: f64, rms_m: f64 },
    /// Returns, but not on one plane: furniture, clutter, a corner.
    Cluttered { rms_m: f64 },
}

impl Shape {
    /// Departure from a plane at which a background stops being called one. Generous: this
    /// only picks the word in a log line, and a real wall seen through this sensor's noise
    /// sits around a centimetre.
    const PLANE_RMS_M: f64 = 0.03;
    /// Fewest returns worth fitting three parameters to.
    const MIN_FIT_ZONES: usize = 12;
}

/// Accumulates arming frames into a [`Background`].
///
/// Several frames rather than one, reduced per zone by median: at 15 Hz a zone drops out
/// often enough that a single frame would freeze holes into the reference, and a median is
/// the reduction that ignores a dropout instead of averaging it in.
#[derive(Debug, Default)]
pub struct Arming {
    samples: Vec<[Option<f64>; N_ZONES]>,
}

impl Arming {
    pub fn new() -> Self {
        Self::default()
    }

    /// Take one frame's slant ranges into the window.
    pub fn observe(&mut self, ranges_m: &[Option<f64>; N_ZONES]) {
        self.samples.push(*ranges_m);
    }

    pub fn frames(&self) -> usize {
        self.samples.len()
    }

    /// Enough frames to reduce.
    pub fn ready(&self, config: &Config) -> bool {
        self.samples.len() >= config.min_arming_frames
    }

    /// Reduce the window to a background, or say why it is not one.
    ///
    /// The refusal path is the whole reason arming is a step and not a flag: a theremin
    /// that silently took a hand as its zero would present as "the duck ignores my hand",
    /// which is indistinguishable from a broken sensor from the outside.
    pub fn finish(
        &self,
        config: &Config,
        beams: &[[f64; 3]; N_ZONES],
    ) -> Result<Background, Refusal> {
        let mut range_m: [Option<f64>; N_ZONES] = [None; N_ZONES];
        for (zone, slot) in range_m.iter_mut().enumerate() {
            let mut seen: Vec<f64> = self
                .samples
                .iter()
                .filter_map(|frame| frame[zone])
                .collect();
            // A zone must have been seen in most of the window to be part of the reference;
            // one sighting in ten frames is a flier, and as a background it would mask a
            // real hand appearing there.
            if seen.len() * 2 <= self.samples.len() {
                continue;
            }
            seen.sort_by(|a, b| a.partial_cmp(b).expect("ranges are finite"));
            *slot = Some(seen[seen.len() / 2]);
        }

        if self.samples.is_empty() {
            return Err(Refusal::NoReturns);
        }
        let shape = fit_shape(&range_m, beams);
        let seen = range_m.iter().filter(|r| r.is_some()).count();
        if seen == 0 {
            return Err(Refusal::NoReturns);
        }

        // The arming check *is* the play check: run the hand test on this candidate with
        // nothing behind it, and refuse a background that already answers "hand".
        let nothing = Background {
            range_m: [None; N_ZONES],
            shape,
        };
        let hand_shaped = nothing
            .detect(&range_m, &[Zone::Empty; N_ZONES], config)
            .is_some();

        // Except when the candidate is a wall, which the play check cannot help reading as
        // one enormous hand. A wall inside the playable band fills the frame — the field of
        // view is 0.828 × d across, narrower than any wall — so a plane is believed as
        // background when it is either beyond the band or covering nearly all of it. A flat
        // palm is planar too, and fails both halves: it is near, and it is partial.
        let is_wall = match shape {
            Shape::Plane { distance_m, .. } => {
                distance_m > config.far_m || seen as f64 >= config.wall_fill * N_ZONES as f64
            }
            Shape::Open { .. } | Shape::Cluttered { .. } => false,
        };
        if hand_shaped && !is_wall {
            return Err(Refusal::SomethingInTheWay);
        }
        Ok(Background { range_m, shape })
    }
}

impl Background {
    /// Find the hand in one frame, if there is one.
    ///
    /// `ranges_m` is the frame's 64 slant ranges, `None` where the sensor reported nothing
    /// usable — the same array [`crate::tof::Reprojector::project`] takes, and the depth
    /// comparison is done on these rather than on reprojected points because a slant range
    /// is what the sensor measured: putting it through forward kinematics first would fold
    /// head-pose error into a number whose whole job is to be compared against itself one
    /// frame later.
    ///
    /// `zones` is that reprojection, used only as a validity mask: a beam the reprojector
    /// calls [`Zone::Floor`] hit the ground, and a duck looking slightly down must play the
    /// hand and not the carpet.
    pub fn detect(
        &self,
        ranges_m: &[Option<f64>; N_ZONES],
        zones: &[Zone; N_ZONES],
        config: &Config,
    ) -> Option<Hand> {
        let mut usable = 0usize;
        let mut foreground: Vec<(usize, f64)> = Vec::new();
        for (i, range) in ranges_m.iter().enumerate() {
            // The floor is not an instrument, and a return the reprojector distrusts is not
            // a measurement. Both are excluded before anything is counted.
            if matches!(zones[i], Zone::Floor { .. } | Zone::TooClose) {
                continue;
            }
            usable += 1;
            let Some(r) = *range else { continue };
            if !(config.near_m..=config.far_m).contains(&r) {
                continue;
            }
            // No background in this zone means nothing was ever there: any return is new.
            let reference = self.range_m[i].unwrap_or(f64::INFINITY);
            if r < reference - config.margin_m {
                foreground.push((i, r));
            }
        }

        if foreground.len() < config.min_zones {
            return None;
        }
        // A foreground that fills the frame is not a hand — the duck walked into something,
        // or the room changed behind the reference. Either way the background is stale and
        // playing it would be a held note nobody asked for.
        if usable > 0 && foreground.len() as f64 > config.max_fill * usable as f64 {
            return None;
        }

        let mut ranges: Vec<f64> = foreground.iter().map(|(_, r)| *r).collect();
        ranges.sort_by(|a, b| a.partial_cmp(b).expect("ranges are finite"));
        // A low percentile, not the minimum: single-zone fliers a few centimetres short of
        // the truth are routine on this sensor, and as the pitch input one would be a chirp.
        let range_m = ranges[ranges.len() / 5];

        let mut centroid = [0.0f64; 3];
        let mut counted = 0usize;
        for (i, _) in &foreground {
            if let Zone::Hit { point, .. } = zones[*i] {
                for (c, p) in centroid.iter_mut().zip(point) {
                    *c += p;
                }
                counted += 1;
            }
        }
        if counted > 0 {
            for c in &mut centroid {
                *c /= counted as f64;
            }
        }

        let span = (config.far_m - config.near_m).max(1e-6);
        Some(Hand {
            range_m,
            closeness: ((config.far_m - range_m) / span).clamp(0.0, 1.0),
            zones: foreground.len(),
            centroid,
        })
    }

    /// Let the reference drift toward what the sensor sees, everywhere the hand is not.
    ///
    /// Slow, so the room can be rearranged without re-arming while a hand held still for a
    /// minute never becomes the new zero. Zones the hand is currently in are left alone —
    /// updating those is exactly how a background model eats its own signal.
    pub fn relax(
        &mut self,
        ranges_m: &[Option<f64>; N_ZONES],
        hand: Option<&Hand>,
        config: &Config,
        dt_s: f64,
    ) {
        let alpha = (dt_s / config.background_tau_s.max(1e-3)).clamp(0.0, 1.0);
        for (i, measured) in ranges_m.iter().enumerate() {
            let Some(r) = *measured else { continue };
            // Anything inside the playable band while a hand is out there might *be* the
            // hand; leave the whole band alone rather than trying to name its zones.
            if hand.is_some() && r <= config.far_m {
                continue;
            }
            match &mut self.range_m[i] {
                Some(reference) => *reference += (r - *reference) * alpha,
                // A zone that was empty and now reads something adopts it at once: there is
                // no old value to drift away from, and the alternative is a permanent hole.
                slot @ None => *slot = Some(r),
            }
        }
    }
}

/// Least squares plane over inverse ranges — see the module docs for why that is linear.
fn fit_shape(range_m: &[Option<f64>; N_ZONES], beams: &[[f64; 3]; N_ZONES]) -> Shape {
    let points: Vec<([f64; 3], f64)> = range_m
        .iter()
        .zip(beams)
        .filter_map(|(r, beam)| r.map(|r| (*beam, r)))
        .filter(|(_, r)| *r > 1e-3)
        .collect();
    if points.len() < Shape::MIN_FIT_ZONES {
        return Shape::Open {
            zones: points.len(),
        };
    }

    // Normal equations for `beam · v = 1/r`, v = n/d.
    let mut ata = [[0.0f64; 3]; 3];
    let mut atb = [0.0f64; 3];
    for (beam, r) in &points {
        let inv = 1.0 / r;
        for (row, (a_row, b)) in ata.iter_mut().zip(atb.iter_mut()).enumerate() {
            for (col, a) in a_row.iter_mut().enumerate() {
                *a += beam[row] * beam[col];
            }
            *b += beam[row] * inv;
        }
    }
    let Some(v) = solve3(ata, atb) else {
        return Shape::Cluttered { rms_m: f64::NAN };
    };
    let norm = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if norm < 1e-9 {
        return Shape::Cluttered { rms_m: f64::NAN };
    }
    let distance_m = 1.0 / norm;

    // Residuals back in metres: the inverse-range error at each zone, scaled by that
    // zone's own range squared, which is the derivative of `r` with respect to `1/r`.
    let mut sum_sq = 0.0;
    for (beam, r) in &points {
        let predicted = beam[0] * v[0] + beam[1] * v[1] + beam[2] * v[2];
        let error_m = (1.0 / r - predicted) * r * r;
        sum_sq += error_m * error_m;
    }
    let rms_m = (sum_sq / points.len() as f64).sqrt();
    if rms_m <= Shape::PLANE_RMS_M {
        Shape::Plane { distance_m, rms_m }
    } else {
        Shape::Cluttered { rms_m }
    }
}

/// Gaussian elimination on a 3×3 symmetric system. `None` if it is singular — fewer than
/// three independent beam directions, which the zone-count guard makes unreachable in
/// practice but which must not be a divide by zero if it happens.
fn solve3(mut a: [[f64; 3]; 3], mut b: [f64; 3]) -> Option<[f64; 3]> {
    for col in 0..3 {
        let (pivot_row, pivot) = (col..3)
            .map(|r| (r, a[r][col].abs()))
            .max_by(|x, y| x.1.partial_cmp(&y.1).expect("finite"))?;
        if pivot < 1e-12 {
            return None;
        }
        a.swap(col, pivot_row);
        b.swap(col, pivot_row);
        for row in (col + 1)..3 {
            let factor = a[row][col] / a[col][col];
            let (pivot_row, target) = {
                let (head, tail) = a.split_at_mut(row);
                (head[col], &mut tail[0])
            };
            for (cell, pivot) in target.iter_mut().zip(pivot_row).skip(col) {
                *cell -= factor * pivot;
            }
            b[row] -= factor * b[col];
        }
    }
    let mut x = [0.0f64; 3];
    for row in (0..3).rev() {
        let mut acc = b[row];
        for col in (row + 1)..3 {
            acc -= a[row][col] * x[col];
        }
        x[row] = acc / a[row][row];
    }
    Some(x)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tof::Reprojector;

    /// The beam table the real thing uses, so the plane geometry under test is the
    /// sensor's own and not an idealisation of it.
    fn beams() -> [[f64; 3]; N_ZONES] {
        *Reprojector::alpha().beams()
    }

    /// A wall perpendicular to the sensor axis at `distance`, as slant ranges: every beam
    /// reaches it at `d / (beam · axis)`.
    fn wall(distance: f64) -> [Option<f64>; N_ZONES] {
        let beams = beams();
        let mut out = [None; N_ZONES];
        for (slot, beam) in out.iter_mut().zip(&beams) {
            *slot = Some(distance / beam[0]);
        }
        out
    }

    /// A blob of `size`×`size` zones at `distance`, centred, over `behind`.
    fn blob(behind: &[Option<f64>; N_ZONES], distance: f64, size: usize) -> [Option<f64>; N_ZONES] {
        let mut out = *behind;
        let start = (ROWS - size) / 2;
        for row in start..start + size {
            for col in start..start + size {
                out[row * COLS + col] = Some(distance);
            }
        }
        out
    }

    fn all_hits() -> [Zone; N_ZONES] {
        [Zone::Hit {
            point: [0.3, 0.0, 0.0],
            range: 0.3,
        }; N_ZONES]
    }

    fn armed(frames: &[[Option<f64>; N_ZONES]]) -> Result<Background, Refusal> {
        let config = Config::default();
        let mut arming = Arming::new();
        for frame in frames {
            arming.observe(frame);
        }
        arming.finish(&config, &beams())
    }

    /// The scenario the whole module exists for: the duck walks up to a wall and stops, so
    /// its background *is* the wall — and a hand in front of that wall still reads as a
    /// hand, because it is nearer than the reference wherever the reference happens to be.
    #[test]
    fn a_hand_in_front_of_a_wall_the_duck_is_facing() {
        let config = Config::default();
        for wall_m in [0.30, 0.40, 0.55, 0.80] {
            let background = armed(&[wall(wall_m); 8]).expect("a wall is a background");
            assert!(
                background
                    .detect(&wall(wall_m), &all_hits(), &config)
                    .is_none(),
                "the wall itself must be silent at {wall_m} m"
            );
            // Somewhere in the playable band, in front of wherever the wall is.
            let hand_m = (wall_m - 0.15).clamp(config.near_m, config.far_m);
            let hand = background
                .detect(&blob(&wall(wall_m), hand_m, 3), &all_hits(), &config)
                .unwrap_or_else(|| panic!("a hand at {hand_m} m in front of a {wall_m} m wall"));
            assert!((hand.range_m - hand_m).abs() < 0.02, "{:?}", hand);
            assert!((0.0..=1.0).contains(&hand.closeness), "{:?}", hand);
        }
    }

    /// A wall is a plane, and the fit says so with the distance it is actually at — the log
    /// line that settles "why would it not arm".
    #[test]
    fn a_wall_reads_as_a_plane_at_its_distance() {
        for wall_m in [0.25, 0.5, 0.9] {
            let background = armed(&[wall(wall_m); 8]).expect("armed");
            match background.shape {
                Shape::Plane { distance_m, rms_m } => {
                    assert!(
                        (distance_m - wall_m).abs() < 0.01,
                        "{distance_m} vs {wall_m}"
                    );
                    assert!(rms_m < 0.005, "a synthetic wall is flat, rms {rms_m}");
                }
                other => panic!("a wall must fit a plane, got {other:?}"),
            }
        }
    }

    /// Arming with a hand already in front of the beak is refused rather than frozen in:
    /// that background would make the hand the silent zero, which from outside looks
    /// exactly like a duck ignoring you.
    #[test]
    fn arming_refuses_a_hand_it_would_have_taken_as_the_zero() {
        assert_eq!(
            armed(&[blob(&wall(0.6), 0.25, 3); 8]),
            Err(Refusal::SomethingInTheWay)
        );
        // The same hand-shaped blob against open space, too — there is no wall to hide it.
        assert_eq!(
            armed(&[blob(&[None; N_ZONES], 0.25, 3); 8]),
            Err(Refusal::SomethingInTheWay)
        );
    }

    /// Open space arms fine: the background is "nothing anywhere", so every return that
    /// arrives later is foreground. Only a sensor producing nothing at all is refused.
    #[test]
    fn open_space_arms_and_an_empty_sensor_does_not() {
        let config = Config::default();
        let far = {
            let mut frame = wall(2.0);
            // Beyond the playable band the reprojector would call these hits; what matters
            // is that they are farther than anything the theremin plays.
            frame[0] = None;
            frame
        };
        let background = armed(&[far; 8]).expect("a far wall is open space to a theremin");
        let hand = background
            .detect(&blob(&far, 0.3, 3), &all_hits(), &config)
            .expect("a hand against a far background");
        assert!((hand.range_m - 0.3).abs() < 0.02);

        assert_eq!(armed(&[[None; N_ZONES]; 8]), Err(Refusal::NoReturns));
        assert_eq!(armed(&[]), Err(Refusal::NoReturns));
    }

    /// Closer is higher — the direction the pitch and the mouth both move — and the ends
    /// of the band are 0 and 1 exactly.
    #[test]
    fn closeness_rises_as_the_hand_approaches() {
        let config = Config::default();
        let background = armed(&[wall(2.0); 8]).expect("armed");
        let closeness = |d: f64| {
            background
                .detect(&blob(&wall(2.0), d, 3), &all_hits(), &config)
                .map(|h| h.closeness)
        };
        let near = closeness(config.near_m).expect("the near end plays");
        let far = closeness(config.far_m).expect("the far end plays");
        let middle = closeness(0.5 * (config.near_m + config.far_m)).expect("the middle plays");
        assert!((near - 1.0).abs() < 1e-9, "{near}");
        assert!(far.abs() < 1e-9, "{far}");
        assert!(far < middle && middle < near);
        // Outside the band there is no note at all, rather than a clamped one.
        assert_eq!(closeness(config.far_m + 0.1), None);
        assert_eq!(closeness(config.near_m - 0.04), None);
    }

    /// Walking into a wall after arming must not play a held note: a foreground that fills
    /// the frame is a stale background, not a hand.
    #[test]
    fn a_wall_that_arrives_after_arming_is_not_a_hand() {
        let config = Config::default();
        let background = armed(&[wall(2.0); 8]).expect("armed");
        assert_eq!(
            background.detect(&wall(0.3), &all_hits(), &config),
            None,
            "a whole frame gone near is the duck moving, not a hand"
        );
        // And the same wall with only part of the frame on it *is* playable — that is a
        // hand-sized thing, whatever it is made of.
        assert!(
            background
                .detect(&blob(&wall(2.0), 0.3, 4), &all_hits(), &config)
                .is_some()
        );
    }

    /// The floor is not an instrument. A duck looking down sees the ground in the lower
    /// rows; those zones must not reach the pitch, however near they read.
    #[test]
    fn floor_returns_are_never_the_hand() {
        let config = Config::default();
        let background = armed(&[wall(2.0); 8]).expect("armed");
        let mut zones = all_hits();
        let mut frame = wall(2.0);
        // The bottom three rows are floor at a range that would otherwise dominate.
        for row in 5..ROWS {
            for col in 0..COLS {
                zones[row * COLS + col] = Zone::Floor {
                    point: [0.2, 0.0, -0.1],
                };
                frame[row * COLS + col] = Some(0.20);
            }
        }
        assert_eq!(background.detect(&frame, &zones, &config), None);
    }

    /// Sway must not play: a background dropping by less than the margin is the trunk
    /// breathing, and crossing it is a hand.
    #[test]
    fn the_margin_absorbs_the_trunks_sway() {
        let config = Config::default();
        let background = armed(&[wall(0.45); 8]).expect("armed");
        for sway in [0.0, 0.02, 0.05, config.margin_m - 0.005] {
            let swayed: [Option<f64>; N_ZONES] =
                std::array::from_fn(|i| background.range_m[i].map(|r| r - sway));
            assert_eq!(
                background.detect(&swayed, &all_hits(), &config),
                None,
                "sway of {sway} m must be silent"
            );
        }
        let hand = blob(&wall(0.45), 0.45 - config.margin_m - 0.03, 3);
        assert!(background.detect(&hand, &all_hits(), &config).is_some());
    }

    /// A zone the sensor drops in most arming frames must not enter the reference: as a
    /// background it would mask a real hand appearing in exactly that zone.
    #[test]
    fn a_flickering_zone_stays_out_of_the_reference() {
        let solid = wall(0.5);
        let mut frames = vec![solid; 8];
        // Zone 0 is seen once in eight; zone 1 in seven.
        for (i, frame) in frames.iter_mut().enumerate() {
            if i > 0 {
                frame[0] = None;
            }
            if i == 0 {
                frame[1] = None;
            }
        }
        let background = armed(&frames).expect("armed");
        assert_eq!(
            background.range_m[0], None,
            "a one-in-eight zone is a flier"
        );
        assert!(background.range_m[1].is_some(), "seven in eight is real");
    }

    /// The background drifts toward a rearranged room, and never toward the hand — a hand
    /// held still for a minute must not become the new zero.
    #[test]
    fn the_reference_drifts_to_the_room_but_not_to_the_hand() {
        let config = Config::default();
        let mut background = armed(&[wall(0.8); 8]).expect("armed");
        let held = blob(&wall(0.8), 0.30, 3);
        let centre = (ROWS / 2) * COLS + COLS / 2;
        let before = background.range_m[centre].expect("the wall is in every zone");

        // Two minutes of a hand held perfectly still, at the frame rate.
        for _ in 0..(15 * 120) {
            let hand = background.detect(&held, &all_hits(), &config);
            assert!(hand.is_some(), "a held hand must keep playing");
            background.relax(&held, hand.as_ref(), &config, 1.0 / 15.0);
        }
        assert!(
            (background.range_m[centre].expect("still there") - before).abs() < 0.01,
            "the hand must not have become the background"
        );

        // The wall itself moving back (someone opened a door) is adopted.
        let moved = wall(1.2);
        for _ in 0..(15 * 600) {
            background.relax(&moved, None, &config, 1.0 / 15.0);
        }
        let after = background.range_m[centre].expect("still there");
        let expected = moved[centre].expect("the wall is in every zone");
        assert!((after - expected).abs() < 0.05, "{after} vs {expected}");
    }

    /// Clutter is called clutter, not a plane — the other half of the diagnostic line.
    #[test]
    fn clutter_does_not_fit_a_plane() {
        // Beyond the playable band, so the theremin has no opinion about it and only the
        // description is under test.
        let mut frame = wall(1.4);
        for (i, slot) in frame.iter_mut().enumerate() {
            if let Some(r) = slot.as_mut()
                && i % 3 == 0
            {
                *r -= 0.25;
            }
        }
        let background = armed(&[frame; 8]).expect("clutter is still a background");
        assert!(
            matches!(background.shape, Shape::Cluttered { .. }),
            "{:?}",
            background.shape
        );
    }
}
