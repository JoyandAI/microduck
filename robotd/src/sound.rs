//! The robot's voice, at play time: pick a wav from the bank, feed it to `aplay`.
//!
//! Ported from the prototype's `play_voice` / `start_wheee`. The codec PCM is exclusive and
//! single-client, which two properties fall out of:
//!
//!  - **One playing child, and a new sound kills it.** That is what lets someone spam the
//!    chirp trigger cleanly — each press cuts the previous call off — and why everything
//!    that plays goes through this one struct, owned by the control loop.
//!  - **The wheee ride streams into a single `aplay`**: start → loop (repeating while held)
//!    → end, written by a paced thread so the pipe never queues more than ~250 ms — else
//!    the release would land that late. The ride has *two* exits, and they are not the
//!    same: a client that says "released" cuts it (kill the child, the writer exits on the
//!    broken pipe), while a hold that merely went stale lands it — the writer is let out of
//!    its loop and writes the end segment into a pipe that is still open. Only the second
//!    one ever plays `wheee_end_*`, which is why [`Ride`] is a state and not a bool.
//!
//! Playing is spawning: nothing here blocks the 50 Hz tick except the deliberately blocking
//! goodbye peck right before power-off, when there is no tick left to miss — and that one is
//! bounded, because a wedged PCM must not be able to hold up the power-off.

use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use crate::intents::WheeeHold;

/// How long the blocking goodbye peck may hold up the power-off. The longest bank sound is
/// well under a second; this is a ceiling on a wedged PCM, not a playback budget.
const BLOCKING_PLAY_MAX: Duration = Duration::from_millis(1500);

/// What the wheee ride is doing. `Landing` exists because the end segment takes real time
/// to play and the trigger keeps asking during it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Ride {
    /// No ride. The PCM is free for one-shots.
    Off,
    /// The writer thread is streaming start → loop into an open pipe.
    Riding,
    /// The writer is on the end segment, or `aplay` is draining it. Reaped when the child
    /// exits; a fresh press supersedes it.
    Landing,
}

/// The player. Constructed once by the control loop; `None`-like when the bank is missing —
/// every play degrades to a debug line, so a robot without a generated bank (or a codec)
/// walks fine and stays quiet.
pub struct Sound {
    bank: PathBuf,
    device: String,
    child: Option<Child>,
    /// The wheee rider loops while this is true. Shared with the writer thread; clearing
    /// it *without* killing the child is what makes the end segment play.
    wheee_held: Arc<AtomicBool>,
    /// What the ride is doing. Not a bool: a plain sound must not flip it, and "landing"
    /// (the end segment is being written) has to be told apart from both "riding" and
    /// "off", or the trigger restarts a ride that is still finishing.
    ride: Ride,
    /// Bank missing is logged once, not per press.
    warned_missing: bool,
}

impl Sound {
    pub fn new(bank: PathBuf, device: String) -> Self {
        Self {
            bank,
            device,
            child: None,
            wheee_held: Arc::new(AtomicBool::new(false)),
            ride: Ride::Off,
            warned_missing: false,
        }
    }

    /// Random wav from the bank's `tag` directory — the prototype's nanos-based pick, which
    /// is exactly as random as a duck needs.
    fn pick(&mut self, tag: &str) -> Option<PathBuf> {
        let dir = self.bank.join(tag);
        let mut files: Vec<_> = std::fs::read_dir(&dir)
            .ok()?
            .filter_map(|e| Some(e.ok()?.path()))
            .filter(|p| p.extension().is_some_and(|x| x == "wav"))
            .collect();
        if files.is_empty() {
            return None;
        }
        files.sort();
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.subsec_nanos() as usize)
            .unwrap_or(0);
        Some(files.swap_remove(nanos % files.len()))
    }

    /// Stop whatever is on the PCM, now. The ride's abrupt exit, and what every one-shot
    /// does to its predecessor.
    fn stop_child(&mut self) {
        self.wheee_held.store(false, Ordering::Relaxed);
        self.ride = Ride::Off;
        if let Some(mut child) = self.child.take()
            && let Ok(None) = child.try_wait()
        {
            let _ = child.kill();
            let _ = child.wait();
        }
    }

    /// The ride's *other* exit: let the writer out of its loop but leave the pipe open, so
    /// it writes the end segment and `aplay` drains it. The child stays parented here and
    /// is reaped by [`Self::reap_landing`] on a later tick.
    fn land_ride(&mut self) {
        self.wheee_held.store(false, Ordering::Relaxed);
        self.ride = Ride::Landing;
        if self.child.is_none() {
            // Nothing to land (the degraded path's one-shot already finished, or never
            // spawned): the PCM is free again straight away.
            self.ride = Ride::Off;
        }
    }

    /// Free the PCM once a landing ride has actually finished. Called per tick while
    /// landing, which is the only cadence available — nothing here waits.
    fn reap_landing(&mut self) {
        match self.child.as_mut().map(|c| c.try_wait()) {
            None | Some(Ok(Some(_))) | Some(Err(_)) => {
                self.child = None;
                self.ride = Ride::Off;
            }
            Some(Ok(None)) => {}
        }
    }

    /// Play a voice-bank sound, cutting off any still-playing one. `blocking` waits for
    /// playback — used right before poweroff so the goodbye peck is heard.
    pub fn play(&mut self, tag: &str, blocking: bool) {
        // A ride owns the single-client PCM for as long as it lasts. Cutting it for a
        // 200 ms chirp would restart the wheee from its start segment on every press of the
        // other trigger — and holding both triggers is the expected way to use them, since
        // either one opens the mouth. The one-shot is dropped, not queued: it is an event,
        // and an event that arrives during a ride is stale by the time the ride ends.
        //
        // The blocking goodbye peck is the exception, and the only one: it is the last
        // sound this process makes, so it takes the PCM off whatever holds it.
        if self.ride != Ride::Off && !blocking {
            tracing::debug!(tag, "sound skipped: the wheee ride has the PCM");
            return;
        }
        self.stop_child();
        let Some(wav) = self.pick(tag) else {
            if !self.warned_missing {
                self.warned_missing = true;
                tracing::warn!(
                    bank = %self.bank.display(),
                    "no voice bank — sounds are skipped (run `sounds ensure-bank`)"
                );
            }
            return;
        };
        let child = Command::new("aplay")
            .args(["-q", "-D", &self.device])
            .arg(&wav)
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn();
        match child {
            Ok(mut c) if blocking => wait_bounded(&mut c, BLOCKING_PLAY_MAX),
            Ok(c) => self.child = Some(c),
            Err(e) => tracing::debug!(error = %e, tag, "aplay failed"),
        }
    }

    /// The ride, level-driven: the loop calls this every tick with what the wheee hold
    /// currently says. The rising edge starts it; the two ways of stopping are different
    /// sounds, which is the whole point of [`WheeeHold`] carrying three states.
    pub fn wheee(&mut self, hold: WheeeHold) {
        match hold {
            // A press during a landing ride supersedes it — `start_wheee` takes the PCM.
            WheeeHold::Held if self.ride != Ride::Riding => self.start_wheee(),
            WheeeHold::Held => {}
            // The client said "released": cut it, as the prototype does.
            WheeeHold::Released if self.ride != Ride::Off => self.stop_child(),
            // The hold went stale — the client stopped re-notifying, or stopped existing.
            // Land the ride rather than chopping it: nobody asked for a cut.
            WheeeHold::Decayed if self.ride == Ride::Riding => self.land_ride(),
            WheeeHold::Decayed if self.ride == Ride::Landing => self.reap_landing(),
            WheeeHold::Released | WheeeHold::Decayed => {}
        }
    }

    /// A bank with no wheee triads — a half-rendered `ensure-bank`, or a bank from before
    /// the wheee was segmented. Fall back to the plain one-shot, and *latch the ride
    /// anyway*: the trigger asks every 20 ms, and without a latch this path would fork,
    /// exec and kill `aplay` fifty times a second (plus a `read_dir` and three ~110 KB
    /// reads per tick) for as long as the trigger is down, with nothing audible to show
    /// for it. Landing it is a no-op — there is no end segment to write.
    fn degraded_wheee(&mut self) {
        self.play("wheee", false);
        self.ride = Ride::Riding;
    }

    /// Stream start → loop (while held) → end into one `aplay`, so the loop wraps gap-free.
    fn start_wheee(&mut self) {
        self.stop_child();
        let dir = self.bank.join("wheee");
        let mut letters: Vec<String> = std::fs::read_dir(&dir)
            .map(|rd| {
                rd.filter_map(|e| {
                    let name = e.ok()?.file_name().into_string().ok()?;
                    Some(
                        name.strip_prefix("wheee_start_")?
                            .strip_suffix(".wav")?
                            .to_owned(),
                    )
                })
                .collect()
            })
            .unwrap_or_default();
        if letters.is_empty() {
            // A bank without triads (or no bank): the one-shot path says what's wrong.
            self.degraded_wheee();
            return;
        }
        letters.sort();
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.subsec_nanos() as usize)
            .unwrap_or(0);
        let letter = letters.swap_remove(nanos % letters.len());
        let seg = |name: &str| read_wav_pcm(&dir.join(format!("wheee_{name}_{letter}.wav")));
        let (Some((rate, start_pcm)), Some((_, loop_pcm)), Some((_, end_pcm))) =
            (seg("start"), seg("loop"), seg("end"))
        else {
            self.degraded_wheee();
            return;
        };
        let child = Command::new("aplay")
            .args([
                "-q",
                "-D",
                &self.device,
                "-t",
                "raw",
                "-f",
                "S16_LE",
                "-c",
                "1",
            ])
            .args(["-r", &rate.to_string(), "--buffer-time=120000"])
            .stdin(Stdio::piped())
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn();
        let Ok(mut child) = child else {
            tracing::debug!("aplay failed; no wheee");
            return;
        };
        let Some(mut stdin) = child.stdin.take() else {
            return;
        };
        self.child = Some(child);
        self.ride = Ride::Riding;
        self.wheee_held.store(true, Ordering::Relaxed);
        let held = self.wheee_held.clone();

        std::thread::Builder::new()
            .name("wheee".into())
            .spawn(move || {
                use std::io::Write;
                // robotd restores SIGPIPE's default disposition at startup (so piping its
                // stdout behaves), which would make a write into a dead aplay kill the
                // whole daemon. Block it on this thread — the writes then fail with EPIPE,
                // which every `send` below already treats as "the ride is over".
                unsafe {
                    let mut set: libc::sigset_t = std::mem::zeroed();
                    libc::sigemptyset(&mut set);
                    libc::sigaddset(&mut set, libc::SIGPIPE);
                    libc::pthread_sigmask(libc::SIG_BLOCK, &set, std::ptr::null_mut());
                }
                // Pace writes to stay only ~250 ms ahead of playback — otherwise the pipe
                // queues ~0.7 s of loop audio and the end starts that late after release.
                const CHUNK: usize = 8192;
                let bytes_per_sec = f64::from(rate) * 2.0;
                let t0 = Instant::now();
                let mut sent = 0usize;
                let mut send = |stdin: &mut std::process::ChildStdin, chunk: &[u8]| -> bool {
                    let ahead = sent as f64 / bytes_per_sec - t0.elapsed().as_secs_f64();
                    if ahead > 0.25 {
                        std::thread::sleep(Duration::from_secs_f64(ahead - 0.25));
                    }
                    sent += chunk.len();
                    stdin.write_all(chunk).is_ok()
                };
                for chunk in start_pcm.chunks(CHUNK) {
                    if !send(&mut stdin, chunk) {
                        return;
                    }
                }
                'ride: while held.load(Ordering::Relaxed) {
                    for chunk in loop_pcm.chunks(CHUNK) {
                        if !send(&mut stdin, chunk) {
                            break 'ride;
                        }
                    }
                }
                // Out of the loop with the pipe still open: the hold decayed and
                // `land_ride` cleared the flag without killing us. Play the ride out.
                for chunk in end_pcm.chunks(CHUNK) {
                    if !send(&mut stdin, chunk) {
                        return;
                    }
                }
            })
            .ok();
    }
}

/// Wait for a child, but not forever. The goodbye peck runs with `poweroff()` on the next
/// line: if the PCM wedges — it is single-client, and the ride this just killed does not
/// release the device synchronously — an unbounded `wait()` leaves the robot powered on with
/// its intents already disabled, which is worse than a clipped goodbye.
fn wait_bounded(child: &mut Child, timeout: Duration) {
    let deadline = Instant::now() + timeout;
    loop {
        match child.try_wait() {
            Ok(Some(_)) => return,
            Err(_) => return,
            Ok(None) if Instant::now() >= deadline => break,
            Ok(None) => std::thread::sleep(Duration::from_millis(20)),
        }
    }
    tracing::warn!(
        ?timeout,
        "aplay did not finish in time; going on without it"
    );
    let _ = child.kill();
    let _ = child.wait();
}

/// Read a PCM wav: (sample_rate, raw S16LE payload). A minimal RIFF chunk walk — enough for
/// the mono 16-bit files the voice bank contains.
fn read_wav_pcm(path: &Path) -> Option<(u32, Vec<u8>)> {
    let b = std::fs::read(path).ok()?;
    if b.len() < 12 || &b[..4] != b"RIFF" || &b[8..12] != b"WAVE" {
        return None;
    }
    let mut rate = 0u32;
    let mut pos = 12usize;
    while pos + 8 <= b.len() {
        let sz = u32::from_le_bytes(b[pos + 4..pos + 8].try_into().ok()?) as usize;
        let body = pos + 8;
        match &b[pos..pos + 4] {
            b"fmt " if body + 8 <= b.len() => {
                rate = u32::from_le_bytes(b[body + 4..body + 8].try_into().ok()?);
            }
            b"data" if rate > 0 => {
                return Some((rate, b[body..(body + sz).min(b.len())].to_vec()));
            }
            _ => {}
        }
        pos = body + sz + (sz & 1);
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The RIFF walk must read back what the `sounds` crate writes — the two halves of the
    /// wheee pipeline meet at this file format.
    #[test]
    fn read_wav_pcm_reads_what_sounds_writes() {
        let dir = std::env::temp_dir().join(format!("sound-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("t.wav");
        let buf: Vec<f32> = (0..480).map(|i| (i as f32 / 480.0).sin()).collect();
        sounds::to_wav(&buf, &path).unwrap();

        let (rate, pcm) = read_wav_pcm(&path).expect("wav must parse");
        assert_eq!(rate, sounds::SR);
        assert_eq!(pcm.len(), 480 * 2, "S16LE mono payload");
        std::fs::remove_dir_all(&dir).ok();
    }

    /// A missing bank must degrade to silence, not errors — robots without a codec walk on.
    #[test]
    fn a_missing_bank_is_silent_not_fatal() {
        let mut sound = Sound::new(PathBuf::from("/nonexistent/bank"), "default".into());
        sound.play("chirp", false);
        sound.wheee(WheeeHold::Held);
        sound.wheee(WheeeHold::Released);
        assert!(sound.child.is_none());
        assert_eq!(sound.ride, Ride::Off);
    }

    /// A bank whose `wheee/` holds no triads (a half-rendered `ensure-bank`, or a bank from
    /// before the wheee was segmented) must latch the ride on the fallback. Without the
    /// latch the trigger re-enters `start_wheee` every 20 ms for as long as it is held —
    /// a `read_dir` and an `aplay` spawn/kill pair, fifty times a second, silently.
    #[test]
    fn a_bank_without_wheee_triads_latches_instead_of_respawning() {
        let dir = std::env::temp_dir().join(format!("sound-triads-{}", std::process::id()));
        std::fs::create_dir_all(dir.join("wheee")).unwrap();
        let mut sound = Sound::new(dir.clone(), "null".into());

        sound.wheee(WheeeHold::Held);
        assert_eq!(sound.ride, Ride::Riding, "the fallback must latch");
        sound.wheee(WheeeHold::Held);
        assert_eq!(sound.ride, Ride::Riding, "a held trigger must not re-enter");
        sound.wheee(WheeeHold::Released);
        assert_eq!(sound.ride, Ride::Off);
        std::fs::remove_dir_all(&dir).ok();
    }

    /// The two exits are different sounds, and only one of them plays `wheee_end_*`: a
    /// client that says "released" gets the cut, a hold that goes stale gets the landing.
    /// Both must end at `Off` — a ride that cannot leave `Landing` blocks every one-shot.
    #[test]
    fn a_decayed_hold_lands_and_a_release_cuts() {
        let dir = std::env::temp_dir().join(format!("sound-exits-{}", std::process::id()));
        std::fs::create_dir_all(dir.join("wheee")).unwrap();
        let mut sound = Sound::new(dir.clone(), "null".into());

        // Decay: land, then reap. No child ever spawned here, so the reap is immediate —
        // on a real ride it takes as many ticks as the end segment lasts.
        sound.wheee(WheeeHold::Held);
        sound.wheee(WheeeHold::Decayed);
        assert_eq!(
            sound.ride,
            Ride::Off,
            "a landing ride with no child frees the PCM"
        );

        // Release: cut, straight to Off, and the writer's loop flag is down either way.
        sound.wheee(WheeeHold::Held);
        sound.wheee(WheeeHold::Released);
        assert_eq!(sound.ride, Ride::Off);
        assert!(!sound.wheee_held.load(Ordering::Relaxed));
        std::fs::remove_dir_all(&dir).ok();
    }
}
