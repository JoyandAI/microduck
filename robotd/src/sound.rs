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
//!    the release would land that late. Cutting the ride kills the child and the writer
//!    exits on the broken pipe.
//!
//! Playing is spawning: nothing here blocks the 50 Hz tick except the deliberately blocking
//! goodbye peck right before power-off, when there is no tick left to miss.

use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

/// The player. Constructed once by the control loop; `None`-like when the bank is missing —
/// every play degrades to a debug line, so a robot without a generated bank (or a codec)
/// walks fine and stays quiet.
pub struct Sound {
    bank: PathBuf,
    device: String,
    child: Option<Child>,
    /// The wheee rider loops while this is true. Shared with the writer thread.
    wheee_held: Arc<AtomicBool>,
    /// Whether the current child is a wheee ride (a plain sound must not flip the flag).
    wheee_riding: bool,
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
            wheee_riding: false,
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

    fn kill_current(&mut self) {
        self.wheee_held.store(false, Ordering::Relaxed);
        self.wheee_riding = false;
        if let Some(mut child) = self.child.take()
            && let Ok(None) = child.try_wait()
        {
            let _ = child.kill();
            let _ = child.wait();
        }
    }

    /// Play a voice-bank sound, cutting off any still-playing one. `blocking` waits for
    /// playback — used right before poweroff so the goodbye peck is heard.
    pub fn play(&mut self, tag: &str, blocking: bool) {
        self.kill_current();
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
            Ok(mut c) if blocking => {
                let _ = c.wait();
            }
            Ok(c) => self.child = Some(c),
            Err(e) => tracing::debug!(error = %e, tag, "aplay failed"),
        }
    }

    /// The held ride, level-driven: the loop calls this every tick with "is the trigger
    /// (freshly) held". The rising edge starts the ride, the falling edge cuts it — the
    /// prototype's release kills the streaming aplay outright, and the writer thread exits
    /// on the broken pipe.
    pub fn wheee(&mut self, held: bool) {
        if held && !self.wheee_riding {
            self.start_wheee();
        } else if !held && self.wheee_riding {
            self.kill_current();
        }
    }

    /// Stream start → loop (while held) → end into one `aplay`, so the loop wraps gap-free.
    fn start_wheee(&mut self) {
        self.kill_current();
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
            self.play("wheee", false);
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
            self.play("wheee", false);
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
        self.wheee_riding = true;
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
                // Released without being killed (the hold decayed): land the ride.
                for chunk in end_pcm.chunks(CHUNK) {
                    if !send(&mut stdin, chunk) {
                        return;
                    }
                }
            })
            .ok();
    }
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
        sound.wheee(true);
        sound.wheee(false);
        assert!(sound.child.is_none());
    }
}
