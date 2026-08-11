//! What `scripts/robot-rescue` decides, exercised against a temporary tree.
//!
//! The rescue path is the code that has to work when everything else does not, and it is the code
//! least amenable to a test: on a real board it only runs when a release cannot start. So the
//! decision is deliberately a pure function of two symlinks — `current` and `golden` — and the
//! script takes `ROBOT_INSTALL_DIR`/`ROBOT_STATE_DIR` from the environment so that function can be
//! asked every question here, on a laptop, with no systemd and no board.
//!
//! In `xtask` because this tests a *repository script* rather than any crate's behaviour, which is
//! what the rest of this directory is for. `sh` and not `bash`: the script runs on a board where
//! the interpreter is whatever `/bin/sh` is, and the flag detection inside it (`mv -T` on GNU,
//! `mv -h` on BSD) exists precisely because the same script has to work here and there.
//!
//! What is *not* covered: whether anything ever calls it. Nothing does yet — see
//! `docs/project/boot-recovery-net.md` for the boot deadline that will.

use std::os::unix::fs::symlink;
use std::path::{Path, PathBuf};
use std::process::{Command, Output};

/// A board tree: `releases/<version>` directories, and whichever symlinks the case needs.
struct Board {
    dir: tempfile::TempDir,
}

impl Board {
    fn with_releases(versions: &[&str]) -> Self {
        let dir = tempfile::tempdir().expect("tempdir");
        for version in versions {
            std::fs::create_dir_all(dir.path().join("releases").join(version)).expect("release");
        }
        std::fs::create_dir_all(dir.path().join("state")).expect("state dir");
        Self { dir }
    }

    fn install(&self) -> PathBuf {
        self.dir.path().to_path_buf()
    }

    fn state(&self) -> PathBuf {
        self.dir.path().join("state")
    }

    /// Link `name` at a release the way the store does: a target relative to the install dir.
    fn link(&self, name: &str, version: &str) -> &Self {
        symlink(
            Path::new("releases").join(version),
            self.dir.path().join(name),
        )
        .expect("symlink");
        self
    }

    fn link_target(&self, name: &str) -> Option<String> {
        std::fs::read_link(self.dir.path().join(name))
            .ok()
            .map(|t| t.to_string_lossy().into_owned())
    }

    fn breadcrumb(&self) -> Option<String> {
        std::fs::read_to_string(self.state().join("rescued")).ok()
    }

    /// Run the script, with a stub `systemctl` on `PATH` recording whatever it is asked to do.
    fn rescue(&self, args: &[&str]) -> Rescue {
        let script = Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .expect("xtask/ has a parent")
            .join("scripts/robot-rescue");

        let stub_dir = self.dir.path().join("stub");
        std::fs::create_dir_all(&stub_dir).expect("stub dir");
        let log = stub_dir.join("systemctl.log");
        std::fs::write(
            stub_dir.join("systemctl"),
            format!("#!/bin/sh\necho \"$@\" >> {}\n", log.display()),
        )
        .expect("stub");
        std::fs::set_permissions(
            stub_dir.join("systemctl"),
            <std::fs::Permissions as std::os::unix::fs::PermissionsExt>::from_mode(0o755),
        )
        .expect("stub mode");

        let output = Command::new("sh")
            .arg(&script)
            .args(args)
            .env("ROBOT_INSTALL_DIR", self.install())
            .env("ROBOT_STATE_DIR", self.state())
            .env(
                "PATH",
                format!(
                    "{}:{}",
                    stub_dir.display(),
                    std::env::var("PATH").unwrap_or_default()
                ),
            )
            .output()
            .expect("run robot-rescue");

        Rescue {
            output,
            systemctl: std::fs::read_to_string(&log).unwrap_or_default(),
        }
    }
}

struct Rescue {
    output: Output,
    systemctl: String,
}

impl Rescue {
    fn code(&self) -> i32 {
        self.output.status.code().expect("exited")
    }

    fn stderr(&self) -> String {
        String::from_utf8_lossy(&self.output.stderr).into_owned()
    }
}

/// No golden means no rollback target, which is every board until 1.0.0 exists. Declining is the
/// answer, and it has to name both reasons: unset in the config, or set but never published.
#[test]
fn declines_when_no_golden_is_published() {
    let board = Board::with_releases(&["1.2.0"]);
    board.link("current", "1.2.0");

    let run = board.rescue(&[]);

    assert_eq!(run.code(), 2, "stderr: {}", run.stderr());
    assert!(run.stderr().contains("no golden release published"));
    assert!(
        run.stderr().contains("updater.toml"),
        "an operator needs to be told where golden is set: {}",
        run.stderr()
    );
    assert_eq!(
        board.link_target("current").as_deref(),
        Some("releases/1.2.0")
    );
}

/// A configured golden that was pruned or never installed. The link resolves to nothing, and
/// swapping onto it would leave `current` pointing at an empty path — a board that cannot exec
/// anything at all, which is worse than the failure being rescued.
#[test]
fn declines_when_golden_is_not_installed() {
    let board = Board::with_releases(&["1.2.0"]);
    board.link("current", "1.2.0").link("golden", "1.0.0");

    let run = board.rescue(&[]);

    assert_eq!(run.code(), 2, "stderr: {}", run.stderr());
    assert!(run.stderr().contains("not installed"), "{}", run.stderr());
    assert_eq!(
        board.link_target("current").as_deref(),
        Some("releases/1.2.0")
    );
}

/// The check that keeps a hardware fault from becoming a reboot loop: if the daemons are down on
/// the release carrying the standing guarantee, this is not a release fault and a swap changes
/// nothing except adding a reboot.
#[test]
fn declines_when_current_is_already_golden() {
    let board = Board::with_releases(&["1.0.0"]);
    board.link("current", "1.0.0").link("golden", "1.0.0");

    let run = board.rescue(&["--reboot"]);

    assert_eq!(run.code(), 2, "stderr: {}", run.stderr());
    assert!(run.stderr().contains("already golden"), "{}", run.stderr());
    assert!(
        run.systemctl.is_empty(),
        "declining must not reboot, even when asked to: {:?}",
        run.systemctl
    );
}

#[test]
fn swaps_current_to_golden_and_records_it() {
    let board = Board::with_releases(&["1.0.0", "1.2.0"]);
    board.link("current", "1.2.0").link("golden", "1.0.0");

    let run = board.rescue(&[]);

    assert_eq!(run.code(), 0, "stderr: {}", run.stderr());
    assert_eq!(
        board.link_target("current").as_deref(),
        Some("releases/1.0.0"),
        "current must point at golden, and at a *relative* target like the store writes"
    );

    let breadcrumb = board.breadcrumb().expect("a breadcrumb in the state dir");
    assert!(
        breadcrumb.contains("1.2.0") && breadcrumb.contains("1.0.0"),
        "the breadcrumb has to say what was swapped for what: {breadcrumb:?}"
    );
}

/// A board with no live release at all — a swap interrupted before it linked, or a `current`
/// deleted by hand. There is nothing to lose and golden is exactly where it should be put.
#[test]
fn acts_when_there_is_no_current_at_all() {
    let board = Board::with_releases(&["1.0.0"]);
    board.link("golden", "1.0.0");

    let run = board.rescue(&[]);

    assert_eq!(run.code(), 0, "stderr: {}", run.stderr());
    assert_eq!(
        board.link_target("current").as_deref(),
        Some("releases/1.0.0")
    );
}

/// `--reboot` is opt-in because the robot may be standing: every unit execs through `current`, so
/// the swap does nothing until they restart, and whoever is holding the robot decides when that is.
#[test]
fn does_not_reboot_unless_asked() {
    let board = Board::with_releases(&["1.0.0", "1.2.0"]);
    board.link("current", "1.2.0").link("golden", "1.0.0");

    let quiet = board.rescue(&[]);
    assert_eq!(quiet.code(), 0, "stderr: {}", quiet.stderr());
    assert!(
        quiet.systemctl.is_empty(),
        "swapped and rebooted without being asked: {:?}",
        quiet.systemctl
    );
    assert!(
        quiet.stderr().contains("systemctl reboot"),
        "having declined to reboot, it must say how: {}",
        quiet.stderr()
    );

    // Back to a release that is not golden, so the second run has something to do.
    std::fs::remove_file(board.install().join("current")).expect("unlink current");
    board.link("current", "1.2.0");

    let loud = board.rescue(&["--reboot"]);
    assert_eq!(loud.code(), 0, "stderr: {}", loud.stderr());
    assert_eq!(loud.systemctl.trim(), "reboot");
}

/// `--dry-run` is what an operator reaches for first, and on a board that is merely suspect it must
/// not be the thing that changes the release.
#[test]
fn dry_run_decides_but_changes_nothing() {
    let board = Board::with_releases(&["1.0.0", "1.2.0"]);
    board.link("current", "1.2.0").link("golden", "1.0.0");

    let run = board.rescue(&["--dry-run"]);

    assert_eq!(run.code(), 0, "stderr: {}", run.stderr());
    assert!(run.stderr().contains("would swap"), "{}", run.stderr());
    assert_eq!(
        board.link_target("current").as_deref(),
        Some("releases/1.2.0")
    );
    assert!(board.breadcrumb().is_none(), "a dry run left a breadcrumb");
    assert!(run.systemctl.is_empty());
}

/// An unknown flag must not be ignored. A timer that grows a typo in its `ExecStart` should fail
/// loudly rather than silently rescue on the wrong terms.
#[test]
fn refuses_an_argument_it_does_not_understand() {
    let board = Board::with_releases(&["1.0.0", "1.2.0"]);
    board.link("current", "1.2.0").link("golden", "1.0.0");

    let run = board.rescue(&["--force"]);

    assert_eq!(run.code(), 1, "stderr: {}", run.stderr());
    assert_eq!(
        board.link_target("current").as_deref(),
        Some("releases/1.2.0")
    );
}
