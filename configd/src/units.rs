//! What systemd and `/proc` say about the robot's daemons.
//!
//! Started as "is `padd` running?", reported alongside the pads because a connected pad and a dead
//! `padd` is the failure that looks like working hardware: the light on the controller is on, the
//! robot ignores it, and nothing in either place says why. That argument was never specific to
//! `padd` — a dead `btd` is a robot no phone can see, with the same silence — so it answers for
//! every unit a release manages.
//!
//! Asked of systemd rather than tracked: these are started, stopped and restarted by systemd, so
//! systemd is the only thing that knows. `configd` deliberately holds no opinion — it does not start
//! them, does not restart them, and reporting is the whole of its involvement.
//!
//! ## Which release is running, and why it cannot be asked
//!
//! `updaterd`, `robotd` and `configd` report their build over their own sockets, so "it did not
//! restart into the new release" is already visible for those three. `btd` and `padd` serve no
//! socket — and a socket would not help, because **the process that needs interrogating is the old
//! one.** Whatever mechanism it would answer through, it predates the answer.
//!
//! So it is read from outside: systemd knows `MainPID`, and `/proc/<pid>/exe` resolves to the path
//! the binary was actually executed from — the `current` symlink already followed, since the kernel
//! records the file rather than the name used to reach it. A release installs to
//! `…/releases/<version>/bin/<name>`, so that path names the release, and it names the *old* one
//! when the restart did not happen.
//!
//! Reading another user's `/proc/<pid>/exe` needs privilege — `btd` runs as `btd`, `padd` as `padd`
//! — which `configd` has and `robotctl` does not. That is the reason this answer is served from
//! here rather than gathered by the CLI.

use duck_ipc_proto as proto;

/// The unit that turns a pad into intents.
pub const PADD: &str = "padd.service";

/// Every unit a daemon release manages, in the order a reader wants them: the update engine, then
/// the robot, then the three that depend on both.
///
/// Hardcoded rather than discovered, and that is a real limitation worth naming: a unit added to a
/// release and not to this list is invisible here. The alternative — asking systemd for everything
/// and filtering — reports units this project does not own, which is worse for a status line.
/// `scripts/install.sh` enables exactly these.
pub const MANAGED: [&str; 5] = [
    "updaterd.service",
    "robotd.service",
    "configd.service",
    "btd.service",
    "padd.service",
];

/// The release a binary at this path was installed as.
///
/// Matches the layout rather than a configured root: any path with a `releases/<version>/` in it
/// yields that version. The root is `updater.toml`'s to choose, and hardcoding a copy of it here is
/// how the two drift apart.
///
/// A path with no such component is not an error — a hand-built binary run from a developer's home
/// directory is a normal thing on a dev board, and the full path is reported alongside.
pub fn release_from_path(path: &str) -> Option<proto::semver::Version> {
    let path = path.strip_suffix(" (deleted)").unwrap_or(path);
    let mut parts = path.split('/');
    while let Some(part) = parts.next() {
        if part == "releases"
            && let Some(version) = parts.next()
            && let Ok(version) = proto::semver::Version::parse(version)
        {
            return Some(version);
        }
    }
    None
}

/// What systemd says about one unit. The narrow question, kept for `pad.status`.
pub async fn state(unit: &str) -> proto::UnitState {
    describe(unit).await.state
}

#[cfg(target_os = "linux")]
pub async fn all() -> Vec<proto::ServiceUnit> {
    let mut units = Vec::with_capacity(MANAGED.len());
    for unit in MANAGED {
        units.push(describe(unit).await);
    }
    units
}

#[cfg(not(target_os = "linux"))]
pub async fn all() -> Vec<proto::ServiceUnit> {
    // Off the board there is no systemd to ask, and inventing an answer would make a laptop look
    // like a robot with every daemon stopped.
    MANAGED
        .into_iter()
        .map(|unit| proto::ServiceUnit {
            unit: unit.to_owned(),
            state: proto::UnitState::Unknown,
            release: None,
            binary: None,
            deleted: false,
        })
        .collect()
}

#[cfg(target_os = "linux")]
pub async fn describe(unit: &str) -> proto::ServiceUnit {
    let (state, pid) = match query(unit).await {
        Ok(answer) => answer,
        Err(e) => {
            // A warning, not an error: this is one line of a status report, and failing to read it
            // must not fail the report.
            tracing::warn!(error = %e, unit, "could not ask systemd about a unit");
            (proto::UnitState::Unknown, None)
        }
    };

    // Only for a running process: a PID from a stopped unit is 0, and `/proc/0` is not a thing.
    let binary = pid.filter(|pid| *pid > 0).and_then(running_binary);
    proto::ServiceUnit {
        unit: unit.to_owned(),
        state,
        release: binary
            .as_deref()
            .and_then(release_from_path)
            .filter(|_| state == proto::UnitState::Active),
        deleted: binary
            .as_deref()
            .is_some_and(|path| path.ends_with(" (deleted)")),
        binary,
    }
}

#[cfg(not(target_os = "linux"))]
pub async fn describe(unit: &str) -> proto::ServiceUnit {
    proto::ServiceUnit {
        unit: unit.to_owned(),
        state: proto::UnitState::Unknown,
        release: None,
        binary: None,
        deleted: false,
    }
}

/// The path the running process was executed from, `(deleted)` marker and all.
///
/// The marker is kept rather than stripped: a binary that is gone from disk is the sharpest
/// evidence a restart did not happen, and losing it here would mean guessing at it later.
#[cfg(target_os = "linux")]
fn running_binary(pid: u32) -> Option<String> {
    match std::fs::read_link(format!("/proc/{pid}/exe")) {
        Ok(path) => Some(path.to_string_lossy().into_owned()),
        Err(e) => {
            // Permission is the interesting failure: this needs privilege over a process owned by
            // another user, so it says "configd is not root" rather than "the process is gone".
            tracing::warn!(error = %e, pid, "could not read the running binary's path");
            None
        }
    }
}

#[cfg(target_os = "linux")]
async fn query(unit: &str) -> Result<(proto::UnitState, Option<u32>), String> {
    let bus = zbus::Connection::system()
        .await
        .map_err(|e| e.to_string())?;

    // `LoadUnit` rather than `GetUnit`: `GetUnit` fails for a unit systemd has not loaded, which is
    // indistinguishable from a unit that does not exist — and those are different answers here.
    // `LoadUnit` loads it if the file is there and fails only when it genuinely is not.
    let path: zbus::zvariant::OwnedObjectPath = match bus
        .call_method(
            Some("org.freedesktop.systemd1"),
            "/org/freedesktop/systemd1",
            Some("org.freedesktop.systemd1.Manager"),
            "LoadUnit",
            &(unit),
        )
        .await
    {
        Ok(reply) => reply.body().deserialize().map_err(|e| e.to_string())?,
        Err(e) => {
            // No such unit: a board on a release older than the one that added it. That is a fact
            // about the install, not a failure to report as one.
            tracing::debug!(error = %e, unit, "no such unit");
            return Ok((proto::UnitState::Absent, None));
        }
    };

    let active: String = property(&bus, &path, "org.freedesktop.systemd1.Unit", "ActiveState")
        .await?
        .try_into()
        .map_err(|e: zbus::zvariant::Error| e.to_string())?;

    let state = match active.as_str() {
        // `activating` counts as active: `padd` spends its first moments connecting to `robotd`, and
        // reporting that as "not running" would make a robot mid-boot look broken.
        "active" | "activating" | "reloading" => proto::UnitState::Active,
        // `failed` is inactive with a reason, and the reason is in the journal rather than here.
        // Collapsing them keeps this a status line rather than a diagnosis.
        "inactive" | "deactivating" | "failed" => proto::UnitState::Inactive,
        other => {
            tracing::warn!(state = other, unit, "unfamiliar unit state");
            proto::UnitState::Unknown
        }
    };

    // Read even when inactive, and discarded there: a failed unit reports the PID it last had, and
    // treating that as a live process would name a release nothing is running.
    let pid: u32 = property(&bus, &path, "org.freedesktop.systemd1.Service", "MainPID")
        .await
        .and_then(|value| u32::try_from(value).map_err(|e: zbus::zvariant::Error| e.to_string()))
        .unwrap_or(0);

    Ok((state, (state == proto::UnitState::Active).then_some(pid)))
}

#[cfg(target_os = "linux")]
async fn property(
    bus: &zbus::Connection,
    path: &zbus::zvariant::OwnedObjectPath,
    interface: &str,
    name: &str,
) -> Result<zbus::zvariant::Value<'static>, String> {
    bus.call_method(
        Some("org.freedesktop.systemd1"),
        path,
        Some("org.freedesktop.DBus.Properties"),
        "Get",
        &(interface, name),
    )
    .await
    .map_err(|e| e.to_string())?
    .body()
    .deserialize::<zbus::zvariant::Value>()
    .map(|value| value.try_to_owned().map(Into::into))
    .map_err(|e| e.to_string())?
    .map_err(|e| e.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The path a release actually installs to, which is what makes "which version is running"
    /// answerable at all.
    #[test]
    fn a_release_path_names_its_version() {
        assert_eq!(
            release_from_path("/opt/robot/daemon/releases/0.4.0/bin/btd"),
            Some(proto::semver::Version::parse("0.4.0").unwrap())
        );
    }

    /// A branch build, which is the case this was written against: the crate version says `0.4.0`
    /// for both the old binary and the new one, and **only the path tells them apart**.
    #[test]
    fn a_dev_release_keeps_the_suffix_that_distinguishes_it() {
        let version = release_from_path("/opt/robot/daemon/releases/0.4.0-dev.271.7610e6e/bin/btd");
        assert_eq!(
            version,
            Some(proto::semver::Version::parse("0.4.0-dev.271.7610e6e").unwrap())
        );
        // And it must compare as older than the release it precedes, or "running an older build
        // than is installed" cannot be detected.
        assert!(version.unwrap() < proto::semver::Version::parse("0.4.0").unwrap());
    }

    /// The sharpest case: the binary is gone from disk and the process is still running it. The
    /// version has to survive the marker the kernel appends, or the one report that proves a restart
    /// did not happen would say "unknown".
    #[test]
    fn a_deleted_binary_still_names_its_release() {
        assert_eq!(
            release_from_path("/opt/robot/daemon/releases/0.3.0/bin/padd (deleted)"),
            Some(proto::semver::Version::parse("0.3.0").unwrap())
        );
    }

    /// A hand-built binary on a dev board is not a release and must not be forced into looking like
    /// one. The full path is reported instead, which is more use than a wrong version.
    #[test]
    fn a_binary_outside_the_layout_has_no_release() {
        assert_eq!(
            release_from_path("/home/pierre/duck/target/debug/btd"),
            None
        );
        // A `releases` component whose child is not a version is not a release either.
        assert_eq!(release_from_path("/srv/releases/nightly/bin/btd"), None);
    }
}
