# The install path has no test

Status: open · Date: 2026-08-05 · Owner: pierre

Three bugs in a row reached a board, all in the install path, none caught by 418 tests or by
`board-test.sh`. This records why, and what would actually close it. Written the same day, while
the reasons are still concrete.

A second section covers two findings that *look* like the same thing and are not: version skew
between a daemon that is already running and a release that has just been installed. Neither would
be caught by any amount of install testing, and they need their own fixes — kept here rather than in
their own document because anyone investigating one will arrive believing it is the other.

## What got through

All three landed within a day, all while installing `btd` and `configd` onto a dev board.

**1. `on_apply` restarts several units in one command.** `systemctl restart robotd configd` fails as
a whole if *either* unit is unknown — and fails without restarting the one that exists. A release
introducing a new daemon therefore could not restart anything, because the unit file arrives *with*
that release and is not installed when `on_apply` runs. The rollback reason was
`not healthy within 30s: unreachable`, which names neither the unit nor the command.

Latent rather than fatal, since a board keeps its own `/etc/robot/updater.toml`. Fixed by
restarting one unit at a time and skipping a unit systemd does not know.

**2. The artifact did not contain the new units.** The packaging workflows name every shipped file
with an explicit `--include`, and the units were added to `install.sh` without being added there.
Found by hand: `ls current/systemd/` on the board.

**3. The artifact did not contain the new binaries.** The Package step copies binaries with an
explicit `cp` per binary. `cargo board --bins` built `btd` and `configd`; nothing staged them.
`btd.service` failed with `203/EXEC`, which reads as a broken daemon rather than an incomplete
artifact.

Bug 3 is the same class as bug 2, in the same file, **two commits after a test was added to stop
bug 2 recurring** — that test checked units and not the binaries they exec. Each fix was correct
and each was too narrow, which is the pattern worth attacking rather than the bugs.

## Why the existing tests could not have caught them

Not an accusation of the tests; they cover what they claim. The point is what nothing covers.

| | covers | does not |
|---|---|---|
| `board-test.sh` | binaries executed on real aarch64 Linux from the *build directory*: engine behaviour, socket modes, `SO_PEERCRED`, the layered authorisation, `setup-board.sh` against a stubbed `systemctl` | unpacking an artifact, installing units, starting services |
| `xtask` tests | the workflow YAML vs `install.sh`, and unit `ExecStart` vs staged binaries | whether the *built artifact* matches either — it reads source files |
| `updater` tests | engine, journal, verification, rollback, with fakes | `install.sh` at all |
| `shipped_config_is_safe_for_a_client_robot` | `deploy/updater.toml`'s content | that the config is installable |

So: **no test takes a real artifact and installs it.** Every check either runs a binary that was
never packaged, or reads a source file that describes packaging without observing it. The two
`xtask` tests I added are strictly better than nothing and still the weaker form — they assert that
two files agree with each other, not that the thing they produce is correct.

## What would close it

Roughly in order of cost.

**A. Assert the artifact's contents.** Run `xtask package` in a test, then inspect the tarball:
every unit named by `install.sh` is present, and every unit's `ExecStart` binary is present. Cheap,
and strictly stronger than the two current tests because it observes the artifact instead of the
YAML that builds it. **Would have caught bugs 2 and 3.**

**B. Install the artifact in a container, with `systemctl` stubbed.** Extend `board-test.sh`: unpack
into a fake root, run `install.sh` against it, and assert what landed where —
`/etc/systemd/system/*.service`, `/usr/lib/sysusers.d/`, the `robotctl` symlink, the state
directory. `setup-board.sh` is already tested this way, so the pattern and the stub exist. Catches
bugs 2 and 3 *and* file-placement regressions in `install.sh`, which nothing tests today.

**C. Real systemd in a container.** `systemd-nspawn`, or a privileged container with systemd as
pid 1. Full fidelity: units actually start, `on_apply` actually restarts, the health gate actually
gates. **Would have caught bug 1** — the only one A and B miss. Real work: cgroup and privilege
setup in CI, and slow.

**D. A board as a self-hosted runner.** Highest fidelity, and the only thing that ever tests the
motor bus, the radio and the timings. Ops cost, and a single point of failure for CI.

## Suggested first step

**A, then B.** Together they cover two of the three bugs and remove the "files agree with each
other" weakness, for a fraction of C's cost. B is the better value of the two because it exercises
`install.sh`, which is 500 lines that nothing currently runs.

C is worth revisiting when `on_apply` grows again — it is the only option that tests the restart
and the gate, and bug 1 will have siblings.

D is a separate conversation, and probably follows M4 rather than preceding it.

## A second, separate problem: version skew on the dev channel

Found in the same session and easily confused with the above, but no amount of install testing
would catch either — in both cases the artifact was complete and correct.

The shape is always the same. **`updaterd` never restarts itself during an update** (§4.1), so it
keeps running the old binary until the next reboot, while everything else on the box moves to the
new release immediately. Any change to a shared contract therefore has a window where a *resident*
daemon and an *installed* one disagree. On the dev channel, where branches are installed over each
other in any order, that window is the normal case rather than a rare one.

Two instances, both real, both cost about an hour.

### 1. A required field added to `HealthResult`

`main` added `consecutive_stale_blocks` to the IMU section and released 0.2.0 from it. A branch that
had merged `main` *before* that did not send the field. `robotd` came up perfectly — served the
socket, loaded both policies, ran the loop at 50 Hz — and the resident 0.2.0 `updaterd` rejected its
reply for a missing field. `RobotIo::health` maps an unparseable answer to `Health::Unreachable`, so
the gate reported `not healthy within 30s: unreachable` about a robot that was entirely healthy, and
reverted a good release.

Fixes, both small:

- **`#[serde(default)]` on new `HealthResult` fields**, so a newer `updaterd` can still parse an
  older `robotd`. Every `--ref` install of a branch predating a health-field addition hits this
  otherwise, which is the entire dev workflow.
- **A distinct `Health::Incompatible`** rather than reusing `Unreachable`, so the reason reads
  "answered in a shape this updaterd does not understand" instead of implying the robot is down.
  Pure diagnostics, and it would have found the above in a minute.

### 2. `API_VERSION` skew between `robotctl` and `updaterd`

`robotctl` is a symlink into `current`, so it follows the installed release. `updaterd` does not. So
the moment a release changes `API_VERSION`, the two are **guaranteed** to disagree until `updaterd`
restarts — and the command that stops working is `robotctl update apply`, which is exactly the one
you would use to get out of it.

Demonstrated by ordinary use rather than by contrivance: install branch A, install branch B while
waiting for CI, and every `robotctl` call fails with `client speaks API v2, daemon speaks v3`. The
escape is to invoke a `robotctl` from a release directory whose version matches the running daemon,
or to restart `updaterd` — neither of which is discoverable from the error.

The handshake at least *caught* it and named both versions, which is more than case 1 managed.

Proposed fix, and the reason it is not already done is that it is a protocol-policy decision rather
than a bug:

- **`hello` should refuse only when the client is *newer* than the daemon.** A v3 daemon can serve a
  v2 client perfectly, because v3 only *added* methods — refusing that direction costs the ability
  to recover and buys nothing. Client-newer-than-daemon must stay a hard failure: there the client
  may ask for something that genuinely is not there.

That change would also make `API_VERSION` mean what it should — "the newest contract I understand"
rather than "the only contract I will speak" — and additive protocol growth would stop being a
breaking change for every client on the box.

### Why these are not install-path bugs

Nothing in options A–D above would have caught either. The artifact was correct both times; what was
wrong was the *pair* of versions running at one moment, which only exists on a machine that has been
updated. Option C (real systemd in a container) would catch the *health-gate* consequence of case 1
if the container also ran an older `updaterd`, but constructing that skew deliberately is a different
kind of test — closer to a compatibility matrix than to an install test, and worth keeping separate.
