# The install path has no test

Status: open · Date: 2026-08-05 · Owner: pierre

Three bugs in a row reached a board, all in the install path, none caught by 416 tests or by
`board-test.sh`. This records why, and what would actually close it. Written after the third one,
while the reasons are still concrete.

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

## Not the same problem

Worth separating, because it was found in the same session and looks similar.

The branch was also un-installable for a while because `main` added a **required** field to
`HealthResult` after the branch had merged it. `robotd` came up perfectly and `updaterd` rejected
its reply, so the gate reported `unreachable` about a healthy robot. That is **version skew between
a resident daemon and an installed one**, not a packaging gap, and no amount of install testing
would catch it — the artifact was complete and correct.

Its own fixes, both small:

- `#[serde(default)]` on new `HealthResult` fields, so a newer `updaterd` can still parse an older
  `robotd`. Every `--ref` install of a branch predating a health-field addition hits this
  otherwise, which is the entire dev workflow.
- A distinct `Health::Incompatible` instead of reusing `Unreachable`, so the rollback reason says
  "answered in a shape this updaterd does not understand" rather than implying the robot is down.
  This one is pure diagnostics and would have saved an hour.
