# The install path has no test

Status: open · Date: 2026-08-05, revised 2026-08-07 and 2026-08-11 · Owner: pierre

Four bugs in a row reached a board, all in the install path, none caught by 418 tests or by
`board-test.sh`. This records why, and what would actually close it. Written the same day, while
the reasons are still concrete.

A second section covers two findings that *look* like the same thing and are not: version skew
between a daemon that is already running and a release that has just been installed. Neither would
be caught by any amount of install testing, and they need their own fixes — kept here rather than in
their own document because anyone investigating one will arrive believing it is the other.

**Revision note.** All four bugs are fixed, and so is the version-skew section that follows them:
case 1's `serde(default)` discipline landed, case 2's handshake proposal was decided *against* with
its reasoning written down, and the "units outlive the release that installed them" consequence is
now a refusal rather than emergent behaviour. What is still open is the *testing* gap the title
names — no test installs a real artifact — plus three smaller decisions marked below.

One change since the first draft moves enough of this document to call out here: `updaterd` and
`btd` are now restarted a few seconds *after* the update replies (`RESTART_AFTER_REPLYING`), and the
release's own `updaterd` is proved to start before the commit (`updaterd --self-test`). Two claims
below were written against the old behaviour and are now false — both are corrected in place, and
the second section's premise in particular no longer holds.

**Second revision note.** The skew half of §4 is now closed by code rather than by argument. Every
daemon publishes what it is running (`duck_ipc_proto::publish_identity`), and `updaterd` compares each
unit against the active release at every start and restarts what is stale
(`updater/src/reconcile.rs`, `Engine::reconcile_running_units`). Two more claims below were true when
written and are not now: that a running/installed mismatch is undecided, and that `btd`'s running
revision cannot be read. Both are corrected in place. `docs/design/restart-order.md` §5 and §7 own
the mechanism and how to read a skew; what stays here is why it was needed.

## What got through

All four landed within a day, all while installing `btd` and `configd` onto a dev board. (The count
read "three" until §4 was added below it.)

**1. `on_apply` restarts several units in one command.** `systemctl restart robotd configd` fails as
a whole if *either* unit is unknown — and fails without restarting the one that exists. A release
introducing a new daemon therefore could not restart anything, because the unit file arrives *with*
that release and nothing installed it. The rollback reason was
`not healthy within 30s: unreachable`, which names neither the unit nor the command.

Latent rather than fatal, since a board keeps its own `/etc/robot/updater.toml`.

Fixed twice, and the second one is the real fix. First, defensively: units are restarted one at a
time and a unit systemd does not know is skipped. Then properly: **`hooks/postinstall` installs the
release's units.** The engine has always had a post-install hook point — `extract → [pre_install] →
swap → [post_install] → apply → health gate` — which runs after `current` moves, so `ExecStart`
resolves, and before the restart, so `on_apply` finds a unit that exists. Nothing used it; only
`scripts/install.sh` ever copied units out of a release, so every new service needed a manual step on
every board, forever. That the mechanism existed and was unused was pointed out in review, not found
by me.

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

**4. `on_apply`'s restart list lives on the board, so a new daemon is never restarted.**  ·
**measured** The most expensive one, and the one already written down here as "latent rather than
fatal".

`[component.daemon.on_apply].units` is read from `/etc/robot/updater.toml` — the *operator's* file,
which `install.sh` deliberately preserves. `configd` was added to the shipped `deploy/updater.toml`
in the branch that introduced it, so a board set up before that keeps `units = ["robotd"]` forever.
Every `configd` release therefore swapped the binary and left the old process running.

What made it cost two hours rather than two minutes is the shape of the failure:

- the update reports **success** — the swap happened, the health gate passed, nothing failed;
- `robotctl update apply` then reports **`already_current`** and does nothing, so the obvious
  recovery command is a no-op;
- the daemon keeps answering, on old code, so it looks like the fix was wrong rather than absent.
  Four wifi fixes were verified as broken against binaries that were never running.

`robotctl version` *does* diagnose this, in as many words: "configd is running X but the installed
daemon release is Y … either the restart did not happen, or it failed". Nobody ran it. A diagnostic
that exists and is not reached for is worth as much as one that does not exist, which is an argument
for the update itself noticing rather than for more diagnostics.

**Fixed** (`engine.rs::units_to_restart`): the restart set is derived from the release's
`systemd/*.service` files, unioned with whatever the config names, minus `updaterd` and `btd` — which
moved from a config convention enforced by a test into a `NEVER_RESTART` constant in code, because
they are properties of what those daemons are rather than operator choices. A board whose
`updater.toml` predates a daemon now restarts it anyway, and needs no edit. What follows is the
reasoning, kept because it is the argument for where such lists belong.

**Derive the restart set from the release, not from the board.** A release ships
`systemd/*.service`, so it already states which units it provides — the same realisation that made
`hooks/postinstall` the right answer for *installing* them. The engine can restart what the release
ships, minus the two documented exclusions (`updaterd`, which is performing the update; `btd`, which
may be the transport it was requested over). Then a release that adds a daemon restarts that daemon
on every board, with no operator edit and no way for a board's config to be silently out of date.

Two smaller options were noted here as companions to the above rather than alternatives to it:
`apply` could gain a `--force` that re-runs the hooks and the restart on an already-current release
(there is precedent — `install --force` exists for the same class of chicken-and-egg), and
`already_current` could compare *running* revisions rather than only the installed one, so it stops
being a no-op precisely when something is wrong.

**The second one is answered, and not where it was proposed.** The question it really posed was
whether a running/installed mismatch should be reported and refused or repaired, and the answer is
repaired — by `updater/src/reconcile.rs`, which runs at every `updaterd` start rather than inside
`apply`. Each unit's running release comes from the identity file its process published, is compared
with what its component has active, and anything stale is restarted. Since `updaterd` restarts itself
five seconds after every applied update, that check runs seconds after every `apply`, so the state
this section is about now heals itself with `Engine::apply` unchanged.

What that leaves is narrower than either option as written. `Engine::apply` still returns
`AlreadyCurrent` on the installed version alone, and `reconcile` deliberately does not repair one
skew: `updaterd`'s own, because a self-restart loop in the process that owns recovery is the one
failure with no way out. Those two meet in a single case — a stale `updaterd`, an operator reaching
for `apply`, `already_current`, no restart scheduled, and nothing to fix it but a hand-run `systemctl
restart updaterd`. **Still open**, as one narrow change to `apply` rather than as two options; a
`--force` flag is what is left if `apply` is not to repair skew by itself. If it is added, its guard
needs its own reasoning: `install --force` refuses while `robotd` answers because it disables the
health gate, and `apply` keeps that gate, so copying the guard by symmetry would disable the flag
exactly when a robot is up and skewed.

**Fixed, and this paragraph used to say otherwise.** `btd` is excluded from the *in-flight* restart
because restarting it drops the BLE connection carrying the update's own progress stream — that part
was and remains right. The conclusion drawn from it was not: this said a `btd` fix "needs a manual
restart or a reboot … a real gap in a phone-driven flow", and it does not any more. The exclusion
expires the moment the outcome is on the wire, so `btd` and `updaterd` both restart five seconds later
via `systemd-run --on-active=5s` (`RESTART_AFTER_REPLYING`). A client sees its outcome and then a
dropped connection, which for BLE is an ordinary reconnect.

**Also fixed, and this paragraph used to say otherwise too.** The observability half was said to
survive: `robotctl version` could not see `btd`, because `btd` serves no socket, so there was nothing
to ask. The premise was that the answer had to come over a socket. It does not — every daemon now
writes its identity to a file at startup, `btd` and `padd` included, so `robotctl health` reads the
release each process was launched from and warns when it disagrees with what is installed. No daemon
is unobservable for the reason this claimed.

### A consequence worth knowing: units outlive the release that installed them

`hooks/postinstall` installs the units a release ships and, by design, leaves them behind on a
rollback — the alternative is recording what was added so a revert can undo it, and the hook's own
comment argues that is not worth it because the next successful update reinstalls whatever it ships.

That reasoning holds for a rollback. It does not hold for a **downgrade to a release that predates a
daemon**: the unit stays, its `ExecStart` names a binary the older release does not contain, and the
daemon fails with `203/EXEC`. Once that daemon is also in `on_apply`'s restart set, the failed restart
fails the *update*, which reverts.

Observed exactly this way: `robotctl update apply daemon` on a board running a dev build resolved to
stable `0.2.0`, which predates `configd`; `configd.service` could not start; the engine rolled back
and said so. The outcome is right — a board should not silently downgrade below the release that
introduced a daemon it is now running — but nothing states the rule, and the error names a systemd
failure rather than the cause.

**Now stated rather than emergent.** `updater/src/orphan.rs` refuses a candidate that lacks a binary
some installed unit execs, and the refusal names the unit, the missing binary and the way past it —
remove the unit, `systemctl disable --now configd.service && rm /etc/systemd/system/configd.service`.
There is no override flag: removing the unit is what the operator means anyway, since a board below
the release that introduced a daemon should not be running that daemon, and the next update that
ships the unit reinstalls it.

Two things about where it runs. It is **not** in preflight, which cannot see the candidate's file
list — both preflight passes run before the artifact is downloaded — so it runs after extraction and
before the swap, where staging is still disposable and nothing is armed. And **no target is exempt**,
unlike `Error::WouldDowngrade`, which fires on `Latest` alone: that guard is about a mirror serving a
stale manifest, this one is about a unit that will not start, and `Ref` is precisely how it was
observed.

It does not run on rollback, reset-to-golden or `select`. Those move backwards on purpose and are how
a board gets off a bad release, so a check that can refuse must not sit in the recovery path
(`docs/design/architecture.md` §1.1) — rolling back onto an orphaned unit stays the documented
behaviour above, and stays self-correcting.

## Why the existing tests could not have caught them

Not an accusation of the tests; they cover what they claim. The point is what nothing covers.

This was written when the answer was "nothing", and the row that says so has since changed. Kept as
the record of what was missing, with what each check covers **today**:

| | covers | does not |
|---|---|---|
| `board-test.sh` | binaries executed on real aarch64 Linux, **and a real artifact unpacked and installed** by `install.sh` and `hooks/postinstall` against a stubbed `systemctl` — placements, modes, ordering, idempotence, and every unit's `ExecStart` binary being present | services actually starting, which needs real systemd |
| `xtask` tests | the workflow YAML vs `install.sh`, and unit `ExecStart` vs staged binaries | whether the *built artifact* matches either — it reads source files |
| `updater` tests | engine, journal, verification, rollback, with fakes | `install.sh` at all |
| `shipped_config_is_safe_for_a_client_robot` | `deploy/updater.toml`'s content | that the config is installable |

What the first row used to say was "unpacking an artifact, installing units, starting services", and
the sentence beneath it was: **no test takes a real artifact and installs it.** That was the finding
this document exists for, and it is fixed — the `xtask` row is still the weaker form, asserting that
two source files agree rather than observing what they produce, but it is no longer the only thing
standing between a packaging mistake and a board.

## What would close it

Revised 2026-08-11, after the restart mechanisms below landed and with one constraint that was not
stated the first time: **CI is already the slowest part of iterating, so the plan is judged on what it
adds to the wait, not only on what it covers.** Everything here therefore names where it runs.

### The budget, first

CI runs on every push and every pull request that touches code, as parallel jobs, so the wait for
green is the *slowest* job — not the total. That single fact decides where new tests belong:

| job | what makes it slow |
|---|---|
| `check` | fmt, clippy, `cargo test --workspace`, the installer lint, and a real `xtask package` |
| `board` | `cargo install cargo-zigbuild` from source, plus QEMU emulation for aarch64 |
| `coverage` | a full instrumented build |

So a millisecond-scale test added to `cargo test` costs nothing anybody notices, while anything that
lands in `board` or `coverage` is paid on every push. Three rules follow, and they are the point of
this section:

- **The default `cargo test --workspace` takes only in-process tests.** No tarballs, no `systemd`, no
  network, no sleeps.
- **Anything that unpacks an artifact or drives a service runs on demand**, as a script or an
  `#[ignore]`d test — never on the pull-request path.
- **No new CI job.** If a check needs an artifact, it hangs off the `xtask package` step `check`
  already runs, and reuses the tarball that step already built.

Both of those numbers used to be worse, and the two removals that fixed them are the shape this
section argues for — **taking work out of CI is worth more than any test below adds**:

- **`coverage` ran the whole instrumented suite twice** on a pull request, head and base, purely to
  print a delta. Removed: `--fail-under-lines` is what catches a regression, and this job went from
  over seven minutes to about two. The cost is stated rather than glossed — a change that lowers
  coverage while staying over the floor no longer says so, and the floor is a ratchet now.
- **A documentation-only change paid the whole bill.** `on:` had no path filter, so editing this file
  cross-compiled for aarch64 under QEMU and built the workspace under instrumentation. Three
  consecutive docs pull requests did exactly that while this plan was being written. Removed with a
  `paths-ignore` for `docs/**` and `*.md`.

  That one has a trap attached, checked rather than assumed: a skipped job reports **no status at
  all**, so filtering a check that is *required* for merge leaves docs pull requests permanently
  pending. It is safe here because `main` has no required status checks — the branch-protection API
  answers 403 on this plan. Turning protection on means revisiting it, and the shape then is a
  filtered job plus a no-op job of the same name.

### 1. Two tests that need no new machinery

In-process, in `cargo test`, and they cover the acting half of the two mechanisms that exist to make
an update self-healing — neither of which was observable by any test. Both are written; what follows
is why they were the first thing to do.

- **`systemd-run` is unobservable.** `schedule_deferred_restarts` hardcodes
  `Command::new("systemd-run")`, while `SYSTEMCTL` two functions away is a `const` precisely so
  `restart_tests` can substitute a stub script. Same treatment, and then assert what has never been
  asserted: `--on-active=5s`, one invocation per unit, both `updaterd` and `btd` named. The flag could
  be wrong today and every test would still pass.
- **`reconcile::check` is never called by a test**, only its pure `verdict_for`. It already takes
  `systemctl` as a parameter, and identities are read through `DUCK_RUNTIME_DIR` — a seam whose own
  comment says it exists so this is testable. Write identity files into a temp runtime directory and
  assert the four outcomes: stale is restarted, `updaterd` is reported and not restarted, a missing
  identity file is left alone, a failed restart reports itself.

### 2. The artifact install — done, and what it left

**This is no longer open, and the title of this document is no longer true.** `scripts/board-test.sh`
packages a real release from the `--include` list in `_build-release.yml`, unpacks it, and runs
`scripts/install.sh` *and* `hooks/postinstall` against it inside the container with a stubbed
`systemctl` (PR #47, 2026-08-07). Eleven assertions: units installed byte-identical at mode 644,
sysusers drop-ins, the `robotctl` symlink resolving through `current`, the journald drop-in,
`daemon-reload` before any `enable` and `configd` before `btd`, operator config files preserved,
idempotence on a second run, a unit `install.sh` does not recognise installed-but-not-started, and
postinstall reproducing the lot on its own.

That covers what this section used to ask for, and the "assert the artifact's contents" idea with it,
because the tarball is open by then. **Bug 2 is closed against the artifact rather than against the
YAML.**

One gap was left, and it is bug 3 — the one class of the four with no strong test. Nothing asked
whether the binary a unit `ExecStart`s is *in* the artifact that shipped the unit, which is exactly how
`btd.service` came to fail with `203/EXEC` on a board where the release looked complete. The
protection was `xtask/tests/artifact.rs`, comparing a workflow against `install.sh` — the
two-files-agree form criticised above. Now asked directly, three lines, in the job that already has
the tree unpacked and `current` pointing at it.

Only `ExecStart` paths inside the release are checked: a unit may deliberately exec out of the base —
the boot recovery net does, so that a broken release cannot break it — and requiring those to be
packaged would be wrong rather than strict.

**What is still genuinely missing here is nothing.** The remaining items are the two below, and they
cover different things rather than more of this one.

### 3. One scripted scenario on a real board

**On demand, before a promotion. Never in CI.** Depends on `scripts/dev-push.sh`, which builds here,
signs with the dev key and applies to a board in about a minute: install the previous release, apply
the new one, then assert every daemon's `/run/<service>/identity.json` names the new release, that
`robotctl health` is clean, and that the update log holds one success.

That one pass is the only thing that observes the transient timer actually firing, `RuntimeDirectory=`
behaving under `ProtectSystem=strict`, real unit states reaching `robotctl health`, and the startup
reconciliation closing the loop for real. For the timing-dependent parts a board is *higher* fidelity
than any container, and it is now cheaper than one.

### 4. Real systemd, locally, for failure injection only

`systemd-nspawn` on a Linux box — **not** a privileged container in CI. After the three items above,
what is left is a short list of things nobody should ask a board to do repeatedly: a unit that exists
and will not start, `kill -9` between the swap and the commit, `systemd-run` itself failing,
`enable --now` on a unit whose `User=` does not exist. **Would have caught bug 1**, which is the only
one the items above miss — and it is also the only way to watch the postinstall hook do its real job
of enabling and starting a unit, which no stub can show.

Last for a reason beyond cost: it can only run on Linux, so never on the machine this is developed on,
and a test that runs somewhere else stops being read.

### Not doing

**A board as a self-hosted runner.** Ops cost, and a single point of failure for CI. Item 3 gets most
of the value on demand, which is where a robot in a room belongs.

## A second, separate problem: version skew on the dev channel

Found in the same session and easily confused with the above, but no amount of install testing
would catch either — in both cases the artifact was complete and correct.

The shape was always the same, and **the sentence that used to open this section is no longer true.**
It read: "`updaterd` never restarts itself during an update (§4.1), so it keeps running the old binary
until the next reboot, while everything else on the box moves to the new release immediately." That was
the root cause of both instances below, and it is fixed — `updaterd` now restarts itself five seconds
after the update replies. The skew window is seconds, not "until someone reboots".

Read the two cases with that in mind. Neither is a live incident any more, and **both are now
closed** — case 1 by fixing what it proposed, case 2 by deciding against it:

- Case 1's `serde(default)` fix was **not** made redundant by the shorter window: version skew is
  only the commonest way to hit it, and any newer-parser-older-sender pair does. Done, on the
  sections where a defaulted zero is honest.
- Case 2's handshake fix was **mostly** made redundant by it, dropping from "the recovery command
  itself stops working" to "a five-second window during which it does" — and when the design
  argument was finally written out, the premise it rested on did not survive. The exact `!=` stays;
  what changed is the message. See below and #77.

Two instances, both real, both cost about an hour.

### 1. A required field added to `HealthResult`

`main` added `consecutive_stale_blocks` to the IMU section and released 0.2.0 from it. A branch that
had merged `main` *before* that did not send the field. `robotd` came up perfectly — served the
socket, loaded both policies, ran the loop at 50 Hz — and the resident 0.2.0 `updaterd` rejected its
reply for a missing field. `RobotIo::health` maps an unparseable answer to `Health::Unreachable`, so
the gate reported `not healthy within 30s: unreachable` about a robot that was entirely healthy, and
reverted a good release.

Fixes, both small, and **both done**:

- **`#[serde(default)]` on new `HealthResult` fields**, so a newer `updaterd` can still parse an
  older `robotd`. Every `--ref` install of a branch predating a health-field addition hit this
  otherwise, which is the entire dev workflow.

  The half that bit was the nested one: every *top-level* field carried `#[serde(default)]`
  already, including `imu`, while `consecutive_stale_blocks` on the nested `ImuHealth` did not.
  `ImuHealth` and `BusHealth` now carry it at the container level, so a field added to either
  defaults rather than failing the whole parse.

  `LoopHealth`, `Battery` and `MotorThermal` deliberately do **not**, and the exception is the
  useful part of the fix: those sections carry *measurements*, where a defaulted zero is a lie an
  older sender never told — a defaulted `percent: 0.0` renders as a flat pack on a robot with a
  full one. The rule is "default what an omission honestly means", not "default everything", and
  `ImuHealth`'s doc comment argues it field by field.
- **A distinct `Health::Incompatible`** rather than reusing `Unreachable`, so the reason reads
  "answered in a shape this updaterd does not understand" instead of implying the robot is down.
  Pure diagnostics, and it would have found the above in a minute.

  Done. It also turned up its own neighbour while being added: `safe_to_restart` was collapsing an
  unreadable reply the same way, and `permits_restart` then read it as *safe* — the opposite of what
  its own comment promised. That became `SafeToRestart::Incompatible` and #68.

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

The proposed fix was **`hello` should refuse only when the client is *newer* than the daemon** — a v3
daemon serves a v2 client perfectly when v3 only *added* methods, so refusing that direction costs
the ability to recover and buys nothing. It would also have made `API_VERSION` mean "the newest
contract I understand" rather than "the only contract I will speak".

**Decided against** (#77), because writing the argument out exposed the premise underneath it: that
bumps are additive. They are not, and the constant does not distinguish them — v5 added `pad.*` and
was additive, v4 made `system.authenticate` mandatory and was not. Accepting older clients would
promise backward compatibility on every past and future bump, with nothing to enforce it and no way
to make a non-additive change afterwards. With one user and one robot, the freedom to change the
wire shape is worth more than a promise the protocol cannot keep. `API_VERSION`'s doc comment now
states that outright, which is the part that was genuinely missing: the rule existed only as an
`!=` in one file.

What did change is the message, because the remaining cost was never leniency — it was that
`client speaks API v2, daemon speaks v3` names no way out, on a board where the two halves are a
symlink and a running process. The refusal now branches on direction: client newer is the seconds
after an update, so retry and then `systemctl restart updaterd`; client older cannot happen through
`/usr/local/bin/robotctl`, which is a symlink into `current`, so the answer is to use that one.
`robotctl` correspondingly stopped appending "install matching versions", which was true and not
actionable.

### Why these are not install-path bugs

Nothing in the plan above would have caught either. The artifact was correct both times; what was
wrong was the *pair* of versions running at one moment, which only exists on a machine that has been
updated. Item 4 (real systemd locally) would catch the *health-gate* consequence of case 1 if the
container also ran an older `updaterd`, but constructing that skew deliberately is a different kind of
test — closer to a compatibility matrix than to an install test, and worth keeping separate.
