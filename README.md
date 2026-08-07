# microduck daemon

The software that runs on the robot, and the machinery that ships it there.

A bipedal robot goes to people who are not developers, so the hard problem is not making it
walk — that already works in the prototype — but making it something you can hand to a
stranger: updates it can apply itself without bricking, a safety authority that outranks
every client, and recovery that works when the robot is already broken.

This repo is the daemons plus the update system. Start with
[`docs/architecture.md`](docs/architecture.md) for how the services fit together, and
[`docs/roadmap.md`](docs/roadmap.md) for what exists today versus what is designed.

If you have a board in front of you and want the commands,
[`docs/cheatsheet.md`](docs/cheatsheet.md) is `robotctl` and `btctl` on one page — including the
three things about restarts after an update that otherwise cost an afternoon.

For the control side specifically, [`docs/robotd-design.md`](docs/robotd-design.md) §3.1 is
the fastest way in — who talks to `robotd` and where the crate boundary sits — with the
per-tick dataflow in §5.10 and the thread-to-thread channels in §7.1. Those three diagrams
are the part that is hardest to reconstruct from prose.

## Getting started

Needs Rust **1.89+** (stable) and nothing else. macOS and Linux both work for development;
the robot is aarch64 Linux.

```bash
cargo test --workspace
```

352 tests, no hardware, no network, no Docker. If they pass, your checkout is sound.

Those tests are also where the engine's failure paths are: a bad signature, a release that
comes up unhealthy, a post-install hook that fails, power loss between the swap and the
health gate. Each drives the real engine with the fault injected rather than a mock of it, so
`updater/tests/apply.rs` is the honest answer to "what does this actually guarantee" — more
so than anything you could run by hand here.

Using the updater for real needs a board. Provisioning one from nothing is one command from
this clone — `./scripts/provision-board.sh [user@]host`, described in
[`deploy/README.md`](deploy/README.md). Everything you do to it afterwards is
[Working on the robot](#working-on-the-robot) below.

## The services

| | |
|---|---|
| `robotd` | motor control, gait policy, **safety authority**. A real 50 Hz loop driving walk/stand through a safety layer that holds the only write handle, plus intents and the four `robot.*` methods the updater needs. **Never run on a robot** ([`docs/robotd-design.md`](docs/robotd-design.md)). |
| `duck-control` | the control core — robot model, bus, sensing, observations, ONNX policy, safety. A library, not a service: no tokio, no sockets, no systemd. |
| `padd` | a gamepad, as an ordinary intent client. No privileged access; it sends what the app and SDK will send. |
| `updaterd` | the update engine. Resident, and deliberately independent of `robotd` — it is the recovery path, so it must work when the robot does not. |
| `mediad` | camera, audio, WebRTC gateway. **Not built yet.** |
| `btd` | BLE: the phone's front door. A pipe carrying the same JSON-RPC lines as every other transport, over one GATT characteristic. Owns no state. **Works on hardware**: a Mac discovers the robot, pairs, authenticates with the PIN and provisions wifi over BLE. **Unencrypted by default** — deliberate while requiring pairing makes every client hang, and the blocker before anything ships: [`docs/app-path-design.md`](docs/app-path-design.md) §5.5. |
| `configd` | wifi (via NetworkManager), robot name, pairing PIN, reboot. Its own service because config must be reachable when `robotd` is dead, and because `btd` owns nothing. **Drives a real NetworkManager on a board**: a network provisioned over BLE is joined and survives a reboot. |

They talk over unix sockets, JSON-RPC 2.0 one object per line. The contract lives in
`duck-ipc-proto`, which depends on serde and semver and nothing else — so `btd` and `robotd`
never inherit the update engine's http/tar/crypto tree.

```
duck-ipc-proto/ the wire contract
duck-control/   the control core: model · bus · IMU · observations · policy · safety
padd/           gamepad → intents — an ordinary socket client, no privileged access
updater/        engine + updaterd
robotd/         control daemon
configd/        wifi · robot name · pairing PIN · reboot
btd/            the BLE front door, plus btctl (a laptop client, never shipped)
robotctl/       the local CLI
xtask/          package · sign · promote — build tooling, never shipped
deploy/         what a robot is configured with: updater.toml, robotd.toml, trust anchor, journald
scripts/        provision-board.sh (from your machine) · provision.sh → setup-board.sh ·
                migrate-network.sh · install.sh (on the board) · board-test.sh (CI)
docs/           architecture · update design · robotd design · app path · roadmap · CI setup · cheat sheet
```

## Working on the robot

Everything below assumes a **dev board**, never a customer robot.

The state of the robot, hardware and software, in one answer — control loop, motor bus, IMU,
battery, servo and board temperatures, then what is running, what is installed, what is
pinned and how the last update went:

```bash
robotctl health
```

```
robot     healthy
  loop      50.1 of 50.0 Hz · 2834 ticks · 0 missed · last 13 ms ago
  bus       ok
  imu       ready
  battery   7.62 V (64%)
  motors    41 °C max (left_knee) · 36 °C mean
  cpu       52 °C

software
  updaterd  0.1.4 (rev abc1234)
  robotd    0.1.5 (rev def5678)
  daemon    0.1.5 installed
            last update 0.1.4 → 0.1.5: applied
```

It exits non-zero when the robot is unhealthy or unreachable, so it can gate a script.
Nothing else there affects the exit code: a flat pack, a hot motor and a pinned component are
reported, not judged — a release must never be rolled back for the state of the board it
landed on.

`version` is the software half on its own, for when that is all you want. What is running and
what is installed are different questions, because `updaterd` never restarts itself during an
update and so legitimately lags the installed release until the next reboot:

```bash
robotctl version
```

```bash
robotctl update status
```

```bash
sudo robotctl update apply daemon
```

```bash
sudo robotctl update rollback daemon
```

Logs go to the journal. The startup line carries version, git revision and the release
directory the process was launched from, at `warn`, so it survives any log level:

```bash
journalctl -u robotd -b
```

Logs say what happened; `monitor` says what is happening. It shows what a client asked for
next to what was actually applied, and names the reason when they differ — safety clamps
things constantly, and "the stick is forward and the robot is still" is unreadable without
that:

```bash
robotctl monitor
```

On a terminal that is a single frame repainting in place: the velocity twist a client asked
for beside what safety applied — one labelled row per axis, with its direction and unit, so
nothing has to be decoded — the IMU's projected gravity and the fall verdict drawn from it,
every joint measured against what it was commanded, and the achieved loop rate as a trace so
a stutter that has already recovered is still visible. A limit is spelled out rather than
named: `deadman — no intent arrived recently, velocity zeroed`, not `deadman`. `q` quits;
`↑`/`↓` scroll the joint list on a window too short for all of it.

Projected gravity is the only IMU quantity on this stream, and it is what `fallen` is decided
from — upright is about `[0, 0, -1]`. The stale-read counters and the rest of the sensing live
in `robotctl health`.

The bottom border names the policy that is loaded — the `.onnx` files, and whether a standing
network is configured at all — because `walk` is a mode two releases with different gaits both
report, and "which network is this?" is the first question when comparing them. A robot with no
policy says so, and one whose policy would not load says that instead, which the stream's
`held` cannot distinguish.

Redirected or piped it prints one line per tick instead — `robotctl monitor > run.log` and
`| grep FALLEN` behave as before. The joint vectors are in `--json`, which carries the whole
state, one object per line:

```bash
robotctl monitor --json --hz 50 > run.jsonl
```

The update history is separate from the journal on purpose — `fsync`ed per entry under
`/var/lib/robot/updater/` — so it survives a robot whose logs were volatile:

```bash
robotctl update log
```

`install.sh` sets up tab-completion for `robotctl` in `/etc/bash_completion.d/`, as a loader
that asks the binary for its own completions — so they follow the installed release instead
of going stale when an update adds a command. For a shell it did not cover, or for a build
you are running straight out of `target/`:

```bash
eval "$(robotctl completions bash)"
```

`zsh`, `fish`, `elvish` and `powershell` work in place of `bash`.

Provisioning a board from scratch, and the log-retention caveats on Armbian, are in
[`deploy/README.md`](deploy/README.md).

### Testing your branch on a board

A board provisioned for this in one command, from a clone — it sends your dev key, waits out the
reboot and streams the log:

```bash
./scripts/provision-board.sh radxa@192.168.1.42 --ref my-branch
```

Add `--local` to send this clone's `provision.sh` instead of fetching it, which is how to test a
change to the provisioning scripts themselves without merging first.

Then the ordinary loop. Push the branch; CI cross-compiles it, signs it with the team dev key,
and publishes a prerelease at the moving tag `daemon-dev-<branch>`. On the board:

```bash
sudo robotctl update apply daemon --ref my-branch
```

```bash
robotctl version
```

```bash
sudo robotctl update rollback daemon
```

No version numbers to copy: the tag moves to whatever that branch last built, while the
version inside (`0.1.0-dev.42.c719ec8`) stays unique per build so two builds are never
confusable. A plain `sudo robotctl update apply daemon` puts the board back on the release
stream, because a prerelease sorts below its release — there is no "leave the dev channel"
step.

Nothing is relaxed for a dev build: same signature and hash verification, same health gate,
same auto-rollback. The difference is the key, and that is what keeps these builds off
customer robots — they refuse a dev key twice over (`allow_dev_keys = false`, and a trusted
key only counts as a dev key if its filename ends `.dev.pub`).

**A board has to opt in once**, which is also what stops this working on a robot that
shouldn't take dev builds. `install.sh` does it, given the public half of the dev key:

```bash
sudo DUCK_TOKEN="$DUCK_TOKEN" DUCK_DEV_KEY=/tmp/team.dev.pub sh /tmp/install.sh
```

It validates the key and flips `allow_dev_keys` in one step, and installs the key under the
name `team.dev.pub` whatever the source file was called — the `.dev.` infix is what classifies
it, and a key landing under any other name is trusted as a *release* key. `team.dev.pub` is not
in the repository on purpose; get it from a team member or regenerate it with
`minisign -R -s ~/.duck-keys/team.dev.key -p team.dev.pub`.

By hand, for a board provisioned some other way — both halves, because either alone leaves a
board that still refuses branch builds with an error that reads like a corrupt release:

```bash
sudo cp team.dev.pub /etc/robot/trusted_keys/team.dev.pub
```

```bash
sudo sed -i 's/^allow_dev_keys.*/allow_dev_keys        = true/' /etc/robot/updater.toml
```

```bash
sudo systemctl restart updaterd
```

While this repository is **private**, the board also needs a GitHub token — a private repo's
release assets are unreachable without one, and `updaterd` reads `GITHUB_TOKEN` from its
environment, so exporting it in your shell does not reach the daemon.

`scripts/install.sh` writes this drop-in for you when given `DUCK_TOKEN`, and restarts
`updaterd` so the running process actually picks it up. The manual steps below are for a board
provisioned some other way:

```bash
sudo mkdir -p /etc/systemd/system/updaterd.service.d
```

Substitute your own token in the next block — it is the only placeholder here:

```bash
sudo tee /etc/systemd/system/updaterd.service.d/token.conf > /dev/null <<'EOF'
[Service]
Environment=GITHUB_TOKEN=ghp_replace_with_your_token
EOF
```

```bash
sudo chmod 600 /etc/systemd/system/updaterd.service.d/token.conf
```

`chmod 600` because a drop-in is world-readable by default, and this one holds a credential.

```bash
sudo systemctl daemon-reload && sudo systemctl restart updaterd
```

A token on a *developer's* board is fine. A token on a customer robot is not, and is why
artifact hosting is still an open question — see `docs/updater-design.md` §6.1.

### Switching between releases

What is on the board, and what it is doing. None of these need root:

```bash
robotctl version          # running vs installed, per daemon, with the git rev
robotctl update status    # per-component state, pin, last attempt
robotctl update check     # is a newer release available; changes nothing
robotctl update log       # recent attempts and their outcomes
```

Switching by downloading. These mutate the robot, so they are root-only by design:

```bash
sudo robotctl update apply daemon                    # the latest release
sudo robotctl update apply daemon --version 0.1.4    # one exact version
sudo robotctl update apply daemon --ref my-branch    # what a branch last built
sudo robotctl update apply daemon --dry-run          # verify, stop before the swap
```

Switching to something the board already has, without a download or a network:

```bash
sudo robotctl update select daemon 0.1.4      # activate an installed release
sudo robotctl update rollback daemon          # the previously installed one
sudo robotctl update reset-to-golden daemon   # the never-pruned known-good one
```

And refusing to move:

```bash
sudo robotctl update pin daemon 0.1.4    # accept nothing else
sudo robotctl update pin daemon          # unpin
```

Installing with no network at all — a factory or offline install, or sideloading a build you
carried in on a stick. This one is `updaterd` rather than `robotctl`, because it is also the
path a board takes *before* there is a daemon to ask, and `updaterd` is deliberately not on
`PATH`:

```bash
sudo /opt/robot/daemon/current/bin/updaterd install --from /media/usb/release
```

The directory holds what a release is: `<version>.manifest.json`, its `.minisig`, the artifact
and the artifact's `.minisig`. Signatures, hashes and compatibility are checked exactly as they
are for a download — `--from` changes where the bytes come from, not what is trusted.

That command refuses to run once a release is live, because it forces `on_apply` and the
health gate off, and doing that to a working robot would silently disable auto-rollback. One
situation needs it anyway, and `robotctl update apply` cannot help with it — a board whose
installed `updaterd` is too old to accept the release that *fixes* being too old. It rolls the
new release back every time, and the binary running that gate is the one being replaced. Stop
the robot and say so explicitly:

```bash
sudo systemctl stop robotd
```

```bash
sudo /opt/robot/daemon/current/bin/updaterd install --from /media/usb/release --force
```

`--force` is itself refused while `robotd` is still answering, since the objection is about a
*working* robot losing its safety net. It gives up auto-rollback for that one install and
nothing else — signatures, hashes and compatibility are still checked, and
`sudo robotctl update rollback daemon` is the recovery path if the release misbehaves.

Three things that are easy to get wrong.

**`rollback` needs a predecessor, but an update creates one.** A freshly provisioned board has
exactly one release, so `rollback` right then has nothing older to go to and says so. Auto-
rollback is *not* affected: applying a release unpacks it alongside the current one and only
then moves `current`, so by the time the health gate runs there are two, and the release you
came from is the target. `rollback_target` picks the highest installed version below `current`
that the journal has not already recorded as bad — so a board with one release is fully
protected from the moment it takes its first update.

The one genuinely unprotected install is the bootstrap itself, which has nothing before it by
definition. `golden` would cover that, and it is deliberately unset until 1.0.0 exists — so
`reset-to-golden` reports honestly that none is configured rather than doing something
surprising.

**`version` shows the live release per component, not the release store.** It will never list
two versions, however many are unpacked under `/opt/robot/daemon/releases/`. Ask the store
directly if you need to know what is there:

```bash
ls -l /opt/robot/daemon/releases/ /opt/robot/daemon/current
```

**`apply --version` needs the release to still exist upstream; `select` does not.** Releases
carrying known-bad builds get deleted from GitHub, so `apply --version 0.1.3` fails on
purpose, while `select 0.1.3` still works on a board that already unpacked it. The asymmetry
is deliberate: no new board can acquire a broken release, and a board that has one keeps its
escape hatch.

## Releasing

Releases are signed **in CI**, never locally, behind an approval gate. Cutting one is a
tag; promoting one re-signs a manifest over the *same bytes* the canary validated, with no
rebuild:

```bash
git tag daemon-staging-v0.2.0 && git push --tags
```

```bash
gh workflow run promote --field version=0.2.0
```

[`docs/ci-setup.md`](docs/ci-setup.md) covers key custody, the secrets, and rotation.

## Conventions

- **Comments say why, not what.** The reason a thing is the way it is outlives the code.
- **Every non-obvious decision gets a test**, and the test's comment says which failure it
  exists to prevent. The rollback paths especially: they only ever run when something else
  has already gone wrong, so they are the code most likely to be quietly broken.
- **Reach for an existing crate** before writing it yourself. Dependency count is not the
  thing being optimised; maintenance is.
- Commit trailers use `Assisted-by:`, not `Co-Authored-By:`, for AI assistance.

## Status

Honest version, kept current in [`docs/roadmap.md`](docs/roadmap.md):

- **Works and is tested:** the update engine end to end — verification, atomic swap, health
  gate, auto-rollback, boot-counter recovery, first-install bootstrap, release packaging and
  signing. Releases are cut and signed in CI; a real one has been published and installed
  through the engine.
- **The dev channel works.** Every branch push publishes a signed build installable with
  `--ref`, verified against the real repository, and refused by a customer-robot config.
- **Open:** artifact hosting. This repo is private, and a robot without a token cannot
  download from it (§6.1). Dev boards have tokens; the fleet will need a public
  artifact-only repository or an object store. Blocks hardware bring-up, not development.
- **`robotd` walks — in principle.** A real 50 Hz loop, one 61-D observation builder, the
  walk/stand policy pair, and a safety layer holding the only write handle. `robot.health`
  means *the loop is meeting its deadline and the policy loaded*, which is what makes
  auto-rollback gate on something real. **None of it has met a robot**: the tests prove the
  logic is self-consistent, not that it walks. Needs ONNX Runtime on the board, which
  `install.sh` now installs.
- **The app path works on hardware, without encryption.** A Mac discovers the robot, bonds, passes
  the PIN and gets real answers back over BLE — but `encrypt_read` hangs CoreBluetooth, so the link
  currently carries no encryption and the PIN travels in clear. That must close before anything
  ships (`docs/app-path-design.md` §5.5). `btd` serves a GATT pipe and `configd`
  serves `net.*`/`system.*` — wifi scan and join, robot name, pairing PIN, reboot — with
  `robotctl net`/`robotctl system` on the robot and `btctl` as a laptop-side BLE client.
  `configd --fake-net` serves the whole surface off-board. A board must be migrated from netplan
  to NetworkManager once (`scripts/migrate-network.sh`) before wifi works.
- **BLE pairing is a six-digit PIN**, default `000000` and therefore not a secret: out of the box
  it proves physical presence and nothing more. Per-robot PINs are a provisioning step that does
  not exist yet, and the security of the app path rests on it.
- **Not started:** `mediad`, the phone app, the SDK, safety authority.
- **Runs on aarch64 Linux, emulated.** `scripts/board-test.sh` runs in CI: it
  cross-compiles for the board and executes 13 checks — rollback, tampered-artifact
  refusal, boot-counter recovery, socket modes, peer-credential authorization — on
  Debian 13 (Trixie), the userland we ship. `BOARD_IMAGES=` runs it against another.
- **Never run on real hardware.** No board yet, so nothing here says anything about motor
  timing, control-loop jitter on a non-RT kernel, thermals or eMMC behaviour. Two specifics:
  `systemctl restart` in `on_apply` has never met real systemd (containers have none), and
  the 30s health-gate timeout is a guess until someone measures a real boot.
