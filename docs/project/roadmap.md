# Roadmap

Status: draft · Date: 2026-08-05, revised 2026-08-26 · Owner: pierre

Companion to [`architecture.md`](../design/architecture.md) (what we're building) and
[`updater-design.md`](../design/updater-design.md) (how it ships). This is *order and sequencing*
— it will change; the design docs shouldn't.

## Where we are

| | |
|---|---|
| `updater/` | engine, verification, store, journal, hooks, preflight, GitHub/HF/local sources, IPC server, systemd unit — **done** |
| `duck-control/` | robot model · bus · IMU · `RobotIo` · observations · ONNX policy · safety — **slices 1–2 done, and run on a robot**. A library: no tokio, no sockets, no systemd |
| `duck-ipc-proto/` | wire contract for `update.*` and `robot.*` — **done**; serde/serde_json/semver only, so nothing on the recovery path pulls the engine's tree |
| `robotd/` | a real 50 Hz loop driving walk/stand through the safety layer, intents, health from deadline adherence and policy state — **slices 1–2 done, and it walks on a board**. Since then: kinematics, contact odometry, the voice, the ToF theremin and the chorale, all hung off the same tick ([`robotd-design.md`](../design/robotd-design.md) §4.4–4.5) |
| `padd/` | gamepad → intents, as an ordinary socket client — **done**, ships in the release and runs as its own unit from boot, so pairing a pad is the only step; needs libudev, installed by CI and the board cross-build |
| `robotctl/` | the operator CLI — `update`, `health`, `version`, `monitor`, `net`, `system`, `robot`, `pad`, `configure`, `quack`, `chorale`, `theremin`, `completions`; depends on `duck-ipc-proto`, not `updater`, so it stays on the recovery path |
| `xtask/` | package · sign · promote — **done**, byte-identical promotion verified |
| `.github/` | ci · release · promote — **all three run for real**: `0.2.0` was tagged to staging, verified through the engine, installed on a board and promoted to stable on 2026-08-05, byte-identical (§16.3) |
| bootstrap | `updaterd install` + `scripts/install.sh` — a robot installs its first release through the **ordinary engine**, so there is no bootstrap-only code path to drift |
| `deploy/` | shipped `updater.toml`, `robotd.toml`, trust anchor, journald retention drop-in |
| `scripts/` | `install.sh` provisioning · `board-test.sh` — **passing in CI**: 13 checks on emulated aarch64, Debian 13 (Trixie) |
| `btd/` | BLE transport adapter — framing, the routed subset, the BlueZ backend, a pairing agent. **Works on hardware**, unencrypted by default — the blocker, [`app-path-design.md`](../design/app-path-design.md) §5.5 |
| `duckctl/` | The robot from a laptop. BLE today; named for the robot rather than the radio, because `mediad` is a second transport |
| `configd/` | wifi over NetworkManager, robot name and the identity it derives from the SoC serial, pairing PIN, reboot. **Drives a real NetworkManager on a board**: provisioned over BLE, joined, and rejoined by itself after a reboot. `--fake-net` still serves the whole surface off-board |
| `mediad/` | camera, mic, encode and the WebRTC gateway, plus the console it serves. **Streaming to a browser on the LAN from a Radxa Zero 3W**, hardware H.264 through `mpph264enc`, with a `control` datachannel alongside — [`remote-webrtc.md`](../design/remote-webrtc.md) §0. Ships in the release and runs as its own unit |
| `tof/` | `tofd`: the head's 8×8 ToF matrix, published on its own socket at 15 Hz. Read by `robotd`'s theremin and drawn by `robotctl monitor`; a board with no sensor fitted runs it anyway and says so |
| tests | **936 passing** on a Mac (`--exclude tof`), a few more on Linux — including the health gate, the battery+thermal readout and the policy/safety path against a real `robotd` process, and `configd`'s authorisation over real sockets in `board-test.sh` |
| missing | the app, the SDK, and reaching a robot from outside the LAN |
| on hardware | walking through the intent API, the update path (install · health gate · commit · auto-rollback), a signed release installed from the stable channel, BLE provisioning of wifi. The loop held 50.0 Hz with `missed=3` in 15022 ticks before inference |
| not on hardware | the numbers M4 exists for: thermals, eMMC write timing, battery under load, and whether logs survive a power cut. The 30s health-gate timeout is still a guess |

## The framing

**The hard part is productisation, not capability.** `microduck_runtime` already walks,
runs gait policies, does perception and mapping. What doesn't exist is a robot you can
hand to a stranger: app-driven updates, safety authority, privacy, provisioning,
recovery. Porting existing capability into the new architecture is laborious but
*known-feasible*; the unknowns are all on the productisation side.

~~**The updater is finished and instrumentally useless.**~~ It had nothing real to ship, which
was the whole argument for the ordering below. That is now over: `0.2.0` ships a robot that
walks, and the update path is how it got onto a board.

## What changed the order: the team arrives (written 2026-07-28)

Others will work on `robotd`/`mediad` and **share builds through the updater**. That
makes two things urgent that would otherwise have waited:

1. **Dev-channel installs** — install a specific branch or commit on a board, without
   cutting a release. This is now ahead of `btd`: teammates will use `robotctl`, not the
   phone app.
2. ~~**A repo and a dev signing key**~~ — **done.** `pollen-robotics/microduck`
   (private), CI green on first fix; `team.dev` key generated. The signing secrets and the
   `release` environment are in place too, and `0.2.0` went out through them.

~~`btd` and the app path slip behind both.~~ Both landed early anyway — see M6 for why the
trigger turned out not to be the phone app.

## Milestones

Each has a test that says "done", because milestones without one drift.

### M1 — Close the loop  ·  **done**

The updater got something real to gate against, and the team got a shared crate boundary.

- **`robotd` skeleton** — heartbeat plus the four `robot.*` methods `updaterd` calls. Its
  state is atomics, not a mutex: a robot whose control loop is wedged must still be able to
  answer "I am not healthy", and needing the loop's lock to answer would hang in exactly the
  case that matters. `--unhealthy` / `--busy` exercise rollback on a bench robot.
- **`duck-ipc-proto` extracted** — `robotd` and `robotctl` depend on it and not on `updater`,
  so nothing on the recovery path links the engine's http/tar/crypto tree.
- **The health gate is real** — `on_apply` restarts `robotd`, `health` is a socket probe, and
  a test fails if either regresses to its inert bootstrap value.
- **One source of truth for the robotd socket** — `robot_socket` at the top level of the
  config; `--robot-socket` is a documented dev override.
- **Logging and version reporting** — every daemon's first line is its own identity (version,
  revision, exe path) at `warn`, so it survives `RUST_LOG=warn`; `robotctl version` reports
  running *and* installed per service, because `updaterd` never restarts itself and so
  legitimately lags until reboot.
- **First-install bootstrap** — `updaterd install` + `scripts/install.sh`, through the
  ordinary engine.

**Done:** `robotd/tests/updater_gate.rs` gates an update against a real `robotd` process over
a real socket and commits; `robotd --unhealthy` reverts the content behind `current`.

`robot-config` was dropped from this milestone: M1's test is about the health gate, and a
heartbeat daemon needs CLI flags, not a shared config store. It lands when something reads it.

### M2 — Dev channel  ·  **done**

Install a branch on a board without cutting a release:

```
sudo robotctl update apply daemon --ref my-branch
```

- **`Target::Ref`** and `manifest_at_ref` on the source trait. `--ref` conflicts with
  `--version` rather than one silently winning.
- **`dev.yml`** — every branch push publishes `<crate>-dev.<run>.<sha7>` to the moving tag
  `daemon-dev-<branch>`, signed with `team.dev`.
- **`xtask package` accepts a prerelease of the crate version** without
  `--allow-version-drift`, so the escape hatch stays reserved for what it was built for.
- **Refs work on `local_dir`** too, which makes the path testable offline and is the sideload
  story. A ref becomes a filename there, so separators and `..` are refused.

Two properties make this safe on every push, both enforced away from the workflow:

- A dev build **cannot become `latest`** — the version is a semver prerelease, and
  `version_under` refuses to read a dev tag as a release version.
- A dev build **cannot install on a customer robot** — `allow_dev_keys` is false there, and a
  trusted key only counts as a dev key if its filename ends `.dev.pub`.

A ref bypasses the downgrade guard by design: a prerelease always sorts below the release a
board is on, so guarding it would refuse every branch install. A plain `apply` returns the
board to the release stream, since `latest` resolves to the highest *stable* version.

- **`apply --from <dir>`** and `scripts/dev-push.sh` — build on a laptop, install on a board, no
  push and no CI run. A per-call source override rather than a config edit, so the board keeps
  reaching GitHub for `--ref`, `--staging` and a return to the release stream. It is an ordinary
  apply: health gate, auto-rollback, dev-key verification. `API_VERSION` moved with it, because a
  daemon one version older would have parsed the option, ignored it, and installed from its
  configured source while reporting success. v7 first ships in **0.5.0**, so each board takes one
  `scripts/dev-push.sh --bootstrap` to get there — the binary that would gate the update is the one
  being replaced.

**Done:** verified against the real repository — `dev.yml` published, `--ref main` installed
over the network, and a customer-robot config refused the same build.

**Open, and it blocks M4:** a private repo's release assets need a token, and a customer robot
has none. See `updater-design.md` §6.1.

### M3 — `robotd` for real  ·  **done**, in two slices

Designed in [`robotd-design.md`](../design/robotd-design.md). `robotd` **replaces**
`microduck_runtime`, by extracting its control core into `duck-control` rather than
reimplementing it — so the prototype keeps running while the daemon grows, and parity
arrives as a consequence of the extraction instead of as a race against a moving target.
Only the alpha variant on the Radxa survives; the other three variants, four IMUs and two
boards are dropped.

**Hardware first, sim after.** An earlier draft of this milestone said the reverse. It was
wrong on the facts: there are boards, and correctness gets settled on them. The simulator's
job is a clean laptop dev environment, not a validation oracle, so it lands after slice 2
and never becomes a second definition of what the robot is. Tests run against a `FakeIo`
backend — no hardware, no network, no Docker, no Python.

**Slice 1 — hold the pose · done, on a board.** A real 50 Hz loop on the Dynamixel
bus, holding whatever pose it starts in. No policy. It exists so `robot.health` means *the
loop is meeting its deadline* rather than *it ticked once* — until now the updater's
auto-rollback has been gating on a placeholder. Holding a pose is also what makes it safe to
hammer install/rollback/power-cut cycles at a bench for a day.

**Slice 2 — walk and stand.** One 61-D observation builder (every alpha policy is
`obs[1,61] → actions[1,14]`), the main-plus-standing policy shaped as it is in the runtime,
`move`/`head`/`stop`/`enable` intents, and a gamepad client that goes through them.

**Safety authority belongs here, not in M6** — `architecture.md` §6 designs it and nothing
implements it. It lands in slice 2, holding the only write handle to the bus, so no policy
and no client *can* command a motor around it. Joint clamp, fall → limp, and an intent
deadman; thermal waits for a measured threshold rather than a guessed one.

**Done:** all three, on a Radxa Zero 3W. It walks driven through the intent API; an update
applied with `robotctl` restarts it cleanly with the gate passing; and a release that comes up
unhealthy reverts on its own. The board also produced the one bug the tests could not: `ort`
*panics* rather than erroring on a runtime below its floor, which killed the control thread and
made health blame the wrong thing. It now holds the pose and names the version instead.

### M4 — Hardware bring-up  ·  in progress

M3 on the Radxa with real motors and IMU. This is where the genuinely unknown numbers
appear: control-loop jitter on a non-RT kernel, ONNX inference rate on Cortex-A55, eMMC
write timing, thermals, battery. Also the first real test of `systemctl restart` in
`on_apply`, and of the health-gate timeouts — 30s is currently a guess.

**Settled already**, because slices 1 and 2 could not be finished without them: the loop holds
its rate on a non-RT kernel (50.0 Hz, `missed=3` in 15022 ticks, before inference), the bus and
the `imu_to_dxl` board answer on `/dev/ttyS2`, `systemctl restart` in `on_apply` works against
real systemd, and the gate commits and reverts for real. **Still open, and they are the reason
this milestone is not closed:** thermals, eMMC write timing, battery under load, whether the 30s
timeout has any margin on a cold boot, and the log-retention question below.

Also the first chance to settle the **log retention** question, which cannot be answered
off-board (`deploy/README.md`):

- `findmnt /var/log` — if Armbian's RAM-log has it on tmpfs, journald's `Storage=persistent`
  is a directory in memory: it survives a clean `reboot` and loses recent logs on a power
  cut, which is how a robot is actually switched off. Decide explicitly: disable the RAM log
  and accept eMMC writes, or keep it and rely on the update history (which does not go
  through `/var/log`).
- `journalctl --list-boots` after a real reboot — two or more entries, or the drop-in is not
  doing what it claims.

**Done when:** it walks on hardware, an update applied via `robotctl` restarts `robotd`
cleanly with the gate passing, and `journalctl -u robotd -b -1` returns the previous boot's
logs after a power cut.

### M5 — `mediad`, WebRTC, SDK  ·  in progress

Camera/mic, encode, perception, the remote gateway. Privacy lands here and not later:
per-session consent and a visible streaming indicator are cheap now and expensive to
bolt on. The SDK's WebSocket + snapshot path (§5.3) is what makes "an LLM drives the
robot" easy.

**Landed, on hardware.** `mediad` ships in the release and runs as its own unit. The camera
reaches a browser on the LAN through the VPU — `mpph264enc` → `webrtcsink`, constrained baseline —
with a `control` datachannel carrying the same JSON-RPC every other transport speaks, and the
console is served by the robot itself so there is a URL and nothing to install
([`remote-webrtc.md`](../design/remote-webrtc.md) §0, [`webrtc-console.md`](../design/webrtc-console.md)).
Two GStreamer plugins had to be built from source to get there, which is its own record
([`media-bringup.md`](media-bringup.md)), and the camera has since got 3A through `rkaiq`.

**Still open, and they are what keeps this milestone from closing:** reaching a robot from outside
the LAN — the design is the same one with a proxy in front (§7), deliberately built second — the
SDK, and the privacy pair. Consent and the streaming indicator are *not* done, and this is the
milestone that was supposed to stop them being bolted on, and the reason given for deferring them
is a hardware one — an LED under software control, which does not yet exist
([`remote-webrtc.md`](../design/remote-webrtc.md) §11).

**Done when:** telepresence works from outside the LAN, and a server-side script can
fetch a frame and send an intent in a few dozen lines.

### M6 — Ship readiness

`btd` + the app update path, provisioning (device identity, calibration, key
installation), recovery mode (§8.2's last link), manifest staleness reporting (§8.4.2),
and the authority arbitration finished.

**`btd` and `configd` landed early**, out of this order. The trigger was wanting to configure a
robot — wifi, name, reboot — from something other than an SSH session, and the work turned out to
be mostly *not* Bluetooth: an API surface and a service to own it, which the phone app, the SDK,
`robotctl` and `mediad`'s gateway all need identically. `btd` is a thin pipe over it.

What that leaves for M6 proper: recovery mode, manifest staleness, authority arbitration, and the
provisioning step the pairing PIN now depends on (below). It also means the app has something to
talk to before the app exists, which is the right order for finding out that an API is wrong.

**The update path over Bluetooth is now driven rather than merely routed**, which is what that
order was for: `update-over-ble.md` records what driving it from `duckctl` turned up, including
one defect that made "start an update and watch it" an update the robot silently never performed.
`update.rollback` and `update.select` are reachable from a phone as of that work.

**Done when:** a non-developer updates the robot from the phone, and a deliberately
bricked release recovers without a laptop.

## Organisation

**One repo, one workspace.** `robotd`, `btd`, `configd`, `padd`, `mediad` and `tofd` are all
siblings now — nothing is outstanding. They co-version because they all ship in the same
`daemon` artifact — one version line is correct, and models version separately already.

**Crate layout as it should end up:**

It ended up wider than this section first drew it — the libraries `robotd` drives are their own
crates now, for the reason `duck-control` was: the compiler is what keeps daemon concerns out of
them. The list lives in [CONTRIBUTING.md](../../CONTRIBUTING.md#the-layout) rather than here,
because it is reference and this page is a record. The shape:

```
duck-ipc-proto/ wire types — serde/serde_json/semver only; btd/robotd/robotctl depend
                on this, never on updater
updater/        engine + updaterd
robotd/         control, gait, safety, the voice, the theremin, the chorale
duck-control/   the control core, extracted from the prototype runtime
kinematics/  odometry/  sounds/  pet-detect/  robotd-params/
                libraries robotd drives — no sockets, nothing systemd starts
configd/        wifi (NetworkManager), robot name, pairing PIN, reboot, gamepad pairing (BlueZ)
padd/           gamepad → intents; a client, with no privilege the app will not have
mediad/         camera, encode, WebRTC gateway, and the console it serves
tof/            tofd — the head's depth sensor, published on its own socket
btd/            BLE transport adapter
robotctl/       CLI
duckctl/        the client, from a laptop (dev tool, never shipped)
xtask/          build/publish tooling — never ships
```

**Docs per concern, not per service.** `architecture.md` is the cross-cutting contract;
a service gets its own design doc only when it earns one (`updater-design.md` is the
model). Resist one giant document.

**Two channels of work for newcomers.** A teammate on `robotd` should be able to: clone,
`cargo test`, run against sim, push a branch, and install it on a board via `--ref`.
That's the whole onboarding path, and M1+M2 are exactly what make it true.

## Decisions that shape work rather than follow it

1. ~~**Signing key custody**~~ — **done.** Three encrypted release keys plus an
   unencrypted dev key in `~/.duck-keys`, all round-trip verified; only `release-1` goes into
   secrets. Releases are signed in CI under `environment: release`, which on this plan scopes
   the secrets but **gates nothing** — no required reviewers, no branch policy, so anyone with
   push access can reach the key. Accepted deliberately while no robot is in the field, and the
   declaration is the hook that turns a real gate on with one settings change. See
   [`ci-setup.md`](ci-setup.md).
2. **Safety authority** (§6) — pulled into M3 for the reason above.
3. **Provisioning** — still needed, but for less than it was. Identity no longer waits on it:
   a robot derives one from its own SoC serial and names itself `duck-c51b`, so a board flashed
   by hand is distinguishable out of the box ([`app-path-design.md`](../design/app-path-design.md)
   §8.2). What is left is calibration and the PIN — BLE pairing security *rests* on a per-robot
   PIN, the factory default is `000000` and public in this repository, so out of the box pairing
   proves physical presence and nothing more. Something has to generate one, print it, and record
   what was printed. Note the PIN cannot come from the identity, which was the plan: the identity
   is published in an advertisement, so anything derived from it is public too.
4. **Privacy** — consent + indicator in M5, not M6.

## Not doing, on purpose

Recorded so they stay decided: A/B image updates, OS/kernel OTA, fleet
dashboards/telemetry, delta updates, staged rollouts, hardware capability matrix,
competing model alternatives per slot (§17), peripheral firmware OTA (§11.1).
