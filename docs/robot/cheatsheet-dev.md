# Cheat sheet — dev board

The commands that only make sense on a board set up by
[`install-dev.md`](install-dev.md). Everything a robot needs day to day is in
[`cheatsheet.md`](cheatsheet.md).

## The dev channel

Install what a branch last built on CI:

```
sudo robotctl update apply --ref <branch> daemon
```

```
sudo robotctl update apply --ref main daemon
```

`--version` pins an exact release instead. Give one of them unless you genuinely mean "go to
stable".

**`apply daemon` with no `--ref` installs the latest *stable* release, which on a dev board is
usually a downgrade.** It is not "install the newest thing"; it is "install what the stable channel
offers". Right after a branch merges, that stable release is still older than everything you have
been testing — and if it predates a daemon that now has a unit file on the board, its `ExecStart`
points at a binary the older release does not contain, the restart fails, and the update rolls back.
That is the gate working, but the command that caused it looked like the obvious one.

The tag `daemon-dev-<branch>` moves with the branch, so there is no version number to copy. The
version *inside* stays unique per build — `0.1.0-dev.42.c719ec8` — so two builds of the same branch
are never confusable. `--ref main` is how a board goes back to mainline without leaving the dev
channel; a plain `apply daemon` leaves it, since a prerelease sorts below its release and there is
no separate opt-out step.

A merge does not publish instantly: CI has to build `main` before `--ref main` resolves to it.

```
gh run list --branch main
```

## Release candidates

What `release.yml` published to staging and nobody has promoted yet — what a canary robot should run
before a promotion:

```
sudo robotctl update apply --staging daemon
```

```
sudo robotctl update apply --staging --version 0.3.0 daemon
```

A candidate is signed with the release key like any release and carries the version it will be
promoted under. What makes it unreachable without the flag is that it is flagged as a prerelease, and
a plain `apply` skips those so no robot drifts onto a build nobody has validated. `--staging` is that
filter's only opt-in, it applies to the one command, and it leaves nothing switched on afterwards.

## After an update — the part that bites

Three things are true at once, and together they cost an afternoon if you do not know them:

- **`btd` is never restarted by an update.** Deliberate: restarting it drops the BLE connection
  carrying the update's own progress stream. So a `btd` fix needs a manual restart or a reboot.
- **`configd` used to be restarted only if the board's `/etc/robot/updater.toml` listed it** — that
  file belongs to the operator and is preserved across installs, so a board set up before `configd`
  existed kept `units = ["robotd"]` and silently ran the old binary. Fixed: the restart set now comes
  from the units the release ships. A board running an older `updaterd` still has the old behaviour
  until it restarts, because `updaterd` never restarts itself.
- **`robotctl update apply` then reports `already_current` and does nothing**, so the obvious
  recovery command is a no-op.

The symptom is a fix that is definitely installed and definitely not working. `robotctl version` says
so in as many words. To check by hand:

```
pgrep -a configd
```

```
sudo readlink /proc/$(pgrep -x configd)/exe; readlink /opt/robot/daemon/current
```

The command line always says `current/bin/configd` because that is what it was exec'd with; the
`/proc/<pid>/exe` link is the release it is *actually* running from. If those disagree, the process
predates the swap.

```
sudo systemctl restart configd
```

```
sudo systemctl restart btd
```

Editing the board's `updater.toml` is no longer needed — the restart set is derived from the release.
The one thing that still requires a manual step is `updaterd` itself, which never restarts itself, so
the fix above only takes effect once it has:

```
sudo systemctl restart updaterd
```

## From a laptop — build here, install on the board

No push, no CI run, no tag. One command from a clone of this repo:

```bash
scripts/dev-push.sh radxa@<board>
```

```bash
export DUCK_BOARD=radxa@<board>
```

```bash
scripts/dev-push.sh
```

It cross-compiles for the board, packages the same artifact a release does, signs it with the dev
key, copies it to `/var/tmp/duck-sideload` on the board and applies it there. Roughly a minute on
an incremental build, against several for a push plus a CI run.

Needs, once:

```bash
cargo install cargo-zigbuild --locked
```

```bash
brew install zig
```

and `team.dev.key` at `~/.duck-keys/team.dev.key` — the secret half of the key CI signs branch
builds with. Set `DUCK_DEV_SECRET_KEY` if yours lives elsewhere. The board must be a
[dev board](install-dev.md); the artifact is signed with a dev key, so a customer robot refuses
it exactly as it refuses `--ref`.

### Or build in a container, with no toolchain to set up

```bash
scripts/dev-push.sh --docker radxa@<board>
```

Needs Docker running and nothing else — no zig, no `cargo-zigbuild`, and no board to copy
libudev from, which is what makes it the answer before you have a board at all. It builds inside
Debian Bookworm on arm64, so on an Apple Silicon Mac the target is the host: nothing is
cross-compiled and libudev is just installed. Same artifact, same `--dry-run` and `--bootstrap`.

Slower to start — a first build compiles the workspace inside the container, and the two modes
keep separate `target/` directories, so switching costs one full rebuild. Reach for it when the
zig path is not set up or has broken; the default is faster day to day and is what CI uses.

Verify without installing:

```bash
scripts/dev-push.sh --dry-run radxa@<board>
```

**This is an ordinary update.** It goes through `robotctl update apply --from <dir>`, so the
signature, the artifact hash, compatibility, the health gate and auto-rollback all run — a build
that does not come up is reverted and the board is back on what it was running. The restart traps
above still apply: `btd` and `updaterd` keep running the old binary until they are restarted.

The version is `<crate>-dev.local.<epoch>.g<sha7>`, so `robotctl version` on the board says which
push it is running and two pushes of the same dirty tree never collide.

To install what is already in that directory, or to point at one you filled yourself:

```bash
sudo robotctl update apply daemon --from /var/tmp/duck-sideload
```

### The first push to a board that has never had one

`apply --from` is part of the release being pushed, so a board whose installed `updaterd` predates
it cannot be asked to use it — `robotctl` and `updaterd` report an API mismatch and refuse. Deliver
it once the ungated way, which stops `robotd` and gives up the health gate for that one install:

```bash
scripts/dev-push.sh --bootstrap radxa@<board>
```

Every push after that is the ordinary command.

## From a laptop — `btctl`

A deliberately small subset of the robot API over Bluetooth LE — a test stand-in for the phone app,
not a product. Built from a clone of this repo. It is an *example*, not a binary, which is why every
invocation says `--example`:

```
cargo run -q -p btd --example btctl -- --name <robot-name> info
```

Or install it once, at the cost of it being a snapshot that does not follow the branch:

```
cargo install --path btd --example btctl
```

```
btctl --name <robot-name> info
```

### Commands

```
btctl scan
```

```
btctl --name <robot-name> info
```

```
btctl --name <robot-name> status
```

```
btctl --name <robot-name> health
```

```
btctl --name <robot-name> wifi status
```

```
btctl --name <robot-name> wifi scan
```

```
btctl --name <robot-name> wifi connect <ssid> --psk <passphrase>
```

```
btctl --name <robot-name> wifi forget <ssid>
```

```
btctl --name <robot-name> name <new-name>
```

```
btctl --name <robot-name> reboot
```

### Global options

- `--name <robot-name>` — connect by advertised name. Without it, the first robot found wins. Worth
  giving always: it skips a slow fallback tier that tries every already-connected peripheral on the
  Mac, earbuds included.
- `--pin <six-digits>` — defaults to `000000`. `robotctl system pin` on the robot shows the real one.
- `--verbose` — print every line sent and received. The first thing to add when something hangs.

### Anything not wrapped above

```
btctl --name <robot-name> call <method> '<json-params>'
```

```
btctl --name <robot-name> call update.check '{"component":"daemon"}'
```

Useful for the refusal boundary, which is worth knowing: motor control, `update.select`,
`update.pin`, `system.pairingPin` and `updaterd`'s private questions to `robotd` are **refused by
`btd` itself** and never reach a daemon. They come back as error code 14, "not available over
Bluetooth". That is a security boundary, not a missing feature —
[`app-path-design.md`](../design/app-path-design.md) §3.1.
