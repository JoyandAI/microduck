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

- **`robotd`, `configd` and `padd` restart during the update. `updaterd` and `btd` restart 5 seconds
  after it replies** — the first cannot restart itself mid-update, and the second may be carrying the
  reply. So a `btd` fix is live a few seconds later, with no manual step. Reconnect and it is there.
- **If one of those two restarts does not happen, the next `updaterd` start fixes it.** Except
  `updaterd` itself, which reports the disagreement rather than restarting itself. Run the apply again
  for that one: it answers `already_current`, names the daemon that is not running it, and schedules
  the restart. `sudo systemctl restart updaterd` does the same by hand.
- **A board running an `updaterd` older than 0.4.0 has none of that** and keeps both on the old binary
  until you restart them. One update fixes it, and only the update after that behaves.
- **`robotctl update apply` reports `already_current` and installs nothing** if you ask for the version
  a board already has — but it is no longer inert. It checks which daemons are running that release and
  restarts the ones that are not, naming them in `stale`. So it *is* the command to reach for when a
  fix looks absent: either it fixes it, or `stale` is empty and the fix was never in that release.

The symptom is a fix that is definitely installed and definitely not working. Ask which release each
daemon is running:

```
robotctl health
```

The `units` block prints one line per daemon with the release its process was launched from, and a
warning naming the restart when that disagrees with what is installed. `build unknown (old)` means
that daemon predates the release which taught it to say — restart it and it will answer.

If a daemon is genuinely stale, restart it — this should not be necessary, so it is worth reading the
journal for why it was:

```
sudo systemctl restart configd
```

`updaterd` is the one that never fixes itself:

```
sudo systemctl restart updaterd
```

Editing the board's `updater.toml` is not needed — the restart set comes from the units the release
ships. `../design/restart-order.md` is the full sequence, step by step.

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

### The first push to a board below 0.5.0

`apply --from` is part of the release being pushed: it needs API version 7, which first ships in
0.5.0. A board running anything earlier has an `updaterd` that cannot be asked to use it, and says
so — `robotctl` and `updaterd` report an API mismatch and refuse the call rather than quietly
installing from the configured source instead. Deliver it once the ungated way, which stops
`robotd` and gives up the health gate for that one install:

```bash
scripts/dev-push.sh --bootstrap radxa@<board>
```

Every push after that is the ordinary command.

## From a laptop — `btctl`

Reaching the robot over Bluetooth LE, with no network and no ssh: [`btctl.md`](btctl.md) has every
command.
