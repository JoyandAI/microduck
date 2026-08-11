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
