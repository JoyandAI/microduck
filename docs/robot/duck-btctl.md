# `duck-btctl` — every command

Talk to a robot over Bluetooth LE from a laptop, with no network and no ssh. It is the phone app's
stand-in, and the way to reach a robot that has never seen a wifi network.

An *example* rather than a binary, so it is never on a robot. `robotctl` is the tool that ships,
and [`cheatsheet.md`](cheatsheet.md) has its commands — most of which have a `duck-btctl`
equivalent below.

## Getting it

Run it from a clone of this repo:

```bash
cargo run -q -p btd --example duck-btctl -- --name <robot-name> info
```

Or install it once, at the cost of a snapshot that no longer follows the branch:

```bash
cargo install --path btd --example duck-btctl
```

```bash
duck-btctl --name <robot-name> info
```

Every command below is written in the installed form. Prefix it with
`cargo run -q -p btd --example duck-btctl --` to run it from the clone instead.

This tool used to install itself as `btctl`. If `which btctl` still finds one, it is a build from
whenever you installed it and it will never change again:

```bash
cargo uninstall btd --bin btctl
```

## Finding a robot

```bash
duck-btctl scan
```

Robots only, with everything else in radio range counted rather than listed. `--verbose` expands
that list, and it is worth reading when the robot you want is not in the first one.

A robot that has never been renamed calls itself `duck-` plus four characters derived from its
serial, so `duck-c51b`. Either half of a robot reported under two names at once — macOS shows
`radxa-zero3 [duck-c51b]` — works as `--name`.

A robot that scanned as `duck-c51b` and then, after one connection, only as `radxa-zero3` is on a
release that gave its name to the advertisement but not to the adapter, and the client cached the
adapter's. Update the robot. That does not clear what the client already cached, so clear it too:
`bluetoothctl remove <mac>` on Linux, or forget the robot in macOS Bluetooth settings.

With no name at all — no `--name`, no `DUCK_ROBOT` — the first robot found wins. With a name, two
robots answering to it is an error rather than a choice:

```
2 robots answer to "radxa-zero3": radxa-zero3, radxa-zero3
```

That happens on a board whose bootloader leaves its serial blank, because it is then named after
its hostname and every board flashed from one image has the same one. Rename one from the robot
itself and use the new name:

```bash
robotctl system set-name ducky
```

## Always the same robot

Put the name in the environment instead of on every command line:

```bash
export DUCK_ROBOT=duck-c51b
```

Put that line in `~/.zshrc` to keep it. Every command below then works without `--name`:

```bash
duck-btctl info
```

`DUCK_PIN` does the same for `--pin`, which a robot with a PIN of its own needs on every command:

```bash
export DUCK_PIN=418299
```

For one command against a different robot, `--name` still wins:

```bash
duck-btctl --name duck-ffff info
```

To ignore the default for one command — a bench with somebody else's robot on it — set it to
nothing:

```bash
DUCK_ROBOT= duck-btctl scan
```

`scan` marks the robot `DUCK_ROBOT` names and lists it first, and every command that goes looking
for it says so before it starts scanning.

## Identity

```bash
duck-btctl --name <robot-name> info
```

Name, serial and uptime.

```bash
duck-btctl --name <robot-name> name <new-name>
```

Up to 24 characters. It takes effect within a few seconds and needs no restart, but the Mac keeps
serving the name it learned earlier, so `scan` and macOS Bluetooth settings both lag behind. Every
later command uses the new name.

A rename does not follow `DUCK_ROBOT`. The tool says so afterwards; the variable has to be changed
by hand, or every later command looks for a name that no longer answers.

```bash
duck-btctl --name <robot-name> reboot
```

## Wifi

```bash
duck-btctl --name <robot-name> wifi status
```

SSID, signal and addresses.

```bash
duck-btctl --name <robot-name> wifi scan
```

Takes a few seconds — the robot sweeps the radio rather than returning the previous scan.

```bash
duck-btctl --name <robot-name> wifi connect <ssid> --psk <passphrase>
```

Omit `--psk` for an open network. Joining disconnects the robot from the network it is on, so an ssh
session over wifi drops; that is the command working. It can take up to 45 seconds to answer.

```bash
duck-btctl --name <robot-name> wifi forget <ssid>
```

## Is it alright

```bash
duck-btctl --name <robot-name> health
```

Whether the control loop is healthy.

```bash
duck-btctl --name <robot-name> status
```

The version handshake and the update status.

## Anything else — `call`

```bash
duck-btctl --name <robot-name> call <method> '<json-params>'
```

Params default to `{}`. These are reachable over Bluetooth but have no wrapper of their own, and are
written without the `duck-btctl --name <robot-name>` in front of them:

| | |
|---|---|
| `call update.check '{"component":"daemon"}'` | Is there a newer release? |
| `call update.apply '{"component":"daemon","target":"latest"}'` | Install the latest stable release. |
| `call update.apply '{"component":"daemon","target":{"ref":"my-branch"}}'` | Install what a branch last built. |
| `call update.apply '{"component":"daemon","target":{"exact":"0.5.1"}}'` | Install one exact version. |
| `call update.listInstalled '{"component":"daemon"}'` | Which releases are on the board. |
| `call update.log '{"limit":20}'` | The update record that survives a wiped journal. |
| `call system.services` | Which daemons are up, and the release each runs. |
| `call pad.status` | Is a gamepad bonded, and is it connected? |
| `call pad.pair '{"timeout_seconds":30}'` | Bond a pad held in pairing mode. |
| `call pad.forget '{"mac":"<address>"}'` | Drop a bond. |

An apply answers once, when it is finished, and `call` waits 60 seconds for that answer. A daemon
update can take longer — the robot carries on regardless, and `status` afterwards says how it
went.

`call update.subscribe` is the progress stream. It never sends an answer, so it prints progress
until the same 60 seconds run out. Nothing else streams: an apply on its own connection is silent
until it is done.

## Global options

- `--name <robot-name>` — which robot. Without it, `DUCK_ROBOT`; without that, the first one found
  wins. Worth giving always: it skips a slow fallback tier that tries every already-connected
  peripheral on the Mac, earbuds included.
- `--pin <six-digits>` — defaults to `DUCK_PIN`, then to `000000`. `robotctl system pin` on the
  robot shows the real one.
- `--verbose` — print every line sent and received, and have `scan` list every device rather than
  only the robots. The first thing to add when something hangs.

## What it prints

Replies go to stdout as pretty JSON, and everything else — progress, diagnosis, what the radio
saw — to stderr. So `duck-btctl ... info > reply.json` keeps the two apart, and a JSON-RPC error
from the robot still exits non-zero.

One command is one connection: it finds the robot, pairs if it has to, proves the PIN, asks, and
disconnects.

## What is refused

Motor control (`robot.move`, `robot.head`, `robot.enable`, `robot.stop`, `robot.init`,
`robot.relax`), high-rate telemetry (`robot.subscribe`), the operator decisions (`update.select`,
`update.pin`, `update.rollback`, `update.resetToGolden`) and the pairing PIN
(`system.pairingPin`, `system.setPairingPin`) are refused by `btd` itself and never reach a daemon.
They come back as error code 14, "not available over Bluetooth".

That is a security boundary rather than a missing feature, and each refusal has its reason next
to it in `btd/src/route.rs` — [`app-path-design.md`](../design/app-path-design.md) §3.1 is the
design.
Those commands are `robotctl` on the robot.

## When it cannot find the robot

```bash
duck-btctl --verbose scan
```

An empty list — not one pair of earbuds — points at the Mac rather than the robot: Bluetooth off,
or the terminal never granted the Bluetooth permission.

A list the robot is missing from points at the robot. It advertises its name in a scan response that
can be missed on its own, so a device reported with no name and no services is a plausible robot;
`--name` connects to one anyway. If macOS shows the robot as paired but a connection or the first
read hangs, the bond is half-finished:

```bash
sudo pkill bluetoothd
```

Forgetting the robot in macOS Bluetooth settings does the same thing. On the robot itself,
`journalctl -u btd -b` says whether the GATT application registered at all.
