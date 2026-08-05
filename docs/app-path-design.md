# The App Path — `btd` and `configd`

Status: draft · Date: 2026-08-04 · Owner: pierre

How a phone configures a robot: wifi, name, reboot, version, and triggering an update.

Companion to [`architecture.md`](architecture.md), which owns the service split and the cross-cutting
contract. This covers the two services that landed together, because **they are one feature** and
every decision in one constrains the other: `btd` owns nothing, so `configd` exists; `configd`
serves a PIN, so `btd` can pair; a method routed in `btd` is a method `configd` must answer.

Sections marked **measured** were established on a Radxa Zero 3W rather than reasoned about.
Everything else is intent, and neither service has met a radio or a real NetworkManager yet.

## 1. The shape

```
        phone ──BLE──▸ btd ──┐
     robotctl ──unix socket──┼──▸ configd ──D-Bus──▸ NetworkManager   (wifi)
    (mediad) ──WebSocket────┘                    └──▸ logind          (reboot)
                             │                    └──▸ config file     (name, PIN)
                             └──▸ updaterd  (update.*)
                             └──▸ robotd    (robot.health)
```

Two rules from `architecture.md` produce this and nothing else was free to vary:

- **§4.1: `btd` owns nothing.** If provisioning or config lived in the BLE service, every other
  service would depend on it, and an SDK would absurdly have to go through Bluetooth to set a
  robot's name.
- **§3.1: config must be reachable when `robotd` is dead.** Provisioning wifi is exactly what
  someone needs when the robot is broken, so config cannot live in the control daemon.

Between them there is no service left to put `net.*` and `system.*` in, hence a fifth one.

**Most of this work was not Bluetooth.** The API surface and the service owning it are needed
identically by the phone app, the SDK, `robotctl` and `mediad`'s remote gateway. `btd` is a thin
pipe over it — and the test of that claim is that adding the seven `net.*`/`system.*` methods cost
`btd` one line each in a routing table.

## 2. Wifi: NetworkManager, and why a board has to be migrated to it  · **measured**

`architecture.md` §3 chose NetworkManager. The board does not have it.

Armbian's headless image runs netplan + `systemd-networkd` + `wpa_supplicant`. Three findings from
the board made the choice again rather than inheriting it:

- **The D-Bus-enabled `wpa_supplicant` holds no interface.** `fi.w1.wpa_supplicant1` is claimed and
  idle (`Interfaces` is an empty array); netplan runs a *second*, `-c`-configured supplicant that
  owns `wlan0` and has no D-Bus at all. So driving `wpa_supplicant` directly would mean displacing
  netplan anyway — with none of NM's failure reporting.
- **netplan cannot report what a phone needs.** It is a config *generator*: no scan API at any
  layer, and `netplan apply` returns "config applied" rather than whether association succeeded.
  "Show me the networks" and "that password was wrong" are the two things a provisioning flow needs
  most, and it answers neither.
- **`RequiredForOnline=no` makes boot worse, not better.** Armbian ships a drop-in turning
  `systemd-networkd-wait-online` into `--any`: succeed when *any* networkd link comes online. Once
  wifi belongs to NM, networkd's only link is a usually-cableless ethernet port, so `--any` can
  never be satisfied. Marking that link not-required removes the only candidate and guarantees the
  failure. Masking the unit is the fix; `NetworkManager-wait-online` is the honest gate.

`scripts/migrate-network.sh` performs the migration once, and refuses to cut over until it has
copied the board's existing credentials into an NM profile — otherwise a headless board goes
offline with no way back. It arms a boot-time backstop that restores netplan and reboots if `wlan0`
has no address after 90s, which is the update system's boot-counter idea applied to a network
change.

### 2.1 `BadKey` is the whole point

`ConnectFailure::BadKey` is why NM was worth a migration. A rejected passphrase is the commonest
provisioning failure there is, and a client that cannot say so leaves the user with nothing to do.
NM reports it as device state reason 7; `configd` maps NM's reasons to `BadKey`, `NotFound`,
`Timeout`, `Unsupported` and `Other`, and an unmapped reason must never become `BadKey` — that
would send someone round a loop retyping a key that was already correct.

`configd` polls NM after `AddAndActivateConnection` rather than returning when activation *starts*,
because "config applied" is the answer netplan gives and the one we rejected.

## 3. The GATT surface: a pipe, not an API

One service, **one characteristic**. A client reads it once for the robot's API version, writes
NDJSON request bytes to it, and subscribes to it for answers — the same JSON-RPC lines every other
transport carries. The read is not optional; see §5 for why it exists.

**No framing header.** The newline that already separates NDJSON messages is the frame delimiter in
both directions. That is safe rather than lucky: `serde_json` escapes a newline inside a string as
`\n`, so a raw `0x0A` never appears inside a serialised object — the same property that makes NDJSON
work on a unix socket. A length prefix would be a BLE-only dialect every client had to implement;
instead a phone does what `robotctl` does: write bytes, read until newline.

Reassembly is capped at 8 KiB, because that buffer is reachable by anyone in radio range.

**Alternatives, and why not:**

| | why not |
|---|---|
| Per-field characteristics (name, ssid, ip, connect…) | Browsable in a generic BLE app, but a second dialect of the same API: every field becomes a UUID plus `btd` code, and `net.scan` (a list) and `update.subscribe` (a stream) fit badly |
| Two characteristics, write and notify | The conventional shape, and written that way first. BlueZ reports a write and a subscription as *separate events*, so two characteristics must be matched across them by device address — guessing at an association that one characteristic gets by construction |

The cost of one characteristic is that it reads oddly in nRF Connect, where the same row is both.

### 3.1 The routed subset is the security boundary

BLE exposes a subset (§4.1). One table in `btd/src/route.rs` decides both *whether* a call is
permitted and *which socket* answers it, because those are the same question: a call is allowed
exactly when the table names a service for it.

**The match over `Call` is exhaustive on purpose.** Adding a protocol method fails `btd`'s build
until someone decides about it. A `_ => None` wildcard would be the safe default in the moment and
wrong over time — it would deny new methods silently, and the first symptom would be a phone app
missing a feature nobody remembered to route. This has already paid for itself once: the seven
`net.*`/`system.*` methods broke the build, as did `updaterd`'s equivalent match.

Refused, each for a reason:

| refused | why |
|---|---|
| `update.select`, `update.pin` | Operator surgery, made with `robotctl` and a record of who did it — not a mistap in a phone UI |
| `update.rollback` | The engine reverts a bad release itself, so the phone needs no button for the ordinary case. Recovery mode (§8.2) should reopen this deliberately |
| `update.resetToGolden` | Factory reset in all but name. Never over a radio |
| `robot.safeToRestart`, `robot.modelApi`, `robot.remoteSessionActive` | `updaterd`'s private questions to `robotd`; a phone reading them learns nothing it can act on |
| `system.pairingPin`, `system.setPairingPin` | **The load-bearing one.** A passkey an unpaired peer could read — or overwrite — would make pairing theatre. `btd` reads it over the unix socket instead |

## 4. Authorisation: two layers, kept apart

| layer | mechanism | decides |
|---|---|---|
| 1 | socket mode `0660`, group `robot` | who may **connect and talk** |
| 2 | `allow_users` / `--allow-user` | who may make **mutating** calls |

Read-only calls skip layer 2 entirely, so support can inspect a robot it may not change.

Two layers because `btd` must be in the `robot` group to reach the sockets at all, and being in
that group must not amount to "may replace the firmware". Both services therefore grant change
authority to the **named service** — `allow_users = ["btd"]` in `updater.toml`,
`--allow-user btd` in `configd.service` — and both have a test refusing `robot` as a group.

**By name, never by uid.** `systemd-sysusers` allocates dynamically, so a number written into a
shipped config is correct on the board it was written for and wrong on the next one. Names resolve
at startup; an unresolvable name warns rather than aborting, because a robot missing an optional
service must still serve status.

`SO_PEERCRED` reports only a peer's **primary** gid, which is the trap here: `SupplementaryGroups=`
gets a process through the socket mode and no further. Missing that is what made every mutating
call over BLE return `PERMISSION_DENIED` while everything read-only worked — the worst shape for a
bug, because it reads as a mystery rather than a configuration error.

### 4.1 Privilege, and where the parser sits

`btd` is unprivileged; `configd` runs as root. That looks backwards and is not.

`btd` is the process parsing bytes from anyone in radio range. `configd` only ever sees typed JSON
arriving over a peer-credentialled local socket. **Putting the parser on the safe side of that
boundary matters more than hardening the dispatcher.**

`configd` needs root for a narrow reason: NM's connection-modify and logind's `Reboot` are both
polkit-gated, there is **no polkit on this image**, and systemd denies both to a session-less
non-root caller. The alternative was installing a JS policy engine to authorise two calls. Unlike
`robotd` it touches no hardware, so its unit sandboxes it properly — `ProtectSystem=strict`, one
writable path, `AF_UNIX` only, empty `CapabilityBoundingSet`. `CAP_SYS_BOOT` is deliberately absent:
logind performs the reboot and `configd` only asks, so a capability there would permit the unclean
`reboot(2)` this design exists to avoid.

If polkit ever arrives for another reason, `configd` should drop to a dedicated user plus two rules.

## 5. Pairing: a PIN, and no button

A six-digit PIN, stored by `configd`, answered to BlueZ by an agent in `btd`. Because the bond uses
**passkey entry** rather than just-works, the link is authenticated and MITM-resistant — which is
what makes `encrypt_authenticated_write` on the characteristic mean something, and satisfies §7's
requirement that anything carrying wifi credentials be paired and encrypted.

**Six digits, not five.** Bluetooth's passkey entry is defined as a six-digit value, 000000–999999,
and BlueZ hands an agent exactly that. Five would have to be padded somewhere, and the two sides
would then disagree about whether the PIN is `12345` or `012345` — a support call nobody can
diagnose. Stored as a *string*, because leading zeros are significant and a `u32` renders `012345`
as `12345`.

The PIN is fetched **per pairing request** rather than cached, so changing it takes effect on the
next pairing rather than the next reboot. A `configd` that cannot answer means the bond is
**refused**: falling back to the default would let anyone pair whenever `configd` hiccuped.

**A read triggers the bond.** The characteristic requires an authenticated encrypted link to
*write*, but `bluer` 0.17 offers no encryption flag for a subscribe — so a central would subscribe
without encryption, write, be refused, and on macOS see neither a prompt nor an error: a client
timing out against a working robot. A read *is* acknowledged, so the characteristic carries one that
requires the bond, and clients read it first. It returns `API_VERSION`, which makes it useful as
well as necessary — a mismatched client can say so before sending anything. Any client, phone app
included, must read before it writes.

**No pairing window, and that is decided rather than deferred.** A per-robot PIN already carries
what a window would add: if it is unique and printed under the robot, knowing it requires physical
access, and anyone who can read the sticker can pick the robot up. A window would defend only
against someone in range while the factory default is still in place, and the answer to that is a
real PIN. A button would add a visible consent moment, a recovery path for a lost PIN, and defence
in depth if a sticker is photographed — none needed for v1, each additive later, since an enclosure
with a button can gate `set_pairable` without changing this design.

**So the security rests entirely on the PIN being per-robot, which makes it a provisioning
obligation rather than a software one.** The factory default is `000000` and public in this
repository: out of the box, pairing proves physical presence and nothing more. Something must
generate a PIN, print it, and record what was printed — `updater-design.md` §5.7's per-device
state, the same slot that owes us a serial number.

### 5.1 Open

- **Bond revocation.** Nothing un-pairs a phone; `bluetoothctl untrust` is the manual escape. Needs
  an API and a rule about who may call it — plausibly not BLE itself.
- **Whether `btd`'s user may reach `org.bluez`.** The image's policy allows any user to send method
  calls to `org.bluez`, and replies and signals are allowed by default, so serving GATT objects
  should work. The narrow risk is `RequestDefaultAgent`, which BlueZ may restrict to root — and
  `btd` needs it, because incoming pairing is routed to the *default* agent. If it is refused, the
  fix is a policy drop-in for the `btd` user, not running as root.

## 6. Testing without a radio  · **measured**

The suite runs on a laptop with no hardware, no network, no D-Bus and no Docker, and that had to
stay true. Two seams make it so:

- **`configd`'s wifi is a trait** with an in-memory fake, as `duck-control` has `RobotIo`.
  `--fake-net` serves the whole `net.*` surface including a wrong-key failure on demand, which is
  awkward to provoke against a real access point.
- **`btd`'s radio is two channels, not a trait.** A `GattLink` trait would need an async `recv` and
  an async `send`, and the session loop waits on both at once — meaning associated types or a fight
  with the borrow checker inside a `select!`. A plain struct holding two `mpsc` channels says the
  same thing, and a test constructs one instead of implementing anything.

So the session tests drive a complete BLE conversation over real unix sockets: a refused call never
reaching the daemon, `robot.*` routing to `robotd` rather than `updaterd`, and every notification of
a subscription stream arriving through a 23-byte MTU.

`board-test.sh` covers what only appears on Linux: the socket modes, `--allow-user` resolving a
name, a group member reading, a non-member blocked by the socket mode, an unnamed member denied a
mutating call **and the refused change not having taken effect**, a rejected passphrase exiting 5
rather than 1, a PIN keeping its leading zero, and no passphrase in the log. Plus `btd --version`,
which is a real cross-link check: `btd` is the only binary pulling C beyond `zstd`, because `bluer`
links libdbus built from vendored source by `zig cc`.

`btctl` (`cargo run -p btd --example btctl`) is the phone's stand-in and the only way to exercise
the radio. An **example, not a binary**, so `btleplug` never reaches the robot; `btleplug` rather
than `bluer` because it must run on a developer's Mac. It reuses `btd::framing`, so the chunking is
genuinely the client half of the robot's own code rather than a reimplementation free to agree with
itself.

### 6.1 What is not tested

- Neither service has met a **real radio** or a **real NetworkManager**. Both type-check for
  aarch64; that is all.
- The **cutover** in `migrate-network.sh` runs only on a freshly flashed board. It was performed by
  hand once, step by step; the script was then re-run over the result to confirm the idempotent
  path. The first person to flash a board is the real test.
- **~73s before BLE answers.** `hci0` does not exist until `aic-bluetooth.service` attaches the
  AIC8800's UART, and `bluetooth.service` spends 26s blocked behind `dbus`. `btd` waits and retries
  rather than exiting — the same lesson as `robotd` waiting for the motor bus — but a phone app
  designed around instant discovery will be disappointed.

## 7. Costs accepted

- **Two D-Bus stacks in the artifact.** `btd` links libdbus through `bluer`; `configd` uses `zbus`.
  A few MB. `bluer` was chosen because a GATT server, advertising and a pairing agent are exactly
  what it exists for, against roughly 700 lines of hand-written `org.bluez` object plumbing. Worth
  revisiting if `bluer` grows a `zbus` backend.
- **A vendored libdbus** is ours to keep current rather than the distro's. Acceptable for a library
  reached only over a local socket by a daemon we wrote.
- **`btd` is deliberately absent from `on_apply`'s restart set**, so it runs the old binary until
  the next reboot. It may be the *transport the update was requested over*: restarting it drops the
  connection carrying `update.subscribe`, and the phone that started the update never learns the
  outcome. Same reason `updaterd` does not restart itself (§8.3).
