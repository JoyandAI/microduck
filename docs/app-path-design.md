# The App Path — `btd` and `configd`

Status: draft · Date: 2026-08-04 · Owner: pierre

How a phone configures a robot: wifi, name, reboot, version, and triggering an update.

Companion to [`architecture.md`](architecture.md), which owns the service split and the cross-cutting
contract. This covers the two services that landed together, because **they are one feature** and
every decision in one constrains the other: `btd` owns nothing, so `configd` exists; `configd`
serves a PIN, so `btd` can pair; a method routed in `btd` is a method `configd` must answer.

Sections marked **measured** were established on a Radxa Zero 3W rather than reasoned about.

**The path works end to end on hardware** (2026-08-05): a Mac discovered the robot, bonded,
read the API version, passed the PIN, and got a real `system.info` back — GATT discovery, chunked
NDJSON both ways, the PIN gate, the routing table and the hop into `configd` over its unix socket.
`configd` answers against a real NetworkManager too, reporting the live SSID and address.

What is **not** yet true: the link carries no encryption (§5.5), `net.connect` has not been driven
over BLE, and nothing has been tested with a phone rather than a laptop.

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

### 3.2 One session per subscription, and the bug that decided it  · **measured**

`btd` keeps one session — one reassembly buffer, one outbound queue, one authorisation state — and
the question was how long it lives. The first answer was "as long as the service", because BlueZ's
callback model gives a subscribe no peer identity and only ever holds *one* notify state per
characteristic, so per-peer sessions looked like machinery for a case that cannot arise. A stale
partial line seemed to cost at most one bad request.

It cost the *next* client instead, which is worse, and it took three symptoms on a board to see it:

| symptom | cause |
|---|---|
| a request answered, then the following one timing out | the outbound receiver was taken out of the shared slot by the first pump, so the second subscription had no pump: the reply was written to a channel nobody read |
| `":0,"result":{"authenticated":true}}` — a reply with its beginning missing | those orphaned chunks surfacing through a later notifier |
| `no robot found`, then the same command working | unrelated: a client-side scan taking one snapshot after a fixed sleep, so whether the advertisement fell inside that window was luck |

Only the third was a client bug. The first two are the same defect: **state that outlives the peer it
belonged to.** A disconnect is invisible in this model, so nothing reset it.

So the session is created when a central subscribes and discarded when it goes away — the reassembler
and the queue go with it, and a reconnecting phone starts unauthenticated, which is the behaviour §5.2
already claimed. Two details are load-bearing and both were wrong first:

- The pump waits on `notifier.stopped()` as well as the queue. Learning of a departure only from a
  failed notify needs a reply to send, so a client that disconnects while idle would hold the slot
  until a request arrived for nobody.
- Teardown clears the slot only if it still holds *its own* sender. A notify to a vanished central
  takes as long as BlueZ takes to give up, by which time a reconnecting central may have installed a
  newer session that a blind clear would kill.

The write path still refuses a write with no live subscription. Accepting one would be a lie: there
is nowhere to send the answer.

## 5. Pairing: just-works, and a PIN the transport checks

A six-digit PIN, stored by `configd`, checked by `btd` before it serves anything. **Not** by the
Bluetooth bond — and that is forced by the spec rather than chosen.

### 5.1 Why BLE cannot carry a printed PIN  · **measured**

The first design had the robot answer BlueZ's passkey request with its stored PIN. On hardware, macOS
displayed *its own* random six-digit code and waited for someone to type it into the robot.

In LE passkey entry one side **displays** a passkey and the other **inputs** it, and the roles follow
from the IO capabilities each side declares. Implementing `request_passkey` declares "this device can
input", so macOS took the display role. A robot with no keyboard cannot fill that role.

The reverse is no better. With `DisplayPasskey` the robot takes the display role, but the **spec has
the displaying side generate the passkey at random** — BlueZ chooses it and hands it to the agent.
There is no way to make it present a value we stored, and a headless robot has nothing to display it
on anyway.

So a fixed, printed-on-the-robot PIN is not expressible in BLE passkey entry. Three options remained:

| | |
|---|---|
| Just-works only | Encrypted, unauthenticated, no PIN. Security is physical presence. What most headless BLE devices do |
| Out-of-band (QR) | Genuinely authenticated and genuinely per-robot. BlueZ's OOB support is thin and no phone app exists to drive it. A large lift for v1 |
| **Just-works plus an app-layer PIN** | **Chosen.** Pair for encryption; check the PIN in the session, where we define the rules |

### 5.2 How it works

Pairing is just-works: every agent handler is `None`, which `bluer` publishes as `NoInputNoOutput`.
The read on the RPC characteristic requires `encrypt_read`, which is what makes a central bond at
all — plain encryption, not `encrypt_authenticated_*`, because a just-works bond can never satisfy
the authenticated variants and demanding them would refuse every client.

Then `btd` serves nothing until the client sends `system.authenticate`. That call is answered by the
transport rather than forwarded, which is why the routing table has a third outcome (`Route::Local`)
alongside "forward" and "refuse". `hello` is the one other call allowed through unauthenticated,
because it reports only versions — the same thing the GATT read already tells an unauthenticated
client — and refusing it would leave a mismatched client unable to learn why nothing works.

Three details that are load-bearing rather than incidental:

- **The PIN is fetched from `configd` per attempt**, not cached, so `robotctl system set-pin` takes
  effect on the next try rather than the next reboot. A `configd` that cannot answer means the
  session is refused rather than admitted.
- **Compared as a string.** `042042` and `42042` are different secrets; a numeric parse would make
  them the same. There is a test for exactly that.
- **Three attempts, then the session closes.** A six-digit PIN is a million guesses over a link that
  is encrypted but not authenticated, so rationing is the only thing making brute force expensive:
  reconnecting costs a full BLE connect and bond. `attempts_remaining` comes back to the client so it
  can say "two left" rather than silently losing its connection.

### 5.3 What this is and is not worth

**The PIN crosses an encrypted-but-unauthenticated link**, so an attacker present *at the moment of
pairing* could capture it. That is the price of the trade, and it is the reason to prefer OOB later
if the threat model ever justifies it. What it buys over just-works alone is that a device which
merely bonds — trivial for anyone in range — still cannot do anything.

**The factory PIN is `000000` and is public in this repository.** Out of the box, therefore, this
proves physical presence and nothing more. `btd` logs a warning on every authentication with the
default, and `robotctl system pin` says so too. Security rests entirely on the PIN being per-robot,
which makes it a **provisioning obligation**: something must generate it, print it, and record what
was printed. That is `updater-design.md` §5.7's per-device state, the same slot that owes us a serial
number.

**No pairing window, and that is decided rather than deferred.** The robot is pairable whenever it
advertises. A per-robot PIN already carries what a window would add: knowing a printed PIN requires
physical access, and anyone who can read the sticker can pick the robot up. A button would add a
visible consent moment, a recovery path for a lost PIN, and defence in depth if a sticker is
photographed — none needed for v1, each additive later, since an enclosure with a button can gate
`set_pairable` without changing this design.

### 5.5 Encryption is currently off, and that is not settled  · **measured**

`encrypt_read` on the characteristic makes the read **hang** on macOS: CoreBluetooth issues the Read
Request, BlueZ refuses it for insufficient encryption, and nothing resolves it — no prompt, no
error, no retry. The client waits out its timeout against a working robot. With the flag off the read
answers instantly, so the requirement is the cause.

So `btd` currently runs with `--insecure-no-pairing` on the test board, and **the PIN crosses an
unencrypted link**. That is worse than §5.3 describes: it is not "encrypted but unauthenticated", it
is neither. Anyone in radio range during the exchange can read the PIN, and thereafter do anything a
client may do.

Unresolved, and the next thing to establish is whether a bond exists at all — `bluetoothctl info
<mac>` reporting `Paired: no` would mean no encryption can ever be established and the flag is a
symptom rather than the cause. Until that is known, moving the requirement to the write is a guess:
it would fail identically if there is no bond to encrypt with.

This must be closed before anything ships. A robot whose provisioning secret is readable by a
bystander is not a robot you can hand to a stranger.

### 5.6 Open

- **Bond revocation.** Nothing un-pairs a phone; `bluetoothctl untrust` is the manual escape. Needs
  an API and a rule about who may call it — plausibly not BLE itself.
- **Rate limiting survives only within a session.** Three wrong PINs close the session, but nothing
  counts across reconnects, so a determined peer can retry indefinitely at the cost of a bond per
  three guesses. A per-address backoff in `btd` is the obvious next step and needs somewhere to keep
  that state across sessions.

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
