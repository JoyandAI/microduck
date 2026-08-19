# Updating a robot from a phone

Status: open, two decisions wanted · Date: 2026-08-19 · Owner: pierre

The update path over Bluetooth is most of the way there and nobody has driven it. `btd` already
routes the update subset, `updaterd` already streams progress to whoever asked, and
`duck-btctl` can reach all of it through `call`. What is missing is one defect that makes the
app's own flow — start an update, watch it — a black hole, two smaller ones in `updaterd`, a
decision about `update.rollback`, and a client surface worth typing.

Ordered by what blocks what. §2.1 is first because nothing below it is testable while it stands.

## 1. What already works, so it is not rebuilt

**The routed subset covers the reads and the trigger.** `btd/src/route.rs` already sends `hello`,
`update.check`, `update.apply`, `update.status`, `update.subscribe`, `update.log` and
`update.listInstalled` to `updaterd`, and `robot.health` to `robotd`. Nothing in the plan below
adds a method to the protocol; the only routing change under discussion is §2.4.

**An apply already streams its own progress.** `updaterd`'s `run_mutating` writes
`update.progress` notifications to the connection the apply arrived on, before the reply
(`updater/src/ipc.rs`). `btd`'s pool pumps every line from an upstream into the session's
outbound queue without reading it (`btd/src/upstream.rs`), and `duck-btctl` prints id-less lines
and keeps waiting. So progress reaches a BLE client during an apply with no subscription at all.

`docs/robot/duck-btctl.md` says the opposite — *"an apply on its own connection is silent until it
is done"*. That is false as the code stands. It should be confirmed against a board and then the
line fixed, because it currently tells an app author to build the mechanism §2.1 exists to fix.

**A client that reconnects mid-update is not lost.** `updaterd` keeps the latest `Progress` per
component and replays it to a new subscriber before forwarding anything live
(`Server::stream_progress`), and `update.status` answers *during* an update — it takes the engine
lock with `try_lock` and falls back to a cached snapshot with the phase patched in from that same
`latest`. Both are what a phone needs after the link drops, and both already exist.

**The reply gets out before the transport dies.** `updaterd` and `btd` are the two units an update
never restarts, and both are restarted about five seconds *after* the reply
(`RESTART_AFTER_REPLYING`, `docs/design/restart-order.md` §1). So a phone-triggered daemon update
answers the phone, and then drops the link. That ordering is deliberate and load-bearing for this
whole path.

**And the update does not depend on the phone.** `architecture.md` §1.1: the robot pulls, the
update runs to completion whether or not anyone is watching, and `updaterd` stops writing to a
client that has gone without stopping the operation.

## 2. What is missing

### 2.1 A long call blocks every other call on the session · `btd`

`Pool` holds **one** connection per service per BLE session, made on demand. `updaterd` serves
one connection with one request at a time: `handle_connection` reads a line, awaits `dispatch`,
then reads the next.

Both orderings of "update the robot and show progress" fail, and neither says so:

| the client does | what happens |
|---|---|
| `update.apply`, then `update.status` while it runs | the status line is written into a socket `updaterd` will not read until the apply finishes, minutes later. The client times out having heard nothing. |
| `update.subscribe`, then `update.apply` | `stream_progress` owns that connection until the peer goes away and never reads a request again. The apply is written into a socket nobody is reading: it never runs, never replies, and never errors. |

The second is worse than a bug — it is an update the owner asked for, that the robot silently did
not perform. A mobile app will hit it on the first screen that has both a progress bar and a
refresh, and a status poller makes the first case routine.

**Fix: a lane per class of call.** `route.rs` already answers two questions per method — may BLE
make this call, and which service owns it — and this is the third of the same kind: which
connection carries it. Three lanes per service — one for short request/response calls, one for
long mutations, one for a stream — and `Pool` keys its map on `(service, lane)`. A session touching
all three costs three sockets instead of one, and the exhaustive match in `route.rs` means a new
method has to be given a lane by whoever adds it.

Reusing one long lane for successive long calls is correct rather than a compromise: `updaterd`
single-flights mutations behind a file lock and answers `BUSY` for the second, so serialising two
applies is what should happen anyway.

Two alternatives, both rejected. **Refuse a second updater call while one is outstanding**, with
`BUSY` — honest instead of silent, three lines, and it breaks exactly the screen this exists to
serve: the status poll fails during the one update it wants to watch. **A connection per call**
would be the clean model, but closing one needs `btd` to know a call has ended, which needs it to
parse replies; it deliberately never does, and that property is what keeps it a transport rather
than a second implementation of the API.

### 2.2 Progress is a firehose aimed at a 20-byte pipe · `updaterd`

The download pump sends a `Progress` per HTTP chunk (`updater/src/source/http.rs` calls
`progress.send` on every chunk written). A serialised progress line is around a hundred bytes,
which is five or six notifications at the 20-byte floor `btd` frames to, and a daemon artifact is
thousands of chunks. `btd` drops lines when the outbound queue is full — correctly: progress is
advisory and blocking would stall every other upstream — so what a phone actually sees is an
arbitrary subset of the percentages, and the last one before a phase change is as droppable as
any other. A bar that jumps 12 → 61 → 34 is worse than one that moves once a second.

**Fix: decimate where the numbers are made.** In the pump, emit only when the whole percent
changes, and no more than a few times a second. That is at most 101 events for a download instead
of thousands, it is one small change in `engine.rs`, and it fixes `robotctl update watch` at the
same time — every transport pays for this today, BLE is only where it is fatal.

Conflating in `btd`'s outbound queue — keep the newest progress line, drop the older ones — would
be the transport-local fix and needs `btd` to parse notifications. Same objection as §2.1.

### 2.3 `update.check` blocks on the engine lock · `updaterd`

`ipc.rs`'s own header states the rule: a long operation holds the engine mutex, so read-only
requests use `try_lock` and fall back rather than blocking — "that is what keeps `status` and
`subscribe` answerable *during* an update". `Call::Check` does not follow it. It takes
`self.engine.lock().await`, so a phone asking "is there an update?" during an apply gets a spinner
that resolves whenever the apply finishes, with nothing to tell it apart from a dead robot.

**Fix: `try_lock`, and `BUSY` when it fails**, which is already the idiom for a mutation that
arrives during one. `Call::Pin` has the same shape and the same fix, and is mutating, so `BUSY` is
exactly right there.

### 2.4 Decision: `update.rollback` over Bluetooth

Refused today, with the reasoning in `route.rs`: the engine reverts a release that fails its own
health gate, so the phone needs no button for the ordinary case, and recovery mode should be what
reopens it.

**The case for opening it.** `update.apply` is already routed and is strictly the more
consequential of the two: it installs code that has never run on this board, from the network,
and rollback returns to a release that ran on it yesterday. The gate covers the release that fails
to come up, which is not the case an owner reaches for a phone about — that is a release that
installs, passes its gate, and then behaves worse: a walking policy that is unsteady rather than
dead, a robot that gets warm, a pad that stops reconnecting. Nothing reverts that but a person, and
the person has a phone in their hand and no ssh. A rollback discards nothing, and it is gated and
auto-reverted like any transition (`Engine::rollback` goes through `transition_to`).

**The case against.** It churns the boot counter, and the link it would arrive over is
unencrypted with a printed default PIN (`app-path-design.md` §5.5, §8.1), so anyone in radio range
who has read this repository can revert someone's robot. That is already true of `apply`, which is
the honest way to put it: opening rollback widens what that peer can do by roughly nothing.

**Recommendation: open `update.rollback`.** Keep `resetToGolden` refused — it is a factory reset in
all but name, and never over a radio.

**And a second question with it: `update.select`.** `update.listInstalled` is already routed, so a
phone can already show every release on the board. Once it shows the list, the natural gesture is
"go back to *that* one", which is `select` rather than `rollback` — `rollback` only ever means the
previous release. `select` downloads nothing, verifies nothing over the network, and is "gated and
rolled back like an update" by its own doc comment, so it is technically the *safest* of the three.
What it is not is a one-tap undo: it needs a version picked out of a list, which is the shape of an
operator decision made with a record of who made it, and that is why it sits with `pin` today.

Either answer is defensible and they lead to different app screens, which is why it is a question
and not a recommendation:

- **Rollback only** — one button, "undo the last update". The app never shows a version picker,
  `listInstalled` stays informational, and `select` stays operator surgery.
- **Rollback and select** — the app shows the installed releases and can activate any of them.
  More useful on a bench and to anyone diagnosing a robot they cannot ssh into; a bigger surface
  to get wrong from a mistap, and the update log will not say which phone made the choice (§2.5).

`pin` stays refused under either: a robot pinned by a mistap refuses every later update and
reports itself as up to date, which is the one failure here that looks like correct behaviour.

### 2.5 A phone-triggered update is logged as `btd`

`updaterd` records the caller's uid and pid from `SO_PEERCRED` on every mutation, which is how
support answers "who triggered this rollback". Over BLE the answer is always `btd`, for every
phone. Recorded rather than fixed: `btd` forwards params verbatim and adding a "who" field would
make it an author of requests rather than a pipe. Worth reopening only if support actually needs
to tell two phones apart.

### 2.6 Client deadlines are total, not idle · `duck-btctl`, and the app

`duck-btctl` waits a fixed budget for a reply — 15 seconds, or 60 for the slow calls. A daemon
apply is minutes: download, verify, extract, swap, hooks, the health gate. Raising the number is
the wrong shape of fix, because the useful signal is already arriving: every progress notification
is proof the robot is alive and working.

**Reset the deadline on every line received**, and give an apply a generous idle budget on top.
Then a stalled mirror times out in seconds while a slow-but-progressing update never does. The app
inherits the principle rather than the code, and it belongs in the app's own notes.

### 2.7 The link drops about five seconds after a daemon apply replies

Not a defect — §1's ordering, working. But every client has to expect it, and neither the tool nor
a future app should render a disconnect that the robot announced as a failure of the update that
just succeeded. The sequence to build against:

1. `update.apply` replies with its outcome. This is the answer; the update is done.
2. About five seconds later `btd` restarts and the connection drops.
3. Reconnect, `hello`, and `update.status`. `last_attempt` carries the outcome of what just ran,
   so a client that missed the reply in step 1 can still report it.

`duck-btctl` should print step 2 before it happens, so the disconnect reads as expected.

### 2.8 Open, deliberately: `from_dir` over Bluetooth

`ApplyOptions.from_dir` names a directory on the robot, which is meaningless to a phone and useful
on a bench. Filtering it would mean `route.rs` deciding on params rather than methods, and
inspecting params is what `btd` does not do (§2.1's third paragraph). Preflight already refuses the
cases that bite — `PrivateTmp` makes `/tmp` a different directory for `updaterd` than for the shell
that copied into it, and it says so — so this is left as it is, and named here so the next person
finds a decision rather than an oversight.

## 3. The `duck-btctl` surface

Today every update operation goes through `call` with hand-written JSON. The commands below mirror
`robotctl update` name for name, so what someone learns on the robot transfers to the radio and
back:

| `duck-btctl` | method |
|---|---|
| `version` | `hello` — the API version, the daemon's version, and the revision it was built from |
| `update check` | `update.check` |
| `update apply [--version V \| --ref R \| --staging] [--dry-run]` | `update.apply` |
| `update status` | `update.status` |
| `update versions` | `update.listInstalled` |
| `update log [--limit N]` | `update.log` |
| `update watch` | `update.subscribe` |
| `update rollback` | `update.rollback`, if §2.4 opens it |

`status` stays what it is — the handshake plus the update status — because it is in every set of
notes anyone has written down, and `version` is the narrower question it currently buries.

Progress prints as a line a person can read (the phase, and the percent when there is one) rather
than as the raw notification, with the reply still going to stdout as JSON so redirection keeps
working. `--verbose` keeps showing every line on the wire.

`duck-btctl` is a stopgap and this is where that matters: the three fixes above are robot-side, so
the app inherits them, and §2.6's timeout shape is client-side, so the tool gets it right and the
app's notes say why.

## 4. Order

1. **§2.1**, the lanes. Nothing below is testable while a second call during an apply is a black
   hole, and it is the only change here a mobile app cannot work around.
2. **§2.3** then **§2.2** — small, independent, and they make progress worth watching.
3. **§2.4**, once answered: the route entries, and the named-one-by-one tests in `route.rs` that
   are the security boundary.
4. **§3** with **§2.6** and **§2.7** — the surface, on top of a transport that behaves.
5. Docs: `duck-btctl.md`'s stale claim (§1) and its command list, and `app-path-design.md` §3.1
   for whatever §2.4 decides.

## 5. What this does not do

**No cancel.** The engine has none, and a half-applied update is worse than a slow one. A client
that walks away is already handled: the update finishes.

**No auto-apply toggle from the phone.** `deploy/updater.toml` deliberately does not opt client
robots into unattended restarts, and that decision is not one to expose as a switch before there
is a robot fleet to reason about.

**No encryption.** `app-path-design.md` §8.1 owns it, it blocks nothing here, and every claim
above about what a peer in radio range can do is written on the assumption that it is still off.
