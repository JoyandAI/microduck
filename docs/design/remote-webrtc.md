# WebRTC: sessions, signalling, and the control channel

How a phone, a browser or a server-side program drives and observes the robot over WebRTC.
[`architecture.md`](architecture.md) §5 states the requirement; this owns the mechanism.

Scoped to **local signalling**: everything below runs on the robot and works on a LAN with no
backend at all. Reaching a robot from outside the LAN is the same design with a proxy in front of
it (§7) — deliberately not the first thing built, because the local case is the one every other
case is defined in terms of.

## 1. What this is not

`webrtcbin` is not used. `mediad` uses **`webrtcsink`** from `gst-plugins-rs`, and the difference
is the whole reason this document is short: `webrtcsink` brings a signalling protocol, a session
model, and per-consumer encoder management, so what is left to design is the *control* surface
rather than the media plumbing.

`webrtcbin` would mean writing all three. It is in Debian and `webrtcsink` is not
([`media-bringup.md`](../project/media-bringup.md) covers how that plugin is built and shipped),
which is the one argument for it — and it is outweighed by the protocol coming for free, because
that protocol is what a remote bridge proxies (§7).

## 2. One session, four streams

```
peer (browser / phone / server-side program)
   ├── video track      camera, hardware H.264 (mpph264enc, constrained-baseline)
   ├── audio track      mic; two-way for telepresence
   ├── datachannel "control"   reliable, ordered      → the robot API (§5)
   └── datachannel "teleop"    unreliable, unordered  → input and high-rate telemetry
```

Two data channels rather than one, for the reason `architecture.md` §5.2 gives: a retransmitted
80 ms-old joystick command is worse than useless, so teleop goes `maxRetransmits: 0` and always
takes the newest.

**The first version opens `control` only.** Teleop is not the near-term priority, and leaving it
out is not merely deferral — §6 is about what it removes.

**`webrtcsink` takes pre-encoded H.264 on its sink pad**, so the pipeline is
`appsrc ! mpph264enc ! h264parse ! webrtcsink` and the encoder never reaches negotiation. Verified
on hardware; the four encoder properties that are decisions rather than defaults are in
[`media-bringup.md`](../project/media-bringup.md).

## 3. `mediad` runs the signalling server itself

`webrtcsink` has `run-signalling-server`, with `signalling-server-host` and
`signalling-server-port` (gst-plugins-rs 0.15.3). So the server runs **in `mediad`'s own process**
and there is no second binary to build, ship or supervise — which matters, because the plugin we
ship is a `.so` while `gst-webrtc-signalling-server` is a separate Rust binary from the same
upstream crate. Not shipping it is a real simplification, not a shortcut.

`webrtcsink`'s own signaller defaults to `ws://127.0.0.1:8443` and connects to the server it just
started. A LAN client connects to the same server directly.

**Bind address is a decision, not a default.** Loopback only would mean a LAN peer cannot reach it
at all and every session goes through a bridge, which defeats the point of a local mode. So it
binds on all interfaces, and §4 is what that implies for who may drive.

## 4. Authorisation: none on the robot, and why that holds both locally and remotely

**No gate in the first version.** A peer that reaches the signalling server can start a session,
drive the robot, and see through its camera. That is a decision, not an oversight.

Usability outranks hardening at this stage, and here the trade is not even close. The robot's
pairing PIN is a shared `000000` — a PIN that is the same on every robot authenticates nobody — so
requiring it over WebRTC would add a step to every first connection and buy no safety at all. An
awkward first connect is a real cost; this particular gate is a real cost with no benefit.

What it costs, stated plainly so nobody has to discover it: anyone on the same network has the
robot and its camera. Fine on a bench and in an office. **Not fine in a home**, which is the thing
to revisit before one ships to one.

### Where authorisation actually lives: the bridge, and it is already there

The remote path does not need a gate on the robot either, because it is authenticated **before it
reaches one** — on both sides:

- **The client** authenticates to the rendezvous service with OAuth, and the service shows it only
  the robots its account owns. Reaching the part of the bridge that routes to a given robot *is*
  the proof.
- **The robot** authenticates outward: its relay holds an account token and connects to the service
  with it (§7). So the robot proves it belongs to the account too.

The service is therefore matching two already-authenticated parties, and a session arriving through
it has been authorised twice over. A `system.authenticate` on top would be a second answer to a
question already answered — and a worse one, since the shared `000000` PIN proves less than an
account token does.

**What this means is that the trust moved rather than vanished**, and it is worth naming where it
went: the robot has no independent check, so the binding between a robot and an account is now the
thing that must be right, and it lives in the service rather than here. That is an acceptable place
for it — it is the only component that can know the answer — but it is a dependency, not an absence
of one.

The one arrangement none of this covers is a robot whose signalling port is exposed to the internet
directly, by a port forward rather than through the bridge. Then there is no bridge to have
authenticated anything, and §4's LAN reasoning does not apply either, because the population that
can reach it is no longer the people in the building. That is a deployment mistake rather than a
design decision, and worth saying out loud precisely because nothing in the robot would notice.

### The hook, if it is ever wanted

`system.authenticate` — the method BLE already uses, added in `API_VERSION` v4. A control channel
would serve that one method and refuse the rest by name until it passes, with the PIN read from
`configd` over its unix socket rather than over the channel being authenticated.

It is named here so the answer exists, not because it is planned. The case for it is narrow: it
adds nothing to the bridged path, which is better authenticated already, and on the LAN it costs a
step per connection while proving only that the peer read a number printed on every robot. If it is
ever wanted, it is cheap — §5's routing table already needs a notion of which methods a transport
may reach, and "which methods before authentication" is the same table with a smaller subset rather
than a new mechanism.

## 5. The control channel is a pipe to the existing API

Frames on `control` are **JSON-RPC 2.0, one object per line**, which is what
[`duck-ipc-proto`](../../duck-ipc-proto/src/lib.rs) already defines and what `robotctl` and `btd`
already speak. Nothing new is invented: `mediad` routes a call to the unix socket of the service
that owns it and pumps replies back.

`btd` is the working precedent and should become the shared one:

| `btd` today | what `mediad` needs |
|---|---|
| `route.rs` — which calls may travel, which socket answers, which lane carries them | the same table, with a *different permitted subset* |
| `session.rs` — the `system.authenticate` gate | the same gate |
| `upstream.rs` — dial the sockets, timeout everything | the same |
| `framing.rs` — BLE MTU chunking | **not needed**; SCTP frames itself |

So three of the four files are transport-independent and one is not. **Lift the routing table into
something both transports use**, parameterised by which subset a transport may reach.

The property worth preserving is not the code, it is the exhaustive match. `route.rs` says it
plainly: adding a variant to `proto::Call` makes the file fail to compile, so a new method cannot
reach the radio because somebody forgot this file existed. A `_ => None` wildcard would deny new
methods silently, and the first symptom would be an app that cannot see a feature nobody remembered
to route. That guarantee has to hold **per transport**, or WebRTC becomes the hole in it.

### What WebRTC may reach, and why it is not BLE's subset

BLE's subset is narrow because the radio is slow and anyone within a few metres can talk to it.
Neither applies here, so WebRTC gets more — but "more" is not "everything", and two categories stay
out:

- **`system.pairingPin` and `system.setPairingPin`.** Not because they would compromise *this*
  transport — §4 leaves it open anyway — but because they authorise a **different** one. A LAN peer
  that can rewrite the pairing PIN can lock a phone out of BLE, which is the recovery path. Keeping
  the PIN off every network transport is the same rule that makes it unroutable to BLE itself.
- **`update.*` mutations.** For now only, and for a different reason than the PIN: applying an
  update restarts `mediad` and drops the session. Wanted later; §8 is what it will take.

### Replies are not correlated, deliberately

`btd` forwards whatever a socket emits without parsing it, and has a test pinning that: a
subscription is a stream of notifications on an open connection, and every one has to reach the
client. Correlating replies to requests would break exactly that. `mediad` inherits the same rule,
which also means **no per-method work in `mediad` when a method is added** — the pipe stays dumb,
and `duck-ipc-proto` stays the only place a method is defined.

The lane concept transfers too, and it is easy to assume it will not. Every daemon serves one
request at a time per connection, so `update.subscribe` followed by anything else on the same
connection hangs — the exact bug `app-path-design.md` §7 records. One datachannel is one ordered
stream with the same hazard, and `btd`'s answer works unchanged: route by method to a per-lane
socket, pump each socket back, never correlate.

## 6. Why `control`-only comes first, and what `teleop` will cost when it lands

`intents.rs` stores each intent in an `ArcSwap` and takes last-writer-wins. That is correct today
because every writer reaches it through a unix socket, where a later message cannot arrive before an
earlier one.

**A reliable, ordered datachannel keeps that true.** SCTP in that mode delivers in order by
definition, so intents arriving over `control` preserve the property `intents.rs` already depends
on. Starting with one channel is therefore not a compromise that stores up work — it means there is
no ordering problem to solve in the first version at all.

### What it costs instead, so nobody is surprised

Head-of-line blocking. On a reliable channel a lost packet stalls everything behind it, including
the control RPCs, so a bad link shows up as *everything* pausing rather than as a stale joystick.
Driving over `control` is fine at a modest rate and gets worse with rate and loss — which is
precisely why `architecture.md` §5.2 specifies a second channel, and why the answer to "the robot
feels laggy over a poor link" is teleop rather than tuning.

### And when teleop lands, it needs sequence numbers

**SCTP with `maxRetransmits: 0` reorders.** A twist from 80 ms ago can land after a fresher one and
win last-writer-wins, and the robot then drives on a stale command with nothing anywhere reporting a
problem. It is not a rare race: it is the normal behaviour of the channel, chosen deliberately.

So teleop frames carry a **monotonic sequence number per stream**, and the writer drops anything not
newer than what it last applied. This is a property of the *transport*, so it belongs in `mediad`
rather than in `robotd` — `robotd` should keep receiving intents whose ordering it can trust, which
is what lets `intents.rs` stay as simple as it is.

Worth writing down before the channel exists rather than after: the failure is silent, it looks like
bad tuning rather than a bug, and the fix is trivial if it is designed in and awkward if a stale
twist has to be diagnosed first.

The deadman needs nothing either way: `safety.gate(command, twist_age)` is already age-based, so a
partition stops the robot with no new code.

## 7. Reaching a robot that is not on your LAN

**The remote path is a bridge to the local signalling server, not a second design.** A relay
process connects outward to a rendezvous service and proxies the same protocol to
`ws://127.0.0.1:8443`. The robot's signalling server, session model, authorisation and control
channel are unchanged; DTLS-SRTP keeps media encrypted end to end even through a relay, which is
worth stating to clients plainly.

Two properties follow, and both are the reason for this shape:

- **Local mode never depends on the bridge.** If the rendezvous service is down, a LAN client still
  connects. Invariant 1 in `architecture.md` — local recovery stays independent — extends to media.
- **The bridge parses nothing.** It proxies the gst signalling protocol, which is the same protocol
  a LAN client speaks. That is the concrete payoff for using `webrtcsink` rather than `webrtcbin`:
  the protocol already exists, so the bridge is a relay rather than a translator.

- **The bridge authenticates, so the robot does not have to.** The relay connects *outward* holding
  an account token, and the service shows a client only the robots its account owns — so a bridged
  session is authorised on both sides before it arrives. §4 covers where that leaves the trust.

  A useful consequence: because the relay is a robot-side process connecting to loopback, the robot
  *can* tell a bridged peer from a LAN one by source address, even though it does not currently act
  on the difference. Nothing is foreclosed if that stops being true.

`reachy_mini` runs exactly this arrangement against a Hugging Face Space, with the robot
registering as a `producer` and the Space keeping a TTL lease refreshed by a heartbeat. Whether we
adopt that service, and how a robot is bound to an account, is out of scope here and stays out
until local mode works.

### The signalling protocol, for whoever writes the bridge

From `gst-plugins-rs` 0.15.3, `net/webrtc/protocol` — the wire is JSON with a `type` tag,
camelCase:

| peer → server | server → peer |
|---|---|
| `setPeerStatus` (`roles`, `meta`, `peerId`) | `welcome` (`peerId`) |
| `startSession` (`peerId`, optional `offer`) | `sessionStarted` (`peerId`, `sessionId`) |
| `endSession` (`sessionId`) | `startSession`, `endSession` |
| `peer` (SDP `offer`/`answer`, or `ice`) | `peer`, `error` (`details`) |
| `list`, `listConsumers` | `list` (`producers`), `listConsumers` (`consumers`) |

Roles are `producer`, `listener`, `consumer`. The robot is a `producer`; `meta` is free-form JSON
and is where a robot's identity goes.

## 8. Updating the robot over WebRTC: not yet, and what it will take

Not in the first permitted subset, and **that is a deferral rather than a principle** — a phone
updating a robot over WebRTC is wanted later, so this section is about what has to be true first
rather than why it must not be.

What makes it awkward today: applying an update restarts `mediad`, which drops the session the
client is watching progress on. `update-over-ble.md` records what "start an update and watch it"
failing silently already cost once, and a session that vanishes mid-update is the same shape of
problem. So the first subset leaves `update.*` mutations out and BLE stays the transport that
survives the restart. Read-only `update.*` calls are in from the start — seeing version and history
over a remote session is useful and costs nothing.

Two things have to change for the mutations, and both are small and specific:

- **The client has to survive the restart.** The protocol already supports it: progress is pushed
  as a JSON-RPC *notification*, which `duck-ipc-proto` documents precisely so a client that
  reconnects mid-update can resubscribe and keep receiving them. So the work is a client that
  reconnects and re-subscribes, not a change to the wire format.
- **`RobotRemoteSessionActive` has to get more specific.**
  `updater/src/preflight.rs::check_no_remote_session` refuses an update while a remote session is
  up, which is right when the session is a *bystander* — someone is on a telepresence call and
  should not have the robot restarted under them. It is wrong when the session is the *requester*.
  Nothing sets that flag true yet, so the distinction can be designed in rather than retrofitted:
  the check needs to know whether this update was asked for over the session it is about to drop.

Worth writing down now precisely because nothing sets the flag yet. The moment `mediad` reports
honestly, an update requested over WebRTC would refuse itself, and that would look like a bug in
the update path rather than a missing distinction here.

## 9. Authority: the premise this feature breaks, noted and not acted on

`intents.rs` says its slots are "single-writer in practice, so last-writer-wins means what it
says". That is true with one gamepad. **It stops being true the moment a pad and a remote peer both
drive**, and the failure is not a contest — it is two writers at 50 Hz interleaving into one slot,
producing a robot that obeys neither.

**Deliberately not solved here.** It is recorded so that a confusing robot has an explanation
waiting, and because the flag that eventually resolves it is cheaper to design before there are two
transports than after. `architecture.md` §6 owns the requirement — defined priority and handoff,
local physical able to preempt remote — and the roadmap has it in M6.

When it is time, the cheap answer is a **single-writer token**: one peer holds the right to write
intents, others are observers, and handing it over is explicit. That is much less than §6's full
arbitration and it removes the interleaving, which is the part that produces nonsense rather than
merely the wrong winner. Priority ordering — physical preempting remote without asking — can come
after, on top of the same token.

What this section is *for* until then: knowing that two simultaneous drivers is a known gap rather
than a mystery, and that the first symptom is a robot ignoring both inputs rather than obeying the
wrong one.

## 10. Building `mediad` at all

The `gstreamer-rs` crates are pkg-config crates, so cross-compiling them needs the *target's*
headers, `.pc` files and shared libraries on the developer's machine. `cargo board` cross-builds
from macOS with `cargo-zigbuild`, and `scripts/ci-cross-deps.sh` says of the one C dependency it
already has that it "is the cost of that one exception, and it is worth reading before adding
another". This is the second, and much larger.

**`scripts/cross-sysroot.sh` unpacks the robot's own Debian packages into a sysroot** — proven:
the full workspace cross-builds against it, and `gstreamer`, `gstreamer-app` and
`gstreamer-webrtc` all resolve at 1.26.2, the same version the board runs.

Three things about it worth knowing before touching it:

- **It serves the whole workspace, not just `mediad`.** `PKG_CONFIG_LIBDIR` *replaces*
  pkg-config's search path rather than adding to it, so a sysroot carrying only GStreamer breaks
  `padd` — whose `gilrs` needs libudev — and does it inside `libudev-sys`, nowhere near anything
  about media. Replacing is still right: `PKG_CONFIG_PATH` is additive to the *host's*, which is
  how pkg-config comes to answer with a macOS library and produce a binary that cannot run on the
  robot.
- **The package list is explicit, not resolved.** Walking Debian `Depends` from the obvious roots
  pulls 543 packages, because `libgstreamer-plugins-bad1.0-dev` declares every optional backend's
  dev package and the closure reaches Qt, Vulkan and OpenEXR. Nineteen packages satisfy what is
  actually needed.
- **A `-dev` package alone is not enough for anything actually linked.** It ships `libfoo.so` as a
  symlink onto the `libfoo.so.N` in the runtime package, so `-lfoo` needs both. Libraries that
  only appear in `Requires.private` need just the `-dev`.

The alternative was building `mediad` on an arm64 runner like the plugins in
[`media-bringup.md`](../project/media-bringup.md). Rejected because it splits the daemon build in
two and leaves nobody able to build `mediad` on a laptop — which for the crate that will need the
most iteration against real hardware is the wrong trade.

## 11. Deferred, with reasons

- **A WebSocket surface for server-side programs** (`architecture.md` §5.3). Same JSON-RPC, no
  media stack, `get_frame` returning a JPEG. It is a few dozen lines once §5's routing exists, and
  it is what makes "an LLM drives the robot" easy — but it is a second transport and the first one
  should work.
- **The `teleop` datachannel.** Not the near-term priority; §6 covers what deferring it removes,
  what it costs in the meantime, and the sequence numbers it will need.
- **Multi-peer video.** One media session at a time, plus control-only clients. Simulcast and
  encode-once-send-many are a real project.
- **Consent and the streaming indicator.** `architecture.md` §7 wants explicit per-session consent
  and a visible indicator, and is right that they are cheap now and expensive later. They need
  hardware that exists — an LED under software control — which is not yet established.
- **TURN.** LAN-only needs none. A bridge does, and it costs real bandwidth; that decision belongs
  with the rendezvous service, not here.
