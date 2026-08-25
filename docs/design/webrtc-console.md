# The WebRTC client: from a test page to the robot's console

Status: proposal · Date: 2026-08-25 · Owner: pierre

`mediad/webclient/index.html` proved the transport. This is what turns it into something a person
uses: served by the robot rather than by `python3`, found through the address `btd` already
broadcasts, and exercising the control surface `route.rs` actually permits.

Companion to [`remote-webrtc.md`](remote-webrtc.md), which owns the transport — signalling,
the session model, authorisation, and what a peer may call. Where the two touch, that page is the
owner and this one points at it.

**Nothing here is built.** The approach, written down before the first line, so the decisions are
arguable rather than implied by a diff.

## 0. What is wrong with it today

Not much, for what it was for. It answered "does a browser get video and a datachannel", and the
answer was yes. Six things are wrong with it as *a client*:

| | |
|---|---|
| it needs `python3 -m http.server` | a second tool and a second terminal, and the page is served from the laptop to talk to the robot |
| the URL is typed by hand | `ws://radxa-zero3.local:8443` — and `radxa-zero3` is the hostname on **every** board flashed from one image (`configd/src/main.rs` says so where it falls back to it), so two robots on one network collide |
| its comment block is mostly warnings | `file://` and Private Network Access, http not https — a header that had to be corrected twice already (`7f52a34`) |
| it exercises the protocol, not the robot | `route.rs` permits move, head, look, pose, mouth, do, sound, enable, init, relax, stop, subscribe, `tof.stream`, `pad.input`. The page offers `robot.health` and a JSON textbox |
| it is shipped nowhere | no `--include` names it, so a robot in the field has no client |
| it cannot say which robot it reached | the producer's `meta` is unset, so `list` returns an id and nothing else |

Every one of those is a step between a person and a working robot, and none of them is about
WebRTC.

## 1. `mediad` serves the page

An HTTP listener in `mediad`, one route, returning the page. `http://<robot>:8080/` and there is
nothing else to run.

This deletes four problems at once rather than one:

- **No `python3`.** The instruction becomes an address.
- **No URL to type.** The page defaults its signalling target to `ws://${location.hostname}:8443`
  — it was served by the robot, so it knows which robot it is talking to.
- **Private Network Access stops applying.** Chrome blocks a request from a *public or opaque*
  origin to a private address. A page served from `192.168.x` has a private origin, so the check
  that broke `file://` is not reached at all. The warning block in the header goes away because
  the failure it warns about cannot happen.
- **One version.** The page and the binary ship together (§1.2), so a client from a checkout can
  no longer be pointed at a robot from a release.

### 1.1 Which HTTP server

**`axum`.** It is already in `Cargo.lock` — `updater` uses it as a dev-dependency for its test
mirror — so the crate is known-good against this toolchain and the cross build.

The alternative is a hand-rolled HTTP/1.1 responder: this serves one file over one method, so it
is perhaps sixty lines. Rejected. Sixty lines of hand-written request parsing bound to
`0.0.0.0` is a parser exposed to everyone on the network, written to avoid a dependency that the
build already resolves — and `hyper` underneath `axum` is the most-read implementation of that
parser in the language. The dependency count is not the thing to optimise here.

`RestrictAddressFamilies=AF_UNIX AF_INET AF_INET6` in `mediad.service` already permits the
listener; nothing in the unit changes.

### 1.2 The page is embedded, not installed

`include_str!("../webclient/index.html")`, served from memory.

The alternative — install it under `/opt/robot/daemon/current/webclient/` and read it at
request time — costs an `--include` line in *three* places (`_build-release.yml`, `dev.yml`,
`scripts/dev-push.sh`), which `xtask`'s packaging tripwires exist to keep in step and which is
exactly the list that has drifted before. It also puts a filesystem read behind a network request
in a unit running `ProtectSystem=strict`, and it makes "which page is this robot serving" a
question with two answers.

Embedding costs a rebuild to change a stylesheet. That is the right trade for a page that is part
of the daemon's interface.

### 1.3 Two ports now, one port later — and why the later matters

`webrtcsink` owns the listener on 8443 (`run-signalling-server`, with only `-host` and `-port` to
say about it), so the page cannot be a route on it. Two ports: 8080 serves the page, 8443 stays
the signalling server. Nothing about the media path changes, which is the entire argument for
doing it this way first.

The one-port variant is to run the signalling server ourselves — upstream publishes it as a
library alongside the plugin — and point `webrtcsink`'s signaller at
`ws://127.0.0.1:8080/ws`. Then one `axum` server serves the page and the protocol on one origin.
That is more work and a second copy of the protocol version to keep in step with the `.so` we
ship, and it should not be in the first change.

**But it is where this ends up, and §3.4 is why.** A browser gives a page served over plain http
to a private address no microphone, and depending on the browser no gamepad either — those are
secure-context APIs, and `http://192.168.1.42` is not a secure context (`http://localhost` is,
which is precisely why nobody has hit this yet). Two-way audio is in `remote-webrtc.md` §2. The
robot will need to serve TLS, and `webrtcsink`'s built-in server offers no way to configure a
certificate, while an `axum` server offers the ordinary one. So the sequence is: two ports now,
our own signalling server when audio or a browser gamepad is wanted, TLS on the same day.

Worth writing down now rather than discovering it while wiring a microphone.

## 2. Finding the robot: `btd` already broadcasts the answer

The last mile is nearly built. `btd` files the robot's IPv4 in its advertisement under company id
`0xFFFF`, and `duck-btctl` already parses it — `Address::At`, `Unassigned`, `Unsaid`, three
answers rather than two, and `scan` prints it today.

So: **`duck-btctl ip`**, which resolves a robot by name (or `DUCK_ROBOT`) and prints the address
on stdout and nothing else — matching the split the tool already keeps, diagnostics on stderr and
data on stdout, so `open "http://$(duck-btctl ip):8080"` works. And **`duck-btctl open`**, which
does that for you.

Neither connects, bonds, or authenticates. It is an advertisement read, so it works on a robot
this laptop has never paired with, and it costs a scan.

The three-way `Address` already carries the right failure text, and this is where it pays:

- `Unassigned` — the robot has no network. The fix is `duck-btctl wifi connect`, and it has to be
  over BLE, because `net.connect` is refused over WebRTC by design (`route.rs`: "a robot that has
  never seen a network cannot be configured over that network").
- `Unsaid` — a release from before `btd` advertised an address. `duck-btctl wifi status` still
  reports it; updating puts it in the list.

That closes a loop worth naming: **BLE provisions the network and then hands you the URL for it.**
The two transports stop being alternatives and become a sequence.

`duck-btctl` is a stopgap for the phone app, so this stays thin — two commands, no new mechanism.
The durable halves are the ones that outlive it: `btd` broadcasting the address, and `mediad`
serving on a known port. The app will do the same two steps natively.

## 3. The page becomes a console

The permitted subset is large and almost none of it is reachable from the page. Reorganised
around what a person came to do:

| | |
|---|---|
| **header** | robot name, release, API version — from `hello` and `system.info`, sent automatically when the channel opens, not clicked |
| **video** | plus link quality from `getStats()`: bitrate, fps, loss, RTT. Today a stream that degrades is a picture that looks worse and a log that says nothing |
| **drive** | keys and an on-screen stick → `robot.move` at a fixed rate; drag on the video → `robot.look` |
| **posture** | `robot.enable`, `init`, `relax`, `stop`, `shutdown` — confirm on the last two |
| **do / sound** | the `Do` and `Sound` enums as menus |
| **telemetry** | `robot.subscribe` at 2 Hz into a live panel: mode, health |
| **console** | the raw JSON box, the log, and the two refusal buttons — collapsed, because they prove the route table rather than drive the robot |

Three constraints on it:

- **The stop button must not read as an e-stop.** `route.rs` permits `robot.stop` on the grounds
  that this channel is reliable and the deadman already stops the robot when intents stop
  arriving, and then says in as many words that the UI should not imply it is a physical e-stop.
  A label, not a big red circle.
- **A version difference is a banner, not a locked door.** `hello` reporting skew says so and the
  page keeps working — same rule `duck-btctl` settled on in #102.
- **Still one file, still no build step.** That constraint is why the client is runnable at all
  and it survives. If it outgrows one file it becomes three — `index.html`, `app.js`, `app.css`,
  three `include_str!`s, still no build step, still no npm.

Driving from the page is also the first real test of two claims `remote-webrtc.md` makes and
nothing has exercised: that the deadman stops the robot when a session drops (§6), and that
ordering over `control` is enough to keep `intents.rs` honest (§6 again). Both are cheap to
believe and expensive to be wrong about.

## 4. The robot names itself before the session starts

`webrtcsink` takes a `meta` structure and the signalling server hands it to every peer in `list`
— the page already logs it and it is empty. Fill it from `configd`: name, serial, release,
`API_VERSION`.

Small, and it pays three times: the page can name the robot before starting a session, a client
that finds two producers can say which is which, and the rendezvous service in §7 needs exactly
this field to route on. Cheapest item here.

## 5. What it opens, but is not

`remote-webrtc.md` §11 defers "a WebSocket surface for server-side programs — same JSON-RPC, no
media stack, `get_frame` returning a JPEG", and calls it a few dozen lines once §5's routing
exists. Once there is an `axum` server in `mediad`, it is a route on a server that already runs,
and the frame is already there: `_frames` in `main.rs` is the raw NV12 tap off the tee, and
nothing reads it yet.

Named so the shape is visible, not proposed here. It is a second transport and the first one
should be good.

## 6. Order

Four changes, each of which stands alone and lands separately:

1. **Serve the page, default the URL to `location.hostname`.** Deletes the python instruction and
   the warning block. Small, and everything else is nicer on top of it.
2. **Producer `meta`.** Smaller still, and independent.
3. **`duck-btctl ip` / `open`.** Client-side only, touches no daemon.
4. **The console.** The large one, done last, on a page that is already reachable.

## 7. Not doing

- **A gate on the console.** §4 of `remote-webrtc.md` owns that decision and nothing here changes
  its terms. A `--no-web` flag to turn the page off is worth having for the home case that section
  flags — one flag, not a mechanism.
- **A JS framework, a bundler, or `gstwebrtc-api`.** The page speaks the protocol by hand because
  a client that needs npm is a client nobody runs. Still true at four times the size.
- **Serving over TLS in this change.** §1.3 says when, and why it is not now.
