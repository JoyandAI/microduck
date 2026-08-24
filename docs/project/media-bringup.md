# Media bring-up on hardware

What a Radxa Zero 3W does about video. Everything below was observed on a board, not inferred —
where something is still an assumption it says so.

## What this settles

`mediad` needs hardware H.264: software encode is not a slower option, it is not an option.
`jpegenc` alone cannot hold 30 fps at 640x480 on this SoC
(`microduck_runtime/src/camera.rs:500`), and H.264 costs more per frame than JPEG, on four
Cortex-A55s that `robotd`'s 50 Hz control loop already shares.

**The VPU encodes H.264, the bitstream is valid, and the encoder is reached through Rockchip's
MPP rather than V4L2.** Two GStreamer plugins have to be built from source to use any of it, and
neither is blocked by anything unknown.

| | |
|---|---|
| VPU encodes 720p H.264 | yes — 60 frames, 428 KB, via `mpi_enc_test` |
| Bitstream valid on this kernel | yes — clean `avdec_h264` decode, High profile level 4, 4:2:0 8-bit |
| Reached through | `/dev/mpp_service` (Rockchip MPP). **Not** V4L2 M2M |
| GStreamer plugin ABI across 1.x | not a risk — a 1.14-built plugin registers cleanly in 1.26.2 |
| `mpph264enc` element | must be built (§ [What has to be built](#what-has-to-be-built)) |
| `webrtcsink` / `webrtcsrc` | must be built, separately |

## What the board has

Nothing media-related is installed on a provisioned board. `scripts/setup-gstreamer.sh` installs
it and reports what the hardware can do; that script is the executable form of this page, and it
is where a command someone needs again should end up.

GStreamer comes from **plain Debian trixie** — `apt-cache policy` shows `deb.debian.org` and
`security.debian.org` with no Armbian multimedia overlay, so the archive's versions apply
exactly:

| package | version |
|---|---|
| `gstreamer1.0-plugins-bad` (has `webrtcbin`) | 1.26.2-3+deb13u3 |
| `libgstreamer-plugins-bad1.0-dev` (has `gstreamer-webrtc-1.0.pc`) | 1.26.2-3+deb13u3 |
| `gstreamer1.0-nice` | 0.1.22-1 |
| `gstreamer1.0-plugins-rs` | **does not exist in any Debian suite** |

The kernel is `6.1.115-vendor-rk35xx`. That matters twice: the camera's MIPI-CSI ISP capture
driver exists only on Armbian's vendor branch, and so do the VPU nodes. `setup-board.sh` already
installs that kernel — for the audio codec's I²S tree, not for video — so the prerequisite was
met before anyone asked for it. A stray `apt upgrade` that pulls the `current` kernel and
repoints `/boot` takes both away.

## The encoder is MPP, not V4L2

`v4l2h264enc` is absent and `/dev/video*` is empty. Neither is a fault:

- On a Rockchip BSP kernel the VPU is exposed as `/dev/mpp_service`, not as a V4L2 M2M encoder.
  `v4l2h264enc` is registered by `gstreamer1.0-plugins-good` only when it finds an encoder node,
  so its absence is the expected shape here rather than a missing package.
- No `/dev/video*` at all is also exactly what an **unattached camera** looks like — the rkisp
  capture nodes appear only once a sensor is probed.

This is worth stating plainly because it is the branch point. Had the kernel exposed a V4L2
encoder, hardware H.264 would have needed nothing out of tree at all.

### The permission trap

`/dev/mpp_service` arrives as `crw------- root root`, mode 0600. A non-root process cannot open
it — and **`mpi_enc_test` against it writes an empty file and exits 0**. No error, no log line.
A zero exit status is therefore evidence of nothing; the file size is the evidence.

`mediad` will run as its own user, like every other daemon here — `tofd` rides into `i2c`,
`padd` into `input`, `btd` into `bluetooth` — so the VPU needs the same treatment: a udev rule
giving the node a group, and `SupplementaryGroups=` on the unit.
`scripts/setup-gstreamer.sh` installs that rule (`99-robot-mpp.rules`, group `video`, mode
0660), following the shape of `configure_tof`'s i2c rule in `setup-board.sh`.

`video` rather than `robot`: `robot` gates the IPC sockets *we* define
([`app-path-design.md`](../design/app-path-design.md) §, the socket-mode-plus-group layering). A
kernel device node is not ours to redefine, and `video` is the distro convention for this device
class, so a developer with `gst-launch` gets in the same way `mediad` does.

## What Radxa's pool provides

Rockchip MPP is not in Debian. Radxa publish it as a GitHub Pages apt repo, and the packages are
taken as **direct `.deb` downloads** rather than by adding the repo to `sources.list` — which is
the route `microduck_runtime/radxa_setup/setup_rkaiq.sh` already uses on this board for
`rkaiq_3A_server`. Base: `https://radxa-repo.github.io/bullseye/pool/main`.

| package | version | why |
|---|---|---|
| `m/mpp/librockchip-mpp1` | 1.5.0-1 | the MPP userspace library |
| `m/mpp/librockchip-vpu0` | 1.5.0-1 | `rockchip-mpp-demos` depends on it at exactly this version |
| `m/mpp/rockchip-mpp-demos` | 1.5.0-1 | `mpi_enc_test` — proves the VPU with no GStreamer involved |
| `m/mpp/librockchip-mpp-dev` | 1.5.0-1 | headers, to build the encoder plugin |
| `libr/librga/librga2` | 2.2.0-1 | Rockchip 2D accelerator; the rockchip plugin depends on it |
| `libr/librga/librga-dev` | 2.2.0-1 | headers, same build |
| `g/gstreamer1.0-rockchip/gstreamer1.0-rockchip1` | 1.14-4 | the MPP GStreamer plugin — see below |

These are bullseye builds and they configure cleanly against glibc 2.41 on trixie.

**`dpkg -i` resolves nothing**, because these do not come from a configured apt source. Every
missing dependency is an unconfigured package rather than an install that repairs itself, so each
set has to name its full closure. That cost three round trips to learn.

### Radxa's prebuilt plugin is *not* decode-only, and this page said it was

`gstreamer1.0-rockchip1_1.14-4` installs and registers cleanly in GStreamer 1.26.2, showing
exactly `mppvideodec` and `mppjpegdec`. That reads as "no encoders", and this page claimed it —
wrongly. `strings` on its `.so` lists `mpph264enc`, `mpph265enc`, `mppjpegenc` and `mppvp8enc`.
They are all there.

**The permission trap is the whole explanation, and it produced four separate misleading results
before that was clear:**

| what it looked like | what was actually true |
|---|---|
| `mpi_enc_test` wrote nothing and **exited 0** | the node was unopenable; a zero exit says nothing |
| Radxa's deb was decode-only | it has every encoder |
| a third-party 1.14-8 deb still showed no `mpph264enc` | same cause again |
| our own CI build lists only the two decoders | a container has no `/dev/mpp_service` either — expected, not a failure |

An MPP plugin **registers its decoders unconditionally, and probes MPP before registering its
encoders.** With `/dev/mpp_service` at `0600 root:root` the probe fails silently, so the encoders
are omitted from a plugin that contains them perfectly well.

So a plugin listing only decoders is evidence about the *device node*, not about the plugin, and
`gst-inspect-1.0 mpph264enc` means nothing until the udev rule is in place.

One thing that install did prove, and it stands: **a plugin built against GStreamer 1.14 registers
without complaint in 1.26.2.** Plugin ABI was the stated reason to fear a source build, and it is
not the risk.

## What has to be built

Two plugins, for two unrelated reasons. Neither substitutes for the other.

| plugin | source | gives | why it cannot be installed |
|---|---|---|---|
| `gstreamer-rockchip` | [`JeffyCN/mirrors`](https://github.com/JeffyCN/mirrors) branch `gstreamer-rockchip`, meson | `mpph264enc` — the hardware encoder | Debian has no Rockchip encoder at all. Radxa's build *does* have them, so this one is about a pin we control, dropping `libx11-6`, and riding along with the plugin below |
| `gst-plugin-webrtc` | [`gst-plugins-rs`](https://gitlab.freedesktop.org/gstreamer/gst-plugins-rs) at 0.15.3, cargo-c | `webrtcsink`, `webrtcsrc` | `gst-plugins-rs` is packaged in **no** Debian suite |

0.15.3 rather than the 0.14.5 the `reachy_mini` SDK documents: 0.14.5 is the floor that matters —
below it a `webrtcsink` deadlock fix between remote description and ICE handling is missing, which
presents as a client spinning forever on "connecting" — and 0.15.3 is simply newer. Both series
declare a GStreamer `v1_22` feature floor and the robot runs 1.26.2, so the newer one costs
nothing.

`webrtcbin` **is** installed, from `gstreamer1.0-plugins-bad`. So a WebRTC session is reachable
today without the second build — at the cost of implementing the signalling protocol ourselves.
`webrtcsink` is preferred because its signalling protocol is what a relay proxies, which is what
makes a central signaling server reusable.

### Why not a prebuilt one

The hardware, the kernel driver and MPP's userspace library all work with **nothing compiled** —
`mpi_enc_test` came out of a deb and encoded 720p H.264 on the first try. What is missing is only
the *GStreamer binding*: a plugin that wraps `librockchip-mpp` as an element a pipeline can use.
`mpi_enc_test` is a standalone program; GStreamer has no idea it exists. The same shape as ONNX
Runtime here — `libonnxruntime.so` is installed from a tarball with nothing compiled, and `ort` is
the binding that makes it reachable.

Prebuilt bindings:

| source | what it has |
|---|---|
| Radxa `bullseye` pool | `gstreamer1.0-rockchip1_1.14-4` — installed and inspected: `mppvideodec` + `mppjpegdec` only |
| Radxa `rk3588s2-bookworm` pool | the same `1.14-4`, byte-identical |
| [`numbqq/gstreamer-rockchip-debs`](https://github.com/numbqq/gstreamer-rockchip-debs) | `1.14-8` — **has every encoder** |

The last one is worth trying before building anything. Its `bookworm/arm64/<board>/` entries are
symlinks into `jammy/arm64/`, so it is an Ubuntu 22.04 build, from
`rockchip-linux/gstreamer-rockchip` (now 404) with Jeffy Chen as maintainer — the same upstream
Radxa built, at a revision with the encoders enabled. `mpph264enc`, `mpph265enc`, `mppjpegenc`
and `mppvp8enc` are all present in the `.so`.

Its `DT_NEEDED` is satisfied by what a board already has after the debs above:
`librockchip_mpp.so.1`, `librga.so.2`, `libgstreamer-1.0.so.0`, `libgstvideo`,
`libgstallocators`, `libgstpbutils`, `libdrm2`, `libglib2.0-0`, `libx11-6`, `libc6 >= 2.33`
against glibc 2.41. Nothing in it is RK3588-specific — the SoC differences live inside MPP, not
the plugin — and `Depends` bounds GStreamer only from below (`>= 1.14`).

Ours are built in [`microduck-gst-plugins`](https://github.com/pollen-robotics/microduck-gst-plugins)
— a repository of its own, deliberately:

- **Not on the board.** An RK3566 compiles the Rust half far too slowly to wait for.
- **Not cross-compiled either.** The daemon cross-builds with `cargo-zigbuild`, and
  `scripts/ci-cross-deps.sh` says outright that its one C dependency "is the cost of that one
  exception, and it is worth reading before adding another". GStreamer would be a much larger
  second one, and both routes — x86 multiarch, or a sysroot with a meson cross file — link against
  an approximation of the target.
- **Natively, on an arm64 runner in a `debian:trixie` container**, which is the robot's own
  userland. Nothing is approximated. arm64 runners are free on public repositories.
- **Public** for a second reason that matters more: the download happens during provisioning and
  from the updater's `preinstall` hook, which runs with a cleared environment and **no token**. The
  same arrangement the daemon already relies on for ONNX Runtime.

It builds both plugins, pins upstream by commit or tag in one `pins.env`, disables `rkximage` and
`kmssrc` (the X11 and KMS sinks in the same tree — a headless robot needs neither, and they are why
the prebuilt Radxa deb depends on `libx11-6`), and publishes a tarball plus its sha256 with a
`MANIFEST` naming the exact upstream ref per plugin. That manifest is the thing the third-party deb
could not answer.

Two traps it guards, both found by reading the trees rather than by failing:
`gst/rockchipmpp/meson.build` ends in `if not mpp_dep.found() → subdir_done()`, so a missing
`librockchip-mpp-dev` makes meson **skip the plugin and succeed**; and `dpkg -i` resolves nothing
for direct `.deb` downloads, so the Radxa closure is installed in one call.

**`mediad.service` must set `GST_PLUGIN_PATH`.** The plugins install to
`/usr/local/lib/gstreamer-1.0`, which GStreamer does **not** search by default — its built-in
path is the distro's `/usr/lib/aarch64-linux-gnu/gstreamer-1.0`, and that directory is
deliberately avoided so an `apt` operation cannot replace or remove them. So the unit needs

```
Environment=GST_PLUGIN_PATH=/usr/local/lib/gstreamer-1.0
```

alongside the `SupplementaryGroups=video` the VPU node needs. Both are easy to forget and both
present the same way at runtime: the encoder simply does not exist, with nothing saying why.

`scripts/setup-gstreamer.sh` consumes it at a **pinned** version — never "latest". Two
provisioning runs a day apart producing different plugins, with nothing recording which, is an
unreproducible media bug waiting to happen. The pin lives in
`[workspace.metadata.gst-plugins]` in `Cargo.toml`, the script carries the literal because it is
fetched standalone with `curl`, and an `xtask` test asserts they agree — the same arrangement, and
the same reason, as `ONNX_VERSION`.

**Trying the third-party deb is not the same as depending on it.** It is one person's per-board dump with no
provenance we control, and it vanishes if that repository does. What it buys cheaply is the answer
to the only real question about the build — whether this plugin works against *our* MPP and
GStreamer versions — and if it does, building the same source ourselves is de-risked rather than
unnecessary. A pinned build of our own is still where this should end up.

**The build could be avoided entirely** by calling MPP's C API from `mediad` over Rust FFI, which
`mpi_enc_test` proves works. That trades a meson build for hand-written and hand-maintained
bindings to a vendor library, which is the worse side of the trade — but it is a real option, not
a dead end, if the plugin turns out to fight GStreamer 1.26.

### The upstream

`rockchip-linux/gstreamer-rockchip` is gone (404). `JeffyCN/mirrors@gstreamer-rockchip` is the
live mirror — last commit 2026-05-21 — and `gst/rockchipmpp` holds `gstmpph264enc.c`,
`gstmpph265enc.c`, `gstmppjpegenc.c`, `gstmppvp8enc.c`. Fork soup exists around it; whichever
fork and tag is used has to be pinned and recorded, for the same reason
`gst-plugin-webrtc` is pinned to ≥ 0.14.5 (below).

### The pin on `gst-plugin-webrtc`

**0.14.5 or newer, not 0.14.4.** The earlier tags miss a `webrtcsink` deadlock fix between remote
description and ICE handling, which presents as a client spinning forever on "connecting".
`reachy_mini`'s SDK install doc records it; note that `reachy-mini-desktop-app` vendors 0.14.4
and is therefore on the wrong side of that line.

Pollen already vendor this plugin for **x86_64** — built natively with `cargo cinstall`,
stripped, committed per-arch, and consumed by CI pinned to a commit ref plus a sha256. An
`aarch64/` sibling in the same place may be less total work than a second pipeline here.

### Where a built plugin belongs

In the daemon release payload, with `GST_PLUGIN_PATH` pointing into `current` — not in apt, and
not in `/opt` on each machine.

The plugin version and `mediad`'s code are entangled: the 0.14.5 story above is exactly a case
where a plugin version determines whether the daemon needs a workaround. A skew is a `mediad`
bug, so it wants `mediad`'s lifecycle — atomic swap, rollback, health gate. `librockchip-mpp` is
the opposite: a system library paired with the *kernel*, wanted by anything touching the VPU,
and belongs in the package manager.

## Measured, and not

**Measured on the board:** GStreamer 1.26.2 and its origin; `webrtcbin` present;
`webrtcsink`/`webrtcsrc` absent; `v4l2h264enc` absent and no `/dev/video*`; `/dev/mpp_service`
present at 0600 root:root; `mpi_enc_test` silently writing nothing as non-root and 428 KB as
root; that bitstream decoding clean as High/4.0; the Radxa debs' dependency closure; the rockchip
plugin loading in 1.26.2 with two decode elements.

**The encode path is closed, on hardware, end to end.** In order:

1. `v1` fetched from the public release, sha256 verified, installed to
   `/usr/local/lib/gstreamer-1.0`.
2. `gst-inspect-1.0 mpph264enc` answers `provided-by
   /usr/local/lib/gstreamer-1.0/libgstrockchipmpp.so` — our build. The third-party deb was
   removed first so the answer could be attributed to something.
3. It **encodes**: `videotestsrc ! mpph264enc profile=baseline header-mode=each-idr bps=2000000 !
   h264parse ! filesink` produced 476 KB for 60 frames of 720p in **0.44 s wall**, source
   generation and pipeline setup included — comfortably faster than realtime. The result decodes
   clean through `avdec_h264`.
4. It works **without root**: after a udev rule puts `/dev/mpp_service` at `660 root:video` and
   the user joins `video`, `gst-inspect` answers as that user. Which is the case `mediad` is in,
   and every check before this one had been under `sudo`.

**Not measured.** No camera has been attached, so the capture half — rkisp, rkaiq, the
`v4l2-ctl`-into-`appsrc` question — is untested here; what is known comes from
`microduck_runtime` on the same hardware. `webrtcsink` registers but has never negotiated with a
peer. And `mediad` does not exist, so nothing has been assembled into a pipeline that runs as a
service.

One thing still worth reading off a real stream: whether the `baseline` profile above lands as
**Constrained** Baseline. `h264parse` distinguishes them in its caps, so
`gst-launch-1.0 -v filesrc location=… ! h264parse ! fakesink` names it. There is no camera attached, so the whole capture path is untested on
this board; what is known about it comes from `microduck_runtime`, which drove an IMX219 on the
same hardware. `mediad` does not exist, so no pipeline has been assembled end to end.

## Two things the pipeline will have to decide

**Capture cannot use `v4l2src`.** The rkisp driver hands it a 2-buffer pool and it requeues too
slowly, dropping every third frame — ~20 fps from a 30 fps sensor, with "lost frames detected".
`v4l2-ctl --stream-mmap` sustains the full rate, so `microduck_runtime` captures with it and
pipes raw frames into a `fdsrc` pipeline (`camera.rs:487`). `mediad` needs either that subprocess
shape or its own V4L2 mmap loop feeding `appsrc`.

**Four `mpph264enc` properties are pipeline decisions, not defaults to inherit.** Read off the
element on the board:

| property | default | what `mediad` should set | why |
|---|---|---|---|
| `profile` | `high` | **`baseline`** | WebRTC's interoperable floor is Constrained Baseline (`profile-level-id 42e01f`). Current browsers negotiate High; older peers do not. The element offers `baseline`/`main`/`high`, so this is a one-word decision rather than the open question it looked like |
| `header-mode` | `first-frame` | **`each-idr`** | SPS/PPS in the first frame *only* means a peer that joins later — or loses that packet — never decodes anything. `reachy_mini`'s Pi pipeline sets exactly this on `v4l2h264enc` via `repeat_sequence_header=1`; same requirement, different spelling |
| `rotation` | `0` | **`180`** on the alpha | the IMX219 is mounted upside down. `microduck_runtime` fixes it with `videoflip method=rotate-180` — a full CPU pass over every frame, on the SoC `robotd` shares. The encoder does it in hardware for nothing |
| `bps` | `0` (auto) | an explicit target | `rc-mode` already defaults to `cbr`, which is what a lossy link wants; the bitrate should not be left to "auto calculate" |

Two things that turn out to need no decision:

- **There is no B-frame knob at all**, so §5.5's "no B-frames" requirement is satisfied by
  construction rather than by configuration.
- **The sink pad accepts `NV12`**, which is exactly what the rkisp capture path emits. No
  `videoconvert`, and no RGA colour conversion, between capture and encode.

Keyframes should come from `min-force-key-unit-interval` rather than a periodic `gop`: WebRTC
drives them from the peer's PLI, and `gop` defaults to one IDR per second whether anybody needed
one or not.

One thing to verify once there is a stream rather than assume: the enum value is `baseline` (66),
while WebRTC negotiates *Constrained* Baseline. A Baseline stream that avoids FMO, ASO and
redundant slices is what a constrained-baseline decoder expects, and MPP has no reason to emit
those — but the SPS constraint flags are worth reading off a real capture.

`webrtcsink` accepts pre-encoded H.264 on its sink pad, so the pipeline is
`appsrc ! mpph264enc ! h264parse ! webrtcsink` and the encoder choice never reaches
negotiation.
