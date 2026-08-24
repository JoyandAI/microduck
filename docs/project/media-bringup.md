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
| `g/gstreamer1.0-rockchip/gstreamer1.0-rockchip1` | 1.14-4 | **decode only** — see below |

These are bullseye builds and they configure cleanly against glibc 2.41 on trixie.

**`dpkg -i` resolves nothing**, because these do not come from a configured apt source. Every
missing dependency is an unconfigured package rather than an install that repairs itself, so each
set has to name its full closure. That cost three round trips to learn.

### Radxa's prebuilt GStreamer plugin is decode-only

`gstreamer1.0-rockchip1_1.14-4` installs, registers cleanly in GStreamer 1.26.2, and provides
exactly two elements: `mppvideodec` and `mppjpegdec`. No encoders — 1.14.4 predates them.

Install it anyway if hardware **decode** is wanted (a two-way telepresence session decodes the
peer's video), and for what it proves: **a plugin built against GStreamer 1.14 registers without
complaint in 1.26.2.** Plugin ABI was the stated reason to fear building this from source, and it
is not the risk.

## What has to be built

Two plugins, for two unrelated reasons. Neither substitutes for the other.

| plugin | source | gives | why it cannot be installed |
|---|---|---|---|
| `gstreamer-rockchip` | [`JeffyCN/mirrors`](https://github.com/JeffyCN/mirrors) branch `gstreamer-rockchip`, meson | `mpph264enc` — the hardware encoder | Debian has no Rockchip encoder at all, and Radxa's build predates the encoders |
| `gst-plugin-webrtc` | [`gst-plugins-rs`](https://gitlab.freedesktop.org/gstreamer/gst-plugins-rs), cargo-c | `webrtcsink`, `webrtcsrc` | `gst-plugins-rs` is packaged in **no** Debian suite |

`webrtcbin` **is** installed, from `gstreamer1.0-plugins-bad`. So a WebRTC session is reachable
today without the second build — at the cost of implementing the signalling protocol ourselves.
`webrtcsink` is preferred because its signalling protocol is what a relay proxies, which is what
makes a central signaling server reusable.

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

**Not measured.** Nobody has built either plugin yet, so `mpph264enc` has never run here — only
MPP's own test binary has. There is no camera attached, so the whole capture path is untested on
this board; what is known about it comes from `microduck_runtime`, which drove an IMX219 on the
same hardware. `mediad` does not exist, so no pipeline has been assembled end to end.

## Two things the pipeline will have to decide

**Capture cannot use `v4l2src`.** The rkisp driver hands it a 2-buffer pool and it requeues too
slowly, dropping every third frame — ~20 fps from a 30 fps sensor, with "lost frames detected".
`v4l2-ctl --stream-mmap` sustains the full rate, so `microduck_runtime` captures with it and
pipes raw frames into a `fdsrc` pipeline (`camera.rs:487`). `mediad` needs either that subprocess
shape or its own V4L2 mmap loop feeding `appsrc`.

**The H.264 profile has to be set deliberately.** The VPU emits **High** profile; WebRTC's
interoperable floor is Constrained Baseline (`profile-level-id 42e01f`). Current browsers
negotiate High and older peers do not. High also permits B-frames, which are latency poison
against [`architecture.md`](../design/architecture.md) §5.5's <200 ms glass-to-glass target —
Rockchip's encoders do not normally emit them, but that wants asserting rather than assuming.

`webrtcsink` accepts pre-encoded H.264 on its sink pad, so the pipeline is
`appsrc ! mpph264enc ! h264parse ! webrtcsink` and the encoder choice never reaches
negotiation.
