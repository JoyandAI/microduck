#!/bin/sh
# Install the GStreamer stack `mediad` needs, and report what this board can actually encode.
#
# Split from `setup-board.sh` for the reasons `migrate-network.sh` is split from it — lifetime
# and risk are different:
#
#  1. **Different lifetime.** The overlay fix and the ONNX install in `setup-board.sh` are
#     what makes a board a robot: without them there is no motor bus and no policy. GStreamer
#     is what makes it a *camera*, and a board doing walking work needs none of it. Around
#     100 MB of media stack installed on every board before anything uses it is a cost with
#     no payer, so this is invoked rather than assumed.
#  2. **Different question.** Everything in `setup-board.sh` either works or fails. Half of
#     this script's value is the report at the end, which answers a question nothing else on
#     the board answers: *can this kernel encode H.264 in hardware, and through which
#     element*. That decides whether `mediad` streams video at a usable frame rate or cooks
#     the CPU `robotd`'s control loop runs on, and it is worth being able to ask repeatably on
#     any board rather than once, by hand, in a bring-up session nobody wrote down.
#
#   sudo sh /tmp/setup-gstreamer.sh
#   sudo /usr/local/sbin/robot-setup-gstreamer          # later, to re-check
#
# Full paths, because this advice gets copy-pasted and /tmp does not survive a reboot. The
# first run leaves a copy at the second path.
#
#   --dev     also install the headers and pkg-config files needed to *build* against
#             GStreamer — `gst-plugin-webrtc` on this board, or an aarch64 sysroot to
#             cross-build `mediad` from a laptop. Not wanted on a shipped robot.
#   --help
#
# Idempotent, and safe to re-run: apt is only invoked for packages that are missing, and
# nothing here needs a reboot.
#
# Radxa Zero 3W on Armbian, Debian 13 (trixie) userland. `apt-cache policy` on a provisioned
# board shows GStreamer coming from deb.debian.org and security.debian.org with no Armbian
# multimedia overlay, so these are plain Debian packages at 1.26.x.
set -eu

SELF=/usr/local/sbin/robot-setup-gstreamer

# Whether to install the -dev packages. Off by default: a shipped robot loads plugins, it does
# not compile them, and the headers are the larger half of the install.
WANT_DEV=0

# Where a hand-built out-of-tree plugin goes until `mediad` ships its own.
#
# `gst-plugins-rs` — which is what `webrtcsink`/`webrtcsrc` come from — is packaged in **no**
# Debian suite: not trixie, not backports, not sid. So the plugin is either built here or
# shipped inside the daemon release, and until `mediad` exists this is where a manual build
# lands. GStreamer does not scan it by default, hence the GST_PLUGIN_PATH advice in the report.
GST_EXTRA_PLUGIN_DIR=/usr/local/lib/gstreamer-1.0

# What the encoder probe looks at. Variables rather than literals for the reason
# `setup-board.sh` makes CMDLINE one: the interesting states of this check are a board that
# has no VPU node and a board on the wrong kernel, and those are exactly the states you least
# want to discover the check is wrong in. As variables they can be pointed at a fixture.
MPP_SERVICE="${MPP_SERVICE:-/dev/mpp_service}"
VIDEO_GLOB="${VIDEO_GLOB:-/dev/video*}"
KERNEL="${KERNEL:-$(uname -r)}"

# Runtime packages, and why each one is here rather than "the usual set".
#
#   gstreamer1.0-tools          gst-inspect-1.0 / gst-launch-1.0. The report below *is*
#                               gst-inspect, and a media fault on a robot in someone's house
#                               is diagnosed with what is already installed.
#   gstreamer1.0-plugins-base   videoconvert, videoscale, videorate, opusenc.
#   gstreamer1.0-plugins-good   videoflip, jpegenc, and the video4linux2 plugin — which is
#                               where `v4l2h264enc` lives if this kernel exposes an encoder.
#   gstreamer1.0-plugins-bad    webrtcbin, the DTLS/SRTP elements it needs, h264parse,
#                               rawvideoparse.
#   gstreamer1.0-nice           ICE. webrtcbin negotiates nothing without it.
#   libnice10                   pulled in by the above; named so a partial install is legible
#                               in the report rather than showing up as "webrtcbin hangs".
#   gstreamer1.0-plugins-ugly   x264enc. Software H.264, GPL — the interim encoder until the
#                               hardware path is settled, and the report says plainly when it
#                               is the only one present.
#   v4l-utils                   v4l2-ctl and media-ctl. Not optional for capture on this
#                               board: gstreamer's own v4l2src is handed a 2-buffer pool by
#                               the vendor rkisp driver and requeues too slowly, dropping
#                               every third frame (~20 fps from a 30 fps sensor), so frames
#                               are captured with `v4l2-ctl --stream-mmap` and piped into
#                               gstreamer. Measured on this hardware — see
#                               `microduck_runtime/src/camera.rs:487`.
#
# Deliberately absent:
#
#   gstreamer1.0-libcamera / libcamera-*   libcamera's mainline rkisp1 pipeline handler does
#                               not drive the *vendor* rkisp this board's camera needs, so it
#                               enumerates nothing here. Installing it buys an element that
#                               finds no camera (`microduck_runtime/radxa_setup/setup.md`).
#   gstreamer1.0-plugins-rs     does not exist in Debian. See GST_EXTRA_PLUGIN_DIR above.
#
# **This list is meant to shrink.** It was assembled from what the pipeline in
# `microduck_runtime/src/camera.rs` uses plus what `webrtcbin` needs to negotiate, which is a
# reasoned guess and not a measurement — `mediad` does not exist yet to be profiled against it.
# Re-read it whenever `mediad`'s pipeline changes shape and drop what nothing loads: every
# package here is disk on a robot, an apt dependency during provisioning, and a security update
# somebody has to care about. `gst-inspect-1.0 --plugin` names what a plugin actually provides,
# which is the check to run before defending an entry.
RUNTIME_PKGS="gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good
gstreamer1.0-plugins-bad gstreamer1.0-nice libnice10 gstreamer1.0-plugins-ugly v4l-utils"

# Build packages. `libgstreamer-plugins-bad1.0-dev` is the load-bearing one: it carries
# `gstreamer-webrtc-1.0.pc`, which is what both `cargo cinstall -p gst-plugin-webrtc` and
# `mediad`'s `gstreamer-webrtc-sys` pkg-config against. The rest is what a Rust cdylib build
# needs to link at all.
DEV_PKGS="pkg-config build-essential libssl-dev libgstreamer1.0-dev
libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev"

say()  { printf '\033[1m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[33mwarning:\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[31merror:\033[0m %s\n' "$*" >&2; exit 1; }

usage() {
    # Steps, not rationale. Why this is a separate script is in the header above, for whoever
    # edits it; someone running --help wants to know what to type.
    cat <<'EOF'
Install the GStreamer stack mediad needs, and report what this board can encode.

  sudo sh /tmp/setup-gstreamer.sh          runtime packages, then the report
  sudo sh /tmp/setup-gstreamer.sh --dev    also the headers, to build against GStreamer
  sudo /usr/local/sbin/robot-setup-gstreamer    re-run later, to re-check the report

Idempotent, and needs no reboot. The first run leaves the copy at that third path.
EOF
    exit 0
}

parse_args() {
    while [ $# -gt 0 ]; do
        case "$1" in
            --dev)  WANT_DEV=1 ;;
            --help|-h) usage ;;
            *) die "unknown argument: $1
  Run with --help for what this takes." ;;
        esac
        shift
    done
}

check_environment() {
    # No path in the message: whatever the operator just typed is what needs `sudo` in front.
    [ "$(id -u)" = 0 ] || die "run as root — re-run that same command with sudo"

    arch="$(uname -m)"
    [ "$arch" = aarch64 ] || die "this targets aarch64 boards, and this box is ${arch}"

    command -v apt-get >/dev/null 2>&1 \
        || die "no apt-get — this expects a Debian/Armbian userland"
}

# Leave a copy behind, so re-checking after a kernel change does not need a re-fetch.
#
# Same reasoning as `setup-board.sh`: /tmp is wiped, and the encoder question is one you come
# back to — a vendor-kernel upgrade is exactly the event that changes the answer.
persist_self() {
    case "$0" in
        sh|-sh|bash|-bash|/dev/fd/*|/proc/self/fd/*) return 0 ;;
    esac
    [ -f "$0" ] || return 0

    # Already running from the installed copy: copying a file onto itself truncates it.
    if [ "$(readlink -f "$0" 2>/dev/null)" = "$(readlink -f "$SELF" 2>/dev/null)" ]; then
        return 0
    fi

    install -m 0755 "$0" "$SELF" 2>/dev/null \
        || warn "could not copy this script to ${SELF}; re-fetch it to re-run."
}

# Install every package in $* that is not already installed.
#
# `dpkg -s` per package rather than one `apt-get install` for the lot: apt is slow to start on
# this board, and a re-run that has nothing to do should cost nothing. It also means the log
# names exactly what was missing, which is the difference between "gstreamer was already fine"
# and "gstreamer was half installed" when reading a bring-up log after the fact.
install_missing() {
    missing=""
    for pkg in "$@"; do
        dpkg -s "$pkg" >/dev/null 2>&1 || missing="$missing $pkg"
    done
    [ -n "$missing" ] || return 0

    say "installing:$missing"
    apt-get update -qq || true
    # shellcheck disable=SC2086  # word-splitting the package list is the point
    apt-get install -y -qq $missing \
        || die "apt failed installing:$missing
  Nothing here is partially usable — webrtcbin without libnice negotiates nothing, and a
  missing encoder is a stream that never starts. Fix the network or the apt sources and
  re-run; this script is idempotent."
}

# Is GStreamer element $1 registered?
#
# Both the distro plugin path and GST_EXTRA_PLUGIN_DIR are searched, so a hand-built
# webrtcsink is found here exactly when `mediad` would find it with the same variable set.
have_element() {
    GST_PLUGIN_PATH="${GST_EXTRA_PLUGIN_DIR}${GST_PLUGIN_PATH:+:$GST_PLUGIN_PATH}" \
        gst-inspect-1.0 "$1" >/dev/null 2>&1
}

# Print "  <name>  <present|absent>", padded, for the report.
element_line() {
    if have_element "$1"; then
        printf '  %-16s present\n' "$1"
    else
        printf '  %-16s absent\n' "$1"
    fi
}

# Everything the encoder question turns on, in one place.
#
# This is the part worth re-running. The three answers, in the order they are worth having:
#
#   v4l2h264enc  the vendor kernel exposes the VPU as a V4L2 M2M encoder. Best case by a
#                distance: the element is in `gstreamer1.0-plugins-good`, already installed
#                above, so hardware encode needs nothing out of tree at all.
#   mpph264enc   only reachable through Rockchip's MPP userspace library plus the
#                `gstreamer-rockchip` plugin, neither of which Debian packages. Real, but it
#                is a library from a third-party repo and a plugin built from source.
#   x264enc      software. Present because it is installed above, and it is not a fallback to
#                be comfortable with: `jpegenc` alone cannot hold 30 fps at 640x480 on this
#                SoC (`microduck_runtime/src/camera.rs:500`), and H.264 costs more per frame
#                than JPEG. It shares four Cortex-A55s with `robotd`'s 50 Hz control loop.
report_encoders() {
    say "encoders"
    for el in v4l2h264enc mpph264enc x264enc vp8enc jpegenc; do
        element_line "$el"
    done

    printf '\n'
    say "kernel and VPU"
    printf '  %-16s %s\n' kernel "$KERNEL"
    case "$KERNEL" in
        *-vendor-rk35xx)
            printf '  %-16s vendor (BSP) — the camera and VPU nodes live here\n' branch ;;
        *)
            printf '  %-16s NOT the vendor kernel\n' branch
            warn "this is not the *-vendor-rk35xx kernel. The mainline rk356x kernels have no
  MIPI-CSI ISP capture driver, so the camera gets no /dev/video capture node, and the VPU
  nodes this section looks for are unlikely to be there either. setup-board.sh installs the
  vendor kernel for the audio codec; a stray 'apt upgrade' can repoint /boot back." ;;
    esac

    if [ -e "$MPP_SERVICE" ]; then
        # Mode and owner, not just presence. "Present" and "usable by a daemon" are different
        # claims, and the gap between them is silent: with the node at 0600 root:root — which is
        # how it arrives — a non-root `mpi_enc_test` writes an empty file and **exits 0**. No
        # error, no log line. Measured on a Zero 3W, and it cost a round trip to find.
        mode="$(stat -c '%a' "$MPP_SERVICE" 2>/dev/null || true)"
        owner="$(stat -c '%U:%G' "$MPP_SERVICE" 2>/dev/null || true)"
        printf '  %-16s present  %s %s\n' "$MPP_SERVICE" "${mode:-?}" "${owner:-?}"
        # The group bit is what a daemon rides in on: `mediad` runs as its own user, like every
        # other daemon here, so it needs group rw rather than root.
        gbit="$(printf '%s' "$mode" | sed 's/.*\(..\)$/\1/' | cut -c1)"
        case "$gbit" in
            6|7) ;;
            *) printf '  %-16s root-only — a non-root mediad cannot open it\n' '' ;;
        esac
    else
        printf '  %-16s absent\n' "$MPP_SERVICE"
    fi

    printf '\n'
    say "V4L2 devices"
    found_dev=0
    found_encoder=0
    # Unquoted: this is a glob to expand, which is why it is not "$VIDEO_GLOB".
    # shellcheck disable=SC2086
    for dev in $VIDEO_GLOB; do
        [ -e "$dev" ] || continue
        found_dev=1
        # `-D` prints Driver Info, including Card type and Device Caps. The caps are what
        # separates a capture node from an encoder: an M2M or Video Output node is one that
        # takes frames *in*, which a camera node never does.
        info="$(v4l2-ctl -d "$dev" -D 2>/dev/null || true)"
        card="$(printf '%s' "$info" \
            | sed -n 's/^[[:space:]]*Card type[[:space:]]*:[[:space:]]*//p' | head -1)"
        caps="$(printf '%s' "$info" | grep -cE 'Video M2M|Video Output' || true)"
        if [ "${caps:-0}" -gt 0 ]; then
            found_encoder=1
            printf '  %-16s %s  [M2M / output — candidate encoder]\n' "$dev" "${card:-?}"
        else
            printf '  %-16s %s\n' "$dev" "${card:-?}"
        fi
    done
    if [ "$found_dev" != 1 ]; then
        printf '  none\n'
        printf '  (an unattached camera looks exactly like this — the rkisp capture nodes\n'
        printf '   appear only once a sensor is probed, so this is not itself a fault)\n'
    fi

    printf '\n'
    say "verdict"
    if have_element v4l2h264enc && [ "$found_encoder" = 1 ]; then
        cat <<EOF
  Hardware H.264 looks reachable through v4l2h264enc, with no out-of-tree anything. Confirm
  it actually encodes before believing it — a registered element and a working one are
  different claims:

    gst-launch-1.0 -v videotestsrc num-buffers=60 ! video/x-raw,width=1280,height=720 \\
      ! v4l2h264enc ! h264parse ! fakesink

  If that runs at speed, mediad's pipeline is appsrc ! v4l2h264enc ! h264parse ! webrtcsink
  and neither Rockchip MPP nor an apt repo for it is needed.
EOF
    elif [ -e "$MPP_SERVICE" ]; then
        cat <<'EOF'
  No v4l2h264enc, but /dev/mpp_service is present. On a Rockchip BSP kernel that is the
  expected shape rather than a fault: the VPU is exposed through Rockchip's MPP, not as a V4L2
  M2M encoder, so the absent v4l2h264enc above is not a missing package.

  Prove the VPU encodes before building any GStreamer plugin. Two debs from Radxa's pool — the
  same plain-.deb-download route microduck_runtime already uses for rkaiq — plus MPP's own test
  binary, which needs no GStreamer at all:

    R=https://radxa-repo.github.io/bullseye/pool/main
    curl -sL -O $R/m/mpp/librockchip-mpp1_1.5.0-1_arm64.deb
    curl -sL -O $R/m/mpp/librockchip-vpu0_1.5.0-1_arm64.deb
    curl -sL -O $R/m/mpp/rockchip-mpp-demos_1.5.0-1_arm64.deb
    sudo dpkg -i librockchip-mpp1_1.5.0-1_arm64.deb librockchip-vpu0_1.5.0-1_arm64.deb \
      rockchip-mpp-demos_1.5.0-1_arm64.deb
    sudo mpi_enc_test -w 1280 -h 720 -t 7 -n 60 -o /tmp/out.h264

  All three debs in one dpkg call: rockchip-mpp-demos depends on librockchip-vpu0 at exactly
  1.5.0-1, so installing it alongside mpp1 alone leaves the demos package unconfigured.

  **sudo, and check the file size.** At 0600 root:root the test writes nothing and still exits
  0 — no error, no log line — so `exit=0` is evidence of nothing. A non-empty /tmp/out.h264 is:
  60 frames of 720p came out at ~428 KB on a Zero 3W.

  `-t` is MPP's coding enum, 7 being H.264; `mpi_enc_test -h` lists them. A bitstream in
  /tmp/out.h264 means the hardware encodes and only the GStreamer binding is missing.

  The encoder plugin has to be built. Radxa's prebuilt one is decode-only — measured, not
  assumed: gstreamer1.0-rockchip1_1.14-4 (needs librga2 from libr/librga in the same pool)
  installs and registers cleanly, and provides exactly `mppvideodec` and `mppjpegdec`. No
  encoders; 1.14.4 predates them.

  Install it anyway if you want hardware *decode*, and for what it proves: a plugin built
  against GStreamer 1.14 registers without complaint in 1.26.2, so plugin ABI is not the risk
  in the build below. The encoders are in the current tree — gstmpph264enc.c, gstmpph265enc.c,
  gstmppjpegenc.c, gstmppvp8enc.c — which is meson-built:

    sudo /usr/local/sbin/robot-setup-gstreamer --dev
    sudo apt-get install -y meson ninja-build
    curl -sL -O $R/m/mpp/librockchip-mpp-dev_1.5.0-1_arm64.deb
    curl -sL -O $R/libr/librga/librga-dev_2.2.0-1_arm64.deb
    sudo dpkg -i librockchip-mpp-dev_1.5.0-1_arm64.deb librga-dev_2.2.0-1_arm64.deb
    git clone -b gstreamer-rockchip --depth 1 https://github.com/JeffyCN/mirrors.git
    cd mirrors && meson setup build && ninja -C build && sudo ninja -C build install
    gst-inspect-1.0 mpph264enc

  Two things to know before trusting the result. `mediad` runs as its own user, so
  /dev/mpp_service needs a udev rule giving it a group — see the mode printed above. And
  mpph264enc on 6.1 kernels has upstream reports of poor or invalid bitstreams, with
  mpph265enc suggested instead; H.265 is the worse WebRTC codec for browser reach, so decode
  what this encodes and look at it before building a pipeline on top of it.
EOF
    else
        cat <<EOF
  No hardware H.264 path found: no v4l2h264enc, no /dev/mpp_service, no M2M node. Either this
  is not the vendor kernel (see above), or the VPU is not exposed by this build.

  x264enc is installed and will work, at a cost worth stating: software H.264 on four A55s
  shares the CPU with robotd's control loop, and jpegenc already cannot hold 30 fps at VGA on
  this SoC. Treat it as the interim encoder for bring-up, not the shipping one.
EOF
    fi
}

report_webrtc() {
    say "WebRTC elements"
    element_line webrtcbin
    element_line webrtcsink
    element_line webrtcsrc

    if ! have_element webrtcsink; then
        cat <<EOF

  webrtcsink/webrtcsrc come from gst-plugin-webrtc in gst-plugins-rs, which Debian does not
  package in any suite. Until mediad ships the plugin in its release payload, build it here:

    sudo /usr/local/sbin/robot-setup-gstreamer --dev
    cargo install cargo-c
    git clone https://gitlab.freedesktop.org/gstreamer/gst-plugins-rs.git && cd gst-plugins-rs
    git checkout 0.14.5
    cargo cinstall -p gst-plugin-webrtc --prefix=/usr/local --release

  0.14.5 or newer, not 0.14.4: the earlier tags miss a webrtcsink deadlock fix between remote
  description and ICE handling that presents as a client spinning forever on "connecting".
  It builds libgstrswebrtc.so; gst-plugin-rsrtp is the sibling plugin the same stack wants.
EOF
    fi
}

report() {
    printf '\n'
    # Line 1 is "gst-inspect-1.0 version 1.26.2"; the last field is the number alone.
    version="$(gst-inspect-1.0 --version 2>/dev/null | head -1 | awk '{print $NF}' || true)"
    say "GStreamer ${version:-version unknown}"
    printf '\n'
    report_webrtc
    printf '\n'
    report_encoders
    printf '\n'
    if [ "$WANT_DEV" = 1 ]; then
        say "build headers installed — pkg-config --modversion gstreamer-webrtc-1.0"
    else
        say "runtime only. Re-run with --dev to install the build headers."
    fi
}

main() {
    parse_args "$@"
    check_environment
    persist_self
    # shellcheck disable=SC2086  # word-splitting the package lists is the point
    install_missing $RUNTIME_PKGS
    # shellcheck disable=SC2086
    [ "$WANT_DEV" = 0 ] || install_missing $DEV_PKGS
    report
}

# Called on the last line so a truncated download — the real failure mode of `curl | sh` —
# defines functions and then does nothing, rather than running half a setup.
main "$@"
