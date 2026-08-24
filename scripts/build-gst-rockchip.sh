#!/bin/sh
# Build the Rockchip MPP GStreamer plugin — `mpph264enc` and friends — from pinned source.
#
# Why ours rather than a package: nobody ships a usable one. Debian has no Rockchip encoder at
# all; Radxa's `gstreamer1.0-rockchip1_1.14-4` is decode-only (measured — `mppvideodec` and
# `mppjpegdec`, nothing else); and the one third-party deb that *does* carry encoders is one
# person's per-board dump with no provenance we control, built for Ubuntu jammy and RK3588.
# See `docs/project/media-bringup.md`.
#
# Building also buys something the packages cannot: this disables `rkximage` and `kmssrc`, the
# X11 and KMS *sinks* in the same source tree. A headless robot has no use for either, and they
# are why the prebuilt deb depends on `libx11-6`. What lands is the one plugin `mediad` needs.
#
#   sudo sh /tmp/build-gst-rockchip.sh
#
# The pin is a commit, not the branch: `gstreamer-rockchip` is a moving mirror, and a plugin
# whose version nobody can name is the thing that makes a media bug unreproducible. Override with
# GST_ROCKCHIP_REF to try another.
#
# Idempotent in the sense that matters — it rebuilds from scratch each time into a temporary
# directory, and only the install step touches the system.
#
# Radxa Zero 3W on Armbian, Debian 13 (trixie) userland, aarch64. It builds natively; there is no
# cross-build here, because the plugin changes roughly never while `mediad` changes hourly, so it
# does not belong in the daily loop.
set -eu

# JeffyCN/mirrors@gstreamer-rockchip, 2026-05-21 ("mppvideodec: Fix input packet leak").
#
# This is the live upstream: `rockchip-linux/gstreamer-rockchip`, which the Radxa and third-party
# debs both name as their homepage, is a 404 now. Jeffy Chen is the maintainer in both.
GST_ROCKCHIP_REPO="${GST_ROCKCHIP_REPO:-https://github.com/JeffyCN/mirrors.git}"
GST_ROCKCHIP_BRANCH="${GST_ROCKCHIP_BRANCH:-gstreamer-rockchip}"
GST_ROCKCHIP_REF="${GST_ROCKCHIP_REF:-dcbcd6454ef892e385b3a782600369eb6c0719db}"

# Where the plugin lands. Not the distro's own plugin directory: this is our artifact, it is
# reached through GST_PLUGIN_PATH, and keeping it out of /usr/lib means an `apt` operation can
# never quietly replace or remove it. `setup-gstreamer.sh` already searches this path, and it is
# the same shape `mediad` will use when the plugin ships inside a release payload.
PREFIX="${PREFIX:-/usr/local}"
PLUGIN_DIR="${PREFIX}/lib/gstreamer-1.0"

# Rockchip MPP and RGA headers, from Radxa's pool. Same plain-.deb route
# `microduck_runtime/radxa_setup/setup_rkaiq.sh` uses, and the same versions the runtime libraries
# come from — a header/library mismatch here is a plugin that builds and then misbehaves.
RADXA_POOL="${RADXA_POOL:-https://radxa-repo.github.io/bullseye/pool/main}"
MPP_DEV_DEB="m/mpp/librockchip-mpp-dev_1.5.0-1_arm64.deb"
RGA_DEV_DEB="libr/librga/librga-dev_2.2.0-1_arm64.deb"

say()  { printf '\033[1m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[33mwarning:\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[31merror:\033[0m %s\n' "$*" >&2; exit 1; }

check_environment() {
    [ "$(id -u)" = 0 ] || die "run as root — re-run that same command with sudo"

    arch="$(uname -m)"
    [ "$arch" = aarch64 ] || die "this builds natively for the board, and this box is ${arch}"

    for tool in git curl dpkg; do
        command -v "$tool" >/dev/null 2>&1 || die "${tool} is required"
    done
}

# The build toolchain, plus the GStreamer headers the plugin links against.
#
# `setup-gstreamer.sh --dev` installs the GStreamer half; naming it here too means this script
# works on a board where nobody ran that, rather than failing in meson with a pkg-config error
# that does not say which package to install.
install_build_deps() {
    pkgs="meson ninja-build build-essential pkg-config git
libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libdrm-dev"
    missing=""
    for pkg in $pkgs; do
        dpkg -s "$pkg" >/dev/null 2>&1 || missing="$missing $pkg"
    done
    if [ -n "$missing" ]; then
        say "installing:$missing"
        apt-get update -qq || true
        # shellcheck disable=SC2086  # word-splitting the package list is the point
        apt-get install -y -qq $missing || die "apt failed installing:$missing"
    fi

    # The two that are not in Debian at all.
    for spec in "librockchip-mpp-dev ${MPP_DEV_DEB}" "librga-dev ${RGA_DEV_DEB}"; do
        pkg="${spec%% *}"
        path="${spec#* }"
        dpkg -s "$pkg" >/dev/null 2>&1 && continue
        deb="$(basename "$path")"
        tmp="$(mktemp -d)"
        say "fetching ${pkg} from Radxa's pool"
        curl -fsSL -o "${tmp}/${deb}" "${RADXA_POOL}/${path}" \
            || { rm -rf "$tmp"; die "cannot download ${deb} from ${RADXA_POOL}/${path}"; }
        # `dpkg -i` resolves nothing here — these are direct downloads, not a configured apt
        # source — so a missing dependency is an unconfigured package rather than a fixed one.
        # Both of these are leaf -dev packages whose runtime halves are already in.
        dpkg -i "${tmp}/${deb}" || { rm -rf "$tmp"; die "dpkg -i ${deb} failed"; }
        rm -rf "$tmp"
    done

    # Verified, because meson's answer to a missing MPP is not an error. `gst/rockchipmpp`'s
    # meson.build ends in `if not mpp_dep.found() → subdir_done()`, so the whole plugin is
    # *silently skipped* and the build succeeds having produced nothing.
    for mod in rockchip_mpp librga; do
        pkg-config --exists "$mod" \
            || die "pkg-config cannot find ${mod}, and meson skips the plugin silently without it.
  Check that librockchip-mpp-dev and librga-dev installed, and that their .pc files are on
  PKG_CONFIG_PATH."
    done
}

# Warn about the distro package rather than removing it.
#
# Two copies of the same plugin on the search path is a coin flip over which registers, and
# `apt` owning one of them means an upgrade can change the answer under us. Removing a package
# on someone's board is their call, not this script's.
check_for_packaged_copy() {
    dpkg -s gstreamer1.0-rockchip1 >/dev/null 2>&1 || return 0
    warn "gstreamer1.0-rockchip1 is installed, and provides the same elements from
  /usr/lib/aarch64-linux-gnu/gstreamer-1.0. Which one registers is not worth leaving to chance:
      sudo dpkg -r gstreamer1.0-rockchip1
  Keep it only if you want its hardware *decoders* and are sure this build does not supersede
  them — this one provides the decoders too."
}

build() {
    src="$(mktemp -d)"
    # shellcheck disable=SC2064  # expand $src now, deliberately
    trap "rm -rf '$src'" EXIT INT TERM

    say "cloning ${GST_ROCKCHIP_BRANCH} at ${GST_ROCKCHIP_REF}"
    # A shallow clone of one branch, then a hard reset to the pin. `--depth 1` alone would give
    # whatever the branch tip is today, which is the thing the pin exists to prevent.
    git clone -q --branch "$GST_ROCKCHIP_BRANCH" "$GST_ROCKCHIP_REPO" "$src/gst-rockchip" \
        || die "cannot clone ${GST_ROCKCHIP_REPO}"
    git -C "$src/gst-rockchip" checkout -q "$GST_ROCKCHIP_REF" \
        || die "commit ${GST_ROCKCHIP_REF} is not in ${GST_ROCKCHIP_BRANCH}"

    say "configuring (rockchipmpp only; X11 and KMS sinks disabled)"
    # `--libdir lib` so plugins_install_dir resolves to ${PREFIX}/lib/gstreamer-1.0 rather than a
    # multiarch subdirectory — one fixed path for GST_PLUGIN_PATH, on every board.
    meson setup "$src/build" "$src/gst-rockchip" \
        --prefix "$PREFIX" \
        --libdir lib \
        --buildtype release \
        -Drockchipmpp=enabled \
        -Drga=enabled \
        -Drkximage=disabled \
        -Dkmssrc=disabled \
        -Dvpxalphadec=disabled \
        >"$src/meson.log" 2>&1 || {
            tail -30 "$src/meson.log" >&2
            die "meson setup failed; the tail of its log is above.
  The tree declares meson_version >= 0.47 and was written against a much older meson, so a
  syntax rejection here is the failure to expect. $(meson --version 2>/dev/null || true) is installed."
        }

    say "building"
    ninja -C "$src/build" >"$src/ninja.log" 2>&1 || {
        tail -30 "$src/ninja.log" >&2
        die "ninja failed; the tail of its log is above"
    }

    built="$(find "$src/build" -name 'libgstrockchipmpp.so' -type f | head -1)"
    [ -n "$built" ] || die "the build produced no libgstrockchipmpp.so.
  That is what a skipped subdir looks like rather than a compile error — see the pkg-config
  check above."

    say "installing to ${PLUGIN_DIR}"
    install -d "$PLUGIN_DIR"
    # Stripped: the debug symbols are several times the plugin and nothing on a robot reads them.
    install -m 0644 "$built" "${PLUGIN_DIR}/libgstrockchipmpp.so"
    strip --strip-unneeded "${PLUGIN_DIR}/libgstrockchipmpp.so" 2>/dev/null || true
}

# What was built, in the form needed to pin it later.
#
# The commit and the sha256 together are what let a release payload carry this plugin and a bug
# report name the exact binary. Printed rather than written to a file: nothing consumes it yet,
# and a file nobody reads is worse than a line in a bring-up log.
report() {
    printf '\n'
    say "built"
    printf '  %-14s %s\n' commit "$GST_ROCKCHIP_REF"
    printf '  %-14s %s\n' path "${PLUGIN_DIR}/libgstrockchipmpp.so"
    printf '  %-14s %s\n' size "$(stat -c '%s' "${PLUGIN_DIR}/libgstrockchipmpp.so" 2>/dev/null || echo '?')"
    printf '  %-14s %s\n' sha256 "$(sha256sum "${PLUGIN_DIR}/libgstrockchipmpp.so" | cut -d' ' -f1)"

    printf '\n'
    say "elements it registers"
    # Through GST_PLUGIN_PATH, which is how `mediad` will find it, so this is the real question
    # rather than a proxy for it.
    if GST_PLUGIN_PATH="$PLUGIN_DIR" gst-inspect-1.0 rockchipmpp 2>/dev/null \
        | sed -n 's/^  \([a-z0-9]*\): /  \1  /p'; then
        :
    else
        warn "gst-inspect could not load the plugin from ${PLUGIN_DIR}.
  Try it directly for the real error:
      GST_PLUGIN_PATH=${PLUGIN_DIR} gst-inspect-1.0 ${PLUGIN_DIR}/libgstrockchipmpp.so"
    fi

    printf '\n'
    # The trap that has now caught us twice: /dev/mpp_service arrives 0600 root:root, and the
    # failures it causes are silent — `mpi_enc_test` writes an empty file and exits 0, and this
    # plugin registers its decoders while omitting the encoders, because registration probes MPP.
    if [ -e /dev/mpp_service ] \
        && [ "$(stat -c '%G' /dev/mpp_service 2>/dev/null || true)" = root ]; then
        warn "/dev/mpp_service is still root-only, so the encoders may be missing from the list
  above even though they were built — this plugin probes MPP when it registers. Fix it with:
      sudo /usr/local/sbin/robot-setup-gstreamer
  then re-run this, or check as root:  sudo gst-inspect-1.0 mpph264enc"
    fi

    cat <<EOF

Prove it encodes, end to end, rather than trusting the element list:

  GST_PLUGIN_PATH=${PLUGIN_DIR} gst-launch-1.0 videotestsrc num-buffers=60 \\
    ! video/x-raw,width=1280,height=720 ! mpph264enc ! h264parse \\
    ! filesink location=/tmp/gst.h264

  gst-launch-1.0 filesrc location=/tmp/gst.h264 ! h264parse ! avdec_h264 ! fakesink

A non-empty /tmp/gst.h264 that decodes clean is the answer. The VPU emits High profile by
default and WebRTC's interoperable floor is Constrained Baseline, so check what
\`gst-inspect-1.0 mpph264enc\` offers for profile before building a pipeline on it.
EOF
}

main() {
    check_environment
    install_build_deps
    check_for_packaged_copy
    build
    report
}

# Called on the last line so a truncated download — the real failure mode of `curl | sh` —
# defines functions and then does nothing, rather than running half a build.
main "$@"
