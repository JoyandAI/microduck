#!/bin/sh
# Build the daemon on this laptop and install it on a board, without CI.
#
# Usage:  scripts/dev-push.sh [--docker] [--dry-run] [--bootstrap] [user@host]
#         DUCK_BOARD=radxa@duck.local scripts/dev-push.sh
#
# Requires the team dev secret key, and one of two build toolchains — `cargo-zigbuild` plus
# `zig`, or `--docker`. The board must be a dev board (`allow_dev_keys = true` and
# `team.dev.pub` in its trusted keys — `deploy/README.md`).
#
# **Two ways to build, same artifact.** The default cross-compiles here with `cargo zigbuild`:
# fastest, and what CI uses. `--docker` builds inside the board's own userland instead, where
# there is nothing to cross and libudev is an `apt-get install` — reach for it when the zig
# toolchain is not set up, when it breaks, or before you have a board to take libudev from.
#
# What this is for: the loop between "I changed a line" and "the robot is running it" was a
# push, a CI run and a `--ref` install. Everything CI does to make that artifact happens
# locally in well under a minute, so the only reason to involve CI is to publish something
# other people install.
#
# **It is an ordinary update.** The board applies this through `robotctl update apply`, so
# preflight, the signature, the artifact hash, compatibility, the health gate and auto-rollback
# all run exactly as they do for a release — a local build that does not come up is reverted and
# the board is back on what it was running. That is the reason `--from` exists as an option on
# `apply` rather than reusing `updaterd install --from`, which has to force the gate off and so
# refuses to touch a live release at all.
#
# **What it deliberately does not do** is anything a release does for provenance. The version
# carries a timestamp, not a tag; the artifact is signed with the dev key, which a customer
# robot refuses; nothing is published, so nobody else can install what you just ran. Cutting a
# release is still a tag and `release.yml`.
#
# The version is `<crate>-dev.local.<epoch>.g<sha7>`: a prerelease, so it sorts below the
# release it precedes and can never look like an upgrade for the fleet, and unique per *push*
# rather than per commit — the tree is expected to be dirty here, and two pushes of the same
# dirty tree must not collide into "already current".
set -eu

cd "$(dirname "$0")/.."

# Where the artifact lands on the board. World-writable parent, so no sudo to copy into it;
# `updaterd` reads it as root.
REMOTE_DIR="${DUCK_SIDELOAD_DIR:-/var/tmp/duck-sideload}"

# The secret half of `team.dev`, the same key `dev.yml` signs branch builds with. Named apart
# from `DUCK_DEV_KEY`, which the provisioning scripts use for the *public* half.
KEY="${DUCK_DEV_SECRET_KEY:-$HOME/.duck-keys/team.dev.key}"

BOOTSTRAP=no
DRY_RUN=no
DOCKER=no
BOARD="${DUCK_BOARD:-}"

while [ $# -gt 0 ]; do
    case "$1" in
        --bootstrap) BOOTSTRAP=yes ;;
        --dry-run) DRY_RUN=yes ;;
        --docker) DOCKER=yes ;;
        -h|--help)
            sed -n '2,6p' "$0" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        -*) echo "unknown option: $1" >&2; exit 2 ;;
        *) BOARD="$1" ;;
    esac
    shift
done

if [ -z "$BOARD" ]; then
    echo "no board: pass one as an argument or set DUCK_BOARD" >&2
    echo "  scripts/dev-push.sh radxa@duck.local" >&2
    exit 2
fi

if [ "$DOCKER" = no ]; then
    # `command -v`, not `cargo zigbuild --version`: the subcommand forwards its arguments to
    # `cargo build`, which rejects `--version`, so asking it that way reports the toolchain as
    # missing on a machine where it is installed and working.
    if ! command -v cargo-zigbuild >/dev/null 2>&1; then
        echo "cargo-zigbuild is not installed; the board target has no linker without it" >&2
        echo "  cargo install cargo-zigbuild --locked" >&2
        echo "  brew install zig" >&2
        echo "or build in a container instead, which needs neither:" >&2
        echo "  scripts/dev-push.sh --docker $BOARD" >&2
        exit 1
    fi
elif ! docker version >/dev/null 2>&1; then
    echo "--docker needs a running Docker daemon" >&2
    echo "  open -a Docker" >&2
    exit 1
fi

if [ ! -f "$KEY" ]; then
    echo "no dev signing key at $KEY" >&2
    echo "The board verifies this artifact like any release, so it has to be signed." >&2
    echo "Get team.dev.key from a team member, or set DUCK_DEV_SECRET_KEY." >&2
    exit 1
fi

# ── the one C dependency, and why this is not `apt-get install libudev-dev:arm64` ──────
#
# `padd` needs libudev (via `gilrs`), which is the only C library anything reaching the board
# links against — `scripts/ci-cross-deps.sh` exists solely to install it for the target on a
# CI runner. That script is Debian-only and cannot help here, and a Mac has no way to install
# an aarch64 Linux library through a package manager.
#
# So take it from the board, which by definition runs the exact library the binary will load
# there. The linker records the SONAME (`libudev.so.1`), not the filename it was given, so a
# copy named `libudev.so` next to a hand-written `.pc` is all `-ludev` needs. Cached: it is one
# `scp` on first use, and `rm -rf` the directory to refresh it.
#
# One difference from CI worth knowing about, and it is inert: `libudev-sys`'s build script
# also probes for `udev_hwdb_new` by linking a test binary with the *host* toolchain, which
# fails on a Mac and leaves its `hwdb` cfg off. `gilrs` calls nothing under that cfg.
#
# `--docker` needs none of this — see `scripts/dev-build.Dockerfile`. It is also the way out if
# the board you would take libudev from is a board you have not set up yet.
SYSROOT="${DUCK_CROSS_SYSROOT:-$HOME/.cache/duck-cross/aarch64}"
if [ "$DOCKER" = no ] && [ ! -f "$SYSROOT/lib/libudev.so" ]; then
    echo "==> fetching libudev from $BOARD for cross-linking (once)"
    remote_lib="$(ssh "$BOARD" 'ls /usr/lib/aarch64-linux-gnu/libudev.so.1 /lib/aarch64-linux-gnu/libudev.so.1 2>/dev/null | head -1')"
    if [ -z "$remote_lib" ]; then
        echo "no libudev.so.1 on $BOARD; padd cannot be linked for it" >&2
        exit 1
    fi
    mkdir -p "$SYSROOT/lib" "$SYSROOT/pkgconfig"
    scp -q "$BOARD:$remote_lib" "$SYSROOT/lib/libudev.so"
    # `find_library` in libudev-sys asks for no particular version, so this only has to parse.
    cat > "$SYSROOT/pkgconfig/libudev.pc" <<EOF
libdir=$SYSROOT/lib
Name: libudev
Description: libudev, copied from a board by scripts/dev-push.sh
Version: 0
Libs: -L\${libdir} -ludev
Cflags:
EOF
fi
if [ "$DOCKER" = no ]; then
    # Prepended rather than replacing: on a Linux host with the multiarch package installed,
    # both are then visible and this one still wins.
    PKG_CONFIG_PATH="$SYSROOT/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
    export PKG_CONFIG_PATH
    # pkg-config refuses to answer for another architecture unless told to.
    export PKG_CONFIG_ALLOW_CROSS=1
fi

SHA="$(git rev-parse HEAD)"
SHA7="$(git rev-parse --short=7 HEAD)"
# Read from `cargo metadata` rather than parsed out of Cargo.toml, the same way `dev.yml`
# derives it: the value lives in `[workspace.package]` and the members inherit it, so grepping
# a member's manifest finds `version.workspace = true` and grepping the root finds a line that
# is only the workspace version by convention.
CRATE="$(cargo metadata --format-version 1 --no-deps | python3 -c 'import json,sys; print(next(p["version"] for p in json.load(sys.stdin)["packages"] if p["name"] == "updater"))')"
VERSION="${CRATE}-dev.local.$(date +%s).g${SHA7}"

rm -rf staged dist

# `DUCK_REVISION` and no `DUCK_BUILD_TIME`, unlike the release workflows, and it is worth a
# measurement rather than a shrug. Both are read with `option_env!` in `duck-ipc-proto`, which
# every daemon depends on, so a value that changes invalidates it and everything above it: with
# a fresh timestamp each run, five crates rebuild on every push whether or not a line changed
# (~30s here); with only the revision, which moves when you commit rather than when you push, an
# unchanged tree rebuilds nothing at all. Nothing visible is lost — `robotctl version` reports
# the revision and not the build time, and the version string carries the push's epoch.
if [ "$DOCKER" = no ]; then
    echo "==> building $VERSION for the board (zigbuild)"
    BIN="target/aarch64-unknown-linux-gnu/release"
    DUCK_REVISION="$SHA" cargo board --bins
else
    echo "==> building $VERSION for the board (docker)"
    # A separate target directory, not the one `cargo board` writes. The two builds produce the
    # same triple through different toolchains and linkers, and cargo's fingerprints do not
    # capture all of that difference — sharing a directory risks it deciding a binary from the
    # other environment is up to date. Two directories cost disk and nothing else.
    BIN="target/docker/aarch64-unknown-linux-gnu/release"

    # Rebuilt every time and cached by Docker, so a change to the Dockerfile takes effect
    # without anyone remembering to bump a tag. The context is `scripts/`, which keeps the
    # repository (and `target/`) out of the daemon's hands.
    docker build -q -t duck-dev-build -f scripts/dev-build.Dockerfile scripts/ >/dev/null

    if [ "$(uname -m)" != arm64 ] && [ "$(uname -m)" != aarch64 ]; then
        echo "    host is $(uname -m): the arm64 container runs under emulation, expect slow" >&2
    fi

    # `--platform linux/arm64` so the binaries are aarch64 wherever this runs. On Apple Silicon
    # that is a native build and the target is the host, which is the entire point: nothing to
    # cross, and libudev came from apt.
    #
    # Registry cache in a named volume rather than the host's `~/.cargo`: crate sources are
    # re-downloaded once instead of two cargos with different ideas about locking sharing a
    # directory. `target/docker` is a bind mount so builds stay incremental across runs and the
    # binaries are here afterwards without a copy step.
    docker run --rm --platform linux/arm64 \
        -v "$PWD:/src" -w /src \
        -v duck-dev-cargo-registry:/usr/local/cargo/registry \
        -e CARGO_TARGET_DIR=/src/target/docker \
        -e DUCK_REVISION="$SHA" \
        duck-dev-build \
        cargo build --release --target aarch64-unknown-linux-gnu --bins
fi

echo "==> packaging"
mkdir -p staged
# The same list as the `cp` block in `dev.yml` and `release.yml`, deliberately: this pushes the
# same artifact a release does, and `xtask/tests/artifact.rs` packages all three lists and checks
# the tarball, so a binary added to one and not the others fails there. Only the directory
# differs, because which toolchain built these is the one thing that does.
cp "$BIN"/updaterd staged/
cp "$BIN"/robotctl staged/
cp "$BIN"/robotd staged/
cp "$BIN"/configd staged/
cp "$BIN"/btd staged/
cp "$BIN"/padd staged/

# No `--base-url`: the manifest `LocalDir` reads names the artifact by bare filename, and
# `package` leaves it bare when no base is given.
#
# `--zstd-level 1` rather than the shipping default of 19. This artifact is read once, by one
# board, and thrown away; at 19 the compression alone is most of the wall-clock of this script,
# which is the one thing it exists to keep short.
cargo run -p xtask -- package \
    --version "$VERSION" \
    --channel daemon \
    --bin-dir staged \
    --out dist \
    --revision "$SHA" \
    --zstd-level 1 \
    --include "updater/systemd/updaterd.service=systemd/updaterd.service" \
    --include "updater/systemd/sysusers.d/robot.conf=systemd/sysusers.d/robot.conf" \
    --include "robotd/systemd/robotd.service=systemd/robotd.service" \
    --include "hooks/postinstall=hooks/postinstall" \
    --include "configd/systemd/configd.service=systemd/configd.service" \
    --include "btd/systemd/btd.service=systemd/btd.service" \
    --include "btd/systemd/sysusers.d/btd.conf=systemd/sysusers.d/btd.conf" \
    --include "padd/systemd/padd.service=systemd/padd.service" \
    --include "padd/systemd/sysusers.d/padd.conf=systemd/sysusers.d/padd.conf" \
    --include "deploy/journald.conf.d/10-robot.conf=deploy/journald.conf.d/10-robot.conf" \
    --include "docs/design/architecture.md=docs/architecture.md" \
    --include "docs/design/updater-design.md=docs/updater-design.md" \
    --include "deploy/README.md=docs/deploy.md" \
    --include "policies/alpha_walking.onnx=policies/alpha_walking.onnx" \
    --include "policies/alpha_stand.onnx=policies/alpha_stand.onnx"

echo "==> signing with $KEY"
cargo run -p xtask -- sign --dir dist --key "$KEY"

# Replaced rather than added to: a directory holding two builds makes "the newest one here"
# ambiguous to read, and nothing on the board needs yesterday's push.
echo "==> copying to $BOARD:$REMOTE_DIR"
# shellcheck disable=SC2029  # expanding the path here is the intent: it is this laptop's
# setting, and the board has no DUCK_SIDELOAD_DIR to read.
ssh "$BOARD" "rm -rf '$REMOTE_DIR' && mkdir -p '$REMOTE_DIR'"
# `scp`, not `rsync`: the artifact is a single compressed blob that changes completely every
# build, so there is no delta to exploit, and this needs nothing on the board that ssh did not
# already bring.
scp -q dist/* "$BOARD:$REMOTE_DIR/"

if [ "$BOOTSTRAP" = yes ]; then
    # For a board whose *installed* `updaterd` predates `apply --from` and therefore cannot be
    # asked to use it — including the push that first delivers it. This is the documented
    # escape hatch, and it costs the health gate for one install: `updaterd install` forces
    # `on_apply` and `health` off, which is why it refuses a live release without `--force`.
    #
    # `updaterd` then has to be restarted explicitly. It never restarts itself during an update
    # — that would kill the process performing it — so the resident daemon keeps running the old
    # binary, and the old binary is the one that does not understand `--from`.
    echo "==> bootstrap install (no health gate, robotd stopped)"
    # shellcheck disable=SC2029  # as above: $REMOTE_DIR is expanded locally on purpose.
    ssh -t "$BOARD" "set -e
        sudo systemctl stop robotd
        sudo /opt/robot/daemon/current/bin/updaterd install --from '$REMOTE_DIR' --force
        sudo systemctl restart updaterd
        sudo systemctl start robotd"
    echo "==> installed $VERSION (ungated); ordinary pushes need no --bootstrap from here"
    exit 0
fi

echo "==> applying on $BOARD"
APPLY="sudo robotctl update apply daemon --from '$REMOTE_DIR' --version '$VERSION'"
[ "$DRY_RUN" = no ] || APPLY="$APPLY --dry-run"

if ssh -t "$BOARD" "$APPLY"; then
    echo "==> $VERSION is live on $BOARD"
else
    status=$?
    echo "==> apply failed (exit $status)" >&2
    # Exit 2 from robotctl is bad usage, and the way this fails on a board that has not had a
    # build with `--from` yet: the daemon refuses the whole call on the API version, because a
    # daemon that merely ignored the option would install from its configured source instead
    # and report success for the wrong release.
    if [ "$status" -eq 2 ]; then
        echo "If robotctl and updaterd report an API mismatch, this board's installed" >&2
        echo "release predates 'apply --from'. Deliver it once, ungated:" >&2
        echo "  scripts/dev-push.sh --bootstrap $BOARD" >&2
    fi
    exit "$status"
fi
