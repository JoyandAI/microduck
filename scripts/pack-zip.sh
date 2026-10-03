#!/bin/sh
#
# Pack a hand-install zip of the daemon release.
#
#   scripts/pack-zip.sh
#   scripts/pack-zip.sh --bin-dir target/aarch64-unknown-linux-gnu/release --out dist
#
# Writes dist/daemon-<version>.zip. Unzipping it gives the release tree
# (`bin/`, `systemd/`, `hooks/`, …) that `/opt/robot/daemon/releases/<version>/`
# is. Install that zip with install-zip.sh. See docs/robot/pack-zip.md.
#
# The file list is the release workflow's (`_build-release.yml`), plus `webd`.
# `bin/`, `hooks/` and `scripts/` are mode 0755 inside the zip; everything else
# is 0644. A zip made by Windows Explorer does not keep those bits, and a board
# then fails `ExecStart` with 203/EXEC.
#
# The package you copy onto a board. The signed update a robot downloads
# is still built by `cargo xtask package`. Install with install-zip.sh;
# the usage line follows however that script is invoked.

set -eu

root=$(CDPATH= cd -- "$(dirname "$0")/.." && pwd)
cd "$root"

BIN_DIR="target/aarch64-unknown-linux-gnu/release"
OUT="dist"
REVISION=""

while [ $# -gt 0 ]; do
    case "$1" in
        --bin-dir)
            BIN_DIR=$2
            shift 2
            ;;
        --out)
            OUT=$2
            shift 2
            ;;
        --revision)
            REVISION=$2
            shift 2
            ;;
        *)
            echo "usage: scripts/pack-zip.sh [--bin-dir DIR] [--out DIR] [--revision SHA]" >&2
            exit 1
            ;;
    esac
done

if [ -z "$REVISION" ]; then
    REVISION=$(git rev-parse --short=12 HEAD 2>/dev/null || echo unknown)
fi

version=$(sed -n \
    '/^\[workspace\.package\]/,/^\[/{s/^version[[:space:]]*=[[:space:]]*"\(.*\)".*/\1/p;}' \
    Cargo.toml | head -1)
[ -n "$version" ] || {
    echo "error: no version under [workspace.package] in Cargo.toml" >&2
    exit 1
}

onnx_floor=$(sed -n \
    '/^\[workspace\.metadata\.onnxruntime\]/,/^\[/{s/^floor[[:space:]]*=[[:space:]]*"\(.*\)".*/\1/p;}' \
    Cargo.toml | head -1)
onnx_target=$(sed -n \
    '/^\[workspace\.metadata\.onnxruntime\]/,/^\[/{s/^target[[:space:]]*=[[:space:]]*"\(.*\)".*/\1/p;}' \
    Cargo.toml | head -1)
[ -n "$onnx_floor" ] && [ -n "$onnx_target" ] || {
    echo "error: [workspace.metadata.onnxruntime] is missing floor or target" >&2
    exit 1
}

workflow=.github/workflows/_build-release.yml
# Same parse as scripts/board-test.sh, except the name may contain a hyphen
# (`pet-detect`). A copy of the list kept here would drift from the release.
binaries=$(sed -n 's/\r$//; s|.*/\([^/ ]*\) staged/$|\1|p' "$workflow")
[ -n "$binaries" ] || {
    echo "error: no staged binaries found in ${workflow}" >&2
    exit 1
}

# webd is the OpenMicroDuck console. The GitHub release recipe does not copy it
# yet; a hand install of this tree is how it reaches the board.
binaries="$binaries
webd"

stage=$(mktemp -d "${TMPDIR:-/tmp}/pack-zip.XXXXXX")
include_list=$(mktemp)
trap 'rm -rf "$stage"; rm -f "$include_list"' EXIT
mkdir -p "$stage/bin"

for binary in $binaries; do
    src="$BIN_DIR/$binary"
    if [ ! -f "$src" ]; then
        echo "error: missing binary $src" >&2
        echo "       build it first: cargo build --release --target aarch64-unknown-linux-gnu -p $binary" >&2
        exit 1
    fi
    cp "$src" "$stage/bin/$binary"
done

includes=$(grep -o -- '--include "[^"]*"' "$workflow" | sed 's/--include "//; s/"$//')
[ -n "$includes" ] || {
    echo "error: no --include pairs found in ${workflow}" >&2
    exit 1
}

printf '%s\n' "$includes" > "$include_list"
while IFS= read -r pair; do
    [ -n "$pair" ] || continue
    src=${pair%%=*}
    dest=${pair#*=}
    if [ ! -f "$src" ]; then
        echo "error: missing $src (needed as $dest in the zip)" >&2
        exit 1
    fi
    mkdir -p "$stage/$(dirname "$dest")"
    cp "$src" "$stage/$dest"
done < "$include_list"

hook=$(sed \
    -e "s/@ONNX_FLOOR@/${onnx_floor}/g" \
    -e "s/@ONNX_TARGET@/${onnx_target}/g" \
    hooks/preinstall.in)
case "$hook" in
    *@ONNX_*)
        echo "error: hooks/preinstall.in still has an @ONNX_ placeholder" >&2
        exit 1
        ;;
esac
mkdir -p "$stage/hooks"
printf '%s\n' "$hook" > "$stage/hooks/preinstall"

names=$(find "$stage/bin" -type f -printf '%f\n' | sort)
list="["
sep=""
for name in $names; do
    list="${list}${sep}\"${name}\""
    sep=", "
done
list="${list}]"

mkdir -p "$OUT"
zip_path="$OUT/daemon-${version}.zip"
rm -f "$zip_path"

cat > "$stage/version.toml" <<EOF
version = "${version}"
channel = "daemon"
revision = "${REVISION}"
binaries = ${list}
EOF

python3 - "$stage" "$zip_path" <<'PY'
import sys
import zipfile
from pathlib import Path

root = Path(sys.argv[1])
out = Path(sys.argv[2])
executable = ("bin/", "hooks/", "scripts/")
with zipfile.ZipFile(out, "w", compression=zipfile.ZIP_DEFLATED) as archive:
    for path in sorted(p for p in root.rglob("*") if p.is_file()):
        rel = path.relative_to(root).as_posix()
        info = zipfile.ZipInfo(rel)
        mode = 0o755 if rel.startswith(executable) else 0o644
        # create_system 3 is Unix. unzip only applies external_attr when the
        # entry says it was made on Unix; the default (0, DOS) lands as 0644
        # and systemd then fails the unit with 203/EXEC.
        info.create_system = 3
        info.external_attr = (0o100000 | mode) << 16
        info.date_time = (1980, 1, 1, 0, 0, 0)
        info.compress_type = zipfile.ZIP_DEFLATED
        with path.open("rb") as handle:
            data = handle.read()
        # Windows checkouts store hooks/scripts as CRLF. dash on the board then
        # fails `set -eu` with `Illegal option -`.
        if rel.startswith(("hooks/", "scripts/")):
            data = data.replace(b"\r\n", b"\n").replace(b"\r", b"")
        archive.writestr(info, data)
PY

echo "packed $zip_path"
echo "  version $version"
echo "  revision $REVISION"
here=$(dirname "$0")
case "$here" in
    ''|.) installer=install-zip.sh ;;
    *) installer="$here/install-zip.sh" ;;
esac
echo "install: sudo sh $installer $zip_path"
