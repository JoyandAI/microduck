#!/bin/sh
#
# Install a hand-built daemon zip into the release store.
#
#   sudo sh scripts/install-zip.sh daemon-0.13.0.zip
#
# Replaces /opt/robot/daemon/releases/<version>/, points current at it, and runs
# the release's hooks/postinstall (units, sysusers, then enable --now). The
# version comes from version.toml inside the zip.
#
# No signature check. This is for a board you are copying a zip onto. A robot
# that takes updates uses updaterd, which does not install this zip.
#
# postinstall starts the units it enables, including robotd, so the servos come
# back under the new binary when this script finishes.

set -eu

ZIP=${1:-}
if [ -z "$ZIP" ] || [ "$ZIP" = "-h" ] || [ "$ZIP" = "--help" ]; then
    echo "usage: sudo sh scripts/install-zip.sh daemon-<version>.zip" >&2
    exit 1
fi

if [ "$(id -u)" -ne 0 ]; then
    echo "error: run as root: sudo sh scripts/install-zip.sh $ZIP" >&2
    exit 1
fi

if [ ! -f "$ZIP" ]; then
    echo "error: no such file: $ZIP" >&2
    exit 1
fi

command -v unzip >/dev/null 2>&1 || {
    echo "error: unzip is not installed" >&2
    exit 1
}

# Resolve before any later cd. A relative path must still work.
ZIP=$(CDPATH= cd -- "$(dirname "$ZIP")" && pwd)/$(basename "$ZIP")

if unzip -Z1 "$ZIP" >/dev/null 2>&1; then
    names=$(unzip -Z1 "$ZIP")
else
    names=$(unzip -l "$ZIP" | awk 'NR>3 && $4 != "" { print $4 }')
fi

name_list=$(mktemp)
trap 'rm -f "$name_list"' EXIT
printf '%s\n' "$names" > "$name_list"
while IFS= read -r name; do
    [ -n "$name" ] || continue
    case "$name" in
        /*|*..*)
            echo "error: zip entry is not a relative path: $name" >&2
            exit 1
            ;;
    esac
done < "$name_list"
rm -f "$name_list"
trap - EXIT

version=$(unzip -p "$ZIP" version.toml | sed -n 's/^version = "\([^"]*\)".*/\1/p')
if [ -z "$version" ]; then
    echo "error: $ZIP has no version.toml, or version is empty" >&2
    exit 1
fi
case "$version" in
    *[!0-9A-Za-z._-]*)
        echo "error: refusing version '$version'" >&2
        exit 1
        ;;
esac

INSTALL_DIR=/opt/robot/daemon
dest="${INSTALL_DIR}/releases/${version}"

if [ -e "${INSTALL_DIR}/current" ] && [ ! -L "${INSTALL_DIR}/current" ]; then
    echo "error: ${INSTALL_DIR}/current exists and is not a symlink" >&2
    exit 1
fi

echo "stopping daemons"
for unit in robotd updaterd btd configd padd mediad tofd; do
    systemctl stop "$unit" 2>/dev/null || true
done

echo "installing $version into $dest"
rm -rf "$dest"
mkdir -p "$dest"
unzip -q -o -d "$dest" "$ZIP"

if [ ! -x "$dest/bin/robotd" ]; then
    echo "error: $dest/bin/robotd is missing or not executable" >&2
    echo "       the zip was packed without unix mode 0755 on bin/" >&2
    exit 1
fi

ln -sfn "releases/${version}" "${INSTALL_DIR}/current"

if [ -f "$dest/hooks/postinstall" ]; then
    echo "running hooks/postinstall"
    (cd "$dest" && sh hooks/postinstall)
else
    systemctl start robotd
fi

echo "installed $version"
echo "  current -> $(readlink "${INSTALL_DIR}/current")"
state=$(systemctl is-active robotd || true)
echo "  ${state:-unknown} robotd"
