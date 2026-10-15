#!/bin/sh
# Install Xbox controller driver (xpadneo) via DKMS
#
# Usage:
#   sudo sh scripts/setup-xpadneo.sh
#
# This script:
# 1. Copies xpadneo source to /usr/src/
# 2. Builds and installs via DKMS
# 3. Loads the module

set -eu

SCRIPT_DIR="$(dirname "$0")"
XPADNEO_DIR="$SCRIPT_DIR/../deploy/hid-xpadneo-dkms"

say() { printf '\033[1m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[33mwarning:\033[0m %s\n' "$*" >&2; }
die() { printf '\033[31merror:\033[0m %s\n' "$*" >&2; exit 1; }

# Check root
[ "$(id -u)" = 0 ] || die "run as root — re-run that same command with sudo"

# Check DKMS
command -v dkms >/dev/null 2>&1 || die "dkms not found — install it first: sudo apt install dkms"

# Check kernel headers
[ -d "/lib/modules/$(uname -r)/build" ] || die "kernel headers not found for $(uname -r)"

# Check source directory
[ -d "$XPADNEO_DIR" ] || die "source directory not found: $XPADNEO_DIR"
[ -f "$XPADNEO_DIR/dkms.conf" ] || die "dkms.conf not found in $XPADNEO_DIR"

# Read version from dkms.conf. A Windows checkout leaves CR on the line, and sed
# then misses PACKAGE_VERSION entirely.
MOD_NAME="hid-xpadneo"
MOD_VERSION=$(sed -n 's/^PACKAGE_VERSION="\([^"]*\)".*/\1/p' "$XPADNEO_DIR/dkms.conf" | tr -d '\r')
[ -n "$MOD_VERSION" ] || die "could not read PACKAGE_VERSION from dkms.conf"

say "installing $MOD_NAME-$MOD_VERSION"

# Check required kernel config
check_kernel_config() {
    local config_file=""
    if [ -f "/proc/config.gz" ]; then
        config_file="/proc/config.gz"
    elif [ -f "/boot/config-$(uname -r)" ]; then
        config_file="/boot/config-$(uname -r)"
    fi

    if [ -n "$config_file" ]; then
        local missing=0
        for opt in CONFIG_HID CONFIG_POWER_SUPPLY CONFIG_CRC16; do
            # /proc/config.gz is compressed; /boot/config-* is plain text. zcat on the
            # plain file fails closed and reports every option missing.
            if [ "$config_file" = "/proc/config.gz" ]; then
                have=$(zcat "$config_file" 2>/dev/null | grep -c "^${opt}=" || true)
            else
                have=$(grep -c "^${opt}=" "$config_file" || true)
            fi
            if [ "$have" = 0 ]; then
                warn "kernel config $opt not enabled — driver may not work"
                missing=1
            fi
        done
        [ "$missing" = 0 ] && say "kernel config check passed"
    else
        warn "could not check kernel config (no /proc/config.gz or /boot/config)"
    fi
}

check_kernel_config

# Copy source to /usr/src/
TARGET_DIR="/usr/src/${MOD_NAME}-${MOD_VERSION}"
say "copying source to $TARGET_DIR"
rm -rf "$TARGET_DIR"
mkdir -p "$TARGET_DIR/src/xpadneo"
cp -r "$XPADNEO_DIR"/* "$TARGET_DIR"/

# Remove existing DKMS module if present
if dkms status "$MOD_NAME" 2>/dev/null | grep -q "$MOD_NAME"; then
    say "removing existing DKMS module"
    dkms remove "$MOD_NAME/$MOD_VERSION" --all 2>/dev/null || true
fi

# Add, build, install via DKMS
say "adding to DKMS"
dkms add -m "$MOD_NAME" -v "$MOD_VERSION"

say "building (this may take a minute)"
dkms build -m "$MOD_NAME" -v "$MOD_VERSION"

say "installing"
dkms install -m "$MOD_NAME" -v "$MOD_VERSION"

# Load the module
say "loading module"
if modprobe "$MOD_NAME" 2>/dev/null; then
    say "module loaded successfully"
else
    warn "failed to load module (may need reboot)"
fi

# Install modprobe config (tells kernel to use xpadneo for Xbox controllers)
MODPROBE_CONF="/etc/modprobe.d/xpadneo.conf"
if [ -f "$XPADNEO_DIR/etc-modprobe.d/xpadneo.conf" ]; then
    say "installing modprobe config"
    cp "$XPADNEO_DIR/etc-modprobe.d/xpadneo.conf" "$MODPROBE_CONF"
    # Reload module to apply new alias
    rmmod hid-xpadneo 2>/dev/null || true
    modprobe hid-xpadneo 2>/dev/null || true
fi

# Install udev rules
UDEV_RULES="/etc/udev/rules.d"
if [ -d "$XPADNEO_DIR/etc-udev-rules.d" ]; then
    say "installing udev rules"
    cp "$XPADNEO_DIR/etc-udev-rules.d/"*.rules "$UDEV_RULES/" 2>/dev/null || true
    udevadm control --reload-rules 2>/dev/null || true
fi

# Verify (module name uses underscore in lsmod, dash in modprobe)
if lsmod | grep -q "^hid_xpadneo"; then
    say "hid_xpadneo is loaded"
else
    warn "hid_xpadneo not loaded yet (may load after reboot)"
fi

# Check dmesg for module load message
if dmesg | tail -20 | grep -q "loaded hid-xpadneo"; then
    say "dmesg confirms module loaded"
fi

say "installation complete"
echo ""
echo "Next steps:"
echo "  1. Reboot if the module did not load: sudo reboot"
echo ""
echo "  2. Connect your Xbox controller via Bluetooth:"
echo "     bluetoothctl scan on"
echo "     # Put controller in pairing mode (hold Xbox button + pair button)"
echo "     bluetoothctl pair <controller-address>"
echo "     bluetoothctl trust <controller-address>"
echo "     bluetoothctl connect <controller-address>"
echo ""
echo "  3. Verify controller is recognized:"
echo "     ls /dev/input/js*"
echo "     jstest /dev/input/js0  # if joystick package is installed"
