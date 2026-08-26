#!/bin/sh
# Install Rockchip's rkaiq 3A engine and the IMX219 tuning, so the camera has auto exposure,
# white balance and noise reduction instead of raw ISP defaults.
#
# Without this the ISP runs with no tuning and no 3A loop at all: the picture is green, noisy,
# and fixed at whatever exposure `mediad` pinned at startup — which is a still photo of one
# lighting condition, not a camera. The engine is what makes the head camera usable in a room
# whose lights change.
#
#   sudo sh /tmp/setup-rkaiq.sh
#   sudo /usr/local/sbin/robot-setup-rkaiq          # later, to re-check
#
# Full paths, because this advice gets copy-pasted and /tmp does not survive a reboot. The
# first run leaves a copy at the second path.
#
# **`hooks/preinstall` runs this too**, from `scripts/setup-rkaiq.sh` in the release, so a board
# provisioned before this existed gets the engine from an ordinary update rather than from
# somebody remembering a command. Same contract as `setup-gstreamer.sh`: never prompt, never be
# fatal, and say what happened.
#
#   --sensor NAME   tuning to install for (default imx219). Radxa's IQ package only ships
#                   imx219 and ov5647; anything else needs its own
#                   <sensor>_<module>_default.json in /etc/iqfiles.
#   --help
#
# Idempotent: the debs are only fetched when missing, the IQ patch is a no-op once applied, and
# the shim is rebuilt each run because it is cheap and because the kernel it probes can change
# under it.
#
# Radxa Zero 3W on the Armbian vendor kernel. Needs the camera overlay active — the ISP
# parameter and statistics nodes only exist there, and the engine has nothing to talk to
# without them.
#
# ── Why a vendor engine at all ────────────────────────────────────────────────
#
# There is no other 3A on this platform. libcamera's rkisp1 IPA drives the *mainline* rkisp1
# driver; this board runs Rockchip's vendor rkisp on the vendor kernel, which is also where the
# hardware encoder `mediad` depends on lives (`docs/project/media-bringup.md`). Choosing
# mainline to get an open 3A would cost the VPU, so the vendor engine — a prebuilt deb from
# Radxa's pool, taken as a direct download the way the MPP packages are — is the route.
#
# ── What differs from the prototype's version of this script ──────────────────
#
# **rkaiq's auto exposure stays enabled here, and that is the whole point.** The prototype
# patched `ae_calib CommCtrl.Enable` to 0 because its runtime owned sensor exposure: it ran a
# software AE loop over decoded MJPEG frames, and pinned exposure outright in laser mode, where
# rkaiq writing its own values at stream start turned the image black. Neither reason survives
# here — there is no laser mode, and `mediad` has no AE of its own. So the engine owns exposure,
# which is exactly the missing behaviour, and `mediad`'s `--exposure`/`--analogue-gain` become
# the values the sensor holds for the few frames before the engine's first stats arrive.
set -e

SENSOR=imx219
SELF=/usr/local/sbin/robot-setup-rkaiq
SHIM_SO=/usr/local/lib/rkaiq_modinfo_shim.so
IQ_DIR=/etc/iqfiles
DROP_IN_DIR=/etc/systemd/system/rkaiq_3A.service.d
PIN=/usr/local/bin/rkaiq-pin-sensor-mode

# Radxa's apt pool, as direct .deb downloads rather than an entry in sources.list — the same
# route `setup-gstreamer.sh` takes for MPP, and for the same reason: one pinned artifact each,
# no third-party repository left enabled on the robot afterwards.
POOL=https://radxa-repo.github.io/bullseye/pool/main
RKAIQ_DEB="$POOL/c/camera-engine-rkaiq/camera_engine_rkaiq_rk3568_arm64-fixed.deb"
IQ_DEB="$POOL/r/rockchip-iqfiles/rockchip-iqfiles-rk356x_0.1.16_all.deb"

# The sensor mode the engine must agree with `mediad` about. See `pin_mode` below for why this
# is here at all, and keep it in step with `pin_sensor_mode` in `mediad/src/pipeline.rs`.
SENSOR_W=1920
SENSOR_H=1080

say()  { printf '== %s\n' "$*"; }
warn() { printf 'WARNING: %s\n' "$*" >&2; }
die()  { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

usage() {
    sed -n '2,/^set -e/{/^set -e/d;s/^# \{0,1\}//;p;}' "$0"
    exit 0
}

while [ $# -gt 0 ]; do
    case "$1" in
        --sensor) SENSOR="${2:?--sensor needs a name}"; shift 2 ;;
        --help|-h) usage ;;
        *) die "unknown argument: $1 (try --help)" ;;
    esac
done

[ "$(id -u)" = 0 ] || die "run as root: sudo sh $0"

# Where this script and its shim source live, so a run from /tmp, from the release, or from a
# clone all find the C file. The release lays both out side by side under scripts/.
HERE="$(cd "$(dirname "$0")" && pwd)"
SHIM_SRC="${HERE}/rkaiq-modinfo-shim.c"

# ── the engine and its tuning ─────────────────────────────────────────────────

fetch_deb() {
    package="$1"
    url="$2"
    if dpkg -s "$package" >/dev/null 2>&1; then
        say "${package} already installed"
        return 0
    fi
    tmp="/tmp/${package}.deb"
    say "fetching ${package}"
    curl -fsSL -o "$tmp" "$url" || die "could not download ${url}
  The camera works without 3A — green, noisy and fixed-exposure — so this is not fatal to the
  robot, but it is fatal to image quality. Check the board's network and re-run:
    sudo ${SELF}"
    dpkg -i "$tmp" || die "dpkg could not install ${tmp}"
    rm -f "$tmp"
}

fetch_deb camera-engine-rkaiq-rk3568 "$RKAIQ_DEB"
fetch_deb rockchip-iqfiles-rk356x "$IQ_DEB"

say "installing ${SENSOR} tuning into ${IQ_DIR}"
mkdir -p "$IQ_DIR"
for f in /usr/share/rockchip-iqfiles-rk356x/"${SENSOR}"_*; do
    [ -e "$f" ] || continue
    [ -e "${IQ_DIR}/$(basename "$f")" ] || cp "$f" "$IQ_DIR/"
done

# `set -e` plus a glob that matches nothing would end the script here, which is the wrong
# outcome for a sensor nobody has tuning for: the engine still runs, badly, and saying so is
# more useful than stopping.
if ! ls "${IQ_DIR}/${SENSOR}"_*.json >/dev/null 2>&1; then
    warn "no IQ tuning for '${SENSOR}' in ${IQ_DIR}.
  Radxa's package ships imx219 and ov5647 only. The engine will run on raw ISP defaults, which
  looks green and noisy. Drop a ${SENSOR}_<module>_default.json in ${IQ_DIR} to fix it."
fi

# Two enum names in Radxa's IQ file are newer than the parser in Radxa's own engine deb. Left
# alone they are warnings, except that the AE strategy field silently falls back to a default —
# and AE is what this script exists to deliver, so it is not a warning we can shrug at.
#
# Only characterised for imx219: another sensor's file may not contain these names at all, and
# rewriting enums in tuning nobody has read is how you get an image that is subtly wrong.
if [ "$SENSOR" = imx219 ] && [ -f "${IQ_DIR}/imx219_rpi-camera-v2_default.json" ]; then
    say "checking IQ enum names against the engine's parser"
    python3 - <<'PY'
# Binary mode throughout: the file has CRLF line endings that text mode would rewrite, and a
# diff of the whole file is not what anyone wants out of this.
path = "/etc/iqfiles/imx219_rpi-camera-v2_default.json"
raw = open(path, "rb").read()
renames = [
    (b"AECV2_STRATEGY_MODE_LOWLIGHT_PRIOR", b"AECV2_STRATEGY_MODE_LOWLIGHT"),
    (b"CALIB_AWB_HDR_FRAME_CHOOSE_MODE_AUTO", b"CALIB_AWB_HDR_FR_CH_AUTO"),
]
changed = 0
for old, new in renames:
    n = raw.count(old)
    if n:
        raw = raw.replace(old, new)
        changed += n
if changed:
    open(path, "wb").write(raw)
    print(f"   renamed {changed} enum value(s) the parser does not know")
else:
    print("   enum names already match the parser")

# The prototype zeroed `ae_calib CommCtrl.Enable` here, to stop rkaiq's AE from fighting a
# runtime that owned exposure itself. Nothing owns exposure now, so AE must be ON — and these
# are the same physical boards, so a file left at 0 by that script (or by an apt upgrade, or by
# hand) is the likeliest way to end up with an engine running and still no auto exposure.
# Assert it rather than assume it: turning it back on is the entire point of this port.
import re
m = re.search(rb'("CommCtrl":\s*\{\s*"Enable":\s*)0', raw)
if m:
    raw = raw[: m.start()] + m.group(1) + b"1" + raw[m.end() :]
    open(path, "wb").write(raw)
    print("   auto exposure was DISABLED in this file — turned it back on")
else:
    print("   auto exposure is enabled")
PY
fi

# ── the ioctl shim ────────────────────────────────────────────────────────────
#
# Without this, `rkaiq_3A_server` segfaults on this kernel before it ever reaches a frame; the
# reasoning is in the C file's header. Built on the board because the struct size it probes for
# belongs to the running kernel.

if [ ! -f "$SHIM_SRC" ]; then
    die "no ${SHIM_SRC} beside this script.
  The release carries scripts/rkaiq-modinfo-shim.c next to scripts/setup-rkaiq.sh; a copy of
  the script alone cannot build the shim, and the engine segfaults without it."
fi

if ! command -v gcc >/dev/null 2>&1; then
    say "installing gcc, to build the shim"
    apt-get install -y --no-install-recommends gcc \
        || die "could not install gcc, so the shim cannot be built"
fi

say "building the ioctl shim"
gcc -shared -fPIC -O2 -o "$SHIM_SO" "$SHIM_SRC" -ldl \
    || die "could not build ${SHIM_SRC}"

# ── the sensor-mode pin ───────────────────────────────────────────────────────
#
# `rkaiq_3A_server` reads the sensor's resolution once, at startup, and programs the ISP input
# with it for every stream afterwards. The IMX219 boots in 3280x2464; `mediad` captures from
# the 1920x1080 mode (`pin_sensor_mode`, which is also what gets 30 fps rather than 21). The
# engine starts long before `mediad` does, so without this it reads the boot mode and every
# later capture dies with CIF_ISP_PIC_SIZE_ERROR — the camera delivers no frame at all, which
# reads as "the camera is broken" rather than "two components disagree by a resolution".
#
# So pin the mode before the engine starts, to the same geometry `mediad` pins. Both pinning it
# is deliberate: `mediad` cannot rely on this script having run, and this script cannot rely on
# `mediad` having started.

say "installing the sensor-mode pin at ${PIN}"
cat > "$PIN" <<PIN_HEAD
#!/bin/sh
# Pin the ${SENSOR} into its ${SENSOR_W}x${SENSOR_H} mode before rkaiq_3A_server reads it.
# Installed by scripts/setup-rkaiq.sh. Keep in step with pin_sensor_mode in
# mediad/src/pipeline.rs.
SENSOR="${SENSOR}"
WANT="${SENSOR_W}x${SENSOR_H}"
PIN_HEAD
cat >> "$PIN" <<'PIN_BODY'
# This runs at sysinit, which can be before the camera driver has probed — so wait for the
# entity rather than concluding there is no camera. Ten seconds, then give up quietly: a board
# with no camera module must still boot, and the engine's own log says the rest.
i=0
while [ "$i" -lt 40 ]; do
    for m in /dev/media*; do
        [ -e "$m" ] || continue
        entity=$(media-ctl -d "$m" -p 2>/dev/null \
            | sed -n "s/^- entity [0-9]*: \(m[0-9]*_[bf]_${SENSOR} [0-9-]*\).*/\1/p" \
            | head -1)
        if [ -n "$entity" ]; then
            if media-ctl -d "$m" --set-v4l2 "\"${entity}\":0[fmt:SRGGB10_1X10/${WANT}]"; then
                echo "pinned ${entity} to ${WANT} on ${m}"
            else
                echo "could not pin ${entity} to ${WANT} on ${m}" >&2
            fi
            exit 0
        fi
    done
    i=$((i + 1))
    sleep 0.25
done
echo "no ${SENSOR} entity appeared; leaving the sensor mode alone" >&2
exit 0
PIN_BODY
chmod 755 "$PIN"

# ── wiring, and the report ────────────────────────────────────────────────────

say "wiring the shim and the pin into rkaiq_3A.service"
mkdir -p "$DROP_IN_DIR"
cat > "${DROP_IN_DIR}/robot.conf" <<DROPIN
# Installed by scripts/setup-rkaiq.sh. Both lines are load-bearing: LD_PRELOAD is what keeps
# the engine from segfaulting on this kernel, and the pin is what keeps it from programming the
# ISP with the sensor's boot resolution.
[Service]
Environment=LD_PRELOAD=${SHIM_SO}
ExecStartPre=${PIN}
DROPIN

systemctl daemon-reload
systemctl enable rkaiq_3A >/dev/null 2>&1 || warn "could not enable rkaiq_3A"

# Leave a copy for the next person, exactly as setup-gstreamer.sh does.
if [ "$(cd "$(dirname "$0")" && pwd)/$(basename "$0")" != "$SELF" ]; then
    mkdir -p /usr/local/sbin
    install -m 755 "$0" "$SELF"
    install -m 644 "$SHIM_SRC" /usr/local/lib/rkaiq-modinfo-shim.c
fi

# The ISP nodes exist only on the vendor kernel with the camera overlay active. Before the
# reboot that brings those up there is nothing to restart into, and that is a normal state
# during provisioning rather than a failure.
if [ -e /dev/video8 ] && [ -e /dev/video9 ]; then
    systemctl restart rkaiq_3A || warn "rkaiq_3A would not restart"
    # The engine either survives its own startup or segfaults in it, and which one it did is the
    # single fact worth reporting. A second is enough for the latter.
    sleep 1
    if pgrep rkaiq_3A_server >/dev/null 2>&1; then
        say "rkaiq_3A_server is running — restart the camera stream for it to take effect:
    sudo systemctl restart mediad"
    else
        warn "rkaiq_3A_server is not running. The camera still works, with no 3A:
    journalctl -t rkaiq -b --no-pager | tail -40
  A segfault here means the shim did not match this kernel (${SHIM_SRC})."
    fi
else
    say "the ISP nodes are not present yet, so nothing to start.
  That is expected before the reboot into the vendor kernel with the camera overlay; rkaiq_3A
  is enabled and starts on the next boot."
fi

say "done"
