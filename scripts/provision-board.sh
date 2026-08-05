#!/bin/sh
# Provision a board from your own machine, in one command.
#
#   export DUCK_TOKEN=...              # only while the repository is private
#   ./scripts/provision-board.sh radxa-zero3
#
# The only script in this directory that runs on the *operator's* machine rather than on a
# robot. Everything it does, it does over ssh; nothing here is installed anywhere.
#
# What it is for is the seam in the middle. `provision.sh` reboots the board and finishes on
# its own, which is right, but from the outside that looks like an ssh session dying followed
# by an unknown interval and a guess about when to log back in. This waits for the board to
# come back, streams the log the unattended half writes, and ends on `robotctl health` — so
# provisioning is one command with continuous output instead of three with a gap.
#
#   --ref BRANCH      provision from a branch instead of main
#   --local           send this clone's scripts/provision.sh instead of having the board fetch
#                     it. What makes testing an unpushed branch possible.
#   --no-dev-key      do not install the team dev key, for a board that should only take
#                     releases. The default is to send it when ~/.duck-keys/team.dev.pub exists.
#   --dev-key PATH    somewhere else to find it.
#
# Needs `ssh` and `scp`, an account on the board that can `sudo`, and nothing else. It expects
# to be able to prompt for the sudo password, so it allocates a terminal for that one command.
set -eu

DEV_KEY_DEFAULT="${HOME}/.duck-keys/team.dev.pub"

HOST=""
REF=""
DEV_KEY="$DEV_KEY_DEFAULT"
NO_DEV_KEY=""
USE_LOCAL=""

# How long to wait for the board to come back after its reboot. Generous on purpose: a first
# boot after an overlay change and a wifi cutover is the slowest this board will ever be, and
# giving up early would report a failure that is really impatience.
BOOT_TIMEOUT=180

# Board-side paths. Duplicated from provision.sh rather than derived, because this script is
# copied to a laptop and run from anywhere — there is nothing to source.
STATE=/var/lib/robot/provision.env
LOG=/var/lib/robot/provision.log

say()  { printf '\033[1m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[33mwarning:\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[31merror:\033[0m %s\n' "$*" >&2; exit 1; }

usage() {
    sed -n '2,22p' "$0" | sed 's/^# \{0,1\}//'
    exit "${1:-0}"
}

while [ $# -gt 0 ]; do
    case "$1" in
        --ref)        REF="${2:?--ref needs a branch}"; shift 2 ;;
        --dev-key)    DEV_KEY="${2:?--dev-key needs a path}"; shift 2 ;;
        --no-dev-key) NO_DEV_KEY=1; shift ;;
        --local)      USE_LOCAL=1; shift ;;
        -h|--help)    usage 0 ;;
        -*)           die "unknown option: $1" ;;
        *)            [ -z "$HOST" ] || die "one board at a time"; HOST="$1"; shift ;;
    esac
done

[ -n "$HOST" ] || usage 2

command -v ssh >/dev/null 2>&1 || die "ssh is required"
command -v scp >/dev/null 2>&1 || die "scp is required"

# Non-interactive ssh, for the polling and the file checks. BatchMode so a board that has gone
# away fails in seconds instead of sitting on a password prompt nobody is watching.
rsh() {
    ssh -o BatchMode=yes -o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new "$HOST" "$@"
}

# True while the board still has provisioning left to do. `provision.sh` removes the state file
# when it finishes, which makes "are we done" a question with a file for an answer rather than
# a log line to pattern-match.
still_provisioning() {
    rsh "test -f ${STATE}" >/dev/null 2>&1
}

# ── checks that are cheaper to fail now than halfway ─────────────────────────

say "checking ${HOST}"
rsh true >/dev/null 2>&1 \
    || die "cannot ssh to ${HOST} without a prompt.
  Set up a key first — this needs to reconnect by itself after the board reboots, which a
  password prompt cannot survive:
    ssh-copy-id ${HOST}"

if [ -z "${DUCK_TOKEN:-}" ]; then
    warn "DUCK_TOKEN is not set. While the repository is private every fetch on the board
  needs it, and GitHub answers 404 rather than 401, so it will look like a wrong URL.
  Continuing in case the repository is public by now."
fi

if [ -n "$NO_DEV_KEY" ]; then
    DEV_KEY=""
elif [ ! -f "$DEV_KEY" ]; then
    if [ "$DEV_KEY" != "$DEV_KEY_DEFAULT" ]; then
        die "--dev-key ${DEV_KEY} is not a readable file"
    fi
    warn "no ${DEV_KEY_DEFAULT}, so this board will not accept --ref builds. Pass
  --dev-key PATH if it lives elsewhere, or --no-dev-key to stop saying this."
    DEV_KEY=""
fi

# ── put what the board needs where the board can reach it ────────────────────

if [ -n "$DEV_KEY" ]; then
    say "sending the dev key"
    scp -q -o StrictHostKeyChecking=accept-new "$DEV_KEY" "${HOST}:/tmp/team.dev.pub" \
        || die "could not copy ${DEV_KEY} to ${HOST}"
fi

# The local copy is the whole point of `--local`: it provisions a board with a `provision.sh`
# that has not been pushed anywhere, which is the only way to test a change to it without
# merging first. Everything the script then fetches still comes from --ref, so a full test of a
# branch is `--local --ref that-branch`.
if [ -n "$USE_LOCAL" ]; then
    _local="$(dirname "$0")/provision.sh"
    [ -f "$_local" ] || die "--local needs ${_local}, and it is not there.
  Run this from a clone, or drop --local and let the board fetch it from ${REF:-main}."
    say "sending this clone's provision.sh"
    scp -q "$_local" "${HOST}:/tmp/provision.sh" || die "could not copy provision.sh"
else
    _raw="https://raw.githubusercontent.com/pollen-robotics/microduck_daemon/${REF:-main}/scripts/provision.sh"
    say "having the board fetch provision.sh from ${REF:-main}"
    # Fetched by the board rather than by this machine and copied over: the board is the one
    # that has to be able to reach GitHub with that token, and finding out here would prove
    # the wrong thing.
    rsh "curl -fsSL ${DUCK_TOKEN:+-H \"Authorization: Bearer ${DUCK_TOKEN}\"} '${_raw}' -o /tmp/provision.sh" \
        || die "the board could not fetch provision.sh from ${REF:-main}.
  A private repository answers 404 rather than 401, so this is either a missing DUCK_TOKEN, a
  token without Contents:Read on the repository, or a branch name that does not exist."
fi

# ── phase 1, which ends in a reboot that takes the connection with it ────────

say "starting provisioning — the board will reboot and this will wait for it"
echo

_env="DUCK_TOKEN='${DUCK_TOKEN:-}'"
[ -z "$REF" ]     || _env="${_env} DUCK_REF='${REF}'"
[ -z "$DEV_KEY" ] || _env="${_env} DUCK_DEV_KEY=/tmp/team.dev.pub"

# `-t` so sudo can prompt for a password, and the exit status deliberately ignored: this
# command ends by rebooting the machine it is running on, so ssh reporting a dropped connection
# is the *expected* outcome. Whether it worked is decided below, by looking at the board.
ssh -t -o StrictHostKeyChecking=accept-new "$HOST" \
    "sudo env ${_env} sh /tmp/provision.sh" || true

echo
say "waiting for ${HOST} to come back (up to ${BOOT_TIMEOUT}s)"

# The board is mid-reboot, so it may still answer for a moment. Wait for it to go before
# waiting for it to return, or this races and declares success against the dying session.
_waited=0
while [ "$_waited" -lt 20 ]; do
    rsh true >/dev/null 2>&1 || break
    sleep 2
    _waited=$((_waited + 2))
done

_waited=0
while ! rsh true >/dev/null 2>&1; do
    sleep 5
    _waited=$((_waited + 5))
    if [ "$_waited" -ge "$BOOT_TIMEOUT" ]; then
        die "${HOST} did not come back within ${BOOT_TIMEOUT}s.
  If the wifi cutover failed, the backstop restores netplan and reboots — which can take a
  second boot. Try again, or look at the board:
    ssh ${HOST} 'sudo cat ${LOG}'"
    fi
done
say "back after ~${_waited}s"

# ── phase 2, which is running unattended on the board ────────────────────────

# Polled rather than `tail -f`: the connection has to survive a service that may still be
# starting, and a poll that reconnects each time cannot be left holding a dead channel. Only
# new bytes are printed, so this reads like a stream.
_seen=0
_quiet=0

# Print whatever has been appended since the last call. Returns 1 when there was nothing, which
# is what the stall detection counts.
drain_log() {
    _size="$(rsh "sudo sh -c 'test -f ${LOG} && wc -c < ${LOG} || echo 0'" 2>/dev/null || echo "$_seen")"
    # Digits only: a stray line from ssh or sudo in that output would otherwise reach the
    # arithmetic below and abort the script for a cosmetic reason.
    _size="$(printf '%s' "$_size" | tr -dc '0-9')"
    [ -n "$_size" ] || _size=$_seen

    if [ "$_size" -gt "$_seen" ]; then
        rsh "sudo tail -c +$((_seen + 1)) ${LOG}" 2>/dev/null || true
        _seen=$_size
        return 0
    fi
    return 1
}

while :; do
    if drain_log; then
        _quiet=0
    else
        _quiet=$((_quiet + 3))
    fi

    if ! still_provisioning; then
        # One more read before leaving. `provision.sh` writes its closing lines and *then*
        # removes the state file, so a loop that breaks the moment the file is gone drops the
        # last thing it said — including which token ended up where, and whether the board came
        # out a dev board. Which is the part worth reading.
        drain_log || true
        break
    fi

    # A board that has stopped writing and still has a state file has either failed or is
    # waiting on something slow. Say so rather than looking identical to progress.
    if [ "$_quiet" -ge 120 ]; then
        warn "nothing new in ${LOG} for two minutes and provisioning has not finished.
  Still waiting, but worth a look:  ssh ${HOST} 'systemctl status robot-provision'"
        _quiet=0
    fi
    sleep 3
done

echo
say "provisioning finished"

# The health report is the point of all of it, and it is also the thing most likely to have
# something to say — a bench board with no servos powered reports unhealthy, correctly.
rsh "robotctl health" || warn "robotctl health did not report cleanly. On a board with no
  servos powered that is the honest answer, not a failed install. The full log is at:
    ssh ${HOST} 'sudo cat ${LOG}'"
