#!/bin/sh
# Provision a freshly flashed board, end to end, in as few commands as a reboot allows.
#
#   export DUCK_TOKEN=...                      # only while the repository is private
#   curl -fsSL -H "Authorization: Bearer $DUCK_TOKEN" .../provision.sh -o /tmp/provision.sh
#   sudo DUCK_TOKEN="$DUCK_TOKEN" sh /tmp/provision.sh
#   sudo reboot
#   sudo /usr/local/sbin/robot-provision
#
# Four commands, and `robotctl health` works when the last one finishes.
#
# This orchestrates `setup-board.sh`, `migrate-network.sh` and `install.sh`; it does not
# duplicate them. They stay separately runnable, and they stay separate for the reasons each
# one states — different lifetimes, different risks. What they never had is anything tying
# them together, so the operator was the glue: fetch, run, fetch, run, reboot, re-run, re-run,
# fetch, run. Nine steps of clerical work with a reboot in the middle, every one of which is
# an opportunity to do them in the wrong order or forget the second half of the wifi cutover.
#
# ## Why it always asks for a reboot
#
# Not because it always needs one, but because *deciding* would mean either re-deriving what
# the two scripts already decided or parsing their output, and both drift. A reboot on a board
# being provisioned costs thirty seconds. What it buys is worth more than that:
#
#   - Phase 2 runs against live boot config, so the motor UART exists by the time `robotd`
#     starts and a bench board's health report is about its servos rather than its overlays.
#   - Your shell after the reboot is a *new login session*, which is what makes the `robot`
#     group live without `newgrp`. See `create_group`.
#
# ## Why it does not reboot on its own
#
# `setup-board.sh` and `migrate-network.sh` both state that they never do, and this keeps
# their rule. The reboot after a wifi cutover is the one moment a headless board can become
# unreachable — the backstop in `migrate-network.sh` exists for exactly that — and it belongs
# to whoever is in a position to go and find the board, not to a script.
set -eu

# ── knobs ────────────────────────────────────────────────────────────────────
#
# Same names as `install.sh`, which is where most of them end up: a fork or a pinned tag is
# one decision for the whole bring-up rather than one per script.
# Kept separately from the resolved values below, because phase 2 has to tell "the operator
# asked for this" apart from "this is the default" — the state file must not silently win over
# something typed on the phase 2 command line. See `load_state`.
ENV_REPO="${DUCK_REPO:-}"
ENV_REF="${DUCK_REF:-}"
ENV_TOKEN="${DUCK_TOKEN:-}"
ENV_DEV_KEY="${DUCK_DEV_KEY:-}"
ENV_FORCE="${DUCK_FORCE_REINSTALL:-}"

REPO="${ENV_REPO:-pollen-robotics/microduck_daemon}"
REF="${ENV_REF:-main}"
RAW="https://raw.githubusercontent.com/${REPO}/${REF}/scripts"

# For a private repository: a token with read access to contents. Carried across the reboot in
# the state file rather than asked for twice — see `save_state` for why not `~/.profile`.
TOKEN="$ENV_TOKEN"

# Path to `team.dev.pub`, to make this a dev board. Usually somewhere under /tmp because it
# arrived by `scp`, which is why phase 1 copies it somewhere that survives the reboot.
DEV_KEY="$ENV_DEV_KEY"

# Passed straight through to install.sh.
FORCE_REINSTALL="$ENV_FORCE"

# ── paths ────────────────────────────────────────────────────────────────────

SELF=/usr/local/sbin/robot-provision
STATE_DIR=/var/lib/robot
STATE="${STATE_DIR}/provision.env"
# Where a dev key is parked across the reboot. A public key, so 0644 is right; the point of
# moving it is only that /tmp does not survive the reboot this asks for.
DEV_KEY_KEPT="${STATE_DIR}/team.dev.pub"

# Persisted copies the two board scripts leave behind, which is what phase 2 should prefer:
# they are on disk, and re-fetching would be a second chance for the network to fail.
SETUP_SELF=/usr/local/sbin/robot-setup-board
MIGRATE_SELF=/usr/local/sbin/robot-migrate-network

say()  { printf '\033[1m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[33mwarning:\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[31merror:\033[0m %s\n' "$*" >&2; exit 1; }

# ── helpers ──────────────────────────────────────────────────────────────────

# Fetch one sibling script into $2. Unlike the `fetch_cmd` in setup-board.sh, which prints a
# command for a human, this one runs it.
fetch() {
    # $1 script name, $2 destination
    if [ -n "$TOKEN" ]; then
        curl -fsSL -H "Authorization: Bearer ${TOKEN}" -o "$2" "${RAW}/$1" && return 0
    else
        curl -fsSL -o "$2" "${RAW}/$1" && return 0
    fi

    # A private repository answers 404, not 401, for a path with no credentials, so `curl -f`
    # reports what looks like a wrong URL. Say which of the two it is rather than leaving the
    # operator to guess at a typo that is not there.
    if [ -n "$TOKEN" ]; then
        die "could not fetch $1 from ${REPO}@${REF}.
  A token was supplied, so a 404 here means it cannot read this repository — check that it
  has Contents:Read on ${REPO}, and that any SSO authorisation was granted."
    fi
    die "could not fetch $1 from ${REPO}@${REF}.
  No DUCK_TOKEN was supplied. While the repository is private every fetch needs one, and
  GitHub answers 404 rather than 401, so this looks like a wrong URL and is not."
}

# Leave a copy at $SELF, so the command this prints for after the reboot exists.
#
# Not possible when piped (`curl | sh`): there is no file to copy, `$0` is the shell. That is
# why the documented invocation downloads first — and why this refuses rather than carrying on
# into a state whose second half cannot be reached.
persist_self() {
    case "$0" in
        */*) ;;
        *) die "run this from a file, not a pipe:
  curl -fsSL ${RAW}/provision.sh -o /tmp/provision.sh
  sudo sh /tmp/provision.sh
  Phase 2 runs after a reboot, so there has to be something left on disk to run." ;;
    esac

    if [ "$(readlink -f "$0")" = "$SELF" ]; then
        return 0
    fi
    install -m 755 "$0" "$SELF"
}

# Write what phase 2 needs to know, 0600, root-only.
#
# This is where the token lives between the two phases. Not `~/.profile`, which was the
# earlier advice: that is a file the operator keeps, readable by their own processes, and it
# outlives provisioning with a credential in it that nobody remembers putting there. This one
# is root-only, in the daemon's own state directory, and `finish` deletes it.
save_state() {
    mkdir -p "$STATE_DIR"
    # Created before it is written, and chmod'ed before anything secret goes in: a token must
    # never exist even briefly in a world-readable file.
    : > "$STATE"
    chmod 600 "$STATE"
    {
        printf 'DUCK_REPO=%s\n' "$REPO"
        printf 'DUCK_REF=%s\n' "$REF"
        printf 'DUCK_TOKEN=%s\n' "$TOKEN"
        printf 'DUCK_DEV_KEY=%s\n' "$1"
        printf 'DUCK_FORCE_REINSTALL=%s\n' "$FORCE_REINSTALL"
        printf 'PROVISION_BOOT_ID=%s\n' "$(boot_id)"
    } > "$STATE"
}

# Read it back. Anything the operator actually typed wins over the file, so a token that was
# wrong the first time is corrected on the phase 2 command line rather than by editing a
# root-owned file — the file is a convenience for crossing the reboot, not the authority.
load_state() {
    [ -f "$STATE" ] || return 1

    # shellcheck disable=SC1090  # a generated file of KEY=value lines, written by save_state.
    . "$STATE"

    REPO="${ENV_REPO:-${DUCK_REPO:-$REPO}}"
    REF="${ENV_REF:-${DUCK_REF:-$REF}}"
    TOKEN="${ENV_TOKEN:-${DUCK_TOKEN:-}}"
    DEV_KEY="${ENV_DEV_KEY:-${DUCK_DEV_KEY:-}}"
    FORCE_REINSTALL="${ENV_FORCE:-${DUCK_FORCE_REINSTALL:-}}"
    RAW="https://raw.githubusercontent.com/${REPO}/${REF}/scripts"
    return 0
}

# Which boot this is, as the kernel's own opaque id — it changes on every boot and on nothing
# else. Phase 1 records it and phase 2 compares, which is how "have you actually rebooted" is
# answered without trusting anything else.
#
# Deliberately not the state file's timestamp against uptime, which was the first attempt and
# is wrong on this hardware: the board has no battery-backed RTC, starts at 1970, and NTP steps
# the clock *during* provisioning — `deploy/README.md` calls that out for TLS. File-time
# arithmetic would be comparing two clocks that disagree by decades. There is no clock in this.
boot_id() {
    cat /proc/sys/kernel/random/boot_id 2>/dev/null || true
}

# Is this still the boot that ran phase 1?
#
# Phase 2 installs the daemon, and until the reboot the overlay is staged rather than live, so
# there is no /dev/ttyS2 — `robotd` would start, see no bus, and report a hardware fault that
# is really an operator who skipped a step. Answers no when the id is unavailable, which fails
# towards letting provisioning continue: refusing on a board that cannot tell us would strand
# it with no way forward at all.
same_boot_as_phase_one() {
    _now="$(boot_id)"
    [ -n "$_now" ] || return 1
    [ -n "${PROVISION_BOOT_ID:-}" ] || return 1
    [ "$_now" = "$PROVISION_BOOT_ID" ]
}

# Create the `robot` group and put the operator in it — in phase 1, which is the whole point.
#
# `install.sh` does this too, and correctly, but it runs *last*: by then the operator's shell
# started before the group existed, a process's groups are fixed at exec, and no one can add a
# group to a running process. So provisioning ended with `newgrp robot` or a logout, on a
# board that had just told them everything was fine.
#
# Doing it here means the reboot does that work: the operator reconnects, that is a new login
# session, and it has the group already. Nothing to run, nothing to explain.
#
# Needs nothing from the release — `groupadd --system` is self-contained. install.sh still
# installs the sysusers.d file from the verified release afterwards, and still owns the
# decision; it just finds the group present and the operator already a member, and says so.
create_group() {
    if getent group robot >/dev/null; then
        say "robot group exists"
    else
        say "creating the robot group"
        groupadd --system robot \
            || die "could not create the robot group; neither daemon will start without it"
    fi

    operator="${SUDO_USER:-}"
    if [ -z "$operator" ] || [ "$operator" = root ]; then
        return 0
    fi

    if id -nG "$operator" 2>/dev/null | tr ' ' '\n' | grep -qx robot; then
        say "${operator} is in the robot group"
        return 0
    fi

    if usermod -aG robot "$operator"; then
        say "added ${operator} to the robot group — the reboot below makes it live"
    else
        warn "could not add ${operator} to the robot group; robotctl will need sudo"
    fi
}

# Park a dev key somewhere that survives the reboot, and answer with where it went.
#
# The documented way to get one onto a board is `scp` into /tmp, and /tmp is cleared by the
# reboot between the two phases. Without this, `DUCK_DEV_KEY=/tmp/team.dev.pub` produces a
# board that provisions cleanly and is silently not a dev board — the failure only shows up
# later as `--ref` being refused, which reads like a broken release.
keep_dev_key() {
    [ -n "$DEV_KEY" ] || return 0
    [ -f "$DEV_KEY" ] || die "DUCK_DEV_KEY=${DEV_KEY} is not a readable file.
  Pass the *public* half — team.dev.pub. install.sh validates it properly in phase 2; this
  only checks it is there, because finding out after a reboot is worse."

    mkdir -p "$STATE_DIR"
    install -m 644 "$DEV_KEY" "$DEV_KEY_KEPT"
    say "kept the dev key at ${DEV_KEY_KEPT} — /tmp does not survive the reboot"
}

# ── phases ───────────────────────────────────────────────────────────────────

phase_one() {
    say "phase 1: board and network"
    create_group
    keep_dev_key

    tmp=/tmp/setup-board.sh
    fetch setup-board.sh "$tmp"
    sh "$tmp"

    tmp=/tmp/migrate-network.sh
    fetch migrate-network.sh "$tmp"
    sh "$tmp"

    if [ -n "$DEV_KEY" ]; then
        save_state "$DEV_KEY_KEPT"
    else
        save_state ""
    fi

    cat <<EOF

$(printf '\033[1m==>\033[0m') phase 1 done. Reboot, then finish:

  sudo reboot
  sudo ${SELF}

Both changes above are staged rather than live — a device-tree overlay and a network stack
cannot swap under a running kernel. The reboot also refreshes your login session, which is
what makes the robot group work without \`newgrp\`.
EOF
}

phase_two() {
    say "phase 2: confirm the board, then install the daemon"

    # The persisted copies, which is what those scripts leave behind for exactly this moment.
    # Re-fetching would work and would also be a second chance for the network to fail.
    if [ -x "$SETUP_SELF" ]; then
        "$SETUP_SELF"
    else
        tmp=/tmp/setup-board.sh
        fetch setup-board.sh "$tmp"
        sh "$tmp"
    fi

    # Separately, and unconditionally: this run is what retires the wifi backstop. Left armed,
    # any later boot where wifi is merely slow reverts this board to netplan.
    if [ -x "$MIGRATE_SELF" ]; then
        "$MIGRATE_SELF"
    else
        tmp=/tmp/migrate-network.sh
        fetch migrate-network.sh "$tmp"
        sh "$tmp"
    fi

    tmp=/tmp/install.sh
    fetch install.sh "$tmp"

    DUCK_REPO="$REPO"
    DUCK_REF="$REF"
    DUCK_TOKEN="$TOKEN"
    DUCK_FORCE_REINSTALL="$FORCE_REINSTALL"
    export DUCK_REPO DUCK_REF DUCK_TOKEN DUCK_FORCE_REINSTALL
    if [ -n "$DEV_KEY" ]; then
        DUCK_DEV_KEY="$DEV_KEY"
        export DUCK_DEV_KEY
    fi
    sh "$tmp"

    finish
}

# Take the credential back out. Provisioning is over, and a token that stays behind is one
# nobody remembers is there — the state file's whole justification was crossing the reboot.
finish() {
    rm -f "$STATE"
    say "provisioning complete; removed ${STATE}"
    cat <<'EOF'

  robotctl health

That works in this shell: the group predates your current login session, because phase 1
created it before the reboot.
EOF
}

main() {
    [ "$(id -u)" = 0 ] || die "run as root — re-run that same command with sudo"
    command -v curl >/dev/null 2>&1 || die "curl is required"

    if load_state; then
        if same_boot_as_phase_one; then
            die "phase 1 has run but this board has not rebooted since.
  Phase 2 installs the daemon, and until the reboot the overlay is staged rather than live —
  so there is no /dev/ttyS2, and robotd would start and report a hardware fault that is
  really a missing reboot.
    sudo reboot
    sudo ${SELF}"
        fi
        persist_self
        phase_two
        return 0
    fi

    persist_self
    phase_one
}

main "$@"
