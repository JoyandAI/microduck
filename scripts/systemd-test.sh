#!/bin/sh
# Drive a real update against real systemd, on demand.
#
# Usage:  scripts/systemd-test.sh
#
# Needs Docker and nothing else. Not run by CI, deliberately — see the bottom of this comment.
#
# **What only this can answer.** Everything else in the tree tests the engine's decisions against a
# stub `systemctl`: which units it would restart, in what order, and what it does when one is
# missing. None of it can see whether a restart *happened*, because none of it has systemd. Three
# mechanisms are therefore untested everywhere else, and all three are what a board depends on after
# every update (docs/design/restart-order.md):
#
#   1. `on_apply` really restarts the units a release ships;
#   2. the deferred `systemd-run --on-active=5s` transient timer really fires, and really replaces
#      `updaterd` — a child process could not, because it would sit in the cgroup being killed;
#   3. `hooks/postinstall` really installs, enables and starts a unit a board has never had.
#
# And one failure only systemd can produce: a unit that installs cleanly and **cannot start**. That
# is bug 1 of `install-path-gap.md`, whose symptom on a board was `not healthy within 30s:
# unreachable` — a message naming neither the unit nor the command.
#
# **Docker rather than `systemd-nspawn`.** The plan in `install-path-gap.md` said nspawn on a Linux
# box, and rejected privileged containers *in CI*. That rejection stands; this is not CI. What
# changed the choice is the plan's own argument against putting this last: a check that can only run
# on a machine nobody develops on stops being read. Docker Desktop runs a privileged arm64 container
# with systemd as pid 1 on the laptop this is written on, which nspawn cannot, and the fidelity that
# matters here — real units, real cgroups, real transient timers — is identical.
#
# **Why not in CI.** It needs `--privileged` and the host's cgroup filesystem, which is a much
# larger thing to hand a workflow than any check here is worth, and the `board` job is already the
# slowest thing in the loop. This is the one to run when `on_apply`, the deferred restarts or the
# hook change — which is rarely, and is exactly when a stub stops being evidence.
set -eu

cd "$(dirname "$0")/.."

WORK=target/systemd-test
FIXTURE="$WORK/fixture"
# Where the tree is mounted in the container. Every path in the generated updater.toml is written
# for this, not for the host.
MOUNT=/fixture
IMAGE=duck-systemd-test
NAME=duck-systemd-test

command -v docker >/dev/null || {
    echo "docker is required: this needs systemd as pid 1, which a container provides and this" >&2
    echo "host does not." >&2
    exit 1
}

pass() { echo "    [ok] $*"; }
fail() { echo "    [FAIL] $*"; exit 1; }

cleanup() { docker rm -f "$NAME" >/dev/null 2>&1 || true; }
trap cleanup EXIT INT TERM

# ── the binaries ──
#
# Built inside the same userland the container runs, reusing the image `dev-push.sh --docker`
# already defines. On an arm64 host that is a native build and the target is the host.
echo "==> building updaterd and robotctl for the container"
docker build -q -t duck-dev-build -f scripts/dev-build.Dockerfile scripts/ >/dev/null
docker run --rm --platform linux/arm64 \
    -v "$PWD:/src" -w /src \
    -v duck-dev-cargo-registry:/usr/local/cargo/registry \
    -e CARGO_TARGET_DIR=/src/target/docker \
    duck-dev-build \
    cargo build --release -p updater -p robotctl --bins >/dev/null
BIN=target/docker/release

# ── the fixture ──
#
# Three releases: one to start on, one to move to, and one that installs a unit which cannot start.
echo "==> minting three signed releases"
rm -rf "$FIXTURE"
mkdir -p "$WORK"
cargo run -q -p test-support --example systemd-fixture -- "$FIXTURE" \
    --updaterd "$BIN/updaterd" \
    --robotctl "$BIN/robotctl" \
    --postinstall hooks/postinstall \
    --prefix "$MOUNT" \
    1.0.0 1.1.0 1.2.0:broken-unit | sed 's/^/    /'

# `local_dir` serves the newest version it can see, so what is offered is decided by what has been
# copied in. One at a time, and the harness controls when.
cp "$FIXTURE"/r/1.0.0/* "$FIXTURE/published/"

# ── the container ──
#
# systemd as pid 1 needs the host's cgroup tree and privileges. `/lib/systemd/systemd` rather than
# `/sbin/init`, which Debian's slim images do not provide.
echo "==> booting systemd"
docker build -q -t "$IMAGE" -f scripts/systemd-test.Dockerfile scripts/ >/dev/null
cleanup
docker run -d --name "$NAME" --privileged --cgroupns=host \
    -v /sys/fs/cgroup:/sys/fs/cgroup:rw \
    -v "$PWD/$FIXTURE:$MOUNT" \
    "$IMAGE" /lib/systemd/systemd >/dev/null

waited=0
until state="$(docker exec "$NAME" systemctl is-system-running 2>/dev/null || true)"
      [ "$state" = running ] || [ "$state" = degraded ]; do
    waited=$((waited + 1))
    [ "$waited" -lt 30 ] || fail "systemd did not come up: ${state:-no answer}"
    sleep 1
done
pass "systemd is pid 1 (${state})"

in_container() { docker exec "$NAME" sh -c "$1"; }

# ── the first install, which is a board's bootstrap ──
#
# Run from the tree rather than from a release, because on a bare board nothing is installed yet.
# `install` forces `on_apply` and the gate off — but the hook still runs, and installing the units is
# the hook's job, so this is also the first observation: does postinstall bring up a daemon a board
# has never had?
echo "==> installing 1.0.0 (the bootstrap path)"
in_container "$MOUNT/bin/updaterd --config $MOUNT/updater.toml install --from $MOUNT/published" \
    > "$WORK/install.log" 2>&1 || { sed 's/^/    /' "$WORK/install.log"; fail "install failed"; }

in_container "test -f /etc/systemd/system/fake-robotd.service" \
    || fail "postinstall did not install a unit the board had never seen"
in_container "systemctl is-active --quiet fake-robotd" \
    || fail "postinstall installed fake-robotd but it is not running"
in_container "systemctl is-active --quiet updaterd" \
    || fail "postinstall installed updaterd.service but it is not running"
pass "postinstall installed, enabled and started units on a board that had none"

live() { docker exec "$NAME" readlink "$MOUNT/opt/daemon/current" | sed 's|releases/||'; }
main_pid() { docker exec "$NAME" systemctl show -p MainPID --value "$1" | tr -d '\r'; }

[ "$(live)" = 1.0.0 ] || fail "current is $(live), expected 1.0.0"
pass "1.0.0 is live"

# ── the update, through the daemon ──
robotd_before="$(main_pid fake-robotd)"
updaterd_before="$(main_pid updaterd)"

cp "$FIXTURE"/r/1.1.0/* "$FIXTURE/published/"
echo "==> applying 1.1.0 through the running updaterd"
in_container "$MOUNT/opt/daemon/current/bin/robotctl update apply daemon" \
    > "$WORK/apply.log" 2>&1 || { sed 's/^/    /' "$WORK/apply.log"; fail "apply failed"; }

[ "$(live)" = 1.1.0 ] || fail "current is $(live) after applying 1.1.0"
pass "1.1.0 is live"

# 1. `on_apply` restarted what the release ships. The gate above already implies the unit came back;
#    the PID is what says it was replaced rather than left alone.
robotd_after="$(main_pid fake-robotd)"
[ -n "$robotd_before" ] && [ "$robotd_before" != "$robotd_after" ] \
    || fail "fake-robotd was not restarted (pid $robotd_before throughout)"
pass "on_apply restarted the daemon the release ships (pid $robotd_before -> $robotd_after)"

# 2. The deferred restart. Scheduled five seconds after the reply, through a transient unit, so this
#    is the one observation a child process could not make: `updaterd` is replaced *by systemd*
#    after the operation it was performing has finished.
echo "==> waiting for the deferred updaterd restart"
waited=0
until [ "$(main_pid updaterd)" != "$updaterd_before" ]; do
    waited=$((waited + 1))
    [ "$waited" -lt 40 ] || fail "updaterd was never restarted (pid $updaterd_before after ${waited}s)"
    sleep 1
done
pass "the transient timer replaced updaterd after ${waited}s (pid $updaterd_before -> $(main_pid updaterd))"

# And the successor is the *new* binary, which is the point of restarting it at all. Its own
# published identity is the answer, because that is what the startup reconciliation reads.
waited=0
until in_container "grep -q 'releases/1.1.0/bin/updaterd' /run/updaterd/identity.json 2>/dev/null"; do
    waited=$((waited + 1))
    [ "$waited" -lt 20 ] || fail "the restarted updaterd is not running 1.1.0"
    sleep 1
done
pass "the restarted updaterd is running 1.1.0"

# And its startup reconciliation found nothing to fix. Asserted as an absence, because the positive
# line is only logged when *every* unit is accounted for and `fake-robotd` is a `sleep` that publishes
# no identity — which reads as `Unknown`, and being left alone is exactly right for it.
if in_container "journalctl -u updaterd -b --no-pager | grep -q 'still running the release it had'"
then
    fail "the successor found a unit stale, so a restart during the update did not take"
fi
pass "its startup reconciliation restarted nothing, as it should"

# ── the injection: a unit that installs and cannot start ──
#
# Bug 1's class. The unit arrives with the release, postinstall installs and enables it, `enable
# --now` fails and is only a warning — and then `on_apply` restarts it, which is not. The update must
# fail, roll back, and say which unit.
cp "$FIXTURE"/r/1.2.0/* "$FIXTURE/published/"
echo "==> applying 1.2.0, which ships a unit that cannot start"
# Exit status is deliberately not the check. A rollback is a *successful* call that reports an
# unsuccessful outcome — `robotctl` exits 0 having told you what happened, and reading the status
# instead would assert a different contract than the one that matters here.
in_container "$MOUNT/opt/daemon/current/bin/robotctl update apply daemon" \
    > "$WORK/broken.log" 2>&1 || true
grep -q rolled_back "$WORK/broken.log" \
    || { sed 's/^/    /' "$WORK/broken.log"; fail "the update did not roll back"; }
grep -q '"outcome": "applied"' "$WORK/broken.log" \
    && fail "the update reported success despite a unit that cannot start"
pass "the update rolled back rather than reporting success"

[ "$(live)" = 1.1.0 ] || fail "rolled back to $(live), expected 1.1.0"
pass "rolled back to 1.1.0"

grep -qi "broken" "$WORK/broken.log" \
    || { sed 's/^/    /' "$WORK/broken.log"; fail "the reason does not name the unit that failed"; }
pass "the reason names the unit, not 'unreachable'"

in_container "systemctl is-active --quiet fake-robotd" \
    || fail "fake-robotd is down after the rollback"
pass "the daemon is running again after the rollback"

echo
echo "==> systemd checks passed"
