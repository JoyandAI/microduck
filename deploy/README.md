# Deployment: OS-level configuration

Status: draft · Date: 2026-07-28 · Owner: pierre

Config that belongs to the *robot image* rather than to any one service. Service units live
next to their service (`updater/systemd/`, `robotd/systemd/`); anything robot-wide is here.

| | |
|---|---|
| `updater.toml` | the config a client robot ships with, installed to `/etc/robot/updater.toml` |
| `trusted_keys/` | release public keys — the trust anchor, installed to `/etc/robot/trusted_keys/` |
| `journald.conf.d/10-robot.conf` | journal persistence and size caps |

## Installing a robot from scratch

Two steps, because they answer to different things. `setup-board.sh` is OS-level bring-up —
device-tree overlays, ONNX Runtime — which changes rarely and needs a reboot. `install.sh`
installs a signed daemon release, which happens on every update. Conflating them would mean
every update re-litigating boot configuration.

Plus one that will not last: `migrate-network.sh` exists only because Armbian's stock image
ships netplan, and it is numbered 1b below rather than 2 for that reason — it gets deleted, not
maintained.

**0. While the repository is private, set up the token first.** Every fetch below is from a
private repo, and `raw.githubusercontent.com` answers **404** rather than 401 for a private
path with no credentials — so `curl -f` reports what looks like a wrong URL. Create a
fine-grained token (resource owner `pollen-robotics`, repository `microduck_daemon`,
permission **Contents: Read-only**) and put it in the shell you are about to work in. That one
permission also covers the release assets `updaterd` fetches later.

```bash
export DUCK_TOKEN=github_pat_replace_with_your_token
```

```bash
RAW=https://raw.githubusercontent.com/pollen-robotics/microduck_daemon/main/scripts
```

```bash
AUTH="Authorization: Bearer $DUCK_TOKEN"
```

These three are re-exported after any reboot. Once the repository is public, drop `AUTH` and
the `-H` from each fetch.

This token is for *your shell*, to fetch scripts. The one `updaterd` needs afterwards to reach
release assets is a different decision with different consequences — see
[While the repository is private, a robot needs a token](#-while-the-repository-is-private-a-robot-needs-a-token)
below. Step 2 is where the two meet: passing `DUCK_TOKEN` to `install.sh` is what writes the
robot's systemd drop-in, so it outlives the shell.

**1. Prepare the board.** Idempotent, and it never reboots on its own: if it changes boot
config it says so and stops, and running it again afterwards continues.

```bash
curl -fsSL -H "$AUTH" "$RAW/setup-board.sh" -o /tmp/setup-board.sh
```

```bash
sudo sh /tmp/setup-board.sh
```

No token on the `sudo` line, unlike step 2: the only thing this script downloads is the ONNX
Runtime tarball from a public `microsoft/onnxruntime` release. It needs credentials to be
*fetched*, not to run.

The first run copies itself to `/usr/local/sbin/robot-setup-board`, so after the reboot it
asks for there is still something to run:

```bash
sudo reboot
```

```bash
sudo /usr/local/sbin/robot-setup-board
```

`/tmp` is cleared on reboot, and a script whose whole job is *change boot config, reboot,
confirm* should not delete itself in the middle of that.

It ends with a status block — motor bus, ONNX Runtime, clock — so "is this board ready" is a
question you can ask on its own, not only as a side effect of installing something.

The one thing it fixes that is otherwise very hard to diagnose: Armbian ships
`overlay_prefix=rk35xx`, but the RK3566 shares device-tree overlays with the RK3568 and they
are named `rk3568-*.dtbo`. With the wrong prefix the loader finds nothing, the board boots
happily, and there is simply no `/dev/ttyS2`. `armbian-config`'s overlay editor crashes for
the same reason, so the file is patched directly.

⚠ A kernel upgrade that repoints `/boot/{Image,dtb,uInitrd}` can undo it. A board that stops
seeing its motors after an `apt upgrade` needs this re-run.

⚠ The `kernel console` line in that status block reads `until the reboot` when the script has
already fixed it and only the running kernel is stale — `/proc/cmdline` cannot change without
a reboot. Anything else there is a real finding.

**1b. Move wifi to NetworkManager** — while Armbian's stock image still ships netplan. A
separate script because it has a different lifetime and a different risk: it disappears the day
we build an image with NetworkManager in it, and it is the one step that can make a headless
board unreachable. `setup-board.sh` only *checks* which stack owns wifi, and warns, because
`configd` drives NetworkManager over D-Bus — a board still on netplan answers every `net.*`
call with "no such device".

```bash
curl -fsSL -H "$AUTH" "$RAW/migrate-network.sh" -o /tmp/migrate-network.sh
```

```bash
sudo sh /tmp/migrate-network.sh
```

```bash
sudo reboot
```

```bash
sudo /usr/local/sbin/robot-migrate-network
```

Four commands, not three: it must run **twice**, either side of the reboot. The first run arms
a boot-time backstop that reverts to netplan if `wlan0` gets no IPv4 address, so a bad cutover
costs a reboot instead of a serial cable; the second run retires it. Leaving it armed means any
later boot where wifi is merely slow reverts the board — `setup-board.sh` warns while it is
still there.

It takes the SSID and key from netplan itself — `/run/netplan/wpa-wlan0.conf` if netplan
generated one, otherwise the `access-points:` stanza in `/etc/netplan/*.yaml` — so an SSH
session over that same wifi survives. If it can read neither it changes **nothing** and prints
the two `nmcli` commands to create the profile by hand; re-running afterwards keeps the profile
it finds.

**2. Install the daemon.**

```bash
curl -fsSL -H "$AUTH" "$RAW/install.sh" -o /tmp/install.sh
```

```bash
sudo DUCK_TOKEN="$DUCK_TOKEN" sh /tmp/install.sh
```

Two commands rather than the obvious `curl … | sudo sh`, and both halves need the token while
the repository is private: the header for the fetch, and the `DUCK_TOKEN` prefix because `sudo`
does not pass the variable through on its own. Piping to `sudo sh` cannot carry either.

Needs `curl` and coreutils and nothing else — `tar` and `zstd` are linked into `updaterd`,
so there is no package to install first. Idempotent, and it never overwrites an existing
`/etc/robot/updater.toml`.

Everything it takes is an environment variable, because it is usually run through
`curl | sh` where flags are awkward to pass:

| | |
|---|---|
| `DUCK_TOKEN` | token for a private repo — the fetch *and* the release assets |
| `DUCK_REPO` | the repository, for a fork or a test repo |
| `DUCK_REF` | the branch config and keys are read from; pin to a tag for a reproducible run |
| `DUCK_DEV_KEY` | path to `team.dev.pub` — makes this a **dev board** (below) |
| `DUCK_FORCE_REINSTALL` | reinstall over a live release using the release's own `updaterd` |

**The first `robotctl health` will say `Permission denied`, and the install is fine** — see
[the group note](#what-ends-up-where) below. `install.sh` prints the one command that fixes it.

### Making it a dev board, so `--ref <branch>` works

A board refuses branch builds twice over: `allow_dev_keys` is false, and a trusted key only
counts as a dev key if its filename ends `.dev.pub`. Both halves are needed, they are
independent checks, and doing one without the other produces a board that still refuses branch
builds — with a signature error that reads like a corrupt release. So pass the key to
`install.sh` and let it do both:

```bash
scp ~/.duck-keys/team.dev.pub pierre@radxa-zero3:/tmp/
```

```bash
sudo DUCK_TOKEN="$DUCK_TOKEN" DUCK_DEV_KEY=/tmp/team.dev.pub sh /tmp/install.sh
```

`team.dev.pub` is deliberately not in the repository —
[`trusted_keys/README.md`](trusted_keys/README.md) explains why. Get the public half from a
team member, or regenerate it from the secret with
`minisign -R -s ~/.duck-keys/team.dev.key -p team.dev.pub`.

It validates before it changes anything: the file must exist, must look like a minisign public
key, and `updater.toml` must already have an `allow_dev_keys` line to flip — that key is
top-level, so appending one would land it inside whichever `[table]` came last. It also
installs the key under the name `team.dev.pub` regardless of what the source file was called,
because the `.dev.` infix is what classifies it; a key installed under any other name is
trusted as a *release* key, and branch builds would then be accepted as reviewed.

The closing report says `DEV BOARD` when this is on, and prints the two commands to undo it.
Never do it to a robot you ship.

### The circularity, and how it is broken

An update needs the updater, and the updater arrives in an update. The way out is one bare
`updaterd` binary — published as the `updaterd-bootstrap-aarch64` release asset, because a
fresh board has no `zstd` binary to open a `.tar.zst` with — run as:

```bash
updaterd install --config /etc/robot/updater.toml
```

That runs the **ordinary engine**: signature verification, extraction, the atomic swap,
the journal entry. So the store and the update log come out in exactly the state the
resident daemon expects on its first start, and there is no bootstrap-only code path that
could drift from how every later update behaves. Two settings are forced for the duration
— `on_apply` and `health` off — because the units live inside the release being installed
and `robotd` cannot be running before its binary exists. `updaterd install` refuses to run
once a release *is* live, so those overrides can never silently disable auto-rollback on a
working robot; use `robotctl update apply` there.

Applying an update never needed a daemon. Mutual exclusion is a file lock in `state_dir`,
not a property of there being one process. What needs a resident daemon is everything
*around* an update — a socket for the app to trigger through, progress to stream back, a
timer so a mandatory release can pull a robot forward with nobody present, and a process
at boot for the boot counter to recover through.

`install` also takes `--from <dir>` to install from local files instead of the network,
which is the offline and factory path, and what CI uses to verify a release before
publishing it.

### ⚠ While the repository is private, a robot needs a token

A private repo's release assets are unreachable without credentials — the
`releases/download/...` URL 404s even with one, so the engine resolves assets through the
release API instead. `updaterd` reads `GITHUB_TOKEN` from its environment, which on a board
means a systemd drop-in, not a shell export.

That is fine on a developer's board and **not** fine on a customer robot: a fleet-wide
credential in an image is one that leaks and cannot be rotated without reflashing, which is
the failure the tiered signing keys exist to avoid.

`install.sh` therefore writes the drop-in **only when `DUCK_TOKEN` was supplied** — mode 600,
and it says so loudly. A customer robot installs from a public artifact repository and passes
no token, so it never reaches that path. Without it `updaterd` would be installed, running,
and unable to fetch a single update, which is most of what it is for.

Artifact hosting is therefore an open decision, not a settled one —
[`../docs/updater-design.md`](../docs/updater-design.md) §6.1 has the options. The cheap one
is a second, public repository holding only signed artifacts: signatures are what make an
artifact safe to serve, not obscurity, and the source stays private.

### The trust chain

1. TLS to `raw.githubusercontent.com` for this script, `updater.toml` and the public keys.
   These cannot come from a release: nothing can be verified until the keys are present.
2. TLS to `github.com` for the bootstrap `updaterd`. **Not yet verified.**
3. That binary verifies the manifest and the artifact against the keys from (1), and
   refuses anything they do not sign.
4. The installer then compares the bootstrap binary's `sha256` against
   `current/bin/updaterd`, which came out of the verified artifact. Equal digests mean the
   binary from (2) was genuine. `release.yml` asserts the two are the same bytes, so a
   mismatch is a real finding rather than a packaging quirk.

Everything else — both unit files, the journald drop-in, `robot.conf` — is taken out of the
*installed* release rather than fetched from the repository, so it is the copy a signature
was checked against.

The residual trust is GitHub itself, which is also where the script came from; step (4)
narrows that window rather than removing it. An install that wants none of it should use
`--from` against files carried in by hand.

### Unattended updates

`updaterd` is already a resident process with a timer — `check_interval` in
`updater.toml` — so there is nothing to add to cron. What the timer is allowed to install
is `auto_apply`:

| | |
|---|---|
| `off` | never; availability is logged, a mandatory release loudly |
| `mandatory` | **the shipped default** — only a release whose `min_supported` floor says the running version must not be used |
| `all` | every available release |

A canary or bench robot that should track `staging` and install each candidate:

```bash
sudo sed -i 's/^auto_apply = .*/auto_apply = "all"/' /etc/robot/updater.toml
```

```bash
sudo sed -i 's/^tag_prefix     = .*/tag_prefix     = "daemon-staging-v"/' /etc/robot/updater.toml
```

```bash
sudo systemctl restart updaterd && journalctl -u updaterd -b | tail -20
```

The first check is 60s after start, then every `check_interval`. `auto_apply = "all"` logs
at `warn` on startup, so "why did this robot restart when nobody asked it to" is answerable
from the journal at any log level.

Don't reach for cron or a systemd timer instead. `robotctl update apply` deliberately
bypasses the known-bad guard — an operator retrying a release may have fixed the cause — so
a timer driving it would inherit the bypass and lose the protection, and one bad release
becomes an endless apply/rollback loop that re-downloads and rewrites the eMMC every
interval. `updater-design.md` §8.1.1 has the detail.

No maintenance window is needed, and that is deliberate. An unattended apply is an ordinary
apply: the preflight asks `robotd` whether it is safe to restart and whether a remote
session is live, *before* any network access, so a robot that is walking or streaming
refuses and retries at the next interval. `safeToRestart` is a better answer to "is now a
bad time" than a clock.

### What ends up where

```
/etc/robot/updater.toml                 config; never touched by an update
/etc/robot/trusted_keys/release-*.pub   trust anchor
/opt/robot/daemon/releases/<version>/   the release tree
/opt/robot/daemon/current -> releases/<version>
/etc/systemd/system/{updaterd,robotd}.service   copied out of the release
/usr/lib/sysusers.d/robot.conf          creates the `robot` group
/var/lib/robot/updater/                 lock, update log, boot counter
/usr/local/bin/robotctl -> current/bin/robotctl
```

Unit files are **copied** rather than symlinked through `current`: read through the
symlink they would change under systemd's feet on every update, and after a rollback
systemd's view would depend on which release happened to be live at the last
`daemon-reload`. `robotctl` *is* a symlink, because it is a tool an operator invokes
rather than a file systemd caches.

Mutating operations are root-only: `allow_uids`/`allow_gids` are deliberately empty in
`updater.toml`. Membership of the `robot` group gets a process as far as *talking* to
`updaterd` — status and logs — and no further. `btd`'s uid joins the allow-list when it
exists, because "may relay an update request from the app" is a narrower claim than "is in
the robot group".

⚠ **The first `robotctl health` after an install usually fails, and the install is fine.**
Both sockets are `root:robot` mode 0660 and `install.sh` puts the operator in `robot` — but a
process's groups are fixed when it starts, so the shell that ran the install is not in the group
it just gained, and both sockets refuse it. One command, in that same shell:

```bash
newgrp robot
```

`id -nG` confirms it, and any new login has it already. This cannot be made fully automatic:
there is no API to add a group to a running process, not even for root, so the shell that
launched the installer can never be fixed from inside it. What `install.sh` does instead is
print that command in its closing report, and `robotctl` names it on the failure rather than
sending you to `systemctl status` — which would show two perfectly healthy daemons.

## Where logs go, and what survives a reboot

Every daemon logs to **stderr**, which systemd captures into the journal. Level is
`RUST_LOG`, set in each unit (`info`).

Two records, with deliberately different durability:

| | where | survives reboot | survives power loss | capped by |
|---|---|---|---|---|
| service logs | journald | only if configured (below) | see the tmpfs caveat | `SystemMaxUse=200M` |
| **update history** | `/var/lib/robot/updater/update-log.jsonl` | **yes** | **yes** | 200 entries |

The update history is not in the journal on purpose. It lives in the engine's `state_dir`
under `/var/lib`, every entry is `fsync`ed as it is appended, and rewrites go through an
atomic temp-file-plus-rename with the parent directory `fsync`ed
(`updater/src/journal.rs`, `updater/src/fsutil.rs`). So "what did this robot install, and
what happened to it" survives even a robot whose journal is volatile, and it is readable
with `robotctl update log` or straight off the disk as JSON lines. That property is
verified by tests, not assumed.

Service logs need the drop-in in this directory. Install it, then:

```bash
sudo systemctl restart systemd-journald
```

Then confirm more than one boot is retained — this is the actual acceptance check, and it
only means anything *after* a real reboot:

```bash
journalctl --list-boots
```

Two or more lines means the previous boot is reachable. One line, or
`no persistent journal was found`, means logs are still RAM-only.

Read a specific service, previous boot:

```bash
journalctl -u robotd -b -1
```

### ⚠ The tmpfs caveat — unverified, needs the board

Armbian images have shipped a RAM-log mechanism (`armbian-ramlog`, similar to `log2ram`)
that mounts `/var/log` as tmpfs to spare the SD card, syncing to disk periodically and on
clean shutdown. If that is active on our Radxa image, `Storage=persistent` gets journald a
directory that is *itself* in RAM: it survives a `reboot` (clean shutdown syncs) but loses
recent logs on a power cut — and a robot is switched off at the wall constantly.

I have not verified this on the target image; there is no hardware yet. Check first:

```bash
findmnt /var/log
```

If it reports `tmpfs`, pick one deliberately:

- **Disable the RAM log** (`systemctl disable --now armbian-ramlog`). Logs become genuinely
  durable; the cost is eMMC write wear. With `info` levels and the caps above the write
  volume is small — the reason those levels were chosen.
- **Keep it** and accept that a power cut loses up to the sync interval. Defensible only if
  the update history — which does not go through `/var/log` — is enough for support.

Do not leave it undecided: the failure mode is silent, and only shows up as "the logs from
the incident are missing" long after the incident.

## Versions, for support

The question is always "what was running?", and on this robot it has **two answers at
once**: `updaterd` never restarts itself during an update (`updater-design.md` §4.1), so
the running binary legitimately lags the installed release until the next reboot. Anything
reporting a single version number is therefore misleading.

`robotctl version` reports both, and flags the disagreement:

```
robotctl   0.1.0  rev unknown

running
  updaterd   0.1.0    rev a1b2c3d
  robotd     0.1.0    rev a1b2c3d

installed
  daemon       0.3.0    rev deadbee

! updaterd is running 0.1.0 but the installed daemon release is 0.3.0.
  Expected right after an update — updaterd never restarts itself, so it keeps
  running the old binary until the next reboot. ...
```

It deliberately works when `updaterd` is **down**, reporting that as a line in the report
rather than exiting — that is when someone is most likely to run it. `--json` gives the
same content for a support bundle.

Four independent places a version is recoverable, so losing one is not fatal:

1. **The startup log line**, first thing each daemon writes, at `warn` so it survives
   `RUST_LOG=warn`: version, git revision, `exe` path, pid. The `exe` path is what tells
   you which release directory the running process actually came from.
2. **`robotctl version`**, over IPC, described above.
3. **`--version`** on every binary, for when nothing is running.
4. **The release itself** — `version.toml` inside each release directory, and
   `robotctl update list` showing each installed release with the revision it was built
   from.

`revision` is compiled in from `DUCK_REVISION` at build time (CI sets it; a laptop build
honestly reports `rev unknown, not a CI build`). It is read at compile time and never from
git at runtime — a shipped robot has no repository.
