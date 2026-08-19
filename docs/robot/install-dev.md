# Installing on a dev board

Getting a board from nothing to a robot you can push branches to.

A dev board trusts the team dev key, so it will install anything anyone on the team builds. A
customer robot is set up differently and deliberately refuses those builds — everything here
assumes a dev board, never a customer robot.

Nothing is relaxed for a dev build: same signature and hash verification, same health gate, same
auto-rollback. The only difference is which key signed it, and that is what keeps these builds
off customer robots — they refuse a dev key twice over. `allow_dev_keys = false`, and a trusted
key only counts as a dev key if its filename ends `.dev.pub`. Both halves of the setup below
exist to flip exactly that.

## Flash the board

Use the [Armbian imager](https://www.armbian.com/radxa-zero-3/). Pick **Radxa Zero 3**, then
**Armbian 26.2.1 Minimal**.

Before writing, fill in the imager's profile — wifi network and password, and the username and
password you want. Doing it there saves a serial console later: the board joins your network on
first boot and is reachable over ssh straight away.

Then add your ssh key, so provisioning can reconnect after it reboots the board:

```bash
ssh-copy-id radxa@192.168.1.42
```

## What you need

- The board's **IP address**. mDNS on this image is unreliable, so a `.local` name resolves when
  it feels like it — use the address from your router's DHCP lease.
- **ssh key access**, from the step above. Provisioning reboots the board and reconnects by
  itself, and a password prompt cannot survive that.
- A **GitHub token**. This repository is private, so its release assets are unreachable without
  one.
- A **clone of this repo**. The dev key it needs is committed at `deploy/dev-key/team.dev.pub`,
  so there is nothing to ask anyone for.

## Install

From a clone on your own machine, two commands:

```bash
export DUCK_TOKEN=github_pat_replace_with_your_token
```

```bash
./scripts/provision-board.sh radxa@192.168.1.42
```

That sends your dev key, starts provisioning, waits out the reboot, streams the log, and ends on
`robotctl health`.

It is a **viewer**, not the thing doing the work — provisioning installs a systemd unit that
resumes at boot, so the board finishes whether or not you are still watching. Ctrl-C costs you
nothing, and you can pick the log back up:

```bash
ssh -t radxa@192.168.1.42 'sudo tail -f /var/lib/robot/provision.log'
```

`--ref BRANCH` provisions from a branch: its scripts run the bring-up, and its build of the daemon
is installed on top. `golden` stays the stable release — it is the boot recovery net's fallback, and
a branch build as golden would give a broken branch a broken fallback — while `current` is the
branch.

Provisioning **fails** if that build cannot be installed, or if it is installed and then rolled back
by the health gate. A dev board quietly running the stable release when a branch was asked for is the
worst failure to debug: everything looks installed and the code under test is not there. Give CI its
minute or two before provisioning, and check with `gh run list --branch BRANCH` if it stops.

Other useful flags: `--name Ducky` names the robot instead of leaving it the `duck-7f3a` it derives
from its own serial (`robotctl system set-name` changes it later, so this only saves a command),
`--local` sends this clone's `provision.sh` instead of fetching it (which is how to test a change to
the provisioning scripts without merging first), and `--no-dev-key` makes a board that only takes
releases.

## Check it worked

```bash
robotctl health
```

The board only counts as a dev board if the key really installed, and that is a thing you can
check rather than a thing you have to remember:

```bash
grep -c 'DEV BOARD' /var/lib/robot/provision.log
```

`1` means yes. `0` means the key did not land, and `--ref` will be refused later with an error
that reads like a corrupt release. That is the failure this check exists to catch early.

Then the real test — put a branch on it:

```bash
sudo robotctl update apply --ref main daemon
```

## When ssh refuses to connect after a reflash

Reflashing regenerates the board's host keys, so the address you used last time now presents a
different one. `StrictHostKeyChecking=accept-new` does not cover it: the host is not new, its key
is. The raw ssh error for this is a wall of text about a possible attack.

```bash
./scripts/provision-board.sh radxa@192.168.1.42 --forget-host-key
```

This matters more than it sounds, because DHCP leases get reused — the address that was one
board last week is a different board today, with a different key.

## When the board comes back at a different address

The wifi cutover in the middle of provisioning can leave the board on a different lease than the
one you gave. `provision-board.sh` goes looking: while it waits for ssh it also asks the robot over
Bluetooth what address it ended up with, and adopts the answer.

```
  bluetooth: the robot reports 192.168.1.57, and 192.168.1.42 was its old lease.
```

Nothing to do — the rest of the run is addressed there.

Three things stop it working, and it says which:

- It needs `cargo` and this clone, because `duck-btctl` is an example rather than an installed binary.
- It can only ask once `btd` is running, which on a board being provisioned for the first time is a
  few minutes into phase 2.
- The robot reports its **wifi** address, so a board you reach over ethernet is not covered.

`--no-ble` turns it off. A robot that has been given its own pairing PIN needs it in `DUCK_PIN`.

## Making an existing board take dev builds

For a board provisioned some other way, or one set up before you had the key. Both halves are
needed: either alone leaves a board that still refuses branch builds.

The easy way is to re-run the installer with the key, which validates it and flips the flag in
one step:

```bash
sudo DUCK_TOKEN="$DUCK_TOKEN" DUCK_DEV_KEY=/tmp/team.dev.pub sh /tmp/install.sh
```

It installs the key as `team.dev.pub` whatever the source file was called. That name matters: the
`.dev.` infix is what classifies a key as a dev key, and a key landing under any other name is
trusted as a **release** key.

By hand, if you would rather see each step:

```bash
sudo cp team.dev.pub /etc/robot/trusted_keys/team.dev.pub
```

```bash
sudo sed -i 's/^allow_dev_keys.*/allow_dev_keys        = true/' /etc/robot/updater.toml
```

```bash
sudo systemctl restart updaterd
```

## The token, by hand

`scripts/install.sh` writes this for you when given `DUCK_TOKEN`. These steps are for a board
provisioned some other way.

`updaterd` reads `GITHUB_TOKEN` from its own environment, so exporting it in your shell does not
reach the daemon — it needs a systemd drop-in.

```bash
sudo mkdir -p /etc/systemd/system/updaterd.service.d
```

Substitute your own token in the next block — it is the only placeholder here:

```bash
sudo tee /etc/systemd/system/updaterd.service.d/token.conf > /dev/null <<'EOF'
[Service]
Environment=GITHUB_TOKEN=ghp_replace_with_your_token
EOF
```

A drop-in is world-readable by default, and this one holds a credential:

```bash
sudo chmod 600 /etc/systemd/system/updaterd.service.d/token.conf
```

```bash
sudo systemctl daemon-reload
```

```bash
sudo systemctl restart updaterd
```

A token on a *developer's* board is fine. A token on a customer robot is not, and is why
artifact hosting is still an open question — see `docs/design/updater-design.md` §6.1.

## Installing without a network

On a board that already has a release, install a local directory the ordinary way — through the
daemon, with the health gate and auto-rollback:

```bash
sudo robotctl update apply daemon --from /media/usb/release
```

That is also what `scripts/dev-push.sh` ends with; [`dev-push.md`](dev-push.md) is the
laptop-to-board path.

The rest of this section is the **bare-board** case: a factory or offline install, before there is
a daemon to ask. It is `updaterd` rather than `robotctl` for that reason, and `updaterd` is
deliberately not on `PATH`:

```bash
sudo /opt/robot/daemon/current/bin/updaterd install --from /media/usb/release
```

The directory holds what a release is: `<version>.manifest.json`, its `.minisig`, the artifact
and the artifact's `.minisig`. Signatures, hashes and compatibility are checked exactly as they
are for a download — `--from` changes where the bytes come from, not what is trusted.

That command refuses to run once a release is live, because it forces `on_apply` and the health
gate off, and doing that to a working robot would silently disable auto-rollback. One situation
needs it anyway, and `robotctl update apply` cannot help with it — a board whose installed
`updaterd` is too old to accept the release that *fixes* being too old. It rolls the new release
back every time, and the binary running that gate is the one being replaced. Stop the robot and
say so explicitly:

```bash
sudo systemctl stop robotd
```

```bash
sudo /opt/robot/daemon/current/bin/updaterd install --from /media/usb/release --force
```

`--force` is itself refused while `robotd` is still answering, since the objection is about a
*working* robot losing its safety net. It gives up auto-rollback for that one install and nothing
else — signatures, hashes and compatibility are still checked, and
`sudo robotctl update rollback daemon` is the recovery path if the release misbehaves.

## Going deeper

[`deploy/README.md`](../../deploy/README.md) is the reference for what all of this actually does:
the trust chain, what ends up where, the other ways in (on the board without a clone, by hand
step by step), where logs go and what survives a reboot.
