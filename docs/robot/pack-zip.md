# Packing a zip and installing it by hand

The package you copy onto a board is a zip. Unzipping it produces the release
tree — `bin/robotd`, `systemd/`, `hooks/`, `models/`, `policies/` — which is
what `/opt/robot/daemon/releases/<version>/` contains. `current` is a symlink
to one of those directories, and the units start
`/opt/robot/daemon/current/bin/robotd`.

A release whose binaries sit next to `systemd/` instead of inside `bin/` fails
at start with `203/EXEC`: the unit's path does not exist.

## Pack

From a clone, after the aarch64 binaries are built
(`target/aarch64-unknown-linux-gnu/release/`):

```bash
sh scripts/pack-zip.sh
```

That writes `dist/daemon-<version>.zip`. The version is
`[workspace.package] version` in `Cargo.toml`.

`--bin-dir` is where the built binaries are. `--out` is where the zip is
written. `--revision` is recorded in `version.toml`; the default is the current
git commit.

The file list is the one in `.github/workflows/_build-release.yml`, plus
`webd`. `webd` has no systemd unit. After install it is
`/opt/robot/daemon/current/bin/webd`, started by hand. Models and policies have
to be in the clone (`duck-detect/models/`, `pet-detect/models/`, `policies/`);
the script stops if one of them is missing.

`bin/`, `hooks/` and `scripts/` are stored as mode `0755`, and each entry is
marked as created on Unix. `unzip` restores those modes only when that mark is
present. Without it every file lands as `0644`, and the unit exits `203/EXEC`
(`Permission denied` while locating the executable) even though `bin/robotd`
is there. Pack the zip with this script. A zip made by dragging the directory
onto an archive tool on Windows drops those bits.

`hooks/` and `scripts/` are stored with Unix newlines. A Windows checkout of
those files is CRLF, and the board's `/bin/sh` (dash) then fails at `set -eu`
with `Illegal option -`.

## Install

Copy the zip and the installer to the board, then:

```bash
sudo sh install-zip.sh daemon-0.13.1.zip
```

From the clone, with the zip still on this machine, the same script is
`scripts/install-zip.sh`. It has to run on the board, as root.

It reads `version.toml` from the zip, stops the daemons, replaces
`/opt/robot/daemon/releases/<version>/`, points `current` at it, and runs
`hooks/postinstall`. That hook installs the units and enables them. `robotd`
is running again when the script returns, on whatever pose the policy holds.

There is no signature check on this path. A board that takes published updates
still does that through `updaterd`.
