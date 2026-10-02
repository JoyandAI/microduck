# Bench examples — cross-compile and copy to the board

`imu_probe`, `hls_read`, `imu_servo_read` and `hls_control` are `duck-control`
examples. They talk to the IMU or the servo UART **without** going through `robotd`.
They are not a daemon release: do not use [`dev-push.md`](dev-push.md) /
`scripts/dev-push.sh` for them.

There is no `hls_write`. Sweeping / writing HD1910s is `hls_control`.

Official `robotd` builds still use `cargo zigbuild` ([`onboarding.md`](../onboarding.md)
§5). The path below is what this tree actually uses on a **Windows host + WSL** when
zig is not installed: `aarch64-linux-gnu-gcc` as the linker.

## One-time (WSL)

Ubuntu in WSL, Rust via rustup, repo on the Windows drive (here `E:\dev\microduck`
→ `/mnt/e/dev/microduck`):

```bash
sudo apt-get update
sudo apt-get install -y gcc-aarch64-linux-gnu
rustup target add aarch64-unknown-linux-gnu
```

Confirm:

```bash
which cargo rustc aarch64-linux-gnu-gcc
# /root/.cargo/bin/cargo   (or $HOME/.cargo/bin)
# /usr/bin/aarch64-linux-gnu-gcc
```

## Build

From WSL, in the clone:

```bash
export PATH="$HOME/.cargo/bin:/usr/bin:/bin"
export CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER=/usr/bin/aarch64-linux-gnu-gcc
cd /mnt/e/dev/microduck

cargo build --target aarch64-unknown-linux-gnu -p duck-control \
  --example imu_probe --example hls_read --example imu_servo_read --example hls_control
```

One example only:

```bash
cargo build --target aarch64-unknown-linux-gnu -p duck-control --example hls_control
```

`--release` is optional (much smaller). Debug lands at:

```text
target/aarch64-unknown-linux-gnu/debug/examples/imu_probe
target/aarch64-unknown-linux-gnu/debug/examples/hls_read
target/aarch64-unknown-linux-gnu/debug/examples/imu_servo_read
target/aarch64-unknown-linux-gnu/debug/examples/hls_control
```

`file` should say `ELF 64-bit LSB pie executable, ARM aarch64`. These are Linux
binaries, not Windows `.exe`.

### If you have zig instead

Same artifacts, repo-default toolchain:

```bash
cargo zigbuild --target aarch64-unknown-linux-gnu.2.31 -p duck-control \
  --example imu_probe --example hls_read --example imu_servo_read --example hls_control
```

## Copy onto the Radxa

Board account used here: `radxa@<host>` (hostname `radxa-zero3` on the bench).
UART `/dev/ttyS2` and I²C `/dev/i2c-imu` (fallback `/dev/i2c-5`) are exclusive
with `robotd` — stop it first.

From the **Windows** clone (PowerShell / Git Bash) or from WSL; paths are the same
relative to the repo:

```bash
BOARD=radxa@radxa-zero3
EX=target/aarch64-unknown-linux-gnu/debug/examples

scp $EX/imu_probe $EX/hls_read $EX/imu_servo_read $EX/hls_control $BOARD:~/
ssh $BOARD 'sudo systemctl stop robotd; chmod +x ~/imu_probe ~/hls_read ~/imu_servo_read ~/hls_control'
```

On the board:

```bash
# I²C IMU. Alias first; some boards answer on i2c-5.
./imu_probe /dev/i2c-imu 0x6A
# ./imu_probe /dev/i2c-5 0x6A
# ./imu_probe --hz 50

# imu_to_servo on the Feetech bus (ID 200, five pings, model 0x4D44).
./imu_probe /dev/ttyS2 --hz 50

# HD1910 bus — joint table, plus the bus IMU row when ID 200 answers.
./hls_read /dev/ttyS2

# Every device on the bus: servo lines and the IMU line (accel included).
./imu_servo_read /dev/ttyS2

# HD1910 bus — sweep / write. Joints not in the sweep hold 0°, not the pose they were in.
./hls_control --port /dev/ttyS2 --mode speed
./hls_control --port /dev/ttyS2 --mode torque
```

Start the daemon again when finished:

```bash
sudo systemctl start robotd
```

Native `cargo run -p duck-control --example …` on Windows is for a USB-serial
dongle (`COM13`, …), not for these aarch64 files.
