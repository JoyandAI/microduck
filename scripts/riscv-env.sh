#!/usr/bin/env bash
# Source this to cross-compile for the K1 board (riscv64gc, glibc 2.39).
#
#   source scripts/riscv-env.sh
#   cargo build --release --target riscv64gc-unknown-linux-gnu -p robotd
#
# The toolchain was unpacked into ~/riscv64-toolchain with `dpkg -x` (no root):
#   gcc/binutils .debs provide the linker; libc6-dev/libgcc-s1 provide the sysroot.
TOOLCHAIN="$HOME/riscv64-toolchain"
export LD_LIBRARY_PATH="$TOOLCHAIN/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export CARGO_TARGET_RISCV64GC_UNKNOWN_LINUX_GNU_LINKER="$TOOLCHAIN/usr/bin/riscv64-linux-gnu-gcc"
export CARGO_TARGET_RISCV64GC_UNKNOWN_LINUX_GNU_RUSTFLAGS="-C link-arg=--sysroot=$TOOLCHAIN"
export CC_riscv64gc_unknown_linux_gnu="$TOOLCHAIN/usr/bin/riscv64-linux-gnu-gcc"
export AR_riscv64gc_unknown_linux_gnu="$TOOLCHAIN/usr/bin/riscv64-linux-gnu-ar"
export CFLAGS_riscv64gc_unknown_linux_gnu="--sysroot=$TOOLCHAIN"
echo "riscv64 cross env ready: $("$TOOLCHAIN/usr/bin/riscv64-linux-gnu-gcc" --version | head -1)"
