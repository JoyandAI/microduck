//! Compile the vendored ULDs into the crate — both generations, plus the probe
//! that decides between them.
//!
//! No autotools, no system library, no Python: `cc` picks the target compiler up
//! from the environment, which is what makes this cross-compile — `cargo board`
//! (cargo-zigbuild) exports `zig cc` for aarch64, the same way it already builds
//! `zstd` for the updater.
//!
//! **Why the six renames.** Each ULD calls its platform hooks by the bare names
//! `RdByte`/`WrByte`/`RdMulti`/`WrMulti`/`SwapBuffer`/`WaitMs`, and we ship one
//! implementation of them (`vendor/platform.c`). Two generations in one binary
//! would therefore define the same six symbols twice. The ULD sources are
//! upstream and unedited, so the rename happens in the preprocessor instead:
//! each generation compiles *its own copy* of `platform.c` with its hooks under
//! a `vl5_`/`vl8_` prefix, and its ULD compiled with the same defines so the call
//! sites follow. `TOF_PLATFORM` names the struct that copy is built against.
//!
//! Warnings are not errors here: `vl53l?cx_api.c` is upstream code we do not
//! edit, and a new compiler finding something in it must not be able to stop a
//! robot release from building.

struct Generation {
    /// Directory under `vendor/`, and the name of the static library.
    dir: &'static str,
    /// Symbol prefix for this generation's platform hooks.
    prefix: &'static str,
    /// The platform struct type its header defines.
    platform: &'static str,
}

const GENERATIONS: [Generation; 2] = [
    Generation {
        dir: "vl53l8cx",
        prefix: "vl8",
        platform: "VL53L8CX_Platform",
    },
    Generation {
        dir: "vl53l5cx",
        prefix: "vl5",
        platform: "VL53L5CX_Platform",
    },
];

/// The hooks ST's sources call, which `vendor/platform.c` implements.
const HOOKS: [&str; 6] = [
    "RdByte",
    "WrByte",
    "RdMulti",
    "WrMulti",
    "SwapBuffer",
    "WaitMs",
];

fn main() {
    let vendor = std::path::Path::new("vendor");

    for generation in &GENERATIONS {
        let dir = vendor.join(generation.dir);
        let mut build = cc::Build::new();
        build
            // The generation's own `platform.h` and `*_buffers.h` come first, so
            // its ULD sees its own header and not the other one's.
            .include(&dir)
            .define("TOF_PLATFORM", generation.platform)
            .file(dir.join(format!("{}_api.c", generation.dir)))
            .file(dir.join("shim.c"))
            // One shared implementation, compiled per generation.
            .file(vendor.join("platform.c"))
            .warnings(false)
            .opt_level(2);
        for hook in HOOKS {
            build.define(hook, format!("{}_{hook}", generation.prefix).as_str());
        }
        build.compile(generation.dir);
    }

    // Generation-agnostic, so no defines and no ULD: it is what decides which of
    // the two above gets to talk to the sensor.
    cc::Build::new()
        .file(vendor.join("probe.c"))
        .warnings(false)
        .opt_level(2)
        .compile("tof_probe");

    println!("cargo::rerun-if-changed=build.rs");
    println!("cargo::rerun-if-changed=vendor/platform.c");
    println!("cargo::rerun-if-changed=vendor/probe.c");
    for generation in &GENERATIONS {
        // The firmware blob is a 550 KB header no source file lists, so it needs
        // naming here or a driver update would not trigger a rebuild.
        for file in ["api.c", "api.h", "buffers.h"] {
            println!(
                "cargo::rerun-if-changed=vendor/{}/{}_{file}",
                generation.dir, generation.dir
            );
        }
        println!(
            "cargo::rerun-if-changed=vendor/{}/platform.h",
            generation.dir
        );
        println!("cargo::rerun-if-changed=vendor/{}/shim.c", generation.dir);
    }
}
