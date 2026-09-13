//! Cycle each HD1910 through a move about **0° (mid-count 2048)**, one joint at a time.
//!
//! Limits and torque are per-servo in [`tune_for`]. The same `torque` is the
//! constant-force command (mode 2) and the position-mode torque cap (mode 0).
//!
//! Compile on a development machine (not on the board):
//!
//! ```text
//! cargo run -p duck-control --example hls_control -- --port COM13 --mode speed
//! cargo run -p duck-control --example hls_control -- --port COM13 --mode torque
//! cargo run -p duck-control --example hls_control -- --port COM13 --mode speed --id 10
//! cargo run -p duck-control --example hls_control -- --port COM13 --mode torque --id 31 --cycles 3
//! ```
//!
//! For the Radxa (aarch64), see `docs/robot/bringup-examples.md`.
//!
//! The UART is exclusive: if this talks to the robot's onboard bus, stop `robotd`
//! first. Joints that are not being swept hold **0°**, not the pose they were
//! found in. Ctrl-C (Unix and Windows) or a finished pass limp every servo.

use std::thread;
use std::time::{Duration, Instant};

use duck_control::ftbus::{self, reg};
use duck_control::model::BAUD_RATE;
use duck_control::{JOINT_IDS, JOINT_NAMES, NUM_JOINTS};
use rustypot::servo::feetech::sts3215::Sts3215Controller;

use std::sync::atomic::{AtomicBool, Ordering};

/// A write's status packet can eat the next 30 ms; this demo reads right after
/// reversing, so give the half-duplex bus a bit more than the control loop.
const READ_TIMEOUT: Duration = Duration::from_millis(80);

/// HLSCL mode 0: position servo (speed + torque limit). XL330 analogue: current-based position.
const MODE_SERVO: u8 = 0;
/// HLSCL mode 2: constant-force / electric. XL330 analogue: current control.
const MODE_ELECTRIC: u8 = 2;

/// Per-servo soft limit about 0° and shared torque.
///
/// `limit_deg` is the half-width: motion stays in `[-limit_deg, +limit_deg]`.
/// `torque` is 0.1% LSB (`0..=1000`); 100 = 10%. Used as electric-mode force
/// and as the `WritePosEx` torque cap in speed mode.
#[derive(Clone, Copy)]
struct Tune {
    limit_deg: f64,
    torque: u16,
}

const fn tune(limit_deg: f64, torque: u16) -> Tune {
    Tune { limit_deg, torque }
}

/// Look up by **servo id**, not by array index. `JOINT_IDS` is 20–24, 30–34,
/// 10–14; editing a table in 10–14-first order used to retune the wrong joint
/// (id 12 kept the old 100 because that slot was `head_yaw`).
const fn tune_for(id: u8) -> Tune {
    match id {
        10 => tune(25.0, 100),  // right_hip_yaw
        11 => tune(24.0, 200),  // right_hip_roll
        12 => tune(30.0, 200),  // right_hip_pitch
        13 => tune(30.0, 100),  // right_knee
        14 => tune(30.0, 100),  // right_ankle
        20 => tune(25.0, 100),  // left_hip_yaw
        21 => tune(24.0, 200),  // left_hip_roll
        22 => tune(30.0, 200),  // left_hip_pitch
        23 => tune(30.0, 100),  // left_knee
        24 => tune(30.0, 100),  // left_ankle
        // Neck carries the head. 1000 (100%) in mode 2 has no speed cap: one
        // poll slammed past ±30° and the pull-in chatter shook the joint.
        30 => tune(30.0, 300), // neck_pitch
        31 => tune(30.0, 100), // head_pitch
        32 => tune(30.0, 100),  // head_yaw
        33 => tune(20.0, 100),  // head_roll
        34 => tune(12.0, 80),   // mouth
        _ => tune(0.0, 0),
    }
}

const _: () = {
    let mut i = 0;
    while i < NUM_JOINTS {
        assert!(tune_for(JOINT_IDS[i]).torque > 0);
        i += 1;
    }
};

const ZERO_RAD: f64 = 0.0;

/// Goal speed for HLSCL `WritePosEx`: **steps/s**, 0 = maximum (not the 0.732 RPM
/// present-speed LSB). 800 ≈ 70 °/s. The first draft used 25, which is ~2 °/s.
const DEFAULT_SPEED_LSB: u16 = 800;

/// Accel register. 0 = maximum (what `FeetechIo::apply_startup` writes). Softer for a demo.
const ACCEL: u8 = 20;

/// How close to the goal counts as arrived, in degrees.
const SETTLE_DEG: f64 = 2.0;

/// Give up waiting for a leg of the sweep after this.
const LEG_TIMEOUT: Duration = Duration::from_secs(8);

/// Still enough to believe torque sign. Per-sample deltas are tiny at 40 ms.
const LEARN_DEG: f64 = 1.5;

/// Below this for [`OUTSIDE_STALL`], treat the joint as jammed on a stop.
const STALL_DEG: f64 = 0.6;

/// Sitting still *outside* the band this long means we are on a hard stop.
/// Flip once to pull in; never keep driving into that stop.
const OUTSIDE_STALL: Duration = Duration::from_millis(350);

/// Do not reverse force again this soon. Inertia after a flip looks like the
/// opposite polarity if we keep re-learning `plus_increases` from drift.
const FLIP_HOLD: Duration = Duration::from_millis(200);

/// Poll period while a joint is moving.
const POLL: Duration = Duration::from_millis(40);

/// One triangle per joint: +limit → −limit → 0°. Override with `--cycles`.
const DEFAULT_CYCLES: u32 = 1;

/// Display / exercise order: IDs 10–14, 20–24, 30–34.
const COL_ORDER: [usize; NUM_JOINTS] = [10, 11, 12, 13, 14, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9];

static RUNNING: AtomicBool = AtomicBool::new(true);

#[derive(Clone, Copy, PartialEq, Eq)]
enum Mode {
    Speed,
    Torque,
}

fn main() {
    if let Err(e) = run() {
        eprintln!("{e}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.is_empty() || args.iter().any(|a| a == "-h" || a == "--help") {
        eprint!("{USAGE}");
        std::process::exit(2);
    }
    let Cli {
        port,
        mode,
        only_id,
        cycles,
    } = parse_cli(&args)?;

    install_sigint();

    let serial = serialport::new(&port, BAUD_RATE)
        .timeout(READ_TIMEOUT)
        .open()
        .map_err(|e| {
            format!(
                "open {port}: {e}\n\
                 on the robot bus, stop robotd first (`sudo systemctl stop robotd`)"
            )
        })?;
    let mut controller = Sts3215Controller::new()
        .with_protocol_v1()
        .with_serial_port(serial);

    let mut alive = [false; NUM_JOINTS];
    let mut n_alive = 0usize;
    for (i, &id) in JOINT_IDS.iter().enumerate() {
        match controller.ping(id) {
            Ok(true) => {
                alive[i] = true;
                n_alive += 1;
            }
            Ok(false) => eprintln!("servo {id} ({}) did not answer ping", JOINT_NAMES[i]),
            Err(e) => eprintln!("ping {id} ({}): {e}", JOINT_NAMES[i]),
        }
    }
    if n_alive == 0 {
        return Err("no duck servo answered".into());
    }
    if let Some(id) = only_id {
        if !JOINT_IDS
            .iter()
            .zip(alive.iter())
            .any(|(&j, &ok)| j == id && ok)
        {
            return Err(format!("servo {id} is not on this bus or did not ping").into());
        }
    }

    println!("parking every live servo at 0°");
    park_at_zero(&mut controller, &alive, None)?;

    println!(
        "{label}  {n_alive}/{n} alive  cycles={cycles}  per-joint limits/torque via tune_for(id)  {port}",
        label = match mode {
            Mode::Speed => format!(
                "speed mode  goal_speed={DEFAULT_SPEED_LSB} steps/s (~{:.0} deg/s)  accel={ACCEL}",
                steps_per_s_to_deg_s(DEFAULT_SPEED_LSB)
            ),
            Mode::Torque => "torque mode".to_string(),
        },
        n = NUM_JOINTS,
    );

    let result = drive(&mut controller, &alive, mode, only_id, cycles);
    limp_all(&mut controller, &alive);
    println!("torque off on every live servo");
    result
}

fn drive(
    controller: &mut Sts3215Controller,
    alive: &[bool; NUM_JOINTS],
    mode: Mode,
    only_id: Option<u8>,
    cycles: u32,
) -> Result<(), Box<dyn std::error::Error>> {
    for &i in &COL_ORDER {
        if !RUNNING.load(Ordering::Relaxed) {
            break;
        }
        if !alive[i] {
            continue;
        }
        let id = JOINT_IDS[i];
        if only_id.is_some_and(|want| want != id) {
            continue;
        }
        park_at_zero(controller, alive, Some(id))?;
        let name = JOINT_NAMES[i];
        let start = read_pos_rad(controller, id)?;
        let tune = tune_for(id);
        println!(
            "\n-- {name} id={id}  pos={:.1} deg  band=[-{lim:.0},{lim:.0}]  torque={tq} ({pct:.0}%) --",
            start.to_degrees(),
            lim = tune.limit_deg,
            tq = tune.torque,
            pct = tune.torque as f64 / 10.0,
        );
        match mode {
            Mode::Speed => sweep_speed(controller, id, tune, cycles)?,
            Mode::Torque => sweep_torque(controller, id, tune, cycles)?,
        }
        if RUNNING.load(Ordering::Relaxed) {
            hold_pose(controller, id, ZERO_RAD, tune)?;
        }
    }
    Ok(())
}

/// Position-hold every live joint at 0°, except `skip` (the one about to sweep).
fn park_at_zero(
    controller: &mut Sts3215Controller,
    alive: &[bool; NUM_JOINTS],
    skip: Option<u8>,
) -> Result<(), Box<dyn std::error::Error>> {
    for (i, &ok) in alive.iter().enumerate() {
        if !ok {
            continue;
        }
        let id = JOINT_IDS[i];
        if skip == Some(id) {
            continue;
        }
        hold_pose(controller, id, ZERO_RAD, tune_for(id))?;
    }
    if !RUNNING.load(Ordering::Relaxed) {
        return Ok(());
    }
    let deadline = Instant::now() + LEG_TIMEOUT;
    loop {
        if !RUNNING.load(Ordering::Relaxed) {
            return Ok(());
        }
        let mut farthest: Option<(u8, f64)> = None;
        for (i, &ok) in alive.iter().enumerate() {
            if !ok {
                continue;
            }
            let id = JOINT_IDS[i];
            if skip == Some(id) {
                continue;
            }
            let pos = read_pos_rad(controller, id)?;
            if (pos - ZERO_RAD).abs() > SETTLE_DEG.to_radians() {
                match farthest {
                    Some((_, d)) if pos.abs() <= d => {}
                    _ => farthest = Some((id, pos.abs())),
                }
            }
        }
        match farthest {
            None => return Ok(()),
            Some((id, _)) if Instant::now() >= deadline => {
                eprintln!(
                    "id {id} did not reach 0° in {}s; continuing",
                    LEG_TIMEOUT.as_secs()
                );
                return Ok(());
            }
            _ => thread::sleep(POLL),
        }
    }
}

fn sweep_speed(
    controller: &mut Sts3215Controller,
    id: u8,
    tune: Tune,
    cycles: u32,
) -> Result<(), Box<dyn std::error::Error>> {
    set_mode(controller, id, MODE_SERVO)?;
    write_u16(controller, id, reg::TORQUE_LIMIT, tune.torque)?;
    write_u8(controller, id, reg::TORQUE_ENABLE, 1)?;
    thread::sleep(Duration::from_millis(20));
    dump_cmd(controller, id, "speed-armed")?;

    let plus = tune.limit_deg.to_radians();
    let minus = -tune.limit_deg.to_radians();
    let plus_l = format!("+{:.0}", tune.limit_deg);
    let minus_l = format!("-{:.0}", tune.limit_deg);
    for _ in 0..cycles {
        go_to(controller, id, plus, &plus_l, tune)?;
        go_to(controller, id, minus, &minus_l, tune)?;
        go_to(controller, id, ZERO_RAD, "0", tune)?;
    }
    Ok(())
}

fn sweep_torque(
    controller: &mut Sts3215Controller,
    id: u8,
    tune: Tune,
    cycles: u32,
) -> Result<(), Box<dyn std::error::Error>> {
    set_mode(controller, id, MODE_ELECTRIC)?;
    write_u16(controller, id, reg::TORQUE_LIMIT, tune.torque)?;
    write_u8(controller, id, reg::TORQUE_ENABLE, 1)?;
    thread::sleep(Duration::from_millis(20));
    dump_cmd(controller, id, "torque-armed")?;

    let hi = tune.limit_deg.to_radians();
    let lo = -tune.limit_deg.to_radians();
    let force = tune.torque as i16;

    // Do not assume +torque increases position. Learn the sign from motion.
    // Already outside (or jammed on a hard stop): only pull toward the band,
    // never keep driving into that stop. Spawning past +limit is not a reverse.
    let mut now = read_pos_rad(controller, id)?;
    let mut dir: i16 = 1;
    write_force(controller, id, dir, force)?;

    let mut last_hit_hi: Option<bool> = if now > hi {
        Some(true)
    } else if now < lo {
        Some(false)
    } else {
        None
    };
    if last_hit_hi.is_some() {
        println!(
            "  start outside  pos={:.1} deg  band=[{:.1},{:.1}]  will pull in, not push the stop",
            now.to_degrees(),
            lo.to_degrees(),
            hi.to_degrees()
        );
    }

    let reversals_needed = cycles.saturating_mul(2);
    let mut reversals = 0u32;
    let mut last_flip = Instant::now();
    let mut plus_increases: Option<bool> = None;
    let mut mark_pos = now;
    let mut mark_at = Instant::now();
    let mut outside_probes = 0u32;

    thread::sleep(POLL);

    while reversals < reversals_needed && RUNNING.load(Ordering::Relaxed) {
        now = read_pos_rad(controller, id)?;
        let drift = now - mark_pos;
        if plus_increases.is_none() && drift.abs() >= LEARN_DEG.to_radians() {
            // Polarity is a property of the actuator. Re-learning it from
            // coasting-after-flip drift inverted the map every poll and
            // bang-banged ±force at the soft stop (the shake on 30/31).
            plus_increases = Some((dir > 0) == (drift > 0.0));
            mark_pos = now;
            mark_at = Instant::now();
        } else if drift.abs() >= STALL_DEG.to_radians() {
            mark_pos = now;
            mark_at = Instant::now();
        }
        let stalled = mark_at.elapsed() >= OUTSIDE_STALL;

        let over = now > hi;
        let under = now < lo;
        let new_edge = if over && last_hit_hi != Some(true) {
            Some(true)
        } else if under && last_hit_hi != Some(false) {
            Some(false)
        } else {
            None
        };

        if over || under {
            let want_up = under;
            let mut announced = false;
            if let Some(pinc) = plus_increases {
                let need = dir_toward(want_up, pinc);
                if need != dir && last_flip.elapsed() >= FLIP_HOLD {
                    dir = need;
                    write_force(controller, id, dir, force)?;
                    last_flip = Instant::now();
                    mark_pos = now;
                    mark_at = Instant::now();
                    println!(
                        "  pull-in  pos={:.1} deg  band=[{:.1},{:.1}]  torque={}  (was into stop / outward)",
                        now.to_degrees(),
                        lo.to_degrees(),
                        hi.to_degrees(),
                        dir * force
                    );
                    announced = true;
                }
            } else if stalled {
                if outside_probes >= 1 {
                    stop_force(controller, id);
                    return Err(format!(
                        "id {id}: jammed at {:.1} deg (band ±{:.0}); not driving further into the stop",
                        now.to_degrees(),
                        tune.limit_deg
                    )
                    .into());
                }
                dir = -dir;
                outside_probes += 1;
                write_force(controller, id, dir, force)?;
                last_flip = Instant::now();
                mark_pos = now;
                mark_at = Instant::now();
                println!(
                    "  stall-flip  pos={:.1} deg  band=[{:.1},{:.1}]  torque={}  (no motion while outside)",
                    now.to_degrees(),
                    lo.to_degrees(),
                    hi.to_degrees(),
                    dir * force
                );
                announced = true;
            }

            if let Some(hit_hi) = new_edge {
                last_hit_hi = Some(hit_hi);
                if plus_increases.is_none() {
                    dir = -dir;
                    write_force(controller, id, dir, force)?;
                    last_flip = Instant::now();
                    mark_pos = now;
                    mark_at = Instant::now();
                }
                reversals += 1;
                println!(
                    "  reverse #{reversals}  pos={:.1} deg  {}  torque={}",
                    now.to_degrees(),
                    if hit_hi {
                        format!(">+{:.0}", tune.limit_deg)
                    } else {
                        format!("<-{:.0}", tune.limit_deg)
                    },
                    dir * force
                );
            } else if plus_increases.is_some() && last_flip.elapsed() >= LEG_TIMEOUT {
                stop_force(controller, id);
                return Err(format!(
                    "id {id}: still outside ±{:.0}° after {}s (pos {:.1}); inward torque too small",
                    tune.limit_deg,
                    LEG_TIMEOUT.as_secs(),
                    now.to_degrees()
                )
                .into());
            } else if !announced {
                println!(
                    "  torque  pos={:.1} deg  band=[{:.1},{:.1}]  torque={}  (outside, inward)",
                    now.to_degrees(),
                    lo.to_degrees(),
                    hi.to_degrees(),
                    dir * force
                );
            }
        } else {
            outside_probes = 0;
            if last_flip.elapsed() >= LEG_TIMEOUT {
                stop_force(controller, id);
                return Err(format!(
                    "id {id}: still inside ±{:.0}° after {}s (pos {:.1}); jammed or torque too small",
                    tune.limit_deg,
                    LEG_TIMEOUT.as_secs(),
                    now.to_degrees()
                )
                .into());
            }
            println!(
                "  torque  pos={:.1} deg  band=[{:.1},{:.1}]  torque={}",
                now.to_degrees(),
                lo.to_degrees(),
                hi.to_degrees(),
                dir * force
            );
        }
        thread::sleep(POLL);
    }

    // Cut force immediately. Chasing 0° still in mode 2 is what left the
    // ankle driven into a stop when the next read timed out.
    stop_force(controller, id);
    if !RUNNING.load(Ordering::Relaxed) {
        return Ok(());
    }
    set_mode(controller, id, MODE_SERVO)?;
    write_u16(controller, id, reg::TORQUE_LIMIT, tune.torque)?;
    write_u8(controller, id, reg::TORQUE_ENABLE, 1)?;
    write_pos_ex(
        controller,
        id,
        ftbus::rad_to_position_count(ZERO_RAD),
        DEFAULT_SPEED_LSB,
        ACCEL,
        tune.torque,
    )?;
    wait_near(controller, id, ZERO_RAD, "0")
}

/// Torque sign that moves position up (`want_up`) given whether +torque increases angle.
fn dir_toward(want_up: bool, plus_increases: bool) -> i16 {
    match (want_up, plus_increases) {
        (true, true) | (false, false) => 1,
        (true, false) | (false, true) => -1,
    }
}

fn write_force(
    controller: &mut Sts3215Controller,
    id: u8,
    dir: i16,
    force: i16,
) -> Result<(), Box<dyn std::error::Error>> {
    write_u16(controller, id, reg::GOAL_CURRENT, sign_mag(dir * force))
}

fn go_to(
    controller: &mut Sts3215Controller,
    id: u8,
    target: f64,
    label: &str,
    tune: Tune,
) -> Result<(), Box<dyn std::error::Error>> {
    if !RUNNING.load(Ordering::Relaxed) {
        return Ok(());
    }
    write_pos_ex(
        controller,
        id,
        ftbus::rad_to_position_count(target),
        DEFAULT_SPEED_LSB,
        ACCEL,
        tune.torque,
    )?;
    wait_near(controller, id, target, label)
}

fn wait_near(
    controller: &mut Sts3215Controller,
    id: u8,
    target: f64,
    label: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    let deadline = Instant::now() + LEG_TIMEOUT;
    loop {
        if !RUNNING.load(Ordering::Relaxed) {
            return Ok(());
        }
        let now = read_pos_rad(controller, id)?;
        if (now - target).abs() <= SETTLE_DEG.to_radians() {
            println!(
                "  {label:>5}  pos={:.1} deg  goal={:.1}",
                now.to_degrees(),
                target.to_degrees()
            );
            return Ok(());
        }
        if Instant::now() >= deadline {
            return Err(format!(
                "id {id} {label}: timeout at {:.1} deg (goal {:.1})",
                now.to_degrees(),
                target.to_degrees()
            )
            .into());
        }
        println!(
            "  {label:>5}  pos={:.1} deg  goal={:.1}",
            now.to_degrees(),
            target.to_degrees()
        );
        thread::sleep(POLL);
    }
}

fn hold_pose(
    controller: &mut Sts3215Controller,
    id: u8,
    rad: f64,
    tune: Tune,
) -> Result<(), Box<dyn std::error::Error>> {
    set_mode(controller, id, MODE_SERVO)?;
    write_u16(controller, id, reg::TORQUE_LIMIT, tune.torque)?;
    write_u8(controller, id, reg::TORQUE_ENABLE, 1)?;
    write_pos_ex(
        controller,
        id,
        ftbus::rad_to_position_count(rad),
        DEFAULT_SPEED_LSB,
        ACCEL,
        tune.torque,
    )?;
    Ok(())
}

/// HLSCL `WritePosEx`: one 7-byte write at ACC (41) =
/// `acc | pos_l pos_h | torque_l torque_h | speed_l speed_h`.
/// Split writes leave goal-torque (44) at 0 and do not arm a move the way the
/// firmware expects.
fn write_pos_ex(
    controller: &mut Sts3215Controller,
    id: u8,
    pos: u16,
    speed: u16,
    acc: u8,
    torque: u16,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut buf = Vec::with_capacity(7);
    buf.push(acc);
    buf.extend_from_slice(&pos.to_le_bytes());
    buf.extend_from_slice(&torque.to_le_bytes());
    buf.extend_from_slice(&speed.to_le_bytes());
    controller
        .write_raw_data(id, reg::ACCEL, buf)
        .map_err(|e| format!("WritePosEx on {id}: {e}").into())
}

/// Mode 33 lives in the EEPROM window. A locked EPROM drops the write, so
/// torque mode stays at 0 and `GOAL_CURRENT` does nothing.
fn set_mode(
    controller: &mut Sts3215Controller,
    id: u8,
    mode: u8,
) -> Result<(), Box<dyn std::error::Error>> {
    write_u8(controller, id, reg::TORQUE_ENABLE, 0)?;
    write_u8(controller, id, reg::LOCK, 0)?;
    write_u8(controller, id, reg::MODE, mode)?;
    write_u8(controller, id, reg::LOCK, 1)?;
    let got = read_u8(controller, id, reg::MODE)?;
    if got != mode {
        return Err(format!("id {id}: mode write {mode} read back {got}").into());
    }
    Ok(())
}

fn dump_cmd(
    controller: &mut Sts3215Controller,
    id: u8,
    tag: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    let mode = read_u8(controller, id, reg::MODE)?;
    let tq = read_u8(controller, id, reg::TORQUE_ENABLE)?;
    let pos = read_u16(controller, id, reg::GOAL_POSITION)?;
    let torque = read_u16(controller, id, reg::GOAL_CURRENT)?;
    let limit = read_u16(controller, id, reg::TORQUE_LIMIT)?;
    let speed = read_u16(controller, id, reg::GOAL_SPEED)?;
    println!(
        "  [{tag}] mode={mode} torque_en={tq} goal_pos={pos} goal_torque={torque} torque_limit={limit} goal_speed={speed}"
    );
    Ok(())
}

fn stop_force(controller: &mut Sts3215Controller, id: u8) {
    let _ = write_u16(controller, id, reg::GOAL_CURRENT, 0);
    let _ = write_u8(controller, id, reg::TORQUE_ENABLE, 0);
    thread::sleep(Duration::from_millis(20));
}

fn limp_all(controller: &mut Sts3215Controller, alive: &[bool; NUM_JOINTS]) {
    for (i, &ok) in alive.iter().enumerate() {
        if ok {
            stop_force(controller, JOINT_IDS[i]);
        }
    }
}

fn read_pos_rad(
    controller: &mut Sts3215Controller,
    id: u8,
) -> Result<f64, Box<dyn std::error::Error>> {
    let mut last = "no attempt".to_string();
    for _ in 0..4 {
        match controller.read_raw_data(id, reg::PRESENT_POSITION, 2) {
            Ok(block) if block.len() >= 2 => {
                let count = u16::from_le_bytes([block[0], block[1]]);
                return ftbus::position_to_rad(count)
                    .ok_or_else(|| format!("id {id}: multi-turn position {count}").into());
            }
            Ok(block) => last = format!("short ({} B)", block.len()),
            Err(e) => last = e.to_string(),
        }
        thread::sleep(Duration::from_millis(15));
    }
    Err(format!("read pos {id}: {last}").into())
}

fn write_u8(
    controller: &mut Sts3215Controller,
    id: u8,
    addr: u8,
    value: u8,
) -> Result<(), Box<dyn std::error::Error>> {
    controller
        .write_raw_data(id, addr, vec![value])
        .map_err(|e| format!("write {addr} on {id}: {e}").into())
}

fn write_u16(
    controller: &mut Sts3215Controller,
    id: u8,
    addr: u8,
    value: u16,
) -> Result<(), Box<dyn std::error::Error>> {
    controller
        .write_raw_data(id, addr, value.to_le_bytes().to_vec())
        .map_err(|e| format!("write {addr} on {id}: {e}").into())
}

fn read_u8(
    controller: &mut Sts3215Controller,
    id: u8,
    addr: u8,
) -> Result<u8, Box<dyn std::error::Error>> {
    let b = controller
        .read_raw_data(id, addr, 1)
        .map_err(|e| format!("read {addr} on {id}: {e}"))?;
    b.first()
        .copied()
        .ok_or_else(|| format!("read {addr} on {id}: empty").into())
}

fn read_u16(
    controller: &mut Sts3215Controller,
    id: u8,
    addr: u8,
) -> Result<u16, Box<dyn std::error::Error>> {
    let b = controller
        .read_raw_data(id, addr, 2)
        .map_err(|e| format!("read {addr} on {id}: {e}"))?;
    if b.len() < 2 {
        return Err(format!("read {addr} on {id}: short").into());
    }
    Ok(u16::from_le_bytes([b[0], b[1]]))
}

fn sign_mag(v: i16) -> u16 {
    let mag = v.unsigned_abs();
    if v < 0 { mag | 0x8000 } else { mag }
}

fn steps_per_s_to_deg_s(lsb: u16) -> f64 {
    lsb as f64 * 360.0 / 4096.0
}

#[cfg(unix)]
fn install_sigint() {
    // SAFETY: the handler only stores to an AtomicBool.
    unsafe {
        libc::signal(
            libc::SIGINT,
            handle_sigint as *const () as libc::sighandler_t,
        );
        libc::signal(
            libc::SIGTERM,
            handle_sigint as *const () as libc::sighandler_t,
        );
    }
}

#[cfg(unix)]
extern "C" fn handle_sigint(_: libc::c_int) {
    RUNNING.store(false, Ordering::Relaxed);
}

#[cfg(windows)]
fn install_sigint() {
    // SAFETY: the handler only stores to an AtomicBool. Returning 1 tells
    // Windows we handled Ctrl-C so the process can limp the bus instead of
    // dying with torque still on (STATUS_CONTROL_C_EXIT).
    unsafe {
        unsafe extern "system" {
            fn SetConsoleCtrlHandler(
                handler: Option<unsafe extern "system" fn(u32) -> i32>,
                add: i32,
            ) -> i32;
        }
        unsafe extern "system" fn handler(_: u32) -> i32 {
            RUNNING.store(false, Ordering::Relaxed);
            1
        }
        SetConsoleCtrlHandler(Some(handler), 1);
    }
}

struct Cli {
    port: String,
    mode: Mode,
    only_id: Option<u8>,
    cycles: u32,
}

fn parse_cli(args: &[String]) -> Result<Cli, Box<dyn std::error::Error>> {
    let mut port = None;
    let mut mode = None;
    let mut only_id = None;
    let mut cycles = DEFAULT_CYCLES;
    let mut i = 0;
    while i < args.len() {
        let a = args[i].as_str();
        if let Some(v) = take_opt(args, &mut i, a, &["--port", "-p"])? {
            port = Some(v);
        } else if let Some(v) = take_opt(args, &mut i, a, &["--mode", "-m"])? {
            mode = Some(parse_mode(&v)?);
        } else if let Some(v) = take_opt(args, &mut i, a, &["--id", "-i"])? {
            only_id = Some(parse_joint_id(&v)?);
        } else if let Some(v) = take_opt(args, &mut i, a, &["--cycles", "-c"])? {
            cycles = parse_cycles(&v)?;
        } else {
            return Err(format!(
                "unexpected {a:?}; use --port, --mode, --id, --cycles (see --help)"
            )
            .into());
        }
    }
    Ok(Cli {
        port: port.ok_or("--port is required")?,
        mode: mode.ok_or("--mode is required (speed or torque)")?,
        only_id,
        cycles,
    })
}

/// `--foo bar`, `--foo=bar`, and the short form. Advances `i` past the value.
fn take_opt(
    args: &[String],
    i: &mut usize,
    a: &str,
    names: &[&str],
) -> Result<Option<String>, Box<dyn std::error::Error>> {
    for name in names {
        if a == *name {
            *i += 1;
            let v = args
                .get(*i)
                .ok_or_else(|| format!("{name} needs a value"))?;
            *i += 1;
            return Ok(Some(v.clone()));
        }
        let prefix = format!("{name}=");
        if let Some(v) = a.strip_prefix(&prefix) {
            if v.is_empty() {
                return Err(format!("{name} needs a value").into());
            }
            *i += 1;
            return Ok(Some(v.to_string()));
        }
    }
    Ok(None)
}

fn parse_mode(s: &str) -> Result<Mode, Box<dyn std::error::Error>> {
    match s {
        "speed" | "pos" | "servo" => Ok(Mode::Speed),
        "torque" | "ele" | "force" => Ok(Mode::Torque),
        other => Err(format!("unknown --mode {other:?}; use speed or torque").into()),
    }
}

fn parse_joint_id(s: &str) -> Result<u8, Box<dyn std::error::Error>> {
    let id: u8 = s.parse().map_err(|_| format!("not a servo id: {s}"))?;
    if !JOINT_IDS.contains(&id) {
        return Err(format!("servo {id} is not a duck joint id").into());
    }
    Ok(id)
}

fn parse_cycles(s: &str) -> Result<u32, Box<dyn std::error::Error>> {
    let n: u32 = s.parse().map_err(|_| format!("not a cycle count: {s}"))?;
    if n == 0 {
        return Err("--cycles must be >= 1".into());
    }
    if n > 100 {
        return Err(format!("--cycles {s} is too large (max 100)").into());
    }
    Ok(n)
}

const USAGE: &str = "\
hls_control — sweep each HD1910 about 0°; limits and torque are per joint

Usage:
  cargo run -p duck-control --example hls_control -- --port <port> --mode <speed|torque> [--id <id>] [--cycles <n>]

--cycles defaults to 1 (one +limit → −limit → 0° triangle). Omit --id to sweep
every live servo.

Edit tune_for() in this file by servo id (not array index): limit_deg is
± about 0°, torque is 0.1% LSB (100 = 10%). Same value for constant force,
WritePosEx, and TORQUE_LIMIT.

Windows:
  cargo run -p duck-control --example hls_control -- --port COM13 --mode speed
  cargo run -p duck-control --example hls_control -- --port COM13 --mode torque --id 14
  cargo run -p duck-control --example hls_control -- --port COM13 --mode torque --id 31 --cycles 3

Linux / the robot:
  sudo systemctl stop robotd
  ./hls_control --port /dev/ttyS2 --mode speed
";
