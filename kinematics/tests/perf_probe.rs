//! Not a test — a probe. Run with:
//!
//!     cargo test -p kinematics --release --test perf_probe -- --ignored --nocapture
//!
//! Times the odometry-shaped workload (both feet, every tick) and the
//! vision-shaped one (head camera). Numbers land in the terminal, not in an
//! assertion: wall-clock thresholds in CI are flakiness, not coverage.

use std::hint::black_box;
use std::time::Instant;

use kinematics::Model;

#[test]
#[ignore = "perf probe, run manually with --release --nocapture"]
fn time_site_pose() {
    let model = Model::alpha();
    let feet = [
        model.site("left_foot").expect("site"),
        model.site("right_foot").expect("site"),
    ];
    let camera = model.site("head_camera").expect("site");

    const ITERS: u32 = 1_000_000;
    let mut angles = vec![0.0f64; model.num_joints()];

    for (label, sites) in [("both feet", &feet[..]), ("head camera", &[camera][..])] {
        let start = Instant::now();
        let mut acc = 0.0f64;
        for i in 0..ITERS {
            // Vary the input so nothing folds away.
            angles[0] = f64::from(i % 100) * 0.001;
            for &site in sites {
                acc += black_box(model.site_pose(site, black_box(&angles))).pos[2];
            }
        }
        let per = start.elapsed().as_nanos() as f64 / f64::from(ITERS);
        println!("{label}: {per:.0} ns/query-set (acc {acc:.3})");
    }
}
