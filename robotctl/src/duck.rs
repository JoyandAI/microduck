//! The monitor's live 3D view of the robot.
//!
//! A joints table says *how far* each servo is from its target; it cannot say what the
//! robot looks like. A leg folded the wrong way, a head pitched into the ground and a
//! robot lying on its side are all just numbers there, and every one of them is obvious
//! the moment the pose is drawn. This module draws it: the same visual model the policies
//! were trained against, posed by the measured joint angles on the wire and tilted by the
//! IMU's projected gravity, rendered into terminal cells.
//!
//! The geometry is baked by `scripts/bake-duck-mesh.py` from the app repository's MJCF
//! and compiled in — `robotctl` stays a single binary with no assets directory, and the
//! board never parses CAD. The bake decimates ~330k CAD triangles to the few thousand a
//! terminal can even express; the renderer here is a plain z-buffered rasterizer over
//! them, orthographic, flat-shaded, drawing two pixels per cell with the half-block
//! glyph so pixels come out square. No GPU, no dependency — a frame is a few
//! milliseconds of arithmetic on the board's own CPU.

use ratatui::buffer::Buffer;
use ratatui::layout::Rect;
use ratatui::style::Color;

/// The baked model, produced by `scripts/bake-duck-mesh.py`. Committed rather than
/// built, because building it needs the app repository and numpy, and CI has neither.
static ASSET: &[u8] = include_bytes!("../assets/duck.bin");

/// Bumped in lockstep with the bake script. A blob from the wrong era must refuse to
/// parse rather than draw garbage limbs.
const FORMAT_VERSION: u32 = 1;

/// Camera elevation, radians above the horizon. The app's sim view sits at about this
/// height: enough to read the ground plane, not so much that the robot foreshortens.
const ELEVATION: f32 = 0.32;

/// Where the camera looks and how much world it frames, meters. The robot stands about
/// 0.25 m tall with its trunk near 0.19; framing 0.36 m centred at 0.15 keeps it whole
/// while standing and still whole lying down after a fall, which is exactly the moment
/// somebody is staring at this view.
const LOOK_AT_Z: f32 = 0.15;
const WINDOW: f32 = 0.36;

/// Ground grid pitch and reach, meters — the sim floor's checkerboard, reduced to the
/// lines a terminal can afford.
const GRID_STEP: f32 = 0.05;
const GRID_REACH: f32 = 0.30;

/// One triangle mesh, shared by every part that instances it.
struct Mesh {
    verts: Vec<[f32; 3]>,
    tris: Vec<[u16; 3]>,
}

/// One rigid body of the kinematic tree, parents before children as the bake orders
/// them, so forward kinematics is a single left-to-right pass.
struct Body {
    /// Index into the body list, `-1` for the trunk.
    parent: i16,
    /// Index into `RobotState::joints` (the `JOINT_NAMES` order), `-1` where the body
    /// has no hinge. The trunk's freejoint is not a hinge; the IMU poses it instead.
    joint: i16,
    pos: [f32; 3],
    quat: [f32; 4],
    axis: [f32; 3],
}

/// One placed mesh: a body, a mesh, a fixed offset within the body, and a colour.
struct Part {
    body: u16,
    mesh: u16,
    rgb: [u8; 3],
    pos: [f32; 3],
    quat: [f32; 4],
}

pub struct Model {
    meshes: Vec<Mesh>,
    bodies: Vec<Body>,
    parts: Vec<Part>,
}

/// The compiled-in model. `None` only if the committed blob and this code disagree,
/// which is a build defect — but a monitor that panics over a cosmetic view would be a
/// monitor that cannot show the joints table, so the failure is "no 3D view" instead.
pub fn model() -> Option<&'static Model> {
    static MODEL: std::sync::OnceLock<Option<Model>> = std::sync::OnceLock::new();
    MODEL.get_or_init(|| Model::parse(ASSET)).as_ref()
}

/// A little-endian cursor over the blob. Every read is checked: the blob is trusted
/// (it is compiled in), but "trusted" is not a reason to index out of bounds.
struct Reader<'a> {
    data: &'a [u8],
    at: usize,
}

impl<'a> Reader<'a> {
    fn take(&mut self, n: usize) -> Option<&'a [u8]> {
        let bytes = self.data.get(self.at..self.at + n)?;
        self.at += n;
        Some(bytes)
    }
    fn u16(&mut self) -> Option<u16> {
        Some(u16::from_le_bytes(self.take(2)?.try_into().ok()?))
    }
    fn i16(&mut self) -> Option<i16> {
        Some(i16::from_le_bytes(self.take(2)?.try_into().ok()?))
    }
    fn u32(&mut self) -> Option<u32> {
        Some(u32::from_le_bytes(self.take(4)?.try_into().ok()?))
    }
    fn f32(&mut self) -> Option<f32> {
        Some(f32::from_le_bytes(self.take(4)?.try_into().ok()?))
    }
    fn vec3(&mut self) -> Option<[f32; 3]> {
        Some([self.f32()?, self.f32()?, self.f32()?])
    }
    fn quat(&mut self) -> Option<[f32; 4]> {
        Some([self.f32()?, self.f32()?, self.f32()?, self.f32()?])
    }
}

impl Model {
    fn parse(data: &[u8]) -> Option<Model> {
        let mut r = Reader { data, at: 0 };
        if r.take(4)? != b"DUCK" || r.u32()? != FORMAT_VERSION {
            return None;
        }
        let (n_meshes, n_bodies, n_parts) = (r.u16()?, r.u16()?, r.u16()?);

        let meshes = (0..n_meshes)
            .map(|_| {
                let (n_verts, n_tris) = (r.u16()?, r.u16()?);
                let verts = (0..n_verts).map(|_| r.vec3()).collect::<Option<_>>()?;
                let tris = (0..n_tris)
                    .map(|_| Some([r.u16()?, r.u16()?, r.u16()?]))
                    .collect::<Option<Vec<_>>>()?;
                Some(Mesh { verts, tris })
            })
            .collect::<Option<Vec<_>>>()?;

        let bodies = (0..n_bodies)
            .map(|_| {
                Some(Body {
                    parent: r.i16()?,
                    joint: r.i16()?,
                    pos: r.vec3()?,
                    quat: r.quat()?,
                    axis: r.vec3()?,
                })
            })
            .collect::<Option<Vec<_>>>()?;

        let parts = (0..n_parts)
            .map(|_| {
                let (body, mesh) = (r.u16()?, r.u16()?);
                let rgb = r.take(4)?; // r, g, b, pad
                Some(Part {
                    body,
                    mesh,
                    rgb: [rgb[0], rgb[1], rgb[2]],
                    pos: r.vec3()?,
                    quat: r.quat()?,
                })
            })
            .collect::<Option<Vec<_>>>()?;

        // Sanity that makes the render loop unable to index out of bounds, so it can
        // stay free of per-triangle checks.
        let sound = bodies
            .iter()
            .enumerate()
            .all(|(i, b)| b.parent < i as i16 && b.joint < 15)
            && parts.iter().all(|p| {
                (p.body as usize) < bodies.len()
                    && (p.mesh as usize) < meshes.len()
                    && meshes[p.mesh as usize]
                        .tris
                        .iter()
                        .flatten()
                        .all(|&v| (v as usize) < meshes[p.mesh as usize].verts.len())
            });
        sound.then_some(Model {
            meshes,
            bodies,
            parts,
        })
    }
}

/// A rotation and a translation — all the transform a rigid body needs.
#[derive(Clone, Copy)]
struct Pose {
    r: [[f32; 3]; 3],
    t: [f32; 3],
}

impl Pose {
    const IDENTITY: Pose = Pose {
        r: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        t: [0.0, 0.0, 0.0],
    };

    fn apply(&self, v: [f32; 3]) -> [f32; 3] {
        [
            self.r[0][0] * v[0] + self.r[0][1] * v[1] + self.r[0][2] * v[2] + self.t[0],
            self.r[1][0] * v[0] + self.r[1][1] * v[1] + self.r[1][2] * v[2] + self.t[1],
            self.r[2][0] * v[0] + self.r[2][1] * v[1] + self.r[2][2] * v[2] + self.t[2],
        ]
    }

    fn then(&self, local: &Pose) -> Pose {
        let mut r = [[0.0; 3]; 3];
        for (i, row) in r.iter_mut().enumerate() {
            for (j, cell) in row.iter_mut().enumerate() {
                *cell = self.r[i][0] * local.r[0][j]
                    + self.r[i][1] * local.r[1][j]
                    + self.r[i][2] * local.r[2][j];
            }
        }
        Pose {
            r,
            t: self.apply(local.t),
        }
    }
}

fn quat_pose(q: [f32; 4], t: [f32; 3]) -> Pose {
    let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    let (w, x, y, z) = (q[0] / n, q[1] / n, q[2] / n, q[3] / n);
    Pose {
        r: [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - w * z),
                2.0 * (x * z + w * y),
            ],
            [
                2.0 * (x * y + w * z),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - w * x),
            ],
            [
                2.0 * (x * z - w * y),
                2.0 * (y * z + w * x),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        t,
    }
}

/// Rotation about a unit axis — Rodrigues, spelled out.
fn axis_pose(axis: [f32; 3], angle: f32) -> Pose {
    let (s, c) = angle.sin_cos();
    let (x, y, z) = (axis[0], axis[1], axis[2]);
    let ic = 1.0 - c;
    Pose {
        r: [
            [c + x * x * ic, x * y * ic - z * s, x * z * ic + y * s],
            [y * x * ic + z * s, c + y * y * ic, y * z * ic - x * s],
            [z * x * ic - y * s, z * y * ic + x * s, c + z * z * ic],
        ],
        t: [0.0; 3],
    }
}

fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn normalized(v: [f32; 3]) -> [f32; 3] {
    let n = dot(v, v).sqrt();
    if n < 1e-9 {
        return [0.0, 0.0, 0.0];
    }
    [v[0] / n, v[1] / n, v[2] / n]
}

/// The trunk's attitude, from the IMU's projected gravity: the smallest rotation that
/// carries the measured gravity direction to straight down. Yaw is unobservable from
/// gravity alone, so there is none — the robot faces where the camera azimuth puts it.
///
/// An all-zero gravity (a `robotd` too old to report one) poses the trunk upright,
/// which is what that robot's monitor showed before this view existed.
fn attitude(gravity: [f64; 3]) -> Pose {
    let g = normalized([gravity[0] as f32, gravity[1] as f32, gravity[2] as f32]);
    if dot(g, g) < 0.5 {
        return Pose::IDENTITY;
    }
    let down = [0.0, 0.0, -1.0];
    let axis = cross(g, down);
    let s = dot(axis, axis).sqrt();
    let c = dot(g, down);
    if s < 1e-6 {
        return if c > 0.0 {
            Pose::IDENTITY
        } else {
            // Hanging exactly upside down: any horizontal axis will do.
            axis_pose([1.0, 0.0, 0.0], std::f32::consts::PI)
        };
    }
    axis_pose([axis[0] / s, axis[1] / s, axis[2] / s], s.atan2(c))
}

/// How long a drawn pose may stay on screen while the wire says the robot has moved.
///
/// The monitor repaints per state — 50 a second — and a frame of this view costs a few
/// milliseconds of the board's CPU, which also runs the control loop being watched. A
/// tool that measurably slows what it measures is a bad tool, so motion is re-rendered
/// at ~12 fps and the frames in between blit the cached pixels, which costs nothing.
/// Turning the camera skips the wait: a hand on `[` must feel the view move.
const RASTER_INTERVAL: std::time::Duration = std::time::Duration::from_millis(80);

/// Scratch the renderer reuses frame to frame, plus the one piece of view state — where
/// the camera stands. Owned by the monitor's `View`.
pub struct DuckView {
    /// Camera azimuth, radians. Starts on the three-quarter view the sim opens with.
    azimuth: f32,
    /// What the pixels currently hold: the pose it was rasterized from, the camera it
    /// was seen by, and when. `None` means the pixels are nothing at all.
    cached: Option<(u64, (u32, usize, usize), std::time::Instant)>,
    depth: Vec<f32>,
    pixels: Vec<[u8; 3]>,
    lit: Vec<bool>,
    verts: Vec<[f32; 3]>,
}

impl DuckView {
    pub fn new() -> Self {
        Self {
            azimuth: -0.7,
            cached: None,
            depth: Vec::new(),
            pixels: Vec::new(),
            lit: Vec::new(),
            verts: Vec::new(),
        }
    }

    /// Swing the camera around the robot. Bound to `[` and `]` in the monitor.
    pub fn orbit(&mut self, radians: f32) {
        self.azimuth = (self.azimuth + radians).rem_euclid(std::f32::consts::TAU);
    }

    /// Pose the model and draw it into `area`, two pixels per cell.
    ///
    /// `joints` is the wire's measured-angle array in `JOINT_NAMES` order and `gravity`
    /// the trunk-frame projected gravity; both come straight off `RobotState`. Missing
    /// angles read as zero, so a short array from an older robot draws a default pose
    /// for the joints it does not name rather than nothing.
    pub fn draw(
        &mut self,
        model: &Model,
        joints: &[f64],
        gravity: [f64; 3],
        area: Rect,
        buf: &mut Buffer,
    ) {
        let (w, h) = (area.width as usize, area.height as usize * 2);
        if w < 8 || h < 8 {
            return;
        }

        // Reuse the pixels when they still say the truth — same pose, or a pose newer
        // than [`RASTER_INTERVAL`] — but never across a camera change, which has to
        // land on the very next paint.
        let pose = pose_key(joints, gravity);
        let camera = (self.azimuth.to_bits(), w, h);
        let now = std::time::Instant::now();
        if let Some((cached_pose, cached_camera, at)) = self.cached
            && cached_camera == camera
            && (cached_pose == pose || now.duration_since(at) < RASTER_INTERVAL)
        {
            self.blit(area, buf);
            return;
        }
        self.cached = Some((pose, camera, now));

        self.depth.clear();
        self.depth.resize(w * h, f32::INFINITY);
        self.pixels.clear();
        self.pixels.resize(w * h, [0, 0, 0]);
        self.lit.clear();
        self.lit.resize(w * h, false);

        // Forward kinematics: parents come first, so one pass suffices. The trunk is
        // posed by the IMU alone; its height is fixed up afterwards from the feet.
        let mut poses: Vec<Pose> = Vec::with_capacity(model.bodies.len());
        for body in &model.bodies {
            let parent = match body.parent {
                -1 => attitude(gravity),
                p => poses[p as usize],
            };
            let mut pose = parent.then(&quat_pose(body.quat, body.pos));
            if body.joint >= 0 {
                let angle = joints.get(body.joint as usize).copied().unwrap_or(0.0) as f32;
                pose = pose.then(&axis_pose(normalized(body.axis), angle));
            }
            poses.push(pose);
        }

        // Every vertex into world space once, kept, because it is read three ways:
        // to find the floor, to shade, and to rasterize.
        self.verts.clear();
        let mut ranges = Vec::with_capacity(model.parts.len());
        let mut floor = f32::INFINITY;
        for part in &model.parts {
            let pose = poses[part.body as usize].then(&quat_pose(part.quat, part.pos));
            let start = self.verts.len();
            for &v in &model.meshes[part.mesh as usize].verts {
                let v = pose.apply(v);
                floor = floor.min(v[2]);
                self.verts.push(v);
            }
            ranges.push(start);
        }
        if !floor.is_finite() {
            return;
        }

        // Camera basis from azimuth and elevation: `fwd` looks at the robot, and the
        // projection is orthographic — at 100 pixels, perspective is affectation.
        let (sa, ca) = self.azimuth.sin_cos();
        let (se, ce) = ELEVATION.sin_cos();
        let fwd = [-ce * ca, -ce * sa, -se];
        let right = normalized(cross(fwd, [0.0, 0.0, 1.0]));
        let up = cross(right, fwd);
        let light = normalized([
            0.4 * right[0] + 0.6 * up[0] - 0.7 * fwd[0],
            0.4 * right[1] + 0.6 * up[1] - 0.7 * fwd[1],
            0.4 * right[2] + 0.6 * up[2] - 0.7 * fwd[2],
        ]);

        let scale = (w as f32 / WINDOW).min(h as f32 / WINDOW);
        // The floor shift rides in the projection instead of re-touching every vertex:
        // the robot is drawn with its lowest point on z = 0, wherever the pose put it.
        let project = |v: [f32; 3]| -> [f32; 3] {
            let p = [v[0], v[1], v[2] - floor];
            let x = w as f32 / 2.0 + dot(p, right) * scale;
            let y = h as f32 / 2.0 - (dot(p, up) - dot([0.0, 0.0, LOOK_AT_Z], up)) * scale;
            [x, y, dot(p, fwd)]
        };

        self.grid(w, h, project);

        for (part, &start) in model.parts.iter().zip(&ranges) {
            let mesh = &model.meshes[part.mesh as usize];
            for tri in &mesh.tris {
                let a = self.verts[start + tri[0] as usize];
                let b = self.verts[start + tri[1] as usize];
                let c = self.verts[start + tri[2] as usize];
                let n = normalized(cross(
                    [b[0] - a[0], b[1] - a[1], b[2] - a[2]],
                    [c[0] - a[0], c[1] - a[1], c[2] - a[2]],
                ));
                // Two-sided: decimation does not guarantee winding, and a hole where a
                // triangle flipped is worse than the lighting being symmetric.
                let shade = 0.32 + 0.68 * dot(n, light).abs();
                let rgb = [
                    (part.rgb[0] as f32 * shade) as u8,
                    (part.rgb[1] as f32 * shade) as u8,
                    (part.rgb[2] as f32 * shade) as u8,
                ];
                self.triangle(project(a), project(b), project(c), rgb, w, h);
            }
        }

        self.blit(area, buf);
    }

    /// The sim floor: grid lines on z = 0, sampled as points finer than a pixel and
    /// z-tested like everything else, so the robot stands in front of the far half of
    /// the grid and on top of the near half.
    fn grid(&mut self, w: usize, h: usize, project: impl Fn([f32; 3]) -> [f32; 3]) {
        let lines = (GRID_REACH / GRID_STEP) as i32;
        let mut plot = |v: [f32; 3]| {
            // A disc, not a square: the square's corners read as clutter from every
            // angle that is not axis-aligned, and the sim's floor is boundless anyway.
            if v[0] * v[0] + v[1] * v[1] > GRID_REACH * GRID_REACH {
                return;
            }
            let p = project(v);
            let (x, y) = (p[0] as i32, p[1] as i32);
            if x < 0 || y < 0 || x >= w as i32 || y >= h as i32 {
                return;
            }
            let i = y as usize * w + x as usize;
            if p[2] < self.depth[i] {
                self.depth[i] = p[2];
                self.pixels[i] = [58, 54, 44];
                self.lit[i] = true;
            }
        };
        let mut t = -GRID_REACH;
        while t <= GRID_REACH {
            for k in -lines..=lines {
                let k = k as f32 * GRID_STEP;
                plot([k, t, 0.0]);
                plot([t, k, 0.0]);
            }
            t += 0.004;
        }
    }

    /// One triangle, projected coordinates in, z-buffered pixels out. Edge-function
    /// fill over the bounding box — the classic, with nothing clever, because at this
    /// resolution the bounding boxes are tiny.
    fn triangle(
        &mut self,
        a: [f32; 3],
        b: [f32; 3],
        c: [f32; 3],
        rgb: [u8; 3],
        w: usize,
        h: usize,
    ) {
        let area2 = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
        if area2.abs() < 1e-6 {
            return;
        }
        let x0 = a[0].min(b[0]).min(c[0]).floor().max(0.0) as usize;
        let x1 = a[0].max(b[0]).max(c[0]).ceil().min(w as f32 - 1.0) as usize;
        let y0 = a[1].min(b[1]).min(c[1]).floor().max(0.0) as usize;
        let y1 = a[1].max(b[1]).max(c[1]).ceil().min(h as f32 - 1.0) as usize;
        if x0 > x1 || y0 > y1 {
            return;
        }
        for y in y0..=y1 {
            for x in x0..=x1 {
                let (px, py) = (x as f32 + 0.5, y as f32 + 0.5);
                let w0 = (b[0] - a[0]) * (py - a[1]) - (b[1] - a[1]) * (px - a[0]);
                let w1 = (c[0] - b[0]) * (py - b[1]) - (c[1] - b[1]) * (px - b[0]);
                let w2 = (a[0] - c[0]) * (py - c[1]) - (a[1] - c[1]) * (px - c[0]);
                let inside =
                    (w0 >= 0.0 && w1 >= 0.0 && w2 >= 0.0) || (w0 <= 0.0 && w1 <= 0.0 && w2 <= 0.0);
                if !inside {
                    continue;
                }
                let z = (a[2] * w1 + b[2] * w2 + c[2] * w0) / area2;
                let i = y * w + x;
                if z < self.depth[i] {
                    self.depth[i] = z;
                    self.pixels[i] = rgb;
                    self.lit[i] = true;
                }
            }
        }
    }

    /// Pixels → cells. `▀` is two vertically stacked pixels: foreground paints the top
    /// one, background the bottom. Where only one of the pair is lit the other stays
    /// the terminal's own background, so the robot sits on whatever theme is running.
    fn blit(&self, area: Rect, buf: &mut Buffer) {
        let w = area.width as usize;
        for row in 0..area.height {
            for col in 0..w {
                let top = self.pixel(col, row as usize * 2, w);
                let bottom = self.pixel(col, row as usize * 2 + 1, w);
                let Some(cell) = buf.cell_mut((area.x + col as u16, area.y + row)) else {
                    continue;
                };
                match (top, bottom) {
                    (Some(t), Some(b)) => {
                        cell.set_symbol("▀").set_fg(rgb(t)).set_bg(rgb(b));
                    }
                    (Some(t), None) => {
                        cell.set_symbol("▀").set_fg(rgb(t));
                    }
                    (None, Some(b)) => {
                        cell.set_symbol("▄").set_fg(rgb(b));
                    }
                    (None, None) => {}
                }
            }
        }
    }

    fn pixel(&self, x: usize, y: usize, w: usize) -> Option<[u8; 3]> {
        let i = y * w + x;
        (*self.lit.get(i)?).then(|| self.pixels[i])
    }
}

fn rgb(c: [u8; 3]) -> Color {
    Color::Rgb(c[0], c[1], c[2])
}

/// The pose, reduced to one comparable number. Angles are quantized to ~0.3° first:
/// sensor noise wiggles every measurement, and re-rasterizing over a wiggle no cell
/// can show would defeat the cache that keeps this view cheap.
fn pose_key(joints: &[f64], gravity: [f64; 3]) -> u64 {
    use std::hash::{Hash, Hasher};
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    for &j in joints {
        ((j * 200.0).round() as i64).hash(&mut hasher);
    }
    for g in gravity {
        ((g * 100.0).round() as i64).hash(&mut hasher);
    }
    hasher.finish()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_committed_blob_parses() {
        let model = model().expect("the baked asset must match FORMAT_VERSION");
        assert_eq!(
            model.bodies.len(),
            15,
            "trunk, two legs of five, head of four"
        );
        assert!(model.parts.len() > 30);
    }

    /// Render the default pose and check the robot actually appears — a regression
    /// here (all-empty cells) is what every arithmetic slip in the projection or the
    /// rasterizer collapses into.
    #[test]
    fn a_standing_robot_renders() {
        let model = model().unwrap();
        let mut view = DuckView::new();
        let area = Rect::new(0, 0, 48, 40);
        let mut buf = Buffer::empty(area);
        view.draw(model, &[0.0; 15], [0.0, 0.0, -1.0], area, &mut buf);

        let drawn = (0..40u16)
            .flat_map(|y| (0..48u16).map(move |x| (x, y)))
            .filter(|&(x, y)| buf.cell((x, y)).is_some_and(|c| c.symbol() != " "))
            .count();
        assert!(
            drawn > 200,
            "only {drawn} cells drawn — the robot is missing"
        );
    }

    /// Optional visual check for humans: `DUCK_DUMP=/tmp/duck.ppm cargo test -p robotctl`
    /// writes the rendered pixels as a PPM to look at.
    #[test]
    fn dump_for_eyeballs() {
        let Ok(path) = std::env::var("DUCK_DUMP") else {
            return;
        };
        let model = model().unwrap();
        let mut view = DuckView::new();
        if let Ok(az) = std::env::var("DUCK_AZ") {
            view.azimuth = az.parse().unwrap();
        }
        let mut joints = [0.0f64; 15];
        if let Ok(spec) = std::env::var("DUCK_JOINTS") {
            for (j, v) in joints.iter_mut().zip(spec.split(',')) {
                *j = v.parse().unwrap();
            }
        }
        let area = Rect::new(0, 0, 100, 100);
        let mut buf = Buffer::empty(area);
        view.draw(model, &joints, [0.0, 0.0, -1.0], area, &mut buf);
        let (w, h) = (100usize, 200usize);
        let mut out = format!("P6 {w} {h} 255\n").into_bytes();
        for y in 0..h {
            for x in 0..w {
                out.extend(view.pixel(x, y, w).unwrap_or([15, 15, 15]));
            }
        }
        std::fs::write(path, out).unwrap();
    }
}

#[cfg(test)]
mod bench {
    use super::*;
    use ratatui::{buffer::Buffer, layout::Rect};

    /// Not a benchmark harness, a stopwatch: `cargo test -p robotctl --release
    /// frame_cost -- --nocapture` prints what one frame costs, which is what decides
    /// whether the view can ride the state stream on the board.
    #[test]
    fn frame_cost() {
        let model = model().unwrap();
        let mut view = DuckView::new();
        let area = Rect::new(0, 0, 56, 48);
        let mut buf = Buffer::empty(area);
        let start = std::time::Instant::now();
        let n = 200;
        for i in 0..n {
            view.orbit(0.01 * i as f32);
            view.draw(model, &[0.1; 15], [0.05, 0.02, -1.0], area, &mut buf);
        }
        println!("one frame: {:?}", start.elapsed() / n);
    }
}
