//! Head-chain FK in the frames the vision stack actually speaks.
//!
//! The model hands out MJCF site poses; the consumers — camera reprojection,
//! ToF point clouds — want cv2 camera axes (+x right, +y down, +z forward) or
//! the ToF sensor's own axes (+x forward along the optical axis, +y left,
//! +z up). The two constant quaternions here are that translation layer,
//! carried over from the prototype runtime where they were tuned against real
//! images; the sign-pin test at the bottom is what keeps an MJCF update from
//! silently making the robot nod the wrong way.

use crate::{Model, Pose, Quat, SiteId};

/// MJCF `head_camera` site → cv2 camera frame. Constant across robot versions:
/// the per-version site quats differ only by an absolute frame choice, and
/// `q_site⁻¹ * q_cv2` comes out the same.
pub const SITE_TO_CV2: Quat = Quat::new(0.5, -0.5, 0.5, -0.5);

/// Sensor frame (+x forward, +y left, +z up — the VL53L5CX/L8CX integration
/// convention) → cv2 camera frame.
pub const SENSOR_IN_CV2_Q: Quat = Quat::new(0.5, 0.5, -0.5, 0.5);

/// The head joints, in the order [`HeadFk`] takes its angles.
const HEAD_JOINTS: [&str; 4] = ["neck_pitch", "head_pitch", "head_yaw", "head_roll"];

/// The head chain with its names resolved once, so per-frame FK does no string
/// work at all.
pub struct HeadFk {
    model: &'static Model,
    camera: SiteId,
    joints: [usize; 4],
}

impl HeadFk {
    /// Panics if the model lacks the head chain — that is a broken asset, and
    /// the embedded one is covered by tests.
    pub fn new(model: &'static Model) -> Self {
        Self {
            model,
            camera: model
                .site("head_camera")
                .expect("model has a head_camera site"),
            joints: HEAD_JOINTS
                .map(|name| model.joint_index(name).expect("model has the head joints")),
        }
    }

    pub fn alpha() -> Self {
        Self::new(Model::alpha())
    }

    /// Camera pose in the trunk frame, cv2 axes.
    ///
    /// `joints` = `[neck_pitch, head_pitch, head_yaw, head_roll]`, radians.
    /// Every other joint is taken at zero — the head chain hangs from the
    /// trunk, so the legs cannot move it *within the trunk frame*.
    pub fn camera_in_trunk_cv2(&self, joints: [f64; 4]) -> Pose {
        // The alpha model has 14 joints; 32 leaves room for any future duck
        // without ever touching the allocator.
        let mut angles = [0.0f64; 32];
        assert!(self.model.num_joints() <= angles.len());
        for (idx, angle) in self.joints.into_iter().zip(joints) {
            angles[idx] = angle;
        }
        let site = self
            .model
            .site_pose(self.camera, &angles[..self.model.num_joints()]);
        Pose::new(site.pos, site.quat * SITE_TO_CV2)
    }

    /// ToF sensor pose in the trunk frame: rotating a sensor-frame vector
    /// (forward, left, up) by the result's quat expresses it in the trunk.
    pub fn tof_in_trunk(&self, joints: [f64; 4]) -> Pose {
        let cam = self.camera_in_trunk_cv2(joints);
        Pose::new(cam.pos, cam.quat * SENSOR_IN_CV2_Q)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn poses_are_finite_and_unit() {
        let fk = HeadFk::alpha();
        let pose = fk.camera_in_trunk_cv2([0.0; 4]);
        assert!(pose.pos.iter().all(|v| v.is_finite()));
        let n: f64 = pose.quat.wxyz().iter().map(|v| v * v).sum();
        assert!((n - 1.0).abs() < 1e-9, "quaternion not unit: {n}");
    }

    /// Tilting head_pitch must move the camera — guards against a chain that
    /// silently lost a joint name in an MJCF update.
    #[test]
    fn head_pitch_moves_the_camera() {
        let fk = HeadFk::alpha();
        let level = fk.camera_in_trunk_cv2([0.0; 4]).pos;
        let tilted = fk.camera_in_trunk_cv2([0.0, 0.5, 0.0, 0.0]).pos;
        let moved = (level[0] - tilted[0])
            .abs()
            .max((level[2] - tilted[2]).abs());
        assert!(
            moved > 1e-3,
            "head_pitch had no effect: {level:?} vs {tilted:?}"
        );
    }

    /// Pin the alpha head sign conventions (camera-forward response to a
    /// positive joint angle, trunk frame: +y left, +z up). The gaze and
    /// laser-tracking sign flips are chosen against these — if an MJCF update
    /// changes them, this fails instead of the robot nodding the wrong way.
    #[test]
    fn alpha_head_axis_conventions() {
        let fk = HeadFk::alpha();
        let forward = |joints: [f64; 4]| {
            // cv2 +z is the optical axis.
            fk.camera_in_trunk_cv2(joints).quat.rotate([0.0, 0.0, 1.0])
        };
        // +head_yaw looks left (+y) on alpha.
        assert!(forward([0.0, 0.0, 0.3, 0.0])[1] > 0.1);
        // +head_pitch looks DOWN (-z) on alpha.
        assert!(forward([0.0, 0.3, 0.0, 0.0])[2] < -0.1);
    }

    /// The MJCF carries a real `tof` site next to the camera. The convention
    /// path (camera × SENSOR_IN_CV2_Q) — what the prototype shipped — must
    /// agree with the asset's own answer: the *orientation* exactly (both
    /// mounts face the same way), the position within the few centimetres
    /// between the two mounts (the prototype knowingly reused the camera's).
    #[test]
    fn the_convention_tof_pose_agrees_with_the_mjcf_tof_site() {
        let model = Model::alpha();
        let fk = HeadFk::alpha();
        let by_convention = fk.tof_in_trunk([0.1, 0.2, -0.1, 0.05]);

        let mut angles = vec![0.0; model.num_joints()];
        for (name, angle) in [
            ("neck_pitch", 0.1),
            ("head_pitch", 0.2),
            ("head_yaw", -0.1),
            ("head_roll", 0.05),
        ] {
            angles[model.joint_index(name).expect("joint exists")] = angle;
        }
        let site = model.site_pose(model.site("tof").expect("tof site"), &angles);

        for (a, b) in by_convention.pos.iter().zip(site.pos) {
            assert!(
                (a - b).abs() < 0.03,
                "{:?} vs {:?}",
                by_convention.pos,
                site.pos
            );
        }
        // Orientation: the convention path borrows the camera's, which is only
        // valid while the asset mounts the two the same way. If a future MJCF
        // tilts the ToF, this is what should fail.
        let camera = model.site_pose(model.site("head_camera").expect("camera site"), &angles);
        for (a, b) in camera.quat.wxyz().iter().zip(site.quat.wxyz()) {
            assert!(
                (a - b).abs() < 1e-9,
                "the tof site no longer shares the camera's orientation — \
                 stop borrowing it: {:?} vs {:?}",
                camera.quat,
                site.quat
            );
        }
    }
}
