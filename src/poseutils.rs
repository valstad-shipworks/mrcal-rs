use glam::{DAffine3, DMat3, DVec3};
use mrcal_sys as sys;
use std::ptr;

/// Rotation matrix from a Rodrigues vector (mrcal's rotation representation:
/// direction is the rotation axis, magnitude is the angle in radians).
pub fn rotation_from_rodrigues(r: DVec3) -> DMat3 {
    let mut rot = [0.0; 9];
    unsafe {
        sys::mrcal_R_from_r_full(
            rot.as_mut_ptr(),
            0,
            0,
            ptr::null_mut(),
            0,
            0,
            0,
            r.to_array().as_ptr(),
            0,
        );
    }
    // mrcal writes a row-major matrix; glam stores column-major
    DMat3::from_cols_array(&rot).transpose()
}

/// Rodrigues vector from a rotation matrix.
pub fn rodrigues_from_rotation(m: &DMat3) -> DVec3 {
    let row_major = m.transpose().to_cols_array();
    let mut r = [0.0; 3];
    unsafe {
        sys::mrcal_r_from_R_full(
            r.as_mut_ptr(),
            0,
            ptr::null_mut(),
            0,
            0,
            0,
            row_major.as_ptr(),
            0,
            0,
        );
    }
    DVec3::from_array(r)
}

/// mrcal's "rt" transform representation: a Rodrigues rotation vector
/// followed by a translation.
pub(crate) fn affine_from_rt(rt: &[f64; 6]) -> DAffine3 {
    let r = DVec3::new(rt[0], rt[1], rt[2]);
    let t = DVec3::new(rt[3], rt[4], rt[5]);
    DAffine3::from_mat3_translation(rotation_from_rodrigues(r), t)
}

pub(crate) fn rt_from_affine(a: &DAffine3) -> [f64; 6] {
    let r = rodrigues_from_rotation(&a.matrix3);
    let t = a.translation;
    [r.x, r.y, r.z, t.x, t.y, t.z]
}

pub(crate) fn pose_from_affine(a: &DAffine3) -> sys::mrcal_pose_t {
    let rt = rt_from_affine(a);
    sys::mrcal_pose_t {
        r: sys::mrcal_point3_t {
            xyz: [rt[0], rt[1], rt[2]],
        },
        t: sys::mrcal_point3_t {
            xyz: [rt[3], rt[4], rt[5]],
        },
    }
}

pub(crate) fn affine_from_pose(p: &sys::mrcal_pose_t) -> DAffine3 {
    let (r, t) = unsafe { (p.r.xyz, p.t.xyz) };
    affine_from_rt(&[r[0], r[1], r[2], t[0], t[1], t[2]])
}
