use crate::geometry;
use glam::{DAffine3, DVec3};
use mrcal_sys as sys;
use std::ptr;

// mrcal signals failure (parallel/diverging rays) by returning (0,0,0)
fn nonzero(p: sys::mrcal_point3_t) -> Option<DVec3> {
    let p = geometry::from_sys(p);
    (p != DVec3::ZERO).then_some(p)
}

macro_rules! triangulate_vt {
    ($(#[$doc:meta])* $name:ident, $sys_fn:ident) => {
        $(#[$doc])*
        ///
        /// `v0`/`v1` are observation directions rotated to a common frame,
        /// `t01` camera 1's position in camera 0's frame. `None` if the rays
        /// don't triangulate — though near-parallel rays can instead land on
        /// a point at astronomical range, so check the distance you get.
        pub fn $name(v0: DVec3, v1: DVec3, t01: DVec3) -> Option<DVec3> {
            let (v0, v1, t01) = (
                geometry::to_sys(v0),
                geometry::to_sys(v1),
                geometry::to_sys(t01),
            );
            nonzero(unsafe {
                sys::$sys_fn(
                    ptr::null_mut(),
                    ptr::null_mut(),
                    ptr::null_mut(),
                    &v0,
                    &v1,
                    &t01,
                )
            })
        }
    };
}

triangulate_vt!(
    /// Basic geometric triangulation: midpoint of the common perpendicular.
    triangulate_geometric,
    mrcal_triangulate_geometric
);
triangulate_vt!(
    /// Lee-Civera L1 triangulation.
    triangulate_leecivera_l1,
    mrcal_triangulate_leecivera_l1
);
triangulate_vt!(
    /// Lee-Civera L-infinity triangulation.
    triangulate_leecivera_linf,
    mrcal_triangulate_leecivera_linf
);
triangulate_vt!(
    /// Lee-Civera Mid2 triangulation (the recommended default).
    triangulate_leecivera_mid2,
    mrcal_triangulate_leecivera_mid2
);
triangulate_vt!(
    /// Lee-Civera wMid2 triangulation.
    triangulate_leecivera_wmid2,
    mrcal_triangulate_leecivera_wmid2
);

/// Lindstrom triangulation.
///
/// `v0_local`/`v1_local` are observation directions in each camera's own
/// frame. `None` if the rays don't triangulate — see the note on
/// [`triangulate_leecivera_mid2`] about near-parallel rays.
pub fn triangulate_lindstrom(
    v0_local: DVec3,
    v1_local: DVec3,
    cam0_from_cam1: DAffine3,
) -> Option<DVec3> {
    let (v0, v1) = (geometry::to_sys(v0_local), geometry::to_sys(v1_local));
    // mrcal's (4,3) Rt matrix: row-major rotation, then the translation row
    let mut rt01 = [0.0; 12];
    rt01[..9].copy_from_slice(&cam0_from_cam1.matrix3.transpose().to_cols_array());
    rt01[9..].copy_from_slice(&cam0_from_cam1.translation.to_array());
    nonzero(unsafe {
        sys::mrcal_triangulate_lindstrom(
            ptr::null_mut(),
            ptr::null_mut(),
            ptr::null_mut(),
            &v0,
            &v1,
            rt01.as_ptr().cast(),
        )
    })
}
