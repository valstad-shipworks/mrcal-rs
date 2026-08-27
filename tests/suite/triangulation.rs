use crate::assert_points_near;
use mrcal::glam::{DAffine3, DVec3};
use mrcal::{
    rotation_from_rodrigues, triangulate_geometric, triangulate_leecivera_l1,
    triangulate_leecivera_linf, triangulate_leecivera_mid2, triangulate_leecivera_wmid2,
    triangulate_lindstrom,
};

type TriangulateVt = fn(DVec3, DVec3, DVec3) -> Option<DVec3>;

const METHODS: [(&str, TriangulateVt); 5] = [
    ("geometric", triangulate_geometric),
    ("leecivera_l1", triangulate_leecivera_l1),
    ("leecivera_linf", triangulate_leecivera_linf),
    ("leecivera_mid2", triangulate_leecivera_mid2),
    ("leecivera_wmid2", triangulate_leecivera_wmid2),
];

fn cam0_from_cam1(r: DVec3, t: DVec3) -> DAffine3 {
    DAffine3::from_mat3_translation(rotation_from_rodrigues(r), t)
}

#[test]
fn all_methods_recover_the_point() {
    let points = [
        DVec3::new(1.0, 2.0, 10.0),
        DVec3::new(-0.5, 0.3, 4.0),
        DVec3::new(0.0, 0.0, 100.0),
    ];
    let t01 = DVec3::new(0.5, 0.0, 0.0);
    for p in points {
        let v0 = p;
        let v1 = p - t01;
        for (name, f) in METHODS {
            let m = f(v0, v1, t01).unwrap_or_else(|| panic!("{name} failed to triangulate {p:?}"));
            assert_points_near(m, p, 1e-6, name);
        }
    }
}

#[test]
fn observation_scale_does_not_matter() {
    let p = DVec3::new(1.0, 2.0, 10.0);
    let t01 = DVec3::new(0.5, 0.0, 0.0);
    let v0 = p * 7.0;
    let v1 = (p - t01).normalize();
    for (name, f) in METHODS {
        let m = f(v0, v1, t01).unwrap_or_else(|| panic!("{name} failed"));
        assert_points_near(m, p, 1e-6, name);
    }
}

#[test]
fn parallel_rays_do_not_triangulate() {
    let v = DVec3::new(0.1, 0.2, 1.0);
    let t01 = DVec3::new(0.5, 0.0, 0.0);
    // Parallel rays meet at infinity. Whether mrcal's (0,0,0) sentinel catches
    // that or the intersection falls through as a huge point comes down to
    // rounding: leecivera_mid2 returns ~3e17 on gcc/Linux and None on
    // clang/macOS for these same inputs.
    for (name, f) in METHODS {
        if let Some(p) = f(v, v, t01) {
            assert!(
                p.length() > 1e12,
                "{name} triangulated parallel rays to {p:?}"
            );
        }
    }
}

#[test]
fn diverging_rays_do_not_triangulate() {
    // Rays intersect behind the cameras
    let v0 = DVec3::new(-1.0, 0.0, 1.0);
    let v1 = DVec3::new(1.0, 0.0, 1.0);
    let t01 = DVec3::new(0.5, 0.0, 0.0);
    for (name, f) in [
        (
            "leecivera_mid2",
            triangulate_leecivera_mid2 as TriangulateVt,
        ),
        ("leecivera_wmid2", triangulate_leecivera_wmid2),
    ] {
        assert!(f(v0, v1, t01).is_none(), "{name} accepted diverging rays");
    }
}

#[test]
fn lindstrom_recovers_the_point() {
    let p = DVec3::new(1.0, 2.0, 10.0);
    let rt01 = cam0_from_cam1(DVec3::ZERO, DVec3::new(0.5, 0.0, 0.0));
    let v0_local = p;
    let v1_local = rt01.inverse().transform_point3(p);
    let m = triangulate_lindstrom(v0_local, v1_local, rt01).unwrap();
    assert_points_near(m, p, 1e-6, "lindstrom identity rotation");
}

#[test]
fn lindstrom_handles_rotated_second_camera() {
    let p = DVec3::new(0.8, -0.5, 6.0);
    let rt01 = cam0_from_cam1(DVec3::new(0.05, -0.1, 0.2), DVec3::new(0.5, 0.1, 0.0));
    let v0_local = p;
    let v1_local = rt01.inverse().transform_point3(p);
    let m = triangulate_lindstrom(v0_local, v1_local, rt01).unwrap();
    assert_points_near(m, p, 1e-6, "lindstrom rotated camera");
}

#[test]
fn vt_methods_handle_rotated_second_camera() {
    // The common frame is camera 0's, so camera 1's direction rotates back
    // into it
    let p = DVec3::new(0.8, -0.5, 6.0);
    let rt01 = cam0_from_cam1(DVec3::new(0.05, -0.1, 0.2), DVec3::new(0.5, 0.1, 0.0));
    let v1_local = rt01.inverse().transform_point3(p);
    let v1 = rt01.matrix3 * v1_local;
    for (name, f) in METHODS {
        let m = f(p, v1, rt01.translation).unwrap_or_else(|| panic!("{name} failed"));
        assert_points_near(m, p, 1e-6, name);
    }
}
