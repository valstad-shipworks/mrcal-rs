use crate::{assert_near, assert_points_near};
use mrcal::glam::{DAffine3, DMat3, DVec3};
use mrcal::{rodrigues_from_rotation, rotation_from_rodrigues};
use std::f64::consts::FRAC_PI_2;

#[test]
fn zero_rodrigues_is_identity() {
    let m = rotation_from_rodrigues(DVec3::ZERO);
    assert_eq!(m, DMat3::IDENTITY);
    assert_eq!(rodrigues_from_rotation(&DMat3::IDENTITY), DVec3::ZERO);
}

#[test]
fn quarter_turn_about_z_has_known_values() {
    let m = rotation_from_rodrigues(DVec3::new(0.0, 0.0, FRAC_PI_2));
    assert_points_near(m * DVec3::X, DVec3::Y, 1e-12, "z quarter turn of x-axis");
    assert_points_near(m * DVec3::Y, -DVec3::X, 1e-12, "z quarter turn of y-axis");
    assert_points_near(m * DVec3::Z, DVec3::Z, 1e-12, "z axis is fixed");
}

#[test]
fn rodrigues_matches_glam_axis_angle() {
    let r = DVec3::new(0.2, -0.3, 0.1);
    let from_mrcal = rotation_from_rodrigues(r);
    let from_glam = DMat3::from_axis_angle(r.normalize(), r.length());
    for c in 0..3 {
        assert_points_near(
            from_mrcal.col(c),
            from_glam.col(c),
            1e-12,
            &format!("column {c}"),
        );
    }
}

#[test]
fn rodrigues_roundtrips_through_rotation_matrix() {
    let r = DVec3::new(0.2, -0.3, 0.1);
    let back = rodrigues_from_rotation(&rotation_from_rodrigues(r));
    assert_points_near(back, r, 1e-12, "rodrigues roundtrip");
}

#[test]
fn rodrigues_rotation_composes_with_daffine3() {
    let a = DAffine3::from_mat3_translation(
        rotation_from_rodrigues(DVec3::new(0.1, 0.2, -0.3)),
        DVec3::new(1.0, 0.0, -2.0),
    );
    let b = DAffine3::from_mat3_translation(
        rotation_from_rodrigues(DVec3::new(-0.2, 0.05, 0.15)),
        DVec3::new(0.5, -1.5, 3.0),
    );
    let p = DVec3::new(0.7, -0.4, 1.9);

    let composed = (a * b).transform_point3(p);
    let nested = a.transform_point3(b.transform_point3(p));
    assert_points_near(composed, nested, 1e-12, "affine composition");

    let back = a.inverse().transform_point3(a.transform_point3(p));
    assert_points_near(back, p, 1e-12, "affine inversion");

    let angle = rodrigues_from_rotation(&a.matrix3).length();
    assert_near(angle, DVec3::new(0.1, 0.2, -0.3).length(), 1e-12, "angle");
}
