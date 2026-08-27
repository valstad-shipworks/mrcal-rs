use mrcal::glam::DVec3;

mod calibration;
mod cameramodel;
mod geometry;
mod lensmodel;
mod optimize;
mod pose;
mod projection;
mod triangulation;

pub const OPENCV8_INTRINSICS: [f64; 12] = [
    1761.181055,
    1761.250444,
    1965.706996,
    1087.518797,
    -0.01266096516,
    0.03590794372,
    -0.0002547045941,
    0.0005275929652,
    0.01968883397,
    0.01482863541,
    -0.0562239888,
    0.0500223357,
];

#[track_caller]
pub fn assert_near(a: f64, b: f64, tol: f64, what: &str) {
    assert!((a - b).abs() < tol, "{what}: {a} vs {b}");
}

#[track_caller]
pub fn assert_points_near(a: DVec3, b: DVec3, tol: f64, what: &str) {
    assert_near(a.x, b.x, tol, &format!("{what} (x)"));
    assert_near(a.y, b.y, tol, &format!("{what} (y)"));
    assert_near(a.z, b.z, tol, &format!("{what} (z)"));
}

#[track_caller]
pub fn assert_same_direction(a: DVec3, b: DVec3, what: &str) {
    assert_points_near(a.normalize(), b.normalize(), 1e-6, what);
}
