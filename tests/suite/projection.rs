use crate::{OPENCV8_INTRINSICS, assert_near, assert_same_direction};
use mrcal::glam::{DVec2, DVec3};
use mrcal::{Error, LensModel, project, project_with_gradients, unproject};

const CORE: [f64; 4] = [1000.0, 1000.0, 320.0, 240.0];

fn test_points() -> Vec<DVec3> {
    vec![
        DVec3::new(0.0, 0.0, 1.0),
        DVec3::new(0.1, 0.2, 1.0),
        DVec3::new(-0.3, 0.05, 2.0),
        DVec3::new(0.2, -0.4, 5.0),
    ]
}

fn with_core(distortions: &[f64]) -> Vec<f64> {
    CORE.iter().chain(distortions).copied().collect()
}

#[track_caller]
fn assert_roundtrip(model: LensModel, intrinsics: &[f64]) {
    let points = test_points();
    let q = project(&points, &model, intrinsics).unwrap();
    let v = unproject(&q, &model, intrinsics).unwrap();
    for (vi, pi) in v.iter().zip(&points) {
        assert_same_direction(vi.expect("converged"), *pi, &format!("{model} roundtrip"));
    }
}

#[test]
fn roundtrip_pinhole() {
    assert_roundtrip(LensModel::Pinhole, &CORE);
}

#[test]
fn roundtrip_stereographic() {
    assert_roundtrip(LensModel::Stereographic, &CORE);
}

#[test]
fn roundtrip_lonlat() {
    assert_roundtrip(LensModel::LonLat, &[400.0, 400.0, 1000.0, 500.0]);
}

#[test]
fn roundtrip_latlon() {
    assert_roundtrip(LensModel::LatLon, &[400.0, 400.0, 1000.0, 500.0]);
}

#[test]
fn roundtrip_opencv4() {
    assert_roundtrip(
        LensModel::OpenCv4,
        &with_core(&[-0.01266, 0.03591, -0.000255, 0.000528]),
    );
}

#[test]
fn roundtrip_opencv5() {
    assert_roundtrip(
        LensModel::OpenCv5,
        &with_core(&[-0.01266, 0.03591, -0.000255, 0.000528, 0.01969]),
    );
}

#[test]
fn roundtrip_opencv8() {
    assert_roundtrip(LensModel::OpenCv8, &OPENCV8_INTRINSICS);
}

#[test]
fn roundtrip_opencv12() {
    let mut intrinsics = OPENCV8_INTRINSICS.to_vec();
    intrinsics.extend([0.001, -0.0005, 0.0002, 0.0008]);
    assert_roundtrip(LensModel::OpenCv12, &intrinsics);
}

#[test]
fn roundtrip_cahvor() {
    assert_roundtrip(
        LensModel::Cahvor,
        &with_core(&[0.01, -0.02, 0.001, -0.0005, 0.0002]),
    );
}

#[test]
fn roundtrip_cahvore_central() {
    // mrcal can only unproject a CAHVORE model whose E terms are zero
    assert_roundtrip(
        LensModel::Cahvore { linearity: 0.25 },
        &with_core(&[0.01, -0.02, 0.001, -0.0005, 0.0002, 0.0, 0.0, 0.0]),
    );
}

#[test]
fn noncentral_cahvore_projects_but_does_not_unproject() {
    let model = LensModel::Cahvore { linearity: 0.25 };
    let intrinsics = with_core(&[0.01, -0.02, 0.001, -0.0005, 0.0002, 0.1, 0.05, -0.02]);
    let q = project(&test_points(), &model, &intrinsics).unwrap();
    assert_eq!(
        unproject(&q, &model, &intrinsics).unwrap_err(),
        Error::UnprojectionFailed
    );
}

#[test]
fn optical_axis_projects_to_center_without_distortions() {
    let models: [(LensModel, Vec<f64>); 8] = [
        (LensModel::Pinhole, CORE.to_vec()),
        (LensModel::Stereographic, CORE.to_vec()),
        (LensModel::LonLat, CORE.to_vec()),
        (LensModel::LatLon, CORE.to_vec()),
        (LensModel::OpenCv4, with_core(&[0.0; 4])),
        (LensModel::OpenCv8, with_core(&[0.0; 8])),
        (LensModel::OpenCv12, with_core(&[0.0; 12])),
        (LensModel::Cahvor, with_core(&[0.0; 5])),
    ];
    for (model, intrinsics) in models {
        let q = project(&[DVec3::new(0.0, 0.0, 1.0)], &model, &intrinsics).unwrap();
        assert_near(q[0].x, CORE[2], 1e-9, &format!("{model} center x"));
        assert_near(q[0].y, CORE[3], 1e-9, &format!("{model} center y"));
    }
}

#[test]
fn stereographic_matches_closed_form() {
    let [fx, fy, cx, cy] = CORE;
    let p = DVec3::new(0.3, -0.2, 1.5);
    let q = project(&[p], &LensModel::Stereographic, &CORE).unwrap();
    // u = 2 p_xy / (norm(p) + p_z), q = f u + c
    let scale = 2.0 / (p.length() + p.z);
    assert_near(q[0].x, fx * p.x * scale + cx, 1e-9, "stereographic x");
    assert_near(q[0].y, fy * p.y * scale + cy, 1e-9, "stereographic y");
}

#[test]
fn pinhole_gradients_are_analytic() {
    let [fx, fy, cx, cy] = CORE;
    let p = DVec3::new(0.3, -0.2, 2.0);
    let r = project_with_gradients(&[p], &LensModel::Pinhole, &CORE).unwrap();

    assert_near(r.q[0].x, fx * p.x / p.z + cx, 1e-9, "pinhole qx");
    assert_near(r.q[0].y, fy * p.y / p.z + cy, 1e-9, "pinhole qy");

    let gx = r.dq_dp[0][0];
    let gy = r.dq_dp[0][1];
    assert_near(gx.x, fx / p.z, 1e-9, "dqx/dpx");
    assert_near(gx.y, 0.0, 1e-9, "dqx/dpy");
    assert_near(gx.z, -fx * p.x / (p.z * p.z), 1e-9, "dqx/dpz");
    assert_near(gy.x, 0.0, 1e-9, "dqy/dpx");
    assert_near(gy.y, fy / p.z, 1e-9, "dqy/dpy");
    assert_near(gy.z, -fy * p.y / (p.z * p.z), 1e-9, "dqy/dpz");

    // qx = fx x/z + cx, qy = fy y/z + cy; intrinsics order (fx, fy, cx, cy)
    assert_eq!(r.dq_dintrinsics.len(), 8);
    let expected = [p.x / p.z, 0.0, 1.0, 0.0, 0.0, p.y / p.z, 0.0, 1.0];
    for (i, e) in expected.iter().enumerate() {
        assert_near(
            r.dq_dintrinsics[i],
            *e,
            1e-9,
            &format!("dq_dintrinsics[{i}]"),
        );
    }
}

fn perturbed(p: DVec3, axis: usize, delta: f64) -> DVec3 {
    let mut a = p.to_array();
    a[axis] += delta;
    DVec3::from_array(a)
}

#[test]
fn opencv8_gradients_match_finite_differences() {
    let model = LensModel::OpenCv8;
    let p = DVec3::new(0.15, -0.1, 1.5);
    let r = project_with_gradients(&[p], &model, &OPENCV8_INTRINSICS).unwrap();
    let d = 1e-6;

    for axis in 0..3 {
        let hi = project(&[perturbed(p, axis, d)], &model, &OPENCV8_INTRINSICS).unwrap()[0];
        let lo = project(&[perturbed(p, axis, -d)], &model, &OPENCV8_INTRINSICS).unwrap()[0];
        let gx = r.dq_dp[0][0].to_array();
        let gy = r.dq_dp[0][1].to_array();
        assert_near(
            gx[axis],
            (hi.x - lo.x) / (2.0 * d),
            1e-3,
            &format!("dqx/dp[{axis}]"),
        );
        assert_near(
            gy[axis],
            (hi.y - lo.y) / (2.0 * d),
            1e-3,
            &format!("dqy/dp[{axis}]"),
        );
    }

    for i in 0..12 {
        let mut hi_i = OPENCV8_INTRINSICS;
        let mut lo_i = OPENCV8_INTRINSICS;
        hi_i[i] += d;
        lo_i[i] -= d;
        let hi = project(&[p], &model, &hi_i).unwrap()[0];
        let lo = project(&[p], &model, &lo_i).unwrap()[0];
        let fd_x = (hi.x - lo.x) / (2.0 * d);
        let fd_y = (hi.y - lo.y) / (2.0 * d);
        assert_near(
            r.dq_dintrinsics[i],
            fd_x,
            1e-3,
            &format!("dqx/dintrinsics[{i}]"),
        );
        assert_near(
            r.dq_dintrinsics[12 + i],
            fd_y,
            1e-3,
            &format!("dqy/dintrinsics[{i}]"),
        );
    }
}

#[test]
fn gradient_layout_matches_per_point_projection() {
    let model = LensModel::OpenCv8;
    let points = [DVec3::new(0.1, 0.2, 1.0), DVec3::new(-0.2, 0.1, 3.0)];
    let batch = project_with_gradients(&points, &model, &OPENCV8_INTRINSICS).unwrap();
    assert_eq!(batch.q.len(), 2);
    assert_eq!(batch.dq_dp.len(), 2);
    assert_eq!(batch.dq_dintrinsics.len(), 2 * 2 * 12);

    for (i, p) in points.iter().enumerate() {
        let single = project_with_gradients(&[*p], &model, &OPENCV8_INTRINSICS).unwrap();
        assert_eq!(batch.q[i], single.q[0], "q[{i}]");
        assert_eq!(batch.dq_dp[i], single.dq_dp[0], "dq_dp[{i}]");
        assert_eq!(
            &batch.dq_dintrinsics[i * 24..(i + 1) * 24],
            &single.dq_dintrinsics[..],
            "dq_dintrinsics block {i}"
        );
    }
}

#[test]
fn intrinsics_count_is_checked_everywhere() {
    let p = [DVec3::new(0.0, 0.0, 1.0)];
    let q = [DVec2::new(320.0, 240.0)];
    let bad = [1.0; 3];
    let expected = Error::IntrinsicsCount {
        expected: 4,
        got: 3,
    };
    assert_eq!(
        project(&p, &LensModel::Pinhole, &bad).unwrap_err(),
        expected
    );
    assert_eq!(
        unproject(&q, &LensModel::Pinhole, &bad).unwrap_err(),
        expected
    );
    assert_eq!(
        project_with_gradients(&p, &LensModel::Pinhole, &bad)
            .map(|r| r.q)
            .unwrap_err(),
        expected
    );
}

#[test]
fn empty_inputs_give_empty_outputs() {
    assert_eq!(project(&[], &LensModel::Pinhole, &CORE).unwrap(), vec![]);
    assert_eq!(unproject(&[], &LensModel::Pinhole, &CORE).unwrap(), vec![]);
    let r = project_with_gradients(&[], &LensModel::Pinhole, &CORE).unwrap();
    assert!(r.q.is_empty() && r.dq_dp.is_empty() && r.dq_dintrinsics.is_empty());
}

#[test]
fn unconverged_pixels_are_none_rather_than_nan() {
    // Pixels far outside the model don't converge, and C returns NaN
    let far = [
        DVec2::new(1e6, 1e6),
        DVec2::new(-5e4, 3e4),
        DVec2::new(320.0, 240.0),
    ];
    let v = unproject(&far, &LensModel::OpenCv8, &OPENCV8_INTRINSICS).unwrap();
    assert_eq!(v[0], None, "far pixel");
    assert_eq!(v[1], None, "far pixel");
    let ok = v[2].expect("center pixel converges");
    assert!(ok.is_finite(), "{ok:?}");
    for vi in v.iter().flatten() {
        assert!(vi.is_finite(), "no NaN escapes as Some: {vi:?}");
    }
}

#[test]
fn degenerate_points_project_to_nan() {
    // Projection has no per-point failure signal: the origin comes back NaN
    let q = project(
        &[DVec3::ZERO, DVec3::new(0.0, 0.0, 1.0)],
        &LensModel::Pinhole,
        &CORE,
    )
    .unwrap();
    assert!(q[0].is_nan(), "{:?}", q[0]);
    assert!(q[1].is_finite());
}
