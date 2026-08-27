//! Tests against a real charuco session solved by mrcal
//! (mrcal_calibration.json), whose solution is camera.cameramodel.

use crate::{assert_near, assert_same_direction};
use mrcal::glam::{DAffine3, DVec2, DVec3};
use mrcal::{CameraModel, LensModel, rotation_from_rodrigues};
use serde_json::Value;
use std::path::PathBuf;

pub fn data_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join(name)
}

pub fn affine_from_rt(rt: &[f64]) -> DAffine3 {
    DAffine3::from_mat3_translation(
        rotation_from_rodrigues(DVec3::new(rt[0], rt[1], rt[2])),
        DVec3::new(rt[3], rt[4], rt[5]),
    )
}

pub fn as_f64_vec(v: &Value) -> Vec<f64> {
    v.as_array()
        .unwrap()
        .iter()
        .map(|x| x.as_f64().unwrap())
        .collect()
}

fn full_model() -> CameraModel {
    CameraModel::from_file(data_path("camera.cameramodel")).unwrap()
}

#[test]
fn full_model_file_loads() {
    let model = full_model();
    assert_eq!(model.lensmodel(), LensModel::OpenCv8);
    assert_eq!(model.imagersize(), (2464, 2056));
    assert_eq!(model.cam_from_ref(), DAffine3::IDENTITY);
    assert_eq!(model.intrinsics().len(), 12);
    assert_near(model.intrinsics()[0], 1465.976861, 1e-6, "fx");
    assert_near(model.intrinsics()[3], 1099.039611, 1e-6, "cy");
}

#[test]
fn intrinsics_only_model_matches_full_model() {
    let full = full_model();
    let intrinsics_only =
        CameraModel::from_file(data_path("camera-intrinsics-only.cameramodel")).unwrap();
    assert_eq!(full.lensmodel(), intrinsics_only.lensmodel());
    assert_eq!(full.imagersize(), intrinsics_only.imagersize());
    assert_eq!(full.intrinsics(), intrinsics_only.intrinsics());
}

#[test]
fn real_model_project_unproject_roundtrip() {
    let model = full_model();
    let points = [
        DVec3::new(0.0, 0.0, 1.0),
        DVec3::new(0.3, 0.2, 1.0),
        DVec3::new(-0.5, 0.4, 1.5),
        DVec3::new(0.6, -0.5, 2.0),
        DVec3::new(-0.2, -0.3, 0.5),
    ];
    let q = model.project(&points).unwrap();
    let (w, h) = model.imagersize();
    for qi in &q {
        assert!(qi.x > 0.0 && qi.x < w as f64, "in imager: {qi:?}");
        assert!(qi.y > 0.0 && qi.y < h as f64, "in imager: {qi:?}");
    }
    let v = model.unproject(&q).unwrap();
    for (vi, pi) in v.iter().zip(&points) {
        assert_same_direction(vi.unwrap(), *pi, "real model roundtrip");
    }
}

#[test]
fn center_pixel_unprojects_to_optical_axis() {
    let model = full_model();
    let &[_, _, cx, cy, ..] = model.intrinsics() else {
        panic!("opencv8 has a 4-element core")
    };
    let v = model.unproject(&[DVec2::new(cx, cy)]).unwrap();
    // The distortions are symmetric about the axis, so the principal point
    // looks straight down +z
    assert_same_direction(v[0].unwrap(), DVec3::Z, "principal point");
}

#[test]
fn reprojection_of_calibration_observations_is_subpixel() {
    let json: Value = serde_json::from_str(
        &std::fs::read_to_string(data_path("mrcal_calibration.json")).unwrap(),
    )
    .unwrap();
    let model = full_model();
    let solution = &json["solution"];

    // The saved model is the calibration solution
    let solved = as_f64_vec(&solution["intrinsics"][0]);
    for (a, b) in model.intrinsics().iter().zip(&solved) {
        assert_near(*a, *b, 1e-5, "model file matches solution intrinsics");
    }
    assert_eq!(as_f64_vec(&solution["calobject_warp"]), [0.0, 0.0]);

    let spacing = json["calibration_object"]["calibration_object_spacing"]
        .as_f64()
        .unwrap();
    let frames = solution["rt_ref_frame"].as_array().unwrap();
    let observations = json["observations_board"].as_array().unwrap();
    assert_eq!(frames.len(), observations.len());

    // The camera sits at the reference, and the warp is zero, so corners lie
    // on a flat grid with x varying fastest
    let mut sum_sq = 0.0;
    let mut n = 0usize;
    for (rt, obs) in frames.iter().zip(observations) {
        let ref_from_board = affine_from_rt(&as_f64_vec(rt));
        let mut board_points = Vec::new();
        let mut observed = Vec::new();
        for (iy, row) in obs.as_array().unwrap().iter().enumerate() {
            for (ix, corner) in row.as_array().unwrap().iter().enumerate() {
                let corner = as_f64_vec(corner);
                let weight = corner[2];
                if weight <= 0.0 {
                    continue;
                }
                let p_board = DVec3::new(ix as f64 * spacing, iy as f64 * spacing, 0.0);
                board_points.push(ref_from_board.transform_point3(p_board));
                observed.push(DVec2::new(corner[0], corner[1]));
            }
        }
        let q = model.project(&board_points).unwrap();
        for (qi, oi) in q.iter().zip(&observed) {
            sum_sq += (*qi - *oi).length_squared();
        }
        n += q.len();
    }

    assert_eq!(n, 3109, "inlier corner observations");
    let rms = (sum_sq / n as f64).sqrt();
    assert!(rms < 0.25, "reprojection RMS {rms} px");
}
