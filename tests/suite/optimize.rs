use crate::calibration::{affine_from_rt, as_f64_vec, data_path};
use crate::{assert_near, assert_points_near};
use mrcal::glam::{DAffine3, DVec2, DVec3};
use mrcal::{
    CalibrationObject, CalibrationProblem, CornerObservation, Error, LensModel, OptimizeFlags,
    project, rotation_from_rodrigues,
};
use serde_json::Value;

fn board_corners(object: &CalibrationObject, ref_from_board: DAffine3) -> Vec<DVec3> {
    let mut points = Vec::new();
    for iy in 0..object.height_n {
        for ix in 0..object.width_n {
            let p = DVec3::new(ix as f64 * object.spacing, iy as f64 * object.spacing, 0.0);
            points.push(ref_from_board.transform_point3(p));
        }
    }
    points
}

#[test]
fn synthetic_pinhole_intrinsics_are_recovered_exactly() {
    let object = CalibrationObject::new(5, 4, 0.05);
    let true_intrinsics = [1000.0, 1000.0, 320.0, 240.0];
    let frames = [
        affine_from_rt(&[0.0, 0.0, 0.0, -0.1, -0.1, 0.8]),
        affine_from_rt(&[0.3, 0.0, 0.0, -0.1, 0.0, 1.0]),
        affine_from_rt(&[0.0, -0.3, 0.1, 0.0, -0.1, 1.2]),
        affine_from_rt(&[-0.2, 0.2, 0.0, -0.15, -0.05, 0.9]),
    ];

    let mut problem = CalibrationProblem::new(LensModel::Pinhole, object).unwrap();
    let seed = [950.0, 1050.0, 315.0, 235.0];
    let cam = problem.add_camera(&seed, (640, 480), None).unwrap();
    assert_eq!(cam, 0);

    for ref_from_board in frames {
        let frame = problem.add_frame(ref_from_board);
        let q = project(
            &board_corners(&object, ref_from_board),
            &LensModel::Pinhole,
            &true_intrinsics,
        )
        .unwrap();
        let corners: Vec<_> = q.iter().map(|&px| CornerObservation::new(px)).collect();
        problem.add_observation(cam, frame, &corners).unwrap();
    }

    // Frames fixed at the truth: the intrinsics are exactly observable
    problem.flags = OptimizeFlags::default()
        .with_intrinsics_distortions(false)
        .with_extrinsics(false)
        .with_frames(false)
        .with_regularization(false)
        .with_outlier_rejection(false);
    let optimized = problem.optimize().unwrap();
    let stats = optimized.stats();

    assert!(
        stats.rms_reprojection_error_px < 1e-6,
        "rms: {}",
        stats.rms_reprojection_error_px
    );
    assert_eq!(stats.board_outliers, 0);
    for (solved, truth) in optimized
        .intrinsics(cam)
        .unwrap()
        .iter()
        .zip(&true_intrinsics)
    {
        assert_near(*solved, *truth, 1e-4, "recovered intrinsics");
    }
}

#[test]
fn real_calibration_reproduces_reference_solution() {
    let json: Value = serde_json::from_str(
        &std::fs::read_to_string(data_path("mrcal_calibration.json")).unwrap(),
    )
    .unwrap();

    let object = CalibrationObject::new(
        json["calibration_object"]["corners_x"].as_u64().unwrap() as usize,
        json["calibration_object"]["corners_y"].as_u64().unwrap() as usize,
        json["calibration_object"]["calibration_object_spacing"]
            .as_f64()
            .unwrap(),
    );
    let seed_intrinsics = as_f64_vec(&json["seed"]["intrinsics"][0]);
    let mut problem = CalibrationProblem::new(LensModel::OpenCv8, object).unwrap();
    let cam = problem
        .add_camera(&seed_intrinsics, (2464, 2056), None)
        .unwrap();

    let observations = json["observations_board"].as_array().unwrap();
    for (seed_rt, obs) in json["seed"]["rt_ref_frame"]
        .as_array()
        .unwrap()
        .iter()
        .zip(observations)
    {
        let frame = problem.add_frame(affine_from_rt(&as_f64_vec(seed_rt)));
        let mut corners = Vec::new();
        for row in obs.as_array().unwrap() {
            for corner in row.as_array().unwrap() {
                let c = as_f64_vec(corner);
                corners.push(CornerObservation::weighted(DVec2::new(c[0], c[1]), c[2]));
            }
        }
        problem.add_observation(cam, frame, &corners).unwrap();
    }
    assert_eq!(problem.num_cameras(), 1);
    assert_eq!(problem.num_frames(), 42);
    assert_eq!(problem.num_observations(), 42);

    let optimized = problem.optimize().unwrap();
    let stats = optimized.stats();

    let solution = &json["solution"];
    let expected_rms = solution["rms_reproj_error_pixels"].as_f64().unwrap();
    assert_near(
        stats.rms_reprojection_error_px,
        expected_rms,
        0.01,
        "rms vs reference solve",
    );

    let solved = as_f64_vec(&solution["intrinsics"][0]);
    let intrinsics = optimized.intrinsics(cam).unwrap();
    for i in 0..4 {
        assert_near(
            intrinsics[i],
            solved[i],
            0.5,
            &format!("core intrinsic {i}"),
        );
    }
    for i in 4..12 {
        assert_near(
            intrinsics[i],
            solved[i],
            5e-3,
            &format!("distortion intrinsic {i}"),
        );
    }

    let expected_frame0 = affine_from_rt(&as_f64_vec(&solution["rt_ref_frame"][0]));
    let frame0 = optimized.ref_from_frame(0).unwrap();
    assert_points_near(
        frame0.translation,
        expected_frame0.translation,
        1e-3,
        "frame 0 translation",
    );

    let marked: usize = (0..optimized.num_observations())
        .map(|i| {
            optimized
                .corners(i)
                .unwrap()
                .iter()
                .filter(|c| c.is_outlier())
                .count()
        })
        .sum();
    assert_eq!(marked, stats.board_outliers, "outlier marks in the pool");

    let model = optimized.camera_model(cam).unwrap();
    assert_eq!(model.lensmodel(), LensModel::OpenCv8);
    assert_eq!(model.imagersize(), (2464, 2056));
    assert_eq!(model.cam_from_ref(), DAffine3::IDENTITY);
    assert_eq!(model.intrinsics(), optimized.intrinsics(cam).unwrap());

    // Two residuals per corner, then regularization; the reported RMS is the
    // norm over the whole vector
    let board_measurements = optimized.num_observations() * 88 * 2;
    let residuals = optimized.residuals();
    assert!(
        residuals.len() > board_measurements,
        "regularization terms follow the board residuals"
    );
    let sum_sq: f64 = residuals.iter().map(|x| x * x).sum();
    assert_near(
        (sum_sq / residuals.len() as f64).sqrt(),
        stats.rms_reprojection_error_px,
        1e-9,
        "rms recomputed from residuals",
    );

    // Per-corner reprojection error
    let board_sum_sq: f64 = residuals[..board_measurements].iter().map(|x| x * x).sum();
    let inliers = board_measurements / 2 - stats.board_outliers;
    let per_corner_rms = (board_sum_sq / inliers as f64).sqrt();
    assert!(per_corner_rms < 0.25, "per-corner RMS {per_corner_rms} px");

    // Per-observation residuals line up with the corners
    let res0 = optimized.corner_residuals(0).unwrap();
    assert_eq!(res0.len(), optimized.corners(0).unwrap().len());
    for (r, c) in res0.iter().zip(optimized.corners(0).unwrap()) {
        if c.is_outlier() {
            assert_eq!(*r, DVec2::ZERO, "outlier residual");
        }
        assert!(r.length() < 5.0, "residual {r:?} within a few pixels");
    }
    assert_eq!(
        optimized
            .corner_residuals(optimized.num_observations())
            .unwrap_err(),
        Error::InvalidIndex {
            what: "observation",
            index: optimized.num_observations(),
            len: optimized.num_observations()
        }
    );

    // A re-solve seeded from this solution lands on the same optimum
    let mut problem = optimized.into_problem();
    problem.flags.outlier_rejection = false;
    let reoptimized = problem.optimize().unwrap();
    assert_near(
        reoptimized.stats().rms_reprojection_error_px,
        expected_rms,
        0.01,
        "rms after re-solve",
    );
    for (a, b) in reoptimized.intrinsics(cam).unwrap().iter().zip(&solved) {
        assert_near(*a, *b, 0.5, "re-solved intrinsics");
    }
}

#[test]
fn moving_camera_pose_is_exposed() {
    let object = CalibrationObject::new(3, 3, 0.1);
    let intrinsics = [1000.0, 1000.0, 320.0, 240.0];
    let pose1 = DAffine3::from_mat3_translation(
        rotation_from_rodrigues(DVec3::new(0.0, 0.1, 0.0)),
        DVec3::new(-0.2, 0.0, 0.0),
    );

    // Everything but the frame pose is fixed and the observations are exact,
    // so the poses read back unchanged
    let mut problem = CalibrationProblem::new(LensModel::Pinhole, object).unwrap();
    let cam0 = problem.add_camera(&intrinsics, (640, 480), None).unwrap();
    let cam1 = problem
        .add_camera(&intrinsics, (640, 480), Some(pose1))
        .unwrap();
    problem.flags = OptimizeFlags::NONE.with_frames(true);

    let ref_from_board = DAffine3::from_translation(DVec3::new(-0.1, -0.1, 1.0));
    let frame = problem.add_frame(ref_from_board);
    for cam in [cam0, cam1] {
        let world = board_corners(&object, ref_from_board);
        let local: Vec<_> = world
            .iter()
            .map(|&p| {
                if cam == cam1 {
                    pose1.transform_point3(p)
                } else {
                    p
                }
            })
            .collect();
        let q = project(&local, &LensModel::Pinhole, &intrinsics).unwrap();
        let corners: Vec<_> = q.iter().map(|&px| CornerObservation::new(px)).collect();
        problem.add_observation(cam, frame, &corners).unwrap();
    }

    let optimized = problem.optimize().unwrap();
    assert!(optimized.stats().rms_reprojection_error_px < 1e-9);
    assert_eq!(optimized.cam_from_ref(cam0).unwrap(), DAffine3::IDENTITY);
    let back = optimized.cam_from_ref(cam1).unwrap();
    assert_points_near(back.translation, pose1.translation, 1e-12, "cam1 pose t");
    assert_points_near(
        back.transform_point3(DVec3::Z),
        pose1.transform_point3(DVec3::Z),
        1e-12,
        "cam1 pose r",
    );
}

#[test]
fn validation_errors() {
    assert_eq!(
        CalibrationProblem::new(LensModel::Pinhole, CalibrationObject::new(1, 4, 0.05))
            .err()
            .unwrap(),
        Error::InvalidCalibrationObject
    );
    assert_eq!(
        CalibrationProblem::new(LensModel::Pinhole, CalibrationObject::new(5, 4, 0.0))
            .err()
            .unwrap(),
        Error::InvalidCalibrationObject
    );

    let object = CalibrationObject::new(3, 3, 0.1);

    let empty = CalibrationProblem::new(LensModel::Pinhole, object).unwrap();
    assert_eq!(empty.optimize().unwrap_err(), Error::EmptyProblem);

    // A zero-state problem must be rejected, not passed to the solver
    let mut all_fixed = CalibrationProblem::new(LensModel::Pinhole, object).unwrap();
    let cam = all_fixed
        .add_camera(&[1000.0, 1000.0, 320.0, 240.0], (640, 480), None)
        .unwrap();
    let frame = all_fixed.add_frame(DAffine3::from_translation(DVec3::Z));
    let corners = vec![CornerObservation::new(DVec2::new(320.0, 240.0)); 9];
    all_fixed.add_observation(cam, frame, &corners).unwrap();
    all_fixed.flags = OptimizeFlags::NONE;
    assert_eq!(all_fixed.optimize().unwrap_err(), Error::EmptyProblem);

    let mut problem = CalibrationProblem::new(LensModel::Pinhole, object).unwrap();
    assert_eq!(
        problem.add_camera(&[1000.0], (640, 480), None).unwrap_err(),
        Error::IntrinsicsCount {
            expected: 4,
            got: 1
        }
    );

    let cam = problem
        .add_camera(&[1000.0, 1000.0, 320.0, 240.0], (640, 480), None)
        .unwrap();
    let frame = problem.add_frame(DAffine3::from_translation(DVec3::Z));
    let corners = vec![CornerObservation::new(DVec2::new(320.0, 240.0)); 9];

    assert_eq!(
        problem.add_observation(7, frame, &corners).unwrap_err(),
        Error::InvalidIndex {
            what: "camera",
            index: 7,
            len: 1
        }
    );
    assert_eq!(
        problem.add_observation(cam, 3, &corners).unwrap_err(),
        Error::InvalidIndex {
            what: "frame",
            index: 3,
            len: 1
        }
    );
    assert_eq!(
        problem
            .add_observation(cam, frame, &corners[..4])
            .unwrap_err(),
        Error::CornerCount {
            expected: 9,
            got: 4
        }
    );
}

#[test]
fn corner_observation_helpers() {
    let c = CornerObservation::new(DVec2::new(10.0, 20.0));
    assert_eq!(c.weight, 1.0);
    assert!(!c.is_outlier());
    assert!(CornerObservation::MISSING.is_outlier());
    assert!(CornerObservation::weighted(DVec2::ZERO, -1.0).is_outlier());
}
