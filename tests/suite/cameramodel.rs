use crate::{OPENCV8_INTRINSICS, assert_near, assert_same_direction};
use mrcal::glam::{DAffine3, DVec3};
use mrcal::{CameraModel, Error, LensModel};

const PINHOLE_MODEL: &str = r#"
{
    'lensmodel':  'LENSMODEL_PINHOLE',
    'intrinsics': [ 1000.0, 1100.0, 320.0, 240.0 ],
    'extrinsics': [ 0, 0, 0, 0, 0, 0 ],
    'imagersize': [ 640, 480 ]
}
"#;

#[test]
fn pinhole_model_from_string() {
    let model: CameraModel = PINHOLE_MODEL.parse().unwrap();
    assert_eq!(model.lensmodel(), LensModel::Pinhole);
    assert_eq!(model.intrinsics(), &[1000.0, 1100.0, 320.0, 240.0]);
    assert_eq!(model.imagersize(), (640, 480));
    assert_eq!(model.cam_from_ref(), DAffine3::IDENTITY);

    let q = model.project(&[DVec3::new(0.0, 0.0, 2.0)]).unwrap();
    assert_near(q[0].x, 320.0, 1e-9, "center x");
    assert_near(q[0].y, 240.0, 1e-9, "center y");
}

#[test]
fn full_model_parse_and_file_roundtrip() {
    let text = r#"
{
    'lensmodel':  'LENSMODEL_OPENCV8',
    'intrinsics': [ 1761.181055, 1761.250444, 1965.706996, 1087.518797,
                    -0.01266096516, 0.03590794372, -0.0002547045941, 0.0005275929652,
                    0.01968883397, 0.01482863541, -0.0562239888, 0.0500223357,],
    'extrinsics': [ 2e-2, -3e-1, -1e-2,  1., 2, -3., ],
    'imagersize': [ 4000, 2200 ]
}
"#;
    let model: CameraModel = text.parse().unwrap();
    assert_eq!(model.lensmodel(), LensModel::OpenCv8);
    assert_eq!(model.imagersize(), (4000, 2200));
    assert_eq!(model.intrinsics(), &OPENCV8_INTRINSICS);
    let pose = model.cam_from_ref();
    assert_eq!(pose.translation, DVec3::new(1.0, 2.0, -3.0));
    let r = mrcal::rodrigues_from_rotation(&pose.matrix3);
    crate::assert_points_near(r, DVec3::new(2e-2, -3e-1, -1e-2), 1e-12, "extrinsics r");

    let p = DVec3::new(0.1, 0.2, 1.0);
    let q = model.project(&[p]).unwrap();
    let v = model.unproject(&q).unwrap();
    assert_same_direction(v[0].unwrap(), p, "cameramodel project roundtrip");

    let dir = std::env::temp_dir().join(format!("mrcal-test-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("roundtrip.cameramodel");
    model.write_to_file(&path).unwrap();
    let reloaded = CameraModel::from_file(&path).unwrap();
    assert_eq!(reloaded.lensmodel(), model.lensmodel());
    assert_eq!(reloaded.imagersize(), model.imagersize());
    // The writer round-trips every f64 bit-exactly
    assert_eq!(reloaded.intrinsics(), model.intrinsics());
    assert_eq!(reloaded.cam_from_ref(), model.cam_from_ref());
    assert_eq!(reloaded.to_string(), model.to_string());
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn invalid_text_fails_to_parse() {
    for bad in [
        "",
        "not a camera model",
        "{ 'lensmodel': 'LENSMODEL_PINHOLE' }",
    ] {
        assert_eq!(
            bad.parse::<CameraModel>().map(|m| format!("{m:?}")),
            Err(Error::CameraModelRead),
            "{bad:?} should not parse"
        );
    }
}

#[test]
fn missing_file_fails() {
    let err = CameraModel::from_file("/nonexistent/path/model.cameramodel").unwrap_err();
    assert_eq!(err, Error::CameraModelRead);
}

#[test]
fn nul_in_path_fails() {
    let err = CameraModel::from_file("bad\0path").unwrap_err();
    assert_eq!(err, Error::InvalidString);
}

#[test]
fn write_to_unwritable_path_fails() {
    let model: CameraModel = PINHOLE_MODEL.parse().unwrap();
    let err = model
        .write_to_file("/nonexistent-dir-for-mrcal-test/model.cameramodel")
        .unwrap_err();
    assert!(matches!(err, Error::CameraModelWrite(_)), "{err:?}");
}

#[test]
fn debug_output_shows_fields() {
    let model: CameraModel = PINHOLE_MODEL.parse().unwrap();
    let s = format!("{model:?}");
    assert!(s.contains("Pinhole"), "{s}");
    assert!(s.contains("imagersize"), "{s}");
}

#[test]
fn is_send_and_sync() {
    fn check<T: Send + Sync>() {}
    check::<CameraModel>();
}

#[test]
fn error_display_messages() {
    assert_eq!(
        Error::InvalidLensModelName("X".into()).to_string(),
        "invalid lens model name: \"X\""
    );
    assert_eq!(
        Error::IntrinsicsCount {
            expected: 4,
            got: 2
        }
        .to_string(),
        "expected 4 intrinsics values, got 2"
    );
    assert_eq!(
        Error::ProjectionFailed.to_string(),
        "mrcal_project() failed"
    );
    assert_eq!(
        Error::UnprojectionFailed.to_string(),
        "mrcal_unproject() failed"
    );
    assert_eq!(
        Error::CameraModelRead.to_string(),
        "failed to read camera model"
    );
    assert_eq!(
        Error::CameraModelWrite("no such file".into()).to_string(),
        "failed to write camera model: no such file"
    );
    assert_eq!(
        Error::InvalidLensModelConfig("spline order must be 2 or 3, got 5".into()).to_string(),
        "invalid lens model configuration: spline order must be 2 or 3, got 5"
    );
    assert_eq!(
        Error::TooManyPoints.to_string(),
        "too many points for mrcal's int-sized counts"
    );
    assert_eq!(
        Error::InvalidString.to_string(),
        "string contains an interior NUL byte or is not UTF-8"
    );
    assert_eq!(
        Error::InvalidIndex {
            what: "camera",
            index: 3,
            len: 1
        }
        .to_string(),
        "camera index 3 out of range: 1 added"
    );
    assert_eq!(
        Error::CornerCount {
            expected: 88,
            got: 4
        }
        .to_string(),
        "expected 88 corner observations, got 4"
    );
    assert_eq!(
        Error::InvalidCalibrationObject.to_string(),
        "calibration object needs at least 2x2 corners and positive spacing"
    );
    assert_eq!(
        Error::EmptyProblem.to_string(),
        "nothing to optimize: empty problem or no free variables"
    );
    assert_eq!(
        Error::OptimizationFailed.to_string(),
        "mrcal_optimize() failed"
    );
}

#[test]
fn writer_preserves_full_precision() {
    // %f would round these distortions to three significant digits
    let model: CameraModel = format!(
        "{{
    'lensmodel':  'LENSMODEL_OPENCV8',
    'intrinsics': [ {} ],
    'extrinsics': [ 1e-9, -2.5e-8, 3e-9, 0.1234567890123, -2.0, 3.5 ],
    'imagersize': [ 4000, 2200 ]
}}",
        OPENCV8_INTRINSICS
            .iter()
            .map(|v| format!("{v:?}"))
            .collect::<Vec<_>>()
            .join(", ")
    )
    .parse()
    .unwrap();

    let reparsed: CameraModel = model.to_string().parse().unwrap();
    assert_eq!(reparsed.intrinsics(), model.intrinsics());
    assert_eq!(reparsed.cam_from_ref(), model.cam_from_ref());
    assert_eq!(reparsed.imagersize(), model.imagersize());
    assert_eq!(reparsed.lensmodel(), model.lensmodel());

    // The small rotation survives; %f would have flattened it to zero
    let r = mrcal::rodrigues_from_rotation(&reparsed.cam_from_ref().matrix3);
    assert!(r.x != 0.0 && r.y != 0.0, "{r:?}");
}

#[test]
fn model_can_be_built_from_parts() {
    let model = CameraModel::new(
        LensModel::OpenCv8,
        &OPENCV8_INTRINSICS,
        DAffine3::from_translation(DVec3::new(1.0, 2.0, 3.0)),
        (4000, 2200),
    )
    .unwrap();
    assert_eq!(model.lensmodel(), LensModel::OpenCv8);
    assert_eq!(model.intrinsics(), &OPENCV8_INTRINSICS);
    assert_eq!(model.imagersize(), (4000, 2200));
    assert_eq!(model.cam_from_ref().translation, DVec3::new(1.0, 2.0, 3.0));

    assert_eq!(
        CameraModel::new(
            LensModel::OpenCv8,
            &[1000.0, 1000.0, 320.0, 240.0],
            DAffine3::IDENTITY,
            (640, 480)
        )
        .unwrap_err(),
        Error::IntrinsicsCount {
            expected: 12,
            got: 4
        }
    );
}

#[test]
fn clone_is_independent_and_equal() {
    let model: CameraModel = PINHOLE_MODEL.parse().unwrap();
    let copy = model.clone();
    drop(model);
    assert_eq!(copy.intrinsics(), &[1000.0, 1100.0, 320.0, 240.0]);
    assert_eq!(copy.imagersize(), (640, 480));
}
