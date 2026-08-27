use mrcal::glam::{DVec2, DVec3};
use mrcal::{
    CalibrationObject, CalibrationProblem, Error, LensModel, project, project_with_gradients,
    unproject,
};

fn all_models() -> Vec<LensModel> {
    vec![
        LensModel::Pinhole,
        LensModel::Stereographic,
        LensModel::LonLat,
        LensModel::LatLon,
        LensModel::OpenCv4,
        LensModel::OpenCv5,
        LensModel::OpenCv8,
        LensModel::OpenCv12,
        LensModel::Cahvor,
        LensModel::Cahvore { linearity: 0.25 },
        LensModel::SplinedStereographic {
            order: 3,
            nx: 30,
            ny: 20,
            fov_x_deg: 170,
        },
    ]
}

#[test]
fn num_params_for_all_models() {
    assert_eq!(LensModel::Pinhole.num_params(), 4);
    assert_eq!(LensModel::Stereographic.num_params(), 4);
    assert_eq!(LensModel::LonLat.num_params(), 4);
    assert_eq!(LensModel::LatLon.num_params(), 4);
    assert_eq!(LensModel::OpenCv4.num_params(), 8);
    assert_eq!(LensModel::OpenCv5.num_params(), 9);
    assert_eq!(LensModel::OpenCv8.num_params(), 12);
    assert_eq!(LensModel::OpenCv12.num_params(), 16);
    assert_eq!(LensModel::Cahvor.num_params(), 9);
    assert_eq!(LensModel::Cahvore { linearity: 0.25 }.num_params(), 12);
    assert_eq!(
        LensModel::SplinedStereographic {
            order: 3,
            nx: 30,
            ny: 20,
            fov_x_deg: 170,
        }
        .num_params(),
        4 + 2 * 30 * 20
    );
}

#[test]
fn known_names() {
    assert_eq!(LensModel::Pinhole.name(), "LENSMODEL_PINHOLE");
    assert_eq!(LensModel::Stereographic.name(), "LENSMODEL_STEREOGRAPHIC");
    assert_eq!(LensModel::LonLat.name(), "LENSMODEL_LONLAT");
    assert_eq!(LensModel::LatLon.name(), "LENSMODEL_LATLON");
    assert_eq!(LensModel::OpenCv4.name(), "LENSMODEL_OPENCV4");
    assert_eq!(LensModel::OpenCv12.name(), "LENSMODEL_OPENCV12");
    assert_eq!(LensModel::Cahvor.name(), "LENSMODEL_CAHVOR");
}

#[test]
fn name_parse_roundtrip_for_all_models() {
    for model in all_models() {
        let name = model.name();
        let parsed: LensModel = name.parse().unwrap_or_else(|e| panic!("{name}: {e}"));
        assert_eq!(parsed, model, "{name}");
        assert_eq!(model.to_string(), name, "Display matches name()");
    }
}

#[test]
fn splined_config_survives_name_roundtrip() {
    let name = "LENSMODEL_SPLINED_STEREOGRAPHIC_order=3_Nx=11_Ny=8_fov_x_deg=200";
    let parsed: LensModel = name.parse().unwrap();
    assert_eq!(
        parsed,
        LensModel::SplinedStereographic {
            order: 3,
            nx: 11,
            ny: 8,
            fov_x_deg: 200,
        }
    );
    assert_eq!(parsed.name(), name);
}

#[test]
fn invalid_names_are_rejected() {
    for bad in ["", "LENSMODEL_BOGUS", "PINHOLE", "lensmodel_pinhole"] {
        assert!(
            matches!(bad.parse::<LensModel>(), Err(Error::InvalidLensModelName(name)) if name == bad),
            "{bad:?} should be rejected"
        );
    }
}

#[test]
fn nul_byte_in_name_is_rejected() {
    assert_eq!(
        "LENSMODEL_\0PINHOLE".parse::<LensModel>(),
        Err(Error::InvalidString)
    );
}

#[test]
fn invalid_splined_configs_are_rejected() {
    let cases = [
        (
            LensModel::SplinedStereographic {
                order: 5,
                nx: 10,
                ny: 10,
                fov_x_deg: 150,
            },
            "spline order",
        ),
        (
            LensModel::SplinedStereographic {
                order: 3,
                nx: 3,
                ny: 10,
                fov_x_deg: 150,
            },
            "Nx, Ny >= 4",
        ),
        (
            LensModel::SplinedStereographic {
                order: 2,
                nx: 2,
                ny: 10,
                fov_x_deg: 150,
            },
            "Nx, Ny >= 3",
        ),
        (
            LensModel::SplinedStereographic {
                order: 3,
                nx: 10,
                ny: 10,
                fov_x_deg: 0,
            },
            "fov_x_deg",
        ),
        (
            LensModel::Cahvore {
                linearity: f64::NAN,
            },
            "finite",
        ),
    ];
    for (model, expected) in cases {
        // mrcal assert(0)s on these, so every entry point must reject them
        let err = model.validate().unwrap_err();
        let Error::InvalidLensModelConfig(why) = &err else {
            panic!("{model:?}: wrong error {err:?}");
        };
        assert!(why.contains(expected), "{model:?}: {why}");

        let n = model.num_params();
        assert_eq!(
            project(&[DVec3::new(0.1, 0.1, 1.0)], &model, &vec![1.0; n]).unwrap_err(),
            err
        );
        assert_eq!(
            unproject(&[DVec2::new(1.0, 1.0)], &model, &vec![1.0; n]).unwrap_err(),
            err
        );
        assert_eq!(
            CalibrationProblem::new(model, CalibrationObject::new(3, 3, 0.1)).unwrap_err(),
            err
        );
        assert!(model.name().parse::<LensModel>().is_err(), "{model:?}");
    }
}

#[test]
fn valid_splined_config_is_accepted() {
    let model = LensModel::SplinedStereographic {
        order: 3,
        nx: 10,
        ny: 8,
        fov_x_deg: 150,
    };
    model.validate().unwrap();
    assert_eq!(model.num_params(), 4 + 2 * 10 * 8);
    let q = project(
        &[DVec3::new(0.0, 0.0, 1.0)],
        &model,
        &vec![0.5; model.num_params()],
    )
    .unwrap();
    assert!(q[0].is_finite(), "{q:?}");
    assert_eq!(model.name().parse::<LensModel>().unwrap(), model);
}

#[test]
fn metadata_reports_model_properties() {
    let pinhole = LensModel::Pinhole.metadata();
    assert!(pinhole.has_core && pinhole.has_gradients);
    assert!(!pinhole.can_project_behind_camera && !pinhole.noncentral);

    let stereographic = LensModel::Stereographic.metadata();
    assert!(stereographic.can_project_behind_camera);

    // CAHVORE is the only noncentral model; it does have gradients in 2.5
    let model = LensModel::Cahvore { linearity: 0.25 };
    let cahvore = model.metadata();
    assert!(cahvore.noncentral);
    assert!(cahvore.has_gradients);
    let p = project_with_gradients(
        &[DVec3::new(0.1, 0.1, 1.0)],
        &model,
        &[
            1000.0, 1000.0, 320.0, 240.0, 0.01, -0.02, 0.001, -0.0005, 0.0002, 0.0, 0.0, 0.0,
        ],
    )
    .unwrap();
    assert!(p.q[0].is_finite() && p.dq_dp[0][0].is_finite());

    // Every model this crate exposes has gradients
    for model in all_models() {
        assert!(model.metadata().has_gradients, "{model}");
        assert!(model.metadata().has_core, "{model}");
    }
}
