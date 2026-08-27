use crate::error::{Error, Result};
use mrcal_sys as sys;
use std::ffi::{CStr, CString};
use std::fmt;
use std::str::FromStr;

/// A mrcal lens model: the projection function type plus its configuration.
///
/// The intrinsics *values* travel separately, as a `&[f64]` of length
/// [`num_params`](LensModel::num_params).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum LensModel {
    Pinhole,
    Stereographic,
    LonLat,
    LatLon,
    OpenCv4,
    OpenCv5,
    OpenCv8,
    OpenCv12,
    Cahvor,
    Cahvore {
        linearity: f64,
    },
    SplinedStereographic {
        /// Spline order: 2 (quadratic) or 3 (cubic).
        order: u16,
        /// Control-point grid width; at least 3 for order 2, 4 for order 3.
        nx: u16,
        /// Control-point grid height; same minimum as `nx`.
        ny: u16,
        /// Horizontal field of view covered by the spline, in (0, 360).
        fov_x_deg: u16,
    },
}

/// Inherent properties of a lens model, from `mrcal_lensmodel_metadata()`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub struct LensModelMetadata {
    /// The first four intrinsics are the `fx, fy, cx, cy` core.
    pub has_core: bool,
    /// Points behind the camera project meaningfully, as for the
    /// stereographic-based models.
    pub can_project_behind_camera: bool,
    /// Gradients are implemented, so `project_with_gradients` and
    /// `unproject` work.
    pub has_gradients: bool,
    /// The rays need not all pass through one point (CAHVORE with nonzero E,
    /// which [`unproject`](crate::unproject) rejects).
    pub noncentral: bool,
}

impl LensModel {
    pub(crate) fn to_sys(self) -> sys::mrcal_lensmodel_t {
        let mut m = sys::mrcal_lensmodel_t {
            type_: match self {
                LensModel::Pinhole => sys::mrcal_lensmodel_type_t_MRCAL_LENSMODEL_PINHOLE,
                LensModel::Stereographic => {
                    sys::mrcal_lensmodel_type_t_MRCAL_LENSMODEL_STEREOGRAPHIC
                }
                LensModel::LonLat => sys::mrcal_lensmodel_type_t_MRCAL_LENSMODEL_LONLAT,
                LensModel::LatLon => sys::mrcal_lensmodel_type_t_MRCAL_LENSMODEL_LATLON,
                LensModel::OpenCv4 => sys::mrcal_lensmodel_type_t_MRCAL_LENSMODEL_OPENCV4,
                LensModel::OpenCv5 => sys::mrcal_lensmodel_type_t_MRCAL_LENSMODEL_OPENCV5,
                LensModel::OpenCv8 => sys::mrcal_lensmodel_type_t_MRCAL_LENSMODEL_OPENCV8,
                LensModel::OpenCv12 => sys::mrcal_lensmodel_type_t_MRCAL_LENSMODEL_OPENCV12,
                LensModel::Cahvor => sys::mrcal_lensmodel_type_t_MRCAL_LENSMODEL_CAHVOR,
                LensModel::Cahvore { .. } => sys::mrcal_lensmodel_type_t_MRCAL_LENSMODEL_CAHVORE,
                LensModel::SplinedStereographic { .. } => {
                    sys::mrcal_lensmodel_type_t_MRCAL_LENSMODEL_SPLINED_STEREOGRAPHIC
                }
            },
            __bindgen_anon_1: Default::default(),
        };
        match self {
            LensModel::Cahvore { linearity } => {
                m.__bindgen_anon_1.LENSMODEL_CAHVORE__config =
                    sys::mrcal_LENSMODEL_CAHVORE__config_t { linearity };
            }
            LensModel::SplinedStereographic {
                order,
                nx,
                ny,
                fov_x_deg,
            } => {
                m.__bindgen_anon_1.LENSMODEL_SPLINED_STEREOGRAPHIC__config =
                    sys::mrcal_LENSMODEL_SPLINED_STEREOGRAPHIC__config_t {
                        order,
                        Nx: nx,
                        Ny: ny,
                        fov_x_deg,
                    };
            }
            _ => {}
        }
        m
    }

    pub(crate) fn from_sys(m: &sys::mrcal_lensmodel_t) -> Option<Self> {
        #[allow(non_upper_case_globals)]
        Some(match m.type_ {
            sys::mrcal_lensmodel_type_t_MRCAL_LENSMODEL_PINHOLE => LensModel::Pinhole,
            sys::mrcal_lensmodel_type_t_MRCAL_LENSMODEL_STEREOGRAPHIC => LensModel::Stereographic,
            sys::mrcal_lensmodel_type_t_MRCAL_LENSMODEL_LONLAT => LensModel::LonLat,
            sys::mrcal_lensmodel_type_t_MRCAL_LENSMODEL_LATLON => LensModel::LatLon,
            sys::mrcal_lensmodel_type_t_MRCAL_LENSMODEL_OPENCV4 => LensModel::OpenCv4,
            sys::mrcal_lensmodel_type_t_MRCAL_LENSMODEL_OPENCV5 => LensModel::OpenCv5,
            sys::mrcal_lensmodel_type_t_MRCAL_LENSMODEL_OPENCV8 => LensModel::OpenCv8,
            sys::mrcal_lensmodel_type_t_MRCAL_LENSMODEL_OPENCV12 => LensModel::OpenCv12,
            sys::mrcal_lensmodel_type_t_MRCAL_LENSMODEL_CAHVOR => LensModel::Cahvor,
            sys::mrcal_lensmodel_type_t_MRCAL_LENSMODEL_CAHVORE => {
                let config = unsafe { m.__bindgen_anon_1.LENSMODEL_CAHVORE__config };
                LensModel::Cahvore {
                    linearity: config.linearity,
                }
            }
            sys::mrcal_lensmodel_type_t_MRCAL_LENSMODEL_SPLINED_STEREOGRAPHIC => {
                let config = unsafe { m.__bindgen_anon_1.LENSMODEL_SPLINED_STEREOGRAPHIC__config };
                LensModel::SplinedStereographic {
                    order: config.order,
                    nx: config.Nx,
                    ny: config.Ny,
                    fov_x_deg: config.fov_x_deg,
                }
            }
            _ => return None,
        })
    }

    /// Check the configuration against what mrcal can evaluate. mrcal
    /// `assert()`s on a bad one, so every entry point validates first.
    pub fn validate(&self) -> Result<()> {
        let bad = |why: String| Err(Error::InvalidLensModelConfig(why));
        match *self {
            LensModel::Cahvore { linearity } if !linearity.is_finite() => {
                bad(format!("CAHVORE linearity must be finite, got {linearity}"))
            }
            LensModel::SplinedStereographic {
                order,
                nx,
                ny,
                fov_x_deg,
            } => {
                let min_knots = match order {
                    2 => 3,
                    3 => 4,
                    _ => return bad(format!("spline order must be 2 or 3, got {order}")),
                };
                if nx < min_knots || ny < min_knots {
                    return bad(format!(
                        "order-{order} splines need Nx, Ny >= {min_knots}, got Nx={nx} Ny={ny}"
                    ));
                }
                if fov_x_deg == 0 || fov_x_deg >= 360 {
                    return bad(format!("fov_x_deg must be in 1..360, got {fov_x_deg}"));
                }
                Ok(())
            }
            _ => Ok(()),
        }
    }

    /// Inherent properties of this model type.
    pub fn metadata(&self) -> LensModelMetadata {
        let m = self.to_sys();
        let meta = unsafe { sys::mrcal_lensmodel_metadata(&m) };
        LensModelMetadata {
            has_core: meta.has_core(),
            can_project_behind_camera: meta.can_project_behind_camera(),
            has_gradients: meta.has_gradients(),
            noncentral: meta.noncentral(),
        }
    }

    /// Number of intrinsics values this model expects (core + distortions).
    pub fn num_params(&self) -> usize {
        let m = self.to_sys();
        let n = unsafe { sys::mrcal_lensmodel_num_params(&m) };
        usize::try_from(n).expect("valid lens model has a non-negative parameter count")
    }

    /// The full configured name, e.g.
    /// `LENSMODEL_SPLINED_STEREOGRAPHIC_order=3_Nx=30_Ny=20_fov_x_deg=170`.
    pub fn name(&self) -> String {
        let m = self.to_sys();
        let mut buf = [0u8; 512];
        let ok =
            unsafe { sys::mrcal_lensmodel_name(buf.as_mut_ptr().cast(), buf.len() as i32, &m) };
        assert!(ok, "mrcal_lensmodel_name() failed");
        CStr::from_bytes_until_nul(&buf)
            .expect("mrcal_lensmodel_name() produced no NUL terminator")
            .to_string_lossy()
            .into_owned()
    }
}

impl FromStr for LensModel {
    type Err = Error;

    fn from_str(s: &str) -> Result<Self> {
        let name = CString::new(s).map_err(|_| Error::InvalidString)?;
        let mut m = sys::mrcal_lensmodel_t::default();
        let ok = unsafe { sys::mrcal_lensmodel_from_name(&mut m, name.as_ptr()) };
        if !ok {
            return Err(Error::InvalidLensModelName(s.to_owned()));
        }
        let model =
            LensModel::from_sys(&m).ok_or_else(|| Error::InvalidLensModelName(s.to_owned()))?;
        model.validate()?;
        Ok(model)
    }
}

impl fmt::Display for LensModel {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.name())
    }
}
