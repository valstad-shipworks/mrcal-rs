use crate::error::{Error, Result};
use crate::lensmodel::LensModel;
use glam::{DVec2, DVec3};
use mrcal_sys as sys;
use std::ptr;

fn check_intrinsics(lensmodel: &LensModel, intrinsics: &[f64]) -> Result<()> {
    lensmodel.validate()?;
    let expected = lensmodel.num_params();
    if intrinsics.len() != expected {
        return Err(Error::IntrinsicsCount {
            expected,
            got: intrinsics.len(),
        });
    }
    Ok(())
}

fn count(n: usize) -> Result<i32> {
    i32::try_from(n).map_err(|_| Error::TooManyPoints)
}

/// Project camera-frame points to pixel coordinates.
///
/// Points mrcal cannot project — at the camera origin, or behind a camera
/// whose model can't ([`LensModel::metadata`]) — come back as NaN, not `Err`.
pub fn project(points: &[DVec3], lensmodel: &LensModel, intrinsics: &[f64]) -> Result<Vec<DVec2>> {
    check_intrinsics(lensmodel, intrinsics)?;
    let n = count(points.len())?;
    let m = lensmodel.to_sys();
    let mut q = vec![DVec2::ZERO; points.len()];
    let ok = unsafe {
        sys::mrcal_project(
            q.as_mut_ptr().cast(),
            ptr::null_mut(),
            ptr::null_mut(),
            points.as_ptr().cast(),
            n,
            &m,
            intrinsics.as_ptr(),
        )
    };
    if !ok {
        return Err(Error::ProjectionFailed);
    }
    Ok(q)
}

/// Result of [`project_with_gradients`].
pub struct Projection {
    /// Pixel coordinates, one per input point.
    pub q: Vec<DVec2>,
    /// Gradient of each pixel coordinate w.r.t. the camera-frame point:
    /// `dq_dp[i][0]` is the gradient of `q[i].x`, `dq_dp[i][1]` of `q[i].y`.
    pub dq_dp: Vec<[DVec3; 2]>,
    /// Gradient of each pixel coordinate w.r.t. the intrinsics, flattened as
    /// `(N, 2, num_params)` row-major. Dense even for splined models, whose
    /// hundreds of parameters have mostly-zero gradients.
    pub dq_dintrinsics: Vec<f64>,
}

/// Project camera-frame points to pixel coordinates, also returning gradients.
///
/// Fails for models without gradients; see [`LensModel::metadata`].
pub fn project_with_gradients(
    points: &[DVec3],
    lensmodel: &LensModel,
    intrinsics: &[f64],
) -> Result<Projection> {
    check_intrinsics(lensmodel, intrinsics)?;
    let n = count(points.len())?;
    let m = lensmodel.to_sys();
    let n_params = intrinsics.len();
    let mut q = vec![DVec2::ZERO; points.len()];
    let mut dq_dp = vec![[DVec3::ZERO; 2]; points.len()];
    let mut dq_dintrinsics = vec![0.0; points.len() * 2 * n_params];
    let ok = unsafe {
        sys::mrcal_project(
            q.as_mut_ptr().cast(),
            dq_dp.as_mut_ptr().cast(),
            dq_dintrinsics.as_mut_ptr(),
            points.as_ptr().cast(),
            n,
            &m,
            intrinsics.as_ptr(),
        )
    };
    if !ok {
        return Err(Error::ProjectionFailed);
    }
    Ok(Projection {
        q,
        dq_dp,
        dq_dintrinsics,
    })
}

/// Unproject pixel coordinates to camera-frame direction vectors.
///
/// Only the direction of the returned vectors is meaningful. mrcal solves
/// each pixel iteratively; those that don't converge are `None`.
pub fn unproject(
    pixels: &[DVec2],
    lensmodel: &LensModel,
    intrinsics: &[f64],
) -> Result<Vec<Option<DVec3>>> {
    check_intrinsics(lensmodel, intrinsics)?;
    let n = count(pixels.len())?;
    let m = lensmodel.to_sys();
    let mut v = vec![DVec3::ZERO; pixels.len()];
    let ok = unsafe {
        sys::mrcal_unproject(
            v.as_mut_ptr().cast(),
            pixels.as_ptr().cast(),
            n,
            &m,
            intrinsics.as_ptr(),
        )
    };
    if !ok {
        return Err(Error::UnprojectionFailed);
    }
    Ok(v.into_iter().map(|v| v.is_finite().then_some(v)).collect())
}
