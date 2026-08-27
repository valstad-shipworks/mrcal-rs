use crate::cameramodel::CameraModel;
use crate::error::{Error, Result};
use crate::lensmodel::LensModel;
use crate::poseutils;
use glam::{DAffine3, DVec2};
use mrcal_sys as sys;
use std::ptr;

/// A chessboard-like calibration object: a regular planar grid of corners.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CalibrationObject {
    /// Number of corners along the x (fastest-varying) direction.
    pub width_n: usize,
    /// Number of corners along the y direction.
    pub height_n: usize,
    /// Distance in meters between adjacent corners.
    pub spacing: f64,
}

impl CalibrationObject {
    pub const fn new(width_n: usize, height_n: usize, spacing: f64) -> Self {
        Self {
            width_n,
            height_n,
            spacing,
        }
    }

    fn corners_per_observation(&self) -> usize {
        self.width_n * self.height_n
    }
}

/// One observed corner of the calibration object.
///
/// Layout-compatible with the `mrcal_point3_t` pool entries: pixel plus
/// weight. A negative weight marks an outlier, which
/// [`optimize`](CalibrationProblem::optimize) also writes back.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CornerObservation {
    pub px: DVec2,
    pub weight: f64,
}

const _: () = {
    assert!(size_of::<CornerObservation>() == size_of::<sys::mrcal_point3_t>());
    assert!(align_of::<CornerObservation>() == align_of::<sys::mrcal_point3_t>());
};

impl CornerObservation {
    /// A corner observed with the default weight of 1.
    pub const fn new(px: DVec2) -> Self {
        Self { px, weight: 1.0 }
    }

    /// A corner weighted inversely to its observation noise.
    pub const fn weighted(px: DVec2, weight: f64) -> Self {
        Self { px, weight }
    }

    /// A corner that was not detected in this observation.
    pub const MISSING: Self = Self {
        px: DVec2::ZERO,
        weight: -1.0,
    };

    pub fn is_outlier(&self) -> bool {
        self.weight < 0.0
    }
}

/// Which parts of the problem [`CalibrationProblem::optimize`] is allowed to
/// move, mirroring mrcal's `mrcal_problem_selections_t`. Start from
/// [`Default`] or [`NONE`](Self::NONE) and adjust with the `with_*` builders:
///
/// ```
/// # use mrcal::OptimizeFlags;
/// let flags = OptimizeFlags::default().with_outlier_rejection(false);
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct OptimizeFlags {
    /// Optimize the fx/fy/cx/cy intrinsics core.
    pub intrinsics_core: bool,
    /// Optimize the distortion coefficients.
    pub intrinsics_distortions: bool,
    /// Optimize the non-reference cameras' poses.
    pub extrinsics: bool,
    /// Optimize the board poses.
    pub frames: bool,
    /// Optimize the board deformation (see [`CalibrationProblem::set_calobject_warp`]).
    pub calobject_warp: bool,
    /// Apply mrcal's light regularization terms.
    pub regularization: bool,
    /// Throw out outlier observations as the solve progresses.
    pub outlier_rejection: bool,
}

impl Default for OptimizeFlags {
    fn default() -> Self {
        Self {
            intrinsics_core: true,
            intrinsics_distortions: true,
            extrinsics: true,
            frames: true,
            calobject_warp: false,
            regularization: true,
            outlier_rejection: true,
        }
    }
}

impl OptimizeFlags {
    /// Nothing is optimized; a base for the `with_*` builders. Solving with
    /// no free variables is [`Error::EmptyProblem`].
    pub const NONE: Self = Self {
        intrinsics_core: false,
        intrinsics_distortions: false,
        extrinsics: false,
        frames: false,
        calobject_warp: false,
        regularization: false,
        outlier_rejection: false,
    };

    /// Set [`intrinsics_core`](Self::intrinsics_core).
    pub const fn with_intrinsics_core(mut self, on: bool) -> Self {
        self.intrinsics_core = on;
        self
    }

    /// Set [`intrinsics_distortions`](Self::intrinsics_distortions).
    pub const fn with_intrinsics_distortions(mut self, on: bool) -> Self {
        self.intrinsics_distortions = on;
        self
    }

    /// Set [`extrinsics`](Self::extrinsics).
    pub const fn with_extrinsics(mut self, on: bool) -> Self {
        self.extrinsics = on;
        self
    }

    /// Set [`frames`](Self::frames).
    pub const fn with_frames(mut self, on: bool) -> Self {
        self.frames = on;
        self
    }

    /// Set [`calobject_warp`](Self::calobject_warp).
    pub const fn with_calobject_warp(mut self, on: bool) -> Self {
        self.calobject_warp = on;
        self
    }

    /// Set [`regularization`](Self::regularization).
    pub const fn with_regularization(mut self, on: bool) -> Self {
        self.regularization = on;
        self
    }

    /// Set [`outlier_rejection`](Self::outlier_rejection).
    pub const fn with_outlier_rejection(mut self, on: bool) -> Self {
        self.outlier_rejection = on;
        self
    }

    fn to_sys(self) -> sys::mrcal_problem_selections_t {
        let mut s = sys::mrcal_problem_selections_t::default();
        s.set_do_optimize_intrinsics_core(self.intrinsics_core);
        s.set_do_optimize_intrinsics_distortions(self.intrinsics_distortions);
        s.set_do_optimize_extrinsics(self.extrinsics);
        s.set_do_optimize_frames(self.frames);
        s.set_do_optimize_calobject_warp(self.calobject_warp);
        s.set_do_apply_regularization(self.regularization);
        s.set_do_apply_outlier_rejection(self.outlier_rejection);
        s
    }
}

/// Statistics reported by a successful [`CalibrationProblem::optimize`].
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct OptimizeStats {
    /// `sqrt(norm2(x) / len(x))` over the whole measurement vector, in
    /// pixels — so it counts each corner twice and includes regularization,
    /// unlike mrcal's header comment. For a per-corner error use
    /// [`residuals`](OptimizedProblem::residuals).
    pub rms_reprojection_error_px: f64,
    /// Total number of corner observations marked as outliers (both
    /// pre-existing and newly rejected).
    pub board_outliers: usize,
}

/// N cameras observing a moving calibration object, solved by
/// `mrcal_optimize()`.
///
/// Build it up with [`add_camera`], [`add_frame`] and [`add_observation`],
/// seeding every parameter, then [`optimize`]. That consumes the problem, so
/// a seed can never be mistaken for a result.
///
/// [`add_camera`]: CalibrationProblem::add_camera
/// [`add_frame`]: CalibrationProblem::add_frame
/// [`add_observation`]: CalibrationProblem::add_observation
/// [`optimize`]: CalibrationProblem::optimize
pub struct CalibrationProblem {
    lensmodel: LensModel,
    object: CalibrationObject,
    intrinsics: Vec<f64>,
    imagersizes: Vec<i32>,
    extrinsics_index: Vec<Option<usize>>,
    rt_cam_ref: Vec<sys::mrcal_pose_t>,
    rt_ref_frame: Vec<sys::mrcal_pose_t>,
    observations: Vec<sys::mrcal_observation_board_t>,
    pool: Vec<CornerObservation>,
    calobject_warp: sys::mrcal_calobject_warp_t,
    pub flags: OptimizeFlags,
    /// Have mrcal print solver progress to stderr.
    pub verbose: bool,
}

impl CalibrationProblem {
    pub fn new(lensmodel: LensModel, object: CalibrationObject) -> Result<Self> {
        lensmodel.validate()?;
        if object.width_n < 2
            || object.height_n < 2
            || object.spacing <= 0.0
            || object.spacing.is_nan()
        {
            return Err(Error::InvalidCalibrationObject);
        }
        Ok(Self {
            lensmodel,
            object,
            intrinsics: Vec::new(),
            imagersizes: Vec::new(),
            extrinsics_index: Vec::new(),
            rt_cam_ref: Vec::new(),
            rt_ref_frame: Vec::new(),
            observations: Vec::new(),
            pool: Vec::new(),
            calobject_warp: sys::mrcal_calobject_warp_t::default(),
            flags: OptimizeFlags::default(),
            verbose: false,
        })
    }

    /// Add a camera with seed intrinsics, returning its index.
    ///
    /// `cam_from_ref` seeds the pose mapping reference-frame points into this
    /// camera. `None` pins the camera at the reference, fixing its pose to
    /// identity; with no such camera the pose gauge is unconstrained.
    pub fn add_camera(
        &mut self,
        intrinsics: &[f64],
        imagersize: (u32, u32),
        cam_from_ref: Option<DAffine3>,
    ) -> Result<usize> {
        let expected = self.lensmodel.num_params();
        if intrinsics.len() != expected {
            return Err(Error::IntrinsicsCount {
                expected,
                got: intrinsics.len(),
            });
        }
        self.intrinsics.extend_from_slice(intrinsics);
        self.imagersizes
            .extend([imagersize.0 as i32, imagersize.1 as i32]);
        self.extrinsics_index.push(cam_from_ref.map(|pose| {
            self.rt_cam_ref.push(poseutils::pose_from_affine(&pose));
            self.rt_cam_ref.len() - 1
        }));
        Ok(self.extrinsics_index.len() - 1)
    }

    /// Add one pose of the calibration object, returning its index. Board
    /// corner `(i, j)` sits at `(j * spacing, i * spacing, 0)`.
    pub fn add_frame(&mut self, ref_from_board: DAffine3) -> usize {
        self.rt_ref_frame
            .push(poseutils::pose_from_affine(&ref_from_board));
        self.rt_ref_frame.len() - 1
    }

    /// Add one observation: `camera` saw the board of `frame`. `corners` is
    /// `width_n * height_n` entries, x fastest, using
    /// [`MISSING`](CornerObservation::MISSING) for undetected ones.
    pub fn add_observation(
        &mut self,
        camera: usize,
        frame: usize,
        corners: &[CornerObservation],
    ) -> Result<usize> {
        if camera >= self.extrinsics_index.len() {
            return Err(Error::InvalidIndex {
                what: "camera",
                index: camera,
                len: self.extrinsics_index.len(),
            });
        }
        if frame >= self.rt_ref_frame.len() {
            return Err(Error::InvalidIndex {
                what: "frame",
                index: frame,
                len: self.rt_ref_frame.len(),
            });
        }
        let expected = self.object.corners_per_observation();
        if corners.len() != expected {
            return Err(Error::CornerCount {
                expected,
                got: corners.len(),
            });
        }
        self.observations.push(sys::mrcal_observation_board_t {
            icam: sys::mrcal_camera_index_t {
                intrinsics: camera as i32,
                extrinsics: self.extrinsics_index[camera].map_or(-1, |i| i as i32),
            },
            iframe: frame as i32,
        });
        self.pool.extend_from_slice(corners);
        Ok(self.observations.len() - 1)
    }

    /// Seed the board deformation (mrcal's `calobject_warp`). Only optimized
    /// when [`OptimizeFlags::calobject_warp`] is set.
    pub fn set_calobject_warp(&mut self, warp: DVec2) {
        self.calobject_warp = sys::mrcal_calobject_warp_t {
            values: warp.to_array(),
        };
    }

    pub fn num_cameras(&self) -> usize {
        self.extrinsics_index.len()
    }

    pub fn num_frames(&self) -> usize {
        self.rt_ref_frame.len()
    }

    pub fn num_observations(&self) -> usize {
        self.observations.len()
    }

    fn check_index(what: &'static str, index: usize, len: usize) -> Result<()> {
        if index < len {
            Ok(())
        } else {
            Err(Error::InvalidIndex { what, index, len })
        }
    }

    /// Solve, consuming the problem. On failure the partially mutated seeds
    /// are dropped.
    pub fn optimize(mut self) -> Result<OptimizedProblem> {
        if self.extrinsics_index.is_empty()
            || self.rt_ref_frame.is_empty()
            || self.observations.is_empty()
        {
            return Err(Error::EmptyProblem);
        }
        let lensmodel = self.lensmodel.to_sys();
        // Zero state variables corrupts memory inside dogleg
        let num_states = unsafe {
            sys::mrcal_num_states(
                self.extrinsics_index.len() as i32,
                self.rt_cam_ref.len() as i32,
                self.rt_ref_frame.len() as i32,
                0,
                0,
                self.observations.len() as i32,
                self.flags.to_sys(),
                &lensmodel,
            )
        };
        if num_states <= 0 {
            return Err(Error::EmptyProblem);
        }
        let num_measurements = unsafe {
            sys::mrcal_num_measurements(
                self.observations.len() as i32,
                0,
                ptr::null(),
                0,
                self.object.width_n as i32,
                self.object.height_n as i32,
                self.extrinsics_index.len() as i32,
                self.rt_cam_ref.len() as i32,
                self.rt_ref_frame.len() as i32,
                0,
                0,
                self.flags.to_sys(),
                &lensmodel,
            )
        };
        let mut residuals = vec![0.0; usize::try_from(num_measurements).unwrap_or(0)];
        let constants = sys::mrcal_problem_constants_t {
            point_min_range: 1.0,
            point_max_range: 1e12,
        };
        let stats = unsafe {
            sys::mrcal_optimize(
                ptr::null_mut(),
                0,
                residuals.as_mut_ptr(),
                (residuals.len() * size_of::<f64>()) as i32,
                self.intrinsics.as_mut_ptr(),
                self.rt_cam_ref.as_mut_ptr(),
                self.rt_ref_frame.as_mut_ptr(),
                ptr::null_mut(),
                &mut self.calobject_warp,
                self.extrinsics_index.len() as i32,
                self.rt_cam_ref.len() as i32,
                self.rt_ref_frame.len() as i32,
                0,
                0,
                self.observations.as_ptr(),
                ptr::null(),
                self.observations.len() as i32,
                0,
                ptr::null(),
                0,
                self.pool.as_mut_ptr().cast(),
                ptr::null_mut(),
                &lensmodel,
                self.imagersizes.as_ptr(),
                self.flags.to_sys(),
                &constants,
                self.object.spacing,
                self.object.width_n as i32,
                self.object.height_n as i32,
                self.verbose,
                false,
            )
        };
        if stats.rms_reproj_error__pixels < 0.0 {
            return Err(Error::OptimizationFailed);
        }
        Ok(OptimizedProblem {
            problem: self,
            residuals,
            stats: OptimizeStats {
                rms_reprojection_error_px: stats.rms_reproj_error__pixels,
                board_outliers: stats.Noutliers_board as usize,
            },
        })
    }
}

/// A successfully solved [`CalibrationProblem`]: the solution parameters and
/// fit statistics.
///
/// For staged solves, [`into_problem`](OptimizedProblem::into_problem) seeds
/// a fresh problem with these values.
pub struct OptimizedProblem {
    problem: CalibrationProblem,
    residuals: Vec<f64>,
    stats: OptimizeStats,
}

impl OptimizedProblem {
    pub fn stats(&self) -> OptimizeStats {
        self.stats
    }

    /// The solved intrinsics of a camera.
    pub fn intrinsics(&self, camera: usize) -> Result<&[f64]> {
        CalibrationProblem::check_index("camera", camera, self.num_cameras())?;
        let n = self.problem.lensmodel.num_params();
        Ok(&self.problem.intrinsics[camera * n..(camera + 1) * n])
    }

    /// The solved pose of a camera; identity for reference cameras.
    pub fn cam_from_ref(&self, camera: usize) -> Result<DAffine3> {
        CalibrationProblem::check_index("camera", camera, self.num_cameras())?;
        Ok(match self.problem.extrinsics_index[camera] {
            Some(i) => poseutils::affine_from_pose(&self.problem.rt_cam_ref[i]),
            None => DAffine3::IDENTITY,
        })
    }

    /// The solved board pose of a frame.
    pub fn ref_from_frame(&self, frame: usize) -> Result<DAffine3> {
        CalibrationProblem::check_index("frame", frame, self.num_frames())?;
        Ok(poseutils::affine_from_pose(
            &self.problem.rt_ref_frame[frame],
        ))
    }

    /// The corners as passed to
    /// [`add_observation`](CalibrationProblem::add_observation), with rejected
    /// outliers now negatively weighted.
    pub fn corners(&self, observation: usize) -> Result<&[CornerObservation]> {
        CalibrationProblem::check_index("observation", observation, self.num_observations())?;
        let n = self.problem.object.corners_per_observation();
        Ok(&self.problem.pool[observation * n..(observation + 1) * n])
    }

    /// The measurement vector at the solution: two weighted pixel residuals
    /// per corner in the order observations were added, then mrcal's
    /// regularization terms.
    pub fn residuals(&self) -> &[f64] {
        &self.residuals
    }

    /// Each corner's residual, in weight-scaled pixels and zero for
    /// outliers, ordered as [`corners`](OptimizedProblem::corners).
    pub fn corner_residuals(&self, observation: usize) -> Result<Vec<DVec2>> {
        CalibrationProblem::check_index("observation", observation, self.num_observations())?;
        let n = self.problem.object.corners_per_observation();
        let start = unsafe {
            sys::mrcal_measurement_index_boards(
                observation as i32,
                self.num_observations() as i32,
                0,
                self.problem.object.width_n as i32,
                self.problem.object.height_n as i32,
            )
        } as usize;
        Ok(self.residuals[start..start + 2 * n]
            .chunks_exact(2)
            .map(|xy| DVec2::new(xy[0], xy[1]))
            .collect())
    }

    /// The solved board deformation (the seed, if it wasn't optimized).
    pub fn calobject_warp(&self) -> DVec2 {
        DVec2::from_array(unsafe { self.problem.calobject_warp.values })
    }

    pub fn num_cameras(&self) -> usize {
        self.problem.num_cameras()
    }

    pub fn num_frames(&self) -> usize {
        self.problem.num_frames()
    }

    pub fn num_observations(&self) -> usize {
        self.problem.num_observations()
    }

    /// Build a [`CameraModel`] for one camera from the solved parameters.
    pub fn camera_model(&self, camera: usize) -> Result<CameraModel> {
        let p = &self.problem;
        let intrinsics = self.intrinsics(camera)?;
        let rt = match p.extrinsics_index[camera] {
            Some(i) => {
                let p = &p.rt_cam_ref[i];
                unsafe {
                    [
                        p.r.xyz[0], p.r.xyz[1], p.r.xyz[2], p.t.xyz[0], p.t.xyz[1], p.t.xyz[2],
                    ]
                }
            }
            None => [0.0; 6],
        };
        let imagersize = (
            p.imagersizes[camera * 2] as u32,
            p.imagersizes[camera * 2 + 1] as u32,
        );
        CameraModel::from_parts(&p.lensmodel, intrinsics, &rt, imagersize)
    }

    /// Reseed a [`CalibrationProblem`] with the solved parameters, e.g. to
    /// re-solve with different [`flags`](CalibrationProblem::flags). Outlier
    /// marks are kept.
    pub fn into_problem(self) -> CalibrationProblem {
        self.problem
    }
}

impl std::fmt::Debug for CalibrationProblem {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("CalibrationProblem")
            .field("lensmodel", &self.lensmodel)
            .field("object", &self.object)
            .field("num_cameras", &self.num_cameras())
            .field("num_frames", &self.num_frames())
            .field("num_observations", &self.num_observations())
            .field("flags", &self.flags)
            .finish_non_exhaustive()
    }
}

impl std::fmt::Debug for OptimizedProblem {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("OptimizedProblem")
            .field("stats", &self.stats)
            .field("num_cameras", &self.num_cameras())
            .field("num_frames", &self.num_frames())
            .field("num_observations", &self.num_observations())
            .finish_non_exhaustive()
    }
}
