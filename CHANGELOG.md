# Changelog

## Unreleased

First public release.

The crate version tracks the bundled mrcal C library version (2.5.2); see
[mrcal-sys's versioning note](mrcal-sys/README.md#versioning).

### Added

- `LensModel::validate()`, rejecting splined and CAHVORE configurations that
  make mrcal abort the process, and `Error::InvalidLensModelConfig`. Every
  entry point that hands a model to C validates first.
- `LensModel::metadata()` and `LensModelMetadata`, exposing `has_core`,
  `can_project_behind_camera`, `has_gradients`, and `noncentral`.
- `CameraModel::new()` to build a model from its parts, `Display` rendering
  the `.cameramodel` format at full `f64` precision, and `Clone`.
- `OptimizedProblem::residuals()` and `corner_residuals()`, the measurement
  vector from the solver.
- `OptimizeFlags::NONE` and `with_*` builders.
- `Debug` for `CalibrationProblem`.
- `Error::TooManyPoints`, returned instead of truncating point counts that
  overflow the FFI's `int`.

### Changed

- `unproject()` returns `Vec<Option<DVec3>>`: mrcal's per-pixel inner solve
  can fail to converge and previously returned NaN inside `Ok`.
- `CameraModel::write_to_file()` writes full precision rather than going
  through mrcal's `%f` writer, which truncated distortion coefficients to six
  decimal places.
- `OptimizedProblem::intrinsics()`, `cam_from_ref()`, `ref_from_frame()`, and
  `corners()` return `Result` instead of panicking on an out-of-range index.
- `Error::CameraModelWrite` carries the underlying I/O error text.
- `Error`, `OptimizeFlags`, `OptimizeStats`, and `LensModelMetadata` are
  `#[non_exhaustive]`.
- `OptimizeStats::rms_reprojection_error_px` documents mrcal's actual
  normalization (over all measurements, including regularization).

### Fixed

- `mrcal-sys` builds on docs.rs, which has no network access, from
  checked-in bindings.
- BLAS/LAPACK is discovered via `pkg-config` with a `MRCAL_LAPACK_LIBS`
  override, instead of assuming `-llapack -lblas`.
- The SuiteSparse build cache is detected in `lib64` as well as `lib`.
- `mrcal-sys` ships its `LICENSE`, and both crates carry `repository`,
  `rust-version`, and authorship metadata.
