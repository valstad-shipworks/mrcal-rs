# mrcal

Safe Rust bindings to [mrcal](https://mrcal.secretsauce.net), the camera
calibration and geometry toolkit.

The C library (plus its libdogleg and SuiteSparse/CHOLMOD dependencies) is
downloaded, built from source, and statically linked by
[`mrcal-sys`](mrcal-sys/README.md); nothing needs to be installed beyond a
C/C++ toolchain, CMake, libclang, and — on Linux — BLAS/LAPACK. No system
mrcal is required.

## What's wrapped

Geometry uses [glam](https://docs.rs/glam) types throughout: points and
pixels are `DVec3`/`DVec2`, rigid transforms are `DAffine3`. The crate
re-exports its `glam` (as `mrcal::glam`) so versions always match.

- **`CameraModel`** — read/write mrcal's `.cameramodel` format (from file or
  string), access lens model, intrinsics, extrinsics, and imager size. The
  writer round-trips every `f64` exactly, unlike mrcal's own `%f` writer
- **`LensModel`** — all mrcal lens models (pinhole, stereographic, lonlat,
  latlon, OpenCV 4/5/8/12, CAHVOR, CAHVORE, splined stereographic), with
  name parsing/formatting, parameter counts, configuration validation, and
  per-model `metadata()`
- **Projection** — `project`, `unproject`, and `project_with_gradients`
  (gradients w.r.t. both the point and the intrinsics)
- **`CalibrationProblem`** — the mrcal calibration optimizer
  (`mrcal_optimize()`): cameras observing a chessboard-like object, solved
  for intrinsics, extrinsics, and board poses, with outlier rejection and
  per-corner residuals
- **Triangulation** — geometric, Lindstrom, and the Lee-Civera family
  (l1, linf, mid2, wmid2)
- **Rodrigues conversions** — `rotation_from_rodrigues` /
  `rodrigues_from_rotation` for interop with mrcal's `rt` pose arrays

Reachable only through the re-exported `mrcal::sys`: discrete-point
observations, `mrcal_optimizer_callback()`, stereo rectification, projection
uncertainty, splined-model knots, and triangulation gradients.

## Example

```rust
use mrcal::glam::{DVec2, DVec3};
use mrcal::{CameraModel, triangulate_leecivera_mid2};

fn main() -> mrcal::Result<()> {
    let model = CameraModel::from_file("camera.cameramodel")?;

    // Camera-frame points -> pixels, and back
    let q = model.project(&[DVec3::new(0.1, -0.2, 1.0)])?;

    // Pixels where mrcal's per-pixel solve doesn't converge are None
    let v = model.unproject(&[DVec2::new(1228.0, 1099.0)])?;

    // Two observation rays -> 3D point
    let p = triangulate_leecivera_mid2(
        DVec3::new(1.0, 2.0, 10.0),
        DVec3::new(0.5, 2.0, 10.0),
        DVec3::new(0.5, 0.0, 0.0),
    );
    Ok(())
}
```

Calibrating a camera from detected chessboard corners:

```rust
use mrcal::glam::DVec2;
use mrcal::{CalibrationObject, CalibrationProblem, CornerObservation, LensModel};

fn main() -> mrcal::Result<()> {
    // An 11x8 grid of corners, 30mm apart
    let object = CalibrationObject::new(11, 8, 0.03);
    let mut problem = CalibrationProblem::new(LensModel::OpenCv8, object)?;

    // Seed with approximate intrinsics; the first camera pins the reference frame
    let seed = [1465.0, 1465.0, 1232.0, 1028.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    let cam = problem.add_camera(&seed, (2464, 2056), None)?;

    for (ref_from_board, corners) in detections {
        let frame = problem.add_frame(ref_from_board); // seed board pose (DAffine3)
        problem.add_observation(cam, frame, &corners)?; // &[CornerObservation]
    }

    // Consumes the problem: solved parameters are only readable on success
    let optimized = problem.optimize()?;
    println!("{}", optimized.stats().rms_reprojection_error_px);
    let residuals = optimized.corner_residuals(0)?; // per-corner error, px
    optimized.camera_model(cam)?.write_to_file("camera.cameramodel")?;
    Ok(())
}
```

## Building

First build needs network access to fetch the pinned C source tarballs (or
set the `*_SRC_DIR` environment variables for offline builds) — see the
[mrcal-sys README](mrcal-sys/README.md) for details and build requirements.

## Licensing

The Rust code in both crates is Apache-2.0. Binaries additionally carry the
licenses of the statically linked C libraries (Apache-2.0, BSD-3-Clause, and
LGPL parts; no GPL) — see [mrcal-sys's licensing
section](mrcal-sys/README.md#licensing).
