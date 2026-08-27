//! Safe Rust bindings to [mrcal](https://mrcal.secretsauce.net), the camera
//! calibration and geometry toolkit.
//!
//! `mrcal-sys` builds and statically links the C library; you need a C/C++
//! toolchain, CMake, libclang, and BLAS/LAPACK on Linux.
//!
//! Wraps camera-model I/O, projection, triangulation, and the calibration
//! optimizer ([`CalibrationProblem`]). Geometry uses [`glam`]: points and
//! pixels are [`glam::DVec3`]/[`glam::DVec2`], transforms [`glam::DAffine3`].

mod cameramodel;
mod error;
mod geometry;
mod lensmodel;
mod optimize;
mod poseutils;
mod projection;
mod triangulation;

pub use cameramodel::CameraModel;
pub use error::{Error, Result};
pub use lensmodel::{LensModel, LensModelMetadata};
pub use optimize::{
    CalibrationObject, CalibrationProblem, CornerObservation, OptimizeFlags, OptimizeStats,
    OptimizedProblem,
};
pub use poseutils::{rodrigues_from_rotation, rotation_from_rodrigues};
pub use projection::{Projection, project, project_with_gradients, unproject};
pub use triangulation::{
    triangulate_geometric, triangulate_leecivera_l1, triangulate_leecivera_linf,
    triangulate_leecivera_mid2, triangulate_leecivera_wmid2, triangulate_lindstrom,
};

pub use glam;
pub use mrcal_sys as sys;
