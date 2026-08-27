use std::fmt;

#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum Error {
    /// The lens-model name was not recognized by mrcal.
    InvalidLensModelName(String),
    /// The configuration is outside what mrcal supports; see
    /// [`LensModel::validate`](crate::LensModel::validate).
    InvalidLensModelConfig(String),
    /// The intrinsics slice length doesn't match what the lens model expects.
    IntrinsicsCount {
        expected: usize,
        got: usize,
    },
    /// The point/pixel count exceeds what mrcal's `int` sizes can address.
    TooManyPoints,
    ProjectionFailed,
    UnprojectionFailed,
    /// mrcal could not parse the camera model; it prints details to stderr.
    CameraModelRead,
    /// The camera model file could not be written; carries the I/O error text.
    CameraModelWrite(String),
    /// A path or string held an interior NUL byte, or wasn't UTF-8.
    InvalidString,
    /// The index refers to an element that was never added.
    InvalidIndex {
        what: &'static str,
        index: usize,
        len: usize,
    },
    /// The number of corner observations doesn't match the calibration object.
    CornerCount {
        expected: usize,
        got: usize,
    },
    /// The calibration object needs at least 2x2 corners and positive spacing.
    InvalidCalibrationObject,
    /// The problem has no cameras, frames, or observations — or the flags
    /// leave no variables free to optimize.
    EmptyProblem,
    /// mrcal_optimize() failed; it prints details to stderr.
    OptimizationFailed,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::InvalidLensModelName(name) => write!(f, "invalid lens model name: {name:?}"),
            Error::InvalidLensModelConfig(why) => {
                write!(f, "invalid lens model configuration: {why}")
            }
            Error::IntrinsicsCount { expected, got } => {
                write!(f, "expected {expected} intrinsics values, got {got}")
            }
            Error::TooManyPoints => write!(f, "too many points for mrcal's int-sized counts"),
            Error::ProjectionFailed => write!(f, "mrcal_project() failed"),
            Error::UnprojectionFailed => write!(f, "mrcal_unproject() failed"),
            Error::CameraModelRead => write!(f, "failed to read camera model"),
            Error::CameraModelWrite(why) => write!(f, "failed to write camera model: {why}"),
            Error::InvalidString => {
                write!(f, "string contains an interior NUL byte or is not UTF-8")
            }
            Error::InvalidIndex { what, index, len } => {
                write!(f, "{what} index {index} out of range: {len} added")
            }
            Error::CornerCount { expected, got } => {
                write!(f, "expected {expected} corner observations, got {got}")
            }
            Error::InvalidCalibrationObject => {
                write!(
                    f,
                    "calibration object needs at least 2x2 corners and positive spacing"
                )
            }
            Error::EmptyProblem => {
                write!(f, "nothing to optimize: empty problem or no free variables")
            }
            Error::OptimizationFailed => write!(f, "mrcal_optimize() failed"),
        }
    }
}

impl std::error::Error for Error {}

pub type Result<T> = std::result::Result<T, Error>;
