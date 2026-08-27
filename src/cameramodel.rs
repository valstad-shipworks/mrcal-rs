use crate::error::{Error, Result};
use crate::lensmodel::LensModel;
use crate::poseutils;
use crate::projection;
use glam::{DAffine3, DVec2, DVec3};
use mrcal_sys as sys;
use std::ffi::CString;
use std::fmt::{self, Write as _};
use std::path::Path;
use std::ptr::NonNull;
use std::str::FromStr;

/// An owned mrcal camera model: lens model + intrinsics + extrinsics + imager size.
///
/// Read from or written to mrcal's `.cameramodel` file format, which
/// [`Display`](fmt::Display) renders at full `f64` precision.
pub struct CameraModel {
    ptr: NonNull<sys::mrcal_cameramodel_VOID_t>,
}

// Plain data owned by this handle; no mrcal API mutates it through &self
unsafe impl Send for CameraModel {}
unsafe impl Sync for CameraModel {}

fn path_to_cstring(path: &Path) -> Result<CString> {
    CString::new(path.to_str().ok_or(Error::InvalidString)?).map_err(|_| Error::InvalidString)
}

/// Render a model in the `.cameramodel` format. Rust's shortest round-trip
/// formatting reads back exactly through mrcal's `strtod` parser.
pub(crate) fn format_cameramodel(
    lensmodel: &LensModel,
    intrinsics: &[f64],
    rt_cam_ref: &[f64; 6],
    imagersize: (u32, u32),
) -> String {
    let mut text = String::new();
    writeln!(text, "{{").unwrap();
    writeln!(text, "    'lensmodel':  '{lensmodel}',").unwrap();
    write!(text, "    'intrinsics': [").unwrap();
    for v in intrinsics {
        write!(text, " {v:?},").unwrap();
    }
    writeln!(text, " ],").unwrap();
    write!(text, "    'extrinsics': [").unwrap();
    for v in rt_cam_ref {
        write!(text, " {v:?},").unwrap();
    }
    writeln!(text, " ],").unwrap();
    let (w, h) = imagersize;
    writeln!(text, "    'imagersize': [ {w}, {h} ],").unwrap();
    writeln!(text, "}}").unwrap();
    text
}

impl CameraModel {
    /// Build a model from its parts. `cam_from_ref` maps reference-frame
    /// points into this camera's frame.
    pub fn new(
        lensmodel: LensModel,
        intrinsics: &[f64],
        cam_from_ref: DAffine3,
        imagersize: (u32, u32),
    ) -> Result<Self> {
        Self::from_parts(
            &lensmodel,
            intrinsics,
            &poseutils::rt_from_affine(&cam_from_ref),
            imagersize,
        )
    }

    pub(crate) fn from_parts(
        lensmodel: &LensModel,
        intrinsics: &[f64],
        rt_cam_ref: &[f64; 6],
        imagersize: (u32, u32),
    ) -> Result<Self> {
        lensmodel.validate()?;
        let expected = lensmodel.num_params();
        if intrinsics.len() != expected {
            return Err(Error::IntrinsicsCount {
                expected,
                got: intrinsics.len(),
            });
        }
        format_cameramodel(lensmodel, intrinsics, rt_cam_ref, imagersize).parse()
    }

    pub fn from_file(path: impl AsRef<Path>) -> Result<Self> {
        let path = path_to_cstring(path.as_ref())?;
        let ptr = unsafe { sys::mrcal_read_cameramodel_file(path.as_ptr()) };
        NonNull::new(ptr)
            .map(|ptr| Self { ptr })
            .ok_or(Error::CameraModelRead)
    }

    /// Write the model in the `.cameramodel` format. Not
    /// `mrcal_write_cameramodel_file()`, whose `%f` truncates distortions to
    /// six decimal places.
    pub fn write_to_file(&self, path: impl AsRef<Path>) -> Result<()> {
        std::fs::write(path, self.to_string()).map_err(|e| Error::CameraModelWrite(e.to_string()))
    }

    fn inner(&self) -> &sys::mrcal_cameramodel_VOID_t__bindgen_ty_1__bindgen_ty_1 {
        unsafe { self.ptr.as_ref().__bindgen_anon_1.__bindgen_anon_1.as_ref() }
    }

    pub fn lensmodel(&self) -> LensModel {
        LensModel::from_sys(&self.inner().lensmodel)
            .expect("a parsed camera model always has a valid lens model")
    }

    pub fn intrinsics(&self) -> &[f64] {
        let n = self.lensmodel().num_params();
        unsafe { self.inner().intrinsics.as_slice(n) }
    }

    fn rt_cam_ref(&self) -> &[f64; 6] {
        &unsafe { self.ptr.as_ref() }.rt_cam_ref
    }

    /// The transform mapping points in the reference frame to the camera
    /// frame (mrcal's `rt_cam_ref`).
    pub fn cam_from_ref(&self) -> DAffine3 {
        poseutils::affine_from_rt(self.rt_cam_ref())
    }

    /// Imager `(width, height)` in pixels.
    pub fn imagersize(&self) -> (u32, u32) {
        let [w, h] = self.inner().imagersize;
        (w, h)
    }

    /// Project camera-frame points to pixels using this model's intrinsics.
    /// See [`project`](crate::project).
    pub fn project(&self, points: &[DVec3]) -> Result<Vec<DVec2>> {
        projection::project(points, &self.lensmodel(), self.intrinsics())
    }

    /// Unproject pixels to camera-frame observation directions using this
    /// model's intrinsics. See [`unproject`](crate::unproject).
    pub fn unproject(&self, pixels: &[DVec2]) -> Result<Vec<Option<DVec3>>> {
        projection::unproject(pixels, &self.lensmodel(), self.intrinsics())
    }
}

impl FromStr for CameraModel {
    type Err = Error;

    /// Parse a model from a string in the `.cameramodel` file format.
    fn from_str(s: &str) -> Result<Self> {
        // mrcal_read_cameramodel_string() reads out of bounds on empty input
        if s.is_empty() {
            return Err(Error::CameraModelRead);
        }
        let len = i32::try_from(s.len()).map_err(|_| Error::CameraModelRead)?;
        let ptr = unsafe { sys::mrcal_read_cameramodel_string(s.as_ptr().cast(), len) };
        NonNull::new(ptr)
            .map(|ptr| Self { ptr })
            .ok_or(Error::CameraModelRead)
    }
}

impl fmt::Display for CameraModel {
    /// The `.cameramodel` file format, at full precision.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&format_cameramodel(
            &self.lensmodel(),
            self.intrinsics(),
            self.rt_cam_ref(),
            self.imagersize(),
        ))
    }
}

impl Clone for CameraModel {
    fn clone(&self) -> Self {
        self.to_string()
            .parse()
            .expect("a model's own rendering always parses")
    }
}

impl Drop for CameraModel {
    fn drop(&mut self) {
        let mut p = self.ptr.as_ptr();
        unsafe { sys::mrcal_free_cameramodel(&mut p) };
    }
}

impl fmt::Debug for CameraModel {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CameraModel")
            .field("lensmodel", &self.lensmodel())
            .field("intrinsics", &self.intrinsics())
            .field("cam_from_ref", &self.cam_from_ref())
            .field("imagersize", &self.imagersize())
            .finish()
    }
}
