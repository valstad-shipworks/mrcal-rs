use glam::{DVec2, DVec3};
use mrcal_sys as sys;

// The FFI casts &[DVec2]/&[DVec3] straight to the mrcal point types
const _: () = {
    assert!(size_of::<DVec2>() == size_of::<sys::mrcal_point2_t>());
    assert!(align_of::<DVec2>() == align_of::<sys::mrcal_point2_t>());
    assert!(size_of::<DVec3>() == size_of::<sys::mrcal_point3_t>());
    assert!(align_of::<DVec3>() == align_of::<sys::mrcal_point3_t>());
};

pub(crate) fn to_sys(v: DVec3) -> sys::mrcal_point3_t {
    sys::mrcal_point3_t { xyz: v.to_array() }
}

pub(crate) fn from_sys(p: sys::mrcal_point3_t) -> DVec3 {
    DVec3::from_array(unsafe { p.xyz })
}
