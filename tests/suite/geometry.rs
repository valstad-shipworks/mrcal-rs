use mrcal::glam::{DVec2, DVec3};
use mrcal::sys;

// The wrappers cast &[DVec2]/&[DVec3] straight to the C point types
#[test]
fn glam_types_are_layout_compatible_with_mrcal_points() {
    assert_eq!(size_of::<DVec2>(), size_of::<sys::mrcal_point2_t>());
    assert_eq!(size_of::<DVec3>(), size_of::<sys::mrcal_point3_t>());

    let v2 = DVec2::new(1.5, -2.5);
    let c2: sys::mrcal_point2_t = unsafe { std::mem::transmute(v2) };
    assert_eq!(unsafe { c2.xy }, [1.5, -2.5]);

    let v3 = DVec3::new(1.0, 2.0, 3.0);
    let c3: sys::mrcal_point3_t = unsafe { std::mem::transmute(v3) };
    assert_eq!(unsafe { c3.xyz }, [1.0, 2.0, 3.0]);
}
