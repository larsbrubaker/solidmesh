// The NaN x distances the slab test keeps: only a ray that runs along an x face plane (offset
// exactly 0, zero x direction) has that plane read as bounding nothing. A NaN x origin, a NaN x
// direction, or a box with a NaN x bound gives the answer it always gave, a miss, from every
// slab entry point (`intersection`, `intersect`, `get_closest_intersection`), front and back
// faces, for +0.0 and -0.0 directions.

use solidmesh_math::{AxisAlignedBoundingBox, IntersectionType, Ray, Vector3};

fn box_x(min_x: f64, max_x: f64) -> AxisAlignedBoundingBox {
    AxisAlignedBoundingBox::new(Vector3::new(min_x, 0.0, 0.0), Vector3::new(max_x, 1.0, 1.0))
}

/// Asserts `bounds` is missed by a ray from `origin` along `direction`, for front-face and
/// back-face rays alike.
fn assert_misses(bounds: &AxisAlignedBoundingBox, origin: Vector3, direction: Vector3) {
    for face in [IntersectionType::FrontFace, IntersectionType::BackFace] {
        let ray = Ray::with_limits(origin, direction, 0.0, f64::INFINITY, face);
        let case = format!("{bounds:?} from {origin:?} along {direction:?}, {face:?}");
        assert!(!ray.intersection(bounds), "intersection hit: {case}");
        assert!(!ray.intersect(bounds).0, "intersect hit: {case}");
        assert_eq!(
            ray.get_closest_intersection(bounds).hit_type,
            IntersectionType::None,
            "closest hit: {case}"
        );
    }
}

#[test]
fn box_with_a_nan_x_bound_touched_on_its_other_x_face_misses() {
    for direction_x in [0.0, -0.0] {
        let direction = Vector3::new(direction_x, 1.0, 0.0);
        assert_misses(
            &box_x(0.0, f64::NAN),
            Vector3::new(0.0, -2.0, 0.5),
            direction,
        );
        assert_misses(
            &box_x(f64::NAN, -3.0),
            Vector3::new(-3.0, -2.0, 0.5),
            direction,
        );
    }
}

#[test]
fn ray_with_a_nan_x_origin_misses() {
    for direction_x in [0.0, -0.0] {
        assert_misses(
            &box_x(0.0, 1.0),
            Vector3::new(f64::NAN, -2.0, 0.5),
            Vector3::new(direction_x, 1.0, 0.0),
        );
    }
}

#[test]
fn ray_on_an_x_face_with_a_nan_x_direction_misses() {
    let direction = Vector3::new(f64::NAN, 1.0, 0.0);
    let origin = Vector3::new(0.0, -2.0, 0.5);
    assert_misses(&box_x(0.0, 1.0), origin, direction);
    // Flat on x at the origin: both x offsets are exactly 0, so only the reciprocal tells a NaN
    // direction from a zero one.
    assert_misses(&box_x(0.0, 0.0), origin, direction);
}
