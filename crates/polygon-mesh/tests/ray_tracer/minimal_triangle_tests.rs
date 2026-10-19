//! `MinimalTriangle`, the shape meshes are picked with: its bounding box holds every corner.

use std::sync::Arc;

use polygon_mesh::ray_tracer::{MinimalTriangle, Traceable};
use vector_math::{Ray, Vector3, Vector3Float};

/// A triangle's box has to hold its own corners. It used to be kept as a float centre and half
/// size, and centre - halfSize rounds: a cylinder cap triangle with a corner at x = -3.8e-15 and
/// another at 0.98 got a box starting at exactly 0, so a ray straight down x = 0 - inside the
/// triangle - missed the box and was never tested against the triangle.
#[test]
fn the_box_holds_every_corner() {
    let corners = [
        Vector3Float::new_f32(0.0, 0.0, 5.0),
        Vector3Float::new_f32(0.980_171_4, 9.951_847, 5.0),
        Vector3Float::new_f32(-3.828_568_6E-15, 10.0, 5.0),
    ];
    let triangle = MinimalTriangle::new(Arc::new(move |_face, vertex| corners[vertex as usize]), 0);

    let bounds = triangle.get_axis_aligned_bounding_box();
    for corner in corners {
        let corner = Vector3::from_vector3_float(corner);
        assert!(
            bounds.min_xyz.x <= corner.x
                && bounds.min_xyz.y <= corner.y
                && bounds.min_xyz.z <= corner.z
        );
        assert!(
            bounds.max_xyz.x >= corner.x
                && bounds.max_xyz.y >= corner.y
                && bounds.max_xyz.z >= corner.z
        );
    }

    let mut ray = Ray::new(Vector3::new(0.0, 9.9, 15.0), -Vector3::UNIT_Z);
    let hit = triangle.get_closest_intersection(&mut ray);
    assert!(hit.is_some());
}

/// A hit outside the ray's distance window is no hit: the 3D view's section plane narrows a
/// pick ray to the stretch past its cut, and a triangle before it (cut away on screen) must not
/// be picked.
#[test]
fn a_hit_outside_the_rays_distance_window_is_no_hit() {
    let corners = [
        Vector3Float::new_f32(-5.0, -5.0, 0.0),
        Vector3Float::new_f32(5.0, -5.0, 0.0),
        Vector3Float::new_f32(0.0, 5.0, 0.0),
    ];
    let triangle = MinimalTriangle::new(Arc::new(move |_face, vertex| corners[vertex as usize]), 0);
    let down = -Vector3::UNIT_Z;
    let start = Vector3::new(0.0, 0.0, 10.0);
    let window = |min: f64, max: f64| {
        let mut ray = Ray::with_limits(start, down, min, max, vector_math::IntersectionType::Both);
        triangle
            .get_closest_intersection(&mut ray)
            .map(|hit| hit.distance_to_hit)
    };
    assert_eq!(window(0.0, f64::INFINITY), Some(10.0));
    assert_eq!(window(9.0, 11.0), Some(10.0));
    assert_eq!(
        window(10.5, f64::INFINITY),
        None,
        "the hit is before the window"
    );
    assert_eq!(window(0.0, 9.5), None, "the hit is past the window");
}
