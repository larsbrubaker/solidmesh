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
