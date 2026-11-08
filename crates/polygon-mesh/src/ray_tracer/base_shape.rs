//! The BVH-item queries every leaf shape shares (collecting contained, contains, crossing and
//! touching items), as functions the leaf shapes ([`super::BoxShape`], [`super::SphereShape`],
//! [`super::TriangleShape`], [`super::MinimalTriangle`], [`super::PlaneShape`]) call from their
//! [`Traceable`] impls. The color half of a shape is [`super::Primitive`].

use solidmesh_math::{AxisAlignedBoundingBox, Plane, Vector3};

use super::{Traceable, PLANE_CROSSED_BY_ERROR};

/// The shape itself when its bounds contain `sub_region`.
pub(crate) fn leaf_get_contained<'a, T: Traceable>(
    shape: &'a T,
    results: &mut Vec<&'a dyn Traceable>,
    sub_region: &AxisAlignedBoundingBox,
) -> bool {
    if shape.get_axis_aligned_bounding_box().contains(sub_region) {
        results.push(shape);
        return true;
    }

    false
}

/// Inside the bounds (with .001 of slack, so a point on a face counts as inside).
pub(crate) fn leaf_contains<T: Traceable>(shape: &T, position: Vector3) -> bool {
    shape
        .get_axis_aligned_bounding_box()
        .contains_point(position)
}

/// The shape itself when the plane crosses its bounds.
pub(crate) fn leaf_get_crossing<'a, T: Traceable>(
    shape: &'a T,
    plane: &Plane,
) -> Vec<&'a dyn Traceable> {
    let bounds = shape.get_axis_aligned_bounding_box();
    if plane.crossed_by(&bounds, PLANE_CROSSED_BY_ERROR) {
        vec![shape]
    } else {
        Vec::new()
    }
}

/// The shape itself when its bounds hold `position` within `error`.
pub(crate) fn leaf_get_touching<T: Traceable>(
    shape: &T,
    position: Vector3,
    error: f64,
) -> Vec<&dyn Traceable> {
    let bounds = shape.get_axis_aligned_bounding_box();
    if bounds.contains_point_within(position, error) {
        vec![shape]
    } else {
        Vec::new()
    }
}
