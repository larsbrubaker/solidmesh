//! The `IBvhItem` members every agg-sharp shape shares (agg-sharp
//! `RayTracer/Primitive/Shapes/BaseShape.cs`: `GetContained`, `Contains`, `GetCrossing`,
//! `GetTouching`), as functions the leaf shapes ([`super::BoxShape`], [`super::SphereShape`],
//! [`super::TriangleShape`], [`super::MinimalTriangle`], [`super::PlaneShape`]) call from their
//! [`Traceable`] impls. The color half of `BaseShape` is [`super::Primitive`].

use vector_math::{AxisAlignedBoundingBox, Plane, Vector3};

use super::{Traceable, PLANE_CROSSED_BY_ERROR};

/// C# `BaseShape.GetContained`: the shape itself when its bounds contain `sub_region`.
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

/// C# `BaseShape.Contains`: inside the bounds (with C#'s .001 slack).
pub(crate) fn leaf_contains<T: Traceable>(shape: &T, position: Vector3) -> bool {
    shape
        .get_axis_aligned_bounding_box()
        .contains_point(position)
}

/// C# `BaseShape.GetCrossing`: the shape itself when the plane crosses its bounds.
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

/// C# `BaseShape.GetTouching`: the shape itself when its bounds hold `position` within `error`.
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
