//! The two SIDES of the section a chain end's notch takes out of the face beyond its corner, as the
//! room reads them. A notch's reach is DIRECTIONAL: the end plane is rarely square to the chain, so
//! the section leans to one side of the corner, and the two sides are what bound it.

use vector_math::Vector3;

use super::{surface_mixed_corner_wall_ends, SurfaceEdgePlan, SurfaceLine};
use crate::topology::{FaceGroup, FaceRegionCache};

/// How near parallel to the plane a tangent line may run before it is read as never crossing it.
const PARALLEL_DOT: f64 = 1e-9;

/// How far past the notch's own omnidirectional reach a section side may stand before it is read
/// as a crossing off at the far end of the arithmetic rather than a side.
const THE_FURTHEST_A_SECTION_MAY_REACH: f64 = 10.0;

/// The two sides of the section `edge`'s notch at `corner` takes out of the region `beyond`,
/// relative to the corner and per unit of `size`: where `edge`'s left and right tangent lines
/// cross the plane `beyond` stands in. `None` where `beyond` is not one plane (the
/// omnidirectional reach `reach_per_size` is kept), or a side cannot be named.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)` on purpose: it refuses a NaN as well.
pub fn try_the_sides(
    edge: &SurfaceEdgePlan,
    corner: Vector3,
    regions: Option<&FaceRegionCache>,
    beyond: Option<&FaceGroup>,
    size: f64,
    reach_per_size: f64,
) -> Option<(Vector3, Vector3)> {
    let (regions, beyond) = (regions?, beyond?);
    if beyond.faces().is_empty() || !(size > 0.0) || !(reach_per_size > 0.0) {
        return None;
    }

    let mesh = regions.mesh();
    let seed = beyond.faces()[0] as usize;
    let normal = Vector3::from_vector3_float(mesh.faces[seed].normal);

    if !(normal.length_squared() > 0.0) {
        return None;
    }

    let normal = normal.get_normal();

    if !surface_mixed_corner_wall_ends::it_is_all_one_plane(Some(regions), Some(beyond), normal) {
        // Not one plane, so there is no one plane to cut the tangent lines with and no section
        // to bound. The omnidirectional reach is kept, which is the reading this replaces.
        return None;
    }

    let on_the_plane = Vector3::from_vector3_float(mesh.vertices[mesh.faces[seed].v0 as usize]);

    let first = try_side(
        edge.left_tangent_line,
        corner,
        on_the_plane,
        normal,
        size,
        reach_per_size,
    )?;
    let second = try_side(
        edge.right_tangent_line,
        corner,
        on_the_plane,
        normal,
        size,
        reach_per_size,
    )?;

    Some((first, second))
}

/// Where one tangent line crosses the plane through `on_the_plane` with `normal`, relative to
/// `corner` and per unit of `size`; `None` where it runs along the plane or crosses it further
/// out than any section reaches.
pub fn try_side(
    tangent: SurfaceLine,
    corner: Vector3,
    on_the_plane: Vector3,
    normal: Vector3,
    size: f64,
    reach_per_size: f64,
) -> Option<Vector3> {
    let direction = tangent.direction;
    let denominator = direction.dot(normal);

    if denominator.abs() < PARALLEL_DOT {
        return None;
    }

    let along = (on_the_plane - tangent.point).dot(normal) / denominator;
    let crossing = tangent.point + (direction * along);

    let side = (crossing - corner) / size;

    (side.length() <= reach_per_size * THE_FURTHEST_A_SECTION_MAY_REACH).then_some(side)
}
