//! What a mixed corner's run-out bites out of the faces it runs across and turns against
//! (`Bevel/Surface/SurfaceMixedCornerBites.cs`, a static class: free functions here). Where the
//! touch row leaves the face it runs across lands here first; the bites themselves land with later
//! packages.

use vector_math::Vector3;

use super::{SurfaceBandEnd, SurfaceMixedCorner};

/// Where the run-out's TOUCH ROW - the circle of radius `2r` about the outer rail centre in the
/// cap - crosses the plane the face it runs across ends on (`ends_on`): of the two crossings, the
/// one nearer `reached_so_far`. `None` where the circle never reaches that plane, or the plane
/// lies in the cap.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x > 0)`, which refuses a NaN as well.
pub fn try_where_the_touch_row_leaves_the_face(
    corner: &SurfaceMixedCorner,
    ends_on: SurfaceBandEnd,
    reached_so_far: Vector3,
) -> Option<Vector3> {
    let centre = corner.outer_rail_centre;
    let radius = 2.0 * corner.size;
    let inward = ends_on.inward;
    let how_far = (ends_on.point - centre).dot(inward);
    let half = (radius * radius) - (how_far * how_far);

    if half < 0.0 {
        return None;
    }

    let along = inward.cross(corner.out_of_the_cap_face);

    if !(along.length() > 0.0) {
        return None;
    }

    let along = along.get_normal();

    let foot = centre + (inward * how_far);
    let reach = half.sqrt();
    let one = foot + (along * reach);
    let other = foot - (along * reach);

    Some(
        if (one - reached_so_far).length_squared() <= (other - reached_so_far).length_squared() {
            one
        } else {
            other
        },
    )
}

/// The corner the wall nobody picked ends at under the blend - where the cap face, the wall and
/// the plane the wall ends on (`wall_ends_at`, facing into the wall) meet - or `None` where the
/// wall carries on, or the three do not meet at a point.
pub fn the_corner_the_wall_ends_at(
    corner: &SurfaceMixedCorner,
    wall_ends_at: Option<SurfaceBandEnd>,
) -> Option<Vector3> {
    let wall_ends_at = wall_ends_at?;

    super::surface_mixed_corner_wall_ends::try_where_the_three_meet(
        corner.outer_rail_centre,
        corner.out_of_the_cap_face,
        corner.corner,
        corner.out_of_the_wall,
        wall_ends_at.point,
        -wall_ends_at.inward,
    )
}
