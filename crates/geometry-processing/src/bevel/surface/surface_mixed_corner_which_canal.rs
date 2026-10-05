//! Which canal a leaning mixed corner's blend is - the cap-riding one or the DUAL - decided by
//! CREASE REACH and never by the lean (`Bevel/Surface/SurfaceMixedCornerWhichCanal.cs`, a static
//! class: free functions here). The cap-riding crease and how far a crease runs land here first;
//! the dual's own crease reading (`TheCornerTakesTheDual`, `TryHowFarTheDualsCreaseRuns`) lands
//! with a later package.

use super::{SurfaceLine, SurfaceMixedCorner};

/// How far off the crease's own line a rim edge's end may stand and still be read as ON it - the
/// part's ruler, as everywhere else a position is compared here.
const ON_THE_CREASE: f64 = 1e-6;

/// How far the crease runs the way the CAP-RIDING run-out travels, which on an L prism is the
/// convex rim the square corner's run-out already dies on; `None` where there is no such crease.
pub fn try_how_far_the_cap_riding_crease_runs(corner: &SurfaceMixedCorner) -> Option<f64> {
    let crease = try_the_cap_riding_crease(corner)?;

    try_how_far_it_runs(corner, crease)
}

/// The crease the CAP-RIDING blend runs out onto, from the corner outwards: the cap's plane met
/// with the wall's, signed by where the ball's contact on the CAP stands at the end of the turn.
/// `None` where the two faces do not meet in a line.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x > 0)`, which refuses a NaN as well.
pub fn try_the_cap_riding_crease(corner: &SurfaceMixedCorner) -> Option<SurfaceLine> {
    let along = corner.out_of_the_cap_face.cross(corner.out_of_the_wall);

    if !(along.length_squared() > 0.0) {
        return None;
    }

    let along = along.get_normal();

    let outwards = corner.spine.t_contact_at(corner.turn_radians()) - corner.corner;

    Some(SurfaceLine::new(
        corner.corner,
        if outwards.dot(along) >= 0.0 {
            along
        } else {
            -along
        },
    ))
}

/// How far the furthest end of the WALL region's boundary edges lying on `crease` stands from
/// the corner, the way that crease runs; `None` where none reaches out past the corner.
///
/// Off the wall region's own rim: where the cap and the wall really are adjacent the crease is a
/// boundary edge of that region. Taken over ALL of them and not the first, because a boolean can
/// leave one crease cut into several collinear pieces; and SIGNED, because an edge on the crease's
/// own line but running the OTHER way out of the corner belongs to the other canal.
pub fn try_how_far_it_runs(corner: &SurfaceMixedCorner, crease: SurfaceLine) -> Option<f64> {
    let mut reach = 0.0;

    let wall = if corner.bead.left_region_key() == corner.shared_region_key {
        &corner.bead.right_region
    } else {
        &corner.bead.left_region
    };

    for edge in wall.boundary_edges() {
        if crease.distance_to(edge.start) > ON_THE_CREASE
            || crease.distance_to(edge.end) > ON_THE_CREASE
        {
            continue;
        }

        reach = f64::max(
            reach,
            f64::max(
                (edge.start - corner.corner).dot(crease.direction),
                (edge.end - corner.corner).dot(crease.direction),
            ),
        );
    }

    (reach > 0.0).then_some(reach)
}
