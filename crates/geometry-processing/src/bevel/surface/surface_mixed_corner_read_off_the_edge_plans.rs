//! Reading a mixed corner's frame off the two edge plans that meet there ([`try_read`] for the
//! ordinary corner, [`try_read_carried`] where the ring is carried by another plan), with the
//! questions the reading is made of: which region the two edges share, whether the bead rests on a
//! crease of it, which side's normal and tangent line to read, and where two skew lines touch.

use solidmesh_math::Vector3;

use super::{SurfaceEdgePlan, SurfaceLine};

/// Whether the bead's profile rests on a CREASE of the face the two edges share - the one place
/// the corner's own tangency is a line the bead touches along rather than a facet it is tangent
/// to. Both halves have to hold: a bead that rests on a crease of the wall NOBODY PICKED is
/// tangent to the shared face as it always was, and what a crease there moves is only where the
/// turn ENDS, which the wall's own tangent line already says.
pub fn rests_on_the_crease_of_the_shared_face(bead: &SurfaceEdgePlan, shared_key: i32) -> bool {
    bead.wall_section
        .as_ref()
        .is_some_and(|section| section.rests_on_the_crease)
        && (if bead.left_band.is_some() {
            bead.left_region_key()
        } else {
            bead.right_region_key()
        }) == shared_key
}

/// The same question asked of a convex edge rather than of a region key: whether the bead rests
/// on a crease of the face IT and `other` share. What the cut edge's own walk has to ask
/// (`SurfaceMixedCornerCutEdge`), so that the two readings agree about which corners are crease
/// landings and which are the ordinary ones.
pub fn rests_on_the_crease_of_the_face_shared_with(
    bead: &SurfaceEdgePlan,
    other: &SurfaceEdgePlan,
) -> bool {
    try_name_the_shared_region(other, bead)
        .is_some_and(|shared_key| rests_on_the_crease_of_the_shared_face(bead, shared_key))
}

/// The one region both plans name, or `None` where they name none or both. Two edges of one
/// crease each that meet at a vertex share exactly one face; anything else is a corner this
/// reading does not describe.
pub fn try_name_the_shared_region(cut: &SurfaceEdgePlan, bead: &SurfaceEdgePlan) -> Option<i32> {
    let mut shared_key = 0;
    let mut found = 0;

    for mine in [cut.left_region_key(), cut.right_region_key()] {
        for theirs in [bead.left_region_key(), bead.right_region_key()] {
            if mine == theirs {
                shared_key = mine;
                found += 1;
            }
        }
    }

    (found == 1).then_some(shared_key)
}

/// The outward normal of the side of a planned edge that is NOT the shared region - the cap
/// face, for the cut. The shared side is not read this way: what the corner needs there is where
/// the bead really touches that face, which on a rolled one is the facet it rolled to, so it
/// comes off the bead's own tangent line instead.
pub fn outward_of_the_other_side(plan: &SurfaceEdgePlan, shared_key: i32) -> Vector3 {
    let left_is_shared = plan.left_region_key() == shared_key;

    if left_is_shared {
        plan.edge.right_normal()
    } else {
        plan.edge.left_normal()
    }
    .get_normal()
}

/// The line one side of a planned edge is walked back to: the side that IS the shared region
/// (`shared`), or the side that is not. On a rolled edge the wall's line lies in the facet the
/// profile rolled to rather than in the chain edge's own facet, which is the whole reason the
/// turn is read off it.
pub fn tangent_line_of(plan: &SurfaceEdgePlan, shared_key: i32, shared: bool) -> SurfaceLine {
    let left_is_shared = plan.left_region_key() == shared_key;

    if left_is_shared == shared {
        plan.left_tangent_line
    } else {
        plan.right_tangent_line
    }
}

/// Where two skew lines standing `apart` from each other touch - the midpoint of their common
/// perpendicular - provided that perpendicular really is `apart` long, within `tolerance`;
/// `None` where they are parallel or stand another distance apart.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > y)` on purpose: it refuses a NaN as well.
pub fn try_touch_at(
    first: SurfaceLine,
    second: SurfaceLine,
    apart: f64,
    tolerance: f64,
) -> Option<Vector3> {
    let between = second.point - first.point;
    let direction_dot = first.direction.dot(second.direction);
    let determinant = 1.0 - (direction_dot * direction_dot);

    if !(determinant.abs() > 1e-12) {
        return None;
    }

    let along_first = between.dot(first.direction);
    let along_second = between.dot(second.direction);
    let here = (along_first - (direction_dot * along_second)) / determinant;
    let there = ((direction_dot * along_first) - along_second) / determinant;

    let on_first = first.point + (first.direction * here);
    let on_second = second.point + (second.direction * there);

    if ((on_first - on_second).length() - apart).abs() > tolerance {
        return None;
    }

    Some((on_first + on_second) / 2.0)
}

/// The ordinary corner, whose ring is carried by - and touches the bead along - the convex edge's
/// own plan.
pub fn try_read(
    arriving: &SurfaceEdgePlan,
    leaving: &SurfaceEdgePlan,
    corner: Vector3,
    size: f64,
    near_enough: f64,
) -> Result<super::SurfaceMixedCorner, crate::bevel::BevelAdvisory> {
    let cut_ordinarily = if arriving.convexity() == crate::topology::EdgeConvexity::Convex {
        arriving
    } else {
        leaving
    };

    try_read_carried(
        cut_ordinarily,
        cut_ordinarily,
        None,
        arriving,
        leaving,
        corner,
        size,
        near_enough,
    )
}

/// A mixed corner read off its convex cut's and concave bead's edge plans - where the cut's ring
/// touches the bead, the bead's axis and its two feet, the cap face and the spine the ball rolls
/// round - or `SurfaceCornerNotSupported` (or `SurfaceMixedCornerBeadRingCrossesNoRail` where no
/// spine stands).
#[allow(clippy::too_many_arguments, clippy::neg_cmp_op_on_partial_ord)]
pub fn try_read_carried(
    ring_carried_by: &SurfaceEdgePlan,
    touching_the_bead: &SurfaceEdgePlan,
    ring_stops_at: Option<super::SurfaceBandEnd>,
    arriving: &SurfaceEdgePlan,
    leaving: &SurfaceEdgePlan,
    corner: Vector3,
    size: f64,
    near_enough: f64,
) -> Result<super::SurfaceMixedCorner, crate::bevel::BevelAdvisory> {
    use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};
    use crate::topology::EdgeConvexity;

    use super::{SurfaceBandEnd, SurfaceMixedCorner, SurfaceMixedCornerSpine};

    let not_supported = || {
        BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceCornerNotSupported,
            vec![corner.into()],
        )
    };

    let cut = ring_carried_by;
    let bead = if arriving.convexity() == EdgeConvexity::Convex {
        leaving
    } else {
        arriving
    };
    if cut.convexity() != EdgeConvexity::Convex || bead.convexity() != EdgeConvexity::Concave {
        return Err(not_supported());
    }

    if !cut.has_center_line || !bead.has_center_line {
        return Err(not_supported());
    }

    let Some(shared_key) = try_name_the_shared_region(cut, bead) else {
        return Err(not_supported());
    };

    if !touching_the_bead.has_center_line {
        return Err(not_supported());
    }

    let on_the_crease = rests_on_the_crease_of_the_shared_face(bead, shared_key);
    let touched = if on_the_crease {
        bead.wall_section.as_ref().and_then(|section| {
            try_touch_at(
                tangent_line_of(touching_the_bead, shared_key, true),
                section.crease_line,
                0.0,
                near_enough,
            )
        })
    } else {
        try_touch_at(
            touching_the_bead.center_line,
            bead.center_line,
            2.0 * size,
            near_enough,
        )
    };
    let Some(tangent_point) = touched else {
        return Err(not_supported());
    };

    let along_the_axis = bead.center_line.direction;
    let along_the_axis_point = bead.center_line.closest_point_to(corner);
    let shared_foot =
        tangent_line_of(bead, shared_key, true).closest_point_to(along_the_axis_point);
    let out_of_the_axis_to_the_shared_face = shared_foot - along_the_axis_point;
    if (out_of_the_axis_to_the_shared_face.length() - size).abs() > near_enough {
        return Err(not_supported());
    }

    let towards_the_shared_face = out_of_the_axis_to_the_shared_face.get_normal();
    if (tangent_point
        - bead.center_line.closest_point_to(tangent_point)
        - (towards_the_shared_face * size))
        .length()
        > near_enough
    {
        return Err(not_supported());
    }

    let out_of_the_cap_face = outward_of_the_other_side(cut, shared_key);
    let away_along_the_cut = SurfaceMixedCorner::away_from_the_corner(touching_the_bead, corner);
    let away_along_the_bead = SurfaceMixedCorner::away_from_the_corner(bead, corner);
    let wall_foot = tangent_line_of(bead, shared_key, false).closest_point_to(along_the_axis_point);
    let out_of_the_axis_to_the_wall = wall_foot - along_the_axis_point;
    if (out_of_the_axis_to_the_wall.length() - size).abs() > near_enough {
        return Err(not_supported());
    }

    let towards_the_wall = out_of_the_axis_to_the_wall.get_normal();
    let out_of_the_wall = -towards_the_wall;
    let square = towards_the_wall
        - (towards_the_shared_face * towards_the_wall.dot(towards_the_shared_face));
    if !(square.length_squared() > 0.0) {
        return Err(not_supported());
    }

    let turn_radians = libm::acos(
        towards_the_shared_face
            .dot(towards_the_wall)
            .clamp(-1.0, 1.0),
    );
    let along_out_of_the_cap = along_the_axis.dot(out_of_the_cap_face);
    if !(along_out_of_the_cap.abs() > SurfaceMixedCorner::SQUARE_ENOUGH) {
        return Err(not_supported());
    }

    let to_the_cap =
        -(bead.center_line.point - corner).dot(out_of_the_cap_face) / along_out_of_the_cap;
    let outer_rail_centre = bead.center_line.point + (along_the_axis * to_the_cap);
    if (outer_rail_centre - corner).length() > (2.0 * size) + near_enough {
        return Err(not_supported());
    }

    let Some(spine) = SurfaceMixedCornerSpine::try_create(
        bead.center_line,
        outer_rail_centre,
        out_of_the_cap_face,
        along_out_of_the_cap,
        false,
        towards_the_shared_face,
        square.get_normal(),
        size,
        turn_radians,
        away_along_the_bead,
        SurfaceMixedCorner::SQUARE_ENOUGH,
    ) else {
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceMixedCornerBeadRingCrossesNoRail,
            vec![corner.into()],
        ));
    };

    Ok(SurfaceMixedCorner::new(
        corner,
        size,
        shared_key,
        cut.clone(),
        bead.clone(),
        tangent_point,
        ring_stops_at.unwrap_or_else(|| SurfaceBandEnd::new(tangent_point, away_along_the_cut)),
        SurfaceBandEnd::new(tangent_point, away_along_the_bead),
        spine,
        outer_rail_centre,
        out_of_the_cap_face,
        out_of_the_wall,
        ring_stops_at.is_some(),
    ))
}
