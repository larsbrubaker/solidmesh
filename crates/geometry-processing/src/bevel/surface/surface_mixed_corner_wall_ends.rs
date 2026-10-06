//! Where the wall nobody picked runs OUT under a mixed corner's blend
//! (`Bevel/Surface/SurfaceMixedCornerWallEnds.cs`, a static class: free functions here). The
//! questions the reading is made of land here first - whether what stands past the wall is one
//! plane, where a rim edge crosses the travel line, and the turn at which a row of the runout
//! crosses the plane the wall ends on - and the turn the run-out's foot crosses it at
//! ([`try_where_the_foot_crosses`]) - and the reading itself ([`try_read`]).

use std::f64::consts::PI;

use vector_math::Vector3;

use super::surface_mixed_corner_run_out;
use super::{SurfaceBandEnd, SurfaceBevelPlan, SurfaceMixedCorner, SurfaceMixedCornerWall};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};
use crate::topology::{FaceGroup, FaceRegionCache, FeatureEdge};

/// How far from the travel line a rim edge's own crossing may stand and still be read as on it -
/// the part's ruler, as everywhere else a position is compared here.
const OFF_THE_LINE: f64 = 1e-6;

/// How far along the turn the travel direction is read over: a hundredth of a radian past the end
/// of the bead, which names the way the runout goes without asking where it ends.
const THE_WAY_IT_LEANS: f64 = 0.01;

/// Whether what stands past the wall's rim, on `face_past_it`, is ONE plane facing `facing` that
/// the blend can be trimmed in. Read of the REGION and not of the one facet, so a region that
/// bends away past its first facet is not cut by a plane only that facet is in. The wall's own
/// next facet is one plane by construction (the wall has ended by bending away); the face the
/// blend itself stands on is refused, because the same pick is walking it back.
pub fn the_material_past_it_is_one_plane(
    regions: Option<&FaceRegionCache>,
    corner: &SurfaceMixedCorner,
    wall: &FaceGroup,
    face_past_it: i32,
    facing: Vector3,
) -> bool {
    if face_past_it < 0 {
        return false;
    }

    if wall.faces().contains(&face_past_it) {
        // The wall's OWN next facet: the travel line runs straight while the wall curves away
        // from it, so what ends the wall here is its own bend and the plane is that facet's.
        return true;
    }

    // C# `regions.Region(...)` on a null cache would throw; no cache reads as no region.
    let Some(region) = regions.and_then(|r| r.region(face_past_it)) else {
        return false;
    };

    if SurfaceBevelPlan::key_of(&region) == corner.shared_region_key {
        // The face the blend itself stands on, which the same pick is walking back: a section
        // bitten into it now stands on material that moves before the build is finished.
        return false;
    }

    it_is_all_one_plane(regions, Some(&region), facing)
}

/// Whether every facet of a region stands in the one plane `facing` names. `None` for either the
/// cache or the region (C#'s nulls) is not one plane.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x > 0)`, which refuses a NaN as well.
pub fn it_is_all_one_plane(
    regions: Option<&FaceRegionCache>,
    region: Option<&FaceGroup>,
    facing: Vector3,
) -> bool {
    let (Some(regions), Some(region)) = (regions, region) else {
        return false;
    };
    if !(facing.length_squared() > 0.0) {
        return false;
    }

    let mesh = regions.mesh();
    let named = facing.get_normal();

    for &face in region.faces() {
        let normal = Vector3::from_vector3_float(mesh.faces[face as usize].normal);

        if !(normal.length_squared() > 0.0)
            || normal.get_normal().dot(named) < FeatureEdge::COPLANAR_FACE_DOT
        {
            return false;
        }
    }

    true
}

/// The turn at which one row of the runout's seed (`at`) crosses the plane the wall ends on,
/// measured from the seed itself; `f64::INFINITY` where that row's own circle never reaches the
/// plane at all, which is a row the wall swallows before the plane could have cut it.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x > 0)`, which refuses a NaN as well.
pub fn where_it_crosses(corner: &SurfaceMixedCorner, at: Vector3, across: SurfaceBandEnd) -> f64 {
    if !(across.signed_distance(at) > 0.0) {
        // Already past the plane where it starts, so there is no turn at which it LEAVES the
        // material: the first root ahead is where its circle comes back INTO the material, and a
        // column there would put the row over material the part does not carry.
        return f64::INFINITY;
    }

    try_turn_onto(corner, at, across, 0.0).unwrap_or(f64::INFINITY)
}

/// The turn at which `at`, carried round the bead's axis, crosses `across` - the first one at or
/// after `after`. Its circle's own frame makes this one arc cosine rather than a search.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x > 0)`, which refuses a NaN as well.
pub fn try_turn_onto(
    corner: &SurfaceMixedCorner,
    at: Vector3,
    across: SurfaceBandEnd,
    after: f64,
) -> Option<f64> {
    // Whether a root has been taken is the Option, rather than reading a turn of 0 as none: a row
    // standing ON the plane at the seed crosses AT nought and would otherwise read as never.
    let mut turn: Option<f64> = None;

    let facing = -across.inward;
    let axis = corner.axis();
    let centre = axis.closest_point_to(at);
    let first = at - centre;

    // A hair of the corner's own sweep, squared up to the axis: it says which WAY the turn goes,
    // which a cross product's sign cannot be asked for on its own.
    let mut nudged = corner.turned_about(at, THE_WAY_IT_LEANS) - at;

    nudged = nudged - (axis.direction * nudged.dot(axis.direction));

    if !(nudged.length_squared() > 0.0) || !(first.length_squared() > 0.0) {
        return None;
    }

    // And the quarter turn on is the CROSS PRODUCT, not that hair's own direction: the chord of a
    // hundredth of a radian leans half of that off the tangent, which would land the row up to a
    // ruler PAST the plane. Square to the axis and to the row's own radius, so it needs no
    // normalising.
    let mut second = axis.direction.cross(first);

    if second.dot(nudged) < 0.0 {
        second = -second;
    }

    let stand_off = (centre - across.point).dot(facing);
    let along_first = first.dot(facing);
    let along_second = second.dot(facing);
    let amplitude = ((along_first * along_first) + (along_second * along_second)).sqrt();

    if !(amplitude > 0.0) || stand_off.abs() > amplitude {
        return None;
    }

    let phase = f64::atan2(along_second, along_first);
    let spread = (-stand_off / amplitude).clamp(-1.0, 1.0).acos();

    for root in [phase + spread, phase - spread] {
        let mut candidate = root;

        while candidate < after {
            candidate += 2.0 * PI;
        }

        if turn.is_none_or(|taken| candidate < taken) {
            turn = Some(candidate);
        }
    }

    turn
}

/// Where one rim edge crosses the travel line (from `from` along unit `travel`), as
/// `(how far along the line, the outward normal of the face standing across it, that face)`: the
/// face of the two on that edge that is not the wall the line runs along. `None` where the edge
/// does not cross the line ahead, runs along it, or its face across is one the line is leaving.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x > y)`, which refuses a NaN as well.
pub fn try_crossing(
    edge: &FeatureEdge,
    from: Vector3,
    travel: Vector3,
) -> Option<(f64, Vector3, i32)> {
    let along = edge.end - edge.start;

    if !(along.length_squared() > 0.0) {
        return None;
    }

    // The edge's own nearest point to the line, and the line's to it: a rim edge that really
    // stops the runout crosses the line, so the two stand together there.
    let between = edge.start - from;
    let along_travel = along.dot(travel);
    let determinant = along.length_squared() - (along_travel * along_travel);

    if !(determinant.abs() > 0.0) {
        return None;
    }

    // An edge the travel runs ALONG is the rim it runs down and not the end of it. The determinant
    // is the square of the sine between them, which float storage leaves tiny rather than zero,
    // and the parameter it gives is noise - so the sine itself is asked, against the part's own
    // ruler read as an angle.
    if !(determinant > along.length_squared() * OFF_THE_LINE * OFF_THE_LINE) {
        return None;
    }

    let on_the_edge = ((between.dot(travel) * along_travel) - between.dot(along)) / determinant;

    if !(on_the_edge > -OFF_THE_LINE) || !(on_the_edge < 1.0 + OFF_THE_LINE) {
        return None;
    }

    let at = edge.start + (along * on_the_edge);
    let reach = (at - from).dot(travel);

    if !(reach > OFF_THE_LINE) || (at - (from + (travel * reach))).length() > OFF_THE_LINE {
        return None;
    }

    // The face ACROSS: the one whose own plane the travel line runs into rather than along.
    let first = edge.normal0.get_normal();
    let second = edge.normal1.get_normal();

    let (normal, face) = if first.dot(travel).abs() > second.dot(travel).abs() {
        (first, edge.face0)
    } else {
        (second, edge.face1)
    };

    if !(normal.dot(travel) > 0.0) {
        // The material of the face across stands on the near side of it, so the line runs INTO
        // it; a normal pointing the other way is a face the runout is leaving rather than meeting.
        return None;
    }

    Some((reach, normal, face))
}

/// Whether `at` stands in the material the plane across the wall's end keeps.
pub fn stands_in_the_material(across: SurfaceBandEnd, at: Vector3) -> bool {
    across.signed_distance(at) > 0.0
}

/// Where three planes (each a point and its normal) meet, or `None` where two of them are
/// parallel.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x > y)`, which refuses a NaN as well.
pub fn try_where_the_three_meet(
    first_point: Vector3,
    first_normal: Vector3,
    second_point: Vector3,
    second_normal: Vector3,
    third_point: Vector3,
    third_normal: Vector3,
) -> Option<Vector3> {
    let determinant = first_normal.dot(second_normal.cross(third_normal));

    if !(determinant.abs() > 1e-9) {
        return None;
    }

    Some(
        ((second_normal.cross(third_normal) * first_point.dot(first_normal))
            + (third_normal.cross(first_normal) * second_point.dot(second_normal))
            + (first_normal.cross(second_normal) * third_point.dot(third_normal)))
            / determinant,
    )
}

/// C# `TryWhereTheFootCrosses`: the turn at which the run-out's FOOT (its scallop point on the
/// wall) crosses `across` and leaves the material, or `None` where it never does: a wall that does
/// not run out under this blend after all. The curve is the meridian arc met with the wall facet by
/// facet, so it is walked to the crossing between the bead's own end and `clears` (the turn the
/// rail would have cleared the wall at), halving sixty times, which is the double's own precision
/// on a part of any size.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x > 0)`, which refuses a NaN as well.
pub fn try_where_the_foot_crosses(
    corner: &SurfaceMixedCorner,
    wall: &SurfaceMixedCornerWall,
    across: SurfaceBandEnd,
    clears: f64,
) -> Option<f64> {
    let inside = surface_mixed_corner_run_out::try_scallop_point(corner, wall, 0.0)?;

    if !(across.signed_distance(inside) > 0.0) {
        return None;
    }

    let outside = surface_mixed_corner_run_out::try_scallop_point(corner, wall, clears)?;

    if across.signed_distance(outside) > 0.0 {
        return None;
    }

    let mut low = 0.0;
    let mut high = clears;

    for _ in 0..60 {
        let middle = (low + high) / 2.0;
        let at = surface_mixed_corner_run_out::try_scallop_point(corner, wall, middle)?;

        if across.signed_distance(at) > 0.0 {
            low = middle;
        } else {
            high = middle;
        }
    }

    Some((low + high) / 2.0)
}

/// C# `TryRead`: the plane the blend is trimmed by where the wall nobody picked runs out under it -
/// the face across the wall's rim, as the half space its material is on - or `Ok(None)` where that
/// wall carries on as far as the runout goes (the ordinary corner; also where there are no
/// `regions`, C#'s null). `Err` only where the wall DOES run out under the blend and what stands
/// past it is not one plane the trim can be taken in.
///
/// Read from the region's own rim: the runout travels along the line the wall and the cap meet in,
/// so the question is which boundary edge of the wall's region that line crosses first.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x > 0)`, which refuses a NaN as well.
pub fn try_read(
    regions: Option<&FaceRegionCache>,
    corner: &SurfaceMixedCorner,
) -> Result<Option<SurfaceBandEnd>, BevelAdvisory> {
    if regions.is_none() {
        return Ok(None);
    }

    // The wall nobody picked is the bead's other region - the one the two edges do not share.
    let wall = if corner.bead.left_region_key() == corner.shared_region_key {
        &corner.bead.right_region
    } else {
        &corner.bead.left_region
    };

    // The runout travels along the line the wall and the cap meet in, from the bead's own foot on
    // the wall outward - which is where every one of its stations' touch points stands.
    let from = corner.cap_corner();
    let mut travel = corner.outer_rail_point(corner.turn_radians() + THE_WAY_IT_LEANS)
        - corner.outer_rail_point(corner.turn_radians());

    travel -= corner.out_of_the_wall * travel.dot(corner.out_of_the_wall);
    travel -= corner.out_of_the_cap_face * travel.dot(corner.out_of_the_cap_face);

    if !(travel.length_squared() > 0.0) {
        return Ok(None);
    }

    let travel = travel.get_normal();

    let mut nearest = f64::MAX;
    let mut face_past_it = -1;
    let mut facing = Vector3::ZERO;
    let mut across = None;

    for edge in wall.boundary_edges() {
        let Some((how_far, normal, face)) = try_crossing(edge, from, travel) else {
            continue;
        };

        if how_far >= nearest {
            continue;
        }

        nearest = how_far;
        face_past_it = face;
        facing = normal;

        // Named by the point the travel line crosses at rather than by a corner of the edge, so the
        // plane is the same one however the rim happens to be split into edges.
        across = Some(SurfaceBandEnd::new(from + (travel * how_far), -normal));
    }

    let Some(across) = across else {
        // The wall carries on as far as the runout goes, which is the ordinary corner.
        return Ok(None);
    };

    // The blend is trimmed by ONE PLANE there and the section is bitten out of it, so the material
    // past that rim has to BE one plane, and one this pick is not blending as well.
    if !the_material_past_it_is_one_plane(regions, corner, wall, face_past_it, facing) {
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceCornerNotSupported,
            vec![corner.corner.into()],
        ));
    }

    Ok(Some(across))
}

/// C# `SurfaceMixedCornerWallEnds.TryBiteTheSection`: the wall's section between the cap, the
/// wall and the band end `across` bitten out one triangle at a time - each pair of section
/// stations with the point the three planes meet at. Refused for a section of fewer than two
/// stations or planes that do not meet at a point.
pub fn try_bite_the_section(
    pool: &mut super::SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    corner: &super::SurfaceMixedCorner,
    across: super::SurfaceBandEnd,
    section: &[i32],
    split_from: i32,
) -> Result<(), crate::bevel::BevelAdvisory> {
    let not_supported = || {
        crate::bevel::BevelAdvisory::new(
            crate::bevel::BevelAdvisoryCode::SurfaceCornerNotSupported,
            vec![corner.corner.into()],
        )
    };

    if section.len() < 2 {
        return Err(not_supported());
    }

    let facing = -across.inward;
    let Some(at_the_corner) = try_where_the_three_meet(
        corner.outer_rail_centre,
        corner.out_of_the_cap_face,
        corner.corner,
        corner.out_of_the_wall,
        across.point,
        facing,
    ) else {
        return Err(not_supported());
    };

    let corners = pool.add(at_the_corner);
    for pair in section.windows(2) {
        if pair[0] == corners || pair[1] == corners {
            continue;
        }

        let rim = [
            at_the_corner,
            pool.position_of(pair[0]),
            pool.position_of(pair[1]),
        ];
        let snap_to = [corners, pair[0], pair[1]];
        super::surface_convex_bite::try_take(
            pool,
            faces,
            &rim,
            facing,
            across.point,
            &snap_to,
            split_from,
        )?;
    }

    Ok(())
}
