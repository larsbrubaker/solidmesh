//! What a mixed corner's run-out bites out of the faces it runs across and turns against
//! (`Bevel/Surface/SurfaceMixedCornerBites.cs`, a static class: free functions here). Where the
//! touch row leaves the face it runs across, the point under the cap ([`under_the_cap`]), and the
//! two bites themselves: the cap face's sectors ([`try_bite_the_cap_face`], each a [`try_sector`])
//! and the wall's strip and scallop ([`try_bite_the_wall`]).

use vector_math::Vector3;

use super::{
    surface_mixed_corner_sector, SurfaceBandEnd, SurfaceMixedCorner, SurfaceMixedCornerFaces,
    SurfaceVertexPool,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

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

/// C# `UnderTheCap`: the point on the cap's own plane straight above `at`, which stays in the wall
/// - carried along the axis, measured against the corner vertex.
pub fn under_the_cap(corner: &SurfaceMixedCorner, at: Vector3) -> Vector3 {
    corner.up_into_the_cap_through(at, corner.corner)
}

/// C# `SurfaceMixedCornerBites.TrySector`: the cap face's sector from `apex` round `stations`
/// bitten out (snapped to the cap corner and the stations); nothing with fewer than two stations.
#[allow(clippy::too_many_arguments)]
pub fn try_sector(
    pool: &mut super::SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    corner: &SurfaceMixedCorner,
    stations: &[i32],
    cap_corner: i32,
    apex: Vector3,
    split_from: i32,
) -> Result<(), crate::bevel::BevelAdvisory> {
    if stations.len() < 2 {
        return Ok(());
    }

    let mut rim = Vec::with_capacity(stations.len() + 1);
    rim.push(apex);
    let mut snap_to = Vec::with_capacity(stations.len() + 1);
    snap_to.push(cap_corner);
    for &vertex in stations {
        rim.push(pool.position_of(vertex));
        snap_to.push(vertex);
    }

    super::surface_convex_bite::try_take(
        pool,
        faces,
        &rim,
        corner.out_of_the_cap_face,
        corner.corner,
        &snap_to,
        split_from,
    )
}

/// C# `TryBiteTheCapFace`: the cap face gives up the SECTOR the ball's touch point traces across
/// it, all the way round - through the patch's own turn and on through the runout, to the point on
/// the sharp edge where the ball clears the wall. Its apex is the bead's axis, which stands in the
/// void rather than on the face: the bite is read as the half planes its sides name, so an apex off
/// the material costs it nothing.
///
/// TWO bites, split at the turn the bead's rim ends at, and not one sector: a convex bite is the
/// intersection of the half planes its sides name, which is the ring itself only while the ring is
/// convex - so a sector may not reach half a turn. The patch's own is `180 - wedge` and the
/// runout's is sixty degrees, each safely under; together they pass half a turn as soon as the
/// void wedge is under sixty. Split at the station they share, the two bites meet along one radius
/// named by the same two points in both, so nothing is left standing between them.
#[allow(clippy::too_many_arguments)]
pub fn try_bite_the_cap_face(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    corner: &SurfaceMixedCorner,
    grid: &[Vec<i32>],
    run_out: &[Vec<i32>],
    the_corner: Option<Vector3>,
    wall_ends_at: Option<SurfaceBandEnd>,
    the_die_point: i32,
    the_sector_runs_to: i32,
    the_cap_exit: i32,
    split_from: i32,
) -> Result<(), BevelAdvisory> {
    let cap_corner = pool.add(corner.cap_corner());

    // The blend's own sector, from the handover round to where the bead's rim ends. Its apex is
    // the axis wherever the ring stands in a plane through the axis, and stands off it where the
    // ring LEANS, so that the wedge between the ring and the plane it was cut at goes to the
    // sector rather than standing under the blend (SurfaceMixedCornerSector).
    let blend = grid[0].clone();

    let apex = surface_mixed_corner_sector::try_apex(
        corner,
        pool.position_of(blend[0]),
        pool.position_of(blend[blend.len() - 1]),
    )?;
    try_sector(pool, faces, corner, &blend, cap_corner, apex, split_from)?;

    // And the runout's, from there round to the point on the sharp edge. Its first station is the
    // blend's last, so the two share that radius rather than each drawing one of their own.
    let mut past = vec![grid[0][grid[0].len() - 1]];

    for (station, column) in run_out.iter().enumerate().skip(1) {
        if station as i64 > the_sector_runs_to as i64 {
            // Past the CAP EXIT, which is where the run-out's touch row ENDS. The columns beyond
            // it stand in the lens and are topped by the band's own far side, under the cap rather
            // than on it, so there is no touch point out there for this rim to visit at all - and
            // the exit itself, which stands at no station, is added below.
            continue;
        }

        let touching = pool.position_of(column[0]);

        if ((touching - corner.outer_rail_centre).dot(corner.out_of_the_cap_face)).abs()
            > pool.near_enough()
        {
            // The ball's touch point has left the cap - where the wall RAN OUT under the blend and
            // the plane across it cut the outer rail off (SurfaceMixedCornerWallEnds). What the
            // cap still owes there is the part's own corner, where the cap, the wall and that
            // plane meet, which stands inside the ball's own reach and so under the blend.
            if let Some(the_corner) = the_corner {
                past.push(pool.add(the_corner));
            }

            break;
        }

        past.push(column[0]);
    }

    // And the CAP EXIT, which is where the touch row itself ends: it stands between two stations
    // rather than at one, so no column's own top is it, and the rim would otherwise run from the
    // last touch point straight to whatever closes it and leave the wedge between to nobody.
    if the_cap_exit >= 0 && past.last() != Some(&the_cap_exit) {
        past.push(the_cap_exit);
    }

    // And where a band ENDED the run-out short of the sharp edge, the rim closes on the point it
    // would have died at rather than on the last touch point: the cap between the two is under the
    // torus that was removed either way.
    if the_die_point >= 0 && past.last() != Some(&the_die_point) {
        past.push(the_die_point);

        // And where that point stands ON the plane the face ends at, the rim follows the face's
        // own edge from there to the part's corner rather than closing straight back across it.
        if let (Some(the_corner), Some(wall_ends_at)) = (the_corner, wall_ends_at) {
            if wall_ends_at
                .signed_distance(pool.position_of(the_die_point))
                .abs()
                <= pool.near_enough()
            {
                past.push(pool.add(the_corner));
            }
        }
    }

    try_sector(
        pool,
        faces,
        corner,
        &past,
        cap_corner,
        corner.outer_rail_centre,
        split_from,
    )
}

/// C# `TryBiteTheWall`: the wall gives up the strip the bead's stop plane left standing at the
/// corner - between the concave edge, the cap face, the meridian plane at the last turn and that
/// stop plane - and then the SCALLOP the runout takes out of it, one trapezoid per station between
/// the trim chord and the cap.
///
/// Facet by facet, because a wall the bead ROLLED along is not one plane and a bite finds only the
/// faces lying in the one plane it is written in; a flat wall is one facet and gets exactly the
/// square it always did. Trapezoid by trapezoid, because the scallop's own curve is concave: the
/// region between it and the cap is not convex, but between two stations it IS - two trim points
/// and the two points above them on the sharp edge - and both ends of any one of them stand on ONE
/// facet, because a station is put on every crease the curve runs over.
#[allow(clippy::too_many_arguments)]
pub fn try_bite_the_wall(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    corner: &SurfaceMixedCorner,
    corner_faces: &SurfaceMixedCornerFaces,
    run_out: &[Vec<i32>],
    run_out_turns: &[f64],
    split_from: i32,
) -> Result<(), BevelAdvisory> {
    for (piece, normal) in corner_faces.strips_under_the_blend() {
        let rim = piece.to_vec();
        let snap_to: Vec<i32> = piece.iter().map(|&at| pool.add(at)).collect();

        // Each piece in its OWN facet's plane, named by a point of that piece rather than by the
        // corner: on a rolled bead the facets past the first do not pass through the corner at
        // all, and a bite written in one plane finds no face in any of the others.
        super::surface_convex_bite::try_take(
            pool, faces, &rim, normal, piece[0], &snap_to, split_from,
        )?;
    }

    for station in 0..run_out.len().saturating_sub(1) {
        let here = &run_out[station];
        let there = &run_out[station + 1];
        let from = pool.position_of(here[here.len() - 1]);
        let to = pool.position_of(there[there.len() - 1]);

        // Which facet this station's own piece stands in, asked at the MIDDLE of the two turns:
        // the trim curve crosses no crease between two stations, because a station is put on every
        // one of them, so the middle names the facet both ends stand on.
        let middle =
            corner.turn_radians() + ((run_out_turns[station] + run_out_turns[station + 1]) / 2.0);

        let Some((stands_in, facing)) = corner_faces.unpicked_face.try_plane_at(middle) else {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceCornerNotSupported,
                vec![corner.corner.into()],
            ));
        };

        // And only while the trim curve is still IN that wall. Where another chain's band stands
        // on the wall, the run-out's columns end on the band instead and their ends lift off the
        // wall into the material: the wall there was taken by that band and not by this scallop.
        if ((from - stands_in).dot(facing)).abs() > pool.near_enough()
            || ((to - stands_in).dot(facing)).abs() > pool.near_enough()
        {
            continue;
        }

        let above_to = pool.add(under_the_cap(corner, to));
        let above_from = pool.add(under_the_cap(corner, from));
        let trapezoid = super::surface_ring::without_repeats(&[
            here[here.len() - 1],
            there[there.len() - 1],
            above_to,
            above_from,
        ]);

        let scallop: Vec<Vector3> = trapezoid.iter().map(|&v| pool.position_of(v)).collect();

        super::surface_convex_bite::try_take(
            pool, faces, &scallop, facing, stands_in, &trapezoid, split_from,
        )?;
    }

    Ok(())
}
