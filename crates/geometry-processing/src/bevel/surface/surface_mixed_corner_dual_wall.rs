//! What the WALL gives up and gains under a leaning one-crease corner's patch - the face rail zero
//! runs across, made to end exactly where that rail runs. Ports
//! `Bevel/Surface/SurfaceMixedCornerDualWall.cs`, a static class (free functions here): its record,
//! where rail zero leaves one crease ([`SurfaceMixedCornerDualWallCrossing`]), the bead's foot on
//! its crease ([`try_the_foot`]) and the root's fan ([`try_fill_the_root`], [`try_fan`]). The rest
//! of the wall's reading lands with later packages.

use vector_math::Vector3;

use super::{surface_fan, SurfaceMixedCorner, SurfaceVertexPool};
use crate::bevel::{BevelAdvisory, BevelAdvisoryArg, BevelAdvisoryCode};

/// C# `SurfaceMixedCornerDualWall.Crossing` (a private readonly struct): where rail zero leaves
/// one crease - the two stations it happens between, and the point.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SurfaceMixedCornerDualWallCrossing {
    /// The last station standing on the side the walk started from.
    pub before: i32,
    /// The first standing past it - the neighbour of
    /// [`SurfaceMixedCornerDualWallCrossing::before`].
    pub after: i32,
    /// Where between the two rail zero meets the crease's own line.
    pub at: Vector3,
}

/// The bead's own FOOT on its crease: where the plane that bead's band stops on crosses the crease
/// line. A point the walkback already left there, so it is added to the pool, which welds it onto
/// that vertex rather than beside it.
pub fn try_the_foot(corner: &SurfaceMixedCorner) -> Result<Vector3, BevelAdvisory> {
    let away = SurfaceMixedCorner::away_from_the_corner(&corner.bead, corner.corner);
    let stops_at = corner.bead_stops_at;
    let leaning = away.dot(stops_at.inward);

    if leaning.abs() < 1e-9 {
        // DEFENSIVE: a band that stops on a plane its own edge runs along stops nowhere.
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceCornerNotSupported,
            vec![BevelAdvisoryArg::Vector3(corner.corner)],
        ));
    }

    Ok(corner.corner + (away * ((stops_at.point - corner.corner).dot(stops_at.inward) / leaning)))
}

/// C# `TryFillTheRoot`: the nose's ROOT between the two crossings, fanned from the CORNER over rail
/// zero - the wall the ball scooped the feature's own tip away from, which nobody has drawn.
/// `grid` is C#'s `int[][]`, station by station, rail zero first.
///
/// From the corner because both creases end there, so the fan's two straight sides are the
/// creases themselves and its arc is rail zero: every side of it is a curve the part or the patch
/// already owns. The sense is MEASURED and not assumed - a fan that doubles back has two triangles
/// turning opposite ways, and that is a refusal rather than a sliver wound to look right.
#[allow(clippy::too_many_arguments)]
pub fn try_fill_the_root(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    corner: Vector3,
    out_of_the_wall: Vector3,
    grid: &[Vec<i32>],
    at_the_bead: SurfaceMixedCornerDualWallCrossing,
    bead_at: i32,
    at_the_edge: SurfaceMixedCornerDualWallCrossing,
    edge_at: i32,
) -> Result<(), BevelAdvisory> {
    let apex = pool.add(corner);
    let mut rails = vec![bead_at];

    let mut station = at_the_bead.after;
    while station <= at_the_edge.before {
        rails.push(grid[station as usize][0]);
        station += 1;
    }

    rails.push(edge_at);

    try_fan(pool, faces, apex, corner, out_of_the_wall, &rails)
}

/// C# `TryFan`: a fan from `apex` over `rails`, refused where it doubles back (C#'s `failure`).
/// What it emitted before the refusal stays in `faces`, as in C#; the caller refuses the whole
/// corner.
pub fn try_fan(
    pool: &SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    apex: i32,
    corner: Vector3,
    out_of_the_wall: Vector3,
    rails: &[i32],
) -> Result<(), BevelAdvisory> {
    let mut sweeping = 0.0;

    let mut rail = 0;
    while rail + 1 < rails.len() {
        let from = pool.position_of(rails[rail]);
        let to = pool.position_of(rails[rail + 1]);
        let turning = (from - corner).cross(to - corner).dot(out_of_the_wall);

        if turning == 0.0 || (sweeping != 0.0 && (turning > 0.0) != (sweeping > 0.0)) {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceCornerNotSupported,
                vec![from.into()],
            ));
        }

        sweeping = turning;

        surface_fan::emit_triangle_facing(
            pool,
            faces,
            apex,
            rails[rail],
            rails[rail + 1],
            out_of_the_wall,
        );

        rail += 1;
    }

    Ok(())
}
