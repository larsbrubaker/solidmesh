//! What the WALL gives up under a two-crease junction's patch, and what it is owed back
//! (`Bevel/Surface/SurfaceTwoCreaseJunctionWall.cs`, a static class): the one face the patch's row
//! zero runs across, made to end exactly where that row runs - two bites and one fill, and no
//! repair. Here are its nested record, where row zero leaves one crease
//! ([`SurfaceTwoCreaseJunctionWallCrossing`]) and how it is found ([`try_the_crossing`]), the bead's
//! foot ([`try_the_foot`]) and the fill ([`try_fill_the_notch`]); the bites land with later
//! packages.

use vector_math::Vector3;

use super::{surface_fan, SurfaceMixedCorner, SurfaceMixedJunctionArm, SurfaceVertexPool};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// C# `SurfaceTwoCreaseJunctionWall.Crossing` (a private readonly struct): where row zero leaves
/// one crease - the two stations it happens between, and the point.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SurfaceTwoCreaseJunctionWallCrossing {
    /// The last station standing on the BEAD's side of the crease.
    pub before: i32,
    /// The first standing on the notch side - the neighbour of
    /// [`SurfaceTwoCreaseJunctionWallCrossing::before`].
    pub after: i32,
    /// Where between the two row zero meets the crease's own line.
    pub at: Vector3,
}

/// C# `TryFillTheNotch`: the notch between the two crossings, fanned from the CORNER over row
/// zero. `grid` is C#'s `int[][]`, station by station, row zero first; `Err` is C#'s `failure`.
///
/// From the corner because both creases end there, so the fan's two straight sides are the creases
/// themselves and its arc is row zero: every side of it is a curve the part or the patch already
/// owns.
///
/// **The sense is MEASURED, not assumed.** Which of the two creases the grid's station zero belongs
/// to is the reading's own business - it is whichever arm came out first, and a mesh that chords
/// one crease and not the other can turn that over - so a fan wound one fixed way about the corner
/// faces out of the wall on one part and into it on the next. What is really required is that the
/// triangles AGREE: a fan sweeping one way about the corner has every one of its triangles turning
/// the same way, and [`surface_fan::emit_triangle_facing`] then winds each to face out. A fan that
/// doubles back has two that do not, and that is a refusal rather than a sliver wound to look
/// right.
#[allow(clippy::too_many_arguments)]
pub fn try_fill_the_notch(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    corner: Vector3,
    out_of_the_wall: Vector3,
    grid: &[Vec<i32>],
    at_the_first_arm: SurfaceTwoCreaseJunctionWallCrossing,
    first_at: i32,
    at_the_second_arm: SurfaceTwoCreaseJunctionWallCrossing,
    second_at: i32,
) -> Result<(), BevelAdvisory> {
    let apex = pool.add(corner);

    // Station zero is the FIRST arm's own end of the turn and the last station the second's, so the
    // run between the two crossings is walked that way about whichever crease is which.
    let mut rails = vec![first_at];

    let mut station = at_the_first_arm.after;
    while station <= at_the_second_arm.after {
        rails.push(grid[station as usize][0]);
        station += 1;
    }

    rails.push(second_at);

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

impl SurfaceTwoCreaseJunctionWallCrossing {
    /// C#'s constructor: row zero leaves the crease between stations `before` and `after`, at `at`.
    pub fn new(before: i32, after: i32, at: Vector3) -> Self {
        SurfaceTwoCreaseJunctionWallCrossing { before, after, at }
    }
}

/// Where row zero (`on_the_wall`, walked from its start where `from_the_start`, else from its end)
/// crosses `arm`'s crease - the line through `corner` along the bead, in the wall facing
/// `out_of_the_wall` - as the two stations it falls between and the point between them.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x < 0)`, which a NaN also passes.
pub fn try_the_crossing(
    arm: &SurfaceMixedJunctionArm,
    corner: Vector3,
    out_of_the_wall: Vector3,
    on_the_wall: &[Vector3],
    from_the_start: bool,
) -> Result<SurfaceTwoCreaseJunctionWallCrossing, BevelAdvisory> {
    let away = SurfaceMixedCorner::away_from_the_corner(&arm.mixed.bead, corner);
    let across = away.cross(out_of_the_wall);

    let count = on_the_wall.len() as i32;
    let step: i32 = if from_the_start { 1 } else { -1 };
    let from = if from_the_start { 0 } else { count - 1 };

    let mut started = (on_the_wall[from as usize] - corner).dot(across);

    let mut station = from + step;
    while station >= 0 && station < count {
        let here = (on_the_wall[station as usize] - corner).dot(across);

        if !(started * here < 0.0) {
            started = here;
            station += step;
            continue;
        }

        let previous = on_the_wall[(station - step) as usize];
        let was = (previous - corner).dot(across);

        return Ok(SurfaceTwoCreaseJunctionWallCrossing::new(
            station - step,
            station,
            previous + ((on_the_wall[station as usize] - previous) * (was / (was - here))),
        ));
    }

    // DEFENSIVE. The reading has already said the canal ends ON this bead and the bead's band ends
    // short of the corner, so row zero leaves the bead's side of the crease somewhere along the
    // turn. A row that never does is a notch with no mouth, which is not this corner drawn
    // differently.
    Err(BevelAdvisory::new(
        BevelAdvisoryCode::SurfaceCornerNotSupported,
        vec![corner.into()],
    ))
}

/// The bead's own FOOT on `arm`'s crease: where the plane the bead's band stops on crosses the
/// crease line through `corner`.
pub fn try_the_foot(
    arm: &SurfaceMixedJunctionArm,
    corner: Vector3,
) -> Result<Vector3, BevelAdvisory> {
    let away = SurfaceMixedCorner::away_from_the_corner(&arm.mixed.bead, corner);
    let stops_at = arm.mixed.bead_stops_at;

    let leaning = away.dot(stops_at.inward);

    if leaning.abs() < 1e-9 {
        // DEFENSIVE: a band that stops on a plane its own edge runs along stops nowhere.
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceCornerNotSupported,
            vec![corner.into()],
        ));
    }

    Ok(corner + (away * ((stops_at.point - corner).dot(stops_at.inward) / leaning)))
}
