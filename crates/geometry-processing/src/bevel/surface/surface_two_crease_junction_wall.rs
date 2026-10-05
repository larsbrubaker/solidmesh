//! What the WALL gives up under a two-crease junction's patch, and what it is owed back
//! (`Bevel/Surface/SurfaceTwoCreaseJunctionWall.cs`, a static class): the one face the patch's row
//! zero runs across, made to end exactly where that row runs - two bites and one fill, and no
//! repair. Here are its nested record, where row zero leaves one crease
//! ([`SurfaceTwoCreaseJunctionWallCrossing`]), and the fill ([`try_fill_the_notch`]); the bites
//! land with later packages.

use vector_math::Vector3;

use super::{surface_fan, SurfaceVertexPool};
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
