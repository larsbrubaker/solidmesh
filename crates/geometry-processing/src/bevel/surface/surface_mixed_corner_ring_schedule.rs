//! A MITRE hand-over's patch, turned rigidly round the bead's axis station by station station by
//! station.

use vector_math::Vector3;

use super::{SurfaceMixedCorner, SurfaceVertexPool};
use crate::bevel::{BevelAdvisory, BevelAdvisoryArg, BevelAdvisoryCode};

/// How near two angles have to be to be one station. A thousandth of a degree: the rails it tells
/// apart are a whole segment apart on an ordinary corner and 3.65 degrees on a leaning one, so
/// nothing real is anywhere near it, and it is an ANGLE rather than a length so it does not scale.
const ANGLE_RULER: f64 = 1e-5;

/// The grid, column by column (one row per `profile` vertex, one column per station): station
/// zero is `profile` (the ring, from the cap face round to the shared face) and the row on the
/// shared face is `turns` (the bead's own end ring, from the shared face round to the wall), both
/// by reuse. `slack` is how far the ring is allowed to stand off the ideal torus - the shared
/// face's own chord sagitta, which is the error the reused row absorbs.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)` on purpose: it refuses a NaN as well.
pub fn try_turn_the_ring_round(
    pool: &mut SurfaceVertexPool,
    corner: &SurfaceMixedCorner,
    profile: &[i32],
    turns: &[i32],
    slack: f64,
) -> Result<Vec<Vec<i32>>, BevelAdvisory> {
    let not_supported = || {
        BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceCornerNotSupported,
            vec![BevelAdvisoryArg::Vector3(corner.corner)],
        )
    };

    if profile.len() < 2 || turns.len() < 2 {
        return Err(not_supported());
    }

    // Every vertex's own angle round the axis, and the turn it has left to reach the meridian
    // where the ball meets the wall.
    let mut angle_of = vec![0.0; profile.len()];
    let mut left_to_turn = vec![0.0; profile.len()];

    for p in 0..profile.len() {
        angle_of[p] = self::angle_of(corner, pool.position_of(profile[p]));
        left_to_turn[p] = corner.turn_radians() - angle_of[p];

        if !(left_to_turn[p] > 0.0) {
            // A ring vertex already at or past the wall's own meridian has no sweep left, so
            // there is no patch to draw from it.
            return Err(not_supported());
        }
    }

    let on_the_shared_face = profile.len() - 1;

    // The schedule: the bead's rails, as fractions of the sweep the ring's own rail on the shared
    // face has to make. A rail the ring has already passed is not a station - on an ordinary corner
    // the ring's rail IS the bead's first one, and this is what keeps that corner one station
    // shorter rather than starting it with a row of no width. Station zero is the ring itself.
    let mut stations: Vec<Option<usize>> = vec![None];

    for (j, &turn) in turns.iter().enumerate() {
        if self::angle_of(corner, pool.position_of(turn))
            > angle_of[on_the_shared_face] + ANGLE_RULER
        {
            stations.push(Some(j));
        }
    }

    if stations.len() < 2 {
        return Err(not_supported());
    }

    let mut grid = vec![vec![0; stations.len()]; profile.len()];

    for p in 0..profile.len() {
        for (station, &rail) in stations.iter().enumerate() {
            let Some(rail) = rail else {
                grid[p][0] = profile[p];
                continue;
            };

            let how_far = (self::angle_of(corner, pool.position_of(turns[rail]))
                - angle_of[on_the_shared_face])
                / left_to_turn[on_the_shared_face];

            let at = corner.turned_about(pool.position_of(profile[p]), how_far * left_to_turn[p]);

            if p != on_the_shared_face {
                grid[p][station] = pool.add(at);
                continue;
            }

            // The row on the shared face is the bead's own rails. What the reuse absorbs is how
            // far the ring's rail stands off the ball's own cylinder, which is the same error the
            // ring carries against the ideal torus - refused by name if that is more than the
            // mesh's own chords already are.
            if (pool.position_of(turns[rail]) - at).length() > slack {
                return Err(BevelAdvisory::new(
                    BevelAdvisoryCode::SurfaceStripSeamDidNotMeet,
                    vec![BevelAdvisoryArg::Vector3(at)],
                ));
            }

            grid[p][station] = turns[rail];
        }
    }

    Ok(grid)
}

/// How far a point stands round the bead's axis, in the corner's own frame: zero towards the
/// shared face and [`SurfaceMixedCorner::turn_radians`] towards the wall. A ring that leans reads
/// NEGATIVE here at the end that has not reached the tangency yet, which is the whole point of
/// asking each vertex rather than the ring.
pub fn angle_of(corner: &SurfaceMixedCorner, at: Vector3) -> f64 {
    let out_of_the_axis = at - corner.axis().closest_point_to(at);

    f64::atan2(
        out_of_the_axis.dot(corner.towards_the_wall_square()),
        out_of_the_axis.dot(corner.towards_the_shared_face()),
    )
}
