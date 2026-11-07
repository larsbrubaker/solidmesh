//! Whether the band flows from a side onto a gentle neighbour: its constant and the flow test
//! ([`flows_onto_a_gentle_neighbour`]).

use vector_math::Vector3;

use super::triangle_edges::{self, StandingMap};
use super::{ChainField, WorkError};

/// How far under a right angle (as the cosine between the unit normals) a convex crease must turn
/// to be gentle: an exact right angle reads its normals' dot as +-1e-16 and must stay a wall (a 2 mm
/// wall under a fillet of 2 is consumed to its rim, not flowed over), as in the foreign-edge
/// room's knife test.
pub const RIGHT_ANGLE_NOISE: f64 = 1e-9;

/// Whether a side's flood carries on over the crease
/// `a`-`b`: the band at this size (`side_reach`'s level line) reaches it, and the face across turns
/// away CONVEXLY by less than a right angle. A knife (past a right angle) takes the flow with zero
/// width, and a concave crease grows over the blend; both keep stopping the flood.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn flows_onto_a_gentle_neighbour(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    standing: &StandingMap,
    side_reach: &ChainField,
    a: i32,
    b: i32,
) -> Result<bool, WorkError> {
    let Some(on) = standing
        .get(&triangle_edges::key(a, b))
        .filter(|on| on.len() == 2)
    else {
        return Ok(false);
    };
    if !(side_reach
        .min_along_segment(positions[a as usize], positions[b as usize])?
        .value
        < 0.0)
    {
        return Ok(false);
    }

    let normal_of = |triangle: i32| triangle_edges::normal(positions, indices[triangle as usize]);
    let first = normal_of(on[0]);
    let across = indices[on[1] as usize];
    let opposite = if across.0 != a && across.0 != b {
        across.0
    } else if across.1 != a && across.1 != b {
        across.1
    } else {
        across.2
    };
    let convex = first.dot(positions[opposite as usize] - positions[a as usize]) < 0.0;
    Ok(convex && first.dot(normal_of(on[1])) > RIGHT_ANGLE_NOISE)
}
