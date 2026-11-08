//! Where a refusal says it happened, and which picked edge stands nearest that place
//! ([`the_edge_nearest_the_place_among`]) - the fallback a pass charges a refusal by when the
//! build could not stamp the edge itself.

use solidmesh_math::Vector3;

use crate::bevel::{BevelAdvisory, BevelAdvisoryArg};
use crate::topology::FeatureEdge;

/// The picked edge nearest the place a refusal names, asked
/// among any set of picks that are separate from one another, which is the groups of a pass or the
/// rim loops within them. Returns the mesh edge when there is a clear winner, `None` otherwise.
///
/// `units` has one entry per thing that can be blamed on its own. Which those are is the caller's
/// reading: two loops of one face's rim are as separate as two groups, so the clear-winner margin
/// has to be asked between them too, and a place that two of them both stand within a cut of is
/// nobody's either way.
pub fn the_edge_nearest_the_place_among<U>(
    units: &[U],
    reason: Option<&BevelAdvisory>,
    clear_by: f64,
) -> Option<i32>
where
    U: AsRef<[FeatureEdge]>,
{
    let mut mesh_edge_index = BevelAdvisory::NO_MESH_EDGE;

    let place = the_refusal_names_a_place(reason)?;

    let mut nearest = f64::MAX;
    let mut runner_up = f64::MAX;

    for unit in units {
        let mut closest = f64::MAX;
        let mut closest_edge = BevelAdvisory::NO_MESH_EDGE;

        for edge in unit.as_ref() {
            let distance = distance_to_the_segment(place, edge.start, edge.end);

            if distance < closest {
                closest = distance;
                closest_edge = edge.mesh_edge_index;
            }
        }

        if closest_edge == BevelAdvisory::NO_MESH_EDGE {
            continue;
        }

        if closest < nearest {
            runner_up = nearest;
            nearest = closest;
            mesh_edge_index = closest_edge;
        } else if closest < runner_up {
            runner_up = closest;
        }
    }

    if mesh_edge_index == BevelAdvisory::NO_MESH_EDGE {
        return None;
    }

    // And only where the answer is NOT a tie. A corner is a place three picked edges all stand on,
    // so a refusal there is every one of theirs, and blaming whichever of them sorts first would
    // put a sentence a user cannot act on against two rows and take the third's answer away from
    // it. The margin is the operation's own size, because that is the reach of a cut: a pick with
    // an edge within one bevel of the place really could be what the refusal is about.
    (runner_up - nearest > clear_by).then_some(mesh_edge_index)
}

/// Where a refusal says it happened, which is the first position among its arguments.
fn the_refusal_names_a_place(reason: Option<&BevelAdvisory>) -> Option<Vector3> {
    reason?.args().iter().find_map(|argument| match argument {
        BevelAdvisoryArg::Vector3(position) => Some(*position),
        _ => None,
    })
}

/// How far a point stands from a segment, which is what "nearest picked edge" is measured with.
fn distance_to_the_segment(point: Vector3, start: Vector3, end: Vector3) -> f64 {
    let along = end - start;
    let length_squared = along.length_squared();

    if length_squared <= 0.0 {
        return (point - start).length();
    }

    let offset = point - start;
    let t = (offset.dot(along) / length_squared).clamp(0.0, 1.0);

    (point - (start + (along * t))).length()
}
