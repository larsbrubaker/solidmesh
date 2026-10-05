//! Which concave runs a bead has to stay out of (`Bevel/BevelRunPartners.cs`, a static class):
//! every other concave run that shares an END with this one, read as the straight tube its ball
//! sweeps ([`BevelPartnerTube`]).

use std::collections::HashSet;

use vector_math::math_helper::degrees_to_radians;
use vector_math::Vector3;

use super::{BevelPartnerTube, BevelSweptChain};

/// C# `StraightWithinDegrees`: how far a run's edges may turn and still be one straight tube.
pub const STRAIGHT_WITHIN_DEGREES: f64 = 0.1;

/// C# `For`: the tubes of every run in `concave_runs` (other than `chain` itself, by identity)
/// that shares an end with `chain`, and how many such runs were skipped because they turn.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn for_chain(
    chain: &BevelSweptChain,
    concave_runs: &[BevelSweptChain],
    size: f64,
) -> (Vec<BevelPartnerTube>, i32) {
    let mut skipped_for_turning = 0;
    let mut tubes = Vec::with_capacity(2);

    if chain.edges.is_empty() || !(size > 0.0) {
        return (tubes, skipped_for_turning);
    }

    let ends = HashSet::from([
        chain.edges[0].from_vertex(),
        chain.edges[chain.edges.len() - 1].to_vertex(),
    ]);

    for other in concave_runs {
        if std::ptr::eq(other, chain) || other.edges.is_empty() {
            continue;
        }

        if !ends.contains(&other.edges[0].from_vertex())
            && !ends.contains(&other.edges[other.edges.len() - 1].to_vertex())
        {
            // Shares no END with this run. It may still cross it, touch it or run beside it, and
            // none of that makes it a partner.
            continue;
        }

        let Some((axis_point, direction)) = try_ball_centre_line(other, size) else {
            skipped_for_turning += 1;

            continue;
        };

        tubes.push(BevelPartnerTube::new(axis_point, direction, size));
    }

    (tubes, skipped_for_turning)
}

/// C# `TryBallCentreLine`: the line a ball of radius `size` rolling along a straight concave run
/// centres on - `(axis_point, direction)` - or `None` for a run that turns or a wedge too open.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn try_ball_centre_line(chain: &BevelSweptChain, size: f64) -> Option<(Vector3, Vector3)> {
    if chain.edges.is_empty() || !(size > 0.0) {
        return None;
    }

    let direction = chain.edges[0].direction();
    let straight = degrees_to_radians(STRAIGHT_WITHIN_DEGREES).cos();

    for edge in &chain.edges {
        if edge.direction().dot(direction) < straight {
            return None;
        }
    }

    let first = &chain.edges[0];
    let normal0 = first.edge.normal0;
    let normal1 = first.edge.normal1;
    let between = 1.0 + normal0.dot(normal1);

    if between < 1e-6 {
        return None;
    }

    Some((
        first.from() + ((normal0 + normal1) * (size / between)),
        direction,
    ))
}
