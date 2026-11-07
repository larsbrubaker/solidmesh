//! How many segments an arc of a given angle is chorded into - the one place that reads the
//! operation's `Segments` as a count per QUARTER TURN rather than a count per arc.
//!
//! A fixed count per arc chords a sharp corner far more coarsely than a blunt one: a 45 degree
//! knife tip turns its fillet through 135 degrees, so eight segments there step 16.875 degrees
//! against a 90 degree edge's 11.25, and the sagitta grows with the square of the step. Reading
//! the number as segments per quarter turn makes the chord error the same everywhere.
//!
//! It has to be ONE helper because two places must agree exactly: a run's profile arc
//! (`BevelEdgeToolBuilder`) and the corner patch's boundary arc over that run
//! (`BevelCornerPatchMesher`) are the seam the two solids meet on.
//!
//! NOT WIRED UP YET: `BevelFeatureMeshBuilder` still hands every tool the operation's own
//! `Segments`. Chording "Bevel to Center Feature.mcx" at 12 walks the cutter union into a T-vertex
//! under Manifold's snap (the run's cleared apex lands 1.35e-6 inside the corner patch's sector
//! wall), so the finer count is blocked on giving a run's non-arc ring points somewhere that is not
//! the patch's own wall, not on anything in this file.

use crate::topology::FeatureEdge;

/// How much arc one segment of the operation's own count stands for.
const QUARTER_TURN_DEGREES: f64 = 90.0;

/// The segment count for an arc of `angle_degrees`, given the operation's segments per quarter
/// turn. Never less than one: an arc still has to be crossed even when it is a sliver.
pub fn segments_for(angle_degrees: f64, segments_per_quarter: i32) -> i32 {
    // `<= 0 || NaN` refuses every angle that is not positive, a NaN included.
    if segments_per_quarter < 1 || angle_degrees <= 0.0 || angle_degrees.is_nan() {
        return 1.max(segments_per_quarter);
    }

    // Half rounds away from zero (f64::round). `as` saturates past i32's range, which only an
    // absurd angle can reach.
    let rounded =
        (f64::from(segments_per_quarter) * angle_degrees / QUARTER_TURN_DEGREES).round() as i32;
    1.max(rounded)
}

/// The ONE count every ring of one operation is built at: the largest any of its arcs asks for,
/// and never fewer than the operation's own `segments_per_quarter`.
///
/// Applied per operation rather than per arc: every seam in a bevel is two solids carrying the
/// same ring of points, and they meet only if the counts match exactly. A corner patch's rows
/// all start on a RIB shared with the neighbouring sector, so per-arc counts would sample one rib
/// two ways and leave a crack. Each selected edge contributes one angle, and it covers the corner
/// patches too: a patch's boundary arc spans the same `180 - theta` as that edge's own profile
/// arc ([`FeatureEdge::turn_angle_degrees`]). The cost - a part with one sharp feature gets finer
/// rounds everywhere - is accepted.
///
/// `edges` is every edge the operation has resolved, across all of its groups; `None` answers the
/// operation's own count.
pub fn effective_segments<'a, I>(edges: Option<I>, segments_per_quarter: i32) -> i32
where
    I: IntoIterator<Item = &'a FeatureEdge>,
{
    let mut effective = 1.max(segments_per_quarter);

    let Some(edges) = edges else {
        return effective;
    };

    for edge in edges {
        effective = effective.max(segments_for(
            edge.turn_angle_degrees.abs(),
            segments_per_quarter,
        ));
    }

    effective
}
