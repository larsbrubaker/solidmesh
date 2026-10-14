//! Whether a bead's end is where its crease ENDED or only where something INTERRUPTED it - the
//! difference between stopping square there and running on through: the running reading of how the
//! faces bounding one end lean against the run ([`Leaning`]) and what one end's reading came to
//! ([`Interruption`]). The readings are `impl` blocks and free functions in this module.

/// Which way the faces bounding one end lean against the run, accumulated as the cap plane scan
/// meets them, and the reading of whether that end is an interruption.
use vector_math::{cs_math, Vector3};

use super::bevel_chain_end_budget;
use super::{BevelAdvisory, BevelAdvisoryCode, BevelChainEnds, BevelSweptChain};
use crate::topology::{ChainEdge, EdgeConvexity, FeatureEdgeSet};

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Leaning {
    /// How squarely the steepest face standing OVER the run's own continuation crosses it, as the
    /// magnitude of its normal's agreement with the run's direction. Zero where none does.
    pub closes_over_the_run: f64,
    /// Whether any face here leans the way the run leaves instead.
    pub falls_away_from_the_run: bool,
}

/// What one end's reading came to - whether it is an interruption to run on through, and how far
/// along the run the crease picks up again on the far side of it.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Interruption {
    /// How far past this end the selection's own crease resumes. Zero where it does not, and the
    /// end runs on exactly where this is positive.
    pub gap: f64,
}

/// How far along the run a face has to lean to count as crossing it, as a cosine: the steepest
/// cap plane a run may stop on ([`BevelChainEnds::MAXIMUM_CAP_PLANE_ANGLE_DEGREES`]).
pub fn along_the_run_agreement() -> f64 {
    (BevelChainEnds::MAXIMUM_CAP_PLANE_ANGLE_DEGREES * std::f64::consts::PI / 180.0).cos()
}

/// Whether a face leaning `agreement` along the run crosses it.
pub fn crosses_the_run(agreement: f64) -> bool {
    agreement > along_the_run_agreement()
}

/// How near two face normals have to stand to be one surface, in degrees.
const SAME_SURFACE_DEGREES: f64 = 1.0;

/// How near another crease has to run to the run's own line to be the same crease carrying on, in
/// degrees.
const CREASE_CARRIES_ON_DEGREES: f64 = 45.0;

/// How far off the run's line a crease end may stand and still be on it.
const OFF_THE_LINE_TOLERANCE: f64 = 0.01;

impl Leaning {
    /// Takes in one bounding face that leans `along_outward` along the run's way out: one that
    /// crosses the run closing back over it raises the closing reading, one that crosses it
    /// falling away marks the end as falling away.
    pub fn saw(&mut self, along_outward: f64) {
        if !crosses_the_run(along_outward.abs()) {
            return;
        }

        if along_outward < 0.0 {
            self.closes_over_the_run = cs_math::max_f64(self.closes_over_the_run, -along_outward);
        } else {
            self.falls_away_from_the_run = true;
        }
    }

    /// Whether the faces at the end close back over the run - something was seen, nothing falls
    /// away, and the closing face crosses the run.
    pub fn closes_over_the_run(&self, saw_candidate: bool) -> bool {
        saw_candidate && !self.falls_away_from_the_run && crosses_the_run(self.closes_over_the_run)
    }

    /// Whether the crease RUNS ON past this end: the end closes over the run, both of the run's own
    /// faces carry on past the vertex, no crease carries on from it, and the crease resumes beyond
    /// - the interruption's gap; no interruption (`Interruption::default()`) otherwise.
    #[allow(clippy::too_many_arguments)]
    pub fn runs_on(
        &self,
        saw_candidate: bool,
        incident: &[i32],
        features: &FeatureEdgeSet,
        own_edge: i32,
        outward: Vector3,
        own_normal0: Vector3,
        own_normal1: Vector3,
        position: Vector3,
        concave_runs: Option<&[BevelSweptChain]>,
    ) -> Interruption {
        if !self.closes_over_the_run(saw_candidate)
            || !both_own_faces_carry_on(incident, features, own_edge, own_normal0, own_normal1)
            || crease_carries_on(incident, features, own_edge, outward)
        {
            return Interruption::default();
        }

        let gap = crease_resumes_beyond(concave_runs, position, outward).0;

        if gap > 0.0 {
            Interruption::new(gap)
        } else {
            Interruption::default()
        }
    }

    /// Whether the crease DIES BURIED at this end: the end closes over the run, no crease carries
    /// on from it, and no concave crease lies on beyond it within twice `reach`.
    #[allow(clippy::too_many_arguments)]
    pub fn dies_buried(
        &self,
        saw_candidate: bool,
        incident: &[i32],
        features: &FeatureEdgeSet,
        own_edge: i32,
        outward: Vector3,
        position: Vector3,
        reach: f64,
    ) -> bool {
        self.closes_over_the_run(saw_candidate)
            && !crease_carries_on(incident, features, own_edge, outward)
            && !crease_lies_beyond(features, position, outward, 2.0 * reach)
    }
}

impl Interruption {
    /// An interruption whose far side is `gap` ahead.
    pub fn new(gap: f64) -> Self {
        Interruption { gap }
    }

    /// Whether the crease runs on past the interruption at all.
    pub fn runs_on(&self) -> bool {
        self.gap > 0.0
    }
}

/// Whether a CONCAVE feature edge starts (or ends) on the run's line ahead of `position`, within
/// `reach`, running on the same way.
pub fn crease_lies_beyond(
    features: &FeatureEdgeSet,
    position: Vector3,
    outward: Vector3,
    reach: f64,
) -> bool {
    let still_the_same_line = (CREASE_CARRIES_ON_DEGREES * std::f64::consts::PI / 180.0).cos();

    for edge in features.edges() {
        if edge.convexity != EdgeConvexity::Concave {
            continue;
        }

        for (at, along) in [
            (edge.start, edge.end - edge.start),
            (edge.end, edge.start - edge.end),
        ] {
            let to_end = at - position;
            let along_run = to_end.dot(outward);

            if along_run <= OFF_THE_LINE_TOLERANCE
                || along_run > reach
                || (to_end - (outward * along_run)).length() > OFF_THE_LINE_TOLERANCE
                || along.get_normal().dot(outward) < still_the_same_line
            {
                continue;
            }

            return true;
        }
    }

    false
}

/// The advisories for a bead whose crease runs on past an interruption wider than twice the
/// end's reach at `size` - at the run's start (`first`) and end (`last`), in that order.
pub fn short_of_the_far_side(
    at_start: Interruption,
    at_end: Interruption,
    size: f64,
    round_profile: bool,
    first: &ChainEdge,
    last: &ChainEdge,
) -> Vec<BevelAdvisory> {
    let mut advisories = Vec::new();

    for (interruption, segment, at) in [(at_start, first, first.from()), (at_end, last, last.to())]
    {
        if interruption.runs_on()
            && interruption.gap > 2.0 * bevel_chain_end_budget::reach(size, round_profile, segment)
        {
            advisories.push(BevelAdvisory::new(
                BevelAdvisoryCode::BeadDoesNotSpanInterruption,
                vec![at.into(), size.into(), interruption.gap.into()],
            ));
        }
    }

    advisories
}

/// How far ahead along the run's line the nearest open concave run starts again, running on the
/// same way - `(gap, which run)`, or `(0, -1)` where none does.
pub fn crease_resumes_beyond(
    concave_runs: Option<&[BevelSweptChain]>,
    position: Vector3,
    outward: Vector3,
) -> (f64, i32) {
    let Some(concave_runs) = concave_runs else {
        return (0.0, -1);
    };

    let still_the_same_line = (CREASE_CARRIES_ON_DEGREES * std::f64::consts::PI / 180.0).cos();
    let mut nearest = 0.0;
    let mut which = -1;

    for (index, run) in concave_runs.iter().enumerate() {
        if run.is_closed {
            continue;
        }

        let first = &run.edges[0];
        let last = &run.edges[run.edges.len() - 1];

        for (at, along) in [
            (first.from(), first.direction()),
            (last.to(), -last.direction()),
        ] {
            let to_end = at - position;
            let along_run = to_end.dot(outward);

            if along_run <= OFF_THE_LINE_TOLERANCE
                || (to_end - (outward * along_run)).length() > OFF_THE_LINE_TOLERANCE
                || along.dot(outward) < still_the_same_line
            {
                continue;
            }

            if nearest == 0.0 || along_run < nearest {
                nearest = along_run;
                which = index as i32;
            }
        }
    }

    (nearest, which)
}

/// Whether BOTH of the run's own faces carry on past the vertex: some other incident feature edge
/// has a face on each one's surface. Read by NORMAL rather than by face index, because a boolean's
/// output carries one plane on both sides of a bite as two regions with two indices.
pub fn both_own_faces_carry_on(
    incident: &[i32],
    features: &FeatureEdgeSet,
    own_edge: i32,
    own_normal0: Vector3,
    own_normal1: Vector3,
) -> bool {
    let still_the_same_surface = (SAME_SURFACE_DEGREES * std::f64::consts::PI / 180.0).cos();
    let mut carries_on0 = false;
    let mut carries_on1 = false;

    for &index in incident {
        let edge = features.edges()[index as usize];

        if edge.mesh_edge_index == own_edge {
            continue;
        }

        for normal in [edge.normal0, edge.normal1] {
            carries_on0 |= normal.dot(own_normal0) >= still_the_same_surface;
            carries_on1 |= normal.dot(own_normal1) >= still_the_same_surface;
        }
    }

    carries_on0 && carries_on1
}

/// Whether another incident feature edge runs on along the run's line (either way, as a line),
/// within [`CREASE_CARRIES_ON_DEGREES`].
pub fn crease_carries_on(
    incident: &[i32],
    features: &FeatureEdgeSet,
    own_edge: i32,
    outward: Vector3,
) -> bool {
    let most_of_the_way_on = (CREASE_CARRIES_ON_DEGREES * std::f64::consts::PI / 180.0).cos();

    incident.iter().any(|&index| {
        let edge = features.edges()[index as usize];

        edge.mesh_edge_index != own_edge
            && (edge.end - edge.start).get_normal().dot(outward).abs() >= most_of_the_way_on
    })
}
