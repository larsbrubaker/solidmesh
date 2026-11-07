//! Plan 5.3's hardest corner, found and named rather than silently mis-built: a vertex where FOUR
//! or more selected edges meet and they are not all the same convexity, so cuts and beads have to
//! interact there and more than one construction could claim the corner.
//!
//! **What is and is not handled, per the plan's resolved decision.** TWO selected edges of mixed
//! convexity meeting at a vertex - the L bracket's ends, one cut chain running into one bead
//! chain - comes out geometrically right and gets no warning. It is BLENDED where the corner admits it
//! (`BevelMixedBlends`: the cut carries on round the rim the bead left, so no fin of bead stands
//! beside the rounded rim), and where it does not - two sizes, two segment counts, an inner corner
//! running off square - it still lands whole, each run capped on the faces of the other
//! ([`super::BevelChainEnds`] counts an opposite-convexity neighbour as something to stop against,
//! because union does not resolve a corner between tools that are booleaned in opposite
//! directions), with the bead added as material the part did not have so it cannot restore what
//! the cut took (`BevelFeatureMeshBuilder`). The blend says out loud when it stood down, so the fin
//! is never shipped silently.
//!
//! THREE, mixed, blends as well, and this is not the place that decides it: a three-edge corner
//! whose faces close a ring of three decomposes into the pairwise blends the two-edge construction
//! already builds, and `BevelMixedBlends` derives that and says out loud when it stood down. This
//! used to be where three was refused, on the grounds that the two rims there wanted rounding into
//! each other as well as into the bead - which was wrong: the blend arc between them IS that
//! rounding, and there is no third surface to want.
//!
//! FOUR or more is what is still refused, and for a reason that is sharp rather than a deferral.
//! Three selected ends of ONE convexity at a vertex already build a corner patch
//! ([`super::BevelCornerPatchSet`]), so at four a patch and a blend would both claim it - and
//! [`super::BevelChainEnds`] reads those two maps in an order whose whole safety is that no vertex
//! is in both. Every run is still capped at the vertex, so the result is a solid and the part still
//! builds, but the caps leave a raw corner where CAD would blend, and no order of
//! subtract-then-union resolves it. That is the "capped result with a warning" the decision asks
//! for, and this is the warning. It says where, because on a part with many groups the user's next
//! question is which corner.
//!
//! Counted across ALL groups at once: the vertex does not care which group each edge was picked
//! into, and the all-edges gesture puts every edge in a group of its own.

use indexmap::{IndexMap, IndexSet};
use vector_math::Vector3;

use super::{BevelAdvisory, BevelAdvisoryCode};
use crate::topology::{BevelEdgeGroupResolution, EdgeConvexity, FeatureEdge};

/// How many selected edges have to meet before a mixed vertex is beyond what blending resolves.
/// Two is the case held to be correct from day one and three is blended too; four is where a
/// corner patch starts claiming the same vertex, which is the whole of the reason - see the module
/// docs.
const MINIMUM_EDGES_TO_WARN_ABOUT: i32 = 4;

/// The line a mixed corner that nothing rounded has to carry - how many selected edges of both
/// kinds meet there, where, and that the join is left square.
///
/// One sentence in one place, because a user reading a group's row cannot tell - and should not
/// have to - which part of the build found the corner. `BevelMixedBlends` says this same line at a
/// three-edge corner it stood down from, and adds its own reason where it has one; this module says
/// it at four or more and adds the reason below.
pub fn capped_corner(edge_count: i32, position: Vector3) -> BevelAdvisory {
    BevelAdvisory::new(
        BevelAdvisoryCode::MixedCornerCapped,
        vec![edge_count.into(), position.into()],
    )
}

/// The warnings per group, in the resolutions' own order, or `None` for a group with no such corner
/// on it. A corner says two things - the capped line and the reason - and both go to every group at
/// it.
pub fn describe(resolutions: &[BevelEdgeGroupResolution]) -> Vec<Option<Vec<BevelAdvisory>>> {
    let mut warnings: Vec<Option<Vec<BevelAdvisory>>> = vec![None; resolutions.len()];
    // Read back in insertion order: the order the lines reach each group's row.
    let mut at_vertex: IndexMap<i32, Corner> = IndexMap::new();

    for (group_index, resolution) in resolutions.iter().enumerate() {
        if !resolution.is_resolved() {
            continue;
        }

        for edge in &resolution.edges {
            record(
                &mut at_vertex,
                edge.start_vertex,
                edge.start,
                edge,
                group_index,
            );
            record(&mut at_vertex, edge.end_vertex, edge.end, edge, group_index);
        }
    }

    for corner in at_vertex.values() {
        if corner.edge_count() < MINIMUM_EDGES_TO_WARN_ABOUT || !corner.is_mixed() {
            continue;
        }

        let capped = capped_corner(corner.edge_count(), corner.position);
        let reason = BevelAdvisory::new(BevelAdvisoryCode::MixedCornerTooManyEdges, Vec::new());

        for &group_index in &corner.groups {
            let line = warnings[group_index].get_or_insert_with(Vec::new);

            line.push(capped.clone());
            line.push(reason.clone());
        }
    }

    warnings
}

fn record(
    at_vertex: &mut IndexMap<i32, Corner>,
    vertex: i32,
    position: Vector3,
    edge: &FeatureEdge,
    group_index: usize,
) {
    at_vertex
        .entry(vertex)
        .or_insert_with(|| Corner::new(position))
        .add(edge, group_index);
}

/// What has been selected at one mesh vertex so far.
struct Corner {
    groups: IndexSet<usize>,
    mesh_edges: IndexSet<i32>,
    position: Vector3,
    saw_convex: bool,
    saw_concave: bool,
}

impl Corner {
    fn new(position: Vector3) -> Self {
        Corner {
            groups: IndexSet::new(),
            mesh_edges: IndexSet::new(),
            position,
            saw_convex: false,
            saw_concave: false,
        }
    }

    /// How many distinct EDGES of the mesh were selected here, not how many pickings of them
    /// there were.
    fn edge_count(&self) -> i32 {
        self.mesh_edges.len() as i32
    }

    fn is_mixed(&self) -> bool {
        self.saw_convex && self.saw_concave
    }

    /// Counts one selected end at this corner, by MESH EDGE so that the same edge picked into two
    /// groups is one edge here as it is in `BevelMixedBlends`.
    ///
    /// The two have to agree or a corner falls between them: a genuine three-edge corner picked
    /// into four groups would blend AND be told it was too crowded to blend, which is one sentence
    /// too many and the wrong one. The GROUP is recorded either way, because every row of the list
    /// that meets at a corner has to carry what is said about it.
    fn add(&mut self, edge: &FeatureEdge, group_index: usize) {
        self.groups.insert(group_index);

        if !self.mesh_edges.insert(edge.mesh_edge_index) {
            return;
        }

        if edge.convexity == EdgeConvexity::Convex {
            self.saw_convex = true;
        } else if edge.convexity == EdgeConvexity::Concave {
            self.saw_concave = true;
        }
    }
}
