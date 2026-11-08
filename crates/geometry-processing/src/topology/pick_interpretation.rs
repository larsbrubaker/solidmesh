//! One candidate reading of a pick ([`PickInterpretation`]) and what kind of reading it is
//! ([`PickInterpretationKind`]); [`super::pick_interpretations`] builds them and
//! [`super::bevel_edge_readings`] rebuilds them from a stored group.

use std::sync::Arc;

use super::bevel_edge_group::BevelEdgeGroup;
use super::feature_edge::FeatureEdge;
use super::feature_edge_chain::FeatureEdgeChain;
use super::feature_edge_set::FeatureEdgeSet;

/// What one candidate reading of a pick selects.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PickInterpretationKind {
    /// The straight crease the picked edge lies in - the picked edge and every mesh edge that
    /// carries it on through a vertex. This is what a plain edge pick means, and on a mesh whose
    /// crease was never split it is the picked edge on its own.
    CollinearRun,
    /// The tangent run the picked edge lies in, at one rung of the tolerance ladder.
    TangentChain,
    /// The whole rim of one of the faces the pick touches - "bevel the top of this box". One
    /// reading per face and never both at once: the two sides of an edge are two different
    /// selections, and a click meant one of them.
    FaceGroup,
}

/// One reading of a pick: a set of edges, the tolerance that produced it, and the group that
/// would be stored if the user settled on this reading.
#[derive(Debug, Clone)]
pub struct PickInterpretation {
    kind: PickInterpretationKind,
    features: Arc<FeatureEdgeSet>,
    seed_edge_index: i32,
    edges: Vec<FeatureEdge>,
    chain: Option<FeatureEdgeChain>,
    curve_tolerance_degrees: f64,
    group: BevelEdgeGroup,
    stops_where_the_shape_does: bool,
}

impl PickInterpretation {
    /// A reading (public because the tests that pin how a list of readings labels itself build
    /// lists the pick never would); `stops_where_the_shape_does` defaults to true, and
    /// `Self::with_stops_where_the_shape_does` sets it.
    pub fn new(
        kind: PickInterpretationKind,
        features: Arc<FeatureEdgeSet>,
        seed_edge_index: i32,
        edges: Vec<FeatureEdge>,
        chain: Option<FeatureEdgeChain>,
        curve_tolerance_degrees: f64,
        group: BevelEdgeGroup,
    ) -> Self {
        PickInterpretation {
            kind,
            features,
            seed_edge_index,
            edges,
            chain,
            curve_tolerance_degrees,
            group,
            stops_where_the_shape_does: true,
        }
    }

    /// The reading with [`Self::stops_where_the_shape_does`] set, as it is built.
    pub(crate) fn with_stops_where_the_shape_does(mut self, stops: bool) -> Self {
        self.stops_where_the_shape_does = stops;
        self
    }

    /// What this reading selects.
    pub fn kind(&self) -> PickInterpretationKind {
        self.kind
    }

    /// The feature edges this reading was derived in, built at [`Self::flat_angle_degrees`].
    /// [`Self::seed_edge_index`] indexes it.
    pub fn features(&self) -> &Arc<FeatureEdgeSet> {
        &self.features
    }

    /// How far the surface had to turn for an edge to be an edge in this reading, in degrees.
    /// This is the number that makes the readings differ from one another.
    pub fn flat_angle_degrees(&self) -> f64 {
        self.features.flat_angle_degrees()
    }

    /// How far the surface may turn and still be walked through, in degrees - the tangent
    /// tolerance the chain was propagated at. The same for every reading; the ladder varies the
    /// flat tolerance, not this.
    pub fn curve_tolerance_degrees(&self) -> f64 {
        self.curve_tolerance_degrees
    }

    /// The picked edge in this reading's own set, which renumbers between readings.
    pub fn seed_edge_index(&self) -> i32 {
        self.seed_edge_index
    }

    /// The edges this reading selects, in walk order for a chain.
    pub fn edges(&self) -> &[FeatureEdge] {
        &self.edges
    }

    /// The chain, with its ends and stop reasons; `None` for a collinear run or face rim
    /// reading.
    pub fn chain(&self) -> Option<&FeatureEdgeChain> {
        self.chain.as_ref()
    }

    /// The group to store on the operation if the user commits to this reading. It captures both
    /// tolerances, so resolving it on a rebuilt mesh re-derives these same edges.
    pub fn group(&self) -> &BevelEdgeGroup {
        &self.group
    }

    /// How many edges this reading selects - what the chooser puts in front of the user
    /// ("1 edge", "64 edges").
    pub fn count(&self) -> usize {
        self.edges.len()
    }

    /// Whether this reading ends where the SHAPE ends rather than where the tessellation of it
    /// does. True for a closed loop, for a run that stopped for any reason other than a junction,
    /// and for one whose stop vertices are still junctions when the part is read at the ladder's
    /// coarsest rung. False - only ever for a [`PickInterpretationKind::TangentChain`] - where
    /// the walk was stopped by the facet joins of a curved surface, which is a reading of the
    /// mesh and not of the part.
    ///
    /// The rounded slot is what this is for. Its top rim is two straight sides and two 12
    /// faceted arcs, and the arc facets turn 15 degrees while the straight-to-arc join turns 7.5.
    /// So at the ladder's TEN degree rung the walk from a straight side runs one facet into each
    /// arc and stops, giving a three edge reading nobody has ever meant: an arc facet either side
    /// of the side that was clicked. Picking an arc facet has no such rung at all - its chain is
    /// itself at half a degree and at ten - so the same loop read from an arc gives the whole rim
    /// and read from a straight side gave three edges, and the fillet the user asked for either
    /// appeared or did not depending on which facet the cursor was over. See
    /// [`super::pick_interpretations::index_of_preferred_chain`].
    pub fn stops_where_the_shape_does(&self) -> bool {
        self.stops_where_the_shape_does
    }
}
