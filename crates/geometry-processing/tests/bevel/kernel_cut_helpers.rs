//! The feature edge between two known positions ([`edge_index`]) and the one-edge group picked
//! there ([`edge_group`]), which the tests that pick a known edge share.

use vector_math::Vector3;

use geometry_processing::topology::bevel_edge_group_resolver;
use geometry_processing::topology::{
    BevelEdgeGroup, BevelSelectionKind, FaceGroup, FeatureEdgeSet,
};

/// The feature edge running between two known positions, whichever way round the
/// mesh stores it, matched `within` that distance at both ends.
pub fn edge_index(features: &FeatureEdgeSet, from: Vector3, to: Vector3, within: f64) -> i32 {
    for (i, edge) in features.edges().iter().enumerate() {
        if ((edge.start - from).length() < within && (edge.end - to).length() < within)
            || ((edge.start - to).length() < within && (edge.end - from).length() < within)
        {
            return i as i32;
        }
    }

    panic!("the fixture has no feature edge from {from:?} to {to:?}");
}

/// The edge pick of the feature edge running between two known positions (matched
/// within 1e-5).
pub fn edge_group(features: &FeatureEdgeSet, from: Vector3, to: Vector3) -> BevelEdgeGroup {
    bevel_edge_group_resolver::create_from_edge(
        features,
        edge_index(features, from, to, 1e-5),
        BevelSelectionKind::Edge,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    )
    .expect("an edge of the set")
}
