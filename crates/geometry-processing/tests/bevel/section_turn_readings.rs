//! The corner readings the `Section*` owner-file classes share (`SectionEdgeLoopAndFaceTests`,
//! `SectionInnerFaceTests`): the sharp turns of the plans a build made,
//! the creases standing on a vertex, and the dihedral angle of a feature or chain edge. One copy
//! here, so the classes read their corners the same way.

use std::rc::Rc;

use geometry_processing::bevel::sections::section_corner;
use geometry_processing::bevel::surface::SurfaceBevelPlan;
use geometry_processing::topology::{ChainEdge, FeatureEdge, FeatureEdgeSet};
use polygon_mesh::Mesh;
use vector_math::math_helper::radians_to_degrees;

/// Every vertex of the plans where the chain turns sharper than the curve
/// tolerance, as its arriving and leaving edges.
pub fn the_sharp_turns_of(plans: &[Rc<SurfaceBevelPlan>]) -> Vec<(ChainEdge, ChainEdge)> {
    let mut turns = Vec::new();

    for plan in plans {
        let count = plan.edges.len();

        for i in 0..count {
            let arriving = plan.edges[(i + count - 1) % count].edge;
            let leaving = plan.edges[i].edge;

            if section_corner::is_sharp_turn(&arriving, &leaving) {
                turns.push((arriving, leaving));
            }
        }
    }

    turns
}

/// Every feature edge of the part that stands on a vertex, read at the
/// operation's bend threshold.
pub fn the_creases_at(part: &Mesh, vertex: i32, bend_threshold_degrees: f64) -> Vec<FeatureEdge> {
    let features = FeatureEdgeSet::create_with_graph(
        part,
        &part.get_mesh_edge_graph(),
        bend_threshold_degrees,
    );

    features
        .edges()
        .iter()
        .filter(|edge| edge.start_vertex == vertex || edge.end_vertex == vertex)
        .cloned()
        .collect()
}

/// The angle between a feature edge's two faces, in degrees.
pub fn feature_dihedral_degrees(edge: &FeatureEdge) -> f64 {
    radians_to_degrees(
        edge.normal0
            .get_normal()
            .dot(edge.normal1.get_normal())
            .clamp(-1.0, 1.0)
            .acos(),
    )
}

/// The angle between a chain edge's two faces, in degrees.
pub fn chain_dihedral_degrees(edge: &ChainEdge) -> f64 {
    radians_to_degrees(
        edge.left_normal()
            .get_normal()
            .dot(edge.right_normal().get_normal())
            .clamp(-1.0, 1.0)
            .acos(),
    )
}
