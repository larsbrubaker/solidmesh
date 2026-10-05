//! Building a run's sections chain vertex by chain vertex (`Bevel/Sections/SectionBuilder.cs`, the
//! un-dotted file of the C# `internal static partial class SectionBuilder`; a module of free
//! functions here, with the walked-side partial in [`super::section_builder_walked_side`]). What is
//! here are the readings the build starts from: the operation's own feature test
//! ([`the_feature_test`]), the feature edge between two chain vertices
//! ([`try_feature_edge_between`]) and the vertex-pair index of the mesh's edges
//! ([`edges_by_vertex_pair`]); the creases a side's fan crosses ([`try_creases_in_the_fan`]); and the
//! profile's interior samples on the canals' circle or corner ellipse
//! ([`place_the_interior_samples`], [`try_the_canals_corner`]). The build itself (`TryBuild`) lands
//! with a later package.

use std::collections::HashSet;
use std::rc::Rc;

use polygon_mesh::{Mesh, MeshEdgeGraph};
use vector_math::Vector3;

use super::section_fan_walk::{self, EdgeAtPair};
use super::{IsAFeatureEdge, NanSectionPoint, SectionPool};
use crate::bevel::BevelAdvisory;
use crate::topology::{ChainEdge, FeatureEdge, FeatureEdgeSet};

/// C# `TheFeatureTest`: the operation's OWN reading of which mesh edges are features - never a
/// coplanarity test, which on a tessellated wall calls every fold a feature and stops a surface walk
/// at the first one (see `SectionSurfaceWalk`). The set is only looked up, so its order cannot
/// reach an output.
pub fn the_feature_test(features: &FeatureEdgeSet) -> IsAFeatureEdge {
    let mesh_edges: HashSet<i32> = features
        .edges()
        .iter()
        .map(|feature| feature.mesh_edge_index)
        .collect();

    Rc::new(move |edge| mesh_edges.contains(&edge))
}

/// C# `TryFeatureEdgeBetween`: the feature edge joining two mesh vertices, oriented to walk from
/// `from` to `to`.
pub fn try_feature_edge_between(
    features: &FeatureEdgeSet,
    from: i32,
    to: i32,
) -> Option<ChainEdge> {
    for &index in features.edges_at_vertex(from) {
        let edge = features.edges()[index as usize];

        if edge.start_vertex == from && edge.end_vertex == to {
            return Some(ChainEdge::new(index, edge, false));
        }

        if edge.end_vertex == from && edge.start_vertex == to {
            return Some(ChainEdge::new(index, edge, true));
        }
    }

    None
}

/// C# `EdgesByVertexPair`: every mesh edge, looked up by the pair of vertices it joins.
pub fn edges_by_vertex_pair(graph: &MeshEdgeGraph) -> EdgeAtPair {
    section_fan_walk::edges_by_vertex_pair(graph)
}

/// C# `SmallestWedgeCosine`: how nearly two normals may line up before the vertex has no wedge to
/// draw a section in. Below this the centre and setback formulas both blow up.
pub const SMALLEST_WEDGE_COSINE: f64 = -0.999999;

/// C# `TryCreasesInTheFan` (the overload with no feature test): the creases among the edges one side
/// of the vertex's fan crosses, read by coplanarity.
#[allow(clippy::too_many_arguments)]
pub fn try_creases_in_the_fan(
    vertex: i32,
    arriving: &ChainEdge,
    leaving: &ChainEdge,
    on_the_left: bool,
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
) -> Result<Vec<i32>, BevelAdvisory> {
    try_creases_in_the_fan_reading(
        vertex,
        arriving,
        leaving,
        on_the_left,
        mesh,
        graph,
        edge_at_pair,
        None,
    )
    .map(|(creases, _)| creases)
}

/// C# `TryCreasesInTheFan` (with the feature test): the fan one side of the vertex turns through
/// ([`section_fan_walk::try_walk`]) and the creases among the edges it crosses - by the operation's
/// own feature reading where one is in hand (any edge without two faces counting), by coplanarity
/// where none is - as `(creases, fan)`.
#[allow(clippy::too_many_arguments)]
pub fn try_creases_in_the_fan_reading(
    vertex: i32,
    arriving: &ChainEdge,
    leaving: &ChainEdge,
    on_the_left: bool,
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    is_a_feature_edge: Option<&dyn Fn(i32) -> bool>,
) -> Result<(Vec<i32>, Vec<i32>), BevelAdvisory> {
    let walked = section_fan_walk::try_walk(
        mesh,
        graph,
        edge_at_pair,
        vertex,
        arriving,
        leaving,
        on_the_left,
        Vector3::from_vector3_float(mesh.vertices[vertex as usize]),
    )?;

    let creases = walked
        .crossed
        .iter()
        .copied()
        .filter(|&edge| match is_a_feature_edge {
            None => section_fan_walk::is_a_crease(mesh, graph, edge),
            Some(is_a_feature) => graph.get_face_count(edge) != 2 || is_a_feature(edge),
        })
        .collect();

    Ok((creases, walked.faces))
}

/// C# `PlaceTheInteriorSamples`: the profile's interior samples (all but its two ends), added to the
/// pool. Where the two canals meet at a corner ([`try_the_canals_corner`]) the arc is the ELLIPSE the
/// corner's leaning canal cuts in the section's plane - the arms squashed across the lean, slerped,
/// and stretched back - with the two boundaries' along-chain drift riding the axis; elsewhere it is
/// the circle about the centres, slerped from `left` to `right` through `wedge`. `Err` where a sample
/// is not a number (where C# throws).
#[allow(clippy::too_many_arguments)]
pub fn place_the_interior_samples(
    arriving: &ChainEdge,
    leaving: &ChainEdge,
    tangent: Vector3,
    left: Vector3,
    right: Vector3,
    wedge: f64,
    radius: f64,
    left_at: Vector3,
    right_at: Vector3,
    left_centre: Vector3,
    right_centre: Vector3,
    pool: &mut SectionPool,
    profile: &mut [i32],
) -> Result<(), NanSectionPoint> {
    let last = profile.len() - 1;

    if let Some((corner, across, squash)) =
        try_the_canals_corner(arriving, leaving, radius, tangent)
    {
        // The two boundaries may stand at different along-chain positions; that drift rides the axis,
        // and only the arm swung about it is the ellipse's.
        let from_axis = corner - (tangent * (corner - left_centre).dot(tangent));
        let to_axis = corner - (tangent * (corner - right_centre).dot(tangent));

        let from_arm = squashed(left_at - from_axis, across, squash).get_normal();
        let to_arm = squashed(right_at - to_axis, across, squash).get_normal();

        let swept = from_arm.dot(to_arm).clamp(-1.0, 1.0).acos();

        for (k, slot) in profile.iter_mut().enumerate().take(last).skip(1) {
            let along = k as f64 / last as f64;
            let arm = slerp(from_arm, to_arm, along, swept);

            *slot = pool.add(
                from_axis
                    + ((to_axis - from_axis) * along)
                    + squashed(arm * radius, across, 1.0 / squash),
            )?;
        }

        return Ok(());
    }

    for (k, slot) in profile.iter_mut().enumerate().take(last).skip(1) {
        let along = k as f64 / last as f64;
        let swept_to = slerp(left, right, along, wedge);

        *slot =
            pool.add(left_centre + ((right_centre - left_centre) * along) + (swept_to * radius))?;
    }

    Ok(())
}

/// C# `TryTheCanalsCorner`: where the two edges' canal AXES come nearest - the corner the section's
/// ellipse is drawn about - with the unit direction the arriving axis leans off the section's normal
/// (`across`) and the cosine of that lean (`squash`). `None` where the chain does not turn here, turns
/// back on itself, an edge has no axis, or the two axes run parallel.
pub fn try_the_canals_corner(
    arriving: &ChainEdge,
    leaving: &ChainEdge,
    radius: f64,
    tangent: Vector3,
) -> Option<(Vector3, Vector3, f64)> {
    let arriving_way = arriving.direction();
    let leaving_way = leaving.direction();

    // How far either axis leans off the section's own normal: the chain's half turn here.
    let cos_beta = arriving_way.dot(tangent).clamp(-1.0, 1.0);
    let leaning = arriving_way - (tangent * cos_beta);
    let sin_beta = leaning.length();

    if sin_beta < 1e-9 || cos_beta < 1e-9 {
        return None;
    }

    let across = leaning / sin_beta;
    let squash = cos_beta;

    // Each edge's own axis: the line a radius clear of BOTH of that edge's faces, run along it.
    let arriving_at = try_the_axis_at(arriving, radius)?;
    let leaving_at = try_the_axis_at(leaving, radius)?;

    let apart = arriving_at - leaving_at;
    let lean = arriving_way.dot(leaving_way);
    let denominator = (lean * lean) - 1.0;

    if denominator.abs() < 1e-12 {
        return None;
    }

    let toward = ((lean * apart.dot(arriving_way)) - apart.dot(leaving_way)) / denominator;
    let back = (toward * lean) - apart.dot(arriving_way);

    let corner =
        ((arriving_at + (arriving_way * back)) + (leaving_at + (leaving_way * toward))) / 2.0;

    Some((corner, across, squash))
}

/// C# `TryTheAxisAt`: a point of one edge's canal axis - a radius clear of both of its faces, at the
/// edge's start - or `None` where the edge's faces are one plane or fold back on each other.
pub fn try_the_axis_at(edge: &ChainEdge, radius: f64) -> Option<Vector3> {
    let cosine = edge.left_normal().dot(edge.right_normal());

    if cosine > FeatureEdge::COPLANAR_FACE_DOT || cosine < SMALLEST_WEDGE_COSINE {
        return None;
    }

    Some(edge.from() - ((edge.left_normal() + edge.right_normal()) * radius / (1.0 + cosine)))
}

/// C# `Squashed`: `arm` with its component `across` scaled by `by`.
pub fn squashed(arm: Vector3, across: Vector3, by: f64) -> Vector3 {
    arm + (across * ((by - 1.0) * arm.dot(across)))
}

/// C# `SectionBuilder.Slerp`: the direction `along` of the way round from `from` to `to`, `wedge`
/// apart, on the great circle between them; `from` itself where the wedge is no turn at all.
pub fn slerp(from: Vector3, to: Vector3, along: f64, wedge: f64) -> Vector3 {
    let spread = wedge.sin();

    if spread.abs() < 1e-12 {
        return from;
    }

    ((from * ((1.0 - along) * wedge).sin()) + (to * (along * wedge).sin())) / spread
}
