//! Building a whole run's sections (`Sections/SectionBuilder.cs`: the `TryBuild` overloads,
//! `TryBuildRun`, `TryBuildLoop`): an end section at each open end, a section at every vertex
//! between, the split vertices blended, the offsets laid along the surface, the junctions'
//! leaving sections put in and the absorbed splits read - the run the sweep is built from.

use std::collections::HashSet;
use std::rc::Rc;

use indexmap::IndexMap;
use polygon_mesh::{Mesh, MeshEdgeGraph};
use vector_math::Vector3;

use super::section_absorbed_splits::SectionAbsorbedSplits;
use super::section_builder_offsets::lay_the_offsets_along_the_surface;
use super::section_builder_station::try_section;
use super::section_builder_walked_side::SectionBuildRefusal;
use super::section_run::IsAFeatureEdge;
use super::{section_end, section_fan_walk, section_split_blend, BevelSection, JunctionSection};
use super::{SectionPool, SectionRun};
use crate::bevel::surface::SurfaceBevelPlan;
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};
use crate::topology::{ChainEdge, FeatureEdgeChain, FeatureEdgeSet};

fn not_built(args: Vec<crate::bevel::BevelAdvisoryArg>) -> SectionBuildRefusal {
    SectionBuildRefusal::Refused(BevelAdvisory::new(
        BevelAdvisoryCode::SurfaceConstructionNotBuilt,
        args,
    ))
}

/// C# `TryBuild(FeatureEdgeChain chain, ...)`.
#[allow(clippy::too_many_arguments)]
pub fn try_build_chain(
    chain: Option<&FeatureEdgeChain>,
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    pool: &mut SectionPool,
    radius: f64,
    segments: i32,
    blend_split_vertices: bool,
) -> Result<SectionRun, SectionBuildRefusal> {
    try_build(
        chain.map(|c| c.edges()),
        chain.is_some_and(|c| c.is_closed()),
        None,
        mesh,
        graph,
        pool,
        radius,
        segments,
        blend_split_vertices,
        None,
    )
}

/// C# `TryBuildRun`.
#[allow(clippy::too_many_arguments)]
pub fn try_build_run(
    edges: &[ChainEdge],
    is_closed: bool,
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    pool: &mut SectionPool,
    radius: f64,
    segments: i32,
    blend_split_vertices: bool,
    features: Option<&FeatureEdgeSet>,
) -> Result<SectionRun, SectionBuildRefusal> {
    try_build(
        Some(edges),
        is_closed,
        None,
        mesh,
        graph,
        pool,
        radius,
        segments,
        blend_split_vertices,
        features.map(the_feature_test),
    )
}

/// C# `TryBuild(SurfaceBevelPlan plan, ...)`.
#[allow(clippy::too_many_arguments)]
pub fn try_build_plan(
    plan: Option<Rc<SurfaceBevelPlan>>,
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    pool: &mut SectionPool,
    radius: f64,
    segments: i32,
    features: Option<&FeatureEdgeSet>,
) -> Result<SectionRun, SectionBuildRefusal> {
    let edges: Vec<ChainEdge> = plan
        .as_ref()
        .map(|plan| plan.edges.iter().map(|planned| planned.edge).collect())
        .unwrap_or_default();
    let is_closed = plan.as_ref().is_some_and(|plan| plan.is_closed);
    try_build(
        Some(&edges),
        is_closed,
        plan,
        mesh,
        graph,
        pool,
        radius,
        segments,
        true,
        features.map(the_feature_test),
    )
}

/// C# `TryBuildLoop`: the closed run round `loop_vertices`, each step a feature edge.
#[allow(clippy::too_many_arguments)]
pub fn try_build_loop(
    loop_vertices: &[i32],
    features: Option<&FeatureEdgeSet>,
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    pool: &mut SectionPool,
    radius: f64,
    segments: i32,
    blend_split_vertices: bool,
) -> Result<SectionRun, SectionBuildRefusal> {
    let Some(features) = features else {
        return Err(not_built(vec![]));
    };
    if loop_vertices.len() < 3 {
        return Err(not_built(vec![]));
    }

    let mut edges = Vec::with_capacity(loop_vertices.len());
    for i in 0..loop_vertices.len() {
        let from = loop_vertices[i];
        let to = loop_vertices[(i + 1) % loop_vertices.len()];
        let Some(walked) = try_feature_edge_between(features, from, to) else {
            return Err(not_built(vec![Vector3::from_vector3_float(
                mesh.vertices[from as usize],
            )
            .into()]));
        };
        edges.push(walked);
    }

    try_build(
        Some(&edges),
        true,
        None,
        mesh,
        graph,
        pool,
        radius,
        segments,
        blend_split_vertices,
        Some(the_feature_test(features)),
    )
}

/// C# `TheFeatureTest`: whether a mesh edge is one of `features`.
pub fn the_feature_test(features: &FeatureEdgeSet) -> IsAFeatureEdge {
    let mesh_edges: HashSet<i32> = features.edges().iter().map(|f| f.mesh_edge_index).collect();
    Rc::new(move |edge| mesh_edges.contains(&edge))
}

/// C# `TryFeatureEdgeBetween`: the feature edge from `from` to `to`, walked that way.
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

/// C#'s private `TryBuild(edges, isClosed, plan, ...)`: the run, or why it was not built.
#[allow(clippy::too_many_arguments)]
pub fn try_build(
    edges: Option<&[ChainEdge]>,
    is_closed: bool,
    plan: Option<Rc<SurfaceBevelPlan>>,
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    pool: &mut SectionPool,
    radius: f64,
    segments: i32,
    blend_split_vertices: bool,
    is_a_feature_edge: Option<IsAFeatureEdge>,
) -> Result<SectionRun, SectionBuildRefusal> {
    let Some(edges) = edges.filter(|edges| edges.len() >= 3) else {
        return Err(not_built(vec![]));
    };

    let edge_at_pair = section_fan_walk::edges_by_vertex_pair(graph);
    let feature = is_a_feature_edge.as_deref();
    let mut junctions: Option<IndexMap<i32, JunctionSection>> = None;
    let count = if is_closed {
        edges.len()
    } else {
        edges.len() + 1
    };
    let mut sections: Vec<Option<BevelSection>> = vec![None; count];

    if !is_closed {
        sections[0] = Some(section_end::try_build(
            &edges[0],
            true,
            mesh,
            graph,
            &edge_at_pair,
            pool,
            radius,
            segments,
        )?);
        sections[edges.len()] = Some(section_end::try_build(
            &edges[edges.len() - 1],
            false,
            mesh,
            graph,
            &edge_at_pair,
            pool,
            radius,
            segments,
        )?);
    }

    let first = if is_closed { 0 } else { 1 };
    for i in first..edges.len() {
        let leaving = &edges[i];
        let arriving = &edges[(i + edges.len() - 1) % edges.len()];
        if arriving.to_vertex() != leaving.from_vertex() {
            return Err(not_built(vec![leaving.from_vertex().into()]));
        }

        let (section, junction) = try_section(
            arriving,
            leaving,
            mesh,
            graph,
            &edge_at_pair,
            pool,
            radius,
            segments,
            feature,
        )?;
        sections[i] = Some((*section).clone());
        if let Some(junction) = junction {
            junctions
                .get_or_insert_with(IndexMap::new)
                .insert(i as i32, junction);
        }
    }

    let mut sections: Vec<BevelSection> = sections
        .into_iter()
        .map(|section| section.expect("every station has its section"))
        .collect();

    let placed = if blend_split_vertices {
        section_split_blend::apply(&mut sections, mesh, pool)?
    } else {
        // C#'s `new SplitPlacement[n]`: every entry the struct's default (0, 0, 0), which its
        // `IsSplit` (both indices >= 0) reads as SPLIT - kept as C# has it; see the R2.P08 report.
        vec![section_split_blend::SplitPlacement::new(0, 0, 0.0); sections.len()]
    };

    lay_the_offsets_along_the_surface(
        &mut sections,
        Some(&placed),
        edges,
        is_closed,
        mesh,
        graph,
        &edge_at_pair,
        pool,
        feature,
        junctions.as_ref(),
    )?;

    let mut sections: Vec<Rc<BevelSection>> = sections.into_iter().map(Rc::new).collect();
    if let Some(junctions) = &junctions {
        for (&station, junction) in junctions {
            sections[station as usize] = junction.leaving_section.clone();
        }
    }

    let plain: Vec<BevelSection> = sections.iter().map(|s| (**s).clone()).collect();
    let swallowed = SectionAbsorbedSplits::behind_the_corners(&plain, edges, is_closed, pool);
    let absorbed = if section_split_blend::absorb_split_vertices() {
        SectionAbsorbedSplits::from(
            &plain,
            &placed,
            is_closed,
            swallowed.as_ref().map(|s| s.0.as_slice()),
            swallowed.as_ref().map(|s| s.1.as_slice()),
        )
    } else {
        None
    };

    Ok(SectionRun::new(
        sections,
        is_closed,
        edges.to_vec(),
        plan,
        is_a_feature_edge,
        absorbed,
        junctions,
    ))
}
