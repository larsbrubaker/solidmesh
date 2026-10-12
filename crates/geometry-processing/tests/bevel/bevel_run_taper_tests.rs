//! Port of `R/Tests/GeometryProcessingTests/BevelRunTaperTests.cs`: the run-out a selected open
//! run walks onto the unselected edges past its free ends ([`BevelRunTaper`]) and the tapered
//! sweep along it (`bevel_tapered_sweep`), and a run built through `bevel_group_tools` tapering
//! only its free end.

use std::sync::Arc;

use geometry_processing::bevel::bevel_group_tools::{try_build, BevelGroupToolsOptions};
use geometry_processing::bevel::{bevel_chain_linker, bevel_tapered_sweep};
use geometry_processing::bevel::{
    BevelChainEnds, BevelRunTaper, BevelSurfaceLabels, BevelSweptSection,
};
use geometry_processing::topology::bevel_edge_group_resolver::create_from_edge;
use geometry_processing::topology::FeatureEdgeSet;
use geometry_processing::topology::{BevelEdgeGroupResolution, BevelSelectionKind};
use polygon_mesh::csg::boolean_processing::{classify_boolean_operand, BooleanOperandVerdict};
use polygon_mesh::solids::platonic_solids;
use vector_math::{Matrix4X4, Vector3};

use crate::topology_test_meshes::create_cylinder;

#[test]
fn selected_neighbor_or_branch_blocks_only_its_end() {
    for branch in [false, true] {
        let mesh = create_cylinder(10.0, 10.0, 24);
        let features = FeatureEdgeSet::create(&mesh);
        let edge = *features
            .edges()
            .iter()
            .find(|e| e.start.z == 5.0 && e.end.z == 5.0)
            .expect("a top rim edge");
        let chain = bevel_chain_linker::link(&[edge]).remove(0);
        let to_vertex = chain.edges[0].to_vertex();
        let neighbour = *features
            .edges()
            .iter()
            .find(|e| {
                e.mesh_edge_index != edge.mesh_edge_index
                    && (e.start_vertex == to_vertex || e.end_vertex == to_vertex)
                    && ((e.start.z != e.end.z) == branch)
            })
            .expect("a neighbour at the run's end");
        let mut labels = BevelSurfaceLabels::new(&mesh, None);
        let taper = BevelRunTaper::create(
            &chain,
            &features,
            Some(labels.at(30.0)),
            &[edge, neighbour],
            30.0,
            0.5,
            0.5,
        );
        let taper = taper.expect("the free start still runs out");
        assert_eq!(taper.end_length, 0.0, "branch {branch}");
        assert_eq!(taper.start_length, 0.5, "branch {branch}");
    }
}

#[test]
fn hard_corners_keep_the_conventional_sweep() {
    let mesh = platonic_solids::create_cube(20.0, 20.0, 20.0);
    let features = FeatureEdgeSet::create(&mesh);
    let edge = features.edges()[0];
    let mut labels = BevelSurfaceLabels::new(&mesh, None);
    let chain = bevel_chain_linker::link(&[edge]).remove(0);
    assert!(BevelRunTaper::create(
        &chain,
        &features,
        Some(labels.at(30.0)),
        &[edge],
        30.0,
        0.5,
        0.5
    )
    .is_none());
}

#[test]
fn runout_length_is_physical_and_tips_close_cleanly() {
    for (sides, round) in [
        (24, true),
        (96, true),
        (192, true),
        (24, false),
        (96, false),
        (192, false),
    ] {
        let mesh = create_cylinder(10.0, 10.0, sides);
        let features = FeatureEdgeSet::create(&mesh);
        let edge = *features
            .edges()
            .iter()
            .find(|e| e.start.z == 5.0 && e.end.z == 5.0)
            .expect("a top rim edge");
        let chain = bevel_chain_linker::link(&[edge]).remove(0);
        let mut labels = BevelSurfaceLabels::new(&mesh, None);
        let taper = BevelRunTaper::create(
            &chain,
            &features,
            Some(labels.at(30.0)),
            &[edge],
            30.0,
            0.5,
            0.5,
        )
        .expect("the run tapers");
        let tool = bevel_tapered_sweep::try_create(
            &taper,
            &BevelChainEnds::overshoot(),
            0.5,
            if round { 8 } else { 1 },
            round,
            BevelSweptSection::Tool,
            0.0,
        )
        .unwrap_or_else(|failure| panic!("{sides} {round}: {failure:?}"));
        assert_eq!(
            classify_boolean_operand(&tool, Matrix4X4::IDENTITY, false),
            BooleanOperandVerdict::Clean
        );
        assert!(tool.is_manifold());
        for face in tool.faces.iter() {
            let a = Vector3::from_vector3_float(tool.vertices[face.v0 as usize]);
            let b = Vector3::from_vector3_float(tool.vertices[face.v1 as usize]);
            let c = Vector3::from_vector3_float(tool.vertices[face.v2 as usize]);
            assert!((b - a).cross(c - a).length() > 1e-12);
        }
        // Each tip lies exactly half a millimeter along real chords from the selected endpoint,
        // even when that distance spans more than one facet. It is one shared vertex, not a zero
        // ring.
        for start in [true, false] {
            let edges: Vec<_> = if start {
                taper.edges[..taper.selected_index as usize]
                    .iter()
                    .rev()
                    .map(|e| e.reversed())
                    .collect()
            } else {
                taper.edges[taper.selected_index as usize + 1..].to_vec()
            };
            let mut remaining = 0.5;
            let mut tip = Vector3::ZERO;
            for part in &edges {
                if remaining <= (part.to() - part.from()).length() {
                    tip = part.from() + part.direction() * remaining;
                    break;
                }
                remaining -= (part.to() - part.from()).length();
            }
            let at_tip = tool
                .vertices
                .iter()
                .filter(|&&v| (Vector3::from_vector3_float(v) - tip).length() < 1e-6)
                .count();
            assert_eq!(at_tip, 1, "{sides} {round} start {start}");
        }
    }
}

#[test]
fn the_conventional_end_keeps_its_overshoot_bound() {
    let mesh = create_cylinder(10.0, 10.0, 24);
    let features = FeatureEdgeSet::create(&mesh);
    let edge = *features
        .edges()
        .iter()
        .find(|e| e.start.z == 5.0 && e.end.z == 5.0)
        .expect("a top rim edge");
    let chain = bevel_chain_linker::link(&[edge]).remove(0);
    let mut labels = BevelSurfaceLabels::new(&mesh, None);
    let mut taper = BevelRunTaper::create(
        &chain,
        &features,
        Some(labels.at(30.0)),
        &[edge],
        30.0,
        0.5,
        0.5,
    )
    .expect("the run tapers");
    taper.edges.truncate(taper.selected_index as usize + 1);
    taper.end_length = 0.0;
    let ends = BevelChainEnds {
        end_maximum_overshoot: 0.0,
        ..BevelChainEnds::overshoot()
    };
    let tool =
        bevel_tapered_sweep::try_create(&taper, &ends, 0.5, 8, true, BevelSweptSection::Tool, 0.0)
            .unwrap_or_else(|failure| panic!("{failure:?}"));
    let end = chain.edges[0];
    let furthest = tool
        .vertices
        .iter()
        .map(|&v| (Vector3::from_vector3_float(v) - end.to()).dot(end.direction()))
        .fold(f64::NEG_INFINITY, f64::max);
    assert!(furthest < 1e-5, "{furthest}");
}

#[test]
fn runout_ending_almost_on_a_facet_joint_has_one_tip() {
    let mesh = create_cylinder(10.0, 10.0, 24);
    let features = FeatureEdgeSet::create(&mesh);
    let edge = *features
        .edges()
        .iter()
        .find(|e| e.start.z == 5.0 && e.end.z == 5.0)
        .expect("a top rim edge");
    let length = (edge.end - edge.start).length() * (1.0 + 1e-13);
    let mut labels = BevelSurfaceLabels::new(&mesh, None);
    let taper = BevelRunTaper::create(
        &bevel_chain_linker::link(&[edge]).remove(0),
        &features,
        Some(labels.at(30.0)),
        &[edge],
        30.0,
        length,
        length,
    )
    .expect("the run tapers");
    let tool = bevel_tapered_sweep::try_create(
        &taper,
        &BevelChainEnds::overshoot(),
        0.5,
        8,
        true,
        BevelSweptSection::Tool,
        0.0,
    )
    .unwrap_or_else(|failure| panic!("{failure:?}"));
    assert!(tool.is_manifold());
}

/// C# `AFreeEndRunsOutEvenWhereTheOtherEndIsSharedWithAnotherPick`: a rim facet picked beside
/// another pick runs out at its free end and keeps the overshoot at the shared one.
#[test]
fn a_free_end_runs_out_even_where_the_other_end_is_shared_with_another_pick() {
    let mesh = create_cylinder(10.0, 10.0, 40);
    let features = Arc::new(FeatureEdgeSet::create(&mesh));
    let seed = *features
        .edges()
        .iter()
        .find(|e| e.start.z == 5.0 && e.end.z == 5.0)
        .expect("a top rim edge");
    let chain = bevel_chain_linker::link(&[seed]).remove(0);

    // The rim edge next door, picked in the same operation but not in this group - which is
    // exactly how G2 stands beside G3 in the fixture.
    let to_vertex = chain.edges[0].to_vertex();
    let neighbour = *features
        .edges()
        .iter()
        .find(|e| {
            e.mesh_edge_index != seed.mesh_edge_index
                && e.start.z == 5.0
                && e.end.z == 5.0
                && (e.start_vertex == to_vertex || e.end_vertex == to_vertex)
        })
        .expect("a neighbouring rim edge");

    let group = create_from_edge(
        &features,
        seed.mesh_edge_index,
        BevelSelectionKind::Edge,
        30.0,
    )
    .expect("the rim edge makes a group");
    let resolution = BevelEdgeGroupResolution {
        group,
        features: Arc::clone(&features),
        failure_reason: None,
        seed_edge_index: seed.mesh_edge_index,
        edges: vec![seed],
        chain: None,
        face_group: None,
    };

    let mut swept: Option<BevelRunTaper> = None;
    let mut labels = BevelSurfaceLabels::new(&mesh, None);
    let selected = [seed, neighbour];
    let mut sweep = |taper: &BevelRunTaper, ends: &BevelChainEnds, section| {
        swept = Some(taper.clone());
        bevel_tapered_sweep::try_create(taper, ends, 2.0, 8, true, section, 0.0).ok()
    };
    let built = try_build(
        &resolution,
        true,
        2.0,
        8,
        BevelGroupToolsOptions {
            selected_across_groups: Some(&selected),
            surface_labels: Some(&mut labels),
            sweep_taper: Some(&mut sweep),
            ..Default::default()
        },
    );

    let tools = match built {
        Ok(built) => built.tools,
        Err(refusal) => panic!("{:?}", refusal.reason),
    };
    let swept = swept.expect(
        "the free end of this pick meets nothing but unselected rim, which is the run-out's own case",
    );
    assert!(
        swept.start_length > 0.0,
        "the FREE end - the one no other pick stands at - is the end that runs out"
    );
    assert_eq!(
        swept.end_length, 0.0,
        "and the shared end keeps the overshoot the two picks are meant to overlap across"
    );
    assert_eq!(tools.len(), 1);
}
