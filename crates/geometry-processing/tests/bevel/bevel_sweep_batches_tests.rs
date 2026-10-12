//! Port of `BevelSweepBatchesTests.cs`: adjacent selected rows of one compatible bucket share one
//! sweep, and every row with its own treatment (a branch, a bead, a corner, a clip, a face rim)
//! stays on its own. C# `[Arguments]` cases are one test each, suffixed with the argument.

use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::bevel_sweep_batches;
use geometry_processing::topology::{
    bevel_edge_group_resolver, pick_interpretations, BevelEdgeGroup, BevelSelectionKind,
    EdgeConvexity, FaceGroup, FeatureEdge, FeatureEdgeSet,
};
use polygon_mesh::solids::platonic_solids;
use polygon_mesh::Mesh;

use crate::topology_test_meshes;

fn group(features: &FeatureEdgeSet, edge: &FeatureEdge) -> BevelEdgeGroup {
    bevel_edge_group_resolver::create_from_edge(
        features,
        pick_interpretations::index_of_mesh_edge(features, edge.mesh_edge_index),
        BevelSelectionKind::Edge,
        30.0,
    )
    .expect("the edge is in the set")
}

fn no_shadows(count: usize) -> Vec<Option<Vec<Mesh>>> {
    vec![None; count]
}

#[test]
fn adjacent_groups_share_one_sweep_and_duplicate_edges_appear_once() {
    let mesh = topology_test_meshes::create_cylinder(10.0, 10.0, 24);
    let features = FeatureEdgeSet::create(&mesh);
    let mut rim: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| edge.start.z == 5.0 && edge.end.z == 5.0)
        .copied()
        .collect();
    // OrderBy is stable, as sort_by is.
    rim.sort_by(|a, b| {
        let angle_a = (a.start.y + a.end.y).atan2(a.start.x + a.end.x);
        let angle_b = (b.start.y + b.end.y).atan2(b.start.x + b.end.x);
        angle_a
            .partial_cmp(&angle_b)
            .expect("rim angles are numbers")
    });
    let mut groups: Vec<BevelEdgeGroup> = rim.iter().map(|edge| group(&features, edge)).collect();
    groups.insert(1, group(&features, &rim[0]));
    let (resolutions, _) = bevel_edge_group_resolver::resolve_all(&groups, &mesh);
    let batches = bevel_sweep_batches::create(
        &resolutions,
        &vec![true; groups.len()],
        &vec![false; groups.len()],
        &no_shadows(groups.len()),
    );
    assert_eq!(batches.len(), 1);
    assert_eq!(batches[0].group_indices.len(), 25);
    assert_eq!(batches[0].chains.len(), 1);
    assert!(batches[0].chains[0].is_closed);
    assert_eq!(batches[0].chains[0].edges.len(), 24);
}

fn a_selected_branch_outside_the_bucket_still_stops_merging(condition: &str) {
    let mesh = platonic_solids::create_cube(20.0, 20.0, 20.0);
    let features = FeatureEdgeSet::create(&mesh);
    let vertex = features.edges()[0].start_vertex;
    let mut groups: Vec<BevelEdgeGroup> = features
        .edges()
        .iter()
        .filter(|edge| edge.start_vertex == vertex || edge.end_vertex == vertex)
        .map(|edge| group(&features, edge))
        .collect();
    assert_eq!(groups.len(), 3);
    if condition == "curve" {
        groups[2].curve_tolerance_degrees = 20.0;
    }
    let (resolutions, _) = bevel_edge_group_resolver::resolve_all(&groups, &mesh);
    let caps = [false, false, condition == "cap"];
    let shadows = vec![
        None,
        None,
        if condition == "shadow" {
            Some(vec![mesh.clone()])
        } else {
            None
        },
    ];
    assert_eq!(
        bevel_sweep_batches::create(&resolutions, &[true, true, true], &caps, &shadows).len(),
        0,
        "the third selected run owns a corner even when it cannot join this sweep batch"
    );
}

#[test]
fn a_selected_branch_outside_the_bucket_still_stops_merging_curve() {
    a_selected_branch_outside_the_bucket_still_stops_merging("curve");
}

#[test]
fn a_selected_branch_outside_the_bucket_still_stops_merging_cap() {
    a_selected_branch_outside_the_bucket_still_stops_merging("cap");
}

#[test]
fn a_selected_branch_outside_the_bucket_still_stops_merging_shadow() {
    a_selected_branch_outside_the_bucket_still_stops_merging("shadow");
}

#[test]
fn a_mixed_convexity_run_keeps_its_separate_treatments() {
    let mesh = topology_test_meshes::create_l_shaped_prism();
    let features = FeatureEdgeSet::create(&mesh);
    let edges = features.edges();
    let pair = edges
        .iter()
        .flat_map(|first| edges.iter().map(move |second| (*first, *second)))
        .find(|(first, second)| {
            first.convexity != second.convexity
                && bevel_chain_linker::link(&[*first, *second]).len() == 1
        })
        .expect("the L has a convex edge running into its concave one");
    let groups = vec![group(&features, &pair.0), group(&features, &pair.1)];
    let (resolutions, _) = bevel_edge_group_resolver::resolve_all(&groups, &mesh);
    assert_eq!(
        bevel_sweep_batches::create(&resolutions, &[true, true], &[false; 2], &no_shadows(2)).len(),
        0
    );
}

#[test]
fn a_branch_keeps_its_independent_runs() {
    let mesh = platonic_solids::create_cube(20.0, 20.0, 20.0);
    let features = FeatureEdgeSet::create(&mesh);
    let vertex = features.edges()[0].start_vertex;
    let groups: Vec<BevelEdgeGroup> = features
        .edges()
        .iter()
        .filter(|edge| edge.start_vertex == vertex || edge.end_vertex == vertex)
        .map(|edge| group(&features, edge))
        .collect();
    assert_eq!(groups.len(), 3);
    let (resolutions, _) = bevel_edge_group_resolver::resolve_all(&groups, &mesh);
    assert_eq!(
        bevel_sweep_batches::create(
            &resolutions,
            &[true, true, true],
            &[false; 3],
            &no_shadows(3)
        )
        .len(),
        0
    );
}

fn incompatible_or_individually_clipped_rows_are_not_batched(condition: &str) {
    let mesh = platonic_solids::create_cube(20.0, 20.0, 20.0);
    let features = FeatureEdgeSet::create(&mesh);
    let first = features.edges()[0];
    let second = *features
        .edges()
        .iter()
        .find(|edge| {
            edge.mesh_edge_index != first.mesh_edge_index
                && (edge.start_vertex == first.start_vertex
                    || edge.end_vertex == first.start_vertex)
        })
        .expect("a cube edge shares its start with two others");
    let mut groups = vec![group(&features, &first), group(&features, &second)];
    if condition == "curve" {
        groups[1].curve_tolerance_degrees = 20.0;
    }
    if condition == "flat" {
        groups[1].flat_angle_degrees = 10.0;
    }
    if condition == "unresolved" {
        groups[1].anchor_direction = Default::default();
    }
    if condition == "face" {
        let face =
            FaceGroup::create_with_graph(&mesh, &mesh.get_mesh_edge_graph(), second.face0, 30.0)
                .expect("the face is on the cube");
        groups[1] = bevel_edge_group_resolver::create_from_face_group(
            &face,
            &second,
            features.flat_angle_degrees(),
        )
        .expect("the edge bounds its own face");
    }
    let (resolutions, _) = bevel_edge_group_resolver::resolve_all(&groups, &mesh);
    let eligible = [true, condition != "ineligible"];
    let caps = [false, condition == "cap"];
    let shadows = vec![
        None,
        if condition == "shadow" {
            Some(vec![mesh.clone()])
        } else {
            None
        },
    ];
    let batches = bevel_sweep_batches::create(&resolutions, &eligible, &caps, &shadows);
    assert_eq!(batches.len(), 0, "{condition}");
}

#[test]
fn incompatible_or_individually_clipped_rows_are_not_batched_ineligible() {
    incompatible_or_individually_clipped_rows_are_not_batched("ineligible");
}

#[test]
fn incompatible_or_individually_clipped_rows_are_not_batched_unresolved() {
    incompatible_or_individually_clipped_rows_are_not_batched("unresolved");
}

#[test]
fn incompatible_or_individually_clipped_rows_are_not_batched_cap() {
    incompatible_or_individually_clipped_rows_are_not_batched("cap");
}

#[test]
fn incompatible_or_individually_clipped_rows_are_not_batched_shadow() {
    incompatible_or_individually_clipped_rows_are_not_batched("shadow");
}

#[test]
fn incompatible_or_individually_clipped_rows_are_not_batched_curve() {
    incompatible_or_individually_clipped_rows_are_not_batched("curve");
}

#[test]
fn incompatible_or_individually_clipped_rows_are_not_batched_flat() {
    incompatible_or_individually_clipped_rows_are_not_batched("flat");
}

#[test]
fn incompatible_or_individually_clipped_rows_are_not_batched_face() {
    incompatible_or_individually_clipped_rows_are_not_batched("face");
}

/// Two CONCAVE rows meeting at a vertex stay two rows, however gently the run turns there. A bead
/// adds, so the corner between two of them is plan 5.3's - both overshoot and their union fills
/// it - and each overshoot is then held to that corner's own void; a merged row mitres the corner
/// instead and carries one wedge that knows nothing about it. Both fixtures are asked because the
/// two ways of getting this wrong are different: the boss's feet turn a right angle (the mitre
/// underfills the corner) and the post's chords turn 7.5 degrees (the mitre looks right and the
/// clip still loses the corner it was to be held to).
fn concave_rows_meeting_at_a_vertex_are_not_batched(turn: &str) {
    let mesh = if turn == "right angle" {
        topology_test_meshes::create_square_boss_on_plate()
    } else {
        topology_test_meshes::create_boss_on_slab(6.0, 48, 3.0)
    };
    let features = FeatureEdgeSet::create_with_angle(&mesh, 20.0);
    let feet: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| edge.convexity == EdgeConvexity::Concave)
        .copied()
        .collect();
    let first = feet[0];
    let second = *feet
        .iter()
        .find(|edge| {
            edge.mesh_edge_index != first.mesh_edge_index
                && (edge.start_vertex == first.end_vertex
                    || edge.end_vertex == first.end_vertex
                    || edge.start_vertex == first.start_vertex
                    || edge.end_vertex == first.start_vertex)
        })
        .expect("the foot loop has a neighbouring edge");
    let groups = vec![group(&features, &first), group(&features, &second)];
    let (resolutions, _) = bevel_edge_group_resolver::resolve_all(&groups, &mesh);
    assert_eq!(
        bevel_sweep_batches::create(&resolutions, &[true, true], &[false; 2], &no_shadows(2)).len(),
        0,
        "{turn}"
    );
}

#[test]
fn concave_rows_meeting_at_a_vertex_are_not_batched_right_angle() {
    concave_rows_meeting_at_a_vertex_are_not_batched("right angle");
}

#[test]
fn concave_rows_meeting_at_a_vertex_are_not_batched_gentle() {
    concave_rows_meeting_at_a_vertex_are_not_batched("gentle");
}

/// A joint between two rows that turns further than their own curve tolerance is a CORNER, and a
/// corner is not something one mitred sweep may swallow: the two runs meet their faces at
/// different angles there, a rolling ball leaves the higher of their two cylinders, and the mitre's
/// bisector cuts below it. Unmerged, the rows meet as 5.3's pair corner and each cutter is trimmed
/// by the other's tube instead.
#[test]
fn convex_rows_meeting_at_a_corner_are_not_batched() {
    let mesh = topology_test_meshes::create_raked_l_block(15.0);
    let features = FeatureEdgeSet::create(&mesh);
    let rim: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| {
            edge.convexity == EdgeConvexity::Convex
                && edge.start.z.abs() < 1e-6
                && edge.end.z.abs() < 1e-6
                && (edge.start.length() < 1e-6 || edge.end.length() < 1e-6)
        })
        .copied()
        .collect();
    assert_eq!(
        rim.len(),
        2,
        "the raked L has exactly two top edges running out of its inner corner"
    );
    let groups: Vec<BevelEdgeGroup> = rim.iter().map(|edge| group(&features, edge)).collect();
    let (resolutions, _) = bevel_edge_group_resolver::resolve_all(&groups, &mesh);
    assert_eq!(
        bevel_sweep_batches::create(&resolutions, &[true, true], &[false; 2], &no_shadows(2)).len(),
        0,
        "these two runs turn 90 degrees into each other, which is a corner and not a curve"
    );
}
