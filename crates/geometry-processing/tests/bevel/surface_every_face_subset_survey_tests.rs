//! ON ANY FACE, EVERY SINGLE EDGE AND ANY TWO OR
//! THREE OF ITS EDGES HAVE TO BEVEL - the claim "Every 1, 2 and 3 Face Selection Should Bevel.mcx"
//! makes, surveyed on the kernel's own copy of that part (`nose_in_notch_test_meshes`, the chevron
//! notch with the knife-edge nose in it).
//!
//! A face here is a REGION - the flood of coplanar facets a click on one facet takes - and its edges
//! are that region's rim. Every subset of one, two or three of a region's rim edges is picked as
//! SEPARATE `Edge` groups, and the region's whole rim is taken once as the single group a FACE click
//! stores; deduplicated by the SET of mesh edges a case resolves to, keeping the first region that
//! names it. A RATCHET: what does not build is listed by name and by the advisory it comes back
//! with, and the list may only shrink. The queue at radius 2 holds 12 of the 219 cases (eight
//! `SurfaceMixedCornerRunOutPastTheCrease` on the nose's ridge with a crease that dies on its end,
//! four honest `SizeCollidesAcrossFace` on the two slopes, 1.784 fitting where 2 was asked); radius
//! 1 is empty; radius 3 holds 46.

use std::collections::{HashMap, HashSet};

use geometry_processing::bevel::surface::surface_mesh_closure;
use geometry_processing::topology::{
    bevel_edge_group_resolver, BevelEdgeGroup, BevelSelectionKind, EdgeConvexity, FaceGroup,
    FeatureEdge, FeatureEdgeSet,
};
use polygon_mesh::{CancelToken, Mesh, MeshEdgeGraph};
use vector_math::number_format::format_f64_max_decimals;

use crate::mesh_measurements::{shell_count, signed_volume};
use crate::nose_in_notch_test_meshes;
use crate::section_every_pick_matrix_tests::{kind, point, show};
use crate::surface_test_build::{try_bevel, TryBevelOptions};

/// The fixture's own segment count.
const SEGMENTS: i32 = 16;

/// The queue at the fixture's own radius 2: 12 of the 219 cases, in two families (see the module
/// header; measured 2026-09-16 - 207 of the 219 build closed, manifold, one shell and the right way
/// in volume).
const EXPECTED_FAILURES_AT_2: &[(&str, &str)] = &[
    (
        "face (-0.71, -0.71, 0): edges (20, 0, 10), (12.5, 7.5, 11.67), (12.5, 7.5, 8.33)",
        "SizeCollidesAcrossFace",
    ),
    (
        "face (-0.71, -0.71, 0): the whole rim",
        "SizeCollidesAcrossFace",
    ),
    (
        "face (0, -0.55, -0.83): edges (7.5, 7.5, 8.33), (10, 5, 10)",
        "SurfaceMixedCornerRunOutPastTheCrease",
    ),
    (
        "face (0, -0.55, -0.83): edges (10, 5, 10), (12.5, 7.5, 8.33)",
        "SurfaceMixedCornerRunOutPastTheCrease",
    ),
    (
        "face (0, -0.55, -0.83): edges (7.5, 7.5, 8.33), (10, 5, 10), (12.5, 7.5, 8.33)",
        "SurfaceMixedCornerRunOutPastTheCrease",
    ),
    (
        "face (0, -0.55, -0.83): the whole rim",
        "SurfaceMixedCornerRunOutPastTheCrease",
    ),
    (
        "face (0, -0.55, 0.83): edges (7.5, 7.5, 11.67), (10, 5, 10)",
        "SurfaceMixedCornerRunOutPastTheCrease",
    ),
    (
        "face (0, -0.55, 0.83): edges (10, 5, 10), (12.5, 7.5, 11.67)",
        "SurfaceMixedCornerRunOutPastTheCrease",
    ),
    (
        "face (0, -0.55, 0.83): edges (7.5, 7.5, 11.67), (10, 5, 10), (12.5, 7.5, 11.67)",
        "SurfaceMixedCornerRunOutPastTheCrease",
    ),
    (
        "face (0, -0.55, 0.83): the whole rim",
        "SurfaceMixedCornerRunOutPastTheCrease",
    ),
    (
        "face (0.71, -0.71, 0): edges (0, 0, 10), (7.5, 7.5, 11.67), (7.5, 7.5, 8.33)",
        "SizeCollidesAcrossFace",
    ),
    (
        "face (0.71, -0.71, 0): the whole rim",
        "SizeCollidesAcrossFace",
    ),
];

/// The same survey at radius 1. EMPTY, measured 2026-09-16: all 219 cases build.
const EXPECTED_FAILURES_AT_1: &[(&str, &str)] = &[];

/// The same survey at radius 3, where 46 of the 219 refuse - measured 2026-09-16: 34
/// `SizeExceedsFaceWidth`, the 4 `SizeCollidesAcrossFace` of radius 2, and the eight nose-face
/// cases as 4 `SurfaceMixedCornerRunOutPastTheCrease` and 4 `SurfaceCornerNotSupported`.
const EXPECTED_FAILURES_AT_3: &[(&str, &str)] = &[
    (
        "face (-0.71, -0.71, 0): edges (10, 10, 3.33), (20, 0, 10)",
        "SizeExceedsFaceWidth",
    ),
    (
        "face (-0.71, -0.71, 0): edges (20, 0, 10), (10, 10, 16.67)",
        "SizeExceedsFaceWidth",
    ),
    (
        "face (-0.71, -0.71, 0): edges (20, 0, 10), (12.5, 7.5, 11.67)",
        "SizeExceedsFaceWidth",
    ),
    (
        "face (-0.71, -0.71, 0): edges (20, 0, 10), (12.5, 7.5, 8.33)",
        "SizeExceedsFaceWidth",
    ),
    (
        "face (-0.71, -0.71, 0): edges (15, 5, 0), (10, 10, 3.33), (20, 0, 10)",
        "SizeExceedsFaceWidth",
    ),
    (
        "face (-0.71, -0.71, 0): edges (15, 5, 0), (20, 0, 10), (10, 10, 16.67)",
        "SizeExceedsFaceWidth",
    ),
    (
        "face (-0.71, -0.71, 0): edges (15, 5, 0), (20, 0, 10), (12.5, 7.5, 11.67)",
        "SizeExceedsFaceWidth",
    ),
    (
        "face (-0.71, -0.71, 0): edges (15, 5, 0), (20, 0, 10), (12.5, 7.5, 8.33)",
        "SizeExceedsFaceWidth",
    ),
    (
        "face (-0.71, -0.71, 0): edges (10, 10, 3.33), (20, 0, 10), (15, 5, 20)",
        "SizeExceedsFaceWidth",
    ),
    (
        "face (-0.71, -0.71, 0): edges (10, 10, 3.33), (20, 0, 10), (10, 10, 16.67)",
        "SizeExceedsFaceWidth",
    ),
    (
        "face (-0.71, -0.71, 0): edges (10, 10, 3.33), (20, 0, 10), (12.5, 7.5, 11.67)",
        "SizeExceedsFaceWidth",
    ),
    (
        "face (-0.71, -0.71, 0): edges (10, 10, 3.33), (20, 0, 10), (12.5, 7.5, 8.33)",
        "SizeExceedsFaceWidth",
    ),
    (
        "face (-0.71, -0.71, 0): edges (20, 0, 10), (15, 5, 20), (10, 10, 16.67)",
        "SizeExceedsFaceWidth",
    ),
    (
        "face (-0.71, -0.71, 0): edges (20, 0, 10), (15, 5, 20), (12.5, 7.5, 11.67)",
        "SizeExceedsFaceWidth",
    ),
    (
        "face (-0.71, -0.71, 0): edges (20, 0, 10), (15, 5, 20), (12.5, 7.5, 8.33)",
        "SizeExceedsFaceWidth",
    ),
    (
        "face (-0.71, -0.71, 0): edges (20, 0, 10), (10, 10, 16.67), (12.5, 7.5, 11.67)",
        "SizeExceedsFaceWidth",
    ),
    (
        "face (-0.71, -0.71, 0): edges (20, 0, 10), (10, 10, 16.67), (12.5, 7.5, 8.33)",
        "SizeExceedsFaceWidth",
    ),
    (
        "face (-0.71, -0.71, 0): edges (20, 0, 10), (12.5, 7.5, 11.67), (12.5, 7.5, 8.33)",
        "SizeCollidesAcrossFace",
    ),
    (
        "face (-0.71, -0.71, 0): the whole rim",
        "SizeCollidesAcrossFace",
    ),
    (
        "face (0, -0.55, -0.83): edges (7.5, 7.5, 8.33), (10, 5, 10)",
        "SurfaceMixedCornerRunOutPastTheCrease",
    ),
    (
        "face (0, -0.55, -0.83): edges (10, 5, 10), (12.5, 7.5, 8.33)",
        "SurfaceMixedCornerRunOutPastTheCrease",
    ),
    (
        "face (0, -0.55, -0.83): edges (7.5, 7.5, 8.33), (10, 5, 10), (12.5, 7.5, 8.33)",
        "SurfaceCornerNotSupported",
    ),
    (
        "face (0, -0.55, -0.83): the whole rim",
        "SurfaceCornerNotSupported",
    ),
    (
        "face (0, -0.55, 0.83): edges (7.5, 7.5, 11.67), (10, 5, 10)",
        "SurfaceMixedCornerRunOutPastTheCrease",
    ),
    (
        "face (0, -0.55, 0.83): edges (10, 5, 10), (12.5, 7.5, 11.67)",
        "SurfaceMixedCornerRunOutPastTheCrease",
    ),
    (
        "face (0, -0.55, 0.83): edges (7.5, 7.5, 11.67), (10, 5, 10), (12.5, 7.5, 11.67)",
        "SurfaceCornerNotSupported",
    ),
    (
        "face (0, -0.55, 0.83): the whole rim",
        "SurfaceCornerNotSupported",
    ),
    (
        "face (0.71, -0.71, 0): edges (10, 10, 3.33), (0, 0, 10)",
        "SizeExceedsFaceWidth",
    ),
    (
        "face (0.71, -0.71, 0): edges (0, 0, 10), (10, 10, 16.67)",
        "SizeExceedsFaceWidth",
    ),
    (
        "face (0.71, -0.71, 0): edges (0, 0, 10), (7.5, 7.5, 11.67)",
        "SizeExceedsFaceWidth",
    ),
    (
        "face (0.71, -0.71, 0): edges (0, 0, 10), (7.5, 7.5, 8.33)",
        "SizeExceedsFaceWidth",
    ),
    (
        "face (0.71, -0.71, 0): edges (5, 5, 0), (10, 10, 3.33), (0, 0, 10)",
        "SizeExceedsFaceWidth",
    ),
    (
        "face (0.71, -0.71, 0): edges (5, 5, 0), (0, 0, 10), (10, 10, 16.67)",
        "SizeExceedsFaceWidth",
    ),
    (
        "face (0.71, -0.71, 0): edges (5, 5, 0), (0, 0, 10), (7.5, 7.5, 11.67)",
        "SizeExceedsFaceWidth",
    ),
    (
        "face (0.71, -0.71, 0): edges (5, 5, 0), (0, 0, 10), (7.5, 7.5, 8.33)",
        "SizeExceedsFaceWidth",
    ),
    (
        "face (0.71, -0.71, 0): edges (10, 10, 3.33), (0, 0, 10), (5, 5, 20)",
        "SizeExceedsFaceWidth",
    ),
    (
        "face (0.71, -0.71, 0): edges (10, 10, 3.33), (0, 0, 10), (10, 10, 16.67)",
        "SizeExceedsFaceWidth",
    ),
    (
        "face (0.71, -0.71, 0): edges (10, 10, 3.33), (0, 0, 10), (7.5, 7.5, 11.67)",
        "SizeExceedsFaceWidth",
    ),
    (
        "face (0.71, -0.71, 0): edges (10, 10, 3.33), (0, 0, 10), (7.5, 7.5, 8.33)",
        "SizeExceedsFaceWidth",
    ),
    (
        "face (0.71, -0.71, 0): edges (0, 0, 10), (5, 5, 20), (10, 10, 16.67)",
        "SizeExceedsFaceWidth",
    ),
    (
        "face (0.71, -0.71, 0): edges (0, 0, 10), (5, 5, 20), (7.5, 7.5, 11.67)",
        "SizeExceedsFaceWidth",
    ),
    (
        "face (0.71, -0.71, 0): edges (0, 0, 10), (5, 5, 20), (7.5, 7.5, 8.33)",
        "SizeExceedsFaceWidth",
    ),
    (
        "face (0.71, -0.71, 0): edges (0, 0, 10), (10, 10, 16.67), (7.5, 7.5, 11.67)",
        "SizeExceedsFaceWidth",
    ),
    (
        "face (0.71, -0.71, 0): edges (0, 0, 10), (10, 10, 16.67), (7.5, 7.5, 8.33)",
        "SizeExceedsFaceWidth",
    ),
    (
        "face (0.71, -0.71, 0): edges (0, 0, 10), (7.5, 7.5, 11.67), (7.5, 7.5, 8.33)",
        "SizeCollidesAcrossFace",
    ),
    (
        "face (0.71, -0.71, 0): the whole rim",
        "SizeCollidesAcrossFace",
    ),
];

/// Every subset of one, two and three of every face region's rim edges, picked as separate Edge
/// groups, plus each region's whole rim as the one group a face click stores.
fn every_subset_of_one_two_or_three_of_a_faces_edges_is_on_the_queue(radius: f64) {
    let part = nose_in_notch_test_meshes::create();
    let graph = part.get_mesh_edge_graph();
    let features = FeatureEdgeSet::create(&part);

    let cases = cases(&part);

    // The extent of the survey, pinned: nine regions whose rims are 7, 7, 5, 5, 4, 4, 4, 3 and 3
    // edges, which is 232 subsets of one, two and three, 22 of them the same selection reached from
    // the region on the other side of a shared edge, and nine whole rims - 219 distinct cases.
    assert_eq!(
        cases.len(),
        219,
        "a survey that stopped covering what it covered is not the same survey: {}",
        regions(&part, &graph, &features)
            .iter()
            .map(|(name, region)| format!("{name} rim {}", region.boundary_edges().len()))
            .collect::<Vec<_>>()
            .join("; ")
    );

    let token = CancelToken::new();
    let mut said: HashMap<String, String> = HashMap::new();

    for case in &cases {
        if let Some(complaint) = complaint_about(&part, &case.groups, case.expected, radius, &token)
        {
            said.insert(case.name.clone(), complaint);
        }
    }

    let names: Vec<&str> = cases.iter().map(|case| case.name.as_str()).collect();

    is_on_the_queue(&names, &said, queue_at(radius));
}

#[test]
fn every_subset_of_one_two_or_three_of_a_faces_edges_is_on_the_queue_1() {
    every_subset_of_one_two_or_three_of_a_faces_edges_is_on_the_queue(1.0);
}

#[test]
fn every_subset_of_one_two_or_three_of_a_faces_edges_is_on_the_queue_2() {
    every_subset_of_one_two_or_three_of_a_faces_edges_is_on_the_queue(2.0);
}

#[test]
fn every_subset_of_one_two_or_three_of_a_faces_edges_is_on_the_queue_3() {
    every_subset_of_one_two_or_three_of_a_faces_edges_is_on_the_queue(3.0);
}

/// The recorded queue for one radius.
fn queue_at(radius: f64) -> &'static [(&'static str, &'static str)] {
    if radius == 1.0 {
        return EXPECTED_FAILURES_AT_1;
    }

    if radius == 3.0 {
        return EXPECTED_FAILURES_AT_3;
    }

    EXPECTED_FAILURES_AT_2
}

/// The four things the recorded queue is held to: no stale names, nothing listed that passes,
/// nothing unlisted that fails, and nothing listed failing a NEW way.
fn is_on_the_queue(
    names: &[&str],
    said: &HashMap<String, String>,
    expected_failures: &[(&str, &str)],
) {
    let known: HashSet<&str> = names.iter().copied().collect();

    let gone_from_the_fixture: Vec<&str> = expected_failures
        .iter()
        .map(|(name, _)| *name)
        .filter(|name| !known.contains(name))
        .collect();
    assert!(
        gone_from_the_fixture.is_empty(),
        "the queue names cases this part no longer has, so the list is stale: {}",
        gone_from_the_fixture.join("; ")
    );

    let now_passing: Vec<&str> = expected_failures
        .iter()
        .map(|(name, _)| *name)
        .filter(|name| !said.contains_key(*name))
        .collect();
    assert!(
        now_passing.is_empty(),
        "these build now - remove them from the queue, which is the only way it ever shrinks: {}",
        now_passing.join("; ")
    );

    // In case order, so the report reads the same on every run.
    let newly_failing: Vec<String> = names
        .iter()
        .filter(|name| said.contains_key(**name))
        .filter(|name| !expected_failures.iter().any(|(queued, _)| queued == *name))
        .map(|name| format!("{name} -> {}", said[*name]))
        .collect();
    assert!(
        newly_failing.is_empty(),
        "nothing outside the queue may fail:\n{}",
        newly_failing.join("\n")
    );

    let failing_differently: Vec<String> = expected_failures
        .iter()
        .filter_map(|&(name, expected)| {
            let complaint = said.get(name)?;

            (kind(complaint) != expected).then(|| {
                format!(
                    "{name} was '{expected}' and is now '{}': {complaint}",
                    kind(complaint)
                )
            })
        })
        .collect();
    assert!(
        failing_differently.is_empty(),
        "a queued case failing a NEW way is a regression, not the queue:\n{}",
        failing_differently.join("\n")
    );
}

/// One build of one selection, as one sentence, or `None` if it built a closed manifold single
/// shell that moved the volume the way its picks allow.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x < y)` on purpose: a NaN volume fails as well.
fn complaint_about(
    part: &Mesh,
    groups: &[BevelEdgeGroup],
    expected: WhichWayTheVolumeGoes,
    radius: f64,
    token: &CancelToken,
) -> Option<String> {
    let outcome = try_bevel(
        part,
        groups,
        radius,
        SEGMENTS,
        token,
        TryBevelOptions::default(),
    );

    let mesh = match outcome.mesh_or_reason() {
        Ok(mesh) => mesh,
        Err(Some(reason)) => {
            return Some(format!(
                "{:?} at {}",
                reason.code(),
                reason.args().first().map_or("no point".to_string(), show)
            ));
        }
        Err(None) => return Some(format!("declined: {:?}", outcome.result.as_ref().err())),
    };

    let mut complaints: Vec<String> = Vec::new();

    let (open, open_at) = surface_mesh_closure::open_edge_count(mesh);
    if open != 0 {
        complaints.push(format!("left open at {}", point(open_at)));
    }

    if !mesh.is_manifold() {
        complaints.push("is not manifold".to_string());
    }

    let shells = shell_count(mesh);

    if shells != 1 {
        complaints.push(format!("came out as {shells} shells"));
    }

    // Which way the volume may go is the SELECTION's business, not the part's: a concave crease
    // is filleted by ADDING material and a convex edge by taking it off.
    let volume = signed_volume(mesh);
    let part_volume = nose_in_notch_test_meshes::VOLUME;

    if expected == WhichWayTheVolumeGoes::Down && !(volume < part_volume) {
        complaints.push(format!(
            "came out at {volume:.3}, not under the part's {part_volume:.3}, though every edge \
             picked is convex"
        ));
    }

    if expected == WhichWayTheVolumeGoes::Up && !(volume > part_volume) {
        complaints.push(format!(
            "came out at {volume:.3}, not over the part's {part_volume:.3}, though every edge \
             picked is concave"
        ));
    }

    (!complaints.is_empty()).then(|| complaints.join(" / "))
}

/// One case of the survey: its name, the groups it picks and which way its volume may go.
struct Case {
    name: String,
    groups: Vec<BevelEdgeGroup>,
    expected: WhichWayTheVolumeGoes,
}

/// Every case the fixture's claim names: per face region, every subset of one, two and three of
/// its rim edges as separate Edge groups, then the region's whole rim as one face group.
/// Deduplicated by the set of mesh edges the selection resolves to, first region to name it keeping
/// it.
fn cases(part: &Mesh) -> Vec<Case> {
    let graph = part.get_mesh_edge_graph();
    let features = FeatureEdgeSet::create(part);

    let mut cases = Vec::new();
    let mut already: HashSet<String> = HashSet::new();

    for (name, region) in regions(part, &graph, &features) {
        let rim = region.boundary_edges();

        for size in 1..=rim.len().min(3) {
            for chosen in combinations(rim.len(), size) {
                let edges: Vec<i32> = chosen
                    .iter()
                    .map(|&index| rim[index].mesh_edge_index)
                    .collect();
                let mut sorted = edges.clone();
                sorted.sort_unstable();

                let key = format!(
                    "edges {}",
                    sorted
                        .iter()
                        .map(|edge| edge.to_string())
                        .collect::<Vec<_>>()
                        .join(",")
                );
                if !already.insert(key) {
                    continue;
                }

                let groups: Vec<BevelEdgeGroup> = edges
                    .iter()
                    .map(|&edge| single_edge_group(&features, edge))
                    .collect();

                cases.push(Case {
                    name: format!(
                        "{name}: edges {}",
                        groups
                            .iter()
                            .map(|group| point(group.anchor_midpoint))
                            .collect::<Vec<_>>()
                            .join(", ")
                    ),
                    groups,
                    expected: which_way(chosen.iter().map(|&index| &rim[index])),
                });
            }
        }

        // And the whole rim as the ONE group a face click stores, which is a different group from
        // the same edges picked one at a time even where the edges are the same.
        cases.push(Case {
            name: format!("{name}: the whole rim"),
            groups: vec![bevel_edge_group_resolver::create_from_face_group(
                &region,
                &rim[0],
                features.flat_angle_degrees(),
            )
            .expect("a rim edge bounds its own region")],
            expected: which_way(rim.iter()),
        });
    }

    cases
}

/// The part's DISTINCT face regions, flooded at the feature set's own flat angle so a region is
/// what a click on one of its facets would take, named by the outward normal they share. Ordered by
/// name (ordinal, stable) so the survey walks them the same way whatever order the facets come in.
fn regions(
    part: &Mesh,
    graph: &MeshEdgeGraph,
    features: &FeatureEdgeSet,
) -> Vec<(String, FaceGroup)> {
    let mut seen: HashSet<i32> = HashSet::new();
    let mut regions: Vec<(String, FaceGroup)> = Vec::new();

    for face in 0..part.faces.len() as i32 {
        if seen.contains(&face) {
            continue;
        }

        let region = FaceGroup::create_with_graph(part, graph, face, features.flat_angle_degrees())
            .expect("every facet seeds a region");

        seen.extend(region.faces().iter().copied());

        let normal = part.faces[face as usize].normal;

        regions.push((
            format!(
                "face ({}, {}, {})",
                format_f64_max_decimals(normal.x as f64, 2),
                format_f64_max_decimals(normal.y as f64, 2),
                format_f64_max_decimals(normal.z as f64, 2)
            ),
            region,
        ));
    }

    regions.sort_by(|a, b| a.0.cmp(&b.0));

    regions
}

/// Which way a selection of these edges may move the part's volume.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum WhichWayTheVolumeGoes {
    /// Convex and concave both picked: the two pull against each other and neither wins by rule.
    EitherWay,
    /// Every edge picked is convex, so the bevel only ever cuts.
    Down,
    /// Every edge picked is concave, so the fillet only ever adds.
    Up,
}

/// Which way this set of edges may move the volume - see [`WhichWayTheVolumeGoes`].
fn which_way<'a>(edges: impl Iterator<Item = &'a FeatureEdge>) -> WhichWayTheVolumeGoes {
    let picked: Vec<&FeatureEdge> = edges.collect();

    if picked
        .iter()
        .all(|edge| edge.convexity == EdgeConvexity::Convex)
    {
        return WhichWayTheVolumeGoes::Down;
    }

    if picked
        .iter()
        .all(|edge| edge.convexity == EdgeConvexity::Concave)
    {
        return WhichWayTheVolumeGoes::Up;
    }

    WhichWayTheVolumeGoes::EitherWay
}

/// One `Edge` group on the named mesh edge - the group a click makes when the selection mode is
/// one edge at a time.
fn single_edge_group(features: &FeatureEdgeSet, mesh_edge_index: i32) -> BevelEdgeGroup {
    let index = features
        .edges()
        .iter()
        .position(|edge| edge.mesh_edge_index == mesh_edge_index)
        .unwrap_or_else(|| {
            panic!(
                "mesh edge {mesh_edge_index} bounds a face region but is no feature edge, so no \
                 click could select it"
            )
        });

    bevel_edge_group_resolver::create_from_edge(
        features,
        index as i32,
        BevelSelectionKind::Edge,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    )
    .expect("the edge is in the set")
}

/// The index combinations of one size, in lexicographic order - the odometer walk, which comes out
/// grouped by size without sorting.
fn combinations(count: usize, size: usize) -> Vec<Vec<usize>> {
    let mut all = Vec::new();

    if size == 0 || size > count {
        return all;
    }

    let mut chosen: Vec<usize> = (0..size).collect();

    loop {
        all.push(chosen.clone());

        let mut digit = size as i32 - 1;

        while digit >= 0 && chosen[digit as usize] == count - size + digit as usize {
            digit -= 1;
        }

        if digit < 0 {
            return all;
        }

        let digit = digit as usize;
        chosen[digit] += 1;

        for after in digit + 1..size {
            chosen[after] = chosen[after - 1] + 1;
        }
    }
}
