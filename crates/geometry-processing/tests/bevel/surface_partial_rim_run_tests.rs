//! Rung 5 of the polygon-space bevel plan, the partial rim
//! run - two and four contiguous rim edges of a 64 sided cylinder, where the chain stops in the
//! middle of a run at BOTH ends and turns a corner in between.
//!
//! The bug this pins: band ends were attached only to the chain's FIRST and LAST edge, so an
//! interior edge's cut was a half space unbounded along its own direction, which took a whole
//! circular segment off the disc. Every edge now carries an end at each of its own two corners.
//!
//! The mitre correction: a convex polyline's mitred inward band of width `w` has area
//! `w L - w^2 tan(gamma/2)` per interior vertex, so integrating over the profile gives
//! `V = S L - (k - 1) tan(gamma / 2) M`, with `S = integral w dy` the section and
//! `M = integral w^2 dy`. Both are read off the CHORDED profile, so the tessellation is in the
//! expectation rather than in the tolerance. The parameter rows are a loop.

use std::sync::Arc;

use solidmesh::bevel::bevel_chain_linker;
use solidmesh::bevel::surface::{surface_bevel_builder, surface_mesh_closure, SurfaceBevelPlan};
use solidmesh::topology::{FaceGroup, FaceRegionCache, FeatureEdge, FeatureEdgeSet};
use solidmesh_math::Vector2;
use solidmesh_mesh::solids::platonic_solids;
use solidmesh_mesh::{CancelToken, Mesh};

use crate::mesh_measurements::{shell_count, signed_volume};
use crate::surface_walkback_checks::area_on;
use crate::topology_test_meshes::create_cylinder;

const RADIUS: f64 = 10.0;

const HEIGHT: f64 = 10.0;

const SIDES: i32 = 64;

const MEASURED: f64 = 1e-4;

/// A run of contiguous rim edges, filleted. Two edges is the smallest run with a mitre in it; four
/// proves the mitre is not a special case of "every edge is an end".
#[test]
fn a_filleted_rim_run_stops_at_its_own_mitres() {
    for edges in [2, 4] {
        assert_the_run_is_bounded(edges, 2.0, true, 8);
    }
}

/// The same run chamfered, where the profile is one chord and the mitres are unchanged.
#[test]
fn a_chamfered_rim_run_stops_at_its_own_mitres() {
    for edges in [2, 4] {
        assert_the_run_is_bounded(edges, 2.0, false, 1);
    }
}

/// The other shape a run of three has: three edges of a CUBE's top rim, open, so that the two
/// corners in the middle are mitres and the two at the ends are notches - the crease really
/// finishes there, and the end face gives up the section the profile cuts from its corner.
///
/// A notched end carries no band end at all: the band is meant to run out to the region's own rim.
/// So the closed form here is three prisms less the two corners the mitres count twice, and
/// nothing about the ends appears in it.
#[test]
fn a_three_edge_run_of_a_cubes_top_mitres_twice_and_notches_twice() {
    for (round_profile, segments) in [(true, 8), (false, 1)] {
        let cube = 20.0;
        let size = 3.0;

        let built = build_on_a_cube(cube, size, round_profile, segments);
        let profile = profile(size, round_profile, segments);

        assert!(built.is_manifold(), "round {round_profile}");
        assert_eq!(shell_count(&built), 1, "round {round_profile}");
        let (open, open_at) = surface_mesh_closure::open_edge_count(&built);
        assert_eq!(
            open, 0,
            "round {round_profile}: the first edge with other than two faces on it is at {open_at:?}"
        );

        // Three prisms the full width of the cube, less the two corners where adjacent runs would
        // each take the same material - the same integral of the width squared rung 2 uses at four
        // corners, with tan(45) = 1 because a cube's rim turns a right angle.
        let removed = (cube * cube * cube) - signed_volume(&built);
        let expected = (3.0 * cube * section(&profile)) - (2.0 * second_moment(&profile));
        assert!(
            (removed - expected).abs() <= 1e-4,
            "round {round_profile}: the notched ends take their full prism and the two mitres \
             share their corners - {removed}, expected {expected}"
        );
    }
}

fn build_on_a_cube(cube: f64, size: f64, round_profile: bool, segments: i32) -> Mesh {
    let mesh = platonic_solids::create_cube(cube, cube, cube);
    let features = FeatureEdgeSet::create(&mesh);
    let graph = mesh.get_mesh_edge_graph();
    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(mesh.clone()),
        Some(graph.clone()),
        features.flat_angle_degrees(),
    ));

    // Three of the four, walked end to end so that the run is open with a mitre in the middle.
    let run = contiguous_run(&features, cube / 2.0, 3);

    let chains = bevel_chain_linker::link(&run);
    assert_eq!(chains.len(), 1);
    let chain = &chains[0];

    assert!(
        !chain.is_closed && chain.edges.len() == 3,
        "the run is {} edges and closed is {}",
        chain.edges.len(),
        chain.is_closed
    );

    let plan = SurfaceBevelPlan::try_create(chain, &regions, size, round_profile)
        .unwrap_or_else(|reason| panic!("the plan refused the run: {:?}", reason.code()));

    surface_bevel_builder::try_build_one(
        &mesh,
        &graph,
        &regions,
        plan,
        segments,
        &CancelToken::new(),
    )
    .unwrap_or_else(|reason| panic!("the build refused the run: {reason:?}"))
}

fn assert_the_run_is_bounded(edges: i32, size: f64, round_profile: bool, segments: i32) {
    let row = format!("{edges} edges, round {round_profile}");
    let built = build(edges, size, round_profile, segments);

    assert!(
        built.is_manifold(),
        "{row}: the strip, its mitres and its two end walls closed the gap the walkback opened"
    );
    assert_eq!(shell_count(&built), 1, "{row}");
    let (open, open_at) = surface_mesh_closure::open_edge_count(&built);
    assert_eq!(
        open, 0,
        "{row}: every edge of the answer is shared by two faces, and the first that is not is at \
         {open_at:?}"
    );

    let profile = profile(size, round_profile, segments);
    let edges = edges as f64;

    let removed = cylinder_volume() - signed_volume(&built);
    let expected = (section(&profile) * edges * chord_length())
        - ((edges - 1.0) * mitre_tangent() * second_moment(&profile));
    assert!(
        (removed - expected).abs() <= MEASURED,
        "{row}: the run removes its section over its chords, less one mitre overlap at each \
         interior corner - {removed}, expected {expected}"
    );

    let cap = area_on(&built, |vertex| {
        (vertex.z as f64 - (HEIGHT / 2.0)).abs() < 1e-5
    });
    let expected_cap = polygon_area()
        - ((size * edges * chord_length()) - ((edges - 1.0) * mitre_tangent() * size * size));
    assert!(
        (cap - expected_cap).abs() <= MEASURED,
        "{row}: the cap gave up the mitred band over the run and nothing else - {cap}, expected \
         {expected_cap}"
    );
}

/// The profile, as the removed section's own outline read from the cap down: x in from the wall
/// plane, y down from the cap, from the tangent point on the cap to the tangent point on the wall.
/// A chamfer is the one-chord case, which is why nothing below asks which profile it is.
fn profile(size: f64, round_profile: bool, segments: i32) -> Vec<Vector2> {
    if !round_profile {
        return vec![Vector2::new(size, 0.0), Vector2::new(0.0, size)];
    }

    let mut chorded = Vec::with_capacity(segments as usize + 1);

    for i in 0..=segments {
        // The rolling ball's centre is at (size, size); the quarter arc runs from straight below it
        // to straight beside it, and the mesh's own blend is this same inscribed chording.
        let angle = (3.0 * std::f64::consts::PI / 2.0)
            - (i as f64 * (std::f64::consts::PI / 2.0) / segments as f64);

        chorded.push(Vector2::new(
            size + (size * angle.cos()),
            size + (size * angle.sin()),
        ));
    }

    chorded
}

/// The section the run sweeps: the area under the profile, exactly as it is chorded.
fn section(profile: &[Vector2]) -> f64 {
    let mut summed = 0.0;

    for i in 0..profile.len() - 1 {
        summed += (profile[i + 1].y - profile[i].y) * (profile[i].x + profile[i + 1].x) / 2.0;
    }

    summed
}

/// The section's first moment about the wall, doubled: `integral w^2 dy` - the term one mitre takes
/// out of the swept volume, since the slice at depth y loses `w(y)^2 tan(gamma / 2)`.
fn second_moment(profile: &[Vector2]) -> f64 {
    let mut summed = 0.0;

    for i in 0..profile.len() - 1 {
        let from = profile[i];
        let to = profile[i + 1];

        summed += (to.y - from.y) * ((from.x * from.x) + (from.x * to.x) + (to.x * to.x)) / 3.0;
    }

    summed
}

/// Half the turn at a rim corner, as its tangent: the polygon's exterior angle is 2 pi / n.
fn mitre_tangent() -> f64 {
    (std::f64::consts::PI / SIDES as f64).tan()
}

fn cylinder_volume() -> f64 {
    polygon_area() * HEIGHT
}

fn polygon_area() -> f64 {
    SIDES as f64 / 2.0 * RADIUS * RADIUS * (2.0 * std::f64::consts::PI / SIDES as f64).sin()
}

fn chord_length() -> f64 {
    2.0 * RADIUS * (std::f64::consts::PI / SIDES as f64).sin()
}

fn build(edges: i32, size: f64, round_profile: bool, segments_per_quarter: i32) -> Mesh {
    let cylinder = create_cylinder(RADIUS, HEIGHT, SIDES);
    let features = FeatureEdgeSet::create(&cylinder);
    let graph = cylinder.get_mesh_edge_graph();
    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(cylinder.clone()),
        Some(graph.clone()),
        features.flat_angle_degrees(),
    ));
    let curve_regions = Arc::new(FaceRegionCache::new(
        Arc::new(cylinder.clone()),
        Some(graph.clone()),
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    ));

    let chains = bevel_chain_linker::link(&contiguous_run(&features, HEIGHT / 2.0, edges as usize));
    assert_eq!(chains.len(), 1);
    let chain = &chains[0];

    assert_eq!(
        chain.edges.len(),
        edges as usize,
        "the linker made a chain of {} rather than {edges}",
        chain.edges.len()
    );

    let plan = SurfaceBevelPlan::try_create_with_curve_regions(
        chain,
        &regions,
        Some(&curve_regions),
        size,
        round_profile,
    )
    .unwrap_or_else(|reason| panic!("the plan refused the run: {:?}", reason.code()));

    surface_bevel_builder::try_build_one(
        &cylinder,
        &graph,
        &regions,
        plan,
        segments_per_quarter,
        &CancelToken::new(),
    )
    .unwrap_or_else(|reason| panic!("the build refused the run: {reason:?}"))
}

/// A run of `count` rim edges at height `top` that join end to end, walked off the rim from
/// wherever the first one happens to be (the same walk the cube builds make).
fn contiguous_run(features: &FeatureEdgeSet, top: f64, count: usize) -> Vec<FeatureEdge> {
    let rim: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| (edge.start.z - top).abs() < 1e-6 && (edge.end.z - top).abs() < 1e-6)
        .copied()
        .collect();

    let mut run = vec![rim[0]];
    let mut at = rim[0].end_vertex;

    while run.len() < count {
        let next = *rim
            .iter()
            .find(|edge| {
                (edge.start_vertex == at || edge.end_vertex == at)
                    && !run
                        .iter()
                        .any(|already| already.mesh_edge_index == edge.mesh_edge_index)
            })
            .expect("the rim carries on");

        run.push(next);
        at = if next.start_vertex == at {
            next.end_vertex
        } else {
            next.start_vertex
        };
    }

    run
}
