//! The WINDOW `surface_seam_over_the_edge` picks a run against - the stretch of the strip its own
//! band covers, read where each band end plane crosses the outermost rail. Two ways of reading it
//! wrong are pinned here, both of them arithmetic rather than geometry. Built on one edge of a cube
//! because the window is a question about numbers along a line and nothing else.

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::surface::surface_seam_over_the_edge::try_pick_the_run_over_the_edge;
use geometry_processing::bevel::surface::{
    SurfaceBandEnd, SurfaceBevelPlan, SurfaceEdgePlan, SurfaceStrip, SurfaceVertexPool,
};
use geometry_processing::topology::{FaceRegionCache, FeatureEdgeSet};
use polygon_mesh::solids::platonic_solids;
use polygon_mesh::Mesh;
use vector_math::Vector3;

const SIZE: f64 = 3.0;

const SEGMENTS: i32 = 3;

/// The cube's edge is twenty long, which is the strip's whole parameter span.
const EDGE_LENGTH: f64 = 20.0;

/// A band end plane that lies nearly ALONG the rail bounds nothing, and must not be read as though
/// it bounded something a hundred thousand millimetres away. `SurfaceStrip::try_cross` turns down a
/// plane the rail runs exactly along, but its guard is an absolute 1e-9 on the slope, so a plane a
/// hundredth of a millionth off square still answers - with a parameter of order 1e8, which as a
/// window start puts the band past the end of everything and refuses every run on the region.
#[test]
fn a_band_end_almost_along_the_rail_does_not_open_the_window_to_infinity() {
    let (read, mut pool, strip) = a_strip_on_a_cube_edge();

    // A plane a hundredth of a millionth off square to the rail, standing a millimetre off it: the
    // slope clears TryCross's guard by an order of magnitude and the crossing it names is 1e8
    // along.
    let askew = ((strip.along() * 1e-8) + read.across).get_normal();

    let ends = vec![SurfaceBandEnd::new(
        strip.point_on(0, 0.0) + read.across,
        askew,
    )];

    let named_at = strip.try_cross(0, ends[0].point, ends[0].inward);
    assert!(
        named_at.is_some(),
        "the fixture only says anything if the crossing really is read rather than refused"
    );
    let named_at = named_at.unwrap_or_default();

    assert!(
        named_at.abs() > 1e6,
        "and if it is read as the absurdity this test is about - instead {named_at}"
    );

    let runs = run_over(&mut pool, &strip, 0.0, EDGE_LENGTH);
    let seam =
        try_pick_the_run_over_the_edge(&pool, &strip, &read.edge, Some(&runs), Some(&ends), true);

    let seam = seam.unwrap_or_else(|refused_at| {
        panic!(
            "a run lying over the whole edge is the one this strip sews to, and a band end plane \
             that bounds nothing may not take it away - instead refused at {refused_at}"
        )
    });

    assert_eq!(seam.len(), 2);
}

/// Two band ends that bound the SAME end of the band - which the mitre planes at a corner can both
/// do - leave the band inside both of them, so the window's start is the FURTHER of the two. Taken
/// last-one-wins, as it was, the window depends on the order the plan happened to list its ends in,
/// and a run that stops short of the real start is sewn to as though it reached.
#[test]
fn two_band_ends_on_one_side_keep_the_tighter_of_them() {
    let (read, mut pool, strip) = a_strip_on_a_cube_edge();

    // Both facing along the walk, so both bound the START: the band really begins at 8.
    let ends = vec![
        SurfaceBandEnd::new(strip.point_on(0, 8.0), strip.along()),
        SurfaceBandEnd::new(strip.point_on(0, 4.0), strip.along()),
    ];

    // A run that stops at 6 - inside the loose end and short of the tight one - so the two readings
    // give opposite answers and the test cannot pass by accident.
    let runs = run_over(&mut pool, &strip, 0.0, 6.0);

    assert!(
        try_pick_the_run_over_the_edge(&pool, &strip, &read.edge, Some(&runs), Some(&ends), true)
            .is_err(),
        "the band begins at 8 and the run stops at 6, so none of the run is under the band"
    );

    let reversed: Vec<SurfaceBandEnd> = ends.iter().rev().cloned().collect();

    assert!(
        try_pick_the_run_over_the_edge(
            &pool,
            &strip,
            &read.edge,
            Some(&runs),
            Some(&reversed),
            true
        )
        .is_err(),
        "and the order the two ends arrive in is no part of where the band begins"
    );
}

/// One run of a tangent line, as two pooled vertices on the left rail.
fn run_over(
    pool: &mut SurfaceVertexPool,
    strip: &SurfaceStrip,
    from: f64,
    to: f64,
) -> Vec<Vec<i32>> {
    vec![vec![
        pool.add(strip.point_on(0, from)),
        pool.add(strip.point_on(0, to)),
    ]]
}

/// The one edge's plan and the in-face direction away from it.
struct CubeEdgeReading {
    edge: SurfaceEdgePlan,
    across: Vector3,
}

/// One top edge of a twenty cube planned and ruled - the plan whose window
/// is being read, the strip it rules, and a direction square to the rail in the face it lies in, to
/// stand a plane off by.
fn a_strip_on_a_cube_edge() -> (CubeEdgeReading, SurfaceVertexPool, SurfaceStrip) {
    let cube = Arc::new(platonic_solids::create_cube_vector(Vector3::new(
        20.0, 20.0, 20.0,
    )));
    let (regions, features) = regions_of(&cube);

    let matching: Vec<usize> = (0..features.count())
        .filter(|&i| {
            let edge = &features.edges()[i];

            (edge.start.y + 10.0).abs() < 1e-5
                && (edge.end.y + 10.0).abs() < 1e-5
                && edge.start.z > 9.9
                && edge.end.z > 9.9
        })
        .collect();

    assert_eq!(matching.len(), 1, "exactly one top edge at y = -10");

    let mut chains = bevel_chain_linker::link(&[features.edges()[matching[0]]]);

    assert_eq!(chains.len(), 1, "exactly one chain");

    let chain = chains.remove(0);

    let plan: Rc<SurfaceBevelPlan> = SurfaceBevelPlan::try_create(&chain, &regions, SIZE, true)
        .unwrap_or_else(|reason| panic!("the plan refused the edge: {reason:?}"));

    let strip = SurfaceStrip::try_create(&plan.edges[0], SEGMENTS)
        .unwrap_or_else(|reason| panic!("the strip refused to rule: {reason:?}"));

    let pool = SurfaceVertexPool::new(&cube.vertices);

    (
        CubeEdgeReading {
            edge: plan.edges[0].clone(),
            across: plan.edges[0].left_inward,
        },
        pool,
        strip,
    )
}

/// The part's feature edges and its region cache at their flat angle.
fn regions_of(mesh: &Arc<Mesh>) -> (Arc<FaceRegionCache>, FeatureEdgeSet) {
    let features = FeatureEdgeSet::create(mesh);
    let regions = FaceRegionCache::new(
        mesh.clone(),
        Some(mesh.get_mesh_edge_graph()),
        features.flat_angle_degrees(),
    );

    (Arc::new(regions), features)
}
