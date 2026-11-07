//! A concave apex crease whose end walls lean too far to cap on runs on under them instead of
//! squaring off, with room enough to reach them.

use polygon_mesh::csg::boolean_processing::do_array;
use polygon_mesh::csg::{BooleanOptions, CsgModes};
use polygon_mesh::solids::platonic_solids::create_cube;
use polygon_mesh::{CancelToken, Mesh};

use geometry_processing::bevel::bevel_chain_end_cap::ResolveOptions;
use geometry_processing::bevel::{bevel_chain_end_budget, bevel_chain_linker, BevelChainEnds};
use geometry_processing::topology::{ChainEdge, EdgeConvexity, FeatureEdge, FeatureEdgeSet};
use vector_math::{Matrix4X4, Vector3};

const SIZE: f64 = 20.0;
const APEX_Y: f64 = 10.0;
const APEX_Z: f64 = 10.0;
const APEX_MIN_X: f64 = 5.0;
const APEX_MAX_X: f64 = 15.0;
const RADIUS: f64 = 8.130264802279754;
const SAME_PLACE: f64 = 1e-4;

fn end_wall_agreement() -> f64 {
    2.0 / 5f64.sqrt()
}

fn reach_the_wall_needs() -> f64 {
    RADIUS / 2f64.sqrt() / 2.0
}

/// Both ends run on under the end wall instead of squaring off.
#[test]
fn both_ends_run_on_under_the_end_wall_instead_of_squaring_off() {
    let (ends, _, _) = apex_crease_ends();
    for (name, cap) in [("the start", &ends.start), ("the far end", &ends.end)] {
        assert!(
            !cap.is_capped(),
            "{name} capped on a plane with normal {:?}",
            cap.plane_normal
        );
    }

    for (name, bound) in [
        ("the start", ends.start_maximum_overshoot),
        ("the far end", ends.end_maximum_overshoot),
    ] {
        assert!(bound > reach_the_wall_needs(), "{name} was given {bound}");
    }
}

/// The end wall cannot be capped on and cannot be an exit either.
#[test]
fn the_end_wall_cannot_be_capped_on_and_cannot_be_an_exit_either() {
    let (ends, _, _) = apex_crease_ends();
    let (run, features) = apex_crease();
    let budget = bevel_chain_end_budget::minimum_bead_cap_agreement(RADIUS, &run[0]);
    assert!(end_wall_agreement() < budget, "the segment asks {budget}");
    assert!(ends.start_exit_planes.is_none());
    assert!(features.count() > 0);
}

fn apex_crease_ends() -> (BevelChainEnds, Vector3, Vector3) {
    let (run, features) = apex_crease();
    let start_vertex = run[0].from();
    let end_vertex = run[run.len() - 1].to();
    let selected: Vec<FeatureEdge> = run.iter().map(|edge| edge.edge).collect();
    let mut warnings = Vec::new();
    let ends = BevelChainEnds::resolve(
        &run,
        false,
        RADIUS,
        Some(&features),
        Some(&selected),
        Some(&mut warnings),
        ResolveOptions::default(),
    );
    (ends, start_vertex, end_vertex)
}

fn apex_crease() -> (Vec<ChainEdge>, FeatureEdgeSet) {
    let part = create_notched_cube();
    let features = FeatureEdgeSet::create(&part);
    let apex: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .copied()
        .filter(|edge| {
            edge.convexity == EdgeConvexity::Concave
                && on_the_apex(edge.start)
                && on_the_apex(edge.end)
        })
        .collect();
    assert!(
        !apex.is_empty(),
        "the notch's apex crease is not in this mesh"
    );

    let run = bevel_chain_linker::link(&apex).remove(0).edges;
    (run, features)
}

fn on_the_apex(point: Vector3) -> bool {
    (point.y - APEX_Y).abs() < SAME_PLACE && (point.z - APEX_Z).abs() < SAME_PLACE
}

fn create_notched_cube() -> Mesh {
    let mut cube = create_cube(SIZE, SIZE, SIZE);
    cube.translate(Vector3::new(SIZE / 2.0, SIZE / 2.0, SIZE / 2.0));

    let mouth0 = Vector3::new(0.0, 0.0, 0.0);
    let mouth1 = Vector3::new(SIZE, 0.0, 0.0);
    let mouth2 = Vector3::new(SIZE, 0.0, SIZE);
    let mouth3 = Vector3::new(0.0, 0.0, SIZE);
    let apex0 = Vector3::new(APEX_MIN_X, APEX_Y, APEX_Z);
    let apex1 = Vector3::new(APEX_MAX_X, APEX_Y, APEX_Z);

    let mut notch = Mesh::new();
    for (a, b, c) in [
        (mouth0, mouth2, mouth3),
        (mouth0, mouth1, mouth2),
        (mouth0, apex1, mouth1),
        (mouth0, apex0, apex1),
        (mouth3, mouth2, apex1),
        (mouth3, apex1, apex0),
        (mouth0, mouth3, apex0),
        (mouth1, apex1, mouth2),
    ] {
        notch.create_face(&[a, b, c]);
    }
    notch.clean_and_merge();

    do_array(
        &[(&cube, Matrix4X4::IDENTITY), (&notch, Matrix4X4::IDENTITY)],
        CsgModes::Subtract,
        &CancelToken::new(),
        &BooleanOptions::default(),
    )
    .expect("the notch cuts the cube")
}
