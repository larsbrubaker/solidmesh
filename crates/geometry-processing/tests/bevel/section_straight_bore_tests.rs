//! The sections of a straight bore's mouth - once per vertex,
//! shared between neighbouring strips, the top's boundary offset by the setback, the wall's riding
//! the vertical crease, every profile point a radius off the canal's axis.

use std::rc::Rc;

use geometry_processing::bevel::sections::bevel_section::SectionSide;
use geometry_processing::bevel::sections::section_builder_run::try_build_chain;
use geometry_processing::bevel::sections::{SectionPool, SectionRun};
use geometry_processing::topology::{FeatureEdgeChain, FeatureEdgeSet};
use polygon_mesh::{Mesh, MeshEdgeGraph};
use vector_math::Vector3;

use crate::bore_test_meshes::create_plate_with_a_round_bore;

const SIDES: i32 = 24;
const BORE_RADIUS: f64 = 3.0;
const THICKNESS: f64 = 10.0;
const RADIUS: f64 = 0.4;
const SEGMENTS: i32 = 8;
const BEND_THRESHOLD_DEGREES: f64 = 20.0;
const AS_NEAR_AS_A_FLOAT_HOLDS: f64 = 1e-6;

struct Built {
    part: Mesh,
    graph: MeshEdgeGraph,
    pool: SectionPool,
    run: SectionRun,
    top_is_on_the_left: bool,
}

fn build() -> Built {
    let part = create_plate_with_a_round_bore(SIDES, BORE_RADIUS, THICKNESS, 6.0);
    let graph = part.get_mesh_edge_graph();
    let features = FeatureEdgeSet::create_with_graph(&part, &graph, BEND_THRESHOLD_DEGREES);
    let chain = FeatureEdgeChain::propagate(&features, the_upper_mouth(&features)).unwrap();
    assert!(
        chain.is_closed() && chain.count() == SIDES as usize,
        "the fixture's mouth walks as a closed loop of {SIDES}"
    );

    let mut pool = SectionPool::new(&part.vertices);
    let run = match try_build_chain(
        Some(&chain),
        &part,
        &graph,
        &mut pool,
        RADIUS,
        SEGMENTS,
        true,
    ) {
        Ok(run) => run,
        Err(failure) => panic!("the straight bore's mouth sections - instead {failure:?}"),
    };
    let first = &run.sections[0];
    let top_is_on_the_left =
        (pool.position_of(first.left_boundary).z - (THICKNESS / 2.0)).abs() < 1e-9;
    Built {
        part,
        graph,
        pool,
        run,
        top_is_on_the_left,
    }
}

#[test]
fn a_straight_bores_mouth_sections_once_per_vertex() {
    let built = build();
    assert_eq!(built.run.sections.len(), SIDES as usize);
    assert!(built.run.is_closed);
    assert!(built.top_is_on_the_left);
    for i in 0..SIDES as usize {
        assert_eq!(
            built.run.sections[i].chain_vertex,
            built.run.edges[i].from_vertex()
        );
    }
}

#[test]
fn each_strips_end_is_its_neighbours_start_by_reference() {
    let run = build().run;
    for i in 0..SIDES {
        let before = (i - 1 + SIDES) % SIDES;
        assert!(Rc::ptr_eq(&run.end_of(before).unwrap(), &run.start_of(i)));
    }
}

#[test]
fn the_top_faces_boundary_is_offset_by_the_setback() {
    let built = build();
    for section in &built.run.sections {
        assert!((section.setback - RADIUS).abs() <= 1e-12);
        let (kind, slide, boundary) = if built.top_is_on_the_left {
            (
                section.left_kind,
                section.left_slide_edge,
                section.left_boundary,
            )
        } else {
            (
                section.right_kind,
                section.right_slide_edge,
                section.right_boundary,
            )
        };
        let at = built.pool.position_of(boundary);
        assert_eq!(kind, SectionSide::Offset);
        assert_eq!(slide, -1);
        let station = section.station;
        let outward = Vector3::new(station.x, station.y, 0.0).get_normal();
        assert!((at - (station + (outward * RADIUS))).length() <= AS_NEAR_AS_A_FLOAT_HOLDS);
    }
}

#[test]
fn the_walls_boundary_rides_the_vertical_crease() {
    let built = build();
    let top = THICKNESS / 2.0;
    for section in &built.run.sections {
        let (kind, slide, boundary) = if built.top_is_on_the_left {
            (
                section.right_kind,
                section.right_slide_edge,
                section.right_boundary,
            )
        } else {
            (
                section.left_kind,
                section.left_slide_edge,
                section.left_boundary,
            )
        };
        let at = built.pool.position_of(boundary);
        assert_eq!(kind, SectionSide::Slide);
        assert!(slide >= 0);
        assert!(the_edge_runs_straight_down(
            &built,
            slide,
            section.chain_vertex
        ));
        assert!((at - (section.station - Vector3::new(0.0, 0.0, RADIUS))).length() <= 1e-12);
        assert!((at.z - (top - RADIUS)).abs() <= 1e-12);
    }
}

#[test]
fn every_profile_point_stands_a_radius_from_the_canals_axis() {
    let built = build();
    let axes = the_canals_axes(&built);
    for section in &built.run.sections {
        assert_eq!(section.profile.len(), (SEGMENTS + 1) as usize);
        assert_eq!(section.profile[0], section.left_boundary);
        assert_eq!(section.profile[SEGMENTS as usize], section.right_boundary);
        for k in 1..SEGMENTS as usize {
            let at = built.pool.position_of(section.profile[k]);
            let stands = axes
                .iter()
                .map(|&(on, way)| off_the_axis(at, on, way))
                .fold(f64::INFINITY, f64::min);
            assert!((stands - RADIUS).abs() <= 1e-9, "it reads {stands}");
        }
    }
}

#[test]
fn the_centre_sits_inside_the_material_of_a_convex_edge() {
    let built = build();
    let top = THICKNESS / 2.0;
    for section in &built.run.sections {
        assert!((section.centre_to - section.centre_from).length() <= 1e-12);
        let centre = section.centre_from;
        assert!((centre.z - (top - RADIUS)).abs() <= 1e-12);
        let apothem = Vector3::new(section.station.x, section.station.y, 0.0).length();
        assert!(
            (Vector3::new(centre.x, centre.y, 0.0).length() - (apothem + RADIUS)).abs() <= 1e-12
        );
    }
}

#[test]
fn the_pool_always_appends() {
    let mut built = build();
    let once = built.pool.add(Vector3::new(1.0, 2.0, 3.0)).unwrap();
    let twice = built.pool.add(Vector3::new(1.0, 2.0, 3.0)).unwrap();
    assert_ne!(twice, once);
    assert_eq!(twice, once + 1);
    assert_eq!(built.pool.count(), once + 2);
    assert_eq!(built.pool.seeded_count(), built.part.vertices.len() as i32);
    for i in 0..SIDES as usize {
        let section = &built.run.sections[i];
        assert!((built.pool.position_of(section.chain_vertex) - section.station).length() <= 1e-12);
    }
}

fn the_canals_axes(built: &Built) -> Vec<(Vector3, Vector3)> {
    built
        .run
        .edges
        .iter()
        .map(|edge| {
            let (from, to) = (edge.from(), edge.to());
            let outward = Vector3::new((from.x + to.x) / 2.0, (from.y + to.y) / 2.0, 0.0);
            (
                (outward.get_normal() * (outward.length() + RADIUS))
                    + Vector3::new(0.0, 0.0, (THICKNESS / 2.0) - RADIUS),
                (to - from).get_normal(),
            )
        })
        .collect()
}

fn off_the_axis(at: Vector3, on: Vector3, way: Vector3) -> f64 {
    let apart = at - on;
    (apart - (way * apart.dot(way))).length()
}

fn the_edge_runs_straight_down(built: &Built, edge: i32, vertex: i32) -> bool {
    let graph = &built.graph;
    let other = if graph.get_vertex0(edge) == vertex {
        graph.get_vertex1(edge)
    } else {
        graph.get_vertex0(edge)
    };
    let run = Vector3::from_vector3_float(built.part.vertices[other as usize])
        - Vector3::from_vector3_float(built.part.vertices[vertex as usize]);
    run.z < 0.0 && Vector3::new(run.x, run.y, 0.0).length() < 1e-9
}

fn the_upper_mouth(features: &FeatureEdgeSet) -> i32 {
    features
        .edges()
        .iter()
        .position(|edge| on_the_mouth(edge.start) && on_the_mouth(edge.end))
        .expect("the fixture has no upper mouth") as i32
}

fn on_the_mouth(at: Vector3) -> bool {
    (at.z - (THICKNESS / 2.0)).abs() < 1e-6
        && (Vector3::new(at.x, at.y, 0.0).length() - BORE_RADIUS).abs() < 1e-4
}
