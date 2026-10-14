//! A station whose reach runs out exactly on a corner of
//! the mesh names that corner - walked, or placed along the split blend's join.

use std::rc::Rc;

use geometry_processing::bevel::sections::bevel_section::BevelSection;
use geometry_processing::bevel::sections::section_builder_run::try_build_run;
use geometry_processing::bevel::sections::{SectionPool, SectionRun};
use geometry_processing::topology::{ChainEdge, FeatureEdgeSet};
use polygon_mesh::Mesh;
use vector_math::Vector3;

const RADIUS: f64 = 2.0;
const SEGMENTS: i32 = 4;

fn the_station_at(run: &SectionRun, at: Vector3) -> Rc<BevelSection> {
    run.sections
        .iter()
        .find(|section| (section.station - at).length() < 1e-9)
        .cloned()
        .unwrap_or_else(|| panic!("the run has no station at {at:?}"))
}

fn a_station_names_the_corner(blend_split_vertices: bool) {
    let part = the_ledge();
    let graph = part.get_mesh_edge_graph();
    let features = FeatureEdgeSet::create_with_graph(&part, &graph, 10.0);
    let mut pool = SectionPool::new(&part.vertices);
    let landing = Vector3::new(0.0, RADIUS, 0.0);
    let corner = the_vertex_at(&part, landing);
    let edges = the_chain(&features);

    let run = match try_build_run(
        &edges,
        false,
        &part,
        &graph,
        &mut pool,
        RADIUS,
        SEGMENTS,
        blend_split_vertices,
        Some(&features),
    ) {
        Ok(run) => run,
        Err(failure) => panic!("the ledge's two-edge chain builds: {failure:?}"),
    };

    let section = the_station_at(&run, Vector3::ZERO);
    let on_the_left = (pool.position_of(section.left_boundary) - landing).length() < 1e-9;
    let on_the_right = (pool.position_of(section.right_boundary) - landing).length() < 1e-9;
    assert!(
        on_the_left || on_the_right,
        "station at {:?}: left {:?} vertex {}; right {:?} vertex {}; the corner is {corner}",
        section.station,
        pool.position_of(section.left_boundary),
        section.left_walk_vertex,
        pool.position_of(section.right_boundary),
        section.right_walk_vertex
    );

    if blend_split_vertices {
        assert!(
            if on_the_left {
                section.left_boundary_is_join_placed
            } else {
                section.right_boundary_is_join_placed
            },
            "with the blend on, that side is placed along the join and not walked"
        );
    }

    let vertex = if on_the_left {
        section.left_walk_vertex
    } else {
        section.right_walk_vertex
    };
    let edge = if on_the_left {
        section.left_walk_edge
    } else {
        section.right_walk_edge
    };
    assert_eq!(vertex, corner);
    assert!(graph.get_vertex0(edge) == corner || graph.get_vertex1(edge) == corner);
}

#[test]
fn a_station_whose_reach_runs_out_on_a_corner_names_it() {
    a_station_names_the_corner(false);
}

#[test]
fn a_join_placed_end_standing_on_a_corner_names_it() {
    a_station_names_the_corner(true);
}

fn the_ledge() -> Mesh {
    let v3 = Vector3::new;
    let a = v3(-10.0, 0.0, 0.0);
    let b1 = v3(-5.0, 0.0, 0.0);
    let b = v3(0.0, 0.0, 0.0);
    let b2 = v3(5.0, 0.0, 0.0);
    let c = v3(10.0, 0.0, 0.0);
    let v = v3(0.0, RADIUS, 0.0);
    let p = v3(-10.0, 8.0, 0.0);
    let q = v3(0.0, 8.0, 0.0);
    let r = v3(10.0, 8.0, 0.0);
    let down = v3(0.0, 0.0, -8.0);
    let up = v3(0.0, 0.0, 1.0);
    let minus_y = v3(0.0, -1.0, 0.0);
    let plus_y = v3(0.0, 1.0, 0.0);
    let minus_x = v3(-1.0, 0.0, 0.0);
    let plus_x = v3(1.0, 0.0, 0.0);

    let mut mesh = Mesh::new();
    for (x, y, z, outward) in [
        (a, b1, v, up),
        (b1, b, v, up),
        (b, b2, v, up),
        (b2, c, v, up),
        (a, v, q, up),
        (a, q, p, up),
        (v, c, r, up),
        (v, r, q, up),
        (a, a + down, b1 + down, minus_y),
        (a, b1 + down, b1, minus_y),
        (b1, b1 + down, b + down, minus_y),
        (b1, b + down, b, minus_y),
        (b, b + down, b2 + down, minus_y),
        (b, b2 + down, b2, minus_y),
        (b2, b2 + down, c + down, minus_y),
        (b2, c + down, c, minus_y),
        (q, r, r + down, plus_y),
        (q, r + down, p + down, plus_y),
        (q, p + down, p, plus_y),
        (p + down, a + down, b1 + down, -up),
        (p + down, b1 + down, b + down, -up),
        (p + down, b + down, b2 + down, -up),
        (p + down, b2 + down, c + down, -up),
        (p + down, c + down, r + down, -up),
        (a, p, p + down, minus_x),
        (a, p + down, a + down, minus_x),
        (c, c + down, r + down, plus_x),
        (c, r + down, r, plus_x),
    ] {
        if (y - x).cross(z - x).dot(outward) < 0.0 {
            mesh.create_face(&[x, z, y]);
        } else {
            mesh.create_face(&[x, y, z]);
        }
    }

    mesh.clean_and_merge();
    mesh
}

fn the_vertex_at(mesh: &Mesh, at: Vector3) -> i32 {
    mesh.vertices
        .iter()
        .position(|&p| (Vector3::from_vector3_float(p) - at).length() < 1e-9)
        .map_or(-1, |i| i as i32)
}

fn the_chain(features: &FeatureEdgeSet) -> Vec<ChainEdge> {
    let mut chain: Vec<ChainEdge> = features
        .edges()
        .iter()
        .enumerate()
        .filter(|(_, edge)| {
            edge.start.y.abs() <= 1e-9
                && edge.end.y.abs() <= 1e-9
                && edge.start.z.abs() <= 1e-9
                && edge.end.z.abs() <= 1e-9
        })
        .map(|(i, edge)| ChainEdge::new(i as i32, *edge, edge.start.x > edge.end.x))
        .collect();
    // The keys are distinct, so the order is the keys' own whatever the sort.
    chain.sort_by(|left, right| left.from().x.total_cmp(&right.from().x));
    chain
}

/// With the split blend off, no station is split, so none is placed along a join: each is laid
/// from its own station. A placement left unset must read as unsplit; read as split, every station
/// would be laid as a join between station 0 and itself.
#[test]
fn rust_only_with_the_blend_off_no_station_is_join_placed() {
    let part = the_ledge();
    let graph = part.get_mesh_edge_graph();
    let features = FeatureEdgeSet::create_with_graph(&part, &graph, 10.0);
    let mut pool = SectionPool::new(&part.vertices);
    let edges = the_chain(&features);

    let run = match try_build_run(
        &edges,
        false,
        &part,
        &graph,
        &mut pool,
        RADIUS,
        SEGMENTS,
        false,
        Some(&features),
    ) {
        Ok(run) => run,
        Err(failure) => panic!("the ledge's two-edge chain builds: {failure:?}"),
    };

    for section in &run.sections {
        assert!(
            !section.left_boundary_is_join_placed && !section.right_boundary_is_join_placed,
            "station {:?} was placed along a join",
            section.station
        );
    }
}
