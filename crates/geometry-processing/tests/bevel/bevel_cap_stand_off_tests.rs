//! A cutter's cap clears a top face that is not flat to the
//! last bit, and a bead's cap is not pushed off the face it stops on.

use geometry_processing::bevel::bevel_chain_end_cap::ResolveOptions;
use geometry_processing::bevel::{bevel_chain_linker, BevelChainEnds};
use geometry_processing::topology::{
    bevel_edge_group_resolver, BevelSelectionKind, FaceGroup, FeatureEdgeSet,
};
use polygon_mesh::Mesh;
use vector_math::{Vector2, Vector3};

use crate::topology_test_meshes::create_rib_on_plate;

const SIZE: f64 = 20.0;
const OUT_OF_FLAT: f64 = 2e-6;
const RADIUS: f64 = 3.0;
const LARGEST_HARMLESS_STAND_OFF: f64 = 1e-4;
const ON_A_PLANE: f64 = 1e-9;

/// A cutter's cap clears a top face that is not flat to the last bit.
#[test]
fn a_cutters_cap_clears_a_top_face_that_is_not_flat_to_the_last_bit() {
    let mesh = box_with_a_tilted_top(OUT_OF_FLAT);
    let features = FeatureEdgeSet::create(&mesh);
    let edge = edge_index(
        &features,
        Vector3::new(10.0, 10.0, 0.0),
        Vector3::new(10.0, 10.0, SIZE),
    );
    let group = bevel_edge_group_resolver::create_from_edge(
        &features,
        edge as i32,
        BevelSelectionKind::Edge,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    )
    .unwrap();
    let resolution = bevel_edge_group_resolver::resolve(&group, &mesh);
    let run = bevel_chain_linker::link_resolution(&resolution).remove(0);
    let ends = BevelChainEnds::resolve(
        &run.edges,
        run.is_closed,
        RADIUS,
        Some(&features),
        Some(&resolution.edges),
        None,
        ResolveOptions::default(),
    );

    assert!(ends.end.is_capped());
    let highest = mesh
        .vertices
        .iter()
        .map(|&v| ends.end.plane_normal.dot(Vector3::from_vector3_float(v)))
        .fold(f64::NEG_INFINITY, f64::max);
    assert!(
        ends.end.plane_distance > highest,
        "the cap must clear the top"
    );
    assert!(ends.end.plane_distance - highest < LARGEST_HARMLESS_STAND_OFF);

    let furthest_below = mesh
        .vertices
        .iter()
        .map(|&v| ends.start.plane_normal.dot(Vector3::from_vector3_float(v)))
        .fold(f64::NEG_INFINITY, f64::max);
    assert!(ends.start.plane_distance > furthest_below);
}

/// A bead's cap is not pushed off the face it stops on.
#[test]
fn a_beads_cap_is_not_pushed_off_the_face_it_stops_on() {
    let part = create_rib_on_plate();
    let features = FeatureEdgeSet::create(&part);
    let foot = edge_index(
        &features,
        Vector3::new(8.0, 5.0, 5.0),
        Vector3::new(8.0, 20.0, 5.0),
    );
    let foot_edge = features.edges()[foot];
    let run = bevel_chain_linker::link(&[foot_edge]).remove(0);
    let ends = BevelChainEnds::resolve(
        &run.edges,
        run.is_closed,
        RADIUS,
        Some(&features),
        Some(&[foot_edge]),
        None,
        ResolveOptions::default(),
    );

    assert!(ends.start.is_capped() || ends.end.is_capped());
    for (name, cap) in [("start", &ends.start), ("end", &ends.end)] {
        if !cap.is_capped() {
            continue;
        }

        let through_the_vertex = part.vertices.iter().any(|&v| {
            (cap.plane_normal.dot(Vector3::from_vector3_float(v)) - cap.plane_distance).abs()
                < ON_A_PLANE
        });
        assert!(
            through_the_vertex,
            "the {name} cap stands off the face it stops on"
        );
    }
}

fn box_with_a_tilted_top(out_of_flat: f64) -> Mesh {
    let mut mesh = Mesh::new();
    let corners = [
        Vector2::new(-10.0, -10.0),
        Vector2::new(10.0, -10.0),
        Vector2::new(10.0, 10.0),
        Vector2::new(-10.0, 10.0),
    ];
    let top_of = |corner: Vector2| {
        if corner.x == 10.0 && corner.y == 10.0 {
            SIZE
        } else {
            SIZE + out_of_flat
        }
    };

    let mut bottom: Vec<Vector3> = corners
        .iter()
        .map(|c| Vector3::new(c.x, c.y, 0.0))
        .collect();
    bottom.reverse();
    mesh.create_face(&bottom);
    let top: Vec<Vector3> = corners
        .iter()
        .map(|&c| Vector3::new(c.x, c.y, top_of(c)))
        .collect();
    mesh.create_face(&top);
    for i in 0..corners.len() {
        let from = corners[i];
        let to = corners[(i + 1) % corners.len()];
        mesh.create_face(&[
            Vector3::new(from.x, from.y, 0.0),
            Vector3::new(to.x, to.y, 0.0),
            Vector3::new(to.x, to.y, top_of(to)),
            Vector3::new(from.x, from.y, top_of(from)),
        ]);
    }

    mesh.clean_and_merge();
    mesh
}

fn edge_index(features: &FeatureEdgeSet, from: Vector3, to: Vector3) -> usize {
    features
        .edges()
        .iter()
        .position(|edge| {
            ((edge.start - from).length() < 1e-5 && (edge.end - to).length() < 1e-5)
                || ((edge.start - to).length() < 1e-5 && (edge.end - from).length() < 1e-5)
        })
        .unwrap_or_else(|| panic!("the fixture has no feature edge from {from:?} to {to:?}"))
}
