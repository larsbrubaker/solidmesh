//! A face too thin for its corners' placement to give it a normal (`FeatureEdge::try_classify`):
//! a triangle whose third corner sits on the line through the other two, as far as single
//! precision can say, carries no crease of its own. Its edges read the turn between the real
//! faces on either side of it, so a flat face with such a sliver in it has no feature edge
//! inside it, and a sliver lying along a real crease leaves that crease where it is.

use solidmesh::topology::{EdgeConvexity, FeatureEdge, FeatureEdgeSet};
use solidmesh_mesh::Mesh;

/// The corner the sliver's third vertex is nudged off the top face's diagonal by: one unit in the
/// last place of 200.5 in single precision, so the sliver is about 1.1e-5 tall while its corners
/// are placed only to about 1.3e-5 - a height the input cannot tell from none.
const NUDGED_X: f64 = 200.500_015_258_789_06;

/// A 1 x 1 x 10 box at x = 200 whose top face (z = 10) is cut along its diagonal from (200, 0)
/// to (201, 1), with a vertex `c` on that diagonal and the zero-height triangle (P0, P2, c) along
/// it. The nudge puts `c` on the side that winds the sliver upside down, so its normal, read off
/// its corners, points down - a fold of 180 degrees against the top face it lies in.
fn box_with_a_sliver_in_its_top(nudged_x: f64) -> Mesh {
    let vertices = [
        200.0, 0.0, 10.0, // P0
        201.0, 0.0, 10.0, // P1
        201.0, 1.0, 10.0, // P2
        200.0, 1.0, 10.0, // P3
        200.0, 0.0, 0.0, // B0
        201.0, 0.0, 0.0, // B1
        201.0, 1.0, 0.0, // B2
        200.0, 1.0, 0.0, // B3
        nudged_x, 0.5, 10.0, // c
    ];
    let faces = [
        0, 1, 2, // top, beside the diagonal
        0, 2, 8, // the sliver along the diagonal
        0, 8, 3, // top
        8, 2, 3, // top
        4, 6, 5, // bottom
        4, 7, 6, // bottom
        4, 5, 1, // y = 0
        4, 1, 0, // y = 0
        5, 6, 2, // x = 201
        5, 2, 1, // x = 201
        6, 7, 3, // y = 1
        6, 3, 2, // y = 1
        7, 4, 0, // x = 200
        7, 0, 3, // x = 200
    ];
    Mesh::from_arrays(&vertices, &faces)
}

/// Three units in the last place off the diagonal: the sliver is 3.2e-5 tall, about two and a half
/// times what a corner's rounding can move it by, yet a corner moved that far could still tilt
/// its normal by most of a radian - far past the quarter degree a flat face's normal is trusted to.
const NUDGED_X_THREE_UNITS: f64 = 200.5 + 3.0 / 65_536.0;

#[test]
fn a_sliver_too_thin_for_a_normal_puts_no_crease_in_a_flat_face() {
    only_the_boxs_own_edges_are_creases(NUDGED_X);
}

#[test]
#[ignore = "open decision: trusting a face's normal only while its corners' placement can tilt it by less than half the finest flat tolerance (0.256 degrees) catches this sliver, but moves three recorded parts (Center Right and Left Side, Inner Right Sides and Center, Unexpected Refusals), the Many Holes section readings, and stops fade_end_row_fold_tests' many-holes-c563-r025 building clean"]
fn a_sliver_two_and_a_half_times_its_placement_still_puts_no_crease_in_a_flat_face() {
    only_the_boxs_own_edges_are_creases(NUDGED_X_THREE_UNITS);
}

fn only_the_boxs_own_edges_are_creases(nudged_x: f64) {
    let mesh = box_with_a_sliver_in_its_top(nudged_x);
    assert!(mesh.is_manifold());

    let features = FeatureEdgeSet::create_with_angle(&mesh, 13.0);
    let inside_the_top: Vec<(i32, i32, f64)> = features
        .edges()
        .iter()
        .filter(|edge| {
            [edge.start_vertex, edge.end_vertex].contains(&8)
                || [(0, 2), (2, 0)].contains(&(edge.start_vertex, edge.end_vertex))
        })
        .map(|edge| (edge.start_vertex, edge.end_vertex, edge.turn_angle_degrees))
        .collect();
    assert!(
        inside_the_top.is_empty(),
        "edges inside the flat top read as creases: {inside_the_top:?}"
    );

    // The box's own twelve edges, every one a convex right angle.
    assert_eq!(features.count(), 12);
    for edge in features.edges() {
        assert_eq!(edge.convexity, EdgeConvexity::Convex);
        assert!((edge.turn_angle_degrees - 90.0).abs() < 1e-6, "{edge:?}");
    }
}

/// One unit in the last place of 200.5 in single precision.
const ULP_AT_200_5: f64 = 1.0 / 65_536.0;

/// A 1 x 1 x 10 box at x = 200 whose top is cut along its diagonal from P0 (200, 0) to P2
/// (201, 1) into a strip of two slivers sharing that diagonal - (P0, B, P2) and (P0, P2, D) - the
/// thin quad a boolean leaves split on its diagonal. `b_x` and `d_x` put B and D at y = 0.5 a unit
/// in the last place either side of the diagonal's midpoint.
fn box_with_a_two_sliver_strip_in_its_top(b_x: f64, d_x: f64) -> Mesh {
    let vertices = [
        200.0, 0.0, 10.0, // 0 P0
        201.0, 0.0, 10.0, // 1 P1
        201.0, 1.0, 10.0, // 2 P2
        200.0, 1.0, 10.0, // 3 P3
        200.0, 0.0, 0.0, // 4 B0
        201.0, 0.0, 0.0, // 5 B1
        201.0, 1.0, 0.0, // 6 B2
        200.0, 1.0, 0.0, // 7 B3
        b_x, 0.5, 10.0, // 8 B
        d_x, 0.5, 10.0, // 9 D
    ];
    let faces = [
        0, 1, 8, 8, 1, 2, 0, 8, 2, // P1's side of the strip, and the first sliver
        0, 2, 9, 0, 9, 3, 9, 2, 3, // the second sliver, and P3's side
        4, 6, 5, 4, 7, 6, 4, 5, 1, 4, 1, 0, 5, 6, 2, 5, 2, 1, 6, 7, 3, 6, 3, 2, 7, 4, 0, 7, 0, 3,
    ];
    Mesh::from_arrays(&vertices, &faces)
}

/// A 1 x 1 x 10 box at x = 200 whose top front edge, P0 (200, 0, 10) to P1 (201, 0, 10), carries a
/// strip of two slivers sharing that edge: (P0, P1, B) with B a hair off the edge in the top's
/// plane, and (P1, P0, D) with D a hair off it in the y = 0 side's plane.
fn box_with_a_two_sliver_strip_on_its_top_front_edge(b_y: f64, d_z: f64) -> Mesh {
    let vertices = [
        200.0, 0.0, 10.0, // 0 P0
        201.0, 0.0, 10.0, // 1 P1
        201.0, 1.0, 10.0, // 2 P2
        200.0, 1.0, 10.0, // 3 P3
        200.0, 0.0, 0.0, // 4 B0
        201.0, 0.0, 0.0, // 5 B1
        201.0, 1.0, 0.0, // 6 B2
        200.0, 1.0, 0.0, // 7 B3
        200.5, b_y, 10.0, // 8 B
        200.5, 0.0, d_z, // 9 D
    ];
    let faces = [
        0, 1, 8, 1, 0, 9, // the strip
        8, 1, 2, 8, 2, 3, 8, 3, 0, // top
        4, 6, 5, 4, 7, 6, // bottom
        4, 5, 9, 5, 1, 9, 4, 9, 0, // y = 0, fanned to D
        5, 6, 2, 5, 2, 1, 6, 7, 3, 6, 3, 2, 7, 4, 0, 7, 0, 3,
    ];
    Mesh::from_arrays(&vertices, &faces)
}

/// Every manifold edge's classification, failing where an edge reads none.
fn every_edge_classified(mesh: &Mesh) -> Vec<FeatureEdge> {
    let graph = mesh.get_mesh_edge_graph();
    (0..graph.edge_count())
        .map(|e| {
            FeatureEdge::try_classify(mesh, &graph, e, 13.0).unwrap_or_else(|| {
                panic!(
                    "edge {}-{} reads no turn, so a face group takes it for an open rim",
                    graph.get_vertex0(e),
                    graph.get_vertex1(e)
                )
            })
        })
        .collect()
}

fn a_flat_two_sliver_strip_is_flat_throughout(b_x: f64, d_x: f64) {
    let mesh = box_with_a_two_sliver_strip_in_its_top(b_x, d_x);
    assert!(mesh.is_manifold());
    for edge in every_edge_classified(&mesh) {
        let on_the_box = [edge.start, edge.end]
            .iter()
            .all(|p| (p.x == 200.0 || p.x == 201.0) && (p.y == 0.0 || p.y == 1.0));
        let d = edge.end - edge.start;
        let along_one_axis = [d.x, d.y, d.z].iter().filter(|&&c| c != 0.0).count() == 1;
        if on_the_box && along_one_axis {
            assert_eq!(edge.convexity, EdgeConvexity::Convex, "{edge:?}");
        } else {
            assert_eq!(edge.convexity, EdgeConvexity::Flat, "{edge:?}");
        }
    }
}

#[test]
fn a_two_sliver_strip_upright_in_a_flat_face_reads_flat() {
    a_flat_two_sliver_strip_is_flat_throughout(200.5 + ULP_AT_200_5, 200.5 - ULP_AT_200_5);
}

#[test]
fn a_two_sliver_strip_flipped_in_a_flat_face_reads_flat() {
    a_flat_two_sliver_strip_is_flat_throughout(200.5 - ULP_AT_200_5, 200.5 + ULP_AT_200_5);
}

fn a_two_sliver_strip_on_a_crease_keeps_the_crease(b_y: f64, d_z: f64) {
    let mesh = box_with_a_two_sliver_strip_on_its_top_front_edge(b_y, d_z);
    assert!(mesh.is_manifold());
    let edges = every_edge_classified(&mesh);
    // The crease stays on the strip's shared edge, a right angle; each half reads the face it lies
    // on, so its edges to that face are flat.
    for edge in edges.iter().filter(|edge| {
        [edge.start_vertex, edge.end_vertex]
            .iter()
            .all(|v| [0, 1, 8, 9].contains(v))
    }) {
        if [edge.start_vertex, edge.end_vertex].contains(&8)
            || [edge.start_vertex, edge.end_vertex].contains(&9)
        {
            assert_eq!(edge.convexity, EdgeConvexity::Flat, "{edge:?}");
        } else {
            assert_eq!(edge.convexity, EdgeConvexity::Convex, "{edge:?}");
            assert!((edge.turn_angle_degrees - 90.0).abs() < 1e-6, "{edge:?}");
        }
    }
}

/// 2^-17: under what rounding to single precision can move a corner here (about 1.3e-5).
const A_HAIR: f64 = 1.0 / 131_072.0;

#[test]
fn a_two_sliver_strip_upright_on_a_crease_keeps_the_crease() {
    a_two_sliver_strip_on_a_crease_keeps_the_crease(A_HAIR, 10.0 - A_HAIR);
}

#[test]
fn a_two_sliver_strip_flipped_on_a_crease_keeps_the_crease() {
    a_two_sliver_strip_on_a_crease_keeps_the_crease(-A_HAIR, 10.0 + A_HAIR);
}
