//! A STOPPED free end in the corner fill, on an end plane leaning a few degrees off square to the
//! chain: a 2 mm round on a straight convex edge along x (top face z = 0 over y > 0, wall y = 0
//! below it), stopped at its start on the plane through the end vertex whose normal leans 4.5
//! degrees off -x, as the mitre between two facets of a 40-sided rim does. The rail on the top face
//! meets that plane at its corner (2 tan 4.5 degrees, 2, 0), which reads a parameter short of the
//! end; the plane's trace on to the end vertex reads the end's own parameter only near the vertex.
//! The corner fill's split round the free end ends the rail at its corner in the plane, reading the
//! end, and the row between the two corners is the round's section in the plane.

use std::rc::Rc;

use solidmesh::bevel::level_line::corner_fill_free_ends::split_round_a_free_end;
use solidmesh::bevel::level_line::fillet_arc_rows::build_in_a_stopped_plane;
use solidmesh::bevel::level_line::{ChainField, CornerFillPiece, EndRayBound, LevelLineMesh};
use solidmesh_math::Vector3;

use crate::chain_field_fixtures::open_chain;

const RADIUS: f64 = 2.0;

/// How far the end plane leans off square to the chain.
fn lean() -> f64 {
    4.5_f64.to_radians()
}

/// The straight chain along x, stopped at its start on the leaning plane.
fn stopped_chain() -> ChainField {
    let mut chain = open_chain(
        vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(4.0, 0.0, 0.0)],
        vec![RADIUS, RADIUS],
    );
    let bound = EndRayBound {
        plane_normal: Vector3::new(-lean().cos(), lean().sin(), 0.0),
        stopped: true,
        ..EndRayBound::default()
    };
    chain.end_ray_bounds = (Some(bound), None);
    chain.end_clips = ChainField::find_end_clips(&chain.points, chain.length, chain.end_ray_bounds);
    chain
}

/// The point at `y` on the top face where the end plane crosses it.
fn on_the_plane_at(y: f64, z: f64) -> Vector3 {
    Vector3::new(y * lean().tan(), y, z)
}

#[test]
fn a_stopped_free_end_ends_the_rail_at_its_corner_in_the_leaning_plane() {
    let chain = Rc::new(stopped_chain());
    let mut mesh = LevelLineMesh::new();
    // The band's loop as the cut leaves it: up the top rail to its corner in the plane, along the
    // plane's trace to the end vertex and down the wall's edge, then along the wall rail.
    let loop_points = [
        (Vector3::new(3.0, RADIUS, 0.0), 3.0),
        (Vector3::new(2.0, RADIUS, 0.0), 2.0),
        (Vector3::new(1.0, RADIUS, 0.0), 1.0),
        (on_the_plane_at(RADIUS, 0.0), RADIUS * lean().tan()),
        (on_the_plane_at(1.5, 0.0), 1.5 * lean().tan()),
        // Near the end vertex the trace reads the end's own parameter.
        (on_the_plane_at(1.0, 0.0), 0.0),
        (Vector3::new(0.0, 0.0, 0.0), 0.0),
        (Vector3::new(0.0, 0.0, -1.0), 0.0),
        (Vector3::new(0.0, 0.0, -RADIUS), 0.0),
        (Vector3::new(1.0, 0.0, -RADIUS), 1.0),
        (Vector3::new(2.0, 0.0, -RADIUS), 2.0),
    ];
    let piece = CornerFillPiece {
        owner: 0,
        vertices: loop_points
            .iter()
            .map(|&(p, _)| mesh.add_vertex(p))
            .collect(),
        parameters: loop_points.iter().map(|&(_, t)| t).collect(),
    };

    let rails = split_round_a_free_end(&mesh, &[chain], &piece);

    assert_eq!(rails.len(), 2, "{rails:?}");
    assert_eq!(
        rails[0].vertices,
        piece.vertices[0..=3].to_vec(),
        "the top rail ends at its corner in the plane, not on along the plane's trace"
    );
    assert_eq!(
        rails[0].parameters[3], 0.0,
        "the corner stands for the end, so the strip runs the wall rail on to the end against it"
    );
    assert_eq!(rails[1].vertices, piece.vertices[8..=10].to_vec());
    assert_eq!(rails[1].parameters[0], 0.0);
}

#[test]
fn the_row_between_the_corners_is_the_rounds_whole_section_in_the_leaning_plane() {
    let chain = stopped_chain();
    let mut mesh = LevelLineMesh::new();
    let top = mesh.add_vertex(on_the_plane_at(RADIUS, 0.0));
    let wall = mesh.add_vertex(Vector3::new(0.0, 0.0, -RADIUS));
    let segments = 8;

    let row = build_in_a_stopped_plane(&mut mesh, &chain, top, wall, segments, &mut Vec::new())
        .expect("both corners stand in the stopped plane");

    assert_eq!(row.len(), segments as usize + 1);
    let normal = Vector3::new(-lean().cos(), lean().sin(), 0.0);
    let axis = |p: Vector3| ((p.y - RADIUS).powi(2) + (p.z + RADIUS).powi(2)).sqrt();
    for &v in &row {
        let p = mesh.position(v);
        assert!(
            p.dot(normal).abs() <= 1e-12,
            "the row lies in the plane: {p:?}"
        );
        assert!(
            (axis(p) - RADIUS).abs() <= 1e-12,
            "the row lies on the round's cylinder: {p:?}"
        );
    }

    // The flat end the row closes with the end vertex: the profile's corner area with its chords,
    // stretched by the plane's lean off square. The row's points are spaced by the arc's own
    // parameter rather than by equal angles, so its chords cut a hair more than equal ones would:
    // within 1e-3 of the equal-angle figure, and never less than the exact section.
    let end_vertex = Vector3::new(0.0, 0.0, 0.0);
    let mut twice_area = Vector3::new(0.0, 0.0, 0.0);
    for pair in row.windows(2) {
        let (a, b) = (mesh.position(pair[0]), mesh.position(pair[1]));
        twice_area += (a - end_vertex).cross(b - end_vertex);
    }
    let area = twice_area.length() / 2.0;
    let n = f64::from(segments);
    let alpha = std::f64::consts::FRAC_PI_2 / n;
    let exact = (1.0 - std::f64::consts::FRAC_PI_4) * RADIUS * RADIUS / lean().cos();
    let chorded = exact + (n * RADIUS * RADIUS / 2.0 * (alpha - alpha.sin()) / lean().cos());
    assert!(
        area >= exact && (area - chorded).abs() <= 1e-3 * chorded,
        "the flat end shows the round's whole section: {area} against {chorded}"
    );
}
