//! A fillet's setback is r tan(phi / 2) for its edge's turn phi, so edges of different turn on ONE
//! chain ask for different setbacks. Each edge keeps its own along its whole length - the round is
//! r everywhere away from the vertex, and the step between two setbacks is that vertex's hole. A
//! rim with two turns adding nothing is open work (the step's hole at v0 is closed 3.6e-3 proud of
//! the leaning front face, a filler rung) and is not tested here yet. Parameter rows are loops over
//! the same-named test.

use std::rc::Rc;

use solidmesh::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use solidmesh::bevel::level_line::{
    gap_fill, BandGapResult, BandProfile, GapFillRefusal, WorkBudget,
};
use solidmesh_math::Vector3;

use crate::level_line_feature_edges::feature_edges_over;
use crate::sweep_mesh_measures;

const SIDE: f64 = 4.0;
const HEIGHT: f64 = 2.0;
const RADIUS: f64 = 0.25;
const SEGMENTS: i32 = 8;

/// A block whose front wall leans out by 45 degrees: the top face's rim is one closed chain with
/// three edges turning 90 degrees and the front edge turning 45. Every strip vertex off both faces
/// of an edge, and more than two sizes from that edge's ends, lies on that edge's own round of
/// radius r - read at every strip triangle's corners, edge midpoints and centroid. With the mean
/// setback the front edge's rows stood up to 0.085 off its r-round at r 0.25; split at the steps,
/// 1.5e-13 at the vertices. A strip built at 0.85 r and read against r reads 0.0155 and fails.
#[test]
fn every_edge_of_a_rim_with_two_turns_is_rounded_at_the_asked_radius() {
    let (positions, indices) = leaning_front_block();
    let gap = build_the_rim(&positions, &indices, RADIUS);
    assert!(
        gap.built(),
        "band {:?}, room {}, set by {:?}",
        gap.refusal,
        gap.room,
        gap.room_set_by
    );

    let filled = gap_fill::fill(&gap, BandProfile::Fillet, SEGMENTS).expect("the fill runs");
    assert_eq!(filled.refusal, GapFillRefusal::None);

    let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
    let kept = sweep_mesh_measures::kept(&mesh);
    assert!(sweep_mesh_measures::is_closed_and_consistent(
        mesh.vertices(),
        &kept
    ));
    assert_eq!(
        sweep_mesh_measures::shell_count(mesh.vertex_count(), &kept),
        1
    );

    let planes = face_planes(&positions, &indices);
    let rim = [0usize, 1, 2, 3];

    // Each rim edge's round: its axis lies r inside both the top face and that edge's wall.
    let top = Vector3::new(0.0, 0.0, 1.0);
    let mut worst = 0.0;
    // Sampled on the SURFACE, not at the vertices: each kept triangle's corners, edge midpoints
    // and centroid. Between two square rows on a straight leg the strip is quads on the round with
    // no vertex in between; a midpoint or centroid of a chord of the arc stands inside it by the
    // chord's sag (0.0012 at r 0.25 over 8 segments), within the bar.
    let on_the_round: Vec<Vector3> = kept
        .iter()
        .flat_map(|&(a, b, c)| {
            let (p, q, r) = (mesh.position(a), mesh.position(b), mesh.position(c));
            [
                p,
                q,
                r,
                (p + q) / 2.0,
                (q + r) / 2.0,
                (r + p) / 2.0,
                (p + q + r) / 3.0,
            ]
        })
        .collect();
    for e in 0..rim.len() {
        let a = positions[rim[e]];
        let b = positions[rim[(e + 1) % rim.len()]];
        let wall = wall_normal(&planes, a, b, top);
        let along = (b - a).get_normal();
        let length = (b - a).length();
        let half_turn = top.dot(wall).clamp(-1.0, 1.0).acos() / 2.0;
        let axis_point = a - ((top + wall).get_normal() * (RADIUS / half_turn.cos()));
        let mut used = 0;
        for &p in &on_the_round {
            let s = (p - a).dot(along);
            let off_top = -(p - a).dot(top);
            let off_wall = -(p - a).dot(wall);
            if s < 2.0 * RADIUS
                || s > length - (2.0 * RADIUS)
                || off_top < 1e-7
                || off_wall < 1e-7
                || off_top > 2.0 * RADIUS
                || off_wall > 2.0 * RADIUS
            {
                continue;
            }

            let from_axis = (p - axis_point) - (along * (p - axis_point).dot(along));
            let error = (from_axis.length() - RADIUS).abs();
            used += 1;
            if error > worst {
                worst = error;
            }
        }

        assert!(used > 0, "edge {e}: no round vertices");
    }

    assert!(
        worst <= RADIUS / 50.0,
        "worst radius error {worst:e} against {:e}",
        RADIUS / 50.0
    );
}

/// The pieces the rim is split into at its two setback steps meet in the steps' corner holes,
/// never across a face there: the rim takes every size its faces fit - left to right across the
/// 4 mm top is 2r, so 1.5 fits everywhere.
#[test]
fn the_rim_takes_a_size_its_faces_fit() {
    for size in [1.0, 1.5] {
        let (positions, indices) = leaning_front_block();
        let gap = build_the_rim(&positions, &indices, size);
        assert!(
            gap.built(),
            "{size}: band {:?}, room {}, set by {:?}",
            gap.refusal,
            gap.room,
            gap.room_set_by
        );
    }
}

fn build_the_rim(positions: &[Vector3], indices: &[(i32, i32, i32)], size: f64) -> BandGapResult {
    band_gap_entry_points::build_pick(
        positions,
        indices,
        &[vec![0, 1, 2, 3]],
        &[true],
        Some(feature_edges_over(positions, indices, 30.0)),
        size,
        BandProfile::Fillet,
        &BandGapBuildOptions {
            chord_tolerance: 0.0,
            snap: 1e-9,
            known_room: None,
            budget: Some(Rc::new(
                WorkBudget::new(WorkBudget::default_limit(indices.len() as i32)).expect("a limit"),
            )),
            seam_epsilon: 0.0,
            sharp_turn_degrees: 30.0,
            fillet_segments: SEGMENTS,
        },
    )
    .expect("a valid pick")
}

/// A Side x Side block, Height tall, whose front wall (y = 0 at the top) leans out to y = -Height
/// at the bottom: 12 triangles, top rim 0-1-2-3 counter-clockwise from above.
fn leaning_front_block() -> (Vec<Vector3>, Vec<(i32, i32, i32)>) {
    let positions = vec![
        Vector3::new(0.0, 0.0, HEIGHT),
        Vector3::new(SIDE, 0.0, HEIGHT),
        Vector3::new(SIDE, SIDE, HEIGHT),
        Vector3::new(0.0, SIDE, HEIGHT),
        Vector3::new(0.0, -HEIGHT, 0.0),
        Vector3::new(SIDE, -HEIGHT, 0.0),
        Vector3::new(SIDE, SIDE, 0.0),
        Vector3::new(0.0, SIDE, 0.0),
    ];
    let quads = [
        (0, 1, 2, 3), // top
        (7, 6, 5, 4), // bottom
        (0, 4, 5, 1), // front, leaning out
        (1, 5, 6, 2), // right
        (2, 6, 7, 3), // back
        (3, 7, 4, 0), // left
    ];
    let mut indices = Vec::new();
    for (a, b, c, d) in quads {
        indices.push((a, b, c));
        indices.push((a, c, d));
    }

    (positions, indices)
}

fn face_planes(positions: &[Vector3], indices: &[(i32, i32, i32)]) -> Vec<(Vector3, Vector3)> {
    let at = |v: i32| positions[v as usize];
    indices
        .iter()
        .map(|&(a, b, c)| (at(a), (at(b) - at(a)).cross(at(c) - at(a)).get_normal()))
        .collect()
}

/// The outward normal of the face, other than the top, that holds the rim edge a-b.
fn wall_normal(planes: &[(Vector3, Vector3)], a: Vector3, b: Vector3, top: Vector3) -> Vector3 {
    planes
        .iter()
        .find(|(point, normal)| {
            (a - *point).dot(*normal).abs() < 1e-9
                && (b - *point).dot(*normal).abs() < 1e-9
                && (*normal - top).length() > 1e-6
        })
        .expect("the rim edge's wall")
        .1
}
