//! A right-angle crease read from single-precision vertices (`foreign_edge_room::is_a_knife`): an
//! exact right angle whose vertices were rounded to `f32`, as every STL and 3MF mesh's are, is not
//! a knife on any of its edges, so the side's rail along it stays one run
//! (`gap_loop_walk::loops_of_side`) and the cut face's boundary stays the end's; a crease turned
//! clearly past a right angle still is one.

use geometry_processing::bevel::level_line::gap_loop_walk::loops_of_side;
use geometry_processing::bevel::level_line::{
    foreign_edge_room, BandGroup, FieldSample, LevelLineMesh, WorkError,
};
use vector_math::Vector3;

/// Rim segments along the crease.
const SEGMENTS: i32 = 12;

/// A wall (side 1, normal +x before the turn) meeting a cut face (untagged, normal -y) at a convex
/// crease along z that is turned `past` radians beyond a right angle, at the T Handle's coordinates
/// (about 50 across and 160 up), turned obliquely so no plane is axis-aligned, every vertex
/// rounded to single precision. Per z station: the wall's outer column A (y 2), its level-line
/// column B (y 1), the crease C (y 0) and the cut face's far column D. The wall's row between B and
/// C is removed (the band took it), so the crease's edges stand on the kept cut face alone.
fn wall_on_a_cut_face(past: f64) -> LevelLineMesh {
    let (sine, cosine) = past.sin_cos();
    let turn = |p: Vector3| {
        let (a, b) = 0.37f64.sin_cos();
        let (c, d) = 0.21f64.sin_cos();
        let about_z = Vector3::new(p.x * b - p.y * a, p.x * a + p.y * b, p.z);
        let about_x = Vector3::new(
            about_z.x,
            about_z.y * d - about_z.z * c,
            about_z.y * c + about_z.z * d,
        );
        let placed = about_x + Vector3::new(47.849, -3.0, 156.335);
        Vector3::new(
            placed.x as f32 as f64,
            placed.y as f32 as f64,
            placed.z as f32 as f64,
        )
    };

    let mut positions = Vec::new();
    for k in 0..=SEGMENTS {
        // Uneven stations on a wall that bulges along z, swept straight along y, as the T Handle's
        // wall is: the crease stays an exact right angle at every station.
        let z = k as f64 * 0.83 + 0.13 * (k as f64 * 1.7).sin();
        let x = -0.02 * (z - 5.0) * (z - 5.0);
        positions.push(turn(Vector3::new(x, 2.0, z)));
        positions.push(turn(Vector3::new(x, 1.0, z)));
        positions.push(turn(Vector3::new(x, 0.0, z)));
        // The cut face leans `past` towards the wall, sharpening the crease past square.
        positions.push(turn(Vector3::new(x - 2.0 * cosine, 2.0 * sine, z)));
    }

    let at = |k: i32, column: i32| 4 * k + column;
    let mut indices = Vec::new();
    let mut tags = Vec::new();
    let mut taken = Vec::new();
    for k in 0..SEGMENTS {
        let (a, b, c, d) = (0, 1, 2, 3);
        indices.push((at(k, b), at(k, a), at(k + 1, a)));
        indices.push((at(k, b), at(k + 1, a), at(k + 1, b)));
        tags.extend([1, 1]);
        taken.extend([false, false]);
        indices.push((at(k, c), at(k, b), at(k + 1, b)));
        indices.push((at(k, c), at(k + 1, b), at(k + 1, c)));
        tags.extend([1, 1]);
        taken.extend([true, true]);
        indices.push((at(k, c), at(k + 1, d), at(k, d)));
        indices.push((at(k, c), at(k + 1, c), at(k + 1, d)));
        tags.extend([0, 0]);
        taken.extend([false, false]);
    }

    let mut mesh =
        LevelLineMesh::from_triangles(&positions, &indices, Some(&tags)).expect("the wall builds");
    mesh.input_is_float_stored = true;
    for (triangle, gone) in taken.into_iter().enumerate() {
        if gone {
            mesh.remove_triangle(triangle as i32);
        }
    }

    mesh
}

/// The crease's edge `k`, from the removed wall triangle standing on it to the kept cut face.
fn crease_is_a_knife(mesh: &LevelLineMesh, k: i32) -> bool {
    foreign_edge_room::is_a_knife(mesh, 6 * k + 3, 6 * k + 5, (4 * k + 2, 4 * (k + 1) + 2))
}

fn side_one_loops(mesh: &LevelLineMesh) -> Vec<geometry_processing::bevel::level_line::GapLoop> {
    let group = BandGroup {
        tag: 1,
        members: vec![0],
        side_of_chain: [(0, 1)].into_iter().collect(),
        ..BandGroup::default()
    };
    let sample = |p: Vector3| -> Result<FieldSample, WorkError> {
        Ok(FieldSample {
            parameter: p.z,
            chain: 0,
            ..FieldSample::default()
        })
    };
    loops_of_side(mesh, &sample, &group, None, None, None, 0.0).expect("no budget")
}

#[test]
fn a_right_angle_read_from_single_precision_vertices_is_no_knife() {
    let mesh = wall_on_a_cut_face(0.0);
    let knives: Vec<i32> = (0..SEGMENTS)
        .filter(|&k| crease_is_a_knife(&mesh, k))
        .collect();
    assert!(
        knives.is_empty(),
        "an exact right angle rounded to single precision reads as a knife at rim edges {knives:?}"
    );
}

#[test]
fn a_crease_clearly_past_a_right_angle_is_still_a_knife() {
    // A hundredth of a radian past square: far beyond what rounding the vertices can tilt.
    let mesh = wall_on_a_cut_face(0.01);
    for k in 0..SEGMENTS {
        assert!(crease_is_a_knife(&mesh, k), "rim edge {k} is a knife");
    }
}

#[test]
fn the_side_rail_along_a_single_precision_right_angle_is_one_run() {
    let mesh = wall_on_a_cut_face(0.0);
    let loops = side_one_loops(&mesh);
    assert_eq!(
        loops.len(),
        1,
        "side 1 leaves one loop: {:?}",
        loops.iter().map(|l| &l.vertices).collect::<Vec<_>>()
    );
    // None of the crease's vertices (column C) is on it: the cut face's boundary is the end's.
    assert!(
        loops[0].vertices.iter().all(|v| v % 4 != 2),
        "the crease stays off side 1's rail: {:?}",
        loops[0].vertices
    );
}

#[test]
fn a_sliver_split_off_a_float_stored_triangle_reads_that_triangles_allowance() {
    // The cut face's triangle on each crease edge is split a ten-thousandth of the way along its
    // edge from the crease's far corner, leaving a sliver on the crease. The sliver lies in its
    // input triangle's plane, so rounding the input tilts it no more than that triangle: a crease a
    // hundredth past square stays a knife, though the sliver's own inradius would allow it 0.1.
    let mut mesh = wall_on_a_cut_face(0.01);
    for k in 0..SEGMENTS {
        let (near, far) = (4 * (k + 1) + 2, 4 * (k + 1) + 3);
        let point = mesh.position(near) + (mesh.position(far) - mesh.position(near)) * 1e-4;
        mesh.split_edge(near, far, point, false)
            .expect("the edge splits");
    }
    for k in 0..SEGMENTS {
        assert!(crease_is_a_knife(&mesh, k), "rim edge {k} is a knife");
    }
}
