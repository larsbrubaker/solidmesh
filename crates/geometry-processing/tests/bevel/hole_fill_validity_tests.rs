//! The fill's validity check on synthetic holes - one unit-radius strip end across a convex edge
//! along x (the part is y <= 0 and z <= 0), one across a concave edge (the part is y <= 0 or z <=
//! 0), and both in one hole (a mixed corner). A fill vertex through either face the row joins - out
//! of a convex part, into a concave one - refuses; a sound one does not.

use solidmesh::bevel::level_line::corner_cap_hole_fill::bulging_fill_vertex;
use solidmesh::bevel::level_line::{CornerCapLastRow, LevelLineMesh};
use solidmesh_math::Vector3;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Interior {
    Sound,
    PokesOutOfTheConvexSide,
    SinksIntoTheConcaveSide,
}

fn v(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::new(x, y, z)
}

fn a_fill_through_a_replaced_face_refuses(
    convex_row: bool,
    concave_row: bool,
    interior: Interior,
    bulging_interior: i32,
) {
    let mut positions: Vec<Vector3> = Vec::new();
    let mut faces: Vec<(i32, i32, i32)> = Vec::new();
    let mut rows: Vec<Vec<i32>> = Vec::new();
    let mut chain_points = Vec::new();
    let mut concave = Vec::new();
    let mut fill_interior = Vec::new();

    let mut add = |p: Vector3| {
        positions.push(p);
        positions.len() as i32 - 1
    };

    if convex_row {
        // the top face z = 0 (normal +z) and the side y = 0 (normal +y), both on the part's side
        faces.push((
            add(v(0.0, 0.0, 0.0)),
            add(v(0.0, -1.0, 0.0)),
            add(v(1.0, -1.0, 0.0)),
        ));
        faces.push((
            add(v(0.0, 0.0, 0.0)),
            add(v(1.0, 0.0, -1.0)),
            add(v(0.0, 0.0, -1.0)),
        ));
        rows.push(vec![
            add(v(0.5, -1.0, 0.0)),
            add(v(0.5, -0.2929, -0.2929)),
            add(v(0.5, 0.0, -1.0)),
        ]);
        chain_points.push(v(0.5, 0.0, 0.0));
        concave.push(false);
        fill_interior.push(if interior == Interior::PokesOutOfTheConvexSide {
            v(0.5, 0.5, -0.4)
        } else {
            v(0.5, -0.4, -0.4)
        });
    }

    if concave_row {
        // the floor z = 0 (normal +z) and the wall y = 0 (normal +y), both on the air's side
        faces.push((
            add(v(10.0, 0.0, 0.0)),
            add(v(11.0, 1.0, 0.0)),
            add(v(10.0, 1.0, 0.0)),
        ));
        faces.push((
            add(v(10.0, 0.0, 0.0)),
            add(v(10.0, 0.0, 1.0)),
            add(v(11.0, 0.0, 1.0)),
        ));
        rows.push(vec![
            add(v(10.5, 1.0, 0.0)),
            add(v(10.5, 0.2929, 0.2929)),
            add(v(10.5, 0.0, 1.0)),
        ]);
        chain_points.push(v(10.5, 0.0, 0.0));
        concave.push(true);
        fill_interior.push(if interior == Interior::SinksIntoTheConcaveSide {
            v(10.5, 0.4, -0.5)
        } else {
            v(10.5, 0.4, 0.4)
        });
    }

    let mut mesh = LevelLineMesh::from_triangles(&positions, &faces, None).expect("builds");
    for t in 0..mesh.triangle_count() {
        mesh.remove_triangle(t);
    }

    let last_rows: Vec<CornerCapLastRow> = rows
        .iter()
        .enumerate()
        .map(|(i, arc)| {
            CornerCapLastRow::new(
                arc.clone(),
                chain_points[i],
                &mesh,
                concave[i],
                Vector3::ZERO,
            )
        })
        .collect();
    let loop_: Vec<Vector3> = rows.iter().flatten().map(|&i| mesh.position(i)).collect();
    let fill: Vec<Vector3> = loop_.iter().copied().chain(fill_interior).collect();

    let at = bulging_fill_vertex(&mesh, &fill, loop_.len() as i32, &last_rows, &loop_);
    let expected = if bulging_interior < 0 {
        -1
    } else {
        loop_.len() as i32 + bulging_interior - 1
    };
    assert_eq!(at, expected);
}

#[test]
fn a_fill_through_a_replaced_face_refuses_convex_sound() {
    a_fill_through_a_replaced_face_refuses(true, false, Interior::Sound, -1);
}

#[test]
fn a_fill_through_a_replaced_face_refuses_convex_pokes_out() {
    a_fill_through_a_replaced_face_refuses(true, false, Interior::PokesOutOfTheConvexSide, 1);
}

#[test]
fn a_fill_through_a_replaced_face_refuses_concave_sound() {
    a_fill_through_a_replaced_face_refuses(false, true, Interior::Sound, -1);
}

#[test]
fn a_fill_through_a_replaced_face_refuses_concave_sinks_in() {
    a_fill_through_a_replaced_face_refuses(false, true, Interior::SinksIntoTheConcaveSide, 1);
}

#[test]
fn a_fill_through_a_replaced_face_refuses_mixed_sound() {
    a_fill_through_a_replaced_face_refuses(true, true, Interior::Sound, -1);
}

#[test]
fn a_fill_through_a_replaced_face_refuses_mixed_pokes_out() {
    a_fill_through_a_replaced_face_refuses(true, true, Interior::PokesOutOfTheConvexSide, 1);
}

#[test]
fn a_fill_through_a_replaced_face_refuses_mixed_sinks_in() {
    a_fill_through_a_replaced_face_refuses(true, true, Interior::SinksIntoTheConcaveSide, 2);
}
