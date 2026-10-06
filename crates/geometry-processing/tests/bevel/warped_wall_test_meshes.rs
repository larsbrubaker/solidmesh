//! Port of the test fixture `Fixtures/WarpedWallTestMeshes.cs`: a block whose front wall is a
//! gently twisted (hyperbolic) sheet of quads, and an L with such a wall in its inside corner.

use polygon_mesh::Mesh;
use vector_math::Vector3;

const COLUMNS: i32 = 10;
const ROWS: i32 = 5;
const WIDTH: f64 = 40.0;
const HEIGHT: f64 = 20.0;
const BACK_Y: f64 = -10.0;
const TWIST: f64 = 0.02;

fn x(i: i32) -> f64 {
    WIDTH * f64::from(i) / f64::from(COLUMNS)
}

fn z(j: i32) -> f64 {
    HEIGHT * f64::from(j) / f64::from(ROWS)
}

fn y(i: i32, j: i32) -> f64 {
    10.0 + (TWIST * (x(i) - (WIDTH / 2.0)) * (z(j) - (HEIGHT / 2.0)))
}

fn warped(i: i32, j: i32) -> Vector3 {
    Vector3::new(x(i), y(i, j), z(j))
}

fn back(i: i32, j: i32) -> Vector3 {
    Vector3::new(x(i), BACK_Y, z(j))
}

/// C# `CreateWarpedWallBlock`.
pub fn create_warped_wall_block() -> Mesh {
    let mut mesh = Mesh::new();
    emit_warped_block(&mut mesh, COLUMNS, true);
    mesh.clean_and_merge();
    mesh
}

/// C# `CreateWarpedWallL`.
pub fn create_warped_wall_l() -> Mesh {
    const EXTENDED: i32 = 15;
    const FRONT_Y: f64 = 30.0;
    let front = |i: i32, j: i32| Vector3::new(x(i), FRONT_Y, z(j));

    let mut mesh = Mesh::new();
    emit_warped_block(&mut mesh, EXTENDED, false);
    for j in 0..ROWS {
        mesh.create_face(&[
            warped(COLUMNS, j),
            warped(COLUMNS, j + 1),
            front(COLUMNS, j + 1),
            front(COLUMNS, j),
        ]);
        mesh.create_face(&[
            back(EXTENDED, j),
            front(EXTENDED, j),
            front(EXTENDED, j + 1),
            back(EXTENDED, j + 1),
        ]);
    }

    for i in COLUMNS..EXTENDED {
        for j in 0..ROWS {
            mesh.create_face(&[
                front(i, j),
                front(i, j + 1),
                front(i + 1, j + 1),
                front(i + 1, j),
            ]);
        }

        if i == COLUMNS {
            mesh.create_face(&[
                Vector3::new(x(i + 1), BACK_Y, HEIGHT),
                Vector3::new(x(i + 1), FRONT_Y, HEIGHT),
                Vector3::new(x(i), FRONT_Y, HEIGHT),
                warped(COLUMNS, ROWS),
                Vector3::new(x(i), BACK_Y, HEIGHT),
            ]);
            mesh.create_face(&[
                warped(COLUMNS, 0),
                Vector3::new(x(i), FRONT_Y, 0.0),
                Vector3::new(x(i + 1), FRONT_Y, 0.0),
                Vector3::new(x(i + 1), BACK_Y, 0.0),
                Vector3::new(x(i), BACK_Y, 0.0),
            ]);
        } else {
            mesh.create_face(&[
                Vector3::new(x(i), BACK_Y, HEIGHT),
                Vector3::new(x(i + 1), BACK_Y, HEIGHT),
                Vector3::new(x(i + 1), FRONT_Y, HEIGHT),
                Vector3::new(x(i), FRONT_Y, HEIGHT),
            ]);
            mesh.create_face(&[
                Vector3::new(x(i), FRONT_Y, 0.0),
                Vector3::new(x(i + 1), FRONT_Y, 0.0),
                Vector3::new(x(i + 1), BACK_Y, 0.0),
                Vector3::new(x(i), BACK_Y, 0.0),
            ]);
        }
    }

    mesh.clean_and_merge();
    mesh
}

fn emit_warped_block(mesh: &mut Mesh, back_columns: i32, close_end_face: bool) {
    for i in 0..back_columns {
        for j in 0..ROWS {
            if i < COLUMNS {
                mesh.create_face(&[
                    warped(i, j),
                    warped(i, j + 1),
                    warped(i + 1, j + 1),
                    warped(i + 1, j),
                ]);
            }
            mesh.create_face(&[
                back(i, j),
                back(i + 1, j),
                back(i + 1, j + 1),
                back(i, j + 1),
            ]);
        }
    }

    for j in 0..ROWS {
        if close_end_face {
            mesh.create_face(&[
                back(COLUMNS, j),
                warped(COLUMNS, j),
                warped(COLUMNS, j + 1),
                back(COLUMNS, j + 1),
            ]);
        }
        mesh.create_face(&[back(0, j), back(0, j + 1), warped(0, j + 1), warped(0, j)]);
    }

    for i in 0..COLUMNS {
        mesh.create_face(&[
            back(i, ROWS),
            back(i + 1, ROWS),
            warped(i + 1, ROWS),
            warped(i, ROWS),
        ]);
        mesh.create_face(&[back(i + 1, 0), back(i, 0), warped(i, 0), warped(i + 1, 0)]);
    }
}
