//! The lobed prism: a box whose front wall is bitten by a disc of `SIDES` chords, extruded along
//! z. Ports `Tests/GeometryProcessingTests/Fixtures/LobedPrismMeshes.cs`; read by
//! `surface_fan_sliver_ring_tests.rs`.

#![allow(dead_code)] // the fixture's whole surface, for the classes that read it

use agg_path::vertex_storage::VertexStorage;
use polygon_mesh::processors::vertex_source_to_mesh::extrude;
use polygon_mesh::Mesh;
use vector_math::math_helper::degrees_to_radians;
use vector_math::Vector2;

pub const RADIUS: f64 = 10.0;
pub const SIDES: i32 = 40;
pub const CENTRE_INSIDE: f64 = 2.0;
pub const WIDTH: f64 = 40.0;
pub const DEPTH: f64 = 20.0;
pub const HEIGHT: f64 = 20.0;
pub const THRESHOLD_DEGREES: f64 = 10.0;
pub const SHORT_WALL_LENGTH: f64 = 8.0;

pub fn centre() -> Vector2 {
    Vector2::new(WIDTH / 2.0, CENTRE_INSIDE)
}

pub fn create() -> Mesh {
    create_to(WIDTH)
}

pub fn create_with_a_short_wall() -> Mesh {
    create_to(right_crossing().x + SHORT_WALL_LENGTH)
}

fn create_to(right_edge: f64) -> Mesh {
    let outside = outside_corners();
    let first = *outside.iter().min().expect("the disc crosses the wall");
    let last = *outside.iter().max().expect("the disc crosses the wall");

    let mut path = VertexStorage::new();
    path.move_to(0.0, 0.0);
    path.line_to_vector(crossing_at_the_side(first - 1, first));
    for corner in first..=last {
        path.line_to_vector(disc_corner(corner));
    }
    path.line_to_vector(crossing_at_the_side(last + 1, last));
    path.line_to(right_edge, 0.0);
    path.line_to(right_edge, DEPTH);
    path.line_to(0.0, DEPTH);
    path.close_polygon();

    extrude(&path, HEIGHT).expect("the lobed outline extrudes")
}

pub fn left_crossing() -> Vector2 {
    let first = *outside_corners()
        .iter()
        .min()
        .expect("the disc crosses the wall");
    crossing_at_the_side(first - 1, first)
}

pub fn right_crossing() -> Vector2 {
    let last = *outside_corners()
        .iter()
        .max()
        .expect("the disc crosses the wall");
    crossing_at_the_side(last + 1, last)
}

pub fn disc_corner(corner: i32) -> Vector2 {
    let angle = degrees_to_radians(360.0 * f64::from(corner) / f64::from(SIDES));
    centre() + (Vector2::new(angle.cos(), angle.sin()) * RADIUS)
}

fn crossing_at_the_side(inside: i32, outside: i32) -> Vector2 {
    let from = disc_corner(inside);
    let to = disc_corner(outside);
    from + ((to - from) * (0.0 - from.y) / (to.y - from.y))
}

fn outside_corners() -> Vec<i32> {
    (0..SIDES)
        .filter(|&corner| disc_corner(corner).y < 0.0)
        .collect()
}
