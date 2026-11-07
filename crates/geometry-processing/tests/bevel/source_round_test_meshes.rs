//! The reduced "Unexpected Gap In Fillet" part: a tessellated post standing in a flat wall, whose
//! crease runs PARALLEL to the post's own axis. Read by `bevel_source_round_tests.rs`.

use std::f64::consts::TAU;

use crate::path_solids::extrude;
use crate::path_solids::Outline;
use polygon_mesh::csg::boolean_processing::do_array;
use polygon_mesh::csg::{BooleanOptions, CsgModes};
use polygon_mesh::solids::platonic_solids;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::{Matrix4X4, Vector2, Vector3};

/// A regular prism drawn with `sides` chords, its vertices exactly on the circle so a fit of them
/// has an exact answer to find, extruded along z about the origin.
pub fn create_polygon_prism(sides: i32, radius: f64, height: f64) -> Mesh {
    let mut path = Outline::new();

    for i in 0..sides {
        let angle = TAU * i as f64 / sides as f64;
        let x = radius * angle.cos();
        let y = radius * angle.sin();

        if i == 0 {
            path.move_to(x, y);
        } else {
            path.line_to(x, y);
        }
    }

    path.close_polygon();

    let mut mesh = extrude(&path, height).expect("a regular polygon extrudes");

    mesh.translate(Vector3::new(0.0, 0.0, -height / 2.0));

    mesh
}

/// That prism standing in a slab whose top face is the plane y = `wall_y`, unioned so the two
/// vertical creases where the post meets the wall are real mesh creases with the boolean's own
/// slivers on them - which is what the fixture is about. `axis` is where the post's axis runs, in
/// x and y.
///
/// The union runs on the manifold kernel, which needs no processing mode or resolution.
pub fn create_post_in_a_wall(sides: i32, radius: f64, axis: Vector2, wall_y: f64) -> Mesh {
    let mut post = create_polygon_prism(sides, radius, 40.0);

    post.translate(Vector3::new(axis.x, axis.y, 0.0));

    // Wide enough that the post's two creases are well inside it, and deep enough that its own
    // far side is nowhere near them.
    let mut wall = platonic_solids::create_cube(120.0, 60.0, 40.0);

    wall.translate(Vector3::new(0.0, wall_y - 30.0, 0.0));

    do_array(
        &[(&post, Matrix4X4::IDENTITY), (&wall, Matrix4X4::IDENTITY)],
        CsgModes::Union,
        &CancelToken::new(),
        &BooleanOptions::default(),
    )
    .expect("the union of a post and a wall succeeds")
}
