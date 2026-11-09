//! Parts built through the real boolean and extrude operations, typical of what a CAD user
//! makes: an L bracket with drilled holes and a plate with a grid of holes. Both carry the
//! long, thin boolean-output triangles and many short crease edges that make surface remeshing
//! hard, which is why they are the parts these tests measure.

use std::f64::consts::{FRAC_PI_2, TAU};

use solidmesh_math::{Matrix4X4, Vector2};
use solidmesh_mesh::csg::boolean_processing::do_array;
use solidmesh_mesh::csg::csg_operations::{subtract, union};
use solidmesh_mesh::csg::{BooleanOptions, CsgModes};
use solidmesh_mesh::processors::outline_to_mesh::extrude_contours;
use solidmesh_mesh::solids::platonic_solids::create_cube;
use solidmesh_mesh::{CancelToken, Mesh};

/// Circle facets for a drilled hole, the count a modest CAD tessellation produces.
const CIRCLE_SIDES: usize = 48;

fn circle(radius: f64) -> Vec<Vector2> {
    (0..CIRCLE_SIDES)
        .map(|i| {
            let angle = TAU * i as f64 / CIRCLE_SIDES as f64;
            Vector2::new(radius * angle.cos(), radius * angle.sin())
        })
        .collect()
}

fn cylinder(radius: f64, height: f64) -> Mesh {
    extrude_contours(&[circle(radius)], height).expect("extruded a circle")
}

fn translated(mut mesh: Mesh, x: f64, y: f64, z: f64) -> Mesh {
    mesh.transform(Matrix4X4::create_translation(x, y, z));
    mesh
}

/// An L bracket with four holes in its base and three in its upright: 5 mm walls.
pub fn bracket_holes() -> Mesh {
    let cancel = CancelToken::new();
    let base = translated(create_cube(80.0, 40.0, 5.0), 0.0, 0.0, 2.5);
    let upright = translated(create_cube(5.0, 40.0, 50.0), -37.5, 0.0, 30.0);
    let mut part = union(&base, &upright, &cancel).expect("union of base and upright");
    for (x, y) in [(10.0, -12.0), (10.0, 12.0), (30.0, -12.0), (30.0, 12.0)] {
        let hole = translated(cylinder(3.0, 30.0), x, y, -10.0);
        part = subtract(&part, &hole, &cancel).expect("drilled the base");
    }
    for (y, z) in [(-12.0, 25.0), (12.0, 25.0), (0.0, 42.0)] {
        let mut hole = cylinder(3.0, 30.0);
        hole.transform(Matrix4X4::create_rotation_y(FRAC_PI_2));
        let hole = translated(hole, -55.0, y, z);
        part = subtract(&part, &hole, &cancel).expect("drilled the upright");
    }
    part
}

/// A 120 mm plate, 4 mm thick, with a 10 by 10 grid of 4 mm holes.
pub fn plate_hole_grid() -> Mesh {
    let plate = translated(create_cube(120.0, 120.0, 4.0), 0.0, 0.0, 2.0);
    let drill = translated(cylinder(2.0, 20.0), 0.0, 0.0, -8.0);
    let mut offsets = Vec::new();
    for i in 0..10 {
        for j in 0..10 {
            offsets.push((-54.0 + 12.0 * i as f64, -54.0 + 12.0 * j as f64));
        }
    }
    let matrices: Vec<Matrix4X4> = offsets
        .iter()
        .map(|&(x, y)| Matrix4X4::create_translation(x, y, 0.0))
        .collect();
    let mut items: Vec<(&Mesh, Matrix4X4)> = vec![(&plate, Matrix4X4::IDENTITY)];
    items.extend(matrices.iter().map(|&m| (&drill, m)));
    do_array(
        &items,
        CsgModes::Subtract,
        &CancelToken::new(),
        &BooleanOptions::default(),
    )
    .expect("drilled the plate")
}
