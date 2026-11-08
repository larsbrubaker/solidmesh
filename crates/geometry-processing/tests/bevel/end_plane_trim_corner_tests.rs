//! `EndPlaneTrim.ToTheCorner` at an OBLIQUE end - the chain runs along +x from its end at the
//! origin, the end plane is -x + 0.8y = 0 (about 39 degrees off square), and the setback is 2. The
//! rail runs along the plane to its corner B, still within the setback of the end ray, then leaves
//! the plane beside the ray's line (C) before it clears the setback (D).

use solidmesh::bevel::level_line::end_plane_trim::to_the_corner;
use solidmesh::bevel::level_line::{GapLoop, LevelLineMesh};
use solidmesh_math::Vector3;

fn v(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::new(x, y, z)
}

fn run(parameters: Vec<f64>) -> GapLoop {
    GapLoop::new(1, false, vec![0, 1, 2, 3], parameters, 0, false, None)
}

#[test]
fn the_last_vertex_on_the_end_plane_is_kept_as_the_corner() {
    let positions = [
        v(0.4, 0.5, 0.0), // A: on the plane, inside the setback - carried along the crease
        v(0.8, 1.0, 0.0), // B: on the plane, inside the setback - the corner
        v(3.0, 1.5, 0.0), // C: off the plane, inside the setback of the ray's line
        v(5.0, 2.5, 0.0), // D: outside the setback
    ];
    let mesh =
        LevelLineMesh::from_triangles(&positions, &[(0, 1, 2), (1, 3, 2)], None).expect("builds");
    let run = run(vec![0.0, 0.1, 3.0, 5.0]);
    let normal = v(-1.0, 0.8, 0.0).get_normal();

    let (cut, corner_inside) = to_the_corner(
        &mesh,
        &run,
        false,
        Vector3::ZERO,
        v(-1.0, 0.0, 0.0),
        normal,
        2.0,
    );

    let cut = cut.expect("a cut");
    // B is the last vertex on the plane before the rail leaves it
    assert_eq!(cut.vertices[0], 1);
    assert_eq!(cut.vertices.len(), 3);
    assert!(corner_inside);
}

/// The same oblique end with the part 100 mm from the origin, where a float step is 7.6e-6: the
/// corner B read through float storage stands 3e-6 off the plane and is still ON it.
#[test]
fn far_from_the_origin_the_corner_is_read_on_the_plane_to_a_float_step() {
    let offset = v(100.0, 100.0, 100.0);
    let normal = v(-1.0, 0.8, 0.0).get_normal();
    let positions = [
        v(0.4, 0.5, 0.0) + offset,
        v(0.8, 1.0, 0.0) + offset + (normal * 3e-6),
        v(3.0, 1.5, 0.0) + offset,
        v(5.0, 2.5, 0.0) + offset,
    ];
    let mesh =
        LevelLineMesh::from_triangles(&positions, &[(0, 1, 2), (1, 3, 2)], None).expect("builds");
    let run = run(vec![0.0, 0.1, 3.0, 5.0]);

    let (cut, corner_inside) =
        to_the_corner(&mesh, &run, false, offset, v(-1.0, 0.0, 0.0), normal, 2.0);

    let cut = cut.expect("a cut");
    assert_eq!(cut.vertices[0], 1);
    assert!(corner_inside);
}

/// A square end whose rail meets the plane outside the setback keeps cutting back to that point.
#[test]
fn a_square_end_still_cuts_back_to_the_first_vertex_outside_the_setback() {
    let positions = [
        v(0.0, 0.5, 0.0), // on the plane x = 0, inside
        v(0.0, 1.5, 0.0), // on the plane, inside
        v(0.0, 2.5, 0.0), // on the plane, outside: the corner
        v(3.0, 2.5, 0.0), // off the plane, outside
    ];
    let mesh =
        LevelLineMesh::from_triangles(&positions, &[(0, 1, 3), (1, 2, 3)], None).expect("builds");
    let run = run(vec![0.0, 0.0, 0.0, 3.0]);

    let (cut, corner_inside) = to_the_corner(
        &mesh,
        &run,
        false,
        Vector3::ZERO,
        v(-1.0, 0.0, 0.0),
        v(-1.0, 0.0, 0.0),
        2.0,
    );

    assert_eq!(cut.expect("a cut").vertices[0], 2);
    assert!(!corner_inside);
}
