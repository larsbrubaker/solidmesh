//! Ports `GeometryProcessingTests/SurfaceMeshClosureTests.cs`: the gate the surface
//! construction's answer has to pass before anything is published - every edge used by exactly two
//! faces. It is read on the built mesh because no step of the build can see it - a step knows the
//! ring it emitted, not whether the ring next door emitted the other side of it.

use geometry_processing::bevel::surface::surface_mesh_closure;
use polygon_mesh::solids::platonic_solids;
use polygon_mesh::Mesh;
use vector_math::Vector3;

/// A solid nothing has been done to is closed, and says nothing about where.
#[test]
fn a_solid_is_closed() {
    let (closed, open_at) =
        surface_mesh_closure::is_closed(&platonic_solids::create_cube(20.0, 20.0, 20.0));

    assert!(closed);
    assert_eq!(open_at, Vector3::ZERO);
}

/// A triangle taken out leaves its three edges used once each, and the place it names is one of
/// them - a place a refusal can be shown at rather than a bare "something went wrong".
#[test]
fn a_missing_triangle_is_three_open_edges_and_a_place_to_show() {
    let cube = platonic_solids::create_cube(20.0, 20.0, 20.0);
    let dropped = without(&cube, 0);

    let (open, open_at) = surface_mesh_closure::open_edge_count(&dropped);
    assert_eq!(open, 3);
    assert!(!surface_mesh_closure::is_closed(&dropped).0);

    let face = &cube.faces[0];
    let nearest = [face.v0, face.v1, face.v2]
        .iter()
        .map(|&corner| {
            (Vector3::from_vector3_float(cube.vertices[corner as usize]) - open_at).length()
        })
        .fold(f64::INFINITY, f64::min);

    assert!(
        nearest < 20.0,
        "the place named is the middle of one of the edges the missing triangle left open"
    );
}

/// And a surface doubled back over itself is not closed either, which is the failure this gate
/// exists for: manifoldness and shell count both accept a fold, and the edge use count does not.
#[test]
fn a_folded_over_face_is_not_closed_either() {
    let mut cube = platonic_solids::create_cube(20.0, 20.0, 20.0);
    let face = cube.faces[0];

    cube.faces.add(face.v0, face.v2, face.v1, &cube.vertices);

    assert_eq!(
        surface_mesh_closure::open_edge_count(&cube).0,
        3,
        "the three edges of the doubled triangle are used three times each"
    );
}

/// The same mesh less one face, which is the only way this fixture is built.
fn without(mesh: &Mesh, dropped: usize) -> Mesh {
    let mut left = Mesh::new();

    for &vertex in &mesh.vertices {
        left.vertices.push(vertex);
    }

    for (index, face) in mesh.faces.iter().enumerate() {
        if index == dropped {
            continue;
        }

        left.faces.add(face.v0, face.v1, face.v2, &left.vertices);
    }

    left
}
