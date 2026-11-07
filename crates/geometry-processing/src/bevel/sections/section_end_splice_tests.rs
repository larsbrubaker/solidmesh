//! Unit tests of `section_end_splice`'s private rim walk (`SwallowedAlong`), which needs private
//! access: the walk reaching a rim point the pool minted past the mesh, which must refuse rather
//! than read past the mesh's vertices.

use polygon_mesh::Mesh;
use vector_math::{Vector3, Vector3Float};

use super::swallowed_along;

/// A square rim on the mesh's four vertices, walked from vertex 0.
fn square() -> Mesh {
    let mut mesh = Mesh::new();
    mesh.vertices = vec![
        Vector3Float::new(0.0, 0.0, 0.0),
        Vector3Float::new(1.0, 0.0, 0.0),
        Vector3Float::new(2.0, 0.0, 0.0),
        Vector3Float::new(2.0, 1.0, 0.0),
    ];
    mesh
}

#[test]
fn rust_only_swallowed_along_reads_the_mesh_rim() {
    let mesh = square();

    // The landing half way along the second chord swallows vertex 1.
    assert_eq!(
        swallowed_along(&mesh, &[0, 1, 2, 3], 0, 1, Vector3::new(1.5, 0.0, 0.0)),
        1
    );
}

#[test]
fn rust_only_swallowed_along_refuses_a_rim_point_past_the_mesh() {
    let mesh = square();

    // Points 7 and 8 are another end's profile, minted into the pool past the mesh's four
    // vertices. Reading them as mesh vertices would run off the end; the walk refuses instead.
    let ring = [0, 1, 7, 8, 3];
    assert_eq!(
        swallowed_along(&mesh, &ring, 0, 1, Vector3::new(5.0, 0.0, 0.0)),
        -1
    );
    assert_eq!(
        swallowed_along(&mesh, &ring, 1, 1, Vector3::new(1.5, 0.0, 0.0)),
        -1
    );
}
