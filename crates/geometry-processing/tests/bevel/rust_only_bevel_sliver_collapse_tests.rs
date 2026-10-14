//! Rust-only checks of [`bevel_sliver_collapse`], which the other tests only reach through the
//! published mesh: a sliver lying on its neighbour's edge is recut away, and a mesh with none is
//! left alone.

use geometry_processing::bevel::bevel_sliver_collapse::{altitude_of, collapse};
use polygon_mesh::{CancelToken, FaceList, Mesh};
use vector_math::Vector3Float;

/// A thin triangle (A, B, C) standing 1e-4 above its long edge AB, with the face across AB below.
fn sliver_on_a_face() -> Mesh {
    let vertices = vec![
        Vector3Float::new(0.0, 0.0, 0.0),
        Vector3Float::new(10.0, 0.0, 0.0),
        Vector3Float::new(5.0, 1e-4, 0.0),
        Vector3Float::new(5.0, -5.0, 0.0),
    ];
    let faces = FaceList::from_int_array(&[0, 1, 2, 1, 0, 3], &vertices);
    Mesh::from_vertices_and_faces(&vertices, &faces)
}

#[test]
fn rust_only_a_sliver_on_its_neighbours_edge_is_recut_away() {
    let mut mesh = sliver_on_a_face();
    let v = &mesh.vertices;
    let altitude = altitude_of(v[0], v[1], v[2]);
    assert!((altitude - 1e-4).abs() < 1e-6, "{altitude}");

    let changed = collapse(&mut mesh, 1e-3, &CancelToken::new()).expect("not cancelled");
    assert!(changed);
    assert_eq!(mesh.faces.len(), 2, "the pair is recut, never added to");
    for face in mesh.faces.iter() {
        let v = &mesh.vertices;
        let thick = altitude_of(
            v[face.v0 as usize],
            v[face.v1 as usize],
            v[face.v2 as usize],
        );
        assert!(thick > 1e-3, "{thick}");
        assert!(face.normal.z > 0.0, "both pieces face the neighbour's way");
    }

    // Nothing thin is left, so a second run changes nothing.
    assert!(!collapse(&mut mesh, 1e-3, &CancelToken::new()).expect("not cancelled"));
}

#[test]
fn rust_only_a_cancelled_collapse_stops_before_its_first_sweep() {
    let mut mesh = sliver_on_a_face();
    let token = CancelToken::new();
    token.cancel();
    assert!(collapse(&mut mesh, 1e-3, &token).is_err());
    assert_eq!(mesh.faces.len(), 2);
}
