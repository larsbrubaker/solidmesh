//! Splitting a mesh into its separate volumes (`processors::split_volumes`): each volume comes
//! out as its own mesh, a mesh of one volume comes out whole, and a cancel stops the walk.

use polygon_mesh::processors::split_volumes::split_volumes_into_meshes;
use polygon_mesh::solids::platonic_solids::create_cube;
use polygon_mesh::{CancelToken, Mesh};

/// Two 10 mm cubes 30 mm apart, as one mesh.
fn two_cubes() -> Mesh {
    let mut mesh = create_cube(10.0, 10.0, 10.0);
    let mut other = create_cube(10.0, 10.0, 10.0);
    other.translate_xyz(30.0, 0.0, 0.0);
    for face in other.faces.iter() {
        mesh.create_face_f32(&[
            other.vertices[face.v0 as usize],
            other.vertices[face.v1 as usize],
            other.vertices[face.v2 as usize],
        ]);
    }
    mesh
}

#[test]
fn rust_only_split_volumes_into_meshes_finds_each_volume() {
    let mut mesh = two_cubes();
    mesh.clean_and_merge();
    let volumes = split_volumes_into_meshes(&mesh, &CancelToken::new(), None).unwrap();
    assert_eq!(volumes.len(), 2);
    assert!(volumes.iter().all(|v| v.faces.len() == 12));

    let one = create_cube(10.0, 10.0, 10.0);
    let volumes = split_volumes_into_meshes(&one, &CancelToken::new(), None).unwrap();
    assert_eq!(volumes.len(), 1);

    let cancelled = CancelToken::new();
    cancelled.cancel();
    assert!(split_volumes_into_meshes(&one, &cancelled, None).is_none());
}

/// The progress told is the share of faces walked, rising to the last face's.
#[test]
fn rust_only_split_volumes_reports_the_share_of_faces_walked() {
    let mut mesh = two_cubes();
    mesh.clean_and_merge();
    let told = std::cell::RefCell::new(Vec::new());
    let report = |ratio: f64| told.borrow_mut().push(ratio);
    split_volumes_into_meshes(&mesh, &CancelToken::new(), Some(&report)).unwrap();

    let told = told.into_inner();
    assert_eq!(told.len(), 24);
    assert_eq!(told[0], 0.0);
    assert_eq!(told[23], 23.0 / 24.0);
    assert!(told.windows(2).all(|pair| pair[0] < pair[1]));
}
