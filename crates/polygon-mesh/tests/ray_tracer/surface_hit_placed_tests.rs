//! `PlacedParts` against the plain tree of `Transform`s it stands beside: the ray window narrows
//! the same way, a part's own bounds cut nothing off that the part would report, and a tree that
//! was not built from the table's mesh is refused instead of answering from the wrong surface.

use std::sync::Arc;

use solidmesh_math::{Matrix4X4, Ray, Vector3, Vector3Float};
use solidmesh_mesh::ray_tracer::{
    BoundingVolumeHierarchy, BvhCreationOptions, MeshSurface, PlacedPart, PlacedParts,
    RayTracerError, TraceableRef, Transform,
};
use solidmesh_mesh::solids::platonic_solids::create_cube;
use solidmesh_mesh::Mesh;

use crate::surface_hit_support::{tree_of, CREASE_30};

fn surface_of(mesh: &Mesh) -> Arc<MeshSurface> {
    Arc::new(MeshSurface::new(mesh, CREASE_30).unwrap())
}

fn parts_at(mesh: &Mesh, matrices: &[Matrix4X4]) -> PlacedParts {
    let tree = tree_of(mesh);
    let surface = surface_of(mesh);
    PlacedParts::new(
        matrices
            .iter()
            .map(|&matrix| PlacedPart::new(tree.clone(), surface.clone(), matrix).unwrap())
            .collect(),
    )
}

/// The same placements as a plain tree of `Transform` nodes.
fn tree_of_transforms(mesh: &Mesh, matrices: &[Matrix4X4]) -> TraceableRef {
    let tree = tree_of(mesh);
    let leaves = matrices
        .iter()
        .map(|&matrix| Arc::new(Transform::with_matrix(tree.clone(), matrix)) as TraceableRef)
        .collect();
    BoundingVolumeHierarchy::create_new_hierarchy(leaves, BvhCreationOptions::default())
        .expect("there are parts")
}

/// A lone part narrows the ray to its hit, as its doc promises and as a tree does: the tracer
/// reads the window to stop looking past what it has already found.
#[test]
fn a_lone_part_narrows_the_ray_to_its_hit() {
    let mesh = create_cube(10.0, 10.0, 10.0);
    let scene = parts_at(&mesh, &[Matrix4X4::IDENTITY]);
    let mut ray = Ray::new(Vector3::new(1.0, 2.0, 50.0), -Vector3::UNIT_Z);
    let hit = scene.trace(&mut ray).expect("the cube is under the ray");
    assert_eq!(hit.info.distance_to_hit, 45.0);
    assert_eq!(ray.max_distance_to_consider, 45.0);
}

/// Whichever child of the top-level tree a part ends up, the window narrows to the hit, the same
/// as the plain tree of transforms narrows it.
#[test]
fn the_window_narrows_on_a_hit_in_either_child_of_the_tree() {
    let mesh = create_cube(10.0, 10.0, 10.0);
    let matrices = [
        Matrix4X4::create_translation(-20.0, 0.0, 0.0),
        Matrix4X4::create_translation(20.0, 0.0, 0.0),
    ];
    let scene = parts_at(&mesh, &matrices);
    let plain = tree_of_transforms(&mesh, &matrices);

    for x in [-20.0, 20.0] {
        let start = Vector3::new(x, 2.0, 50.0);
        let mut ray = Ray::new(start, -Vector3::UNIT_Z);
        let hit = scene.trace(&mut ray).expect("a part is under the ray");
        let mut same = Ray::new(start, -Vector3::UNIT_Z);
        let plain_hit = plain
            .get_closest_intersection(&mut same)
            .expect("the plain tree finds it too");

        assert_eq!(hit.info.distance_to_hit, plain_hit.distance_to_hit);
        assert_eq!(
            ray.max_distance_to_consider, same.max_distance_to_consider,
            "the ray through x = {x} narrowed differently from the plain tree"
        );
        assert_eq!(ray.max_distance_to_consider, hit.info.distance_to_hit);
    }
}

/// A shadow ray stops at its first hit, so like a tree's it keeps the window it came with.
#[test]
fn a_shadow_ray_keeps_its_window_as_a_tree_does() {
    let mesh = create_cube(10.0, 10.0, 10.0);
    let scene = parts_at(&mesh, &[Matrix4X4::IDENTITY]);
    let mut ray = Ray::new(Vector3::new(1.0, 2.0, 50.0), -Vector3::UNIT_Z);
    ray.is_shadow_ray = true;
    assert!(scene.trace(&mut ray).is_some());
    assert_eq!(ray.max_distance_to_consider, f64::INFINITY);
}

/// A triangle counts a ray within its float slack of an edge as a hit, so a ray just outside its
/// box can still hit it. A part's bounds must not throw away what the part would have reported:
/// the set of parts answers exactly as the tree of transforms does.
#[test]
fn a_parts_own_box_cuts_off_nothing_the_part_would_report() {
    let triangle = Mesh::from_arrays(&[0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 10.0, 0.0], &[0, 1, 2]);
    let triangle_surface = surface_of(&triangle);
    let triangle_tree = tree_of(&triangle);
    let cube = create_cube(10.0, 10.0, 10.0);
    let cube_surface = surface_of(&cube);
    let cube_tree = tree_of(&cube);
    let beside = Matrix4X4::create_translation(-30.0, 0.0, 0.0);

    let scene = PlacedParts::new(vec![
        PlacedPart::new(triangle_tree.clone(), triangle_surface, Matrix4X4::IDENTITY).unwrap(),
        PlacedPart::new(cube_tree.clone(), cube_surface, beside).unwrap(),
    ]);
    let plain = BoundingVolumeHierarchy::create_new_hierarchy(
        vec![
            Arc::new(Transform::with_matrix(triangle_tree, Matrix4X4::IDENTITY)) as TraceableRef,
            Arc::new(Transform::with_matrix(cube_tree, beside)) as TraceableRef,
        ],
        BvhCreationOptions::default(),
    )
    .expect("there are parts");

    let start = Vector3::new(-1e-5, 1.0, 100.0);
    let mut ray = Ray::new(start, -Vector3::UNIT_Z);
    let plain_hit = plain.get_closest_intersection(&mut ray.clone());
    let hit = scene.trace(&mut ray);
    assert!(plain_hit.is_some(), "the fixture must be a hit in the tree");
    assert_eq!(
        hit.map(|hit| hit.info.distance_to_hit),
        plain_hit.map(|hit| hit.distance_to_hit),
        "the set of parts must agree with the tree of transforms"
    );
}

/// A table from another mesh would answer every hit from the wrong surface with no sign of it,
/// so a tree and table that do not describe the same bounds are refused, by both constructors.
#[test]
fn a_tree_from_another_mesh_is_refused() {
    let cube = create_cube(10.0, 10.0, 10.0);
    let tree = tree_of(&cube);
    let mut far_box = create_cube(1.0, 1.0, 1.0);
    far_box.transform(Matrix4X4::create_translation(100.5, 100.5, 100.5));
    let wrong_table = surface_of(&far_box);

    let refused = PlacedPart::new(tree.clone(), wrong_table.clone(), Matrix4X4::IDENTITY);
    assert!(matches!(
        refused,
        Err(RayTracerError::InvalidArgument(text)) if text.contains("mesh")
    ));
    let chain = Transform::with_matrix(tree.clone(), Matrix4X4::IDENTITY);
    assert!(PlacedPart::from_transform(&chain, wrong_table).is_err());

    // A table with no faces cannot describe a tree that has some.
    let empty = Arc::new(MeshSurface::new(&Mesh::new(), CREASE_30).unwrap());
    assert!(PlacedPart::new(tree.clone(), empty, Matrix4X4::IDENTITY).is_err());

    // The check is on the mesh's own space: the matrix does not enter it.
    let right_table = surface_of(&cube);
    let placed =
        Matrix4X4::create_scale(3.0, 1.0, 0.5) * Matrix4X4::create_translation(1000.0, -40.0, 7.0);
    assert!(PlacedPart::new(tree.clone(), right_table.clone(), placed).is_ok());
    let chain = Transform::with_matrix(tree, placed);
    assert!(PlacedPart::from_transform(&chain, right_table).is_ok());
}

/// The refusal must not turn away a real mesh's own tree: one with a degenerate face and unused
/// vertices far away, and one with a NaN corner.
#[test]
fn a_messy_mesh_is_accepted_with_its_own_tree() {
    // A vertex no face uses, far outside the part, must not count towards the table's bounds.
    let unused = Mesh::from_arrays(
        &[
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 1.0, 0.0, 900.0, 900.0, 900.0,
        ],
        &[0, 1, 2, 0, 1, 3],
    );
    assert!(PlacedPart::new(tree_of(&unused), surface_of(&unused), Matrix4X4::IDENTITY).is_ok());

    let mut with_nan = Mesh::from_arrays(
        &[
            0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 4.0, 0.0, 4.0, 4.0, 0.0, 9.0, 9.0, 0.0,
        ],
        &[0, 1, 2, 1, 3, 2, 3, 4, 2],
    );
    with_nan.vertices[4] = Vector3Float::new_f32(f32::NAN, 9.0, 0.0);
    assert!(PlacedPart::new(
        tree_of(&with_nan),
        surface_of(&with_nan),
        Matrix4X4::IDENTITY
    )
    .is_ok());
}
