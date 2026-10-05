//! Rust-only checks of the ray tracer's trace path (`polygon_mesh::ray_tracer`): a Morton-built
//! hierarchy finds the same nearest hits as a flat list, a pick recovers the hit face through
//! `MinimalTriangle::face_index`, `Transform` moves rays in and hits out, and every traceable
//! can cross threads.

use std::sync::Arc;

use polygon_mesh::ray_tracer::{
    BoundingVolumeHierarchy, BvhCreationOptions, BvhIterator, MinimalTriangle, PlaneShape,
    Traceable, TraceableRef, Transform, TriangleShape, UnboundCollection,
};
use polygon_mesh::Mesh;
use vector_math::{IntersectionType, Matrix4X4, Ray, Vector3};

/// MatterCAD's `AddTraceables` without a material: one `MinimalTriangle` per face reading the
/// mesh's corners.
fn mesh_traceables(mesh: Arc<Mesh>) -> Vec<TraceableRef> {
    (0..mesh.faces.len() as i32)
        .map(|face_index| {
            let mesh = mesh.clone();
            let triangle: TraceableRef = Arc::new(MinimalTriangle::new(
                Arc::new(move |fi, vi| {
                    let face = &mesh.faces[fi as usize];
                    let vertex = match vi {
                        0 => face.v0,
                        1 => face.v1,
                        _ => face.v2,
                    };
                    mesh.vertices[vertex as usize]
                }),
                face_index,
            ));
            triangle
        })
        .collect()
}

fn grid_mesh() -> Arc<Mesh> {
    let mut mesh = Mesh::new();
    for y in 0..20 {
        for x in 0..20 {
            let (x, y) = (f64::from(x), f64::from(y));
            let z = (x * 0.37 + y * 0.11).sin();
            mesh.create_face(&[
                Vector3::new(x, y, z),
                Vector3::new(x + 1.0, y, z),
                Vector3::new(x + 1.0, y + 1.0, z + 0.5),
            ]);
        }
    }

    Arc::new(mesh)
}

#[test]
fn rust_only_morton_hierarchy_finds_the_same_hits_as_a_flat_list() {
    let mesh = grid_mesh();
    let traceables = mesh_traceables(mesh.clone());
    let flat = BoundingVolumeHierarchy::create_new_hierarchy(
        traceables.clone(),
        BvhCreationOptions::SingleUnboundCollection,
    )
    .unwrap();
    let tree =
        BoundingVolumeHierarchy::create_new_hierarchy(traceables, BvhCreationOptions::default())
            .unwrap();
    assert!(tree.as_any().is::<BoundingVolumeHierarchy>());

    // Every face is a leaf exactly once.
    let leaves = BvhIterator::new(tree.as_ref())
        .iter()
        .filter(|item| item.bvh.as_any().is::<MinimalTriangle>())
        .count();
    assert_eq!(leaves, mesh.faces.len());

    let mut hits = 0;
    for i in 0..200 {
        let x = 0.3 + f64::from(i % 20) + f64::from(i / 20) * 0.013;
        let y = 0.2 + f64::from(i / 10) * 0.97;
        let ray = Ray::new(Vector3::new(x, y, 50.0), -Vector3::UNIT_Z);
        let expected = flat.get_closest_intersection(&mut ray.clone());
        let actual = tree.get_closest_intersection(&mut ray.clone());
        assert_eq!(expected.is_some(), actual.is_some(), "ray {i}");
        if let (Some(expected), Some(actual)) = (expected, actual) {
            hits += 1;
            assert_eq!(expected.distance_to_hit, actual.distance_to_hit, "ray {i}");
            assert_eq!(
                expected
                    .closest_hit_as::<MinimalTriangle>()
                    .unwrap()
                    .face_index,
                actual
                    .closest_hit_as::<MinimalTriangle>()
                    .unwrap()
                    .face_index
            );
        }
    }
    assert!(hits > 50);
}

#[test]
fn rust_only_transform_moves_rays_in_and_hits_out() {
    let triangle: TraceableRef = Arc::new(TriangleShape::new(
        Vector3::new(-1.0, -1.0, 0.0),
        Vector3::new(1.0, -1.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        7,
    ));
    let moved = Transform::with_matrix(triangle, Matrix4X4::create_translation(10.0, 0.0, 5.0));
    let mut ray = Ray::new(Vector3::new(10.0, 0.0, 20.0), -Vector3::UNIT_Z);
    let hit = moved.get_closest_intersection(&mut ray).unwrap();
    assert_eq!(hit.hit_type, IntersectionType::FrontFace);
    assert_eq!(hit.distance_to_hit, 15.0);
    assert_eq!(hit.hit_position, Vector3::new(10.0, 0.0, 5.0));
    assert_eq!(hit.closest_hit_as::<TriangleShape>().unwrap().index, 7);
    assert_eq!(
        moved.get_axis_aligned_bounding_box().min_xyz,
        Vector3::new(9.0, -1.0, 5.0)
    );

    // The untransformed ray misses.
    let mut miss = Ray::new(Vector3::new(0.0, 0.0, 20.0), -Vector3::UNIT_Z);
    assert!(moved.get_closest_intersection(&mut miss).is_none());
}

#[test]
fn rust_only_plane_shape_hits_in_front_and_within_range() {
    let bed = PlaneShape::from_normal_and_distance(Vector3::UNIT_Z, 0.0);
    let mut ray = Ray::new(Vector3::new(1.0, 2.0, 3.0), -Vector3::UNIT_Z);
    let hit = bed.get_closest_intersection(&mut ray).unwrap();
    assert_eq!(hit.hit_position, Vector3::new(1.0, 2.0, 0.0));
    assert_eq!(hit.distance_to_hit, 3.0);

    // Pointing away: behind the origin is not a hit.
    let mut away = Ray::new(Vector3::new(1.0, 2.0, 3.0), Vector3::UNIT_Z);
    assert!(bed.get_closest_intersection(&mut away).is_none());

    let mut short = ray.clone();
    short.max_distance_to_consider = 2.0;
    assert!(bed
        .get_closest_intersection_within_ray_distance_range(&short)
        .is_none());
    assert!(bed
        .get_closest_intersection_within_ray_distance_range(&ray)
        .is_some());
}

#[test]
fn rust_only_collections_take_the_nearest_hit() {
    let near: TraceableRef = Arc::new(TriangleShape::new(
        Vector3::new(-1.0, -1.0, 2.0),
        Vector3::new(1.0, -1.0, 2.0),
        Vector3::new(0.0, 1.0, 2.0),
        1,
    ));
    let far: TraceableRef = Arc::new(TriangleShape::new(
        Vector3::new(-1.0, -1.0, 0.0),
        Vector3::new(1.0, -1.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        2,
    ));
    let collection = UnboundCollection::new(vec![far.clone(), near.clone()]);
    let node = BoundingVolumeHierarchy::new(far, near);
    for traceable in [&collection as &dyn Traceable, &node] {
        let mut ray = Ray::new(Vector3::new(0.0, 0.0, 10.0), -Vector3::UNIT_Z);
        let hit = traceable.get_closest_intersection(&mut ray).unwrap();
        assert_eq!(hit.closest_hit_as::<TriangleShape>().unwrap().index, 1);
        assert_eq!(hit.distance_to_hit, 8.0);
        assert!(traceable.contains(Vector3::new(0.0, 0.0, 2.0)));
    }
}

#[test]
fn rust_only_traceables_are_send_and_sync() {
    fn assert_send_sync<T: Send + Sync + ?Sized>() {}
    assert_send_sync::<dyn Traceable>();
    assert_send_sync::<BoundingVolumeHierarchy>();
    assert_send_sync::<MinimalTriangle>();

    let tree = BoundingVolumeHierarchy::create_new_hierarchy(
        mesh_traceables(grid_mesh()),
        BvhCreationOptions::LocalOrderClustering,
    )
    .unwrap();
    let handle = std::thread::spawn(move || {
        let mut ray = Ray::new(Vector3::new(5.5, 5.2, 50.0), -Vector3::UNIT_Z);
        tree.get_closest_intersection(&mut ray).is_some()
    });
    assert!(handle.join().unwrap());
}
