//! Rust-only checks of the shapes, materials and tree queries that have no C# test of their
//! own: `SphereShape` front/back hits and its iterator, `SolidMaterial` colors through
//! `Primitive::get_color`, `TriangleShapeUv` hits, `GetCrossing`/`GetTouching` through a BVH,
//! `IntersectionIterator` through a `Transform`, and `wrap_up`.

use std::sync::Arc;

use agg_path::ColorF;
use polygon_mesh::ray_tracer::{
    wrap_up, BoundingVolumeHierarchy, BoxShape, BvhCreationOptions, MaterialRef, RayTracerError,
    SolidMaterial, SphereShape, Traceable, TraceableRef, Transform, TriangleShape, TriangleShapeUv,
};
use vector_math::{IntersectionType, Matrix4X4, Plane, Ray, Vector2Float, Vector3};

fn red() -> MaterialRef {
    Arc::new(SolidMaterial::new(ColorF::RED, 0.0, 0.0, 2.0))
}

#[test]
fn rust_only_sphere_front_and_back_hits() {
    let sphere = SphereShape::new(Vector3::ZERO, 1.0, Some(red()));
    let mut ray = Ray::new(Vector3::new(0.0, 0.0, -5.0), Vector3::UNIT_Z);
    let front = sphere.get_closest_intersection(&mut ray).unwrap();
    assert_eq!(front.hit_type, IntersectionType::FrontFace);
    assert_eq!(front.distance_to_hit, 4.0);
    assert_eq!(front.normal_at_hit, -Vector3::UNIT_Z);

    let mut back_ray = ray.clone();
    back_ray.intersection_type = IntersectionType::BackFace;
    let back = sphere.get_closest_intersection(&mut back_ray).unwrap();
    assert_eq!(back.hit_type, IntersectionType::BackFace);
    assert_eq!(back.distance_to_hit, 6.0);
    assert_eq!(back.normal_at_hit, -Vector3::UNIT_Z);

    let mut both = ray.clone();
    both.intersection_type = IntersectionType::Both;
    let hits = sphere.intersection_iterator(&both).unwrap();
    assert_eq!(hits.len(), 2);
    assert_eq!(hits[0].distance_to_hit, 4.0);
    assert_eq!(hits[1].distance_to_hit, 6.0);

    let color = front.closest_hit_object.unwrap().as_primitive().unwrap();
    assert_eq!(color.get_color(&front).unwrap(), ColorF::RED);
}

#[test]
fn rust_only_material_defaults_and_missing_material() {
    let material = SolidMaterial::new(ColorF::BLUE, 0.25, 0.5, 3.0);
    use polygon_mesh::ray_tracer::MaterialAbstract;
    assert_eq!(material.reflection(), 0.25);
    assert_eq!(material.transparency(), 0.5);
    assert_eq!(material.gloss(), 3.0);
    assert_eq!(material.refraction(), 0.5);
    assert!(!material.has_texture());

    let triangle = TriangleShape::new(
        Vector3::new(-1.0, -1.0, 0.0),
        Vector3::new(1.0, -1.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        -1,
    );
    let mut ray = Ray::new(Vector3::new(0.0, 0.0, 1.0), -Vector3::UNIT_Z);
    let hit = triangle.get_closest_intersection(&mut ray).unwrap();
    let primitive = hit.closest_hit_object.unwrap().as_primitive().unwrap();
    assert_eq!(primitive.get_color(&hit), Err(RayTracerError::NoMaterial));

    assert!((wrap_up(1.7) - -0.3).abs() < 1e-12);
    assert!((wrap_up(-1.1) - 0.9).abs() < 1e-12);
    assert!((wrap_up(-2.3) - -0.3).abs() < 1e-12);
}

#[test]
fn rust_only_triangle_shape_uv_hits_as_itself() {
    let triangle = TriangleShapeUv::new(
        Vector3::new(-1.0, -1.0, 0.0),
        Vector3::new(1.0, -1.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector2Float::new(0.0, 0.0),
        Vector2Float::new(1.0, 0.0),
        Vector2Float::new(0.5, 1.0),
        Some(red()),
        3,
    );
    let mut ray = Ray::new(Vector3::new(0.0, 0.0, 1.0), -Vector3::UNIT_Z);
    let hit = triangle.get_closest_intersection(&mut ray).unwrap();
    assert!(hit.hit_object_is(&triangle));
    let primitive = hit.closest_hit_object.unwrap().as_primitive().unwrap();
    assert_eq!(primitive.get_color(&hit).unwrap(), ColorF::RED);
    assert_eq!(triangle.triangle().index, 3);
}

#[test]
fn rust_only_crossing_and_touching_through_a_bvh() {
    let boxes: Vec<TraceableRef> = (0..4)
        .map(|i| {
            let x = f64::from(i) * 2.0;
            Arc::new(
                BoxShape::new(
                    Vector3::new(x, 0.0, 0.0),
                    Vector3::new(x + 1.0, 1.0, 1.0),
                    None,
                )
                .unwrap(),
            ) as TraceableRef
        })
        .collect();
    let tree = BoundingVolumeHierarchy::create_new_hierarchy(
        boxes,
        BvhCreationOptions::LocalOrderClustering,
    )
    .unwrap();

    // x = 2.5 crosses only the second box.
    let plane = Plane::new(Vector3::UNIT_X, 2.5);
    let crossing = tree.get_crossing(&plane).unwrap();
    assert_eq!(crossing.len(), 1);
    assert_eq!(crossing[0].get_axis_aligned_bounding_box().min_xyz.x, 2.0);

    let touching = tree
        .get_touching(Vector3::new(6.5, 0.5, 0.5), 0.001)
        .unwrap();
    assert_eq!(touching.len(), 1);
    assert_eq!(touching[0].get_axis_aligned_bounding_box().min_xyz.x, 6.0);

    let transformed = Transform::with_matrix(tree, Matrix4X4::IDENTITY);
    assert!(matches!(
        transformed.get_crossing(&plane),
        Err(RayTracerError::NotSupported(_))
    ));
}

#[test]
fn rust_only_intersection_iterator_through_a_transform() {
    let sphere: TraceableRef = Arc::new(SphereShape::new(Vector3::ZERO, 1.0, None));
    let moved = Transform::with_matrix(sphere, Matrix4X4::create_translation(0.0, 0.0, 10.0));
    let mut ray = Ray::new(Vector3::ZERO, Vector3::UNIT_Z);
    ray.intersection_type = IntersectionType::Both;
    let hits = moved.intersection_iterator(&ray).unwrap();
    assert_eq!(hits.len(), 2);
    assert_eq!(hits[0].hit_position, Vector3::new(0.0, 0.0, 9.0));
    assert_eq!(hits[1].hit_position, Vector3::new(0.0, 0.0, 11.0));
}
