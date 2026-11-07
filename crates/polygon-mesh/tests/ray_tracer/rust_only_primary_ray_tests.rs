//! Rust-only checks of `RayTracer::trace_primary_ray` and `Scene`: the nearest hit across several
//! shapes wins, a miss reports no hit, and moving the shapes into a hierarchy keeps what a ray
//! hits.

use std::sync::Arc;

use polygon_mesh::ray_tracer::{
    AntiAliasing, Background, BoundingVolumeHierarchy, RayTracer, Scene, SphereShape, TraceableRef,
};
use vector_math::ColorF;
use vector_math::{IntersectionType, Ray, Vector3};

fn sphere(z: f64) -> TraceableRef {
    Arc::new(SphereShape::new(Vector3::new(0.0, 0.0, z), 1.0, None))
}

fn down_ray() -> Ray {
    Ray::new(Vector3::new(0.0, 0.0, 100.0), -Vector3::UNIT_Z)
}

#[test]
fn rust_only_ray_tracer_and_scene_defaults_are_pinned() {
    let tracer = RayTracer::new();
    assert_eq!(tracer.anti_aliasing, AntiAliasing::Medium);
    assert_eq!(AntiAliasing::VeryHigh as i32, 32);
    assert!(tracer.multi_threaded);
    assert!(tracer.render_diffuse && tracer.render_highlights && tracer.render_shadow);
    assert!(tracer.render_reflection && tracer.render_refraction);
    assert!(!tracer.trace_with_ray_bundles);

    let scene = Scene::new();
    assert!(scene.shapes.is_empty());
    assert_eq!(
        scene.background,
        Background::new(ColorF::new(0.0, 0.0, 0.0, 0.0), 0.2)
    );
}

#[test]
fn rust_only_trace_primary_ray_takes_the_nearest_shape() {
    let mut scene = Scene::new();
    // the far sphere first, so the near one has to replace its hit
    scene.shapes.push(sphere(0.0));
    scene.shapes.push(sphere(10.0));
    let tracer = RayTracer::new();

    let mut ray = down_ray();
    let info = tracer.trace_primary_ray(&mut ray, &scene);
    assert_eq!(info.hit_type, IntersectionType::FrontFace);
    assert!((info.distance_to_hit - 89.0).abs() < 1e-9, "{info:?}");
    assert!(info.hit_object_is(scene.shapes[1].as_ref()));

    let mut miss = Ray::new(Vector3::new(5.0, 0.0, 100.0), -Vector3::UNIT_Z);
    let info = tracer.trace_primary_ray(&mut miss, &scene);
    assert_eq!(info.hit_type, IntersectionType::None);
    assert_eq!(info.distance_to_hit, f64::MAX);
    assert!(info.closest_hit_object.is_none());

    let empty = Scene::new();
    let info = tracer.trace_primary_ray(&mut down_ray(), &empty);
    assert_eq!(info.hit_type, IntersectionType::None);
}

#[test]
fn rust_only_move_shapes_into_hierarchy_keeps_the_hits() {
    let mut scene = Scene::new();
    assert!(scene.move_shapes_into_bounding_volume_hierarchy().is_none());
    assert!(scene.shapes.is_empty());

    scene.shapes.push(sphere(0.0));
    scene.shapes.push(sphere(10.0));
    scene.shapes.push(sphere(-10.0));
    let root = scene
        .move_shapes_into_bounding_volume_hierarchy()
        .expect("a hierarchy");
    assert_eq!(scene.shapes.len(), 1);
    assert!(Arc::ptr_eq(&scene.shapes[0], &root));
    assert!(root
        .as_any()
        .downcast_ref::<BoundingVolumeHierarchy>()
        .is_some());

    let info = RayTracer::new().trace_primary_ray(&mut down_ray(), &scene);
    assert_eq!(info.hit_type, IntersectionType::FrontFace);
    assert!((info.distance_to_hit - 89.0).abs() < 1e-9, "{info:?}");
}
