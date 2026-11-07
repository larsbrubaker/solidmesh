//! A `TriangleShape`'s projection axis and its front-face-only hits. The rays-on-a-circle case
//! needs the model's mesh-to-BVH conversion, so it lives in mattercad-model
//! (`crates/mattercad-model/tests/scene_core/polygon_trace_tests.rs`).

use std::sync::Arc;

use polygon_mesh::ray_tracer::{MaterialRef, SolidMaterial, Traceable, TriangleShape};
use vector_math::ColorF;
use vector_math::{IntersectionType, Ray, Vector3};

#[test]
fn triangle_major_axis() {
    let triangle0 = TriangleShape::new(
        Vector3::new(-47.06726, 16.94526, 1.143421),
        Vector3::new(-30.66048, 39.52726, 1.143422),
        Vector3::new(-45.91038, 19.8672, 1.143422),
        -1,
    );
    assert_eq!(triangle0.major_axis(), 2u8);

    let triangle1 = TriangleShape::new(
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(0.0, 0.0, 0.0),
        -1,
    );
    assert_eq!(triangle1.major_axis(), 0u8);

    let triangle2 = TriangleShape::new(
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(0.0, 0.0, 0.0),
        -1,
    );
    assert_eq!(triangle2.major_axis(), 1u8);
}

/// Whether `info`'s closest hit is exactly `shape`, the same object and not an equal one.
fn hit_is(info: &polygon_mesh::ray_tracer::IntersectInfo<'_>, shape: &TriangleShape) -> bool {
    info.closest_hit_as::<TriangleShape>()
        .is_some_and(|hit| std::ptr::eq(hit, shape))
}

#[test]
fn polygon_hit_tests() {
    let red_stuff: MaterialRef = Arc::new(SolidMaterial::new(
        ColorF::new(1.0, 0.0, 0.0, 1.0),
        0.0,
        0.0,
        2.0,
    ));
    {
        let facing_positive_x = TriangleShape::with_material(
            Vector3::new(0.0, 1.0, -1.0),
            Vector3::new(0.0, 0.0, 1.0),
            Vector3::new(0.0, -1.0, -1.0),
            Some(red_stuff.clone()),
            -1,
        );
        let mut ray = Ray::new(Vector3::new(1.0, 0.0, 0.0), Vector3::new(-1.0, 0.0, 0.0));
        let positive_x_info = facing_positive_x
            .get_closest_intersection(&mut ray)
            .expect("a hit");
        assert!(positive_x_info.hit_position == Vector3::new(0.0, 0.0, 0.0));
        assert!(positive_x_info.hit_type == IntersectionType::FrontFace);
        assert!(hit_is(&positive_x_info, &facing_positive_x));
        assert!(positive_x_info.distance_to_hit == 1.0);

        let mut ray = Ray::new(Vector3::new(-1.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
        let negative_x_info = facing_positive_x.get_closest_intersection(&mut ray);
        assert!(negative_x_info.is_none());
    }
    {
        let facing_negative_x = TriangleShape::with_material(
            Vector3::new(0.0, -1.0, -1.0),
            Vector3::new(0.0, 0.0, 1.0),
            Vector3::new(0.0, 1.0, -1.0),
            Some(red_stuff.clone()),
            -1,
        );
        let mut ray = Ray::new(Vector3::new(1.0, 0.0, 0.0), Vector3::new(-1.0, 0.0, 0.0));
        let positive_x_info = facing_negative_x.get_closest_intersection(&mut ray);
        assert!(positive_x_info.is_none());

        let mut ray = Ray::new(Vector3::new(-1.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
        let negative_x_info = facing_negative_x
            .get_closest_intersection(&mut ray)
            .expect("a hit");
        assert!(negative_x_info.hit_position == Vector3::new(0.0, 0.0, 0.0));
        assert!(negative_x_info.hit_type == IntersectionType::FrontFace);
        assert!(hit_is(&negative_x_info, &facing_negative_x));
        assert!(negative_x_info.distance_to_hit == 1.0);
    }
}
