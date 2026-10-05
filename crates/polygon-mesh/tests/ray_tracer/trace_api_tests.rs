//! Port of `MatterHackers.RayTracer.TraceAPITests` (agg-sharp
//! `Tests/Agg.Tests/Agg.RayTracer/TraceAPITests.cs`). `PlaneGetDistanceToIntersection` uses only
//! vector-math and lives in `crates/vector-math/tests/ray_tracer/trace_api_tests.rs`.

use std::sync::Arc;

use polygon_mesh::ray_tracer::{
    BvhIterator, Traceable, TraceableRef, Transform, TriangleShape, UnboundCollection,
};
use vector_math::{Matrix4X4, Vector3};

fn is<T: Traceable>(item: &BvhIterator<'_, '_>) -> bool {
    item.bvh.as_any().is::<T>()
}

#[test]
fn enumerate_bvh() {
    // create a bvh hierarchy
    let level4_a: TraceableRef = Arc::new(TriangleShape::new(
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(0.0, 0.0, 3.0),
        Vector3::new(0.0, 1.0, 2.0),
        -1,
    ));
    let level4_b: TraceableRef = Arc::new(TriangleShape::new(
        Vector3::new(0.0, 0.0, 11.0),
        Vector3::new(0.0, 0.0, 13.0),
        Vector3::new(0.0, 1.0, 12.0),
        -1,
    ));
    let level4_c: TraceableRef = Arc::new(TriangleShape::new(
        Vector3::new(3.0, 0.0, 1.0),
        Vector3::new(3.0, 0.0, 3.0),
        Vector3::new(3.0, 1.0, 2.0),
        -1,
    ));
    let level3_a: TraceableRef =
        Arc::new(UnboundCollection::new(vec![level4_a, level4_b, level4_c]));
    let level3_b: TraceableRef = Arc::new(TriangleShape::new(
        Vector3::new(43.0, 0.0, 1.0),
        Vector3::new(43.0, 0.0, 3.0),
        Vector3::new(43.0, 1.0, 2.0),
        -1,
    ));
    let level2_a: TraceableRef = Arc::new(Transform::with_matrix(
        level3_a,
        Matrix4X4::create_translation(0.0, 0.0, 40.0),
    ));
    let level2_b: TraceableRef = Arc::new(Transform::with_matrix(
        level3_b,
        Matrix4X4::create_translation(0.0, 40.0, 0.0),
    ));
    let level1: TraceableRef = Arc::new(UnboundCollection::new(vec![level2_a, level2_b]));
    let root = Transform::new(level1);

    // enumerate it and check it
    assert_eq!(BvhIterator::new(&root).iter().count(), 9);

    let none = Matrix4X4::create_translation(0.0, 0.0, 0.0);
    let up_z = Matrix4X4::create_translation(0.0, 0.0, 40.0);
    let up_y = Matrix4X4::create_translation(0.0, 40.0, 0.0);
    for (count, item) in BvhIterator::new(&root).iter().enumerate() {
        let (type_ok, depth, transform) = match count {
            0 => (is::<Transform>(&item), 0, none),
            1 => (is::<UnboundCollection>(&item), 1, none),
            2 => (is::<Transform>(&item), 2, none),
            3 => (is::<UnboundCollection>(&item), 3, up_z),
            4..=6 => (is::<TriangleShape>(&item), 4, up_z),
            7 => (is::<Transform>(&item), 2, none),
            8 => (is::<TriangleShape>(&item), 3, up_y),
            _ => unreachable!("only nine items"),
        };
        assert!(type_ok, "item {count} has the wrong type");
        assert_eq!(item.depth, depth, "item {count}");
        assert_eq!(item.transform_to_world, transform, "item {count}");
    }
}
