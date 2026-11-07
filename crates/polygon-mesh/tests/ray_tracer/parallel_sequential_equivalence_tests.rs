//! The ray-tracer cases of the parallel/sequential equivalence checks: a BVH built with the
//! process-wide sequential flag on and off must be the same tree and trace the same hits. The
//! rest of the checks live in vector-math's
//! `tests/vector_math/parallel_sequential_equivalence_tests.rs`, which has no ray tracer.
//!
//! Not run here: the parallel binned-SAH BVH builder and the ray-traced image matching across
//! the flag (there is no binned-SAH builder or image renderer yet). Materials play no part in the
//! tree, so the triangles here have none.

use std::sync::{Arc, Mutex};

use polygon_mesh::ray_tracer::{
    BoundingVolumeHierarchy, BvhCreationOptions, BvhIterator, MinimalTriangle, Traceable,
    TraceableRef, Transform, TriangleShape, UnboundCollection,
};
use vector_math::{parallel, IntersectionType, Ray, Vector3};

/// Serializes the tests that flip the process-wide flag, so no test sees another's setting.
static SEQUENTIAL_FLAG: Mutex<()> = Mutex::new(());

const TRIANGLE_COUNT: usize = 800;
const TRIANGLE_SIZE: f64 = 3.0;

/// A 64-bit LCG returning values in [0, 1), so every run builds the same triangles.
struct DeterministicNumbers {
    state: u64,
}

impl DeterministicNumbers {
    fn new(seed: u64) -> Self {
        DeterministicNumbers { state: seed }
    }

    fn next(&mut self) -> f64 {
        self.state = self
            .state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (self.state >> 11) as f64 / (1u64 << 53) as f64
    }
}

fn triangle_corner(random: &mut DeterministicNumbers) -> Vector3 {
    let x = random.next() * 100.0;
    let y = random.next() * 100.0;
    let z = random.next() * 100.0;
    Vector3::new(x, y, z)
}

/// 800 deterministic triangles with a fixed x/y footprint, so `test_rays` can aim straight down
/// z at a point guaranteed to be inside each.
fn create_test_triangles() -> Vec<TraceableRef> {
    let mut random = DeterministicNumbers::new(12345);
    let mut triangles: Vec<TraceableRef> = Vec::with_capacity(TRIANGLE_COUNT);
    for _ in 0..TRIANGLE_COUNT {
        let corner = triangle_corner(&mut random);
        let z1 = random.next();
        let v1 = corner + Vector3::new(TRIANGLE_SIZE, 0.0, z1);
        let z2 = random.next();
        let v2 = corner + Vector3::new(0.0, TRIANGLE_SIZE, z2);
        triangles.push(Arc::new(TriangleShape::new(corner, v1, v2, -1)));
    }

    triangles
}

/// One ray per 13th triangle, fired down -z through its centroid in x/y.
fn test_rays() -> Vec<Ray> {
    let mut random = DeterministicNumbers::new(12345);
    let mut rays = Vec::new();
    for i in 0..TRIANGLE_COUNT {
        let corner = triangle_corner(&mut random);

        // Consume the same two values the triangle build consumes for its z offsets.
        random.next();
        random.next();

        if i % 13 != 0 {
            continue;
        }

        let target = corner + Vector3::new(TRIANGLE_SIZE / 3.0, TRIANGLE_SIZE / 3.0, 0.0);
        rays.push(Ray::new(
            Vector3::new(target.x, target.y, 1000.0),
            -Vector3::UNIT_Z,
        ));
    }

    rays
}

fn build_bvh(options: BvhCreationOptions, sequential: bool) -> TraceableRef {
    let was_sequential = parallel::sequential();
    parallel::set_sequential(sequential);
    let built = BoundingVolumeHierarchy::create_new_hierarchy(create_test_triangles(), options);
    parallel::set_sequential(was_sequential);
    built.expect("800 triangles build a tree")
}

fn type_name(item: &dyn Traceable) -> &'static str {
    let any = item.as_any();
    if any.is::<BoundingVolumeHierarchy>() {
        "BoundingVolumeHierarchy"
    } else if any.is::<TriangleShape>() {
        "TriangleShape"
    } else if any.is::<MinimalTriangle>() {
        "MinimalTriangle"
    } else if any.is::<UnboundCollection>() {
        "UnboundCollection"
    } else if any.is::<Transform>() {
        "Transform"
    } else {
        "Other"
    }
}

/// One string per node in walk order: depth, node type and bounds.
fn describe_bvh(root: &dyn Traceable) -> Vec<String> {
    BvhIterator::new(root)
        .iter()
        .map(|item| {
            let bounds = item.bvh.get_axis_aligned_bounding_box();
            format!(
                "{}|{}|{}|{}",
                item.depth,
                type_name(item.bvh),
                bounds.min_xyz,
                bounds.max_xyz
            )
        })
        .collect()
}

fn assert_same_bvh(expected: &dyn Traceable, actual: &dyn Traceable) {
    let expected_shape = describe_bvh(expected);
    let actual_shape = describe_bvh(actual);

    // Guards against a vacuous pass.
    assert!(expected_shape.len() > 100);
    assert_eq!(actual_shape.len(), expected_shape.len());
    assert_eq!(actual_shape, expected_shape);

    let mut hit_count = 0;
    for ray in test_rays() {
        let expected_hit = expected.get_closest_intersection(&mut ray.clone());
        let actual_hit = actual.get_closest_intersection(&mut ray.clone());

        assert_eq!(actual_hit.is_none(), expected_hit.is_none());
        if let (Some(expected_hit), Some(actual_hit)) = (expected_hit, actual_hit) {
            assert_eq!(actual_hit.hit_type, expected_hit.hit_type);
            assert_eq!(actual_hit.distance_to_hit, expected_hit.distance_to_hit);
            assert_eq!(actual_hit.hit_position, expected_hit.hit_position);
            if expected_hit.hit_type != IntersectionType::None {
                hit_count += 1;
            }
        }
    }

    assert!(hit_count > 0);
}

#[test]
fn locally_ordered_clustering_bvh_matches_across_sequential_flag() {
    let _guard = SEQUENTIAL_FLAG.lock().unwrap_or_else(|e| e.into_inner());
    let parallel_built = build_bvh(BvhCreationOptions::LocalOrderClustering, false);
    let sequential_built = build_bvh(BvhCreationOptions::LocalOrderClustering, true);

    assert_same_bvh(parallel_built.as_ref(), sequential_built.as_ref());
}
