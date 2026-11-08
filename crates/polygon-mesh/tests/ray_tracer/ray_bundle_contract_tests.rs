//! What a [`RayBundle`] promises its caller beyond the traversal itself, and the parts of the
//! lone-ray contract that only a test-built leaf can reach: a retraced bundle holds only the
//! last trace's results, a bad ray index is refused rather than a panic, every ray tests
//! exactly the leaves a lone ray tests in the same order, and the fallbacks for non-finite rays,
//! unsound boxes and widened windows do what they promise.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use solidmesh_math::{AxisAlignedBoundingBox, IntersectionType, Matrix4X4, Plane, Ray, Vector3};
use solidmesh_mesh::ray_tracer::{
    BoundingVolumeHierarchy, BvhCreationOptions, IntersectInfo, PlaneShape, RayBundle,
    RayTracerError, Traceable, TraceableRef, Transform, TriangleShape,
};

use super::ray_bundle_tests::{
    assert_bundles_match_lone, assert_same_hit, bvh, cube, irregular, jittered, perspective_grid,
    sphere, torus, v, Noise, Triangle, ALL_SIZES,
};

fn triangle(z: f64) -> TraceableRef {
    Arc::new(TriangleShape::new(
        Vector3::new(-1.0, -1.0, z),
        Vector3::new(1.0, -1.0, z),
        Vector3::new(0.0, 1.0, z),
        0,
    ))
}

#[test]
fn a_second_trace_replaces_the_first_traces_results() {
    let near_mesh = bvh(&cube());
    let far_away = triangle(-50.0);
    let rays = vec![Ray::new(v(0.1, 0.2, 9.0), v(0.0, 0.0, -1.0))];
    let mut bundle = RayBundle::new(rays);
    bundle.trace(near_mesh.as_ref()).expect("a valid bundle");
    assert!(bundle.results()[0].is_some(), "the ray hits the cube");

    // The ray still has the cube's narrowed window, so the far triangle is out of reach.
    bundle.trace(far_away.as_ref()).expect("a valid bundle");
    assert!(
        bundle.results()[0].is_none(),
        "the first trace's hit is gone"
    );
}

#[test]
fn hits_left_by_a_direct_call_do_not_reach_the_next_trace() {
    let mesh = bvh(&cube());
    let far_away = triangle(-50.0);
    let rays = vec![
        Ray::new(v(0.1, 0.2, 9.0), v(0.0, 0.0, -1.0)),
        Ray::new(v(0.3, 0.2, 9.0), v(0.0, 0.0, -1.0)),
    ];
    let mut bundle = RayBundle::new(rays);
    mesh.trace_ray_bundle(&mut bundle, &[0, 1])
        .expect("valid indices");

    bundle.trace(far_away.as_ref()).expect("a valid bundle");
    assert!(bundle.results()[0].is_none());
    assert!(bundle.results()[1].is_none());
}

#[test]
fn a_ray_index_past_the_end_is_refused_not_a_panic() {
    let shapes: [TraceableRef; 3] = [
        bvh(&cube()),
        triangle(0.0),
        Arc::new(Transform::new(bvh(&cube()))),
    ];
    for shape in shapes {
        let mut bundle = RayBundle::new(vec![Ray::new(v(0.0, 0.0, 5.0), v(0.0, 0.0, -1.0)); 2]);
        let outcome = shape.trace_ray_bundle(&mut bundle, &[99]);
        assert!(matches!(outcome, Err(RayTracerError::InvalidArgument(_))));
        let outcome = bundle.trace_one(2, shape.as_ref());
        assert!(matches!(outcome, Err(RayTracerError::InvalidArgument(_))));
    }
}

/// The bits of a ray's origin and direction: how a leaf recognises a ray that has no index.
type RayKey = [u64; 6];

fn ray_key(ray: &Ray) -> RayKey {
    [
        ray.origin.x.to_bits(),
        ray.origin.y.to_bits(),
        ray.origin.z.to_bits(),
        ray.direction_normal.x.to_bits(),
        ray.direction_normal.y.to_bits(),
        ray.direction_normal.z.to_bits(),
    ]
}

/// The order every ray asked every leaf, shared by a whole tree's leaves.
#[derive(Default)]
struct VisitLog(Mutex<Vec<(RayKey, u32)>>);

impl VisitLog {
    fn record(&self, ray: &Ray, leaf: u32) {
        self.0
            .lock()
            .expect("a test's log is never poisoned")
            .push((ray_key(ray), leaf));
    }

    /// Takes what has been recorded, as the sequence of leaves per ray.
    fn take_sequences(&self) -> HashMap<RayKey, Vec<u32>> {
        let visits = std::mem::take(&mut *self.0.lock().expect("a test's log is never poisoned"));
        let mut sequences: HashMap<RayKey, Vec<u32>> = HashMap::new();
        for (key, leaf) in visits {
            sequences.entry(key).or_default().push(leaf);
        }
        sequences
    }
}

/// A leaf that records each ray asking it, and either tests a real triangle or returns a
/// scripted hit, with bounds a test can set to whatever it needs (NaN, inverted).
struct RecordingLeaf {
    id: u32,
    log: Arc<VisitLog>,
    triangle: TriangleShape,
    bounds: AxisAlignedBoundingBox,
    script: Option<(f64, IntersectionType)>,
}

impl Traceable for RecordingLeaf {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn children(&self) -> Option<Vec<&dyn Traceable>> {
        None
    }

    fn axis_to_world(&self) -> Matrix4X4 {
        Matrix4X4::IDENTITY
    }

    fn get_surface_area(&self) -> f64 {
        self.triangle.get_surface_area()
    }

    fn get_axis_aligned_bounding_box(&self) -> AxisAlignedBoundingBox {
        self.bounds
    }

    fn get_center(&self) -> Vector3 {
        self.triangle.get_center()
    }

    fn get_axis_center(&self, axis: usize) -> f64 {
        self.triangle.get_axis_center(axis)
    }

    fn get_contained<'a>(
        &'a self,
        results: &mut Vec<&'a dyn Traceable>,
        sub_region: &AxisAlignedBoundingBox,
    ) -> bool {
        self.triangle.get_contained(results, sub_region)
    }

    fn contains(&self, position: Vector3) -> bool {
        self.triangle.contains(position)
    }

    fn get_closest_intersection<'a>(&'a self, ray: &mut Ray) -> Option<IntersectInfo<'a>> {
        self.log.record(ray, self.id);
        match self.script {
            Some((distance_to_hit, hit_type)) => Some(IntersectInfo {
                hit_type,
                closest_hit_object: Some(self),
                distance_to_hit,
                ..IntersectInfo::new()
            }),
            None => self.triangle.get_closest_intersection(ray),
        }
    }

    fn get_intersect_cost(&self) -> f64 {
        self.triangle.get_intersect_cost()
    }

    fn intersection_iterator<'a>(
        &'a self,
        ray: &Ray,
    ) -> Result<Vec<IntersectInfo<'a>>, RayTracerError> {
        self.triangle.intersection_iterator(ray)
    }

    fn get_crossing(
        &self,
        plane: &solidmesh_math::Plane,
    ) -> Result<Vec<&dyn Traceable>, RayTracerError> {
        self.triangle.get_crossing(plane)
    }

    fn get_touching(
        &self,
        position: Vector3,
        error: f64,
    ) -> Result<Vec<&dyn Traceable>, RayTracerError> {
        self.triangle.get_touching(position, error)
    }
}

/// A scripted or real leaf with explicit bounds.
fn leaf(
    log: &Arc<VisitLog>,
    id: u32,
    bounds: AxisAlignedBoundingBox,
    script: Option<(f64, IntersectionType)>,
) -> TraceableRef {
    Arc::new(RecordingLeaf {
        id,
        log: log.clone(),
        triangle: TriangleShape::new(
            v(5.0, -9.0, -9.0),
            v(5.0, 9.0, -9.0),
            v(5.0, 0.0, 9.0),
            id as i32,
        ),
        bounds,
        script,
    })
}

/// A tree over recording leaves wrapping `triangles`; the leaf ids are the triangle indices.
fn recording_bvh(triangles: &[Triangle], log: &Arc<VisitLog>) -> TraceableRef {
    let items: Vec<TraceableRef> = triangles
        .iter()
        .enumerate()
        .map(|(i, t)| {
            let triangle = TriangleShape::new(t[0], t[1], t[2], i as i32);
            Arc::new(RecordingLeaf {
                id: i as u32,
                log: log.clone(),
                bounds: triangle.get_axis_aligned_bounding_box(),
                triangle,
                script: None,
            }) as TraceableRef
        })
        .collect();
    BoundingVolumeHierarchy::create_new_hierarchy(items, BvhCreationOptions::LocalOrderClustering)
        .expect("a mesh has triangles")
}

/// Runs lone and bundled traces of `rays`, comparing every ray's result, final window and exact
/// sequence of leaf tests; returns the bundle's lone-side sequence count so a test can check it
/// has teeth.
fn assert_same_leaf_tests(
    label: &str,
    shape: &TraceableRef,
    log: &Arc<VisitLog>,
    rays: &[Ray],
    sizes: &[usize],
) -> usize {
    let mut lone_results = Vec::with_capacity(rays.len());
    let mut lone_windows = Vec::with_capacity(rays.len());
    for ray in rays {
        let mut lone = ray.clone();
        lone_results.push(shape.get_closest_intersection(&mut lone));
        lone_windows.push(lone.max_distance_to_consider);
    }
    let lone_sequences = log.take_sequences();

    for &size in sizes {
        let size = size.min(rays.len()).max(1);
        for (chunk_index, chunk) in rays.chunks(size).enumerate() {
            let mut bundle = RayBundle::new(chunk.to_vec());
            bundle.trace(shape.as_ref()).expect("a valid bundle");
            for (offset, ray) in chunk.iter().enumerate() {
                let index = chunk_index * size + offset;
                let label = format!("{label}, bundles of {size}");
                assert_same_hit(
                    &label,
                    index,
                    bundle.results()[offset].as_ref(),
                    lone_results[index],
                );
                assert_eq!(
                    bundle.rays()[offset].max_distance_to_consider.to_bits(),
                    lone_windows[index].to_bits(),
                    "{label} ray {index}: final window"
                );
                let _ = ray;
            }
        }
        let bundled_sequences = log.take_sequences();
        assert_eq!(
            bundled_sequences, lone_sequences,
            "{label}, bundles of {size}: a different sequence of leaf tests"
        );
    }
    lone_sequences.len()
}

/// Rays through a grid from varying eyes, every one distinct.
fn varied_rays() -> Vec<Ray> {
    let mut rays = perspective_grid(v(0.5, 0.3, 9.0), 32, 4.0);
    rays.extend(jittered(perspective_grid(v(-3.0, 2.0, 7.0), 24, 4.0), 0.3));
    for eye in [v(-7.0, 0.3, 4.0), v(7.0, 0.3, 4.0), v(0.3, -7.0, 4.0)] {
        rays.extend(perspective_grid(eye, 12, 5.0));
    }
    rays
}

#[test]
fn a_bundle_tests_exactly_the_leaves_lone_rays_test_in_the_same_order() {
    let log = Arc::new(VisitLog::default());
    for (name, triangles) in [
        ("irregular", irregular(48, 32)),
        ("torus", torus(40, 16)),
        ("cube", cube()),
    ] {
        let mesh = recording_bvh(&triangles, &log);
        let rays = varied_rays();
        let sequences = assert_same_leaf_tests(name, &mesh, &log, &rays, ALL_SIZES);
        assert!(sequences > 0, "{name}: no ray tested a leaf");
    }
}

/// A lone ray with a NaN origin component still passes every box it finds on the axes that
/// are not x, and goes on to test leaves; rays like it would be lost by anything that trusted
/// a bound over them.
fn non_finite_rays() -> Vec<Ray> {
    // Slanted so no direction component is zero: the interval test has a bound on every axis.
    let down = v(0.01, 0.02, -1.0);
    let values = [f64::NAN, f64::INFINITY, f64::NEG_INFINITY];
    let mut rays = Vec::new();
    // Regular rays first: they miss everything, so the interval test can speak for the bundle.
    for x in 0..8 {
        rays.push(Ray::new(v(x as f64 * 0.01, 100.0, 9.0), down));
    }
    for (i, &value) in values.iter().enumerate() {
        let shift = i as f64 * 0.013;
        rays.push(Ray::new(v(0.2 + shift, value, 9.0), down));
        rays.push(Ray::new(v(value, 0.2 + shift, 9.0), down));
        rays.push(Ray::new(v(0.2 + shift, 0.1, value), down));
        rays.push(Ray::new(v(0.2 + shift, 0.1, 9.0), v(value, 0.02, -1.0)));
        rays.push(Ray::new(v(0.3 + shift, 0.1, 9.0), v(0.01, value, -1.0)));
        rays.push(Ray::new(v(0.4 + shift, 0.1, 9.0), v(0.1, 0.02, value)));
    }
    rays
}

#[test]
fn rays_with_non_finite_values_match_lone_rays() {
    for (name, triangles) in [("cube", cube()), ("sphere", sphere(16, 12))] {
        let log = Arc::new(VisitLog::default());
        let mesh = recording_bvh(&triangles, &log);
        let sequences =
            assert_same_leaf_tests(name, &mesh, &log, &non_finite_rays(), &[2, 3, 7, 16, 64]);
        assert!(sequences > 0, "{name}: no non-finite ray tested a leaf");
    }
}

/// A node with NaN in its box falls back to lone rays. Without the fallback the interval test
/// reads the NaN as a bound on rays whose lone test ignores that plane entirely.
#[test]
fn a_node_with_a_nan_box_matches_lone_rays() {
    let log = Arc::new(VisitLog::default());
    let nan_top = AxisAlignedBoundingBox::new(v(5.0, 0.0, -1.0), v(6.0, f64::NAN, 1.0));
    let unsound: TraceableRef = Arc::new(BoundingVolumeHierarchy::new(
        leaf(&log, 0, nan_top, None),
        leaf(&log, 1, nan_top, None),
    ));
    let origin = v(0.0, 0.001, 0.0);
    let rays = vec![
        // Misses the box on z, so the bundle's first ray cannot speak for the rest.
        Ray::new(origin, v(1.0, 0.1, 5.0)),
        // Passes: the NaN on y gives its slab no upper limit.
        Ray::new(origin, v(1.0, 0.1, 0.05)),
        // Fails on y, and makes y's reciprocal direction mixed across the bundle.
        Ray::new(origin, v(1.0, -0.1, 0.05)),
    ];
    let sequences = assert_same_leaf_tests("NaN box", &unsound, &log, &rays, &[3]);
    assert!(sequences > 0, "the middle ray must reach a leaf");
}

/// A child whose box is NaN on one side sits outside what its parent's union test covers: the
/// rays it lets past would be carried in untested if the parent skipped its own check.
#[test]
fn nodes_with_unsound_children_match_lone_rays() {
    let log = Arc::new(VisitLog::default());
    let finite = AxisAlignedBoundingBox::new(v(-1.0, -1.0, -1.0), v(1.0, 1.0, 1.0));
    let nan_low_y = AxisAlignedBoundingBox::new(v(-1.0, f64::NAN, -1.0), v(1.0, 1.0, 1.0));
    let inverted = AxisAlignedBoundingBox::new(v(1.0, 1.0, 1.0), v(-1.0, -1.0, -1.0));
    for bounds in [nan_low_y, inverted] {
        let child: TraceableRef = Arc::new(BoundingVolumeHierarchy::new(
            leaf(&log, 0, bounds, None),
            leaf(&log, 1, bounds, None),
        ));
        // Both of the parent's children are nodes, so nothing gates the parent's own box: only
        // its refusal to descend past an unsound child tests the rays the parent would reject.
        let sibling: TraceableRef = Arc::new(BoundingVolumeHierarchy::new(
            leaf(&log, 2, finite, Some((4.0, IntersectionType::FrontFace))),
            leaf(&log, 3, finite, None),
        ));
        let parent: TraceableRef = Arc::new(BoundingVolumeHierarchy::new(child, sibling));
        // The centre first, so an early hit carries the rest down: outward rays that fail the
        // parent then reach a child that would let them through.
        let centre_out = || std::iter::once(0).chain((1..=8).flat_map(|k| [-k, k]));
        let mut rays = Vec::new();
        for y in centre_out() {
            for x in centre_out() {
                rays.push(Ray::new(
                    v(x as f64 * 0.31 + 0.01, y as f64 * 0.31 + 0.02, 9.0),
                    v(0.02, 0.03, -1.0),
                ));
            }
        }
        assert_same_leaf_tests("unsound child", &parent, &log, &rays, &[1, 7, 64, 289]);
    }
}

fn window_rays(noise_seed: u64) -> Vec<Ray> {
    let mut noise = Noise(noise_seed);
    let mut rays = Vec::new();
    for ray in perspective_grid(v(0.4, 0.2, 9.0), 32, 4.0) {
        let min = 2.0 + 10.0 * noise.next().abs();
        let max = min + 0.5 + 8.0 * noise.next().abs();
        let mut windowed = Ray::with_limits(
            ray.origin,
            ray.direction_normal,
            min,
            max,
            IntersectionType::Both,
        );
        windowed.is_shadow_ray = noise.next() > 0.3;
        rays.push(windowed);
    }
    rays
}

/// Rays whose windows differ ray by ray and start after their first hit, as a section plane
/// makes them: only the stretch past the cut counts.
#[test]
fn rays_with_minimum_distances_and_differing_windows_match_lone_rays() {
    for (name, triangles) in [("torus", torus(32, 16)), ("sphere", sphere(24, 16))] {
        let log = Arc::new(VisitLog::default());
        let mesh = recording_bvh(&triangles, &log);
        let sequences =
            assert_same_leaf_tests(name, &mesh, &log, &window_rays(11), &[1, 7, 64, 256, 1024]);
        assert!(sequences > 0, "{name}: no windowed ray tested a leaf");
    }
}

/// Every ray goes through the box, but its window starts past the far side: only the window
/// test can prove the miss.
#[test]
fn a_window_starting_past_the_box_is_proved_a_miss_by_the_interval_test() {
    let log = Arc::new(VisitLog::default());
    let mesh = recording_bvh(&sphere(16, 12), &log);
    let rays: Vec<Ray> = (0..16)
        .map(|i| {
            Ray::with_limits(
                v(i as f64 * 0.01, 0.1, 9.0),
                v(0.0, 0.0, -1.0),
                1000.0,
                f64::INFINITY,
                IntersectionType::FrontFace,
            )
        })
        .collect();
    let mut bundle = RayBundle::new(rays.clone());
    bundle.trace(mesh.as_ref()).expect("a valid bundle");
    assert!(bundle.stats().interval_misses > 0, "{:?}", bundle.stats());
    assert_same_leaf_tests("past the box", &mesh, &log, &rays, &[2, 16]);
    assert!(bundle.results().iter().all(Option::is_none));
}

/// A leaf that reports a hit behind the ray: a lone node keeps looking and returns the far
/// child's answer, even when that answer is nothing.
#[test]
fn a_hit_behind_the_ray_is_not_kept_when_the_far_child_finds_nothing() {
    let log = Arc::new(VisitLog::default());
    let bounds = AxisAlignedBoundingBox::new(v(-1.0, -1.0, -1.0), v(1.0, 1.0, 1.0));
    let behind = leaf(&log, 0, bounds, Some((-5.0, IntersectionType::FrontFace)));
    let empty = leaf(&log, 1, bounds, None);
    let tree: TraceableRef = Arc::new(BoundingVolumeHierarchy::new(behind, empty));
    let rays = vec![Ray::with_limits(
        v(-3.0, 0.1, 0.2),
        v(1.0, 0.0, 0.0),
        -100.0,
        f64::INFINITY,
        IntersectionType::FrontFace,
    )];
    let mut lone = rays[0].clone();
    assert!(
        tree.get_closest_intersection(&mut lone).is_none(),
        "the lone path returns the far child's nothing"
    );
    log.take_sequences();
    assert_same_leaf_tests("hit behind", &tree, &log, &rays, &[1]);
}

fn placed(mesh: &TraceableRef, matrix: Matrix4X4) -> TraceableRef {
    Arc::new(Transform::with_matrix(mesh.clone(), matrix))
}

#[test]
fn non_uniform_and_mirrored_transforms_under_a_bvh_match_lone_rays() {
    let mesh = bvh(&torus(24, 12));
    let stretched = Matrix4X4::create_scale(2.5, 0.4, 1.2);
    let mirrored = Matrix4X4::create_scale(-1.0, 1.0, 1.0)
        * Matrix4X4::create_rotation_y(0.6)
        * Matrix4X4::create_translation(-1.5, 0.3, 0.0);
    let turned = Matrix4X4::create_rotation_x(1.2) * Matrix4X4::create_translation(2.0, -1.0, 1.0);
    let inner = BoundingVolumeHierarchy::new(placed(&mesh, stretched), placed(&mesh, mirrored));
    let tree: TraceableRef = Arc::new(BoundingVolumeHierarchy::new(
        Arc::new(inner),
        placed(&mesh, turned),
    ));
    for eye in [v(0.4, 0.3, 14.0), v(-9.0, 4.0, 6.0)] {
        let rays = perspective_grid(eye, 32, 6.0);
        assert_bundles_match_lone("transforms in a tree", &tree, &rays, ALL_SIZES);
    }
}

/// A plane behind the mesh: the node narrows a ray to the mesh, then a plane that ignores its
/// window copies a farther distance over it, widening the ray. The bundle retraces those.
#[test]
fn a_plane_widening_a_rays_window_in_a_tree_is_retraced_alone() {
    let mesh = bvh(&sphere(16, 12));
    let plane: TraceableRef = Arc::new(PlaneShape::new(Plane::new(v(0.0, 0.0, 1.0), -3.0)));
    // Plane on the left: rays going -z visit the right child, the sphere, first.
    let tree: TraceableRef = Arc::new(BoundingVolumeHierarchy::new(plane, mesh));
    let rays = perspective_grid(v(0.5, 0.2, 15.0), 32, 7.0);
    let mut bundle = RayBundle::new(rays.clone());
    bundle.trace(tree.as_ref()).expect("a valid bundle");
    assert!(
        bundle.stats().retraced_rays > 0,
        "a ray's window was never widened: {:?}",
        bundle.stats()
    );
    assert_bundles_match_lone("plane behind", &tree, &rays, ALL_SIZES);
}
