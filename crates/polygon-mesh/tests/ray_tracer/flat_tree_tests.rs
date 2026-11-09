//! The flat GPU tree against the existing `f64` BVH: same hits on varied meshes for large
//! deterministic ray sets, conservative bounds, the byte layout a WGSL struct mirrors,
//! determinism, the refusals, and the fixed-stack bound. The fixtures and the comparison rules
//! are in `flat_tree_support`.

use solidmesh_math::{IntersectionType, Ray, Vector3};
use solidmesh_mesh::ray_tracer::flat_tree::{
    bounds_max_f32, bounds_min_f32, child_code, FlatNode, FlatRay, FlatTree, FlatTreeBuffer,
    FlatTreeError, FlatTriangle, FACE_BACK, FACE_BOTH, FACE_FRONT, MAX_LEAF_TRIANGLES,
    MAX_TREE_DEPTH, NODE_STRIDE, STACK_CAPACITY, TRIANGLE_STRIDE,
    WEBGPU_DEFAULT_MAX_STORAGE_BINDING,
};

use crate::flat_tree_support::{
    bumpy_sphere, camera_rays, compare, compare_rays, flat_grid, long_thin_tube, random_rays,
    rays_at_triangles, scene_100k, straight_down_rays, tied_centroids, v,
};

fn compare_all(triangles: &[[Vector3; 3]], what: &str) {
    for (name, rays) in [
        (
            "camera front",
            camera_rays(triangles, 200, IntersectionType::FrontFace),
        ),
        (
            "camera both",
            camera_rays(triangles, 200, IntersectionType::Both),
        ),
        (
            "random front",
            random_rays(triangles, 40_000, 0x1234, IntersectionType::FrontFace),
        ),
        (
            "random both",
            random_rays(triangles, 40_000, 0x9876, IntersectionType::Both),
        ),
    ] {
        let label = format!("{what}/{name}");
        let comparison = compare(triangles, &rays, &label);
        assert!(
            comparison.same_triangle + comparison.tied_triangle > comparison.rays / 100,
            "{label}: rays mostly missed the scene: {comparison:?}"
        );
        println!("{label}: {comparison:?}");
    }
}

#[test]
fn a_bumpy_sphere_traces_the_same() {
    let mut triangles = Vec::new();
    bumpy_sphere(v(0.0, 0.0, 0.0), 1.0, 70, 140, &mut triangles);
    compare_all(&triangles, "sphere");
}

#[test]
fn a_flat_coplanar_grid_traces_the_same() {
    let mut triangles = Vec::new();
    flat_grid(180, 0.37, v(-30.0, -30.0, 0.0), &mut triangles);
    compare_all(&triangles, "flat grid");
}

#[test]
fn an_integer_grid_traces_the_same_straight_down_on_its_edges() {
    let mut triangles = Vec::new();
    flat_grid(120, 1.0, v(0.0, 0.0, 0.0), &mut triangles);
    let rays = straight_down_rays(120);
    // These rays lie exactly on grid lines, where the reference's slab test multiplies a zero by
    // an infinite reciprocal and drops the box, so it loses some of them; the flat traversal must
    // not lose any, down a shared edge or anywhere else.
    let comparison = compare_rays(&triangles, &rays, "integer grid edges");
    assert_eq!(comparison.both_miss, 0, "{comparison:?}");
    assert_eq!(comparison.reference_only, 0, "{comparison:?}");
    assert_eq!(
        comparison.same_triangle + comparison.tied_triangle + comparison.flat_only,
        comparison.rays,
        "the flat traversal lost a ray on the plate: {comparison:?}"
    );
}

#[test]
fn tied_centroids_trace_the_same() {
    // Every ray that hits tests every coincident triangle, so a few thousand already shows it.
    compare_all(&tied_centroids(2_500), "tied centroids");
}

#[test]
fn a_long_thin_part_traces_the_same() {
    let triangles = long_thin_tube(2_000, 12);
    // The whole-box views of `compare_all` would nearly all miss a tube this thin, so the rays are
    // aimed at it: a close camera along its length, and random rays at its triangles.
    let mut rays = Vec::new();
    for column in 0..400 {
        for row in 0..100 {
            let x = 900.0 + column as f64 * 0.5 + 0.21;
            let y = -0.1 + (row as f64 + 0.37) * 0.002;
            let origin = v(1000.0, 0.03, 2.0);
            let direction = (v(x, y, 0.02) - origin).get_normal();
            rays.push(Ray::new(origin, direction));
        }
    }
    rays.extend(rays_at_triangles(
        &triangles,
        60_000,
        0x4242,
        IntersectionType::Both,
    ));
    let comparison = compare(&triangles, &rays, "long thin part");
    assert!(
        comparison.same_triangle + comparison.tied_triangle > rays.len() / 10,
        "{comparison:?}"
    );
    println!("long thin part: {comparison:?}");
}

#[test]
fn a_scene_of_100k_triangles_traces_the_same() {
    let triangles = scene_100k();
    assert!(triangles.len() >= 100_000);
    compare_all(&triangles, "100k scene");
}

#[test]
fn a_rays_window_and_faces_are_honored_on_a_known_triangle() {
    let triangles = vec![[v(0.0, 0.0, 0.0), v(4.0, 0.0, 0.0), v(0.0, 4.0, 0.0)]];
    let tree = FlatTree::build(&triangles, 1 << 20).expect("built");
    let down = |mask: u32, t_min: f32, t_max: f32| FlatRay {
        origin: [1.0, 1.0, 5.0],
        direction: [0.0, 0.0, -1.0],
        t_min,
        t_max,
        face_mask: mask,
    };
    let up = |mask: u32| FlatRay {
        origin: [1.0, 1.0, -5.0],
        direction: [0.0, 0.0, 1.0],
        t_min: 0.0,
        t_max: f32::MAX,
        face_mask: mask,
    };

    let hit = tree
        .trace_closest(&down(FACE_FRONT, 0.0, f32::MAX))
        .expect("the front of the triangle faces up");
    assert_eq!(hit.t, 5.0);
    assert_eq!(hit.source_index, 0);
    assert!(hit.front_face);
    // The hit's weights rebuild the hit point.
    let point = [
        hit.barycentric[0] * 0.0 + hit.barycentric[1] * 4.0 + hit.barycentric[2] * 0.0,
        hit.barycentric[0] * 0.0 + hit.barycentric[1] * 0.0 + hit.barycentric[2] * 4.0,
    ];
    assert_eq!(point, [1.0, 1.0]);

    assert!(tree
        .trace_closest(&down(FACE_BACK, 0.0, f32::MAX))
        .is_none());
    assert!(tree.trace_closest(&up(FACE_FRONT)).is_none());
    let back = tree.trace_closest(&up(FACE_BOTH)).expect("both faces");
    assert!(!back.front_face);
    assert!(tree.trace_any(&up(FACE_BACK)));

    // The window: an end before the hit is no hit, and the ends are inclusive.
    assert!(tree.trace_closest(&down(FACE_BOTH, 0.0, 4.99)).is_none());
    assert!(tree.trace_closest(&down(FACE_BOTH, 5.01, 9.0)).is_none());
    assert!(tree.trace_closest(&down(FACE_BOTH, 5.0, 5.0)).is_some());
}

/// Every node's and triangle's box contains what is under it, against the exact `f64` corners,
/// and each source triangle appears exactly once. Walked with an explicit stack.
fn assert_conservative(triangles: &[[Vector3; 3]]) {
    let tree = FlatTree::build(triangles, WEBGPU_DEFAULT_MAX_STORAGE_BINDING).expect("built");
    let flat = tree.triangles();
    let sizes = tree.sizes();
    assert_eq!(sizes.triangle_count as usize, triangles.len());
    assert!(sizes.depth <= MAX_TREE_DEPTH, "{sizes:?}");
    assert!(3 * (sizes.depth as usize) < STACK_CAPACITY);

    let mut seen = vec![false; triangles.len()];
    let inside =
        |min: f32, max: f32, value: f64| f64::from(min) <= value && value <= f64::from(max);
    let check = |node: &FlatNode, slot: usize, record: &FlatTriangle, corners: &[Vector3; 3]| {
        let boxes = [
            (node.min_x[slot], node.max_x[slot]),
            (node.min_y[slot], node.max_y[slot]),
            (node.min_z[slot], node.max_z[slot]),
        ];
        for axis in 0..3 {
            for corner in 0..3 {
                let exact = corners[corner][axis];
                let stored = [record.v0, record.v1, record.v2][corner][axis];
                assert!(
                    inside(boxes[axis].0, boxes[axis].1, exact),
                    "the source corner {exact} is outside its box on axis {axis}"
                );
                assert!(
                    inside(boxes[axis].0, boxes[axis].1, f64::from(stored)),
                    "the stored corner {stored} is outside its box on axis {axis}"
                );
            }
        }
    };

    // (node index, the ancestor (node, slot) boxes above it)
    let mut stack: Vec<(usize, Vec<(usize, usize)>)> = vec![(0, Vec::new())];
    while let Some((index, ancestors)) = stack.pop() {
        let node = &tree.nodes()[index];
        for slot in 0..4 {
            let code = node.child[slot];
            if code == child_code::EMPTY {
                continue;
            }
            let mut path = ancestors.clone();
            path.push((index, slot));
            if child_code::is_leaf(code) {
                let first = child_code::first_triangle(code) as usize;
                let count = child_code::count(code) as usize;
                assert!((1..=MAX_LEAF_TRIANGLES as usize).contains(&count));
                for record in &flat[first..first + count] {
                    let source = record.source_index as usize;
                    assert!(!seen[source], "triangle {source} appears twice");
                    seen[source] = true;
                    for &(ancestor, ancestor_slot) in &path {
                        check(
                            &tree.nodes()[ancestor],
                            ancestor_slot,
                            record,
                            &triangles[source],
                        );
                    }
                }
            } else {
                stack.push((code as usize, path));
            }
        }
    }
    assert!(seen.iter().all(|&s| s), "a triangle was lost");
}

#[test]
fn bounds_contain_every_descendant_triangle() {
    let mut sphere = Vec::new();
    bumpy_sphere(v(1.0 / 3.0, -0.1, 7.7), 1.0 / 7.0, 40, 80, &mut sphere);
    assert_conservative(&sphere);

    let mut grid = Vec::new();
    // Offsets with no exact f32 form, so the narrowing always rounds one way or the other.
    flat_grid(100, 0.1, v(0.1, 0.2, 1.0 / 3.0), &mut grid);
    assert_conservative(&grid);
    assert_conservative(&tied_centroids(3_000));
    assert_conservative(&long_thin_tube(300, 8));
}

#[test]
fn the_bound_narrowing_rounds_outward_with_a_step_to_spare() {
    for value in [0.1, 1.0 / 3.0, -0.1, 7.7e12, 1.0, 0.0, -2.5] {
        let (low, high) = (bounds_min_f32(value), bounds_max_f32(value));
        assert!(f64::from(low) <= value, "{value}");
        assert!(value <= f64::from(high), "{value}");
        assert!(f64::from(low) < value && value < f64::from(high), "{value}");
        // Rounded down then one step: at most two f32 steps below an f32-representable value.
        assert!(low >= (value as f32).next_down().next_down(), "{value}");
        assert!(high <= (value as f32).next_up().next_up(), "{value}");
    }
}

#[test]
fn the_records_have_the_byte_layout_a_wgsl_struct_reads() {
    assert_eq!(NODE_STRIDE, 112);
    assert_eq!(TRIANGLE_STRIDE, 40);
    assert_eq!(std::mem::size_of::<FlatNode>(), 112);
    assert_eq!(std::mem::size_of::<FlatTriangle>(), 40);

    let node = FlatNode {
        min_x: [1.0, 2.0, 3.0, 4.0],
        max_x: [5.0, 6.0, 7.0, 8.0],
        min_y: [9.0, 10.0, 11.0, 12.0],
        max_y: [13.0, 14.0, 15.0, 16.0],
        min_z: [17.0, 18.0, 19.0, 20.0],
        max_z: [21.0, 22.0, 23.0, 24.0],
        child: [100, 200, 300, 400],
    };
    let bytes = node.to_le_bytes();
    let f32_at = |offset: usize| {
        f32::from_le_bytes(bytes[offset..offset + 4].try_into().expect("four bytes"))
    };
    let u32_at = |offset: usize| {
        u32::from_le_bytes(bytes[offset..offset + 4].try_into().expect("four bytes"))
    };
    // min_x at 0, max_x at 16, min_y at 32, max_y at 48, min_z at 64, max_z at 80, child at 96.
    assert_eq!(f32_at(0), 1.0);
    assert_eq!(f32_at(12), 4.0);
    assert_eq!(f32_at(16), 5.0);
    assert_eq!(f32_at(32), 9.0);
    assert_eq!(f32_at(48), 13.0);
    assert_eq!(f32_at(64), 17.0);
    assert_eq!(f32_at(80), 21.0);
    assert_eq!(f32_at(92), 24.0);
    assert_eq!(u32_at(96), 100);
    assert_eq!(u32_at(108), 400);
    assert_eq!(FlatNode::from_le_bytes(&bytes), node);

    let triangle = FlatTriangle {
        v0: [1.0, 2.0, 3.0],
        v1: [4.0, 5.0, 6.0],
        v2: [7.0, 8.0, 9.0],
        source_index: 12345,
    };
    let bytes = triangle.to_le_bytes();
    let f32_at = |offset: usize| {
        f32::from_le_bytes(bytes[offset..offset + 4].try_into().expect("four bytes"))
    };
    let u32_at = |offset: usize| {
        u32::from_le_bytes(bytes[offset..offset + 4].try_into().expect("four bytes"))
    };
    // v0 at 0, v1 at 12, v2 at 24, source_index at 36.
    assert_eq!(f32_at(0), 1.0);
    assert_eq!(f32_at(8), 3.0);
    assert_eq!(f32_at(12), 4.0);
    assert_eq!(f32_at(24), 7.0);
    assert_eq!(u32_at(36), 12345);
    assert_eq!(FlatTriangle::from_le_bytes(&bytes), triangle);
}

#[test]
fn a_leaf_code_never_collides_with_an_empty_slot() {
    assert_eq!(child_code::leaf(0, 1), 0x8000_0000);
    let top = child_code::leaf((1 << 29) - 2, 4);
    assert!(child_code::is_leaf(top));
    assert_ne!(top, child_code::EMPTY);
    assert_eq!(child_code::first_triangle(top), (1 << 29) - 2);
    assert_eq!(child_code::count(top), 4);
    assert!(!child_code::is_leaf(child_code::EMPTY));
    assert!(!child_code::is_leaf(7));
}

#[test]
fn the_serialized_buffers_decode_to_the_same_records() {
    let mut triangles = Vec::new();
    bumpy_sphere(v(0.0, 0.0, 0.0), 1.0, 30, 60, &mut triangles);
    let tree = FlatTree::build(&triangles, WEBGPU_DEFAULT_MAX_STORAGE_BINDING).expect("built");
    let node_bytes = tree.node_bytes();
    let triangle_bytes = tree.triangle_bytes();
    let sizes = tree.sizes();
    assert_eq!(node_bytes.len() as u64, sizes.node_bytes);
    assert_eq!(triangle_bytes.len() as u64, sizes.triangle_bytes);
    assert_eq!(sizes.node_bytes, sizes.node_count * 112);
    assert_eq!(sizes.triangle_bytes, sizes.triangle_count * 40);

    let nodes: Vec<FlatNode> = node_bytes
        .as_chunks::<NODE_STRIDE>()
        .0
        .iter()
        .map(FlatNode::from_le_bytes)
        .collect();
    let records: Vec<FlatTriangle> = triangle_bytes
        .as_chunks::<TRIANGLE_STRIDE>()
        .0
        .iter()
        .map(FlatTriangle::from_le_bytes)
        .collect();
    assert_eq!(nodes, tree.nodes());
    assert_eq!(records, tree.triangles());
}

#[test]
fn building_twice_gives_identical_bytes() {
    let mut grid = Vec::new();
    flat_grid(100, 0.5, v(-10.0, -10.0, 2.0), &mut grid);
    for triangles in [&grid, &tied_centroids(4_000), &scene_100k()] {
        let first = FlatTree::build(triangles, WEBGPU_DEFAULT_MAX_STORAGE_BINDING).expect("built");
        let second = FlatTree::build(triangles, WEBGPU_DEFAULT_MAX_STORAGE_BINDING).expect("built");
        assert_eq!(first.node_bytes(), second.node_bytes());
        assert_eq!(first.triangle_bytes(), second.triangle_bytes());
        assert_eq!(first.sizes(), second.sizes());
    }
}

#[test]
fn an_oversized_triangle_buffer_is_refused_with_the_count_that_fits() {
    let triangles = tied_centroids(1_000);
    let error = FlatTree::build(&triangles, 4_096).expect_err("too big");
    let FlatTreeError::BufferTooLarge {
        buffer,
        needed_bytes,
        limit_bytes,
        triangle_count,
        triangles_that_fit,
    } = error.clone()
    else {
        panic!("wrong error: {error}");
    };
    assert_eq!(buffer, FlatTreeBuffer::Triangles);
    assert_eq!(needed_bytes, 40_000);
    assert_eq!(limit_bytes, 4_096);
    assert_eq!(triangle_count, 1_000);
    assert_eq!(triangles_that_fit, 102);
    let text = error.to_string();
    assert!(text.contains("1000") && text.contains("102"), "{text}");

    // The number is honest: that many triangles do fit.
    FlatTree::build(&triangles[..102], 4_096).expect("the reported count fits");
    assert!(matches!(
        FlatTree::build(&triangles[..103], 4_096),
        Err(FlatTreeError::BufferTooLarge { .. })
    ));
}

#[test]
fn an_oversized_node_buffer_is_refused() {
    // Two triangles fit 100 bytes of triangles, but their single 112-byte root node does not.
    let triangles = tied_centroids(2);
    let error = FlatTree::build(&triangles, 100).expect_err("the root node is too big");
    assert!(
        matches!(
            error,
            FlatTreeError::BufferTooLarge {
                buffer: FlatTreeBuffer::Nodes,
                needed_bytes: 112,
                limit_bytes: 100,
                triangle_count: 2,
                triangles_that_fit: 1,
            }
        ),
        "{error:?}"
    );
}

#[test]
fn nothing_to_trace_and_bad_coordinates_are_refused_with_what_to_do() {
    assert_eq!(
        FlatTree::build(&[], WEBGPU_DEFAULT_MAX_STORAGE_BINDING).expect_err("no triangles"),
        FlatTreeError::NoTriangles
    );

    let nan = [
        [v(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), v(0.0, 1.0, 0.0)],
        [v(f64::NAN, 0.0, 0.0), v(1.0, 0.0, 0.0), v(0.0, 1.0, 0.0)],
    ];
    let error = FlatTree::build(&nan, 1 << 20).expect_err("a NaN corner");
    assert_eq!(
        error,
        FlatTreeError::InvalidCoordinate { triangle_index: 1 }
    );
    assert!(error.to_string().contains("triangle 1"), "{error}");

    let huge = [[v(1e19, 0.0, 0.0), v(1.0, 0.0, 0.0), v(0.0, 1.0, 0.0)]];
    assert!(matches!(
        FlatTree::build(&huge, 1 << 20),
        Err(FlatTreeError::InvalidCoordinate { triangle_index: 0 })
    ));
}

#[test]
fn a_tiny_scene_is_one_root_with_a_leaf_child() {
    for count in 1..=4u32 {
        let triangles = tied_centroids(count as usize);
        let tree = FlatTree::build(&triangles, 1 << 20).expect("built");
        let sizes = tree.sizes();
        assert_eq!(sizes.node_count, 1, "{sizes:?}");
        assert_eq!(sizes.depth, 1);
        let code = tree.nodes()[0].child[0];
        assert!(child_code::is_leaf(code));
        assert_eq!(child_code::count(code), count);
        assert_eq!(tree.nodes()[0].child[1], child_code::EMPTY);
    }
}
