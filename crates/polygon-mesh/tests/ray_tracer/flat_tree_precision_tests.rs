//! The flat traversal at the edges of `f32`: a ray lying exactly on a slab's plane on the box's
//! maximum side at or near zero, and triangles at huge coordinates where a cubic product would
//! overflow; and a degenerate chain in a scene, which must not make the rest of the tree worse.

use solidmesh_mesh::ray_tracer::flat_tree::{
    child_code, FlatRay, FlatTree, FACE_BOTH, MAX_TREE_DEPTH,
};

use crate::flat_tree_support::{bumpy_sphere, tied_centroids, v};

fn ray(origin: [f32; 3], direction: [f32; 3]) -> FlatRay {
    FlatRay {
        origin,
        direction,
        t_min: 0.0,
        t_max: f32::MAX,
        face_mask: FACE_BOTH,
    }
}

#[test]
fn a_ray_on_the_max_plane_of_a_box_on_the_negative_side_hits() {
    // A ray along y exactly on x = 0, the maximum side of the triangle's box.
    let triangle = [v(-1.0, 0.0, -1.0), v(0.0, 0.0, -1.0), v(0.0, 0.0, 1.0)];
    let tree = FlatTree::build(&[triangle], 1 << 20).expect("built");
    let flat = ray([0.0, -5.0, 0.0], [0.0, 1.0, 0.0]);
    let hit = tree.trace_closest(&flat).expect("the ray runs along x = 0");
    assert_eq!(hit.t, 5.0);
    assert!(tree.trace_any(&flat));
}

#[test]
fn a_ray_through_a_box_corner_at_the_origin_with_negative_coordinates_hits() {
    // The ray runs down z with x = y = 0, the box's maximum side on both axes.
    let triangle = [v(-2.0, -2.0, 0.0), v(0.0, -2.0, 0.0), v(0.0, 0.0, 0.0)];
    let tree = FlatTree::build(&[triangle], 1 << 20).expect("built");
    let flat = ray([0.0, 0.0, 5.0], [0.0, 0.0, -1.0]);
    let hit = tree
        .trace_closest(&flat)
        .expect("a ray down the shared corner");
    assert_eq!(hit.t, 5.0);
}

#[test]
fn a_ray_parallel_to_a_slab_outside_it_misses_and_inside_it_hits_at_negative_coordinates() {
    let triangle = [
        v(-3.0, -1.0, -3.0),
        v(-2.0, -1.0, -3.0),
        v(-2.0, -1.0, -1.0),
    ];
    let tree = FlatTree::build(&[triangle], 1 << 20).expect("built");
    // On the box's maximum x plane, inside its z extent.
    let inside = ray([-2.0, -5.0, -2.0], [0.0, 1.0, 0.0]);
    assert_eq!(tree.trace_closest(&inside).expect("a hit").t, 4.0);
    // Parallel to x and outside the box on it: no box test may be finite-garbage true.
    let outside = ray([-2.0, -5.0, 4.0], [0.0, 1.0, 0.0]);
    assert!(tree.trace_closest(&outside).is_none());
}

#[test]
fn a_triangle_at_huge_coordinates_is_still_hit() {
    for size in [1e13, 1e15, 1e17, 1e18] {
        let triangle = [
            v(-size, -size, size),
            v(size, -size, size),
            v(0.0, size, size),
        ];
        let tree = FlatTree::build(&[triangle], 1 << 20).expect("the coordinates are in range");
        let flat = ray([0.0, 0.0, 0.0], [0.0, 0.0, 1.0]);
        let hit = tree
            .trace_closest(&flat)
            .unwrap_or_else(|| panic!("the ray goes straight through the triangle at {size}"));
        assert!(hit.t.is_finite(), "{}", hit.t);
        let error = (f64::from(hit.t) - size).abs() / size;
        assert!(error < 1e-5, "t {} at {size}", hit.t);
        assert!(tree.trace_any(&flat), "any-hit at {size}");
    }
}

/// The summed child-box area of the part of the tree whose boxes are entirely left of `x`.
fn area_left_of(tree: &FlatTree, x: f32) -> f64 {
    let mut total = 0.0;
    for node in tree.nodes() {
        for slot in 0..4 {
            if node.child[slot] == child_code::EMPTY || node.max_x[slot] >= x {
                continue;
            }
            let dx = f64::from(node.max_x[slot] - node.min_x[slot]);
            let dy = f64::from(node.max_y[slot] - node.min_y[slot]);
            let dz = f64::from(node.max_z[slot] - node.min_z[slot]);
            total += 2.0 * (dx * dy + dy * dz + dz * dx);
        }
    }
    total
}

#[test]
fn a_degenerate_chain_does_not_make_the_rest_of_the_tree_worse() {
    let mut sphere = Vec::new();
    bumpy_sphere(v(0.0, 0.0, 0.0), 6.0, 25, 50, &mut sphere);
    let alone = area_left_of(&FlatTree::build(&sphere, 1 << 30).expect("built"), 100.0);

    // A chain of coincident triangles far to the side: the builder cannot separate them, and in a
    // scene of this mix its binary tree comes out hundreds of levels tall.
    let mut with_chain = sphere.clone();
    for corners in tied_centroids(4_000) {
        with_chain.push([
            corners[0] + v(500.0, 0.0, 0.0),
            corners[1] + v(500.0, 0.0, 0.0),
            corners[2] + v(500.0, 0.0, 0.0),
        ]);
    }
    let tree = FlatTree::build(&with_chain, 1 << 30).expect("built");
    let sizes = tree.sizes();
    let with = area_left_of(&tree, 100.0);
    println!("sphere area alone {alone}, with a chain beside it {with}");
    assert!(sizes.depth <= MAX_TREE_DEPTH, "{sizes:?}");
    // With a chain in the scene the builder clusters the sphere a little differently, 2.05 times
    // the area of the sphere alone, and that tree is what must survive: only the chain is split
    // by count. Replacing the whole tree made the sphere's area 2.7 times its own.
    assert!(with < alone * 2.3, "{alone} grew to {with}");
}
