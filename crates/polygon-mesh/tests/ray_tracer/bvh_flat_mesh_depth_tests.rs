//! The tree over a flat mesh. A part lying in a plane has an axis of no extent, and the tree must
//! come out as shallow as any other: a deep tree overflows the stack of whatever builds, traces
//! or drops it. These run on the ordinary test-thread stack, on the flat shapes CAD parts have (a
//! welded grid, an integer grid, a finely tessellated plate), so a build that recursed once per
//! face aborts this binary, and a depth that merely grew fails the bound here.

use std::sync::Arc;

use solidmesh_math::{Ray, Vector3, Vector3Float};
use solidmesh_mesh::ray_tracer::{
    BoundingVolumeHierarchy, BvhCreationOptions, MinimalTriangle, Traceable, TraceableRef,
    VertexFunc,
};
use solidmesh_mesh::Mesh;

/// A welded `n` by `n` grid of vertices `step` apart in the plane that `place` maps (u, v) into.
fn welded_grid(n: i32, step: f64, place: fn(f64, f64) -> Vector3) -> Mesh {
    let mut mesh = Mesh::new();
    for j in 0..n {
        for i in 0..n {
            let p = place(f64::from(i) * step, f64::from(j) * step);
            mesh.vertices.push(Vector3Float::new(p.x, p.y, p.z));
        }
    }
    for j in 0..n - 1 {
        for i in 0..n - 1 {
            let a = j * n + i;
            mesh.faces.add(a, a + 1, a + n, &mesh.vertices);
            mesh.faces.add(a + 1, a + n + 1, a + n, &mesh.vertices);
        }
    }
    mesh
}

fn in_xy(u: f64, v: f64) -> Vector3 {
    Vector3::new(u, v, 0.0)
}

fn in_yz(u: f64, v: f64) -> Vector3 {
    Vector3::new(4.25, u, v)
}

fn in_xz(u: f64, v: f64) -> Vector3 {
    Vector3::new(u, -3.5, v)
}

fn tree_of(mesh: &Mesh) -> TraceableRef {
    let corners: Arc<Vec<[Vector3Float; 3]>> = Arc::new(
        mesh.faces
            .iter()
            .map(|face| {
                [
                    mesh.vertices[face.v0 as usize],
                    mesh.vertices[face.v1 as usize],
                    mesh.vertices[face.v2 as usize],
                ]
            })
            .collect(),
    );
    let vertex_func: VertexFunc =
        Arc::new(move |face, corner| corners[face as usize][corner.min(2) as usize]);
    let leaves = (0..mesh.faces.len() as i32)
        .map(|face| Arc::new(MinimalTriangle::new(vertex_func.clone(), face)) as TraceableRef)
        .collect();
    BoundingVolumeHierarchy::create_new_hierarchy(leaves, BvhCreationOptions::default())
        .expect("the mesh has faces")
}

/// The tree's depth and leaf count, walked without recursing, so a bad tree fails a test.
fn depth_and_leaves(root: &dyn Traceable) -> (usize, usize) {
    let (mut deepest, mut leaves) = (0, 0);
    let mut stack: Vec<(&dyn Traceable, usize)> = vec![(root, 1)];
    while let Some((node, depth)) = stack.pop() {
        deepest = deepest.max(depth);
        match node.children() {
            Some(children) if !children.is_empty() => {
                stack.extend(children.into_iter().map(|child| (child, depth + 1)));
            }
            _ => leaves += 1,
        }
    }
    (deepest, leaves)
}

/// A well-shaped tree: the 63 bits of a code are at most 63 levels of splits, and halving tied
/// runs adds one level per doubling of the faces.
fn assert_shallow(mesh: &Mesh, what: &str) {
    let tree = tree_of(mesh);
    let (depth, leaves) = depth_and_leaves(tree.as_ref());
    assert_eq!(leaves, mesh.faces.len(), "{what}: every face is a leaf");
    let bound = 64 + 2 * (usize::BITS - leaves.leading_zeros()) as usize;
    assert!(depth <= bound, "{what}: depth {depth} over {leaves} faces");
}

#[test]
fn a_welded_flat_integer_grid_gets_a_shallow_tree_at_every_size() {
    for n in [60, 80, 120, 200] {
        assert_shallow(&welded_grid(n, 1.0, in_xy), &format!("integer grid n={n}"));
    }
}

#[test]
fn a_finely_tessellated_flat_plate_gets_a_shallow_tree() {
    // One big plate of tiny facets, at a step with no round binary form.
    assert_shallow(&welded_grid(150, 0.37, in_xy), "fine plate");
}

#[test]
fn a_flat_mesh_on_any_axis_gets_a_shallow_tree() {
    assert_shallow(&welded_grid(120, 1.0, in_yz), "flat in x");
    assert_shallow(&welded_grid(120, 1.0, in_xz), "flat in y");
}

#[test]
fn faces_with_one_center_get_a_shallow_tree() {
    // Every center identical: no code can separate them, and the tree still may not be a chain.
    let mut mesh = Mesh::new();
    for _ in 0..20_000 {
        mesh.create_face(&[
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(9.0, 0.0, 0.0),
            Vector3::new(0.0, 9.0, 0.0),
        ]);
    }
    assert_shallow(&mesh, "one center");
}

#[test]
fn a_ray_finds_the_face_a_flat_grid_really_has_there() {
    // Depth is not enough: the tree must still find the face that lies under the ray.
    let mesh = welded_grid(120, 1.0, in_xy);
    let tree = tree_of(&mesh);
    for &(x, y) in &[(0.3, 0.2), (58.6, 33.1), (118.2, 118.7), (77.5, 0.25)] {
        let mut ray = Ray::new(Vector3::new(x, y, 10.0), Vector3::new(0.0, 0.0, -1.0));
        let info = tree
            .get_closest_intersection(&mut ray)
            .expect("the plate is under the ray");
        assert!(
            info.hit_position.z.abs() < 1e-6,
            "hit {:?}",
            info.hit_position
        );
        let leaf = info
            .closest_hit_as::<MinimalTriangle>()
            .expect("a mesh triangle");
        let face = &mesh.faces[leaf.face_index as usize];
        let corners = [
            mesh.vertices[face.v0 as usize],
            mesh.vertices[face.v1 as usize],
            mesh.vertices[face.v2 as usize],
        ];
        let span = |of: fn(&Vector3Float) -> f64| {
            let (low, high) = corners
                .iter()
                .map(of)
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(low, high), value| {
                    (low.min(value), high.max(value))
                });
            low..=high
        };
        assert!(
            span(|v| f64::from(v.x)).contains(&x) && span(|v| f64::from(v.y)).contains(&y),
            "the reported face does not lie under ({x}, {y})"
        );
    }
}

#[test]
fn a_flat_grids_tree_splits_by_position_however_its_faces_are_ordered() {
    // The codes must really separate a flat mesh's faces: a tree cut only by count would split a
    // shuffled grid into two halves that each cover the whole plate, and every ray would visit
    // nearly every leaf. Spatial halves barely overlap.
    let mut grid = welded_grid(150, 1.0, in_xy);
    let mut faces: Vec<(i32, i32, i32)> = grid.faces.iter().map(|f| (f.v0, f.v1, f.v2)).collect();
    let mut state: u64 = 0x9e37_79b9_7f4a_7c15;
    for at in (1..faces.len()).rev() {
        state = state.wrapping_mul(0x5851_f42d_4c95_7f2d).wrapping_add(1);
        faces.swap(at, (state >> 33) as usize % (at + 1));
    }
    let vertices = grid.vertices.clone();
    grid.faces.clear();
    for (a, b, c) in faces {
        grid.faces.add(a, b, c, &vertices);
    }

    let tree = tree_of(&grid);
    let root = tree
        .as_any()
        .downcast_ref::<BoundingVolumeHierarchy>()
        .expect("a tree over many faces");
    let (left, right) = (
        root.left().get_axis_aligned_bounding_box(),
        root.right().get_axis_aligned_bounding_box(),
    );
    let overlap = |axis: usize| {
        (left.max_xyz[axis].min(right.max_xyz[axis]) - left.min_xyz[axis].max(right.min_xyz[axis]))
            .max(0.0)
    };
    let plate = 149.0 * 149.0;
    let shared = overlap(0) * overlap(1);
    assert!(
        shared < plate * 0.1,
        "the root halves share {shared} of a plate of {plate}"
    );
}

/// A hash of the tree's shape and leaves, so a change to any tree shows.
fn fingerprint(root: &dyn Traceable) -> u64 {
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    let mut stack: Vec<&dyn Traceable> = vec![root];
    while let Some(node) = stack.pop() {
        let token = match node.as_any().downcast_ref::<MinimalTriangle>() {
            Some(leaf) => leaf.face_index as u64,
            None => u64::MAX,
        };
        hash = (hash ^ token).wrapping_mul(0x100_0000_01b3);
        if let Some(children) = node.children() {
            stack.extend(children);
        }
    }
    hash
}

/// A rolling surface of faces, so no axis lacks extent: the ordinary case.
fn bumpy_grid(n: i32) -> Mesh {
    welded_grid(n, 0.75, |u, v| {
        Vector3::new(u, v, (u * 0.7).sin() + (v * 0.3).cos() * 2.5)
    })
}

#[test]
fn a_mesh_with_extent_on_every_axis_keeps_its_exact_tree() {
    // Pinned from the builder as it was before flat meshes were fixed: picking results and
    // saved scenes depend on the tree, so nothing but a degenerate input may change it.
    assert_eq!(
        fingerprint(tree_of(&bumpy_grid(40)).as_ref()),
        PINNED_OPTIMIZED
    );
    assert_eq!(fingerprint(tree_of(&bumpy_grid(90)).as_ref()), PINNED_LARGE);
}

const PINNED_OPTIMIZED: u64 = 7658561446158612763;
const PINNED_LARGE: u64 = 15547780621510510827;
