//! Rust-only tests for mesh editing and cleanup that other tests exercise only indirectly:
//! `merge_vertices`, `split`, `split_on_planes`, `repair_t_junctions`, `clean_and_merge`,
//! `remove_unused_vertices`, `remove_degenerate_faces` and the vertex sorters.

use polygon_mesh::solids::platonic_solids;
use polygon_mesh::vertex_sorters::{
    VertexDistanceFromPointSorter, VertexSorter, VertexXAxisSorter, VertexXYZAxisWithRotation,
};
use polygon_mesh::{Mesh, UnionFind};
use vector_math::{Plane, Vector3, Vector3Float};

#[test]
fn rust_only_union_find_joins_sets_larger_root_wins() {
    let mut uf = UnionFind::new(5);
    uf.union(0, 1);
    uf.union(2, 1); // {0,1} is larger, so 0 stays the root
    assert_eq!(uf.find(2), 0);
    uf.union(3, 4); // equal sizes: the first argument's root wins
    assert_eq!(uf.find(4), 3);
    assert_ne!(uf.find(3), uf.find(0));
}

#[test]
fn rust_only_merge_vertices_welds_near_vertices_and_drops_collapsed_faces() {
    let mut mesh = Mesh::new();
    // two triangles sharing an edge, but with the shared corners 0.0005 apart
    mesh.create_face(&[
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(10.0, 0.0, 0.0),
        Vector3::new(0.0, 10.0, 0.0),
    ]);
    mesh.create_face(&[
        Vector3::new(10.0005, 0.0, 0.0),
        Vector3::new(10.0, 10.0, 0.0),
        Vector3::new(0.0, 10.0005, 0.0),
    ]);
    assert_eq!(mesh.vertices.len(), 6);

    let mut reports = 0;
    mesh.merge_vertices(0.001, 0.0, Some(&mut |_, _| reports += 1));
    assert_eq!(mesh.vertices.len(), 4);
    assert_eq!(mesh.faces.len(), 2);
    assert!(reports > 0);
    // the welded corner sits at the average of the two it replaced
    assert!(mesh.vertices.iter().any(
        |v| *v == Vector3Float::new_f32(((10.0f64 + 10.0005f32 as f64) / 2.0) as f32, 0.0, 0.0)
    ));

    // a face whose corners all weld together goes away
    let mut tiny = Mesh::new();
    tiny.create_face(&[
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(0.0001, 0.0, 0.0),
        Vector3::new(0.0, 0.0001, 0.0),
    ]);
    tiny.merge_vertices(0.001, 0.0, None);
    assert_eq!(tiny.faces.len(), 0);
    assert_eq!(tiny.vertices.len(), 1);
}

#[test]
fn rust_only_merge_vertices_leaves_a_welded_cube_alone() {
    let mut cube = platonic_solids::create_cube(10.0, 10.0, 10.0);
    let before = cube.changed_count();
    cube.merge_vertices(0.001, 0.0, None);
    assert_eq!(cube.vertices.len(), 8);
    assert_eq!(cube.faces.len(), 12);
    assert_eq!(cube.changed_count(), before);
}

#[test]
fn rust_only_split_cuts_a_cube_into_a_closed_mesh() {
    let mut cube = platonic_solids::create_cube(10.0, 10.0, 10.0);
    let volume = cube.get_volume();
    cube.split(Plane::new(Vector3::UNIT_Z, 1.0), 0.001, None, true, false);
    assert!(cube.is_manifold());
    assert!((cube.get_volume() - volume).abs() < 1e-3);
    assert!(cube.vertices.iter().filter(|v| v.z == 1.0).count() >= 4);

    // discarding the negative side keeps only the faces above the plane
    let mut top = platonic_solids::create_cube(10.0, 10.0, 10.0);
    top.split(Plane::new(Vector3::UNIT_Z, 1.0), 0.001, None, true, true);
    assert!(top.faces.iter().all(|f| {
        [f.v0, f.v1, f.v2]
            .iter()
            .all(|&v| top.vertices[v as usize].z >= 1.0 - 1e-4)
    }));
}

#[test]
fn rust_only_split_on_planes_slices_without_opening_seams() {
    let mut cube = platonic_solids::create_cube(10.0, 10.0, 10.0);
    let volume = cube.get_volume();
    cube.split_on_planes(Vector3::new(0.0, 0.0, 2.0), &[2.0, -1.0, 2.0, 3.0], 0.01);
    assert!(cube.is_manifold());
    assert!((cube.get_volume() - volume).abs() < 1e-3);
    // no face crosses any plane
    for face in cube.faces.iter() {
        let zs = [face.v0, face.v1, face.v2].map(|v| f64::from(cube.vertices[v as usize].z));
        for plane in [-1.0, 2.0, 3.0] {
            let below = zs.iter().any(|&z| z < plane - 1e-4);
            let above = zs.iter().any(|&z| z > plane + 1e-4);
            assert!(!(below && above), "face crosses plane {plane}");
        }
    }
}

#[test]
fn rust_only_repair_t_junctions_splits_the_long_edge() {
    // a big triangle whose top edge has the corner of two small triangles in its middle
    let mut mesh = Mesh::new();
    mesh.vertices = vec![
        Vector3Float::new(0.0, 0.0, 0.0),
        Vector3Float::new(10.0, 0.0, 0.0),
        Vector3Float::new(5.0, -5.0, 0.0),
        Vector3Float::new(5.0, 0.0, 0.0),
        Vector3Float::new(5.0, 5.0, 0.0),
    ];
    let vertices = mesh.vertices.clone();
    mesh.faces.add(0, 2, 1, &vertices);
    mesh.faces.add(0, 3, 4, &vertices);
    mesh.faces.add(3, 1, 4, &vertices);
    // 8 edges, only 3-4 shared
    assert_eq!(mesh.get_non_manifold_edges().len(), 7);

    let mut last_progress = 0.0;
    mesh.repair_t_junctions(Some(&mut |p, _| last_progress = p), false, 0.1);
    assert_eq!(last_progress, 1.0);
    // the big triangle is now two, both using the middle vertex
    assert_eq!(mesh.faces.len(), 4);
    assert_eq!(mesh.vertices.len(), 5);
    // every interior edge now has two faces: only the 4 outline edges are open
    assert_eq!(mesh.get_non_manifold_edges().len(), 4);
    // winding is kept: every normal still points the same way as the original faces
    assert!(
        mesh.faces.iter().all(|f| f.normal.z < 0.0) || mesh.faces.iter().all(|f| f.normal.z > 0.0)
    );
}

#[test]
fn rust_only_clean_and_merge_welds_soup_and_skips_manifold() {
    let mut soup = Mesh::new();
    let cube = platonic_solids::create_cube(10.0, 10.0, 10.0);
    soup.copy_all_faces(&cube, vector_math::Matrix4X4::IDENTITY);
    assert_eq!(soup.vertices.len(), 36);
    soup.clean_and_merge();
    assert_eq!(soup.vertices.len(), 8);
    assert_eq!(soup.faces.len(), 12);
    assert!(soup.is_manifold());

    let before = soup.changed_count();
    soup.clean_and_merge();
    assert_eq!(soup.changed_count(), before);

    // with no faces, the orphaned vertices are dropped
    let mut empty = Mesh::new();
    empty.vertices.push(Vector3Float::new(1.0, 2.0, 3.0));
    empty.clean_and_merge();
    assert_eq!(empty.vertices.len(), 0);
}

#[test]
fn rust_only_remove_unused_vertices_renumbers_faces() {
    let mut mesh = Mesh::new();
    mesh.vertices = vec![
        Vector3Float::new(9.0, 9.0, 9.0),
        Vector3Float::new(0.0, 0.0, 0.0),
        Vector3Float::new(1.0, 0.0, 0.0),
        Vector3Float::new(0.0, 1.0, 0.0),
    ];
    let vertices = mesh.vertices.clone();
    mesh.faces.add(1, 2, 3, &vertices);
    mesh.remove_unused_vertices();
    assert_eq!(mesh.vertices.len(), 3);
    let f = mesh.faces[0];
    assert_eq!((f.v0, f.v1, f.v2), (0, 1, 2));
    assert_eq!(mesh.faces[0].normal, Vector3Float::new(0.0, 0.0, 1.0));
}

#[test]
fn rust_only_remove_degenerate_faces_drops_repeats_and_slivers() {
    let mut mesh = Mesh::new();
    mesh.vertices = vec![
        Vector3Float::new(0.0, 0.0, 0.0),
        Vector3Float::new(1.0, 0.0, 0.0),
        Vector3Float::new(0.0, 1.0, 0.0),
        Vector3Float::new(2.0, 0.0, 0.0),
    ];
    let vertices = mesh.vertices.clone();
    mesh.faces.add(0, 1, 2, &vertices); // good
    mesh.faces.add(0, 0, 2, &vertices); // repeated index
    mesh.faces.add(0, 1, 3, &vertices); // collinear, zero area
    mesh.remove_degenerate_faces(1e-6);
    assert_eq!(mesh.faces.len(), 1);
    // an area threshold above the good face drops it too
    mesh.remove_degenerate_faces(1.0);
    assert_eq!(mesh.faces.len(), 0);
}

#[test]
fn rust_only_vertex_sorters_order_and_find() {
    let a = Vector3::new(1.0, 5.0, 0.0);
    let b = Vector3::new(2.0, 0.0, 0.0);
    assert!(VertexXAxisSorter.compare(a, b) < 0);
    assert!(VertexXAxisSorter.compare(b, a) > 0);
    assert_eq!(VertexXAxisSorter.compare(a, a), 0);

    let mut vertices = vec![
        Vector3::new(3.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(1.0005, 0.0, 0.0),
        Vector3::new(2.0, 0.0, 0.0),
    ];
    vertices.sort_by(|p, q| VertexXAxisSorter.compare(*p, *q).cmp(&0));
    let found = VertexXAxisSorter.find_vertices(&vertices, Vector3::new(1.0, 0.0, 0.0), 0.001);
    assert_eq!(found.len(), 2);

    let rotated = VertexXYZAxisWithRotation::default();
    assert_eq!(rotated.compare(a, a), 0);
    assert_eq!(rotated.compare(a, b), -rotated.compare(b, a));
    let mut rotated_vertices = vertices.clone();
    rotated_vertices.sort_by(|p, q| rotated.compare(*p, *q).cmp(&0));
    let found = rotated.find_vertices(&rotated_vertices, Vector3::new(2.0, 0.0, 0.0), 0.001);
    assert_eq!(found, vec![Vector3::new(2.0, 0.0, 0.0)]);

    let near = Vector3::new(0.224374, 0.805696, 0.383724);
    assert!(VertexDistanceFromPointSorter.compare(near, Vector3::new(10.0, 0.0, 0.0)) < 0);
}
