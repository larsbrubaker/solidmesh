//! The edge graph built a budget of faces at a time (`MeshEdgeGraphSteps`) is the graph built in
//! one go (`Mesh::get_mesh_edge_graph`), on closed solids, a mesh with non-manifold and
//! degenerate faces, an empty mesh, Batman.stl and a 100,000-face sheet, at budgets from one face
//! up; and a big mesh's graph takes many steps, so a frame can stop between them.

use polygon_mesh::processors::stl_processing;
use polygon_mesh::solids::platonic_solids;
use polygon_mesh::{CancelToken, Mesh, MeshEdgeGraph, MeshEdgeGraphSteps};
use vector_math::Vector3Float;

/// The graph built in steps of `budget` faces, and how many steps it took.
fn stepped_graph(mesh: &Mesh, budget: usize) -> (MeshEdgeGraph, usize) {
    let mut steps = MeshEdgeGraphSteps::new(mesh);
    let mut count = 0;
    loop {
        count += 1;
        if let Some(graph) = steps.step(budget) {
            return (graph, count);
        }
        assert!(count < 10_000_000, "the stepped graph never finished");
    }
}

fn batman() -> Mesh {
    let path = format!("{}/tests/fixtures/Batman.stl", env!("CARGO_MANIFEST_DIR"));
    stl_processing::load_file(std::path::Path::new(&path), &CancelToken::new(), None)
        .expect("Batman.stl loads")
}

/// A flat `columns x rows` grid of squares, two triangles each.
fn sheet(columns: i32, rows: i32) -> Mesh {
    let mut mesh = Mesh::new();
    for y in 0..=rows {
        for x in 0..=columns {
            mesh.vertices
                .push(Vector3Float::new(f64::from(x), f64::from(y), 0.0));
        }
    }
    let at = |x: i32, y: i32| y * (columns + 1) + x;
    for y in 0..rows {
        for x in 0..columns {
            mesh.faces
                .add(at(x, y), at(x + 1, y), at(x + 1, y + 1), &mesh.vertices);
            mesh.faces
                .add(at(x, y), at(x + 1, y + 1), at(x, y + 1), &mesh.vertices);
        }
    }
    mesh
}

/// Three faces on one edge, a boundary, and two degenerate faces (a repeated corner), whose
/// repeated slots compare equal.
fn non_manifold_and_degenerate() -> Mesh {
    let mut mesh = Mesh::new();
    for (x, y, z) in [
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, -1.0, 0.0),
        (0.0, 0.0, 1.0),
    ] {
        mesh.vertices.push(Vector3Float::new(x, y, z));
    }
    mesh.faces.add(0, 1, 2, &mesh.vertices);
    mesh.faces.add(0, 3, 1, &mesh.vertices);
    mesh.faces.add(0, 1, 4, &mesh.vertices);
    mesh.faces.add(2, 2, 4, &mesh.vertices);
    mesh.faces.add(1, 4, 1, &mesh.vertices);
    mesh
}

#[test]
fn the_edge_graph_built_in_steps_is_the_graph_built_in_one_go() {
    let meshes = [
        ("tetrahedron", platonic_solids::create_tetrahedron(1.0)),
        ("cube", platonic_solids::create_cube(10.0, 10.0, 10.0)),
        ("icosahedron", platonic_solids::create_icosahedron(1.0)),
        ("non-manifold", non_manifold_and_degenerate()),
        ("empty", Mesh::new()),
        ("Batman", batman()),
        ("sheet", sheet(250, 200)),
    ];
    for (name, mesh) in &meshes {
        let one_go = mesh.get_mesh_edge_graph();
        for budget in [1, 7, 1_000, 4_096, usize::MAX / 8] {
            let (stepped, _) = stepped_graph(mesh, budget);
            assert_eq!(stepped, one_go, "{name} in steps of {budget} faces");
        }
    }
}

#[test]
fn a_big_mesh_s_edge_graph_spans_many_steps() {
    let mesh = sheet(250, 200);
    assert_eq!(mesh.faces.len(), 100_000);
    let (_, count) = stepped_graph(&mesh, 4_096);
    // Filling alone is 100,000 / 4,096 steps; the sort, merges and walk add more.
    assert!(count > 50, "the graph was built in {count} steps");
}
