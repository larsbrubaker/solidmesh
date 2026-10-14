//! A polyline inserted into a flat region as a chain of edges, and the part it encloses taken out.

use geometry_processing::bevel::level_line::LevelLineMesh;
use vector_math::Vector3;

fn square() -> [Vector3; 4] {
    [
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(1.0, 1.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
    ]
}

const HALVES: [(i32, i32, i32); 2] = [(0, 1, 2), (0, 2, 3)];

fn line() -> [Vector3; 3] {
    [
        Vector3::new(0.0, 0.3, 0.0),
        Vector3::new(0.4, 0.6, 0.0),
        Vector3::new(1.0, 0.5, 0.0),
    ]
}

#[test]
fn a_polyline_across_a_square_becomes_edges_and_cuts_off_the_top() {
    let mut mesh = LevelLineMesh::from_triangles(&square(), &HALVES, None).expect("a square");
    let line = line();
    let chain = mesh
        .insert_polyline(&line, Vector3::new(0.0, 0.0, 1.0), &|_, _| true)
        .expect("no budget");
    assert!(chain.is_some());
    let chain = chain.expect("a chain");

    assert!((area(&mesh) - 1.0).abs() <= 1e-12);
    assert_eq!(mesh.position(chain[0]), line[0]);
    assert_eq!(mesh.position(chain[chain.len() - 1]), line[line.len() - 1]);
    for i in 0..chain.len() - 1 {
        assert!(
            mesh.has_edge(chain[i], chain[i + 1]),
            "{i}: {:?} to {:?}",
            mesh.position(chain[i]),
            mesh.position(chain[i + 1])
        );
    }

    // Every chain vertex lies on the polyline.
    for &v in &chain {
        let p = mesh.position(v);
        let y = if p.x <= 0.4 {
            0.3 + (0.75 * p.x)
        } else {
            0.6 - ((p.x - 0.4) / 6.0)
        };
        assert!((p.y - y).abs() <= 1e-12);
    }

    let smallest = kept(&mesh)
        .into_iter()
        .map(|t| triangle_area(&mesh, t))
        .fold(f64::INFINITY, f64::min);
    assert!(smallest > 0.0);

    // Above the polyline: 1 - (0.18 + 0.33).
    let removed = mesh.remove_enclosed(3, &chain, &|_| true);
    assert!(removed > 0);
    assert!((area(&mesh) - 0.51).abs() <= 1e-12);
}

fn kept(mesh: &LevelLineMesh) -> Vec<i32> {
    (0..mesh.triangle_count())
        .filter(|&t| !mesh.is_removed(t))
        .collect()
}

fn triangle_area(mesh: &LevelLineMesh, t: i32) -> f64 {
    let tri = mesh.triangles()[t as usize];
    (mesh.position(tri.b) - mesh.position(tri.a))
        .cross(mesh.position(tri.c) - mesh.position(tri.a))
        .length()
        / 2.0
}

fn area(mesh: &LevelLineMesh) -> f64 {
    kept(mesh).into_iter().map(|t| triangle_area(mesh, t)).sum()
}

/// A polyline with a point that is not a number is refused and the mesh is left as it was: a guard
/// against reading the orientation sign of a NaN, which has none.
#[test]
fn rust_only_a_polyline_with_a_nan_point_is_refused_and_the_mesh_untouched() {
    let mut mesh = LevelLineMesh::from_triangles(&square(), &HALVES, None).expect("a square");
    let before = (mesh.vertices.clone(), mesh.triangle_count());
    let nan_line = [
        Vector3::new(0.0, 0.3, 0.0),
        Vector3::new(f64::NAN, 0.6, 0.0),
        Vector3::new(1.0, 0.5, 0.0),
    ];

    let chain = mesh
        .insert_polyline(&nan_line, Vector3::new(0.0, 0.0, 1.0), &|_, _| true)
        .expect("no budget");

    assert!(chain.is_none());
    assert_eq!(mesh.vertices, before.0);
    assert_eq!(mesh.triangle_count(), before.1);

    let flat_plane =
        mesh.insert_polyline(&line()[..], Vector3::new(f64::NAN, 0.0, 1.0), &|_, _| true);
    assert!(flat_plane.expect("no budget").is_none());
    assert_eq!(mesh.vertices, before.0);
}
