//! Rust-only tests for `MinimalHoleFill` (`minimal_hole_fill`). geometry3Sharp ships no tests
//! for it in agg-sharp; the expected values were printed by geometry3Sharp itself (MatterCAD's
//! built `geometry3Sharp.dll`, driven from a scratch console building the same meshes).

use std::collections::HashMap;

use super::dmesh3::DMesh3;
use super::dmesh3_tests::check_valid;
use super::mesh_boundary_loops::MeshBoundaryLoops;
use super::minimal_hole_fill::MinimalHoleFill;
use super::vector3d::aspect_ratio;

/// The surface of an `n` x `n` x `n` cube of unit cells, each face an `n` x `n` grid of
/// diagonal-split quads wound outward, with the +z face left open. Vertices are numbered in
/// first-use order.
pub(super) fn open_box(n: i32) -> DMesh3 {
    let faces: [([i32; 3], [i32; 3], [i32; 3]); 5] = [
        ([0, 0, 0], [0, 1, 0], [1, 0, 0]),
        ([0, 0, 0], [1, 0, 0], [0, 0, 1]),
        ([0, n, 0], [0, 0, 1], [1, 0, 0]),
        ([0, 0, 0], [0, 0, 1], [0, 1, 0]),
        ([n, 0, 0], [0, 1, 0], [0, 0, 1]),
    ];
    let mut m = DMesh3::default();
    let mut ids: HashMap<[i32; 3], i32> = HashMap::new();
    for (o, u, v) in faces {
        let mut vid = |i: i32, j: i32, m: &mut DMesh3| {
            let p = [
                o[0] + i * u[0] + j * v[0],
                o[1] + i * u[1] + j * v[1],
                o[2] + i * u[2] + j * v[2],
            ];
            *ids.entry(p)
                .or_insert_with(|| m.append_vertex([p[0] as f64, p[1] as f64, p[2] as f64]))
        };
        for j in 0..n {
            for i in 0..n {
                let p00 = vid(i, j, &mut m);
                let p10 = vid(i + 1, j, &mut m);
                let p11 = vid(i + 1, j + 1, &mut m);
                let p01 = vid(i, j + 1, &mut m);
                assert!(m.append_triangle(p00, p10, p11) >= 0);
                assert!(m.append_triangle(p00, p11, p01) >= 0);
            }
        }
    }
    m
}

fn fill_open_box(n: i32) -> (DMesh3, MinimalHoleFill) {
    let mut m = open_box(n);
    let loops = MeshBoundaryLoops::new(&m);
    assert_eq!(loops.loops.len(), 1);
    let mut fill = MinimalHoleFill::default();
    assert!(fill.apply(&mut m, &loops.loops[0]).expect("no exception"));
    (m, fill)
}

#[test]
fn rust_only_minimal_fill_closes_an_open_box_with_flat_well_shaped_triangles() {
    for n in [1, 3] {
        let (m, fill) = fill_open_box(n);
        check_valid(&m);
        assert!(m.is_closed());
        assert!(fill.fill_vertices.is_empty());
        for &t in &fill.fill_triangles {
            assert!(m.is_triangle(t));
            let [a, b, c] = m.get_triangle(t);
            let (pa, pb, pc) = (m.vertex(a), m.vertex(b), m.vertex(c));
            // the fill is the flat top face
            assert_eq!([pa[2], pb[2], pc[2]], [n as f64; 3]);
            assert!(aspect_ratio(pa, pb, pc) < 10.0);
        }
        let area: f64 = fill
            .fill_triangles
            .iter()
            .map(|&t| {
                let [a, b, c] = m.get_triangle(t);
                super::vector3d::area(m.vertex(a), m.vertex(b), m.vertex(c))
            })
            .sum();
        assert!((area - (n * n) as f64).abs() < 1e-9);
    }
}

#[test]
fn rust_only_minimal_fill_matches_geometry3sharp() {
    let (m, fill) = fill_open_box(1);
    assert_eq!(
        (m.triangle_count(), m.vertex_count(), m.max_vertex_id()),
        (12, 8, 9)
    );
    assert_eq!(fill.fill_triangles, vec![13, 12]);
    let tris: Vec<[i32; 3]> = fill
        .fill_triangles
        .iter()
        .map(|&t| m.get_triangle(t))
        .collect();
    assert_eq!(tris, vec![[7, 5, 4], [5, 7, 6]]);

    let (m, fill) = fill_open_box(3);
    assert_eq!(
        (m.triangle_count(), m.vertex_count(), m.max_vertex_id()),
        (100, 52, 53)
    );
    assert_eq!(fill.fill_triangles, (92..=101).rev().collect::<Vec<_>>());
    let tris: Vec<[i32; 3]> = fill
        .fill_triangles
        .iter()
        .map(|&t| m.get_triangle(t))
        .collect();
    assert_eq!(
        tris,
        vec![
            [42, 26, 50],
            [51, 42, 50],
            [26, 42, 24],
            [24, 42, 25],
            [42, 51, 45],
            [33, 51, 36],
            [36, 51, 39],
            [50, 26, 27],
            [45, 33, 32],
            [45, 51, 33],
        ]
    );
    assert!(fill.fill_vertices.is_empty());
}
