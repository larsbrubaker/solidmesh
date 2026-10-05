//! Rust-only tests for the pieces `MinimalHoleFill` builds on: `DSubmesh3` (`dsubmesh3`),
//! `RegionOperator` (`region_operator`) with `MeshEditor.ReinsertSubmesh`, the
//! `MeshQueries` edge-length statistics (`mesh_queries`) and `MathUtil.Area` / `AspectRatio`
//! (`vector3d`). geometry3Sharp ships no tests for them in agg-sharp.

use super::dmesh3::{DMesh3, INVALID_ID};
use super::dmesh3_tests::check_valid;
use super::dsubmesh3::DSubmesh3;
use super::mesh_queries::{edge_length_stats, edge_length_stats_from_edges};
use super::region_operator::RegionOperator;
use super::vector3d::{area, aspect_ratio};

/// A closed unit cube, two triangles per face wound outward; face `f` is triangles `2f, 2f+1`
/// and face 0 is z = 0.
fn cube() -> DMesh3 {
    let mut m = DMesh3::default();
    for i in 0..8 {
        m.append_vertex([(i & 1) as f64, ((i >> 1) & 1) as f64, ((i >> 2) & 1) as f64]);
    }
    for [a, b, c, d] in [
        [0, 2, 3, 1],
        [4, 5, 7, 6],
        [0, 1, 5, 4],
        [2, 6, 7, 3],
        [0, 4, 6, 2],
        [1, 3, 7, 5],
    ] {
        assert!(m.append_triangle(a, b, c) >= 0);
        assert!(m.append_triangle(a, c, d) >= 0);
    }
    m
}

#[test]
fn rust_only_area_and_aspect_ratio() {
    assert_eq!(area([0.0; 3], [3.0, 0.0, 0.0], [0.0, 4.0, 0.0]), 6.0);
    // 3-4-5: abc / (8 (s-a)(s-b)(s-c)) = 60 / 48
    assert_eq!(
        aspect_ratio([0.0; 3], [3.0, 0.0, 0.0], [0.0, 4.0, 0.0]),
        1.25
    );
    let h = 3.0_f64.sqrt() / 2.0;
    let eq = aspect_ratio([0.0; 3], [1.0, 0.0, 0.0], [0.5, h, 0.0]);
    assert!((eq - 1.0).abs() < 1e-12);
}

#[test]
fn rust_only_edge_length_stats() {
    let m = cube();
    let stats = edge_length_stats(&m, 0).expect("cube has edges");
    // 12 unit edges and 6 face diagonals
    assert_eq!(stats.min, 1.0);
    assert_eq!(stats.max, 2.0_f64.sqrt());
    let mut sum = 0.0;
    for eid in 0..m.max_edge_id() {
        let [a, b] = m.get_edge_v(eid);
        sum += super::vector3d::distance(m.vertex(a), m.vertex(b));
    }
    assert_eq!(stats.avg, sum / 18.0);

    let face0 = m.get_tri_edges(0);
    let from = edge_length_stats_from_edges(&m, face0);
    assert_eq!((from.min, from.max), (1.0, 2.0_f64.sqrt()));

    assert!(edge_length_stats(&DMesh3::default(), 0).is_err());
    assert!(edge_length_stats_from_edges(&m, [INVALID_ID]).avg.is_nan());
}

#[test]
fn rust_only_submesh_maps_vertices_and_finds_border() {
    let base = cube();
    let mut sub = DSubmesh3::new();
    sub.compute(&base, &[0, 1]).expect("live triangles");
    sub.compute_boundary_info(&base, &[0, 1]);

    let s = &sub.sub_mesh;
    assert_eq!((s.vertex_count(), s.triangle_count()), (4, 2));
    // vertices appended in first-use order: triangle 0 is [0, 2, 3], triangle 1 adds 1
    assert_eq!(sub.sub_to_base_v, vec![0, 2, 3, 1]);
    for sv in 0..4 {
        let bv = sub.map_vertex_to_base_mesh(sv);
        assert_eq!(sub.map_vertex_to_submesh(bv), sv);
        assert_eq!(s.vertex(sv), base.vertex(bv));
        assert!(sub.is_base_border_v(bv));
    }
    assert_eq!(sub.map_vertex_to_base_mesh(4), INVALID_ID);
    for bv in 4..8 {
        assert!(!sub.is_base_border_v(bv));
    }

    let mut bad = DSubmesh3::new();
    assert!(bad.compute(&base, &[0, 99]).is_err());
}

#[test]
fn rust_only_back_propagate_reinserts_an_edited_region_and_keeps_the_mesh_closed() {
    let mut base = cube();
    {
        let mut op = RegionOperator::new(&mut base, &[0, 1]).expect("live triangles");
        // edit the submesh: split the face diagonal, adding a vertex at the face centre
        let diag = op.region.sub_mesh.find_edge(0, 2);
        let (res, _) = op.region.sub_mesh.split_edge(diag, 0.5);
        assert_eq!(res, super::dmesh3::MeshResult::Ok);
        assert!(op.back_propropagate(true).expect("boundary info"));

        let tris = op.current_base_triangles().to_vec();
        assert_eq!(tris.len(), 4);
        assert!(tris.iter().all(|&t| op.base_mesh.is_triangle(t)));
        let interior = op.current_base_interior_vertices().expect("live triangles");
        assert_eq!(interior.len(), 1);
        assert_eq!(op.base_mesh.vertex(interior[0]), [0.5, 0.5, 0.0]);
    }
    check_valid(&base);
    assert!(base.is_closed());
    assert_eq!((base.vertex_count(), base.triangle_count()), (9, 14));
}

#[test]
fn rust_only_back_propagate_splits_edges_the_base_already_has() {
    // An unedited face region: its diagonal joins two border vertices that the base mesh
    // also joins, so the repair splits it before re-inserting (as C# does).
    let mut base = cube();
    {
        let mut op = RegionOperator::new(&mut base, &[0, 1]).expect("live triangles");
        assert!(op.back_propropagate(true).expect("boundary info"));
        assert_eq!(op.current_base_triangles().len(), 4);
    }
    check_valid(&base);
    assert!(base.is_closed());
    assert_eq!(base.triangle_count(), 14);

    // Without the repair the face goes back as two triangles on the original corners.
    let mut base = cube();
    {
        let mut op = RegionOperator::new(&mut base, &[0, 1]).expect("live triangles");
        assert!(op.back_propropagate(false).expect("boundary info"));
        assert!(op
            .current_base_interior_vertices()
            .expect("live triangles")
            .is_empty());
    }
    check_valid(&base);
    assert!(base.is_closed());
    assert_eq!((base.vertex_count(), base.triangle_count()), (8, 12));
}

/// Values printed by geometry3Sharp itself (the `geometry3Sharp.dll` MatterCAD builds, driven
/// from a scratch console) for the same inputs: the statistics and metrics bit for bit, and the
/// cube after a face region is split and re-inserted, triangle ids and vertex order included.
#[test]
fn rust_only_region_operator_matches_geometry3sharp() {
    let base = cube();
    let stats = edge_length_stats(&base, 0).expect("cube has edges");
    assert_eq!(
        (stats.min, stats.max, stats.avg),
        (1.0, std::f64::consts::SQRT_2, 1.1380711874576983)
    );
    let (a, b, c) = ([0.0; 3], [1.0, 0.2, 0.0], [0.3, 0.9, 0.1]);
    assert_eq!(aspect_ratio(a, b, c), 1.0033506349359798);
    assert_eq!(area(a, b, c), 0.4230839160261236);

    let expected_tris: [[i32; 3]; 14] = [
        [8, 3, 1],
        [8, 2, 3],
        [4, 5, 7],
        [4, 7, 6],
        [0, 1, 5],
        [0, 5, 4],
        [2, 6, 7],
        [2, 7, 3],
        [0, 4, 6],
        [0, 6, 2],
        [1, 3, 7],
        [1, 7, 5],
        [8, 0, 2],
        [8, 1, 0],
    ];
    for edit in [true, false] {
        let mut base = cube();
        {
            let mut op = RegionOperator::new(&mut base, &[0, 1]).expect("live triangles");
            if edit {
                let diag = op.region.sub_mesh.find_edge(0, 2);
                let _ = op.region.sub_mesh.split_edge(diag, 0.5);
            }
            assert!(op.back_propropagate(true).expect("boundary info"));
            assert_eq!(op.current_base_triangles(), &[1, 0, 12, 13]);
            assert_eq!(op.current_base_interior_vertices().expect("live"), vec![8]);
        }
        let tris: Vec<[i32; 3]> = base
            .triangle_indices()
            .map(|t| base.get_triangle(t))
            .collect();
        assert_eq!(tris, expected_tris);
        assert_eq!(base.vertex(8), [0.5, 0.5, 0.0]);
    }
}
