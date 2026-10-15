//! Rust-only tests for `MeshAutoRepair` (`mesh_auto_repair`) and `modulo_iteration`. The pinned
//! values were printed by geometry3Sharp itself on the same meshes.

use super::dmesh3::DMesh3;
use super::dmesh3_tests::check_valid;
use super::mesh_auto_repair::{modulo_iteration, MeshAutoRepair};
use super::minimal_hole_fill_tests::open_box;

fn digest(m: &DMesh3) -> (usize, usize, Vec<[i32; 3]>) {
    let tris = m.triangle_indices().map(|t| m.get_triangle(t)).collect();
    (m.vertex_count(), m.triangle_count(), tris)
}

#[test]
fn rust_only_modulo_iteration_visits_every_id_once() {
    assert_eq!(modulo_iteration(1, 31337).expect("n > 0"), vec![0]);
    let mut ids = modulo_iteration(10, 31337).expect("n > 0");
    // 31337 % 10 == 7: 0, 7, 4, 1, 8, 5, 2, 9, 6, 3
    assert_eq!(ids, vec![0, 7, 4, 1, 8, 5, 2, 9, 6, 3]);
    ids.sort();
    assert_eq!(ids, (0..10).collect::<Vec<_>>());
    assert!(modulo_iteration(0, 31337).is_err());
}

#[test]
fn rust_only_auto_repair_closes_an_open_box() {
    for n in [1, 3] {
        let mut m = open_box(n);
        assert!(MeshAutoRepair::default()
            .apply(&mut m)
            .expect("no exception"));
        check_valid(&m);
        assert!(m.is_closed());
    }
}

/// The open box with one triangle wound backward, repaired: counts, ids and a position-weighted
/// digest of every triangle's vertices equal geometry3Sharp's.
#[test]
fn rust_only_auto_repair_matches_geometry3sharp() {
    for (n, expected) in [(1, (8, 12, 14, 420133_i64)), (3, (52, 100, 102, 201919573))] {
        let mut m = open_box(n);
        assert_eq!(m.reverse_tri_orientation(1), super::dmesh3::MeshResult::Ok);
        assert!(MeshAutoRepair::default()
            .apply(&mut m)
            .expect("no exception"));
        let (v, t, tris) = digest(&m);
        let h: i64 = tris
            .iter()
            .enumerate()
            .map(|(i, t)| {
                (i as i64 + 1) * (t[0] as i64 * 7 + t[1] as i64 * 131 + t[2] as i64 * 1031)
            })
            .sum();
        assert_eq!(
            (v, t, m.max_triangle_id(), h),
            (expected.0, expected.1, expected.2, expected.3)
        );
        assert!(m.is_closed());
    }
}
