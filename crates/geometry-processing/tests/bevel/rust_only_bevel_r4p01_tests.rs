//! Rust-only checks of bevel package R4.P01 where Rust fixes a C# bug: the virtual rail's minted
//! vertices joining a fill keep its end-face capping (divergence 4850).

use geometry_processing::bevel::level_line::open_gap_fill::with_minted;
use geometry_processing::bevel::level_line::{GapFillRefusal, GapFillResult};

#[test]
fn rust_only_a_fade_capped_by_end_faces_keeps_its_flag_when_the_rail_minted_vertices() {
    let capped = GapFillResult {
        ends_capped_by_end_faces: true,
        refusal: GapFillRefusal::None,
        added_triangles: vec![4, 5],
        added_vertices: vec![10, 11],
        triangles_of_chain: vec![2],
        vertices_of_chain: vec![2],
        work_spent: 7,
        work_limit: 100,
        ..GapFillResult::default()
    };

    let joined = with_minted(capped, &[8, 9]);

    assert!(joined.ends_capped_by_end_faces);
    assert_eq!(joined.added_vertices, vec![8, 9, 10, 11]);
    assert_eq!(joined.vertices_of_chain, vec![4]);
    assert_eq!(joined.added_triangles, vec![4, 5]);
    assert_eq!(joined.work_spent, 7);
    assert_eq!(joined.work_limit, 100);
}
