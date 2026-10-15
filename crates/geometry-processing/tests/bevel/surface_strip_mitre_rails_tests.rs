//! Rust-only tests of the mitre's between-station bookkeeping (`surface_strip_mitre_rails`:
//! `merge`, `beside` and `on_the_chord`) and of point mode's exact weld (`point_candidates::weld`),
//! pinned directly; the surface tests also reach them through a whole build
//! (`surface_test_build.rs`).

use geometry_processing::bevel::level_line::point_candidates;
use geometry_processing::bevel::surface::surface_strip_mitre_rails::{beside, merge, on_the_chord};
use vector_math::Vector3;

#[test]
fn rust_only_merge_takes_the_union_per_chord_after_the_sector() {
    let sector = vec![vec![], vec![7], vec![8]];
    let union = vec![vec![1, 2], vec![3], vec![]];
    assert_eq!(
        merge(sector.clone(), Some(&union)),
        vec![vec![1, 2], vec![7, 3], vec![8]]
    );
    assert_eq!(merge(sector.clone(), None), sector);
}

#[test]
fn rust_only_beside_leads_or_follows_and_ignores_a_chord_off_the_ring() {
    let mut between = vec![vec![5], vec![]];
    beside(&mut between, 0, 4, true);
    beside(&mut between, 0, 6, false);
    beside(&mut between, 2, 9, true);
    beside(&mut between, -1, 9, true);
    assert_eq!(between, vec![vec![4, 5, 6], vec![]]);
}

#[test]
fn rust_only_on_the_chord_is_the_chord_before_the_rail_when_counted_down() {
    let mut between = vec![vec![1], vec![2], vec![3]];
    on_the_chord(&mut between, 1, 1, 10);
    on_the_chord(&mut between, 1, -1, 11);
    on_the_chord(&mut between, 0, -1, 12);
    assert_eq!(between, vec![vec![11], vec![10], vec![3]]);
}

#[test]
fn rust_only_the_weld_joins_negative_zero_to_zero() {
    let positions = [
        Vector3::new(0.0, 1.0, 2.0),
        Vector3::new(-0.0, 1.0, 2.0),
        Vector3::new(0.0, 1.0, 2.5),
        Vector3::new(0.0, 1.0, 2.0),
    ];
    assert_eq!(point_candidates::weld(&positions), vec![0, 0, 2, 0]);
}
