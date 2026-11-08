//! Rust-only checks of the R1.P12 package: the level line's chaining of crossed edges and a mixed
//! corner's sector apex.

use solidmesh::bevel::level_line::band_cut::{chain, key};
use solidmesh::bevel::level_line::LevelLineChain;
use solidmesh::bevel::surface::surface_mixed_corner_sector::{
    the_trace, try_apex, try_apex_between, try_cross,
};
use solidmesh::bevel::surface::{
    SurfaceBandEnd, SurfaceLine, SurfaceMixedCorner, SurfaceMixedCornerSpine,
};
use solidmesh::bevel::BevelAdvisoryCode;
use solidmesh_math::Vector3;

#[test]
fn rust_only_an_open_run_is_one_chain_walked_from_its_lower_end() {
    let chains = chain(&[(2, 5), (1, 2), (5, 9)]);
    assert_eq!(chains, vec![LevelLineChain::new(vec![1, 2, 5, 9], false)]);
}

#[test]
fn rust_only_a_ring_closes_without_repeating_its_start() {
    let chains = chain(&[(3, 4), (4, 7), (3, 7)]);
    assert_eq!(chains, vec![LevelLineChain::new(vec![3, 4, 7], true)]);
}

#[test]
fn rust_only_separate_runs_and_rings_come_out_separately_and_nothing_is_nothing() {
    let chains = chain(&[(10, 11), (20, 21), (21, 22), (20, 22)]);
    assert_eq!(
        chains,
        vec![
            LevelLineChain::new(vec![10, 11], false),
            LevelLineChain::new(vec![20, 21, 22], true),
        ]
    );
    assert!(chain(&[]).is_empty());
    assert_eq!(key(5, 2), (2, 5));
    assert_eq!(key(2, 5), (2, 5));
}

/// A square mixed corner at (7, 7, 7), size 1, its axis +z through the origin (shared with
/// `rust_only_bevel_r2p01_tests.rs`).
pub fn mixed_corner(hands_over_at_a_mitre: bool) -> SurfaceMixedCorner {
    let plan = crate::rust_only_bevel_r1p06_tests::square_edge_plan(1.0, true);
    SurfaceMixedCorner::new(
        Vector3::new(7.0, 7.0, 7.0),
        1.0,
        0,
        plan.clone(),
        plan,
        Vector3::default(),
        SurfaceBandEnd {
            inward: Vector3::new(1.0, 0.0, 0.0),
            ..Default::default()
        },
        SurfaceBandEnd::default(),
        SurfaceMixedCornerSpine::try_create(
            SurfaceLine::new(Vector3::default(), Vector3::new(0.0, 0.0, 1.0)),
            Vector3::default(),
            Vector3::new(0.0, 0.0, 1.0),
            1.0,
            true,
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            1.0,
            std::f64::consts::FRAC_PI_2,
            Vector3::new(1.0, 0.0, 0.0),
            1e-9,
        )
        .expect("a square spine"),
        Vector3::default(),
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(0.0, 1.0, 0.0),
        hands_over_at_a_mitre,
    )
}

#[test]
fn rust_only_the_sector_apex_is_where_the_radius_meets_the_trace() {
    let corner = mixed_corner(false);
    assert_eq!(the_trace(&corner), Vector3::new(0.0, 1.0, 0.0));

    // The trace through (2, 0, 0) runs along y; the radius through (3, 3, 0) meets it at (2, 2, 0).
    let apex = try_apex(
        &corner,
        Vector3::new(2.0, 0.0, 0.0),
        Vector3::new(3.0, 3.0, 0.0),
    )
    .unwrap();
    assert!((apex - Vector3::new(2.0, 2.0, 0.0)).length() < 1e-12);

    // A trace through the axis gives the axis itself.
    let on_the_axis = try_apex(
        &corner,
        Vector3::new(0.0, 5.0, 0.0),
        Vector3::new(3.0, 3.0, 0.0),
    )
    .unwrap();
    assert_eq!(on_the_axis, corner.outer_rail_centre);

    // A radius parallel to the trace crosses nowhere: refused at the corner.
    let refused = try_apex(
        &corner,
        Vector3::new(2.0, 0.0, 0.0),
        Vector3::new(0.0, 3.0, 0.0),
    )
    .unwrap_err();
    assert_eq!(refused.code(), BevelAdvisoryCode::SurfaceCornerNotSupported);
}

#[test]
fn rust_only_a_sector_between_corners_without_a_mitre_closes_on_the_rail_centre() {
    let plain = mixed_corner(false);
    let mitre = mixed_corner(true);
    let first_on_the_cap = Vector3::new(2.0, 0.0, 0.0);
    let last_on_the_cap = Vector3::new(3.0, 3.0, 0.0);

    assert_eq!(
        try_apex_between(&plain, &plain, first_on_the_cap, last_on_the_cap).unwrap(),
        plain.outer_rail_centre
    );

    // The first hands over at a mitre: its trace through (2, 0, 0) meets the last's radius.
    let apex = try_apex_between(&mitre, &plain, first_on_the_cap, last_on_the_cap).unwrap();
    assert!((apex - Vector3::new(2.0, 2.0, 0.0)).length() < 1e-12);

    let crossed = try_cross(
        &plain,
        Vector3::default(),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 4.0, 0.0),
        Vector3::new(1.0, -1.0, 0.0),
    )
    .unwrap();
    assert!((crossed - Vector3::new(4.0, 0.0, 0.0)).length() < 1e-12);
}
