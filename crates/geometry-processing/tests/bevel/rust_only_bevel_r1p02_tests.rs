//! Rust-only tests for the bevel-core package R1.P02's methods that no ported C# test class covers
//! yet: the row zipper (`SurfaceRowZip`), the end cut's run readings (`SurfaceEndCut`), the rolled
//! section's concave cap (`SurfaceEndFollowsTheWalkedChords.TryCapWithTheSection`) and the shared
//! side reading (`SurfaceSplitSewing.StandsOnTheSide`). Each pins the C# rule on a hand-built case.

use geometry_processing::bevel::surface::{
    surface_end_cut, surface_end_follows_the_walked_chords, surface_row_zip, surface_split_sewing,
    SurfaceVertexPool,
};
use geometry_processing::bevel::BevelAdvisoryCode;
use vector_math::Vector3;

/// Two rows of equal count alternate, the upper winning each tie, and a row of 2 chords hands over
/// to a row of 3 with every point used.
#[test]
fn rust_only_the_zipper_alternates_and_uses_every_point() {
    let mut band = Vec::new();
    surface_row_zip::sew(&mut band, &[0, 1, 2], &[10, 11, 12]);
    assert_eq!(band, vec![(0, 10, 1), (1, 10, 11), (1, 11, 2), (2, 11, 12)]);

    let mut band = Vec::new();
    surface_row_zip::sew(&mut band, &[0, 1, 2], &[10, 11, 12, 13]);
    // (i+1)*3 <= (j+1)*2: upper steps at i=0 only once j has reached 1.
    assert_eq!(
        band,
        vec![
            (0, 10, 11),
            (0, 11, 1),
            (1, 11, 12),
            (1, 12, 2),
            (2, 12, 13)
        ]
    );

    let mut band = Vec::new();
    surface_row_zip::sew(&mut band, &[], &[1, 2]);
    assert!(band.is_empty());
}

/// A triangle is wound away from the ball's centre on a convex corner and towards it in a pocket.
#[test]
fn rust_only_emit_outward_faces_out_of_the_material() {
    let mut pool = SurfaceVertexPool::new(&[]);
    let a = pool.add(Vector3::new(1.0, 0.0, 0.0));
    let b = pool.add(Vector3::new(0.0, 1.0, 0.0));
    let c = pool.add(Vector3::new(0.0, 0.0, 1.0));

    for (outside, sign) in [(false, 1.0), (true, -1.0)] {
        let mut faces = Vec::new();
        surface_row_zip::emit_outward(&pool, &mut faces, a, c, b, Vector3::ZERO, outside);
        assert_eq!(faces.len(), 1);
        let (v0, v1, v2) = faces[0];
        let from = pool.position_of(v0);
        let normal = (pool.position_of(v1) - from).cross(pool.position_of(v2) - from);
        assert!(normal.dot(Vector3::new(1.0, 1.0, 1.0)) * sign > 0.0);
    }
}

/// The vertices standing along a run come back in order along it, its ends and anything off it
/// left out; the side reading says where along the side a point stands.
#[test]
fn rust_only_standing_between_reads_the_run_in_order() {
    let mut pool = SurfaceVertexPool::new(&[]);
    let from = pool.add(Vector3::new(0.0, 0.0, 0.0));
    let to = pool.add(Vector3::new(10.0, 0.0, 0.0));
    let late = pool.add(Vector3::new(7.0, 0.0, 0.0));
    let off = pool.add(Vector3::new(5.0, 1.0, 0.0));
    let early = pool.add(Vector3::new(2.0, 0.0, 0.0));

    let faces = vec![(from, late, off), (off, early, to)];
    let used = surface_end_cut::vertices_of(&faces);
    assert_eq!(
        used.iter().copied().collect::<Vec<_>>(),
        vec![from, late, off, early, to]
    );

    assert_eq!(
        surface_end_cut::standing_between(&pool, &used, from, to),
        vec![early, late]
    );

    let (on, where_) = surface_split_sewing::stands_on_the_side(
        &pool,
        Vector3::ZERO,
        Vector3::new(10.0, 0.0, 0.0),
        Vector3::new(7.0, 0.0, 0.0),
    );
    assert!(on);
    assert!((where_ - 0.7).abs() < 1e-12);
    assert!(
        !surface_split_sewing::stands_on_the_side(
            &pool,
            Vector3::ZERO,
            Vector3::new(10.0, 0.0, 0.0),
            Vector3::new(10.0, 0.0, 0.0)
        )
        .0
    );
}

/// The rolled section's cap covers the quarter square between the corner, the flat foot, the
/// profile and the walked rim, and a section with no area is refused with nothing left behind.
#[test]
fn rust_only_the_concave_cap_covers_the_section_or_refuses() {
    let mut pool = SurfaceVertexPool::new(&[]);
    // The profile runs from the wall foot (index 0) to the flat foot.
    let profile = vec![
        pool.add(Vector3::new(0.0, 4.0, 0.0)),
        pool.add(Vector3::new(3.0, 3.0, 0.0)),
        pool.add(Vector3::new(4.0, 0.0, 0.0)),
    ];
    let walked = vec![pool.add(Vector3::new(0.0, 2.0, 0.0))];
    let normal = Vector3::new(0.0, 0.0, 1.0);
    let mut faces = Vec::new();

    let along = surface_end_follows_the_walked_chords::try_cap_with_the_section(
        &mut pool,
        &mut faces,
        &profile,
        0,
        Vector3::ZERO,
        normal,
        &walked,
    )
    .expect("a quarter section is a ring the ears can finish");

    assert_eq!(along, vec![Vec::<i32>::new(); 2]);
    let area: f64 = faces
        .iter()
        .map(|&(a, b, c)| {
            let from = pool.position_of(a);
            0.5 * (pool.position_of(b) - from)
                .cross(pool.position_of(c) - from)
                .dot(normal)
        })
        .sum();
    // Corner, flat foot (4,0), (3,3), wall foot (0,4), walked (0,2): shoelace area 12.
    assert!((area - 12.0).abs() < 1e-9);

    let mut pool = SurfaceVertexPool::new(&[]);
    let flat = vec![
        pool.add(Vector3::new(1.0, 0.0, 0.0)),
        pool.add(Vector3::new(2.0, 0.0, 0.0)),
    ];
    let mut faces = vec![(0, 1, 0)];
    let refused = surface_end_follows_the_walked_chords::try_cap_with_the_section(
        &mut pool,
        &mut faces,
        &flat,
        1,
        Vector3::ZERO,
        normal,
        &[],
    )
    .expect_err("a section on one line covers nothing");
    assert_eq!(
        refused.code(),
        BevelAdvisoryCode::SurfaceStripEndNotSupported
    );
    assert_eq!(faces, vec![(0, 1, 0)]);
}
