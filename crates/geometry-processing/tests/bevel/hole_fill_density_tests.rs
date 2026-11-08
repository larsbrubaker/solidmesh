//! The filler's interior matches the strips' density. Read against the same pick closed by the
//! older rings, whose corners are laid at the strips' own spacing: the whole fill with the filler
//! may carry at most a tenth more triangles. Parameter cases run as one loop per test.

use std::rc::Rc;
use std::time::Instant;

use solidmesh::bevel::level_line::{gap_fill, BandGapResult, BandProfile, GapFillRefusal};

use crate::hole_fill_corner_tests::{
    build, fill_with_the_filler, fixture_feature, room_of, two_faces,
};
use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_fixture::LevelLineFixture;
use crate::level_line_test_geometry::cube_solid;
use crate::several_chains_fixtures::FOLD_DEGREES;

const ALL_OPEN: [bool; 3] = [false, false, false];

/// The cube corner fills at the strips' density.
#[test]
fn the_cube_corner_fills_at_the_strips_density() {
    for (profile, size) in [
        (BandProfile::Fillet, 0.2),
        (BandProfile::Fillet, 0.5),
        (BandProfile::Chamfer, 0.5),
    ] {
        let (positions, indices) = cube_solid(1.0);
        let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);
        let gap = || {
            build(
                &positions,
                &indices,
                &two_faces(),
                &ALL_OPEN,
                &feature,
                size,
                profile,
            )
        };
        holds_the_strips_density(&gap, profile, 4, &format!("cube {profile:?} at {size}"));
    }
}

/// Outer faces fill at the strips' density.
#[test]
fn outer_faces_fill_at_the_strips_density() {
    for (profile, fraction_of_room) in [(BandProfile::Fillet, 0.42), (BandProfile::Chamfer, 0.5)] {
        let fixture = Rc::new(LevelLineFixture::load("outer-faces-r04"));
        let feature = fixture_feature(&fixture);
        let room = room_of(
            &fixture.positions,
            &fixture.indices,
            &fixture.chains,
            &fixture.closed,
            &feature,
            profile,
        );
        let gap = || {
            build(
                &fixture.positions,
                &fixture.indices,
                &fixture.chains,
                &fixture.closed,
                &feature,
                fraction_of_room * room,
                profile,
            )
        };
        holds_the_strips_density(
            &gap,
            profile,
            fixture.segments,
            &format!("outer faces {profile:?} at {fraction_of_room} of {room}"),
        );
    }
}

/// Holds the strips' density. Measured 2026-09-23: the cube fillet 162 triangles with
/// the filler against 246 with rings, the cube chamfer 20 against 24, outer-faces' fillet at 0.42
/// of room 23,539 against 24,643 and its chamfer 1,538 against 1,544. `added_triangles` is the
/// whole band - strips and caps.
fn holds_the_strips_density(
    gap: &dyn Fn() -> BandGapResult,
    profile: BandProfile,
    segments: i32,
    label: &str,
) {
    let rings_gap = gap();
    let watch = Instant::now();
    let rings = gap_fill::fill(&rings_gap, profile, segments).expect("the fill");
    let rings_ms = watch.elapsed().as_secs_f64() * 1000.0;
    let filler_gap = gap();
    let watch = Instant::now();
    let filled = fill_with_the_filler(&filler_gap, profile, segments);
    let filler_ms = watch.elapsed().as_secs_f64() * 1000.0;
    println!(
        "{label}: {} triangles with the filler ({filler_ms:.0} ms), {} with rings ({rings_ms:.0} ms)",
        filled.added_triangles.len(),
        rings.added_triangles.len()
    );
    assert_eq!(filled.refusal, GapFillRefusal::None, "{label}");
    assert_eq!(rings.refusal, GapFillRefusal::None, "{label}");
    // The allowance truncates to a whole triangle count.
    assert!(
        filled.added_triangles.len() as i32 <= (1.1 * rings.added_triangles.len() as f64) as i32,
        "{label}"
    );
}
