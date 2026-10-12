//! Ports `Tests/GeometryProcessingTests/LevelLine/ChainToChainRoomTests.cs`: two stretches of ONE
//! chain can run close across a side (the hourglass face's neck), and where they do the BUILD reads
//! the pinch - it refuses BY SIZE naming the neck's half width - while the room stays a reading of
//! WIDTH. C# `[Arguments]` rows are loops over the same-named test.

use std::time::Instant;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::band_gap_reading;
use geometry_processing::bevel::level_line::gap_fill;
use geometry_processing::bevel::level_line::seam_edges::FeatureEdgeTest;
use geometry_processing::bevel::level_line::{
    BandGapRefusal, BandGapResult, BandProfile, GapFillRefusal,
};
use vector_math::{Matrix4X4, Vector3};

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_room_geometry::hourglass_solid;
use crate::level_line_test_geometry::open_edges;

const FOLD_DEGREES: f64 = 45.0;

/// C# `Motion`: a fixed proper rotation and a translation - the motion `RoomRigidMotionTests` uses.
fn motion() -> Matrix4X4 {
    Matrix4X4::create_rotation(Vector3::new(0.3, -0.7, 1.1))
        * Matrix4X4::create_translation_vector(Vector3::new(11.5, -3.25, 7.125))
}

/// C# `Pick`.
struct Pick {
    positions: Vec<Vector3>,
    indices: Vec<(i32, i32, i32)>,
    chain: Vec<i32>,
    feature: FeatureEdgeTest,
}

/// C# `Hourglass`: `HourglassSolid` at the given neck width (C#'s other defaults: lobe 10, neck
/// length 4, half height 4), optionally moved by `motion`.
fn hourglass(neck_width: f64, moved: bool) -> Pick {
    let (mut positions, indices, rim) = hourglass_solid(10.0, neck_width, 4.0, 4.0);
    if moved {
        let by = motion();
        positions = positions.into_iter().map(|p| p.transform(by)).collect();
    }

    let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);
    Pick {
        positions,
        indices,
        chain: rim,
        feature,
    }
}

/// C# `RoomOf`.
fn room_of(pick: &Pick) -> f64 {
    band_gap_reading::room_of(
        &pick.positions,
        &pick.indices,
        &pick.chain,
        true,
        Some(pick.feature.clone()),
        BandProfile::Chamfer,
        1e-9,
    )
    .expect("the room reads")
}

/// C# `Build`.
fn build(pick: &Pick, size: f64) -> BandGapResult {
    band_gap_entry_points::build(
        &pick.positions,
        &pick.indices,
        &pick.chain,
        true,
        Some(pick.feature.clone()),
        size,
        BandProfile::Chamfer,
        &BandGapBuildOptions::default(),
    )
    .expect("the pick builds or refuses by name")
}

/// C# `ASizeThatClosesTheNeckRefusesNamingItsHalfWidth`: a size that closes the neck refuses BY
/// SIZE, and the room it names is a size KNOWN to fit - at or under the neck's half width, and
/// within an eighth of it.
#[test]
fn a_size_that_closes_the_neck_refuses_naming_its_half_width() {
    for (neck_width, expected) in [(2.0, 1.0), (1.0, 0.5)] {
        let pick = hourglass(neck_width, false);

        // Well inside the room - the face is half a lobe wide - and well outside the neck.
        let refused = build(&pick, 3.0);

        assert_eq!(
            refused.refusal,
            BandGapRefusal::Size,
            "{neck_width}: the band's two fronts meet across the neck and cut the side in two"
        );
        assert!(
            refused.room <= expected,
            "{neck_width}: the band closes across the neck at its half width, and a size above that cannot build: {}",
            refused.room
        );
        assert!(
            refused.room > expected * 0.8,
            "{neck_width}: three bisections of a factor-of-two bracket put the answer within an eighth of the true limit, so it is never worse than 8/9 of it: {}",
            refused.room
        );
        assert!(
            refused.pinch_probes <= 9,
            "{neck_width}: a pinch refusal is bought in a handful of band builds: {}",
            refused.pinch_probes
        );
        assert!(!refused.pinches_at_every_size_tried, "{neck_width}");
        assert!(
            refused.room < room_of(&pick),
            "{neck_width}: a refusal never names a room above the room the width allows"
        );
    }
}

/// C# `TheRoomIsTheFacesWidthAndSaysNothingOfTheNeck`: the room stays a reading of WIDTH - the wide
/// face, not the neck.
#[test]
fn the_room_is_the_faces_width_and_says_nothing_of_the_neck() {
    for neck_width in [2.0, 1.0] {
        let pick = hourglass(neck_width, false);

        // Nothing is spent looking for a neck that most picks do not have, so this number is half a
        // lobe - the greatest distance inside the face.
        let room = room_of(&pick);
        assert!(
            room > 2.0,
            "{neck_width}: the face is half a lobe wide: {room}"
        );
    }
}

/// C# `TheNeckRoomNamedBuildsAndAHairAboveItRefuses`: the room a refusal names BUILDS and fills
/// closed; well above the sliver the ladder leaves unresolved the pinch is still read.
#[test]
fn the_neck_room_named_builds_and_a_hair_above_it_refuses() {
    for neck_width in [2.0, 1.0] {
        let pick = hourglass(neck_width, false);
        let neck = build(&pick, 3.0).room;

        let at = build(&pick, neck);
        assert_eq!(at.refusal, BandGapRefusal::None, "{neck_width}");

        let filled = gap_fill::fill(&at, BandProfile::Chamfer, 1).expect("the fill runs");
        assert_eq!(filled.refusal, GapFillRefusal::None, "{neck_width}");
        let mesh = filled.mesh.as_ref().expect("a filled mesh");
        assert!(
            open_edges(&mesh.borrow()).is_empty(),
            "{neck_width}: the size a refusal names is still one closed shell"
        );

        // Well above the sliver the ladder leaves unresolved: the pinch is still read, and still
        // names a room that stands at or under this one.
        let above = build(&pick, neck * 1.5);
        assert_eq!(above.refusal, BandGapRefusal::Size, "{neck_width}");
        assert!(above.room <= neck * 1.5, "{neck_width}");
    }
}

/// C# `ThePinchRefusalCostsAHandfulOfProbes`: the ladder is the whole price of a pinch refusal, so
/// the count is pinned.
#[test]
fn the_pinch_refusal_costs_a_handful_of_probes() {
    // C# `[Arguments(2.0)]`, its one row.
    {
        let neck_width = 2.0;
        let pick = hourglass(neck_width, false);

        let started = Instant::now();
        let refused = build(&pick, 3.0);
        let elapsed = started.elapsed();

        assert_eq!(refused.refusal, BandGapRefusal::Size);
        assert!(
            refused.pinch_probes > 0,
            "a pinch room is read by probing, and the count is reported"
        );
        assert!(
            refused.pinch_probes <= 9,
            "six halvings and three bisections at the very most: {} probes in {} ms",
            refused.pinch_probes,
            elapsed.as_millis()
        );
    }
}

/// C# `TheNeckReadsTheSameAfterARigidMotion`: turning and moving the hourglass says the same
/// numbers.
#[test]
fn the_neck_reads_the_same_after_a_rigid_motion() {
    for neck_width in [2.0, 1.0] {
        let still = build(&hourglass(neck_width, false), 3.0);
        let moved = build(&hourglass(neck_width, true), 3.0);

        assert_eq!(moved.refusal, still.refusal, "{neck_width}");

        // The ladder's sizes are halvings and midpoints of the size ASKED, so both placements probe
        // the very same sizes; the rooms can differ only by at most one bisection step.
        assert!(
            (moved.room - still.room).abs() <= still.room.max(moved.room) / 8.0 + 1e-9,
            "{neck_width}: the neck is a reading of the part, not of where it stands: {} still, {} moved",
            still.room,
            moved.room
        );

        // And whatever each named, it is a size that BUILDS where it was named.
        assert_eq!(
            build(&hourglass(neck_width, false), still.room).refusal,
            BandGapRefusal::None,
            "{neck_width}"
        );
        assert_eq!(
            build(&hourglass(neck_width, true), moved.room).refusal,
            BandGapRefusal::None,
            "{neck_width}"
        );

        let still_room = room_of(&hourglass(neck_width, false));
        let moved_room = room_of(&hourglass(neck_width, true));
        assert!(
            (moved_room - still_room).abs() <= still_room.abs() * 1e-6,
            "{neck_width}: and so is the width: {still_room} / {moved_room}"
        );
    }
}
