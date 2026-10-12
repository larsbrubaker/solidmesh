//! Ports `Tests/GeometryProcessingTests/LevelLine/KnownRoomRefusesFirstTests.cs`: a build handed a
//! room it cannot fit in refuses BEFORE it does any work - the caller already holds the answer for
//! any size above it. C# `[Arguments]` rows are loops over the same-named test.

use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::band_gap_reading;
use geometry_processing::bevel::level_line::{BandGapRefusal, BandProfile, WorkBudget};

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_test_geometry::{cube_solid, cylinder_solid};
use crate::several_chains_fixtures;

const FOLD_DEGREES: f64 = 45.0;

/// What "does not grow with the model" is held to. The early refusal leaves before the work scope
/// is even opened, so it spends nothing at all; the ceiling stands well above that on purpose, while
/// anything that touches the MESH still fails it.
const SMALL_CEILING: i64 = 5000;

/// C# `Math.BitIncrement` for a positive finite value.
fn bit_increment(value: f64) -> f64 {
    f64::from_bits(value.to_bits() + 1)
}

fn known(room: f64) -> BandGapBuildOptions {
    BandGapBuildOptions {
        known_room: Some(room),
        ..BandGapBuildOptions::default()
    }
}

/// C# `ASizeOverAKnownRoomRefusesWithoutReadingTheModel`: the same rim on a coarse and a fine
/// tessellation of one cylinder refuses with the same sentence at the same handful of work units.
#[test]
fn a_size_over_a_known_room_refuses_without_reading_the_model() {
    for sides in [32, 2048] {
        let (positions, indices) = cylinder_solid(sides, 5.0, 4.0);
        let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);
        let rim: Vec<i32> = (sides..sides + sides).collect();

        let room = band_gap_reading::room_of(
            &positions,
            &indices,
            &rim,
            true,
            Some(Rc::clone(&feature)),
            BandProfile::Chamfer,
            1e-9,
        )
        .expect("the room reads");

        assert!(room.is_finite(), "a cylinder's rim runs out of its own cap");
        assert!(room > 0.0);

        let budget = Rc::new(
            WorkBudget::new(WorkBudget::default_limit(indices.len() as i32))
                .expect("a positive limit"),
        );
        let refused = band_gap_entry_points::build(
            &positions,
            &indices,
            &rim,
            true,
            Some(feature),
            room * 1.5,
            BandProfile::Chamfer,
            &BandGapBuildOptions {
                chord_tolerance: 0.0,
                snap: 1e-9,
                known_room: Some(room),
                budget: Some(Rc::clone(&budget)),
                ..BandGapBuildOptions::default()
            },
        )
        .expect("the build refuses by name");

        assert_eq!(refused.refusal, BandGapRefusal::Size);
        assert_eq!(
            refused.room, room,
            "the refusal names the room it was handed, and not a second reading of it"
        );
        assert!(
            budget.spent.get() <= SMALL_CEILING,
            "{sides} sides cost {} units, which must not grow with the model",
            budget.spent.get()
        );
    }
}

/// C# `APickOfSeveralChainsRefusesOverItsKnownRoomWithoutBuilding`: the same for a pick of several
/// chains, handed the pick's one room and asked for half again as much.
#[test]
fn a_pick_of_several_chains_refuses_over_its_known_room_without_building() {
    for fixture in ["A", "B", "C"] {
        let (positions, indices, chains) = several_chains_fixtures::named(fixture);
        let feature =
            feature_edges_over(&positions, &indices, several_chains_fixtures::FOLD_DEGREES);
        let closed = vec![true; chains.len()];

        let room = band_gap_reading::room_of_pick(
            &positions,
            &indices,
            &chains,
            &closed,
            Some(Rc::clone(&feature)),
            BandProfile::Chamfer,
            1e-9,
        )
        .expect("the room reads");

        assert!(room.is_finite(), "{fixture}");
        assert!(room > 0.0, "{fixture}");

        let budget = Rc::new(
            WorkBudget::new(WorkBudget::default_limit(indices.len() as i32))
                .expect("a positive limit"),
        );
        let refused = band_gap_entry_points::build_pick(
            &positions,
            &indices,
            &chains,
            &closed,
            Some(feature),
            room * 1.5,
            BandProfile::Chamfer,
            &BandGapBuildOptions {
                chord_tolerance: 0.0,
                snap: 1e-9,
                known_room: Some(room),
                budget: Some(Rc::clone(&budget)),
                ..BandGapBuildOptions::default()
            },
        )
        .expect("the build refuses by name");

        assert_eq!(refused.refusal, BandGapRefusal::Size, "{fixture}");
        assert_eq!(refused.room, room, "{fixture}");
        assert!(
            budget.spent.get() <= SMALL_CEILING,
            "{fixture}: the pick's refusal cost {} units",
            budget.spent.get()
        );
    }
}

/// C# `TheCheapRefusalSaysWhatTheFullBuildSays`: a build handed the room says at once what a
/// build that reads its own room spends the whole band build to say, field for field.
#[test]
fn the_cheap_refusal_says_what_the_full_build_says() {
    // C# `[Arguments(32)]`, its one row.
    {
        let sides = 32;
        let (positions, indices) = cylinder_solid(sides, 5.0, 4.0);
        let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);
        let rim: Vec<i32> = (sides..sides + sides).collect();

        let room = band_gap_reading::room_of(
            &positions,
            &indices,
            &rim,
            true,
            Some(Rc::clone(&feature)),
            BandProfile::Chamfer,
            1e-9,
        )
        .expect("the room reads");
        let size = room * 1.5;

        let build = |options: &BandGapBuildOptions| {
            band_gap_entry_points::build(
                &positions,
                &indices,
                &rim,
                true,
                Some(Rc::clone(&feature)),
                size,
                BandProfile::Chamfer,
                options,
            )
            .expect("the build refuses by name")
        };
        let itself = build(&BandGapBuildOptions::default());
        let told = build(&known(room));

        assert_eq!(told.refusal, itself.refusal);
        assert_eq!(told.room, itself.room);
        assert_eq!(told.pinch_probes, itself.pinch_probes);
        assert_eq!(
            told.pinches_at_every_size_tried,
            itself.pinches_at_every_size_tried
        );
        assert_eq!(told.offending_chain, itself.offending_chain);
        assert_eq!(told.offending_triangle, itself.offending_triangle);
        assert_eq!(told.offending_edge, itself.offending_edge);
    }
}

/// C# `TheKnownRoomItselfStillBuilds`: a size EQUAL to the room builds, and one bit above it
/// refuses - nothing is rounded off in the early refusal.
#[test]
fn the_known_room_itself_still_builds() {
    let (positions, indices) = cube_solid(2.0);
    let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);
    let rim = [4, 5, 6, 7];

    let room = band_gap_reading::room_of(
        &positions,
        &indices,
        &rim,
        true,
        Some(Rc::clone(&feature)),
        BandProfile::Chamfer,
        1e-9,
    )
    .expect("the room reads");

    let build = |size: f64| {
        band_gap_entry_points::build(
            &positions,
            &indices,
            &rim,
            true,
            Some(Rc::clone(&feature)),
            size,
            BandProfile::Chamfer,
            &known(room),
        )
        .expect("the build builds or refuses by name")
    };
    let at = build(room);
    let above = build(bit_increment(room));

    assert_eq!(
        at.refusal,
        BandGapRefusal::None,
        "the room IS the largest size that builds, handed in or read"
    );
    assert_eq!(above.refusal, BandGapRefusal::Size);
    assert_eq!(above.room, room);
}

/// C# `AReadingThatIsNotAboutSizeIsNotOneToHandIn`: a chain across a face with nothing counted as
/// a fold has no crease to bevel, so its reading is not a statement about size, and the build says
/// what is wrong BY NAME.
#[test]
fn a_reading_that_is_not_about_size_is_not_one_to_hand_in() {
    let (positions, indices) = cube_solid(1.0);
    let across = [4, 6];

    let reading = band_gap_reading::read_room(
        &positions,
        &indices,
        &across,
        false,
        Some(Rc::new(|_, _| false)),
        BandProfile::Fillet,
        1e-9,
        None,
        0.0,
        f64::INFINITY,
    )
    .expect("the room reads");

    assert!(
        !reading.is_a_size_statement(),
        "a pick with no crease to bevel has no room to offer: read {}",
        reading.room
    );
    assert!(!reading.room.is_infinite());

    let refused = band_gap_entry_points::build(
        &positions,
        &indices,
        &across,
        false,
        Some(Rc::new(|_, _| false)),
        0.2,
        BandProfile::Fillet,
        &BandGapBuildOptions::default(),
    )
    .expect("the build refuses by name");

    assert_eq!(
        refused.refusal,
        BandGapRefusal::ChainHasNoCreaseToBevel,
        "handed no room, the build still names what is wrong with the pick"
    );
}

/// C# `AGoodPicksReadingIsASizeStatement`: a reading of a pick that CAN be built is a size
/// statement, and a room to hand in.
#[test]
fn a_good_picks_reading_is_a_size_statement() {
    let (positions, indices) = cube_solid(2.0);
    let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);

    let reading = band_gap_reading::read_room(
        &positions,
        &indices,
        &[4, 5, 6, 7],
        true,
        Some(feature),
        BandProfile::Chamfer,
        1e-9,
        None,
        0.0,
        f64::INFINITY,
    )
    .expect("the room reads");

    assert!(reading.is_a_size_statement());
    assert!(reading.room > 0.0);
}

/// C# `PickValidationStillWinsOverTheSizeRefusal`: a pick whose chains share a vertex is refused
/// by name on the input mesh before any size is looked at.
#[test]
fn pick_validation_still_wins_over_the_size_refusal() {
    let (positions, indices, chains) = several_chains_fixtures::named("A");
    let feature = feature_edges_over(&positions, &indices, several_chains_fixtures::FOLD_DEGREES);

    // The same chain twice: every vertex is shared, so the pick touches itself.
    let touching = vec![chains[0].clone(), chains[0].clone()];

    let refused = band_gap_entry_points::build_pick(
        &positions,
        &indices,
        &touching,
        &[true, true],
        Some(feature),
        1000.0,
        BandProfile::Chamfer,
        &BandGapBuildOptions {
            chord_tolerance: 0.0,
            snap: 1e-9,
            known_room: Some(0.001),
            budget: None,
            ..BandGapBuildOptions::default()
        },
    )
    .expect("the build refuses by name");

    assert_eq!(
        refused.refusal,
        BandGapRefusal::ChainsTouch,
        "what is wrong with the PICK is said before what is wrong with the size"
    );
}
