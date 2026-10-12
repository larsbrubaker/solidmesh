//! Port of `LevelLine/SideWitnessTests.cs`: a side's survival is decided by a VERTEX, not inside a
//! search tolerance - right up against its room a pick still builds one closed loop a side, and a
//! pick with room to spare never takes the path at all.

use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::gap_fill;
use geometry_processing::bevel::level_line::side_witness;
use geometry_processing::bevel::level_line::{
    BandGapRefusal, BandGapResult, BandProfile, GapFillRefusal,
};
use vector_math::{Matrix4X4, Vector3};

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_fixture::LevelLineFixture;
use crate::level_line_test_geometry::{cube_solid, cylinder_solid, open_edges};
use crate::{raked_box_fixture, several_chains_fixtures};

const FOLD_DEGREES: f64 = 45.0;

/// C# `Pick`: one pick, built at any size and with the room either read or handed in.
struct Pick {
    name: String,
    build_with: Box<dyn Fn(f64, Option<f64>) -> BandGapResult>,
    profile: BandProfile,
    segments: i32,
    probe: f64,
}

/// C# `PickNamed`.
fn pick_named(name: &str, motion: Option<Matrix4X4>, scale: f64) -> Pick {
    match name {
        "cube-top-rim" => {
            let (positions, indices) = cube_solid(1.0);
            moved(
                name,
                &positions,
                indices,
                vec![4, 5, 6, 7],
                BandProfile::Chamfer,
                1,
                motion,
                scale,
            )
        }
        "cylinder-top-rim" => {
            let (positions, indices) = cylinder_solid(16, 1.0, 2.0);
            moved(
                name,
                &positions,
                indices,
                (16..32).collect(),
                BandProfile::Fillet,
                4,
                motion,
                scale,
            )
        }
        "raked-box-top-rim" => {
            let (positions, indices, rim) = raked_box_fixture::pick(false, None);
            moved(
                name,
                &positions,
                indices,
                rim,
                BandProfile::Fillet,
                4,
                motion,
                scale,
            )
        }
        _ => {
            let fixture = LevelLineFixture::load(name);
            let profile = if fixture.chamfer {
                BandProfile::Chamfer
            } else {
                BandProfile::Fillet
            };
            moved(
                name,
                &fixture.positions,
                fixture.indices.clone(),
                fixture.chains[0].clone(),
                profile,
                fixture.segments,
                motion,
                scale,
            )
        }
    }
}

/// C# `Moved`: the same pick on the same part, optionally turned and scaled - the part moves, not
/// the pick.
#[allow(clippy::too_many_arguments)]
fn moved(
    name: &str,
    positions: &[Vector3],
    indices: Vec<(i32, i32, i32)>,
    chain: Vec<i32>,
    profile: BandProfile,
    segments: i32,
    motion: Option<Matrix4X4>,
    scale: f64,
) -> Pick {
    let moved: Vec<Vector3> = positions
        .iter()
        .map(|&p| match motion {
            Some(m) => (p * scale).transform(m),
            None => p * scale,
        })
        .collect();
    let feature = feature_edges_over(&moved, &indices, FOLD_DEGREES);
    Pick {
        name: name.to_string(),
        build_with: Box::new(move |size, room| {
            let options = BandGapBuildOptions {
                known_room: room,
                ..BandGapBuildOptions::default()
            };
            band_gap_entry_points::build(
                &moved,
                &indices,
                &chain,
                true,
                Some(Rc::clone(&feature)),
                size,
                profile,
                &options,
            )
            .expect("the pick builds or refuses by name")
        }),
        profile,
        segments,
        // The size the room is first read at scales with the part (C#'s note on the span-over-size
        // cost law).
        probe: scale,
    }
}

/// C# `EverySizeUnderTheRoomKeepsItsLoop`.
#[test]
fn every_size_under_the_room_keeps_its_loop() {
    for name in [
        "internal-fillet-face8-r1",
        "internal-fillet-face8-r2",
        "cube-top-rim",
        "cylinder-top-rim",
        "raked-box-top-rim",
    ] {
        builds_up_against_its_room(&pick_named(name, None, 1.0));
    }
}

/// C# `ATurnedOrScaledPartKeepsTheSamePromise`.
#[test]
fn a_turned_or_scaled_part_keeps_the_same_promise() {
    for name in ["internal-fillet-face8-r1", "cube-top-rim"] {
        builds_up_against_its_room(&pick_named(
            name,
            Some(several_chains_fixtures::motion()),
            1.0,
        ));
        builds_up_against_its_room(&pick_named(name, None, 100.0));
        builds_up_against_its_room(&pick_named(name, None, 0.01));
    }
}

/// C# `ABuildWithRoomToSpareNeverLooksForAWitness`.
#[test]
fn a_build_with_room_to_spare_never_looks_for_a_witness() {
    for name in [
        "internal-fillet-face8-r1",
        "cube-top-rim",
        "cylinder-top-rim",
    ] {
        let pick = pick_named(name, None, 1.0);
        let room = (pick.build_with)(pick.probe, None).room;

        side_witness::reset();
        let built = (pick.build_with)(room * 0.5, None);

        assert_eq!(built.refusal, BandGapRefusal::None, "{name}");
        assert_eq!(
            side_witness::tally().inserted,
            0,
            "{name}: a side that survives on its own is left exactly as the refinement made it"
        );
    }
}

/// C# `ABoxRimAtItsRoomIsStillOneLoopASide`.
#[test]
fn a_box_rim_at_its_room_is_still_one_loop_a_side() {
    let pick = pick_named("cube-top-rim", None, 1.0);
    let room = (pick.build_with)(pick.probe, None).room;

    for fraction in [0.99, 1.0] {
        let built = (pick.build_with)(room * fraction, None);

        assert_eq!(
            built.refusal,
            BandGapRefusal::None,
            "the box's rim builds at {fraction} of its room"
        );

        let filled = gap_fill::fill(&built, pick.profile, pick.segments).expect("the fill");

        assert_eq!(filled.refusal, GapFillRefusal::None);
        let mesh = filled.mesh.as_ref().expect("a filled mesh");
        assert!(open_edges(&mesh.borrow()).is_empty());
    }
}

/// C# `BuildsUpAgainstItsRoom`: builds at the fractions of the room where a side is at its
/// thinnest, both ways round.
fn builds_up_against_its_room(pick: &Pick) {
    let read = (pick.build_with)(pick.probe, None);
    let room = read.room;

    assert!(room.is_finite(), "{} runs out of something", pick.name);
    assert!(
        room > 0.0,
        "{} read {} at size {}, refusal {:?}",
        pick.name,
        room,
        pick.probe,
        read.refusal
    );

    for fraction in [0.95, 0.99, 0.999, 1.0] {
        let size = room * fraction;
        for known in [None, Some(room)] {
            side_witness::reset();
            let built = (pick.build_with)(size, known);
            let tally = side_witness::tally();

            // Where a witness was needed at all it must be the STRUCTURAL line - the reading's own
            // last surviving point - and never the decision search.
            assert_eq!(
                tally.by_the_decision_search, 0,
                "{} at {fraction} of its room leaned on the decision search",
                pick.name
            );
            if tally.entered > 0 {
                assert!(
                    tally.by_the_readings_point > 0,
                    "{} at {fraction} of its room entered the path and found nothing",
                    pick.name
                );
            }

            assert_eq!(
                built.refusal,
                BandGapRefusal::None,
                "{} at {fraction} of its room {room} (known room: {}) must build",
                pick.name,
                known.is_some()
            );

            let filled = gap_fill::fill(&built, pick.profile, pick.segments).expect("the fill");

            assert_eq!(
                filled.refusal,
                GapFillRefusal::None,
                "{} at {fraction} of its room must fill",
                pick.name
            );
            let mesh = filled.mesh.as_ref().expect("a filled mesh");
            assert!(
                open_edges(&mesh.borrow()).is_empty(),
                "{} at {fraction} of its room is one closed shell",
                pick.name
            );
        }
    }
}
