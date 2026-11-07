//! The room is a property of the part's SHAPE - scale the part by k and the room scales by k - and
//! the app's fixed snap only ever reads it low.

use geometry_processing::bevel::level_line::band_gap_reading::{
    read_room, read_room_of_pick, room_margin,
};
use geometry_processing::bevel::level_line::chain_on_the_mesh;
use geometry_processing::bevel::level_line::{BandProfile, RoomReading, RoomReadingKind};
use vector_math::Vector3;

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_fixture::LevelLineFixture;
use crate::level_line_room_geometry::l_prism;
use crate::level_line_test_geometry::cube_solid;
use crate::open_chain_fixtures;
use crate::raked_box_fixture;
use crate::several_chains_fixtures;

/// The relative accuracy these rows hold a self room to - MEASURED over these fixtures.
const ROOM_ACCURACY: f64 = 5e-2;

const SCALES: [f64; 3] = [0.01, 1.0, 100.0];

/// The app's own snap, which does NOT scale with the part.
const APP_SNAP: f64 = 1e-9;

/// One part and one pick, with the room's TRUTH where the shape names it exactly.
struct Part {
    positions: Vec<Vector3>,
    indices: Vec<(i32, i32, i32)>,
    chain: Vec<i32>,
    profile: BandProfile,
    truth: f64,
}

fn part_named(name: &str) -> Part {
    let (positions, indices, chain, profile, truth) = match name {
        "cube-top-rim" => {
            // A 2-wide box chamfered at 45 degrees runs out of the top face at half the width.
            let (positions, indices) = cube_solid(1.0);
            (
                positions,
                indices,
                vec![4, 5, 6, 7],
                BandProfile::Chamfer,
                1.0,
            )
        }
        "raked-box-rim" => {
            let (positions, indices, rim) = raked_box_fixture::pick(false, None);
            (
                positions,
                indices,
                rim,
                BandProfile::Fillet,
                raked_box_fixture::true_room(),
            )
        }
        "internal-fillet-face8-r1" => {
            let fixture = LevelLineFixture::load(name);
            let profile = if fixture.chamfer {
                BandProfile::Chamfer
            } else {
                BandProfile::Fillet
            };

            // An L face of arm 4: the band closes on the inner corner's bisector.
            let truth = 4.0 * 2f64.sqrt() / (1.0 + 2f64.sqrt());
            let chain = fixture.chains[0].clone();
            (fixture.positions, fixture.indices, chain, profile, truth)
        }
        _ => {
            // The L prism - carried for the SCALING, which needs no truth.
            let (positions, indices, rim) = l_prism();
            (positions, indices, rim, BandProfile::Fillet, f64::NAN)
        }
    };

    Part {
        positions,
        indices,
        chain,
        profile,
        truth,
    }
}

/// The room at `scale`, the snap scaled with the part.
fn room_at_scale(part: &Part, scale: f64) -> f64 {
    room_at_scale_with_snap(part, scale, APP_SNAP * scale)
}

/// The room at `scale` under a given snap.
fn room_at_scale_with_snap(part: &Part, scale: f64, snap: f64) -> f64 {
    reading_at_scale(part, scale, snap).room
}

/// The whole reading.
fn reading_at_scale(part: &Part, scale: f64, snap: f64) -> RoomReading {
    let scaled: Vec<Vector3> = part.positions.iter().map(|&p| p * scale).collect();
    let feature = feature_edges_over(&scaled, &part.indices, 30.0);
    read_room(
        &scaled,
        &part.indices,
        &part.chain,
        true,
        Some(feature),
        part.profile,
        snap,
        None,
        0.0,
        f64::INFINITY,
    )
    .expect("a valid pick")
}

/// The smallest unit setback along the chain at `scale`.
fn smallest_unit_setback_of(part: &Part, scale: f64) -> f64 {
    let scaled: Vec<Vector3> = part.positions.iter().map(|&p| p * scale).collect();
    let edges = chain_on_the_mesh::chain_edges(&part.chain, true);
    let units = chain_on_the_mesh::unit_setbacks(
        &scaled,
        &part.indices,
        &part.chain,
        true,
        &edges,
        part.profile,
    )
    .expect("every chain edge on two triangles");
    units.into_iter().fold(f64::INFINITY, f64::min)
}

/// The room scales with the part; one test per case.
fn the_room_scales_with_the_part(name: &str) {
    let part = part_named(name);
    let unscaled = room_at_scale(&part, 1.0);

    assert!(unscaled > 0.0, "{name} has a room to scale");

    // Every row here is a SELF room.
    assert_eq!(
        reading_at_scale(&part, 1.0, APP_SNAP).room_set_by.kind,
        RoomReadingKind::SelfRoom
    );

    for scale in SCALES {
        let room = room_at_scale(&part, scale);
        let expected = unscaled * scale;

        assert!(
            (room - expected).abs() / expected < 1e-9,
            "{name} at x{scale} read {room} where the unscaled reading scaled is {expected}"
        );
    }
}

#[test]
fn the_room_scales_with_the_part_cube_top_rim() {
    the_room_scales_with_the_part("cube-top-rim");
}

#[test]
fn the_room_scales_with_the_part_l_prism_rim() {
    the_room_scales_with_the_part("l-prism-rim");
}

#[test]
fn the_room_scales_with_the_part_raked_box_rim() {
    the_room_scales_with_the_part("raked-box-rim");
}

#[test]
fn the_room_scales_with_the_part_internal_fillet_face8_r1() {
    the_room_scales_with_the_part("internal-fillet-face8-r1");
}

/// The app's fixed snap only ever reads the room LOW; one test per case: the scaled room give or
/// take the margin `room_margin` takes off, and never above it when scaled down.
fn the_apps_fixed_snap_only_ever_reads_the_room_low(name: &str) {
    let part = part_named(name);
    let unscaled = room_at_scale(&part, 1.0);

    for scale in SCALES {
        let scaled_room = unscaled * scale;
        let room = room_at_scale_with_snap(&part, scale, APP_SNAP);

        let largest_margin = room_margin(
            smallest_unit_setback_of(&part, scale),
            APP_SNAP.max(APP_SNAP * scale),
        );

        assert!(
            (scaled_room - room).abs() <= largest_margin + 1e-11 * scaled_room,
            "{name} at x{scale} read {room}, {} off the scaled room {scaled_room} against a margin of {largest_margin}",
            scaled_room - room
        );
        if scale < 1.0 {
            assert!(
                room <= scaled_room,
                "{name} at x{scale} read {room} ABOVE the scaled room {scaled_room}"
            );
        }
    }
}

#[test]
fn the_apps_fixed_snap_only_ever_reads_the_room_low_cube_top_rim() {
    the_apps_fixed_snap_only_ever_reads_the_room_low("cube-top-rim");
}

#[test]
fn the_apps_fixed_snap_only_ever_reads_the_room_low_l_prism_rim() {
    the_apps_fixed_snap_only_ever_reads_the_room_low("l-prism-rim");
}

#[test]
fn the_apps_fixed_snap_only_ever_reads_the_room_low_raked_box_rim() {
    the_apps_fixed_snap_only_ever_reads_the_room_low("raked-box-rim");
}

#[test]
fn the_apps_fixed_snap_only_ever_reads_the_room_low_internal_fillet_face8_r1() {
    the_apps_fixed_snap_only_ever_reads_the_room_low("internal-fillet-face8-r1");
}

/// One test per case: a FOREIGN room
/// (an open-chain fade on a short cylinder) and a CHAIN-TO-CHAIN room (fixture C) scale too.
fn the_foreign_and_chain_to_chain_rooms_scale_with_the_part_too(name: &str, kind: RoomReadingKind) {
    let read = |scale: f64| -> RoomReading {
        if name == "fade-on-a-short-cylinder" {
            let (positions, indices) = open_chain_fixtures::cylinder_of_height(3.0);
            let scaled: Vec<Vector3> = positions.iter().map(|&p| p * scale).collect();
            let features = feature_edges_over(&scaled, &indices, open_chain_fixtures::FOLD_DEGREES);
            return read_room(
                &scaled,
                &indices,
                &open_chain_fixtures::top_rim_run(1),
                false,
                Some(features),
                BandProfile::Chamfer,
                APP_SNAP * scale,
                None,
                0.0,
                f64::INFINITY,
            )
            .expect("a valid pick");
        }

        let (plate, faces, chains) = several_chains_fixtures::named("C");
        let moved: Vec<Vector3> = plate.iter().map(|&p| p * scale).collect();
        let features = feature_edges_over(&moved, &faces, 30.0);
        read_room_of_pick(
            &moved,
            &faces,
            &chains,
            &[true, true],
            Some(features),
            BandProfile::Chamfer,
            APP_SNAP * scale,
            None,
            f64::NAN,
            0.0,
            f64::INFINITY,
        )
        .expect("a valid pick")
    };

    let unscaled = read(1.0);

    assert_eq!(
        unscaled.room_set_by.kind, kind,
        "{name} is here to carry a {kind:?} room, and read a {:?} one",
        unscaled.room_set_by.kind
    );

    for scale in SCALES {
        let reading = read(scale);
        let expected = unscaled.room * scale;

        assert_eq!(
            reading.room_set_by.kind, kind,
            "{name} at x{scale} was set by {:?}",
            reading.room_set_by.kind
        );
        assert!(
            (reading.room - expected).abs() / expected < 1e-9,
            "{name} at x{scale} read {} where the unscaled reading scaled is {expected}",
            reading.room
        );
    }
}

#[test]
fn the_foreign_and_chain_to_chain_rooms_scale_with_the_part_too_fade_on_a_short_cylinder() {
    the_foreign_and_chain_to_chain_rooms_scale_with_the_part_too(
        "fade-on-a-short-cylinder",
        RoomReadingKind::Foreign,
    );
}

#[test]
fn the_foreign_and_chain_to_chain_rooms_scale_with_the_part_too_two_chains_across_a_gap() {
    the_foreign_and_chain_to_chain_rooms_scale_with_the_part_too(
        "two-chains-across-a-gap",
        RoomReadingKind::ChainToChain,
    );
}

/// One test per case: within the stated
/// RELATIVE accuracy of the truth, and UNDER it at every scale.
fn the_room_is_under_the_truth_by_the_stated_accuracy(name: &str) {
    let part = part_named(name);

    for scale in SCALES {
        let truth = part.truth * scale;
        let room = room_at_scale(&part, scale);

        assert!(
            room <= truth,
            "{name} at x{scale} read {room} above its truth {truth}"
        );
        assert!(
            (truth - room) / truth < ROOM_ACCURACY,
            "{name} at x{scale} read {room} against a truth of {truth}"
        );
    }
}

#[test]
fn the_room_is_under_the_truth_by_the_stated_accuracy_cube_top_rim() {
    the_room_is_under_the_truth_by_the_stated_accuracy("cube-top-rim");
}

#[test]
fn the_room_is_under_the_truth_by_the_stated_accuracy_raked_box_rim() {
    the_room_is_under_the_truth_by_the_stated_accuracy("raked-box-rim");
}

#[test]
fn the_room_is_under_the_truth_by_the_stated_accuracy_internal_fillet_face8_r1() {
    the_room_is_under_the_truth_by_the_stated_accuracy("internal-fillet-face8-r1");
}
