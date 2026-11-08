//! A room set by a growing neighbour's FAR crease - away from the picked face - is told apart from
//! one set by the face itself, so the user reads which crease the bevel runs into and not "the face
//! is wide". The rib of `StripClipRibTests`: each flank's far crease with the back face, P R (x = 9)
//! or Q R (x = 11), bounds a chamfer's room.

use solidmesh::bevel::level_line::band_gap_reading;
use solidmesh::bevel::level_line::band_room;
use solidmesh::bevel::level_line::{BandProfile, RoomReading, RoomReadingKind};
use solidmesh_math::Vector3;

use crate::level_line_feature_edges::feature_edges_over;
use crate::strip_clip_rib_meshes;

const P: Vector3 = Vector3::new(9.0, 8.0, 10.0);

const Q: Vector3 = Vector3::new(11.0, 8.0, 10.0);

const R: Vector3 = Vector3::new(10.0, 8.0, 20.0);

/// The rib's far crease names the chamfer's room.
#[test]
fn the_ribs_far_crease_names_the_chamfers_room() {
    let reading = read(BandProfile::Chamfer);
    let said = format!("room {:?}, set by {:?}", reading.room, reading.room_set_by);

    assert_eq!(reading.room_set_by.kind, RoomReadingKind::Foreign, "{said}");
    assert!(reading.room_set_by.is_a_growing_crease(), "{said}");

    let at = reading.room_set_by.crease_at.expect("CreaseAt");
    let off_the_crease = distance_to_segment(at, P, R).min(distance_to_segment(at, Q, R));
    assert!(
        off_the_crease < 1e-9,
        "CreaseAt {at:?} should stand on P R or Q R"
    );
}

/// The far creases' foot, P and Q, stands 8 from the
/// chain across the top, and a 90-degree chamfer's setback is its size (less the reading's margin
/// of two snaps, 2e-9).
#[test]
fn splitting_the_far_creases_off_leaves_the_room() {
    let reading = read(BandProfile::Chamfer);

    assert!(
        (reading.room - 8.0).abs() <= 1e-8,
        "room {:?}, set by {:?}",
        reading.room,
        reading.room_set_by
    );
}

/// The rib's FILLET room is the ridge leaving the round
/// (the pierce room), which is no crease.
#[test]
fn a_lower_pierce_room_stays_plain_foreign() {
    let reading = read(BandProfile::Fillet);

    assert_eq!(reading.room_set_by.kind, RoomReadingKind::Foreign);
    assert!(
        !reading.room_set_by.is_a_growing_crease(),
        "room {:?}, set by {:?}",
        reading.room,
        reading.room_set_by
    );
}

/// Only a far crease alone under the room names the crease.
#[test]
fn only_a_far_crease_alone_under_the_room_names_the_crease() {
    let at = || P;

    assert!(band_room::the_foreign_source(3.0, 2.0, 2.0, &at).is_a_growing_crease());
    assert_eq!(
        band_room::the_foreign_source(3.0, 2.0, 2.0, &at).crease_at,
        Some(P)
    );
    assert!(
        !band_room::the_foreign_source(2.0, 2.0, 2.0, &at).is_a_growing_crease(),
        "a tie is the face's own reading"
    );
    assert!(
        !band_room::the_foreign_source(3.0, 2.0, 1.5, &at).is_a_growing_crease(),
        "the pierce room undercut the crease"
    );
    assert!(!band_room::the_foreign_source(2.0, 3.0, 2.0, &at).is_a_growing_crease());
    assert_eq!(
        band_room::the_foreign_source(f64::INFINITY, f64::INFINITY, f64::INFINITY, &at).kind,
        RoomReadingKind::None
    );
}

fn read(profile: BandProfile) -> RoomReading {
    let positions = strip_clip_rib_meshes::positions(None);
    let indices = strip_clip_rib_meshes::ribbed();
    let is_feature_edge = feature_edges_over(&positions, &indices, 20.0);
    band_gap_reading::read_room(
        &positions,
        &indices,
        &strip_clip_rib_meshes::CHAIN,
        false,
        Some(is_feature_edge),
        profile,
        1e-9,
        None,
        0.0,
        f64::INFINITY,
    )
    .expect("the reading")
}

fn distance_to_segment(point: Vector3, a: Vector3, b: Vector3) -> f64 {
    let along = b - a;
    let t = ((point - a).dot(along) / along.length_squared()).clamp(0.0, 1.0);
    (point - (a + (along * t))).length()
}
