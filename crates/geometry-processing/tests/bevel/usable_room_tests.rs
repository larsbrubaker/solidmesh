//! Usable room tests: what the room a refusal names MEANS - it is the largest size that builds, so
//! asking for exactly it is answered with a solid and not with the same refusal again. A test that
//! covers several picks loops over them.

use std::rc::Rc;

use solidmesh::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use solidmesh::bevel::level_line::band_gap_reading;
use solidmesh::bevel::level_line::gap_fill;
use solidmesh::bevel::level_line::seam_edges::FeatureEdgeTest;
use solidmesh::bevel::level_line::{BandGapRefusal, BandGapResult, BandProfile, GapFillRefusal};
use solidmesh_math::Vector3;

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_fixture::LevelLineFixture;
use crate::level_line_room_geometry::{box_with_square_hole, hourglass_solid};
use crate::level_line_test_geometry::{cube_solid, open_edges};

const FOLD_DEGREES: f64 = 45.0;

/// One pick - the solid, its chain and feature edges - built at a size with or without
/// a known room, or asked its room without building.
struct Pick {
    positions: Vec<Vector3>,
    indices: Vec<(i32, i32, i32)>,
    chain: Vec<i32>,
    feature: FeatureEdgeTest,
    profile: BandProfile,
    segments: i32,
}

impl Pick {
    /// The build of this pick at `size`, handed `room` where the caller already read it.
    fn build_with(&self, size: f64, room: Option<f64>) -> BandGapResult {
        band_gap_entry_points::build(
            &self.positions,
            &self.indices,
            &self.chain,
            true,
            Some(Rc::clone(&self.feature)),
            size,
            self.profile,
            &BandGapBuildOptions {
                known_room: room,
                ..BandGapBuildOptions::default()
            },
        )
        .expect("the pick builds or refuses by name")
    }

    /// A build that reads its own room.
    fn build(&self, size: f64) -> BandGapResult {
        self.build_with(size, None)
    }

    /// The room this pick reads.
    fn room(&self) -> f64 {
        band_gap_reading::room_of(
            &self.positions,
            &self.indices,
            &self.chain,
            true,
            Some(Rc::clone(&self.feature)),
            self.profile,
            1e-9,
        )
        .expect("the room reads")
    }
}

fn solid_pick(positions: Vec<Vector3>, indices: Vec<(i32, i32, i32)>, chain: Vec<i32>) -> Pick {
    let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);
    Pick {
        positions,
        indices,
        chain,
        feature,
        profile: BandProfile::Chamfer,
        segments: 1,
    }
}

/// The pick a test row names.
fn pick_named(name: &str) -> Pick {
    match name {
        "cube-top-rim" => {
            let (positions, indices) = cube_solid(1.0);
            solid_pick(positions, indices, vec![4, 5, 6, 7])
        }
        "box-mouth-rim" => {
            let (positions, indices, rim) = box_with_square_hole(2.0, 0.5, 1.0);
            solid_pick(positions, indices, rim)
        }
        "strip-face" => {
            let (positions, indices) = strip_solid();
            solid_pick(positions, indices, vec![4, 5, 6, 7])
        }
        "hourglass-face" => {
            // The pinch: the whole top-face rim of two lobes joined by a neck, where the room is
            // the neck's half width and not half a lobe - see ChainToChainRoomTests.
            let (positions, indices, rim) = hourglass_solid(10.0, 2.0, 4.0, 4.0);
            solid_pick(positions, indices, rim)
        }
        _ => {
            let fixture = Rc::new(LevelLineFixture::load(name));
            let lookup = Rc::clone(&fixture);
            Pick {
                positions: fixture.positions.clone(),
                indices: fixture.indices.clone(),
                chain: fixture.chains[0].clone(),
                feature: Rc::new(move |a, b| lookup.is_feature_edge(a, b)),
                profile: if fixture.chamfer {
                    BandProfile::Chamfer
                } else {
                    BandProfile::Fillet
                },
                segments: fixture.segments,
            }
        }
    }
}

/// The 4 by 20 by 4 box with a top face of two triangles; the ridge-survives and room rigid
/// motion tests read this one too.
pub fn strip_solid() -> (Vec<Vector3>, Vec<(i32, i32, i32)>) {
    let positions = vec![
        Vector3::new(-2.0, -10.0, -2.0),
        Vector3::new(2.0, -10.0, -2.0),
        Vector3::new(2.0, 10.0, -2.0),
        Vector3::new(-2.0, 10.0, -2.0),
        Vector3::new(-2.0, -10.0, 2.0),
        Vector3::new(2.0, -10.0, 2.0),
        Vector3::new(2.0, 10.0, 2.0),
        Vector3::new(-2.0, 10.0, 2.0),
    ];
    let indices = vec![
        (0, 2, 1),
        (0, 3, 2),
        (4, 5, 6),
        (4, 6, 7),
        (0, 1, 5),
        (0, 5, 4),
        (1, 2, 6),
        (1, 6, 5),
        (2, 3, 7),
        (2, 7, 6),
        (3, 0, 4),
        (3, 4, 7),
    ];
    (positions, indices)
}

/// The next representable double above a positive finite value.
fn bit_increment(value: f64) -> f64 {
    f64::from_bits(value.to_bits() + 1)
}

/// For every pick that has a bounded room, the size EQUAL to
/// the room builds, closes and takes volume; and for a room read off WIDTH, a BIT above it refuses
/// by size naming that very same room. A room a PINCH refusal named is a size KNOWN to fit and not
/// the largest, so a size well above it is asked instead.
#[test]
fn the_room_named_is_a_size_that_builds() {
    for name in [
        "cube-top-rim",
        "box-mouth-rim",
        "strip-face",
        "hourglass-face",
        "internal-fillet-face8-r1",
        "internal-fillet-face8-r2",
        "internal-fillet-face9-r1",
        "internal-fillet-face9-r2",
    ] {
        let pick = pick_named(name);
        let at_one = pick.build(1.0);
        let room = at_one.room;

        // A build that cut the band and still refused is a PINCH refusal: the side left other than
        // its one closed loop at the size asked, and the room named was found on the pinch
        // search's grid.
        let pinches =
            at_one.refusal == BandGapRefusal::Size && !at_one.removed_triangles.is_empty();

        assert!(
            room.is_finite(),
            "{name}: every pick here runs out of something"
        );
        assert!(room > 0.0, "{name}: {room}");

        let at = pick.build(room);
        assert_eq!(
            at.refusal,
            BandGapRefusal::None,
            "{name}: the room IS the largest size that builds, so asking for it is answered with a solid"
        );

        let filled = gap_fill::fill(&at, pick.profile, pick.segments).expect("the fill runs");
        assert_eq!(filled.refusal, GapFillRefusal::None, "{name}");
        let mesh = filled.mesh.as_ref().expect("a filled mesh");
        assert!(
            open_edges(&mesh.borrow()).is_empty(),
            "{name}: a closed pick at its room is still one closed shell"
        );

        if pinches {
            // What a pinch room owes instead: it was bought in a handful of probes, and a size
            // well above it - outside the sliver the ladder leaves unresolved - still refuses,
            // naming no more room than that size.
            let well_above = pick.build(room * 1.5);
            assert_eq!(well_above.refusal, BandGapRefusal::Size, "{name}");
            assert!(well_above.room <= room * 1.5, "{name}");
            assert!(
                at_one.pinch_probes <= 9,
                "{name}: a pinch refusal costs a handful of band builds: {}",
                at_one.pinch_probes
            );
            continue;
        }

        let above = pick.build(bit_increment(room));
        assert_eq!(above.refusal, BandGapRefusal::Size, "{name}");
        assert!(
            (above.room - room).abs() <= 1e-9 * room.max(1.0),
            "{name}: the room is a fact about the part and the pick, not about the size that was asked: {} against {room}",
            above.room
        );
    }
}

/// ONE reading. The room a build reads
/// for itself and the room asked for without building must be the same number to the bit, and a
/// build handed that room must make exactly the mesh a build that read it itself made.
#[test]
fn a_room_asked_for_is_the_room_a_build_reads_and_builds_the_same_mesh() {
    for (name, size) in [
        ("cube-top-rim", 0.4),
        ("cube-top-rim", 0.8),
        ("box-mouth-rim", 0.5),
        ("box-mouth-rim", 1.0),
        ("strip-face", 0.5),
        ("strip-face", 1.0),
        ("hourglass-face", 0.5),
        ("hourglass-face", 1.0),
        ("internal-fillet-face8-r1", 0.5),
        ("internal-fillet-face8-r1", 1.0),
        ("internal-fillet-face8-r2", 0.5),
        ("internal-fillet-face8-r2", 1.0),
        ("internal-fillet-face9-r1", 0.5),
        ("internal-fillet-face9-r1", 1.0),
        ("internal-fillet-face9-r2", 0.5),
        ("internal-fillet-face9-r2", 1.0),
    ] {
        let pick = pick_named(name);
        let asked = pick.room();
        let read = pick.build_with(size, None);

        if read.refusal == BandGapRefusal::None {
            assert_eq!(
                read.room, asked,
                "{name} {size}: the room is a statement about the part and the pick, read one way"
            );
        } else {
            // A pinch is not a width, so the room a refusal names may stand BELOW the room the
            // width allows - but never above it: one reading still, read at the size asked.
            assert!(
                read.room <= asked,
                "{name} {size}: a refusal never names a room above the width's: {} of {asked}",
                read.room
            );
        }

        let told = pick.build_with(size, Some(asked));
        assert_eq!(told.refusal, read.refusal, "{name} {size}");
        if read.refusal != BandGapRefusal::None {
            // A known room does not skip the pinch: the loops are walked by every build anyway, so
            // the cheap path and the honest one refuse alike and name the same number.
            assert_eq!(told.room, read.room, "{name} {size}");
            continue;
        }

        let told_mesh = told.mesh.as_ref().expect("a built mesh").borrow();
        let read_mesh = read.mesh.as_ref().expect("a built mesh").borrow();
        assert_eq!(
            told_mesh.vertex_count(),
            read_mesh.vertex_count(),
            "{name} {size}"
        );
        assert_eq!(
            told_mesh.triangle_count(),
            read_mesh.triangle_count(),
            "{name} {size}"
        );
        assert_eq!(
            told.removed_triangles.len(),
            read.removed_triangles.len(),
            "{name} {size}"
        );

        for vertex in 0..read_mesh.vertex_count() {
            assert!(
                (told_mesh.position(vertex) - read_mesh.position(vertex)).length() <= 1e-12,
                "{name} {size}: vertex {vertex} must land where the build that read its own room put it"
            );
        }
    }
}

/// A room is a SIZE, so it never reads below zero, even where the
/// margin a kept strip needs (two snaps' worth) stands above the geometric room.
#[test]
fn the_room_never_reads_negative() {
    let (positions, indices) = cube_solid(2.0);
    let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);
    let rim = [4, 5, 6, 7];

    // A snap of a whole unit on a cube of side four: the margin stands above the room the width
    // allows. The cube's rim has 2 of geometric room at this size; a snap of 1.1 asks a margin of
    // 2.2 off it, and before the clamp the answer came back as -0.2.
    let room = band_gap_reading::room_of(
        &positions,
        &indices,
        &rim,
        true,
        Some(feature),
        BandProfile::Chamfer,
        1.1,
    )
    .expect("the room reads");

    assert!(
        room >= 0.0,
        "a room below zero is not a size a user could be offered: {room}"
    );
}

/// Whatever size is refused, the number the
/// refusal offers instead is strictly smaller.
#[test]
fn a_refusal_never_names_a_room_at_or_above_the_size_it_refused() {
    for (name, size) in [
        ("cube-top-rim", 1.1),
        ("cube-top-rim", 1.0),
        ("box-mouth-rim", 2.0),
        ("strip-face", 2.0),
        ("internal-fillet-face9-r2", 2.0),
    ] {
        let pick = pick_named(name);
        let result = pick.build(size);

        assert_eq!(
            result.refusal,
            BandGapRefusal::Size,
            "{name} {size}: these sizes stand at or above the pick's geometric room"
        );
        assert!(result.room < size, "{name} {size}: {}", result.room);
    }
}
