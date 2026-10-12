//! Port of `LevelLine/RoomRigidMotionTests.cs`: the room is a statement about the PART, so it may not
//! depend on where in the world the part is standing - the same pick, rigidly moved, reads the same
//! room, says the same things about it, and builds at every size it built at, removing the same
//! volume. This is the requirement the dented prism of `RailMonotoneTests` broke: it built at 0.1,
//! 0.2, 0.4 and 0.8 where it stood and refused BY SIZE at all four after a rotation and a
//! translation. C# `[Arguments]` rows are loops over the same-named test.

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::band_gap_reading;
use geometry_processing::bevel::level_line::gap_fill;
use geometry_processing::bevel::level_line::{
    BandGapResult, BandProfile, GapFillRefusal, RoomReading,
};
use vector_math::{Matrix4X4, Vector3};

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_room_geometry::{box_with_square_hole, hourglass_solid};
use crate::level_line_test_geometry::{
    cube_solid, cylinder_solid, signed_volume, signed_volume_of,
};
use crate::rail_monotone_tests::rounded_dented_prism_solid;
use crate::usable_room_tests::strip_solid;

const FOLD_DEGREES: f64 = 20.0;

/// Segments round a fill's arc - the same both placements, so the count cannot be the difference.
const FILL_SEGMENTS: i32 = 4;

const NAMES: [&str; 6] = [
    "dented-prism",
    "cube-top-rim",
    "box-mouth-rim",
    "strip-face",
    "cylinder-top-rim",
    "hourglass-face",
];

/// A fixed proper rotation (three turns, no reflection) and a translation.
fn motion() -> Matrix4X4 {
    Matrix4X4::create_rotation(Vector3::new(0.3, -0.7, 1.1))
        * Matrix4X4::create_translation_vector(Vector3::new(11.5, -3.25, 7.125))
}

#[derive(Clone)]
struct Solid {
    positions: Vec<Vector3>,
    indices: Vec<(i32, i32, i32)>,
    chain: Vec<i32>,
    profile: BandProfile,
}

fn solid_named(name: &str) -> Solid {
    let (positions, indices, chain, profile) = match name {
        "dented-prism" => {
            let (p, i, rim) = rounded_dented_prism_solid();
            (p, i, rim, BandProfile::Fillet)
        }
        "cube-top-rim" => {
            let (p, i) = cube_solid(1.0);
            (p, i, vec![4, 5, 6, 7], BandProfile::Chamfer)
        }
        "box-mouth-rim" => {
            let (p, i, rim) = box_with_square_hole(2.0, 0.5, 1.0);
            (p, i, rim, BandProfile::Chamfer)
        }
        "strip-face" => {
            let (p, i) = strip_solid();
            (p, i, vec![4, 5, 6, 7], BandProfile::Chamfer)
        }
        "cylinder-top-rim" => {
            let (p, i) = cylinder_solid(16, 1.0, 2.0);
            (p, i, (16..32).collect(), BandProfile::Fillet)
        }
        "hourglass-face" => {
            // The pinch, whose room is not read off any one point of the side but off the band's
            // own topology - a search, and so the pick most at risk of answering differently when
            // moved.
            let (p, i, rim) = hourglass_solid(10.0, 2.0, 4.0, 4.0);
            (p, i, rim, BandProfile::Chamfer)
        }
        _ => panic!("no such pick: {name}"),
    };
    Solid {
        positions,
        indices,
        chain,
        profile,
    }
}

fn moved(solid: &Solid) -> Solid {
    let motion = motion();
    Solid {
        positions: solid
            .positions
            .iter()
            .map(|p| p.transform(motion))
            .collect(),
        ..solid.clone()
    }
}

fn read(solid: &Solid) -> RoomReading {
    band_gap_reading::read_room(
        &solid.positions,
        &solid.indices,
        &solid.chain,
        true,
        Some(feature_edges_over(
            &solid.positions,
            &solid.indices,
            FOLD_DEGREES,
        )),
        solid.profile,
        1e-9,
        None,
        0.0,
        f64::INFINITY,
    )
    .expect("the room reads")
}

fn build(solid: &Solid, size: f64) -> BandGapResult {
    band_gap_entry_points::build(
        &solid.positions,
        &solid.indices,
        &solid.chain,
        true,
        Some(feature_edges_over(
            &solid.positions,
            &solid.indices,
            FOLD_DEGREES,
        )),
        size,
        solid.profile,
        &BandGapBuildOptions::default(),
    )
    .expect("a valid pick")
}

#[test]
fn the_room_is_the_same_after_a_rigid_motion() {
    for name in NAMES {
        let still = read(&solid_named(name));
        let moved = read(&moved(&solid_named(name)));

        assert_eq!(moved.read, still.read, "{name}");
        assert_eq!(moved.unbounded(), still.unbounded(), "{name}");
        assert_eq!(
            moved.chain_divides_sides, still.chain_divides_sides,
            "{name}"
        );

        if still.read && !still.unbounded() {
            assert!(
                (moved.room - still.room).abs() <= still.room.abs() * 1e-9,
                "the room is a reading of the part and not of where the part stands: {} still, {} moved ({name})",
                still.room,
                moved.room
            );
        }
    }
}

#[test]
fn every_size_that_builds_still_builds_after_a_rigid_motion() {
    for name in NAMES {
        for size in [0.1, 0.2, 0.4, 0.8] {
            let still_solid = solid_named(name);
            let moved_solid = moved(&still_solid);
            let still = build(&still_solid, size);
            let moved = build(&moved_solid, size);

            assert_eq!(
                moved.refusal, still.refusal,
                "{name} at {size} must answer the same moved as still"
            );

            if !still.built() {
                continue;
            }

            // The volume has to be read off a CLOSED mesh: a gap's mesh has its band removed and
            // nothing laid in its place, and the divergence sum over an open surface moves with the
            // origin, so a translated part would read differently however faithful the cut was. So
            // fill first.
            let still_fill =
                gap_fill::fill(&still, still_solid.profile, FILL_SEGMENTS).expect("the fill runs");
            let moved_fill =
                gap_fill::fill(&moved, moved_solid.profile, FILL_SEGMENTS).expect("the fill runs");

            assert_eq!(
                moved_fill.refusal, still_fill.refusal,
                "{name} at {size} must fill the same moved as still"
            );

            if still_fill.refusal != GapFillRefusal::None {
                continue;
            }

            let still_mesh = still_fill.mesh.as_ref().expect("a filled mesh").borrow();
            let moved_mesh = moved_fill.mesh.as_ref().expect("a filled mesh").borrow();

            // The strongest form of the claim: not a volume that agrees to a tolerance but the SAME
            // cut, vertex for vertex. Where this differs the refinement made a different decision
            // somewhere.
            assert_eq!(
                moved_mesh.vertices().len(),
                still_mesh.vertices().len(),
                "{name} at {size} must be the same cut moved as still"
            );
            assert_eq!(
                moved_mesh.triangles().len(),
                still_mesh.triangles().len(),
                "{name} at {size} must be the same cut moved as still"
            );

            let whole = signed_volume_of(&still_solid.positions, &still_solid.indices).abs();
            let still_taken = (whole - signed_volume(&still_mesh).abs()).abs();
            let moved_taken = (signed_volume_of(&moved_solid.positions, &moved_solid.indices)
                .abs()
                - signed_volume(&moved_mesh).abs())
            .abs();

            // Both readings are a DIFFERENCE of two volumes of the whole part's size, so the
            // rounding a rotation leaves scales with the part and not with the sliver taken: the
            // part is what the tolerance is relative to.
            assert!(
                (moved_taken - still_taken).abs() <= whole * 1e-9,
                "{name} at {size} takes the same volume wherever it stands"
            );
        }
    }
}
