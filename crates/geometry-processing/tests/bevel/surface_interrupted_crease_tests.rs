//! Port of `SurfaceInterruptedCreaseTests.cs`: "Center Flows Through.mcx" in SURFACE mode - the
//! notch's centre crease picked BOTH sides of the nose welded over it, which is two selected pieces
//! of ONE crease rather than two creases.
//!
//! The Onshape control answers this pick with one cylinder of the fillet's radius running the whole
//! height of the part, the nose kept where it stands proud of it - and MEASURED, that control has no
//! cylinder at all between the nose's two faces: the nose is a tetrahedron, so every point of the
//! bead between its two planes is inside it. Two beads each dying into one of those faces are
//! therefore the same solid as one bead through the nose, which is what this pair builds as.
//!
//! What is still named is the size that would make the bead pass UNDER the nose instead of into its
//! faces - feet further from the crease than the nose's own root. On this fixture that starts
//! between 7 and 7.5 mm, the ridge standing 7.071 from the crease. C# `[Arguments]` rows are a loop.

use geometry_processing::bevel::{BevelAdvisoryArg, BevelAdvisoryCode};
use geometry_processing::topology::{BevelEdgeGroup, FeatureEdgeSet};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

use crate::bevel_volume_probes::{probe_cube, volume_inside};
use crate::mesh_measurements::signed_volume;
use crate::nose_in_notch_cut::anchor;
use crate::nose_in_notch_test_meshes as nose;
use crate::surface_test_build::{
    self, assert_sound, expect_built, TryBevelOptions, TryBevelOutcome,
};
use crate::topology_test_meshes::{
    create_two_aligned_blocks_on_a_plate, the_first_blocks_near_crease,
    the_second_blocks_near_crease,
};

/// The fixture's own radius and chording - 6 mm at 8 segments, as the stored file carries.
const RADIUS: f64 = 6.0;

const SEGMENTS: i32 = 8;

/// What a place named on the mesh can be asked to be: the point is a VERTEX of the part, and a
/// mesh vertex is three floats, so the fixture's exact 20/3 arrives 1.6e-7 off it.
const AS_NEAR_AS_A_FLOAT_HOLDS: f64 = 1e-6;

/// What the Onshape control measures on this pick: 6178.773 mm^3, against the part's own 6055.556.
const CONTROL_VOLUME: f64 = 6178.772;

/// The most a pair of strips CAPPED at the nose could measure: the part plus the 13.333 mm of the
/// crease's 20 that the two picked pieces cover, of the 154.513 mm^3 a full height bead adds.
const SQUARED_OFF_AT_THE_NOSE: f64 = 6055.556 + (154.513 * 13.333 / 20.0);

/// Where the boolean behind the corpus part chords the upper right crease - its t = 0.6.
fn the_chord_on_the_upper_right_crease() -> Vector3 {
    Vector3::new(12.0, 8.0, 12.0)
}

/// THE PAIR BUILDS, as two beads each dying into one of the nose's own faces - which on this shape
/// is the control's own answer and not an approximation of it. What is asserted is the solid:
/// closed, manifold, one shell, the control's volume to a chord, no raw corner left on the crease
/// line, and the sliver under the nose's lower face actually filled.
#[test]
fn two_pieces_of_one_crease_under_a_nose_build_as_two_beads_ending_on_its_faces() {
    for the_crease_arrives_chorded in [false, true] {
        let part = if the_crease_arrives_chorded {
            nose::create_with_upper_right_crease_split_at(the_chord_on_the_upper_right_crease())
        } else {
            nose::create()
        };
        let features = FeatureEdgeSet::create(&part);
        let (groups, names) = nose::centre_flows_through_groups(&features);

        assert_eq!(names, vec!["below the nose", "above the nose"]);

        let outcome = try_bevel(&part, &groups, RADIUS);
        let mesh = expect_built(
            outcome,
            "the nose spans the strip - every rail meets its lower face before its upper - so the \
             two pieces are two ordinary ends, each dying on a face",
        );

        assert_sound(&mesh);

        let volume = signed_volume(&mesh);

        assert!(
            (volume - CONTROL_VOLUME).abs() <= 5.0,
            "the control measures {CONTROL_VOLUME:.3} against the part's own {:.3}, and this \
             measures {volume:.3} - the difference being the 8 chords' excess over the cylinder \
             (chorded {the_crease_arrives_chorded})",
            nose::VOLUME
        );

        assert!(
            volume > SQUARED_OFF_AT_THE_NOSE,
            "two strips capped at the nose could not reach {SQUARED_OFF_AT_THE_NOSE:.3}, which is \
             the part plus the 13.333 mm of crease the pick covers - this is {volume:.3}"
        );

        // The corner is GONE, said as geometry: the control's nearest vertex to the crease line is
        // its two tangent lines, 4.24 away, so anything left on the line is raw corner. The two cap
        // fans' apexes on the floor and the ceiling are not - they are in those planes.
        let on_the_crease: Vec<Vector3> = mesh
            .vertices
            .iter()
            .map(|&vertex| Vector3::from_vector3_float(vertex))
            .filter(|vertex| {
                (vertex.x - 10.0).abs() < 0.1
                    && (vertex.y - 10.0).abs() < 0.1
                    && vertex.z > 0.01
                    && vertex.z < nose::SIZE - 0.01
            })
            .collect();

        assert_eq!(
            on_the_crease.len(),
            0,
            "the rounding runs the whole height of the crease, so no vertex may be left on the \
             crease line itself - and these are: {on_the_crease:?}"
        );

        // And that the material is really there in the thinnest place: the same 0.2 mm cube
        // BevelCentreFlowsThroughTests fills on the boolean leg, in the sliver under the nose's
        // lower face at z = 8, which is open air in the part and empty where a strip stopped.
        let under_the_nose = probe_cube(Vector3::new(10.0, 7.75, 8.0), 0.2);
        let probe_volume = 0.2 * 0.2 * 0.2;
        let inside = volume_inside(&mesh, &under_the_nose);

        assert!(
            (inside - probe_volume).abs() <= probe_volume * 0.02,
            "the sliver between the crease and the cylinder under the nose's lower face has to be \
             filled - it is where a pair capped at the nose leaves nothing at all: {inside}"
        );
    }
}

/// THE CASE THE NAME IS KEPT FOR: a size whose feet stand further from the crease than the nose's
/// own root, so the bead would pass UNDER the nose rather than into its faces. At 7.5 the two feet
/// stand at y = 4.697 against a ridge at y = 5. Named at the first end in the planned order, which
/// is the lower piece's upper end.
#[test]
fn a_bead_that_would_pass_under_the_nose_is_still_named() {
    let part = nose::create();
    let features = FeatureEdgeSet::create(&part);
    let (groups, _) = nose::centre_flows_through_groups(&features);

    let outcome = try_bevel(&part, &groups, 7.5);

    assert!(
        !outcome.built(),
        "at 7.5 the feet clear the ridge and the bead would run under the nose, which is not a \
         construction this has"
    );

    let reason = outcome.reason().expect("a refusal names its reason");

    assert_eq!(
        reason.code(),
        BevelAdvisoryCode::SurfaceCreaseInterruptedByAFeature,
        "the crease is interrupted rather than ended - instead {reason:?}"
    );

    let BevelAdvisoryArg::Vector3(named) = reason.args()[0] else {
        panic!("the end is named as a place - instead {reason:?}");
    };

    assert!(
        (named - nose::lower_crease_end()).length() < AS_NEAR_AS_A_FLOAT_HOLDS,
        "the end named is the first one in the planned order, which is the lower piece's upper \
         end {:?} - instead {named:?}",
        nose::lower_crease_end()
    );
}

/// The whole range the construction is honest over, one build each: every size whose feet stay
/// inside the nose's root reach - the ridge ends stand 7.071 from the crease - builds the same
/// shape.
#[test]
fn every_size_whose_feet_clear_the_ridge_builds() {
    for size in [2.0, 3.0, 4.0, 5.0, 6.0, 7.0] {
        let part = nose::create();
        let features = FeatureEdgeSet::create(&part);
        let (groups, _) = nose::centre_flows_through_groups(&features);

        let mesh = expect_built(
            try_bevel(&part, &groups, size),
            &format!("the feet at {size} stand inside the nose's root"),
        );

        assert_sound(&mesh);

        assert!(
            signed_volume(&mesh) > nose::VOLUME,
            "a fillet on a concave crease adds material, whatever its size ({size})"
        );
    }
}

/// The piece BELOW the nose picked on its own still builds: one chain never reaches any of the
/// readings at all - a run cannot resume itself - so a single pick is answered before anything is
/// measured, and the strip goes on ending against the nose's own lower face.
#[test]
fn one_piece_alone_still_ends_against_the_nose() {
    let part = nose::create();
    let features = FeatureEdgeSet::create(&part);
    let (groups, _) = nose::centre_flows_through_groups(&features);

    let mesh = expect_built(
        try_bevel(&part, &groups[..1], RADIUS),
        "the lower piece ends against the nose's lower face and always built",
    );

    assert_sound(&mesh);
}

/// The piece ABOVE the nose on its own, the same guard said at the other end of the same crease.
/// Worth both ends because the two are not symmetrical on the corpus mesh.
#[test]
fn the_piece_above_the_nose_alone_ends_against_it_too() {
    let part = nose::create();
    let features = FeatureEdgeSet::create(&part);
    let (groups, _) = nose::centre_flows_through_groups(&features);

    let mesh = expect_built(
        try_bevel(&part, &groups[1..2], RADIUS),
        "the upper piece ends against the nose's upper face",
    );

    assert_sound(&mesh);
}

/// TWO COLLINEAR CREASES THAT ARE NOT ONE. Two square blocks on a plate, plate between them, both
/// feet picked: nothing stands on either crease's faces, each really does end where its own block
/// ends, and the pair has to go on building. What turns the resumption reading down is that the
/// resuming run does not stand on the same two SURFACES.
#[test]
fn two_collinear_creases_with_ordinary_plate_between_them_still_build() {
    for (standing_proud, land) in [(true, 8.0), (false, 8.0), (false, 2.0)] {
        let part = create_two_aligned_blocks_on_a_plate(standing_proud, land);
        let features = FeatureEdgeSet::create(&part);
        let (first_from, first_to) = the_first_blocks_near_crease();
        let (second_from, second_to) = the_second_blocks_near_crease(land);

        let groups = vec![
            anchor(&features, first_from, first_to),
            anchor(&features, second_from, second_to),
        ];

        let together = try_bevel(&part, &groups, 2.0);
        let alone = try_bevel(&part, &groups[..1], 2.0);

        let reason_code = together.reason().map(|reason| reason.code());
        let alone_code = alone.reason().map(|reason| reason.code());

        assert_ne!(
            reason_code,
            Some(BevelAdvisoryCode::SurfaceCreaseInterruptedByAFeature),
            "nothing stands on these two creases - what is between them is plate - so neither end \
             may be read as an interruption: {:?} ({standing_proud}, {land})",
            together.reason()
        );

        // The whole claim, and said so that it does not depend on the fixture building: picking
        // the SECOND block's crease has to change nothing whatever about the first.
        assert_eq!(
            (together.built(), reason_code),
            (alone.built(), alone_code),
            "one crease alone says '{:?}' and the two together say '{:?}' ({standing_proud}, {land})",
            alone.reason(),
            together.reason()
        );

        if !together.built() {
            // Which is where the PROUD variant stands, for a reason of its own: a 2 mm band off
            // the plate's top reaches into the block standing on that same face, the walkback's
            // own standing-feature guard. It refuses identically with one block picked or two.
            assert_eq!(
                reason_code,
                Some(BevelAdvisoryCode::SurfaceCutMeetsStandingFeature),
                "the only thing that may stop this fixture is the band meeting the block - instead \
                 {:?} ({standing_proud}, {land})",
                together.reason()
            );

            continue;
        }

        let mesh = expect_built(together, "it built");

        assert_sound(&mesh);
    }
}

/// A surface-mode build of this fixture's groups, at this class's own chording.
fn try_bevel(part: &Mesh, groups: &[BevelEdgeGroup], size: f64) -> TryBevelOutcome {
    surface_test_build::try_bevel(
        part,
        groups,
        size,
        SEGMENTS,
        &CancelToken::new(),
        TryBevelOptions::default(),
    )
}
