//! The inside-corner mirror of `bevel_three_run_corner_tests` - THREE concave runs of one 5 mm
//! fillet meeting at one concave junction on the nose-in-notch part, the selection "5mm Bevel
//! Center Bottom.mcx" carries (`nose_in_notch_test_meshes::centre_bottom_groups`), held to the
//! Onshape control's numbers.
//!
//! The two nose creases end at the ridge, and the runs that would partner them there - the two
//! UPPER nose creases - are not selected. So what bounds each nose bead is the PLANE of the face
//! across the ridge: each bead rides the slope until its own tangent line meets that plane, 4.28 mm
//! past the crease, and dies there. The rebuild itself is `bead_sequence::light_tools_and_fills`, the fill
//! patch trimmed to its own void cone, then `bead_sequence::clip_and_union`.

use vector_math::Vector3;

use crate::bead_sequence::{clip_and_union, light_tools_and_fills};
use crate::mesh_measurements::{shell_count, signed_volume};
use crate::nose_in_notch_test_meshes as meshes;
use geometry_processing::topology::FeatureEdgeSet;

const RADIUS: f64 = 5.0;

const SEGMENTS: i32 = 8;

/// What the Onshape control measures - the part's 6055.556 plus a fillet of 62.273.
const CONTROL_VOLUME: f64 = 6117.829;

/// How far the rebuild's volume may sit from the control's. Two chordings of one fillet disagree
/// by the sum of their sagittas over its area, and this one steps 8 segments where the control is
/// all but analytic; the whole fillet is 62.273 mm^3, so this is a twentieth of what is being
/// measured.
const VOLUME_TOLERANCE: f64 = 3.0;

/// Where the left nose bead dies, 4.28 mm past its crease's end: on the left slope, on the bead's
/// own tangent line there, and on the plane of the upper nose face carried out over the notch.
fn left_run_out() -> Vector3 {
    Vector3::new(2.2608, 2.2608, 8.1739)
}

/// The right nose bead's run-out, [`left_run_out`] mirrored about x = 10.
fn right_run_out() -> Vector3 {
    Vector3::new(17.7392, 2.2608, 8.1739)
}

/// How near a vertex of the rebuild has to come to a run-out point for the bead to have ended
/// there. A fifth of a millimetre: under the 0.907 mm the whole gore between the two tangent lines
/// is wide, and far over the chord of one arc segment.
const RUN_OUT_TOLERANCE: f64 = 0.2;

#[test]
fn the_three_beads_fill_the_junction_and_the_nose_beads_die_on_the_upper_nose_faces_plane() {
    let part = meshes::create();
    let features = FeatureEdgeSet::create(&part);
    let (groups, _) = meshes::centre_bottom_groups(&features);
    let built = light_tools_and_fills(&part, &groups, RADIUS, SEGMENTS).expect(
        "the three beads and their fill patch have to build and union onto the part at all",
    );
    let (result, tally) = clip_and_union(&part, &built.clippable, built.solids);

    assert_eq!(
        tally.unclipped, 0,
        "every bead and the fill have to be trimmed to what they are entitled to occupy - a solid \
         that goes on unclipped is one that may stand in air out of the far side of a wall"
    );

    assert_eq!(
        shell_count(&result),
        1,
        "all three beads meet at one corner, so none of them may be left floating"
    );

    let volume = signed_volume(&result);

    assert!(
        (volume - CONTROL_VOLUME).abs() <= VOLUME_TOLERANCE,
        "the control measures {CONTROL_VOLUME:.3} mm^3 against the part's own {:.3}, and this \
         measures {volume:.3}. More than that is a bead standing in air past the material it should \
         have stopped on; less is a bead that stopped at its crease and left the ridge standing",
        meshes::VOLUME
    );

    for (name, run_out) in [("left", left_run_out()), ("right", right_run_out())] {
        let nearest = result
            .vertices
            .iter()
            .map(|vertex| (vertex.as_vector3() - run_out).length())
            .fold(f64::INFINITY, f64::min);

        assert!(
            nearest <= RUN_OUT_TOLERANCE,
            "the {name} nose bead has to run 4.28 mm past its crease's end to ({:.3}, {:.3}, \
             {:.3}), where its tangent line on the slope meets the plane of the face across the \
             ridge - the nearest vertex of the rebuild is {nearest:.3} away",
            run_out.x,
            run_out.y,
            run_out.z
        );
    }
}
