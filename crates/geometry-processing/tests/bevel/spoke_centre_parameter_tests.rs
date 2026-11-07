//! A corner strip's
//! jump-end spoke is drawn about one chain point (`corner_rail_strip::spoke_centre_parameter`), and
//! that point must not turn on which leg float noise made a tie vertex's own reading name. Outer
//! faces' wedge tip on the 135-degree joint at 57.03 read 57.995 on one source and 56.064 on
//! another, 4e-7 apart, and its spokes crossed the strip on the second.

use geometry_processing::bevel::level_line::corner_rail_strip::spoke_centre_parameter;

const TIE: f64 = 1e-9;

#[test]
fn a_tip_against_a_plain_vertex_is_read_on_the_partners_leg_whichever_leg_it_reads() {
    let tip = (56.064, 57.995);
    let low = spoke_centre_parameter(tip, 56.064, (57.1276, 57.1276), 57.1276, TIE);
    let high = spoke_centre_parameter(tip, 57.995, (57.1276, 57.1276), 57.1276, TIE);
    assert_eq!(low, high);
    assert!((low - (57.995 + 57.1276) / 2.0).abs() <= 1e-12);
}

#[test]
fn two_tie_vertices_on_one_joint_read_the_same_whichever_leg_either_reads() {
    // The inner tip and an outer vertex both spanning the joint at 57.03: each end's leg is chosen
    // from where its partner stands, and a partner that is itself a tie stands for its whole span -
    // so neither end's own noisy reading may move the other's choice.
    let tip = (56.064, 57.995);
    let outer = (56.5, 57.6);
    let answers = [
        spoke_centre_parameter(tip, 56.064, outer, 56.5, TIE),
        spoke_centre_parameter(tip, 57.995, outer, 56.5, TIE),
        spoke_centre_parameter(tip, 56.064, outer, 57.6, TIE),
        spoke_centre_parameter(tip, 57.995, outer, 57.6, TIE),
    ];

    for answer in answers {
        assert_eq!(answer, answers[0], "{answers:?}");
    }
}
