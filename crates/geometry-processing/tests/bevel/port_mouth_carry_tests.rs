//! Port of `LevelLine/PortMouthCarryTests.cs`: one 64-edge port mouth of Many Holes at 0.375, as the
//! app handed it to the kernel (sweep watch 20 of
//! `BevelManyHolesRegressionTests.EveryPortMouthTakesTheHalfMillimetreFilletLarsAsked`). Its skew
//! rows are carried round the mouth (`FilletArc.AlongTheChain`), and each end's turn is the chain's
//! tangent change from its reading to the row's mean. With the tangent read piecewise, the row
//! reading ON a mouth vertex turned a half-joint (2.8 degrees) while its neighbour 0.0005 before the
//! vertex did not turn, the two rows folded through each other, and the Boolean kernel read the
//! answer SelfIntersecting.

use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::BandGapBuildOptions;
use geometry_processing::bevel::level_line::gap_fill;
use geometry_processing::bevel::level_line::GapFillRefusal;
use polygon_mesh::csg::boolean_processing::BooleanOperandVerdict;

use crate::boolean_kernel_gate;
use crate::end_face_builds::{as_saved, build_chains, default_budget};
use crate::fillet_thin_front_fin::profile_of;
use crate::level_line_fixture::LevelLineFixture;
use crate::round_mouth_fillet_crossings::crossing_pairs;
use crate::sweep_mesh_measures;

#[test]
fn a_carried_port_mouth_does_not_cross_itself() {
    let fixture = Rc::new(LevelLineFixture::load("many-holes-port-mouth-r0375"));
    let options = BandGapBuildOptions {
        budget: Some(default_budget(&fixture, true)),
        ..as_saved(&fixture, fixture.known_room)
    };
    let all: Vec<usize> = (0..fixture.chains.len()).collect();
    let gap = build_chains(&fixture, &all, fixture.size, &options);
    assert!(
        gap.built(),
        "band {:?}, room {}, set by {:?}",
        gap.refusal,
        gap.room,
        gap.room_set_by
    );

    let filled =
        gap_fill::fill(&gap, profile_of(&fixture), fixture.segments).expect("the fill runs");
    assert_eq!(filled.refusal, GapFillRefusal::None);

    let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
    let kept = sweep_mesh_measures::kept(&mesh);
    assert!(sweep_mesh_measures::is_closed_and_consistent(
        mesh.vertices(),
        &kept
    ));
    assert_eq!(crossing_pairs(&mesh), 0);
    assert_eq!(
        boolean_kernel_gate::verdict(&mesh),
        BooleanOperandVerdict::Clean
    );
}
