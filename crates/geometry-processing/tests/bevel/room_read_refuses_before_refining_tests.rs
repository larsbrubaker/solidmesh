//! A build of one chain handed no room reads its own before it refines, and where that room
//! already refuses the size the build refuses there: the refinement could only spend work on a
//! band that is never cut.

use std::rc::Rc;

use solidmesh::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use solidmesh::bevel::level_line::band_gap_reading;
use solidmesh::bevel::level_line::{
    BandGapRefusal, BandProfile, FieldCounters, LevelLineWorkPhase, WorkBudget,
};

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_test_geometry::cylinder_solid;

const FOLD_DEGREES: f64 = 45.0;

/// A cylinder's rim asked for half again its room refuses by size, naming that room, without one
/// unit of refinement - on a coarse and a fine tessellation alike.
#[test]
fn a_size_over_the_room_read_in_the_build_refuses_without_refining() {
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
        assert!(room.is_finite() && room > 0.0, "{sides} sides: room {room}");

        let budget = Rc::new(
            WorkBudget::new(WorkBudget::default_limit(indices.len() as i32))
                .expect("a positive limit"),
        );
        FieldCounters::set_meter_on(true);
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
                known_room: None,
                budget: Some(Rc::clone(&budget)),
                ..BandGapBuildOptions::default()
            },
        );
        budget.tally_phase();
        FieldCounters::set_meter_on(false);
        let refused = refused.expect("the build refuses by name");

        assert_eq!(refused.refusal, BandGapRefusal::Size, "{sides} sides");
        assert_eq!(refused.room, room, "{sides} sides");
        let refined = budget.phase_units.borrow()[LevelLineWorkPhase::Refinement as usize];
        assert_eq!(
            refined, 0,
            "{sides} sides: the refinement spent {refined} units on a band refused by its room"
        );
    }
}
