//! Inner Face Should Bevel's raked bore, both mouths as the app hands them to the kernel. Each
//! mouth crosses two of the cube's convex edges, where its rim turns 90 degrees (a mitre); with the
//! skew rows' carry switched hard every size read self-intersecting, and the carry now fades
//! (`fillet_arc`'s along-the-chain carry). Parameter rows are a loop over the same-named test.

use std::rc::Rc;

use solidmesh::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use solidmesh::bevel::level_line::gap_fill;
use solidmesh::bevel::level_line::{BandProfile, GapFillRefusal, WorkBudget};
use solidmesh_mesh::csg::boolean_processing::BooleanOperandVerdict;

use crate::boolean_kernel_gate;
use crate::level_line_fixture::LevelLineFixture;
use crate::round_mouth_fillet_crossings::crossing_pairs;
use crate::sweep_mesh_measures;

#[test]
fn both_mouths_build_clean() {
    for size in [2.0, 1.0, 0.5] {
        let fixture = Rc::new(LevelLineFixture::load("inner-face-bore-both-mouths-r2"));
        let profile = if fixture.chamfer {
            BandProfile::Chamfer
        } else {
            BandProfile::Fillet
        };
        let mut budget = WorkBudget::new(WorkBudget::default_limit(fixture.indices.len() as i32))
            .expect("a positive limit");
        budget.input_is_float_stored = true;
        let options = BandGapBuildOptions {
            chord_tolerance: 0.0,
            snap: 1e-9,
            known_room: fixture.known_room,
            budget: Some(Rc::new(budget)),
            seam_epsilon: fixture.seam_epsilon,
            sharp_turn_degrees: fixture.sharp_turn_degrees,
            fillet_segments: fixture.segments,
        };
        let features = Rc::clone(&fixture);
        let gap = band_gap_entry_points::build_pick(
            &fixture.positions,
            &fixture.indices,
            &fixture.chains,
            &fixture.closed,
            Some(Rc::new(move |a, b| features.is_feature_edge(a, b))),
            size,
            profile,
            &options,
        )
        .expect("the pick is well formed");
        assert!(
            gap.built(),
            "size {size}: band {:?}, room {}, set by {:?}",
            gap.refusal,
            gap.room,
            gap.room_set_by
        );

        let filled = gap_fill::fill(&gap, profile, fixture.segments).expect("the fill runs");
        assert_eq!(filled.refusal, GapFillRefusal::None, "size {size}");
        let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
        let kept = sweep_mesh_measures::kept(&mesh);
        assert!(
            sweep_mesh_measures::is_closed_and_consistent(mesh.vertices(), &kept),
            "size {size}"
        );
        assert_eq!(crossing_pairs(&mesh), 0, "size {size}");
        assert_eq!(
            boolean_kernel_gate::verdict(&mesh),
            BooleanOperandVerdict::Clean,
            "size {size}"
        );
    }
}
