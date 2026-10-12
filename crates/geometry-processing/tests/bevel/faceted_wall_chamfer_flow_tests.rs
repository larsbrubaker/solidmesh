//! Port of `LevelLine/FacetedWallChamferFlowTests.cs`: a 2 mm chamfer on one wall crease of a
//! 40-sided radius-10 prism - the setback is wider than the 1.57 facet, so by decision 4 it FLOWS
//! onto the next gently turning facet, as Onshape does. The level line refused it "the most that
//! fits is about 1.55" - the facet width, with no other bevel anywhere. C# `[Arguments]` rows are a
//! loop over the same-named test.

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::gap_fill;
use geometry_processing::bevel::level_line::{BandProfile, GapFillRefusal};
use polygon_mesh::csg::boolean_processing::BooleanOperandVerdict;

use crate::boolean_kernel_gate;
use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_test_geometry::{cylinder_solid, signed_volume, signed_volume_of};
use crate::round_mouth_fillet_crossings::crossing_pairs;
use crate::sweep_mesh_measures;

/// The chamfer at 2 (setback 2 on a 1.57 facet) and a fillet at 25 (setback 25 tan 4.5 = 1.97):
/// both were refused at the facet's far crease, and both flow onto the next facet.
#[test]
fn a_bevel_wider_than_its_facet_flows_onto_the_next_facet() {
    for (profile, size) in [(BandProfile::Chamfer, 2.0), (BandProfile::Fillet, 25.0)] {
        const SIDES: i32 = 40;
        let (positions, indices) = cylinder_solid(SIDES, 10.0, 10.0);
        let feature = feature_edges_over(&positions, &indices, 1.0);
        let chains = vec![vec![0, SIDES]];
        let before = signed_volume_of(&positions, &indices);

        let options = BandGapBuildOptions {
            chord_tolerance: 0.0,
            snap: 1e-9,
            known_room: None,
            ..BandGapBuildOptions::default()
        };
        let gap = band_gap_entry_points::build_pick(
            &positions,
            &indices,
            &chains,
            &[false],
            Some(feature),
            size,
            profile,
            &options,
        )
        .expect("the pick is well formed");
        assert!(
            gap.built(),
            "band {:?}, room {}, set by {:?}",
            gap.refusal,
            gap.room,
            gap.room_set_by
        );

        let filled = gap_fill::fill_with_budget(&gap, profile, 4, None).expect("the fill runs");
        assert_eq!(filled.refusal, GapFillRefusal::None);
        let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
        assert_eq!(crossing_pairs(&mesh), 0);

        let kept = sweep_mesh_measures::kept(&mesh);
        assert!(sweep_mesh_measures::is_closed_and_consistent(
            mesh.vertices(),
            &kept
        ));
        assert_eq!(
            sweep_mesh_measures::shell_count(mesh.vertex_count(), &kept),
            1
        );
        assert_eq!(
            boolean_kernel_gate::verdict(&mesh),
            BooleanOperandVerdict::Clean
        );

        let after = signed_volume(&mesh);
        assert!(after > 0.0);
        assert!(after < before);
    }
}
