//! The narrowest statement of the ridge defect - a face coarse enough that the level line runs
//! THROUGH the middle of its triangles rather than across their corners. A 4 x 20 strip face made
//! of two triangles, its whole rim picked, inset by 1. Every corner of both triangles stands on the
//! chain and so reads inside, while the long diagonal between them rises a whole millimetre above
//! the line in its middle - the ridge of kept surface that ought to survive down the strip.
//! Classified by its corners alone each triangle is removed whole, and what should be one inset
//! rectangle comes apart. Nothing about the strip is special: it is the L-shaped top face of
//! "Simple Internal Fillet" with the L straightened out.

use solidmesh::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use solidmesh::bevel::level_line::gap_fill;
use solidmesh::bevel::level_line::{BandGapRefusal, BandProfile, GapFillRefusal};

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_test_geometry::{
    open_edges, signed_volume_of, volume_of_every_triangle_even_removed,
};
use crate::usable_room_tests::strip_solid;

const FOLD_DEGREES: f64 = 45.0;

/// The requirement: the picked face keeps ONE closed loop of surface, every vertex of it on the
/// true line, the solid untouched by the cut and the fill closing it.
#[test]
fn a_ridge_through_a_coarse_triangle_survives_the_removal() {
    let (positions, indices) = strip_solid();
    let before = signed_volume_of(&positions, &indices);
    let rim = [4, 5, 6, 7];

    let gap = band_gap_entry_points::build(
        &positions,
        &indices,
        &rim,
        true,
        Some(feature_edges_over(&positions, &indices, FOLD_DEGREES)),
        1.0,
        BandProfile::Chamfer,
        &BandGapBuildOptions::default(),
    )
    .expect("a valid pick");

    assert_eq!(
        gap.refusal,
        BandGapRefusal::None,
        "the strip is 4 wide and the setback is 1, so there is room either side"
    );
    {
        let mesh = gap.mesh.as_ref().expect("the working mesh").borrow();
        assert!((volume_of_every_triangle_even_removed(&mesh) - before).abs() <= 1e-9);

        // One loop for the top face - the inset rectangle, whole, not three pieces - and one for
        // the walls.
        assert_eq!(gap.loops.len(), 2);
        assert!(gap.loops.iter().all(|l| l.is_closed));

        let field = gap.chain().expect("the chain");
        for gap_loop in &gap.loops {
            for &vertex in &gap_loop.vertices {
                let value = field
                    .evaluate(mesh.position(vertex))
                    .expect("no budget")
                    .value;
                assert!(value.abs() <= 1e-9);
            }
        }
    }

    let filled = gap_fill::fill(&gap, BandProfile::Chamfer, 1).expect("the fill runs");
    assert_eq!(filled.refusal, GapFillRefusal::None);
    let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
    assert!(open_edges(&mesh).is_empty());
}
