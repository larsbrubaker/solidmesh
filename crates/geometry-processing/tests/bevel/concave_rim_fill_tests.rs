//! Ports `Tests/GeometryProcessingTests/LevelLine/ConcaveRimFillTests.cs`: a CONCAVE closed chain -
//! the floor-to-wall rim of a square pocket - filleted. The result is closed, wound the way the
//! input was, and its volume GROWS, because a fillet on a concave edge fills the corner with
//! material instead of taking it away.

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::{
    gap_fill, BandGapRefusal, BandProfile, GapFillRefusal,
};

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_test_geometry::{
    open_edges, signed_volume, signed_volume_of, smallest_triangle_area,
};
use crate::level_line_test_solids::square_pocket;

const FOLD_DEGREES: f64 = 20.0;

/// C# `Math.Sign` of a double.
fn sign(value: f64) -> i32 {
    if value > 0.0 {
        1
    } else if value < 0.0 {
        -1
    } else {
        0
    }
}

/// C# `APocketFloorRimFillsClosedAndKeepsItsWinding`.
#[test]
fn a_pocket_floor_rim_fills_closed_and_keeps_its_winding() {
    let (positions, indices, floor_rim) = square_pocket(2.0, 1.0, 1.0, 0.0);
    let before = signed_volume_of(&positions, &indices);

    let gap = band_gap_entry_points::build(
        &positions,
        &indices,
        &floor_rim,
        true,
        Some(feature_edges_over(&positions, &indices, FOLD_DEGREES)),
        0.2,
        BandProfile::Fillet,
        &BandGapBuildOptions::default(),
    )
    .expect("the pick builds or refuses by name");

    assert_eq!(gap.refusal, BandGapRefusal::None);

    let fill = gap_fill::fill(&gap, BandProfile::Fillet, 4).expect("the fill runs");

    assert_eq!(fill.refusal, GapFillRefusal::None);
    let mesh = fill.mesh.as_ref().expect("a filled mesh").borrow();
    assert!(open_edges(&mesh).is_empty());

    let after = signed_volume(&mesh);

    assert_eq!(sign(after), sign(before));
    assert!(
        after > before,
        "a fillet on a concave rim fills the corner: the pocket keeps more material, not less"
    );
    assert!(smallest_triangle_area(&mesh) > 0.0);
}
