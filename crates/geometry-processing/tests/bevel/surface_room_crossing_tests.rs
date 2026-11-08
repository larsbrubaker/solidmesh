//! What makes a skirt over a
//! band a CROSSING to sew rather than a collision to refuse, read on the pair of cuts alone - the
//! numbers "Every Chamfer Works.mcx" measures at its (-1, 40, 0) mixed corner, where the run-out
//! reaches 9.719 mm across a cap that runs 8 mm. The question is whether the run-out's touch row,
//! stopped where the face stops it, gets past the band's FOOT LINE.

use solidmesh::bevel::surface::{SurfaceRoomCrossing, SurfaceRoomCut, SurfaceRoomFootprint};
use solidmesh_math::Vector3;

/// The fixture's own radius, which every reach below is drawn at.
const RADIUS: f64 = 3.756;

/// How far the cap runs from that corner to the y = 48 edge the band stands on.
const ACROSS_THE_CAP: f64 = 8.0;

/// What the run-out reaches per unit of size there: 9.719 mm out of 3.756 mm of radius.
const THE_RUN_OUTS_RATE: f64 = 2.5876;

/// A skirt whose ideal reach OVERRUNS the face it crosses is still a crossing.
#[test]
fn a_skirt_that_overruns_the_face_still_crosses_the_strip_in_front_of_it() {
    let band = the_cap_edges_band(1.0);
    let skirt = the_run_outs_skirt(THE_RUN_OUTS_RATE);

    assert!(
        skirt.setback_per_size * RADIUS > ACROSS_THE_CAP,
        "the case is only itself while the run-out's ideal reach is past the far edge of the face"
    );

    let crossing = SurfaceRoomCrossing::try_cross(&band, &skirt, RADIUS)
        .expect("the touch row, trimmed at that edge, is still well past the band's foot line");

    assert!((crossing.gap - ACROSS_THE_CAP).abs() <= 1e-9);
    assert!(
        (crossing.overlap - ((RADIUS * (1.0 + THE_RUN_OUTS_RATE)) - ACROSS_THE_CAP)).abs() <= 1e-9,
        "and the two reach into each other by exactly what the pair rule used to refuse the size over"
    );
}

/// And the honest refusal is still refused: a run-out whose touch row dies BEFORE the band's foot
/// line never meets the strip at all.
#[test]
fn a_skirt_whose_touch_row_dies_short_of_the_foot_line_is_not_a_crossing() {
    let band = the_cap_edges_band(1.0);

    // 3.756 mm of run-out across 8 mm of cap, against a foot line 4.244 mm out.
    let skirt = the_run_outs_skirt(1.0);

    assert!(
        skirt.setback_per_size * RADIUS < ACROSS_THE_CAP - (RADIUS * band.setback_per_size),
        "the case is only itself while the touch row dies in front of the strip's near rail"
    );
    assert!(
        SurfaceRoomCrossing::try_cross(&band, &skirt, RADIUS).is_none(),
        "two footprints that never meet are the room they are, and there is nothing to sew"
    );
}

/// The band on the y = 48 cap edge, taking the cap toward the corner.
fn the_cap_edges_band(setback_per_size: f64) -> SurfaceRoomCut {
    SurfaceRoomCut {
        shape: SurfaceRoomFootprint::Band,
        plan_index: 0,
        edge_index: 0,
        side: 0,
        ..SurfaceRoomCut::new(
            1,
            Vector3::new(-21.0, 48.0, 0.0),
            Vector3::new(-1.0, 48.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, -1.0, 0.0),
            20.0,
            setback_per_size,
            ACROSS_THE_CAP,
        )
    }
}

/// The mixed corner's skirt at (-1, 40, 0), travelling straight at that band across the cap. Its
/// room is infinite, as a run-out's is.
fn the_run_outs_skirt(setback_per_size: f64) -> SurfaceRoomCut {
    SurfaceRoomCut {
        shape: SurfaceRoomFootprint::RunOutSkirt,
        plan_index: 1,
        corner_key: 28,
        ..SurfaceRoomCut::new(
            1,
            Vector3::new(-4.756, 40.0, 0.0),
            Vector3::new(-1.0, 40.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            3.756,
            setback_per_size,
            f64::INFINITY,
        )
    }
}
