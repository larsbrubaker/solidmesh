//! Where "Bevel to Center Feature"'s left-slope top edge (G2, fixture chain 2: v18 - v2) ends on
//! the notch's concave crease, the Onshape control runs its round out FLAT in the right slope's
//! plane - app frame x + y = 20, the plane of the unpicked right-slope top edge v12 - v18 and of
//! the top face's own flat edge v18 - v7, which is collinear with it. Without the square-crease end
//! clip the rebuild carries the round on past that plane into the top face's far triangles and
//! stops it about a radius from the crease vertex v18, leaving ~79 degree creases on the top face.
//! Each test's cases run as a loop inside it.
//!
//! Not yet asserted: that the right tip's pair corner is the mitre of its two rounds (0.519 off
//! with the patch laid on the mitre and its crease split in: the filler's patch is too coarse for
//! a 3.4 round - the next rung).

use std::collections::HashSet;
use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::BandGapBuildOptions;
use geometry_processing::bevel::level_line::{
    band_room, gap_fill, BandGapRefusal, BandGapResult, GapFillRefusal, GapFillResult,
    LevelLineWork, WorkBudget,
};
use vector_math::{Vector2, Vector3};

use crate::end_face_builds::{as_saved, build_chains, default_budget, feature_of};
use crate::fillet_thin_front_fin::profile_of;
use crate::level_line_fixture::LevelLineFixture;
use crate::mesh_surface_distance::closest_point_on_triangle;
use crate::sweep_mesh_measures;

/// This fixture's frame is the app's shifted by (-20, -118.5, 0).
const TO_APP: Vector3 = Vector3::new(20.0, 118.5, 0.0);

/// At sizes 3.4 and 3.0, no output vertex below the top
/// face near the crease stands past the right slope's plane. Measured (both sizes build and fill)
/// without the end clip (`square_crease_end_clip`): 2.708 past it at 3.0 and
/// 3.084 at 3.4, among them app (10.696, 10.477, 18.961), the vertex the app test read as the G2
/// run-out's worst (0.829 from the control).
#[test]
fn the_crease_end_stops_in_the_right_slopes_plane() {
    for size in [3.4, 3.0] {
        // The clip is switched off for this one build: the switch rides on the build's and the
        // fill's budgets, so no other test sees it.
        let unclipped = past_the_plane(size, true);
        assert!(unclipped > 2.5, "{size}: the curl, without the clip");

        let worst = past_the_plane(size, false);
        assert!(
            worst <= 1e-3,
            "{size}: the round stands {worst:.3} past the right slope's plane"
        );
    }
}

/// The whole pick at the saved 3.4 fills
/// without intersecting itself where the source does not - the check the app's level-line route
/// makes before it keeps a build. Failing it, the app quietly takes the existing Surface
/// construction instead.
///
/// Asked at 3.4 only, the size the app builds. The same check reads true on builds the app's route
/// has not been checked on: the whole pick at 3.0, the right tip's chain alone at 2.5 and 3.4, and
/// 15mm Bevel Top Edge's edge alone at every size from 1 to 6.59.
#[test]
fn at_the_saved_size_the_whole_pick_does_not_intersect_itself() {
    let fixture = Rc::new(LevelLineFixture::load("bevel-to-center-feature-r34"));
    let all: Vec<usize> = (0..fixture.chains.len()).collect();
    let gap = build_chains(&fixture, &all, fixture.size, &as_saved(&fixture, None));
    let filled =
        gap_fill::fill(&gap, profile_of(&fixture), fixture.segments).expect("the fill runs");
    assert_eq!(filled.refusal, GapFillRefusal::None);
    let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
    let kept = sweep_mesh_measures::kept(&mesh);
    assert!(
        !sweep_mesh_measures::intersects_itself_where_the_source_did_not(
            mesh.vertices(),
            &kept,
            &fixture.positions,
            &fixture.indices
        ),
        "the app would drop the build for the existing construction"
    );
}

/// At sizes 1, 3.4, 4, 5.5, 6.3 and 6.59: "15mm Bevel Top
/// Edge" is the same part with the same edge picked alone, and the clipped end fills at every size
/// up to the file's 6.59. Read SAME sense, the corners where the rails meet the plane are made
/// vertices on the plane's edges (`EndClipCorners`) and the end is capped in the plane. Read
/// opposite sense, with no corners, the level line ran on along the right slope's top edge and the
/// fill refused FillWouldNotClose at 3.4, 4, 5.5 to 6.3 and ArcSinksIntoTheSolid at 6.59.
#[test]
fn the_edge_alone_fills_with_the_clip_at_every_size() {
    let fixture = Rc::new(LevelLineFixture::load("fifteen-mm-top-edge-r659"));
    for size in [1.0, 3.4, 4.0, 5.5, 6.3, 6.59] {
        let gap = build_edge_alone(&fixture, size);
        assert_eq!(
            gap.refusal,
            BandGapRefusal::None,
            "{size}: room {} set by {:?}",
            gap.room,
            gap.room_set_by
        );
        let filled =
            gap_fill::fill(&gap, profile_of(&fixture), fixture.segments).expect("the fill runs");
        assert_eq!(
            filled.refusal,
            GapFillRefusal::None,
            "{size}: {}",
            describe(&filled)
        );
        let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
        let kept = sweep_mesh_measures::kept(&mesh);
        assert!(sweep_mesh_measures::is_closed_and_consistent(
            mesh.vertices(),
            &kept
        ));
        assert_eq!(
            sweep_mesh_measures::shell_count(mesh.vertex_count(), &kept),
            1
        );
    }
}

/// Past the crease's length below the end vertex
/// (6.667, down to the nose) the section's foot leaves the crease, and the fill refused
/// ArcSinksIntoTheSolid at 7.0. The room reads the crease's length, so 7.0 is refused by size and
/// the reading offered to a drag names the same number, to a float's noise.
#[test]
fn past_the_crease_the_edge_alone_is_refused_by_size() {
    let fixture = Rc::new(LevelLineFixture::load("fifteen-mm-top-edge-r659"));
    let crease_length = (fixture.positions[18] - fixture.positions[8]).length();

    let refused = build_edge_alone(&fixture, 7.0);
    assert_eq!(refused.refusal, BandGapRefusal::Size);
    assert!(refused.room <= crease_length && refused.room > crease_length - 1e-6);

    let read = band_room::read_room_of_one(
        &fixture.positions,
        &fixture.indices,
        &fixture.chains[0],
        fixture.closed[0],
        Some(feature_of(&fixture)),
        profile_of(&fixture),
        1e-9,
        &Rc::new(WorkBudget::new(i64::MAX).expect("a positive limit")),
        0.0,
        20.0,
    )
    .expect("the room reads");
    // The reading offered to a drag reads the nose's vertex at the crease's foot through the
    // unclipped ball end, a float's noise under the crease's length.
    assert!(read.room <= crease_length && read.room > crease_length - 1e-6);

    // The offer is floored to the row's three decimals and built before it is shown: 6.666 builds
    // and fills. At the crease's exact length the section's foot stands on the nose's vertex, and
    // the build's own reading at that size names the vertex a float's noise nearer (6.6666658).
    let at_the_offer = build_edge_alone(&fixture, (refused.room * 1000.0).floor() / 1000.0);
    assert_eq!(at_the_offer.refusal, BandGapRefusal::None);
    assert_eq!(
        gap_fill::fill(&at_the_offer, profile_of(&fixture), fixture.segments)
            .expect("the fill runs")
            .refusal,
        GapFillRefusal::None
    );
}

/// At sizes 3.4 and 3.0, each knife tip's round reaches the BOTTOM face - the midpoint of the
/// round's bottom arc, `size` from the tip's axis toward the tip, is on the output surface, as it
/// is on the Onshape control. The corner fill's rows at a free end (`corner_fill_free_ends`) carry
/// a station on each rail wherever the other has one, so the last row lies in the bottom plane.
#[test]
fn each_tips_round_reaches_the_bottom_face() {
    let fixture = Rc::new(LevelLineFixture::load("bevel-to-center-feature-r34"));
    let all: Vec<usize> = (0..fixture.chains.len()).collect();
    for size in [3.4, 3.0] {
        let gap = build_chains(&fixture, &all, size, &as_saved(&fixture, None));
        assert_eq!(gap.refusal, BandGapRefusal::None, "{size}");
        let filled =
            gap_fill::fill(&gap, profile_of(&fixture), fixture.segments).expect("the fill runs");
        assert_eq!(
            filled.refusal,
            GapFillRefusal::None,
            "{size}: {}",
            describe(&filled)
        );
        let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
        let kept = sweep_mesh_measures::kept(&mesh);
        let p = mesh.vertices();

        // The tip's axis stands size in from the x face and size off the slope: (size, size (1 +
        // sqrt 2)) on the left.
        let axis_y = size * (1.0 + 2f64.sqrt());
        let to_the_tip = Vector2::new(-size, -axis_y).get_normal();
        for (axis, mirror) in [
            (Vector2::new(size, axis_y), 1.0),
            (Vector2::new(20.0 - size, axis_y), -1.0),
        ] {
            let mid = axis + (Vector2::new(to_the_tip.x * mirror, to_the_tip.y) * size);
            let q = Vector3::new(mid.x, mid.y, 0.0) - TO_APP;
            let best = kept
                .iter()
                .map(|&(a, b, c)| {
                    (closest_point_on_triangle(q, p[a as usize], p[b as usize], p[c as usize]) - q)
                        .length()
                })
                .fold(f64::INFINITY, f64::min);
            assert!(
                best <= 0.05,
                "{size}: the round's bottom arc midpoint {:?} is {best:.3} off the output",
                q + TO_APP
            );
        }
    }
}

/// The edge-alone build of `fifteen-mm-top-edge-r659`: no chord tolerance, a 1e-9 snap, sharp
/// turns from 20 degrees, and the fixture's own fillet segments.
fn build_edge_alone(fixture: &Rc<LevelLineFixture>, size: f64) -> BandGapResult {
    let all: Vec<usize> = (0..fixture.chains.len()).collect();
    build_chains(
        fixture,
        &all,
        size,
        &BandGapBuildOptions {
            chord_tolerance: 0.0,
            snap: 1e-9,
            sharp_turn_degrees: 20.0,
            fillet_segments: fixture.segments,
            ..BandGapBuildOptions::default()
        },
    )
}

/// The refusal message: the refusal, the offending vertex and the open edge's first end, in the
/// app's frame, so a failure names where to look in the app.
fn describe(filled: &GapFillResult) -> String {
    let at = |v: i32| match &filled.mesh {
        Some(mesh) if v >= 0 => format!("{:?}", mesh.borrow().vertices()[v as usize] + TO_APP),
        _ => "default".to_string(),
    };
    format!(
        "{:?} at {}, open {}",
        filled.refusal,
        at(filled.offending_vertex),
        at(filled.open_edge.0)
    )
}

/// How far past the right slope's plane the filled whole pick's farthest vertex
/// under the top face near the crease stands. `clips_off` turns the square-crease end clip off
/// for this call only, carried on the build's and the fill's budgets.
fn past_the_plane(size: f64, clips_off: bool) -> f64 {
    let fixture = Rc::new(LevelLineFixture::load("bevel-to-center-feature-r34"));
    let pick = [0usize, 1, 2, 3];
    let budget = default_budget(&fixture, false);
    budget.square_crease_clips_off.set(clips_off);
    let gap = {
        let _work = LevelLineWork::begin(Some(Rc::clone(&budget)));
        build_chains(
            &fixture,
            &pick,
            size,
            &BandGapBuildOptions {
                budget: Some(budget),
                ..as_saved(&fixture, None)
            },
        )
    };
    assert_eq!(
        gap.refusal,
        BandGapRefusal::None,
        "room {} set by {:?}",
        gap.room,
        gap.room_set_by
    );
    let triangle_count = gap.mesh.as_ref().map_or(0, |m| m.borrow().triangle_count());
    let fill_budget = Rc::new(
        WorkBudget::new(WorkBudget::default_limit(triangle_count)).expect("a positive limit"),
    );
    fill_budget.square_crease_clips_off.set(clips_off);
    let filled = gap_fill::fill_with_budget(
        &gap,
        profile_of(&fixture),
        fixture.segments,
        Some(fill_budget),
    )
    .expect("the fill runs");
    assert_eq!(
        filled.refusal,
        GapFillRefusal::None,
        "{}",
        describe(&filled)
    );
    let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
    let kept = sweep_mesh_measures::kept(&mesh);
    assert!(sweep_mesh_measures::is_closed_and_consistent(
        mesh.vertices(),
        &kept
    ));
    assert_eq!(
        sweep_mesh_measures::shell_count(mesh.vertex_count(), &kept),
        1
    );

    let mut seen = HashSet::new();
    kept.iter()
        .flat_map(|&(a, b, c)| [a, b, c])
        .filter(|&v| seen.insert(v))
        .map(|v| mesh.vertices()[v as usize] + TO_APP)
        .filter(|v| {
            v.z < 20.0 - 1e-6
                && v.z > 15.0
                && (Vector2::new(v.x, v.y) - Vector2::new(10.0, 10.0)).length() < 5.0
        })
        .map(|v| v.x + v.y - 20.0)
        .fold(f64::NEG_INFINITY, f64::max)
}
