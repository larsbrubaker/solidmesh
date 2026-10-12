//! Port of `LevelLine/EndFaceCarryTests.cs`: plan stage 2, the end face's half (flag on) - where a
//! round ends on an END FACE, the strip is carried to that face's plane and cut into it, so the
//! crease's old corner at the end stops being a vertex of anything the fill keeps. Bottom Center
//! Edge's apex end is the app control's witness.

use std::rc::Rc;

use geometry_processing::bevel::level_line::gap_fill;
use geometry_processing::bevel::level_line::GapFillResult;
use vector_math::Vector3;

use crate::end_face_builds::{at_bend, build_chains, FlatEnds};
use crate::fillet_thin_front_fin::profile_of;
use crate::level_line_fixture::LevelLineFixture;

const BEND_DEGREES: f64 = 20.0;

/// The fixture's chain 0 built and filled with flat ends on, as both tests build it.
fn built_and_filled(fixture: &Rc<LevelLineFixture>) -> GapFillResult {
    let _flat_ends = FlatEnds::set(true);
    let gap = build_chains(fixture, &[0], fixture.size, &at_bend(BEND_DEGREES));
    assert!(gap.built(), "{:?}, room {}", gap.refusal, gap.room);
    gap_fill::fill(&gap, profile_of(fixture), fixture.segments).expect("the fill runs")
}

// The apex end is concave, bounded and same-sense, but its end face leans ACROSS the chain, so
// EndFaceFill grows it along the crease run through V instead of trimming and V stayed on 10 kept
// triangles. What is owed: the rows past t* carried on to the plane, each point stopping where it
// meets it, so the last row IS the section.
#[test]
fn bottom_center_edge_apex_leaves_the_filled_mesh() {
    let fixture = Rc::new(LevelLineFixture::load("bottom-center-edge-op0"));
    let apex = Vector3::new(-10.0, -108.5, 6.6667);
    let fill = built_and_filled(&fixture);
    assert!(fill.filled(), "{:?}", fill.refusal);
    let mesh = fill.mesh.as_ref().expect("the filled mesh").borrow();
    let on_apex = (0..mesh.triangle_count())
        .filter(|&t| !mesh.is_removed(t))
        .filter(|&t| {
            let tri = mesh.triangles()[t as usize];
            [tri.a, tri.b, tri.c]
                .iter()
                .any(|&c| (mesh.position(c) - apex).length() < 1e-3)
        })
        .count();
    let chain = &fixture.chains[0];
    println!(
        "chain ends {:?} .. {:?}; kept triangles on the apex {on_apex}",
        fixture.positions[chain[0] as usize],
        fixture.positions[*chain.last().expect("a chain") as usize]
    );
    assert_eq!(on_apex, 0);
}

// The app control's worst two vertices were arc points of the FAN at the ridge-tip end (chain
// vertex (-5, -113.5, 10)). The control's shape past the run-out is the round cut by the nose
// face's plane: every row point there is on the round or in that plane
// (OpenGapFill.CutSectionArc).
#[test]
fn bottom_center_edge_rows_near_the_ridge_tip_lie_on_the_round() {
    let fixture = Rc::new(LevelLineFixture::load("bottom-center-edge-op0"));
    let fill = built_and_filled(&fixture);
    assert!(fill.filled(), "{:?}", fill.refusal);

    // The concave round's axis: r from both faces on the crease, on their open side.
    let chain = &fixture.chains[0];
    let first = chain[0];
    let last = *chain.last().expect("a chain");
    let start = fixture.positions[first as usize];
    let end = fixture.positions[last as usize];
    let along = (end - start).get_normal();
    let at = |v: i32| fixture.positions[v as usize];
    let normals: Vec<Vector3> = fixture
        .indices
        .iter()
        .filter(|&&(a, b, c)| [a, b, c].contains(&first) && [a, b, c].contains(&last))
        .map(|&(a, b, c)| (at(b) - at(a)).cross(at(c) - at(a)).get_normal())
        .collect();
    let r = fixture.size;
    let axis = start + ((normals[0] + normals[1]) * (r / (1.0 + normals[0].dot(normals[1]))));

    // The nose face the wall-side face runs out onto: past the run-out the round is cut by its
    // plane, so a point there is on the round or in that plane.
    let nose = Vector3::new(0.0, -2.0, 3.0).get_normal();

    // Every row point past the run-out (the wall-side face ends on the ridge at t 5.066).
    let mesh = fill.mesh.as_ref().expect("the filled mesh").borrow();
    let mut distinct: Vec<i32> = Vec::new();
    for &v in &fill.added_vertices {
        if !distinct.contains(&v) {
            distinct.push(v);
        }
    }
    let farthest = distinct
        .iter()
        .filter(|&&v| (mesh.position(v) - start).dot(along) > 5.1)
        .map(|&v| {
            let offset = mesh.position(v) - axis;
            let off_round = ((offset - (along * offset.dot(along))).length() - r).abs();
            f64::min(off_round, nose.dot(mesh.position(v) - end).abs())
        })
        .fold(None, |most: Option<f64>, x| {
            Some(most.map_or(x, |m| m.max(x)))
        })
        .unwrap_or(f64::MAX);
    println!("farthest row point past the run-out from the round cut by the nose plane {farthest}");
    assert!(farthest < 1e-6, "{farthest}");
}
