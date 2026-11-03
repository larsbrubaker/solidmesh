//! A chain in a pick of several open chains flows over a narrow end face exactly as it does alone.
//! On "Fillet Should go to Sides and Stay Within Bounds.mcx" the chain c13 (vertices 1 to 6) ends
//! on the 0.032 sliver end face y = 95; alone, its band at 0.05 flows past the sliver onto the next
//! faces (`narrow_face_flow::tag_flowed_where_it_builds`) and builds, carrying the room it had
//! without the flow. Picked with the far chain c27 (vertices 9 to 10), whose band never meets it,
//! the pick has to read and build c13 the same way.

use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::{BandGapResult, BandProfile, WorkBudget};

use crate::level_line_fixture::LevelLineFixture;

fn build(fixture: &Rc<LevelLineFixture>, chains: &[Vec<i32>], size: f64) -> BandGapResult {
    build_handed(fixture, chains, size, None)
}

fn build_handed(
    fixture: &Rc<LevelLineFixture>,
    chains: &[Vec<i32>],
    size: f64,
    known_room: Option<f64>,
) -> BandGapResult {
    let options = BandGapBuildOptions {
        chord_tolerance: 0.0,
        snap: 1e-9,
        known_room,
        budget: Some(Rc::new(
            WorkBudget::new(WorkBudget::default_limit(fixture.indices.len() as i32))
                .expect("a positive limit"),
        )),
        seam_epsilon: fixture.seam_epsilon,
        sharp_turn_degrees: fixture.sharp_turn_degrees,
        fillet_segments: fixture.segments,
    };
    let features = Rc::clone(fixture);
    band_gap_entry_points::build_pick(
        &fixture.positions,
        &fixture.indices,
        chains,
        &vec![false; chains.len()],
        Some(Rc::new(move |a, b| features.is_feature_edge(a, b))),
        size,
        BandProfile::Fillet,
        &options,
    )
    .expect("the pick is well formed")
}

/// Alone, c13 flows over its sliver end face and builds, remembering its room without the flow.
#[test]
fn the_chain_alone_flows_over_its_sliver() {
    let fixture = Rc::new(LevelLineFixture::load("goes-to-sides-c13-r005"));
    let gap = build(&fixture, &[vec![1, 6]], 0.05);
    assert!(gap.built(), "band {:?}, room {}", gap.refusal, gap.room);
    assert!(gap.unflowed_room.is_some_and(|room| room < 0.05));
}

/// Picked with c27, c13 flows over its sliver just the same: the pick builds at 0.05, carrying the
/// room c13 has alone without the flow.
#[test]
fn the_chain_in_a_pick_flows_over_its_sliver_as_it_does_alone() {
    let fixture = Rc::new(LevelLineFixture::load("goes-to-sides-c13-r005"));
    let alone = build(&fixture, &[vec![1, 6]], 0.05);
    let pick = build(&fixture, &[vec![1, 6], vec![9, 10]], 0.05);
    assert!(
        pick.built(),
        "band {:?}, room {}, set by {:?}",
        pick.refusal,
        pick.room,
        pick.room_set_by
    );
    assert_eq!(pick.unflowed_room, alone.unflowed_room);
}

/// Handed the room the app reads first (the plain one, which refuses the size), the pick still
/// builds by the flow, as one open chain handed its room does, and carries that handed room.
#[test]
fn a_pick_handed_a_room_below_the_size_builds_by_the_flow() {
    let fixture = Rc::new(LevelLineFixture::load("goes-to-sides-c13-r005"));
    let chains = [vec![1, 6], vec![9, 10]];
    let plain = build(&fixture, &chains, 0.05)
        .unflowed_room
        .expect("the pick flows at 0.05");
    let pick = build_handed(&fixture, &chains, 0.05, Some(plain));
    assert!(
        pick.built(),
        "band {:?}, room {}, set by {:?}",
        pick.refusal,
        pick.room,
        pick.room_set_by
    );
    assert_eq!(pick.unflowed_room, Some(plain));
}

/// Handed a room below the size where no flow holds it either (the flowed room is 11.2), the pick
/// refuses by size naming the handed room, as it does with no open chain to flow.
#[test]
fn a_pick_handed_a_room_no_flow_holds_refuses_with_it() {
    let fixture = Rc::new(LevelLineFixture::load("goes-to-sides-c13-r005"));
    let chains = [vec![1, 6], vec![9, 10]];
    let pick = build_handed(&fixture, &chains, 12.0, Some(0.0316));
    assert_eq!(
        pick.refusal,
        geometry_processing::bevel::level_line::BandGapRefusal::Size
    );
    assert_eq!(pick.room, 0.0316);
}

/// On demand: what a drag above the handed-in room costs a pick of two open chains.
#[test]
#[ignore = "on demand: a measurement that asserts nothing"]
fn probe_the_drag_cost_above_a_handed_room() {
    let fixture = Rc::new(LevelLineFixture::load("goes-to-sides-c13-r005"));
    let chains = [vec![1, 6], vec![9, 10]];
    for size in [0.05, 12.0] {
        let start = std::time::Instant::now();
        let mut refusals = Vec::new();
        for _ in 0..20 {
            refusals.push(build_handed(&fixture, &chains, size, Some(0.0316)).refusal);
        }
        eprintln!(
            "DRAG size {size}: {:?} per build, {:?}",
            start.elapsed() / 20,
            refusals[0]
        );
    }
}
