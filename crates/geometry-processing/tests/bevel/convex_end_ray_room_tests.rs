//! "Chain and Face Should Both work.mcx"'s chain pick c20, whose convex oblique end carries an
//! unbounded end ray that a crease past the end face lies ON. The band is clipped at the end face,
//! so that edge is read against the end vertex, and the room is set by what really bounds the band.

use std::rc::Rc;

use solidmesh::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use solidmesh::bevel::level_line::band_room_pair_room_pick;
use solidmesh::bevel::level_line::corner_cap_flat_ends;
use solidmesh::bevel::level_line::seam_edges::FeatureEdgeTest;
use solidmesh::bevel::level_line::{BandProfile, WorkBudget};

use crate::level_line_fixture::LevelLineFixture;

fn profile_of(fixture: &LevelLineFixture) -> BandProfile {
    if fixture.chamfer {
        BandProfile::Chamfer
    } else {
        BandProfile::Fillet
    }
}

fn feature_of(fixture: &Rc<LevelLineFixture>) -> FeatureEdgeTest {
    let lookup = Rc::clone(fixture);
    Rc::new(move |a, b| lookup.is_feature_edge(a, b))
}

/// Clears this thread's flat-ends override when dropped, however the test leaves.
struct FlatEndsOverride;

impl Drop for FlatEndsOverride {
    fn drop(&mut self) {
        corner_cap_flat_ends::set_use_flat_ends_override(None);
    }
}

/// Chain and face c20 builds at a small size.
#[test]
fn chain_and_face_c20_builds_at_a_small_size() {
    let fixture = Rc::new(LevelLineFixture::load("chain-and-face-c20-r005"));
    let gap = band_gap_entry_points::build_pick(
        &fixture.positions,
        &fixture.indices,
        &fixture.chains,
        &fixture.closed,
        Some(feature_of(&fixture)),
        fixture.size,
        profile_of(&fixture),
        &BandGapBuildOptions {
            chord_tolerance: 0.0,
            snap: 1e-9,
            known_room: None,
            budget: Some(Rc::new(
                WorkBudget::new(WorkBudget::default_limit(fixture.indices.len() as i32))
                    .expect("a positive limit"),
            )),
            seam_epsilon: fixture.seam_epsilon,
            sharp_turn_degrees: fixture.sharp_turn_degrees,
            fillet_segments: fixture.segments,
        },
    )
    .expect("the pick builds or refuses by name");
    println!(
        "band {:?}, room {}, set by {:?}",
        gap.refusal, gap.room, gap.room_set_by
    );
    assert!(gap.built(), "{:?}", gap.refusal);
    assert!(gap.room > 1.0, "{}", gap.room);
}

/// The same c20 chain picked with a
/// second, far edge, so the room is read by the MULTI-chain reading - which with flat ends on must
/// take the same convex-end rule.
#[test]
fn chain_and_face_c20_with_a_second_chain_reads_the_room_past_its_end_face() {
    let fixture = Rc::new(LevelLineFixture::load("chain-and-face-c20-r005"));
    let chains = vec![fixture.chains[0].clone(), vec![0, 1]];
    let closed = vec![fixture.closed[0], false];
    let picked = {
        corner_cap_flat_ends::set_use_flat_ends_override(Some(true));
        let _clear = FlatEndsOverride;
        band_room_pair_room_pick::room_of_pick(
            &fixture.positions,
            &fixture.indices,
            &chains,
            &closed,
            Some(feature_of(&fixture)),
            profile_of(&fixture),
            1e-9,
            f64::NAN,
            fixture.seam_epsilon,
            fixture.sharp_turn_degrees,
            fixture.segments,
        )
        .expect("the room reads")
    };

    assert!(
        picked.room > 1.0,
        "{}: set by {:?}",
        picked.room,
        picked.set_by
    );
}
