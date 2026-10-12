//! Port of `LevelLine/NameEveryTightChainTests.cs`: a pick of several chains says in ONE reading
//! which of them cannot be built at a size, so the app drops them together instead of learning one
//! per rebuild. What a user sees is unchanged - every chain that fits builds, every one that does
//! not is named with its reason - and only the number of passes moves. C# `[Arguments]` cases are
//! one test each, suffixed with the argument.

use std::collections::HashSet;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::band_gap_reading::read_room_of_pick;
use geometry_processing::bevel::level_line::pick_validation;
use geometry_processing::bevel::level_line::{
    BandGapRefusal, BandProfile, ChainsThatCannotFit, RoomCandidate, RoomReading, RoomReadingKind,
    RoomSource,
};
use vector_math::Vector3;

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_test_geometry::cube_solid;
use crate::several_chains_fixtures::{self, FOLD_DEGREES};

/// C#'s `BandGap.ReadRoom(positions, indices, chains, closed, feature, BandProfile.Chamfer, 1e-9,
/// null)`, every chain closed.
fn read_room(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
) -> RoomReading {
    let feature = feature_edges_over(positions, indices, FOLD_DEGREES);
    read_room_of_pick(
        positions,
        indices,
        chains,
        &vec![true; chains.len()],
        Some(feature),
        BandProfile::Chamfer,
        1e-9,
        None,
        f64::NAN,
        0.0,
        f64::INFINITY,
    )
    .expect("a valid pick")
}

/// Every pair of chains that touch, not merely the first: three chains where 0-1 and 1-2 meet name
/// both pairs and the vertex each meets at.
#[test]
fn every_touching_pair_is_named_at_once() {
    let chains = vec![vec![4, 5, 6, 7], vec![4, 0], vec![0, 1]];

    let pairs = pick_validation::every_touching_pair(&chains);

    assert_eq!(pairs.len(), 2);
    assert_eq!(pairs[0], (0, 1, 4));
    assert_eq!(pairs[1], (1, 2, 0));
}

/// Giving the same three chains in another order permutes the INDICES and changes nothing else:
/// the same two pairs, meeting at the same two vertices.
#[test]
fn the_order_of_the_chains_permutes_the_indices_and_nothing_else() {
    let chains = vec![vec![0, 1], vec![4, 0], vec![4, 5, 6, 7]];

    let pairs = pick_validation::every_touching_pair(&chains);

    assert_eq!(pairs.len(), 2);
    assert_eq!(pairs[0], (0, 1, 0));
    assert_eq!(pairs[1], (1, 2, 4));
}

/// A pick whose chains stand apart names no pair at all.
#[test]
fn chains_that_stand_apart_name_no_pair() {
    let (_, _, chains) = several_chains_fixtures::named("A");

    assert_eq!(pick_validation::every_touching_pair(&chains).len(), 0);
}

/// Reading every pair does not change what the build refuses: the FIRST pair of the list that is
/// not a CORNER is, bit for bit, the pair `BandGapRefusal::ChainsTouch` names. Here the rim and 4-0
/// meet at the corner 4, which is built rather than refused; 4-0 and 0-1 meet end to end at 0 with
/// only two picked edges there.
#[test]
fn the_first_pair_is_still_what_the_refusal_names() {
    let (positions, indices) = cube_solid(1.0);
    let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);
    let chains = vec![vec![4, 5, 6, 7], vec![4, 0], vec![0, 1]];
    let closed = [true, false, false];

    let built = band_gap_entry_points::build_pick(
        &positions,
        &indices,
        &chains,
        &closed,
        Some(feature),
        0.1,
        BandProfile::Chamfer,
        &BandGapBuildOptions::default(),
    )
    .expect("the pick builds or refuses by name");
    let corner_vertices: HashSet<i32> = pick_validation::corners_of(&chains, &closed, false)
        .iter()
        .map(|c| c.vertex)
        .collect();
    let first = pick_validation::every_touching_pair(&chains)
        .into_iter()
        .find(|p| !corner_vertices.contains(&p.2))
        .expect("a pair that is not a corner");
    assert_eq!(first, (1, 2, 0));

    assert_eq!(built.refusal, BandGapRefusal::ChainsTouch);
    assert_eq!(
        (
            built.offending_chain,
            built.other_chain,
            built.offending_vertex
        ),
        first
    );
}

/// Fixture C - a hole 0.3 from the plate's rim, the pick's room set by the two chains closing on
/// each other - at a size over the PAIR's room but under what either chain has alone names exactly
/// ONE chain, and the survivor really does fit: its own reading stands at or above the size and it
/// builds.
#[test]
fn a_pair_too_tight_for_both_drops_one_and_the_survivor_builds() {
    let (positions, indices, chains) = several_chains_fixtures::named("C");
    let reading = read_room(&positions, &indices, &chains);

    // Between the pair's room (the 0.3 gap shared between two bands) and the 0.3 one band alone
    // reaches.
    let size = 0.25;
    assert!(
        reading.room < size,
        "the pick as a whole does not fit at this size: {}",
        reading.room
    );

    let cannot_fit = ChainsThatCannotFit::read(&reading, size);

    assert_eq!(
        cannot_fit.dropped.len(),
        1,
        "dropping one of the two gives the other the whole gap"
    );
    assert_eq!(cannot_fit.unresolved.len(), 0);
    assert_eq!(
        cannot_fit.dropped[0].because.kind,
        RoomReadingKind::ChainToChain
    );

    let survivors: Vec<Vec<i32>> = chains
        .iter()
        .enumerate()
        .filter(|&(k, _)| k as i32 != cannot_fit.dropped[0].chain)
        .map(|(_, c)| c.clone())
        .collect();
    let of_survivors = read_room(&positions, &indices, &survivors);
    assert!(
        of_survivors.room >= size,
        "what is left must really fit, or the pass was wasted: {}",
        of_survivors.room
    );

    let built = band_gap_entry_points::build_pick(
        &positions,
        &indices,
        &survivors,
        &vec![true; survivors.len()],
        Some(feature_edges_over(&positions, &indices, FOLD_DEGREES)),
        size,
        BandProfile::Chamfer,
        &BandGapBuildOptions::default(),
    )
    .expect("the survivors build or refuse by name");
    assert_eq!(built.refusal, BandGapRefusal::None);
}

/// A size the whole pick has room for names nobody.
fn at_a_size_every_chain_fits_it_names_none(fixture: &str) {
    let (positions, indices, chains) = several_chains_fixtures::named(fixture);
    let reading = read_room(&positions, &indices, &chains);

    let cannot_fit = ChainsThatCannotFit::read(&reading, reading.room);

    assert!(
        cannot_fit.every_chain_fits(),
        "at the pick's own room every chain fits, so nothing may be blamed"
    );
}

#[test]
fn at_a_size_every_chain_fits_it_names_none_a() {
    at_a_size_every_chain_fits_it_names_none("A");
}

#[test]
fn at_a_size_every_chain_fits_it_names_none_b() {
    at_a_size_every_chain_fits_it_names_none("B");
}

#[test]
fn at_a_size_every_chain_fits_it_names_none_c() {
    at_a_size_every_chain_fits_it_names_none("C");
}

/// A size no chain of the pick can take names them ALL, each with the reading behind it - and not
/// because the pick's least room is under the size, but because each chain's own reading is.
#[test]
fn at_a_size_no_chain_fits_it_names_them_all_with_their_reasons() {
    let (positions, indices, chains) = several_chains_fixtures::named("A");
    let reading = read_room(&positions, &indices, &chains);

    // The plate is 2 thick, so every chain's own foreign candidate - the wall's height - is well
    // under this.
    let cannot_fit = ChainsThatCannotFit::read(&reading, 20.0);

    assert_eq!(cannot_fit.dropped.len(), chains.len());
    for one in &cannot_fit.dropped {
        assert_ne!(one.because.kind, RoomReadingKind::None);
        assert!(one.room < 20.0);
    }
}

fn named_at(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    size: f64,
) -> Vec<i32> {
    let reading = read_room(positions, indices, chains);
    ChainsThatCannotFit::read(&reading, size)
        .dropped
        .iter()
        .map(|d| d.chain)
        .collect()
}

/// The answer is about the PART, not about how the pick was typed or where the part stands: the
/// chains given the other way round name the same chain (its index permuted with it), and so does
/// the part under a rigid motion.
#[test]
fn the_answer_does_not_move_with_chain_order_or_a_rigid_motion() {
    let (positions, indices, chains) = several_chains_fixtures::named("C");
    let size = 0.25;

    let straight = named_at(&positions, &indices, &chains, size);
    let reversed = named_at(
        &positions,
        &indices,
        &[chains[1].clone(), chains[0].clone()],
        size,
    );
    let motion = several_chains_fixtures::motion();
    let moved_positions: Vec<Vector3> = positions.iter().map(|p| p.transform(motion)).collect();
    let moved = named_at(&moved_positions, &indices, &chains, size);

    assert_eq!(reversed.len(), straight.len());
    assert_eq!(moved.len(), straight.len());
    assert_eq!(
        moved, straight,
        "a part that is merely moved names the same chain"
    );

    // The HOLE goes and the plate's rim stays, whichever order the two were given in: either could
    // have stayed alone, and the shorter chain is the one dropped - a reading of the part, not of
    // the order.
    assert_eq!(straight[0], 1);
    assert_eq!(
        reversed[0], 0,
        "reversed, the hole is chain 0 - the same chain, at the index this order gave it"
    );
}

/// A ChainToChain row with its two optional rooms (C#'s positional record arguments).
fn pair_row(chain: i32, other: i32, room: f64, without: f64, at_most: f64) -> RoomCandidate {
    RoomCandidate {
        room_without_the_other: without,
        room_without_the_other_at_most: at_most,
        ..RoomCandidate::new(RoomSource::chain_to_chain(chain, other), room)
    }
}

/// A Self row naming its group's members.
fn self_row(chain: i32, room: f64, members: &[i32]) -> RoomCandidate {
    RoomCandidate {
        group_members: Some(members.to_vec()),
        ..RoomCandidate::new(RoomSource::self_room(chain), room)
    }
}

/// When only ONE of a tight pair could survive alone, that is the one that stays - the tie-break
/// never gets a say, and the pair is never dropped whole.
#[test]
fn when_only_one_of_a_pair_can_survive_the_other_is_the_one_dropped() {
    let reading = RoomReading {
        read: true,
        room: 0.1,
        room_set_by: RoomSource::chain_to_chain(0, 1),
        candidates: vec![
            RoomCandidate::new(RoomSource::foreign(0), 5.0),
            RoomCandidate::new(RoomSource::foreign(1), 5.0),
            // Chain 0 is still short of the size with chain 1 gone, and that is KNOWN (both
            // setbacks uniform, so the over-read is the same number); chain 1 is clear without
            // chain 0.
            pair_row(0, 1, 0.1, 0.15, 0.15),
            pair_row(1, 0, 0.1, 0.9, 0.9),
            self_row(0, 5.0, &[0, 1]),
        ],
        ..Default::default()
    };

    let cannot_fit = ChainsThatCannotFit::read(&reading, 0.5);

    assert_eq!(cannot_fit.dropped.len(), 1);
    assert_eq!(cannot_fit.dropped[0].chain, 0);
}

/// A group of SEVERAL chains whose own self reading is under the size cannot be blamed from one
/// reading, so it comes back whole and the caller falls back to today's one-at-a-time blame for
/// it. Nothing is guessed and the group is not dropped together.
#[test]
fn a_group_of_several_that_runs_out_of_itself_comes_back_unresolved() {
    let reading = RoomReading {
        read: true,
        room: 0.2,
        room_set_by: RoomSource::self_room(0),
        candidates: vec![
            RoomCandidate::new(RoomSource::foreign(0), 5.0),
            RoomCandidate::new(RoomSource::foreign(1), 5.0),
            RoomCandidate::new(RoomSource::foreign(2), 5.0),
            self_row(0, 0.2, &[0, 1]),
            // A group of one CAN be blamed: that chain is the only thing in it.
            self_row(2, 0.2, &[2]),
        ],
        ..Default::default()
    };

    let cannot_fit = ChainsThatCannotFit::read(&reading, 0.5);

    assert_eq!(cannot_fit.dropped.len(), 1);
    assert_eq!(cannot_fit.dropped[0].chain, 2);
    assert_eq!(cannot_fit.unresolved.len(), 1);
    assert_eq!(cannot_fit.unresolved[0], vec![0, 1]);
}

/// A pair whose setbacks VARY along the chains has no known over-read, and a chain is never
/// condemned on the under-read: both ends of that pair would go where dropping one will do, and a
/// loop that fits must be built (decision 6). Silence means "may stay", and the caller's re-read
/// decides.
#[test]
fn a_pair_whose_setbacks_vary_condemns_neither_end() {
    let reading = RoomReading {
        read: true,
        room: 0.1,
        room_set_by: RoomSource::chain_to_chain(0, 1),
        chain_lengths: vec![10.0, 4.0],
        candidates: vec![
            RoomCandidate::new(RoomSource::foreign(0), 5.0),
            RoomCandidate::new(RoomSource::foreign(1), 5.0),
            // The under-read says both ends are short of the size; the over-read is not known,
            // which is what a varying setback leaves behind.
            pair_row(0, 1, 0.1, 0.15, f64::INFINITY),
            pair_row(1, 0, 0.1, 0.15, f64::INFINITY),
        ],
        ..Default::default()
    };

    let cannot_fit = ChainsThatCannotFit::read(&reading, 0.5);

    assert_eq!(
        cannot_fit.dropped.len(),
        1,
        "one of the pair must go, but neither is CONDEMNED, so the tie-break picks one"
    );
    assert_eq!(cannot_fit.dropped[0].chain, 1, "the shorter chain goes");
}

/// A group whose every member is leaving for some other reason has nothing left to resolve, so it
/// does not come back unresolved - which would otherwise hold a caller back from building the rest.
#[test]
fn a_group_whose_every_member_is_going_is_not_unresolved() {
    let reading = RoomReading {
        read: true,
        room: 0.2,
        room_set_by: RoomSource::self_room(0),
        candidates: vec![
            RoomCandidate::new(RoomSource::foreign(0), 0.2),
            RoomCandidate::new(RoomSource::foreign(1), 0.2),
            RoomCandidate::new(RoomSource::foreign(2), 5.0),
            self_row(0, 0.2, &[0, 1]),
        ],
        ..Default::default()
    };

    let cannot_fit = ChainsThatCannotFit::read(&reading, 0.5);

    // IsEquivalentTo: the same members in any order.
    let mut dropped: Vec<i32> = cannot_fit.dropped.iter().map(|d| d.chain).collect();
    dropped.sort();
    assert_eq!(dropped, vec![0, 1]);
    assert_eq!(cannot_fit.unresolved.len(), 0);
    assert!(
        !cannot_fit.every_chain_fits(),
        "chains were named, so nobody may read this as build-now"
    );
}

/// A reading whose zero is NOT a size statement blames nobody for not fitting.
#[test]
fn a_reading_that_is_not_a_size_statement_names_nobody() {
    let reading = RoomReading {
        read: true,
        room: 0.0,
        chains_are_apart: false,
        ..Default::default()
    };

    assert!(ChainsThatCannotFit::read(&reading, 1.0).every_chain_fits());
}
