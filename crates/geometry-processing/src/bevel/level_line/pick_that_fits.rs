//! The largest part of a pick that can be built at a size ([`PickThatFits`]): read the pick, drop
//! every chain the reading names, RE-READ what is left, and go round again until a reading names
//! nobody. Each round is one pick reading
//! (`band_gap_reading::read_room_of_pick`) and one naming of it ([`ChainsThatCannotFit`]).
//!
//! The re-reading is not belt and braces. Dropping a chain RE-TAGS the sides: the dropped chain's
//! own crease, and whatever stood behind it, become feature edges FOREIGN to a survivor which had
//! no row against them while that chain was picked. So a reading of the survivors can be lower
//! than any reading of the whole pick was, and no single reading can promise that its survivors
//! fit. Measured on the five-hole plate at size 1.0: the first reading drops the outer rim and the
//! near end hole, the survivors then read 0.600 because the plate's own edge is now foreign to the
//! far end hole, and a second round names that hole.
//!
//! **It terminates.** A round either names somebody - and then the next round reads a strictly
//! smaller pick - or names nobody and is the last. So there are at most as many rounds as the pick
//! has chains, and in practice one or two: the five-hole plate settles in 2 rounds at 1.0 and at
//! 1.6.
//!
//! **It is not a promise either.** [`PickThatFits::fits`] says whether the last reading really
//! did come back at or above the size with nothing unresolved. A pick whose trouble is a side
//! group of several chains running out of ITSELF cannot be resolved by reading alone, and comes
//! back with `unresolved` filled and `fits` false for the caller to blame one chain at a time as
//! it always did.

use std::collections::HashSet;
use std::rc::Rc;

use vector_math::Vector3;

use super::band_gap_reading::{read_room_of_pick, ReadRoomError};
use super::seam_edges::FeatureEdgeTest;
use super::{
    pick_validation, BandProfile, ChainCannotFit, ChainsThatCannotFit, RoomReading,
    RoomReadingKind, RoomSource, WorkBudget,
};

/// What is left of a pick once every chain that cannot be built at the size
/// has been dropped, round after round.
#[derive(Debug, Clone)]
pub struct PickThatFits {
    /// What is left of the pick, as indices into the chains the caller gave, in their order.
    pub chains: Vec<i32>,
    /// Every chain dropped, over every round, by ascending chain index.
    pub dropped: Vec<ChainCannotFit>,
    /// Side groups the readings could not blame - see [`ChainsThatCannotFit`].
    pub unresolved: Vec<Vec<i32>>,
    /// The last reading taken, of `chains` - `None` if nothing was left to read.
    pub last: Option<RoomReading>,
    /// How many readings were taken.
    pub rounds: i32,
    /// True when what is left really does read at or above the size with nothing unresolved - the
    /// one thing a caller may check before building.
    pub fits: bool,
}

impl PickThatFits {
    /// The settled pick: `fits` is decided here, from what was kept and the last reading.
    fn new(
        chains: Vec<i32>,
        dropped: Vec<ChainCannotFit>,
        unresolved: Vec<Vec<i32>>,
        last: Option<RoomReading>,
        rounds: i32,
        size: f64,
    ) -> Self {
        let fits = !chains.is_empty()
            && unresolved.is_empty()
            && last
                .as_ref()
                .is_some_and(|last| last.is_a_size_statement() && last.room >= size);

        PickThatFits {
            chains,
            dropped,
            unresolved,
            last,
            rounds,
            fits,
        }
    }

    /// Reads the pick, drops what cannot be built at `size`, and repeats on the
    /// survivors until a reading names nobody. Takes a `budget` and honours its cancellation
    /// exactly as `read_room_of_pick` does - the budget is SHARED across the rounds, so a pick that
    /// cannot be settled inside it comes back with an unread last reading rather than spending
    /// without limit.
    ///
    /// `seam_epsilon` is the same number the BUILD is given - see `SeamEdges`. It must be, and this
    /// is the one place it can be forgotten unseen: a settling reads, condemns and its verdict is
    /// memoised as the build's known room, so a settling blind to a doubled loop drops it and pins
    /// the build to a room the build itself would never have read. Zero (the usual value) reads
    /// with no seam rule; the usual `sharp_turn_degrees` is infinity.
    #[allow(clippy::too_many_arguments)]
    pub fn read(
        positions: &[Vector3],
        indices: &[(i32, i32, i32)],
        chains: &[Vec<i32>],
        closed: &[bool],
        is_feature_edge: Option<FeatureEdgeTest>,
        profile: BandProfile,
        size: f64,
        snap: f64,
        budget: Option<Rc<WorkBudget>>,
        seam_epsilon: f64,
        sharp_turn_degrees: f64,
    ) -> Result<PickThatFits, ReadRoomError> {
        pick_validation::check_pick_arguments(positions, indices, chains, closed)?;
        let budget = match budget {
            Some(budget) => budget,
            None => Rc::new(
                WorkBudget::new(WorkBudget::default_limit(indices.len() as i32))
                    .expect("the default limit is a positive count of work"),
            ),
        };

        let mut alive: Vec<i32> = (0..chains.len() as i32).collect();
        let mut dropped: Vec<ChainCannotFit> = Vec::new();
        let mut unresolved: Vec<Vec<i32>> = Vec::new();
        let mut last: Option<RoomReading> = None;
        let mut rounds = 0;

        // At most one round per chain: every round that is not the last drops at least one chain,
        // so the next round reads a strictly smaller pick.
        while !alive.is_empty() && rounds <= chains.len() as i32 {
            let pick: Vec<Vec<i32>> = alive.iter().map(|&k| chains[k as usize].clone()).collect();
            let pick_closed: Vec<bool> = alive.iter().map(|&k| closed[k as usize]).collect();
            let reading = read_room_of_pick(
                positions,
                indices,
                &pick,
                &pick_closed,
                is_feature_edge.clone(),
                profile,
                snap,
                Some(budget.clone()),
                size,
                seam_epsilon,
                sharp_turn_degrees,
            )?;
            rounds += 1;

            if !reading.read || !reading.is_a_size_statement() {
                // The reading ran out of work, or the pick refuses by NAME - neither is a statement
                // about size and neither names a chain to drop. The caller is told what the reading
                // said.
                last = Some(reading);
                break;
            }

            let named = ChainsThatCannotFit::read(&reading, size);
            unresolved.clear();
            for group in &named.unresolved {
                unresolved.push(group.iter().map(|&local| alive[local as usize]).collect());
            }

            if named.dropped.is_empty() {
                if reading.room >= size || alive.len() > 1 {
                    // Nobody to blame: either it fits, or the trouble is one this reading cannot
                    // pin on a chain (an unresolved group), and going round again would read the
                    // same thing.
                    last = Some(reading);
                    break;
                }

                // One chain left and it does not fit. Its own reading is the whole answer, and a
                // pick of one fills no candidate table, so it is named here rather than by the
                // table walk - with what that reading says set the room, never a guess at Foreign.
                dropped.push(ChainCannotFit {
                    chain: alive[0],
                    because: in_terms_of_the_pick(reading.room_set_by, &alive),
                    room: reading.room,
                    round: rounds,
                });
                alive.clear();
                last = Some(reading);
                break;
            }

            for one in &named.dropped {
                // The reason is stated in THIS round's indices - the round reads a pick of the
                // survivors - so it is put back into the caller's own numbering along with the
                // chain it is about. Without that, a drop in round two blames whichever original
                // chain happens to sit at the survivor's position: on the five-hole plate at 1.0,
                // hole 5 was reported condemned by "chain 3", a hole three holes away that it
                // never touched.
                dropped.push(ChainCannotFit {
                    chain: alive[one.chain as usize],
                    because: in_terms_of_the_pick(one.because, &alive),
                    room: one.room,
                    round: rounds,
                });
            }

            let going: HashSet<i32> = named
                .dropped
                .iter()
                .map(|d| alive[d.chain as usize])
                .collect();
            alive.retain(|k| !going.contains(k));
            last = None;
        }

        if last.is_none() && !alive.is_empty() {
            // The last round dropped somebody and the loop ran out of rounds; read once more so
            // what comes back is a reading OF the survivors and not of a pick that no longer
            // exists.
            let pick: Vec<Vec<i32>> = alive.iter().map(|&k| chains[k as usize].clone()).collect();
            let pick_closed: Vec<bool> = alive.iter().map(|&k| closed[k as usize]).collect();
            last = Some(read_room_of_pick(
                positions,
                indices,
                &pick,
                &pick_closed,
                is_feature_edge,
                profile,
                snap,
                Some(budget),
                size,
                seam_epsilon,
                sharp_turn_degrees,
            )?);
            rounds += 1;
        }

        // `sort_by_key` is stable: drops of one chain index keep their order.
        dropped.sort_by_key(|d| d.chain);

        Ok(PickThatFits::new(
            alive, dropped, unresolved, last, rounds, size,
        ))
    }
}

/// A reason a round stated in ITS OWN chain numbering, put back into the
/// caller's. A round reads a pick of the survivors, so every chain index in a [`RoomSource`] it
/// produces - the chain and, for a pair, the other chain - is a position in `alive` and means
/// nothing to the caller until it is looked up there.
fn in_terms_of_the_pick(local: RoomSource, alive: &[i32]) -> RoomSource {
    let original = |chain: i32| {
        if chain >= 0 && (chain as usize) < alive.len() {
            alive[chain as usize]
        } else {
            -1
        }
    };

    match local.kind {
        RoomReadingKind::Foreign => RoomSource {
            chain: original(local.chain),
            ..local
        },
        RoomReadingKind::SelfRoom => RoomSource::self_room(original(local.chain)),
        RoomReadingKind::ChainToChain => {
            RoomSource::chain_to_chain(original(local.chain), original(local.other_chain))
        }
        _ => RoomSource::nothing(),
    }
}
