//! Who has to be dropped from a pick of several chains for the REST of it to build at a size,
//! read from ONE [`RoomReading`] ([`ChainsThatCannotFit`], [`ChainCannotFit`]). Ports
//! `Bevel/LevelLine/ChainsThatCannotFit.cs`. It reads the candidates the pick's room reading
//! gathered ([`RoomCandidate`], `band_gap_reading::read_room_of_pick`); `PickThatFits` is the
//! read-drop-re-read loop around it.

// C#'s negated comparisons (`!(room < size)`, `!(x > 0)`) are kept as written: they are what
// decides which way a NaN reading falls.
#![allow(clippy::neg_cmp_op_on_partial_ord)]

use std::cmp::Ordering;
use std::collections::{BTreeMap, HashMap, HashSet};

use super::{RoomReading, RoomReadingKind, RoomSource};

/// C# `ChainCannotFit`: one chain the pick cannot keep at the size asked, and the reading that
/// says so.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChainCannotFit {
    /// The chain's index in the pick.
    pub chain: i32,
    /// Which reading named it - see [`RoomSource`].
    pub because: RoomSource,
    /// That reading's room, so an advisory can say the number as well as the cause.
    pub room: f64,
    /// Which reading condemned it, counting from 1. A chain named in round 1 was too tight for the
    /// pick as the user picked it; a chain named later was condemned by the RE-TAGGING that
    /// dropping an earlier chain caused - its neighbour's crease became a foreign edge - which is a
    /// different thing to tell a user, and a different thing to read when the numbers look
    /// surprising. See `PickThatFits`.
    pub round: i32,
}

impl ChainCannotFit {
    /// C#'s constructor with its default round of 1.
    pub fn new(chain: i32, because: RoomSource, room: f64) -> Self {
        ChainCannotFit {
            chain,
            because,
            room,
            round: 1,
        }
    }
}

/// C# `ChainsThatCannotFit`: who has to be dropped from a pick of several chains for the REST of
/// it to build at a size, read from ONE [`RoomReading`] rather than one chain per rebuild.
///
/// A fifty-loop face rim at a size two of its loops cannot take used to cost a pass each: the
/// build refused, the caller dropped the one chain the refusal named, and the whole pick was read
/// and built again. The reading already holds every chain's Foreign candidate, every ordered
/// ChainToChain pair and every group's Self candidate (`RoomReading::candidates`, the margin
/// already off), which is enough to name all of them together.
///
/// **What a user sees does not change.** Every chain that fits still builds, and each one that
/// does not is still named with the reading and the number behind it. Only the number of passes
/// changes.
///
/// **THIS IS ONE READING, NOT THE ANSWER.** What comes back is every chain THIS reading can
/// condemn. The survivors are not thereby known to fit, and the caller must re-read them and ask
/// again until a reading names nobody - `PickThatFits` is that loop. The reason is not the pair
/// prune: it is that dropping a chain RE-TAGS the sides. The dropped chain's own crease, and
/// whatever stood behind it, become newly FOREIGN to a survivor that never had a row against them,
/// so a reading of the survivors can be lower than any reading of the whole pick was. Measured on
/// the five-hole plate at size 1.0: one reading drops the outer rim and the near end hole, and the
/// survivors then read 0.600, which condemns the far end hole that nothing had named.
///
/// **What one reading cannot say at all.** A group of several chains whose SELF reading is under
/// the size has run out of its own material somewhere between them, and nothing in the reading
/// says which of them is at fault - that group comes back in [`ChainsThatCannotFit::unresolved`]
/// and the caller falls back to blaming one chain at a time there.
/// [`ChainsThatCannotFit::every_chain_fits`] is false while anything is unresolved, so no caller
/// can read "nobody was named" as "build now".
#[derive(Debug, Clone, PartialEq)]
pub struct ChainsThatCannotFit {
    /// The chains to drop, by ascending chain index, each with the reading that condemned it.
    pub dropped: Vec<ChainCannotFit>,
    /// Side groups of several chains whose own Self reading is under the size. One reading cannot
    /// say which member is at fault, so they are handed back whole rather than guessed at or
    /// dropped together.
    pub unresolved: Vec<Vec<i32>>,
}

/// C#'s `Comparer<double>.Default`, which `OrderBy` sorts by: NaN equals NaN and sorts below
/// every number.
fn compare_like_csharp(a: f64, b: f64) -> Ordering {
    match (a.is_nan(), b.is_nan()) {
        (true, true) => Ordering::Equal,
        (true, false) => Ordering::Less,
        (false, true) => Ordering::Greater,
        (false, false) => a.partial_cmp(&b).unwrap_or(Ordering::Equal),
    }
}

impl ChainsThatCannotFit {
    /// True when THIS reading named nobody and left nothing unresolved. It is not a licence to
    /// build: a pick may need several rounds of read-drop-re-read before one reading says this,
    /// and it is `PickThatFits.Fits` - which also checks that the last reading's room really
    /// reaches the size - that a caller asks before building.
    pub fn every_chain_fits(&self) -> bool {
        self.dropped.is_empty() && self.unresolved.is_empty()
    }

    /// C# `Read`: reads a pick's room table for the chains that cannot stay at `size`.
    ///
    /// Three rules, in this order, so the answer is the same whatever order the candidates were
    /// read in:
    /// 1. A chain whose FOREIGN candidate is under the size goes - certain, and nothing another
    ///    chain leaving can give it back.
    /// 2. A SELF candidate under the size names its group's one chain, or the whole group as
    ///    unresolved.
    /// 3. A CHAIN TO CHAIN pair under the size must lose one of its two. If k would still be under
    ///    the size with j gone (`RoomCandidate::room_without_the_other`) then k goes whoever else
    ///    does, and likewise j; if only one of the two must go it is the one that goes; if either
    ///    may stay, the SHORTER chain goes (`RoomReading::chain_lengths`) - build as much as
    ///    possible - and never both. A pair one of whose chains is already going is already
    ///    answered and takes nobody with it, so no chain is dropped whose every tight partner is
    ///    leaving anyway.
    ///
    /// The pairs are walked tightest first, ties by the longer of their two chains, so which of
    /// several interlocking pairs is settled first is a statement about the part. Nothing here
    /// reads a chain INDEX except as the last resort between two chains of exactly equal length,
    /// which is a genuinely symmetric pick - two identical holes tight against each other - where
    /// either answer is the same to a user.
    ///
    /// C# throws `ArgumentNullException` on a null reading; the borrow cannot be null here.
    pub fn read(reading: &RoomReading, size: f64) -> ChainsThatCannotFit {
        let mut dropped: HashMap<i32, ChainCannotFit> = HashMap::new();
        let mut unresolved: Vec<Vec<i32>> = Vec::new();
        if !reading.is_a_size_statement() {
            // The zero this reading carries is not a statement about size at all - the pick
            // refuses by name, and blaming chains for not fitting would be an answer to a
            // question nobody asked.
            return ChainsThatCannotFit {
                dropped: Vec::new(),
                unresolved: Vec::new(),
            };
        }

        let drop = |dropped: &mut HashMap<i32, ChainCannotFit>,
                    chain: i32,
                    because: RoomSource,
                    room: f64| {
            dropped
                .entry(chain)
                .or_insert_with(|| ChainCannotFit::new(chain, because, room));
        };

        for candidate in &reading.candidates {
            if candidate.source.kind == RoomReadingKind::Foreign && candidate.room < size {
                drop(
                    &mut dropped,
                    candidate.source.chain,
                    candidate.source,
                    candidate.room,
                );
            }
        }

        let mut tight_groups: Vec<Vec<i32>> = Vec::new();
        for candidate in &reading.candidates {
            if candidate.source.kind != RoomReadingKind::SelfRoom || !(candidate.room < size) {
                continue;
            }

            match &candidate.group_members {
                Some(members) if members.len() != 1 => {
                    // Several chains share this side and it has run out of itself somewhere
                    // between them; which one to blame is beyond one reading. Held back until the
                    // pair pass has had its say, since a group whose every member goes for some
                    // OTHER reason needs no blame at all.
                    tight_groups.push(members.clone());
                }
                members => {
                    let chain = members.as_ref().map_or(candidate.source.chain, |m| m[0]);
                    drop(&mut dropped, chain, candidate.source, candidate.room);
                }
            }
        }

        // The ordered pair rows, by (chain, other), so a pair can be asked what EITHER of its two
        // would have if the other left. (C# also keeps the under-read per pair; nothing reads it,
        // as condemnation reads only the optimistic bound below.)
        let mut at_most_without_the_other: HashMap<(i32, i32), f64> = HashMap::new();
        let mut tight_pairs: HashSet<(i32, i32)> = HashSet::new();
        for candidate in &reading.candidates {
            if candidate.source.kind != RoomReadingKind::ChainToChain {
                continue;
            }

            let k = candidate.source.chain;
            let j = candidate.source.other_chain;
            at_most_without_the_other.insert((k, j), candidate.room_without_the_other_at_most);
            if candidate.room < size {
                tight_pairs.insert((k.min(j), k.max(j)));
            }
        }

        // The pairs in an order the GEOMETRY fixes: tightest first, then the one whose longer
        // chain is longer, and only then by index. Walking them by index would make WHICH of two
        // interlocking pairs is settled first - and so who is dropped - a matter of how the pick
        // was typed. The four keys make the order total, so the set's own order never shows.
        let mut tight: Vec<(i32, i32)> = tight_pairs.into_iter().collect();
        tight.sort_by(|p, q| {
            compare_like_csharp(
                pair_room_of(reading, p.0, p.1),
                pair_room_of(reading, q.0, q.1),
            )
            .then_with(|| {
                compare_like_csharp(
                    length_of(reading, q.0).max(length_of(reading, q.1)),
                    length_of(reading, p.0).max(length_of(reading, p.1)),
                )
            })
            .then(p.0.cmp(&q.0))
            .then(p.1.cmp(&q.1))
        });

        for (low, high) in tight {
            if dropped.contains_key(&low) || dropped.contains_key(&high) {
                continue;
            }

            // Condemnation reads the OPTIMISTIC bound - the most the chain could possibly have
            // once the other left. Reading the under-read instead condemned both ends of a pair
            // where dropping one would have done, and a loop that fits must never go unbuilt
            // (decision 6: build every loop that fits). Where the optimistic bound is not known
            // the chain may stay and the caller's re-read of the survivors decides.
            let low_stays = may_stay_without_the_other(&at_most_without_the_other, low, high, size);
            let high_stays =
                may_stay_without_the_other(&at_most_without_the_other, high, low, size);
            let pair = pair_room_of(reading, low, high);
            if !low_stays {
                drop(
                    &mut dropped,
                    low,
                    RoomSource::chain_to_chain(low, high),
                    pair,
                );
            }

            if !high_stays {
                drop(
                    &mut dropped,
                    high,
                    RoomSource::chain_to_chain(high, low),
                    pair,
                );
            }

            if low_stays && high_stays {
                // Either would do, so the SHORTER chain goes: build as much as possible, and say
                // it in a number read off the part rather than off the order the pick was typed
                // in. Dropping the later index instead made a face rim's answer depend on whether
                // the holes or the outer loop came first - the same part, the same size, two
                // different results for a user.
                let low_length = length_of(reading, low);
                let high_length = length_of(reading, high);
                // Equal to the last bit - two identical holes tight against each other - is
                // genuinely symmetric: either answer looks the same to a user, so the later index
                // settles it.
                let shorter = if low_length < high_length { low } else { high };

                let other = if shorter == low { high } else { low };
                drop(
                    &mut dropped,
                    shorter,
                    RoomSource::chain_to_chain(shorter, other),
                    pair,
                );
            }
        }

        // Only now: a group every member of which is leaving anyway has nothing left to resolve.
        for members in tight_groups {
            if !members.iter().all(|m| dropped.contains_key(m)) {
                unresolved.push(members);
            }
        }

        let by_chain: BTreeMap<i32, ChainCannotFit> = dropped.into_iter().collect();
        ChainsThatCannotFit {
            dropped: by_chain.into_values().collect(),
            unresolved,
        }
    }
}

/// True when chain `chain` would be clear of the size once `other` left. A pair row that was
/// never read (skipped, or its setbacks outside the derivation) says nothing, and silence is read
/// as "may stay" - the chain is then only dropped by the fixed rule, never condemned on a number
/// nobody measured.
fn may_stay_without_the_other(
    without_the_other: &HashMap<(i32, i32), f64>,
    chain: i32,
    other: i32,
    size: f64,
) -> bool {
    match without_the_other.get(&(chain, other)) {
        Some(room) if room.is_finite() => !(*room < size),
        _ => true,
    }
}

/// A chain's own length, or zero where the reading carries none - a pick of one chain, or a
/// reading built by hand. Zero for both makes the choice fall through to the last-resort index
/// rule.
fn length_of(reading: &RoomReading, chain: i32) -> f64 {
    if chain >= 0 && (chain as usize) < reading.chain_lengths.len() {
        reading.chain_lengths[chain as usize]
    } else {
        0.0
    }
}

/// The pair's own room, for the advisory: the LEAST of its two ordered rows, since that is the
/// one the pick's room would have been set by. NaN if neither row survived.
fn pair_room_of(reading: &RoomReading, low: i32, high: i32) -> f64 {
    let mut least = f64::NAN;
    for candidate in &reading.candidates {
        if candidate.source.kind == RoomReadingKind::ChainToChain
            && ((candidate.source.chain == low && candidate.source.other_chain == high)
                || (candidate.source.chain == high && candidate.source.other_chain == low))
        {
            least = if least.is_nan() {
                candidate.room
            } else {
                // C#'s Math.Min: NaN wins.
                if candidate.room.is_nan() {
                    f64::NAN
                } else {
                    least.min(candidate.room)
                }
            };
        }
    }

    least
}
