//! Walking the loops a band gap leaves, side by side (`Bevel/LevelLine/GapLoopWalk.cs`, a static
//! class: free functions here). The tests the walk's verdict is made of land here first - which
//! side a both-sided loop is, which open pieces are a member's own, what each chain side left, and
//! whether each side closed; the walk itself and the corner groups' test
//! (`EveryCornerGroupLeavesOneLoop`) land with later packages.

use super::{BandGroup, ChainSideLoops, GapLoop};

/// Which side a loop that runs on BOTH sides belongs to: the side most of its vertices read, or -
/// where the smaller side holds a quarter of them or more - the side the group gives it.
pub fn side_of_a_both_sided_loop(vertex_sides: &[i32], group_side: i32) -> i32 {
    let left = vertex_sides.iter().filter(|&&s| s == 1).count();
    let right = vertex_sides.iter().filter(|&&s| s == 2).count();
    if 4 * left.min(right) >= vertex_sides.len() {
        return group_side;
    }

    if left > right {
        1
    } else {
        2
    }
}

/// `theirs` without the open pieces that are an open member's OWN (open, single-owned by a
/// non-closed, non-end member of `cluster`): a piece is set aside only when its member still owns
/// a vertex of a kept loop, and only one per member; where a member has more than two own pieces
/// (stray fragments, a side that did not close) everything is kept and the refusal stands.
pub fn without_open_members_own_pieces(
    theirs: Vec<GapLoop>,
    cluster: &[i32],
    closed: &[bool],
    end_members: &[i32],
) -> Vec<GapLoop> {
    if theirs.len() < 2 {
        return theirs;
    }

    let is_own_piece = |l: &GapLoop| {
        !l.is_closed
            && !l.has_mixed_owners
            && l.owners.len() == 1
            && cluster.contains(&l.owners[0])
            && !closed[l.owners[0] as usize]
            && !end_members.contains(&l.owners[0])
    };

    // How many own pieces each member has, in first-seen order (C#'s GroupBy).
    let own_pieces_of = |pieces: &mut dyn Iterator<Item = &GapLoop>| {
        let mut counts: Vec<(i32, usize)> = Vec::new();
        for piece in pieces {
            match counts
                .iter_mut()
                .find(|(owner, _)| *owner == piece.owners[0])
            {
                Some((_, count)) => *count += 1,
                None => counts.push((piece.owners[0], 1)),
            }
        }
        counts
    };

    // Stray fragments are a side that did not close, and keep the refusal.
    if own_pieces_of(&mut theirs.iter().filter(|l| is_own_piece(l)))
        .iter()
        .any(|&(_, count)| count > 2)
    {
        return theirs;
    }

    let mut rest: Vec<GapLoop> = theirs
        .iter()
        .filter(|l| !is_own_piece(l))
        .cloned()
        .collect();
    if rest.is_empty() {
        // A cluster of one open member cut in two: both pieces are its, as its one-chain build
        // leaves them.
        return if cluster.len() == 1 && theirs.len() == 2 {
            theirs.into_iter().take(1).collect()
        } else {
            theirs
        };
    }

    let set_aside = own_pieces_of(
        &mut theirs
            .iter()
            .filter(|l| is_own_piece(l) && rest.iter().any(|r| r.owners.contains(&l.owners[0]))),
    );
    if set_aside.iter().any(|&(_, count)| count > 1) {
        return theirs;
    }

    // C#'s `rest.AddRange(theirs.Where(.. !rest.Any(..)))` reads `rest` as it grows, so a member's
    // second own piece is not added once its first is.
    for piece in &theirs {
        if is_own_piece(piece) && !rest.iter().any(|r| r.owners.contains(&piece.owners[0])) {
            rest.push(piece.clone());
        }
    }

    rest
}

/// Whether an END group's pieces are all open and all its own, with every member owning one.
pub fn end_group_leaves_open_pieces(group: &BandGroup, loops: &[GapLoop]) -> bool {
    let pieces: Vec<&GapLoop> = loops.iter().filter(|l| l.group_tag == group.tag).collect();
    if pieces.is_empty()
        || pieces
            .iter()
            .any(|l| l.is_closed || l.owners.iter().any(|o| !group.members.contains(o)))
    {
        return false;
    }

    group
        .members
        .iter()
        .all(|k| pieces.iter().any(|l| l.owners.contains(k)))
}

/// Whether open chain `k` has more FREE ends (ends no other chain shares) than groups carve an end
/// out of it - an end that stops inside its group.
pub fn ends_inside_its_group(
    k: usize,
    groups: &[BandGroup],
    chains: &[Vec<i32>],
    closed: &[bool],
) -> bool {
    if closed[k] {
        return false;
    }

    let chain = &chains[k];
    let free_ends = [chain[0], chain[chain.len() - 1]]
        .iter()
        .filter(|v| !(0..chains.len()).any(|j| j != k && chains[j].contains(v)))
        .count();
    let member = k as i32;
    let carved = groups
        .iter()
        .filter(|g| {
            (g.end_of >= 0 && g.members.contains(&member)) || g.end_members.contains(&member)
        })
        .count();
    free_ends > carved
}

/// Whether an end member's pieces are exactly its two open, single-owned pieces - one per end.
pub fn is_one_piece_per_end(pieces: &[GapLoop], member: i32, group: &BandGroup) -> bool {
    group.end_members.contains(&member)
        && pieces.len() == 2
        && pieces.iter().all(|l| {
            !l.is_closed && !l.has_mixed_owners && l.owners.len() == 1 && l.owners[0] == member
        })
}

/// The walk's answer where it failed at a corner: false, and that it failed at a corner (C#'s
/// `out bool`, here the second of the pair).
pub fn failed_at_a_corner() -> (bool, bool) {
    (false, true)
}

/// What each chain side left: per group, per member, the loops it owns on its side, how many of
/// them closed, the MIXED loops it shares (counted against every chain that minimises on them),
/// the other chains it met (sorted, once each), and whether that side is OWED nothing more - one
/// loop of its own, no mixed loop, and closed where the chain is.
pub fn what_each_chain_side_left(
    groups: &[BandGroup],
    closed: &[bool],
    loops: &[GapLoop],
) -> Vec<ChainSideLoops> {
    let mut rows = Vec::new();
    for group in groups {
        for &k in &group.members {
            let side = group.side_of_chain[&k];

            let mine: Vec<&GapLoop> = loops
                .iter()
                .filter(|l| l.side == side && l.chain == k && !l.has_mixed_owners)
                .collect();
            let mixed: Vec<&GapLoop> = loops
                .iter()
                .filter(|l| l.has_mixed_owners && l.owners.contains(&k))
                .collect();

            let mut met_chains: Vec<i32> = mixed
                .iter()
                .flat_map(|l| l.owners.iter())
                .chain(mine.iter().flat_map(|l| l.owners.iter()))
                .copied()
                .filter(|&other| other != k)
                .collect();
            met_chains.sort_unstable();
            met_chains.dedup();

            rows.push(ChainSideLoops {
                chain: k,
                side,
                group_tag: group.tag,
                loops: mine.len() as i32,
                closed_loops: mine.iter().filter(|l| l.is_closed).count() as i32,
                mixed_loops: mixed.len() as i32,
                met_chains,
                owed: mine.len() == 1
                    && mixed.is_empty()
                    && (!closed[k as usize] || mine[0].is_closed),
            });
        }
    }

    rows
}

/// Whether each of the two sides left exactly one loop, and it closed.
pub fn each_side_leaves_one_closed_loop(loops: &[GapLoop]) -> bool {
    [1, 2].iter().all(|&side| {
        let mine: Vec<&GapLoop> = loops.iter().filter(|l| l.side == side).collect();
        mine.len() == 1 && mine[0].is_closed
    })
}
