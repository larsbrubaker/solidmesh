//! Walking the loops a band gap leaves, side by side. Here are the tests the walk's verdict is made
//! of - which side a both-sided loop is, which open pieces are a member's own, what each chain side
//! left, and whether each side closed, and the corner groups' test
//! ([`every_corner_group_leaves_one_loop`]) - the boundary step that follows another
//! ([`follows`], [`one_of`]) and the walk itself ([`loops_of_side`]).

use std::collections::{HashMap, HashSet};

use indexmap::IndexMap;

use std::rc::Rc;

use solidmesh_math::Vector3;

use super::triangle_edges::{self, level_line_triangle_corners};
use super::{consumed_to_the_rim, corner_fill, foreign_edge_room};
use super::{BandGroup, ChainSideLoops, GapLoop, LevelLineMesh, PickCorner};
use super::{ChainField, FieldSample, LevelLineWork, WorkError};

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

    // How many own pieces each member has, in first-seen order.
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

    // The membership test reads `rest` as it grows, so a member's second own piece is not added
    // once its first is.
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

/// The walk's answer where it failed at a corner: false, and that it failed at a corner (the
/// second of the pair).
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

/// Whether every CORNER group (a group not carved round a free end) leaves one loop per cluster of
/// members that meet at corners - closed where the cluster rings the face - with each member's
/// stretch one run of the loop and the members meeting at corners in the order their stretches
/// start; an end group must leave its own open pieces. Returns the verdict and whether it failed AT
/// A CORNER, which the caller reads as a pinch there.
///
/// Two members are joined, and a change of owner between them allowed, when they hold a corner
/// vertex together ANYWHERE; tying each transition to a corner on this face is the fill's work. An
/// END member is a cluster of its own, and a member CARRIED onto the face past a step corner joins
/// no cluster through itself but is held with each cluster it meets at a carried end, owning one
/// stretch per such end. Where `set_aside_cut_fronts`, an open member's front cut in two by
/// something across its side face is not read as a second loop.
pub fn every_corner_group_leaves_one_loop(
    groups: &[BandGroup],
    closed: &[bool],
    loops: &[GapLoop],
    chains: &[Vec<i32>],
    corners: &[PickCorner],
    set_aside_cut_fronts: bool,
) -> (bool, bool) {
    let meet = |a: i32, b: i32| {
        corners
            .iter()
            .any(|c| c.chains.contains(&a) && c.chains.contains(&b))
    };
    let corner_ends: HashSet<i32> = corners.iter().map(|c| c.vertex).collect();

    for group in groups {
        let members = &group.members;
        let of_group: Vec<GapLoop> = loops
            .iter()
            .filter(|l| {
                if l.has_mixed_owners {
                    l.side == group.tag && l.owners.iter().all(|o| members.contains(o))
                } else {
                    l.group_tag == group.tag
                        && members.contains(&l.chain)
                        && group.side_of_chain.get(&l.chain) == Some(&l.side)
                }
            })
            .cloned()
            .collect();

        if group.end_of >= 0 {
            if !end_group_leaves_open_pieces(group, loops) {
                return (false, false);
            }

            continue;
        }

        // Clusters: connected components of "meet at a corner" over the members.
        let mut cluster_of: HashMap<i32, i32> = HashMap::new();
        for &k in members {
            if cluster_of.contains_key(&k) {
                continue;
            }

            let mut stack = vec![k];
            cluster_of.insert(k, k);
            while let Some(at) = stack.pop() {
                for &next in members {
                    if !cluster_of.contains_key(&next)
                        && meet(at, next)
                        && !group.end_members.contains(&at)
                        && !group.end_members.contains(&next)
                        && !group.carried_ends.contains_key(&at)
                        && !group.carried_ends.contains_key(&next)
                    {
                        cluster_of.insert(next, k);
                        stack.push(next);
                    }
                }
            }
        }

        // The cores, grouped by first appearance of each cluster's key, so their order is the same
        // on every run.
        let mut cores: IndexMap<i32, Vec<i32>> = IndexMap::new();
        for &k in members {
            cores.entry(cluster_of[&k]).or_default().push(k);
        }

        for core in cores.values() {
            if core.iter().all(|k| group.carried_ends.contains_key(k)) {
                continue;
            }

            let mut carried_stretches: IndexMap<i32, usize> = IndexMap::new();
            for &c in group.carried_ends.keys() {
                let chain = &chains[c as usize];
                let mut ends = vec![chain[0]];
                if chain[chain.len() - 1] != chain[0] {
                    ends.push(chain[chain.len() - 1]);
                }
                let count = ends
                    .iter()
                    .filter(|&&v| {
                        corners.iter().any(|corner| {
                            corner.vertex == v
                                && corner.chains.contains(&c)
                                && corner.chains.iter().any(|x| core.contains(x))
                        })
                    })
                    .count();
                if count > 0 {
                    carried_stretches.insert(c, count);
                }
            }

            let cluster: Vec<i32> = core
                .iter()
                .copied()
                .chain(carried_stretches.keys().copied())
                .collect();
            let touching: Vec<GapLoop> = of_group
                .iter()
                .filter(|l| l.owners.iter().any(|o| core.contains(o)))
                .cloned()
                .collect();
            let theirs = if set_aside_cut_fronts {
                without_open_members_own_pieces(touching, &cluster, closed, &group.end_members)
            } else {
                touching
            };
            if cluster.len() == 1 && is_one_piece_per_end(&theirs, cluster[0], group) {
                continue;
            }

            if theirs.len() != 1 || theirs[0].owners.iter().any(|o| !cluster.contains(o)) {
                return (false, false);
            }

            let lp = &theirs[0];
            if cluster.len() == 1 {
                if lp.has_mixed_owners || (closed[cluster[0] as usize] && !lp.is_closed) {
                    return (false, false);
                }

                continue;
            }

            // A member carried onto this face past a step corner reaches it only round its ends,
            // so the cluster does not ring the face.
            let rings = !cluster.iter().any(|k| group.carried_ends.contains_key(k))
                && cluster.iter().all(|&k| {
                    let chain = &chains[k as usize];
                    closed[k as usize]
                        || (corner_ends.contains(&chain[0])
                            && corner_ends.contains(&chain[chain.len() - 1]))
                });
            if rings && !lp.is_closed {
                return failed_at_a_corner();
            }

            // Each vertex's owners: its minimiser, and every member of the cluster that ties it
            // there - a tie the walk broke by chain index belongs to both.
            let Some(owners) = lp.vertex_owners.as_ref() else {
                return failed_at_a_corner();
            };

            let n = owners.len();
            let sets: Vec<Vec<i32>> = (0..n)
                .map(|i| match &lp.vertex_ties {
                    None => vec![owners[i]],
                    Some(ties) => ties[i]
                        .iter()
                        .copied()
                        .filter(|&k| k == owners[i] || cluster.contains(&k))
                        .collect(),
                })
                .collect();
            if cluster
                .iter()
                .any(|k| !sets.iter().any(|set| set.contains(k)))
            {
                return failed_at_a_corner();
            }

            // Each member's stretch is ONE run of the vertices it owns (cyclic on a closed loop),
            // and the stretches in the order they start are the members in the order they meet.
            let mut starts: Vec<(usize, i32)> = Vec::new();
            for &k in &cluster {
                let mut begins: Vec<usize> = (0..n)
                    .filter(|&i| {
                        sets[i].contains(&k)
                            && if lp.is_closed {
                                !sets[(i + n - 1) % n].contains(&k)
                            } else {
                                i == 0 || !sets[i - 1].contains(&k)
                            }
                    })
                    .collect();
                if begins.is_empty() && lp.is_closed && sets.iter().all(|set| set.contains(&k)) {
                    // A stretch that is the WHOLE closed loop: it starts where another member's
                    // stretch ends.
                    begins = (0..n)
                        .filter(|&i| {
                            sets[(i + n - 1) % n]
                                .iter()
                                .any(|&c| c != k && !sets[i].contains(&c))
                        })
                        .take(1)
                        .collect();
                }

                // A BOTH-SIDED member has both its rails in this group: two stretches, unless the
                // loop also runs ROUND its free end, which makes the two rails one stretch.
                let stretches = match carried_stretches.get(&k) {
                    Some(&carried_ends) => carried_ends,
                    None if group.both_sided.contains(&k)
                        && !ends_inside_its_group(k as usize, groups, chains, closed) =>
                    {
                        2
                    }
                    None => 1,
                };
                if begins.len() != stretches {
                    return failed_at_a_corner();
                }

                starts.extend(begins.into_iter().map(|b| (b, k)));
            }

            let distinct: HashSet<usize> = starts.iter().map(|&(start, _)| start).collect();
            if distinct.len() != starts.len() {
                return failed_at_a_corner();
            }

            // Stable; the starts are distinct, so nothing ties.
            starts.sort_by_key(|&(start, _)| start);
            let runs: Vec<i32> = starts.iter().map(|&(_, chain)| chain).collect();
            let steps = if lp.is_closed {
                runs.len()
            } else {
                runs.len() - 1
            };
            for i in 0..steps {
                if !meet(runs[i], runs[(i + 1) % runs.len()]) {
                    return failed_at_a_corner();
                }
            }
        }
    }

    (true, false)
}

/// The boundary step that follows `step` (from, to), found by rotating
/// round its end vertex through the kept triangles until the fan runs out - or `None` where it does
/// not (or a triangle does not leave the vertex).
pub fn follows(mesh: &LevelLineMesh, step: (i32, i32)) -> Option<(i32, i32)> {
    let v = step.1;
    let t = one_of(mesh, step.0, v)?;
    let leaving = |triangle: i32| {
        level_line_triangle_corners(mesh.triangles()[triangle as usize])
            .into_iter()
            .find(|c| c.0 == v)
            .map(|c| c.1)
    };

    // The corner leaving v in that triangle's own winding; then round the fan, edge by edge.
    let mut going = leaving(t)?;
    for _ in 0..=mesh.triangle_count() {
        let standing = mesh.triangles_on(v, going);
        if standing.len() == 1 {
            return Some((v, going));
        }

        // The neighbour is the one traversing this edge the OTHER way; from it the fan turns on.
        let neighbour = standing.iter().copied().find(|&i| {
            level_line_triangle_corners(mesh.triangles()[i as usize])
                .iter()
                .any(|c| c.0 == going && c.1 == v)
        })?;
        going = leaving(neighbour)?;
    }

    None
}

/// The one live triangle traversing the edge `from` -> `to` that way, or
/// `None`.
pub fn one_of(mesh: &LevelLineMesh, from: i32, to: i32) -> Option<i32> {
    mesh.triangles_on(from, to).iter().copied().find(|&i| {
        level_line_triangle_corners(mesh.triangles()[i as usize])
            .iter()
            .any(|c| c.0 == from && c.1 == to)
    })
}

/// The gap's boundary on one side, walked with the kept surface on
/// the left. After the removal a gap edge stands on exactly one triangle, so the steps are read
/// straight off the mesh: each open edge whose remaining triangle belongs to this side, in that
/// triangle's own direction - and, across a KNIFE the band reached (or along a side consumed
/// exactly to its rim, `consumed_rims`), the edge whose kept triangle is the untagged far face.
///
/// Which step FOLLOWS a step is asked of the triangle fan ([`follows`]), not of the arriving
/// vertex: the boundary may pass through one vertex twice, and a successor kept per vertex
/// threaded a pinched boundary into scrambled loops. An open run is started from its own first
/// step. Each loop belongs to the chain that MINIMISES the group's field (`sample`) at it, read at
/// every vertex; a loop whose minimiser changes is mixed. With `tie_fields`, each vertex also names
/// every member whose field ties the owner's there ([`corner_fill::fields_tie`]).
#[allow(clippy::too_many_arguments)]
pub fn loops_of_side(
    mesh: &LevelLineMesh,
    sample: &dyn Fn(Vector3) -> Result<FieldSample, WorkError>,
    group: &BandGroup,
    tie_fields: Option<&[Rc<ChainField>]>,
    side_of_both_sided: Option<&dyn Fn(i32, Vector3) -> i32>,
    consumed_rims: Option<&[(Vector3, Vector3)]>,
    snap: f64,
) -> Result<Vec<GapLoop>, WorkError> {
    let side = group.tag;
    let mut steps: Vec<(i32, i32)> = Vec::new();

    // A KNIFE the band reached leaves an open edge whose kept triangle is the untagged far face:
    // the crease stretch is this side's rail there. Which side it belongs to is the removed
    // triangle's tag, so the removed triangles are read only when such an edge exists.
    let mut our_removed_on: Option<HashMap<(i32, i32), i32>> = None;
    let mut ours_across_a_knife = |edge: (i32, i32), kept: i32| {
        let removed_on = our_removed_on.get_or_insert_with(|| {
            let mut map = HashMap::new();
            for r in 0..mesh.triangle_count() {
                let t = mesh.triangles()[r as usize];
                if mesh.is_removed(r) && t.tag == side {
                    for (from, to) in level_line_triangle_corners(t) {
                        map.insert(triangle_edges::key(from, to), r);
                    }
                }
            }

            map
        });
        let Some(&removed) = removed_on.get(&edge) else {
            return false;
        };
        foreign_edge_room::is_a_knife(mesh, removed, kept, edge)
            || consumed_rims.is_some_and(|rims| {
                consumed_to_the_rim::lies_on_a_rim(
                    rims,
                    mesh.position(edge.0),
                    mesh.position(edge.1),
                    snap,
                )
            })
    };

    // The mesh's edges come in ascending (low, high) order, so the steps are gathered in a
    // canonical order and the loops do not depend on when the walk was asked.
    for edge in mesh.edges() {
        let standing = mesh.triangles_on(edge.0, edge.1);
        if standing.len() != 1 {
            continue;
        }

        let kept_tag = mesh.triangles()[standing[0] as usize].tag;
        if kept_tag != side && !(kept_tag == 0 && ours_across_a_knife(edge, standing[0])) {
            continue;
        }

        let t = mesh.triangles()[standing[0] as usize];
        if let Some(step) = level_line_triangle_corners(t)
            .into_iter()
            .find(|c| triangle_edges::key(c.0, c.1) == edge)
        {
            steps.push(step);
        }
    }

    let ours: HashSet<(i32, i32)> = steps.iter().copied().collect();
    let mut reached: HashSet<(i32, i32)> = HashSet::new();
    for &step in &steps {
        if let Some(following) = follows(mesh, step) {
            if ours.contains(&following) {
                reached.insert(following);
            }
        }
    }

    let mut loops = Vec::new();
    let mut walked: HashSet<(i32, i32)> = HashSet::new();

    // An open run has to be started from its own first step, or it comes out as two pieces.
    let heads: Vec<(i32, i32)> = steps
        .iter()
        .copied()
        .filter(|s| !reached.contains(s))
        .collect();
    for &start in heads.iter().chain(steps.iter()) {
        if walked.contains(&start) {
            continue;
        }

        let mut run: Vec<i32> = Vec::new();
        let mut current = start;
        let mut closed = false;
        while walked.insert(current) {
            LevelLineWork::charge(1)?;
            run.push(current.0);
            match follows(mesh, current).filter(|f| ours.contains(f)) {
                None => {
                    // The last step of an open run: its end vertex closes nothing, so it is carried
                    // as the run's last point.
                    run.push(current.1);
                    break;
                }
                Some(following) => {
                    current = following;
                    if current == start {
                        closed = true;
                        break;
                    }
                }
            }
        }

        if run.len() > 1 {
            let mut samples = Vec::with_capacity(run.len());
            for &v in &run {
                samples.push(sample(mesh.position(v))?);
            }

            let owner = samples[0].chain;
            let mixed = samples.iter().any(|x| x.chain != owner);
            // A loop's owner is a member of its group.
            let chain_side = if mixed {
                side
            } else if let (Some(side_of), true) =
                (side_of_both_sided, group.both_sided.contains(&owner))
            {
                let sides: Vec<i32> = run
                    .iter()
                    .map(|&v| side_of(owner, mesh.position(v)))
                    .collect();
                side_of_a_both_sided_loop(&sides, group.side_of_chain[&owner])
            } else {
                group.side_of_chain[&owner]
            };
            let mut owners: Vec<i32> = Vec::new();
            for x in &samples {
                if !owners.contains(&x.chain) {
                    owners.push(x.chain);
                }
            }

            owners.sort();
            let mut gap_loop = GapLoop::new(
                chain_side,
                closed,
                run.clone(),
                samples.iter().map(|x| x.parameter).collect(),
                owner,
                mixed,
                Some(owners),
            );
            gap_loop.vertex_owners = Some(samples.iter().map(|x| x.chain).collect());
            gap_loop.group_tag = side;
            if let Some(fields) = tie_fields {
                let mut ties = Vec::with_capacity(run.len());
                for (i, &v) in run.iter().enumerate() {
                    let mut tied = Vec::new();
                    for &k in &group.members {
                        if k == samples[i].chain
                            || corner_fill::fields_tie(
                                &fields[k as usize],
                                &fields[samples[i].chain as usize],
                                mesh.position(v),
                            )?
                        {
                            tied.push(k);
                        }
                    }

                    tied.sort();
                    ties.push(tied);
                }

                gap_loop.vertex_ties = Some(ties);
            }

            loops.push(gap_loop);
        }
    }

    Ok(loops)
}

/// The short form,
/// with the cut fronts set aside and the corner failure unread.
pub fn every_chain_side_leaves_one_owned_loop(
    groups: &[BandGroup],
    closed: &[bool],
    loops: &[GapLoop],
    chains: Option<&[Vec<i32>]>,
) -> bool {
    every_chain_side_leaves_one_owned_loop_at(groups, closed, loops, chains, true).0
}

/// Whether
/// every chain side the groups read left exactly one loop of its own (closed for a closed chain),
/// an end group its open pieces, and no mixed-owner loop outside an end group - read per corner
/// group where the chains meet at corners. Returns `(holds, failed at a corner)`.
pub fn every_chain_side_leaves_one_owned_loop_at(
    groups: &[BandGroup],
    closed: &[bool],
    loops: &[GapLoop],
    chains: Option<&[Vec<i32>]>,
    set_aside_cut_fronts: bool,
) -> (bool, bool) {
    if let Some(chains) = chains {
        let corners = super::pick_validation::corners_of(chains, closed, true);
        if !corners.is_empty() {
            return every_corner_group_leaves_one_loop(
                groups,
                closed,
                loops,
                chains,
                &corners,
                set_aside_cut_fronts,
            );
        }
    }

    let end_tags: HashSet<i32> = groups
        .iter()
        .filter(|g| g.end_of >= 0)
        .map(|g| g.tag)
        .collect();
    if loops
        .iter()
        .any(|l| l.has_mixed_owners && !end_tags.contains(&l.group_tag))
    {
        return (false, false);
    }

    for group in groups {
        if group.end_of >= 0 {
            if !end_group_leaves_open_pieces(group, loops) {
                return (false, false);
            }

            continue;
        }

        for &k in &group.members {
            let side = group.side_of_chain[&k];
            let mine: Vec<GapLoop> = loops
                .iter()
                .filter(|l| l.group_tag == group.tag && l.chain == k && l.side == side)
                .cloned()
                .collect();
            if is_one_piece_per_end(&mine, k, group) {
                continue;
            }

            if mine.len() != 1 || (closed[k as usize] && !mine[0].is_closed) {
                return (false, false);
            }
        }
    }

    (true, false)
}
