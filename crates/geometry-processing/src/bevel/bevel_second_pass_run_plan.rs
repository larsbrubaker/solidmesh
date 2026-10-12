//! What ONE merged run of the second pass is going to sweep, and which of the groups in it that
//! sweep answers for (`Bevel/BevelSecondPassRunPlan.cs`). Built from a run as
//! [`super::bevel_crease_merge::merge`] grouped it; `BevelSecondPass` sweeps it.
//!
//! **Why the first member's reading is not enough.** The crease merge is transitive: A is put
//! with B for sharing an edge with it and C is put with them for sharing one with B, and A and C
//! may share nothing at all. So a run's members do not all read the same crease - they read
//! overlapping stretches of one - and sweeping the first member's stretch while marking every
//! member rebuilt takes the others' first-pass beads off the batch and puts nothing where they
//! were. The union of the members' readings is what the run is named by, and what it sweeps.
//!
//! **And the union has to be one contiguous run.** A sweep carries one frame head to tail, so the
//! union is only swept as one bead if [`super::bevel_chain_linker::link`] can link the whole of it
//! into a single chain. Where it cannot - two stretches of one crease with a gap between them, or a
//! junction where three of the union's edges meet - the run falls back to sweeping the LONGEST
//! member's reading alone, and only the members that reading covers are answered for. The rest are
//! handed back to keep the bead the first pass built, which is the same trade the pass makes
//! everywhere else: a fillet along the crease as it was beats no fillet at all.

use std::collections::HashSet;

use super::bevel_chain_linker::{self, BevelSweptChain};
use crate::topology::{BevelEdgeGroupResolution, ChainEdge, FeatureEdge};

/// C# `BevelSecondPassRunPlan`.
#[derive(Debug, Clone)]
pub struct BevelSecondPassRunPlan {
    /// The reading the sweep is built FROM: its group's own tolerances, its feature set and its
    /// selection. One of the members' own, since every one of them describes the same surface.
    pub crease: BevelEdgeGroupResolution,
    /// The run the sweep carries its frame along where that is not [`Self::crease`]'s own chain -
    /// the linked union of the members' readings - and `None` where it is, which is every run whose
    /// members all read the same edges and so is nearly every run there is.
    pub linked_runs: Option<Vec<BevelSweptChain>>,
    /// Every edge this run will sweep over, which is what the pass offers the other runs as the
    /// selection they may run out into.
    pub edges: Vec<FeatureEdge>,
    /// The members whose crease the swept reading really does run along, in the order the run
    /// reports them - the first names the run. Only these are marked rebuilt, because only these
    /// have a replacement for the bead that would be withdrawn.
    pub covered: Vec<usize>,
    /// The members the swept reading does not answer for, which keep their first-pass bead. Empty
    /// on every run whose readings joined.
    pub not_covered: Vec<usize>,
    /// How many mesh edges the swept crease came to, which is what a row is told its crease became.
    pub edge_count: i32,
}

/// A reading's chain edges, empty where it walked none.
fn chain_edges(crease: &BevelEdgeGroupResolution) -> &[ChainEdge] {
    crease.chain.as_ref().map_or(&[], |chain| chain.edges())
}

/// A member's mesh edges, empty where it has no reading.
fn reading(mesh_edges_of_crease: &[Option<Vec<i32>>], member: usize) -> &[i32] {
    mesh_edges_of_crease[member].as_deref().unwrap_or(&[])
}

impl BevelSecondPassRunPlan {
    /// What one merged run sweeps: the union of its members' readings where those join into one
    /// chain, and the longest of them where they do not.
    ///
    /// `members` is the run as the crease merge grouped it; `creases` is every group's reading on
    /// the cut part, by resolution index (`None` where there is none), and `mesh_edges_of_crease`
    /// the mesh edges of each of those readings, in the same indexing. The merge only ever puts a
    /// group with a reading in a run; `None` comes back for a run that breaks that (C# would
    /// dereference the missing reading).
    pub fn for_members(
        members: &[usize],
        creases: &[Option<BevelEdgeGroupResolution>],
        mesh_edges_of_crease: &[Option<Vec<i32>>],
    ) -> Option<BevelSecondPassRunPlan> {
        let first = *members.first()?;
        let mut union = Vec::new();
        let mut in_the_union = HashSet::new();

        for &member in members {
            for edge in chain_edges(creases[member].as_ref()?) {
                if in_the_union.insert(edge.edge.mesh_edge_index) {
                    union.push(edge.edge);
                }
            }
        }

        let first_crease = creases[first].as_ref()?;

        if covers(reading(mesh_edges_of_crease, first), in_the_union.len()) {
            // The earliest member already reads the whole union, which is every run whose members
            // read the same crease from their own ends of it. Swept exactly as it was before the
            // union was worked out at all - its own walk order, its own chain - so nothing about
            // those runs moves.
            return Some(BevelSecondPassRunPlan {
                crease: first_crease.clone(),
                linked_runs: None,
                edges: first_crease.edges.clone(),
                covered: members.to_vec(),
                not_covered: Vec::new(),
                edge_count: chain_edges(first_crease).len() as i32,
            });
        }

        let linked = bevel_chain_linker::link(&union);

        if linked.len() == 1 && linked[0].edges.len() == union.len() {
            let edge_count = union.len() as i32;
            return Some(BevelSecondPassRunPlan {
                crease: first_crease.clone(),
                linked_runs: Some(linked),
                edges: union,
                covered: members.to_vec(),
                not_covered: Vec::new(),
                edge_count,
            });
        }

        // THE READINGS WILL NOT JOIN. The longest of them is swept - it is the most of the crease
        // any one reading has - and only the members it covers are answered for; see the module
        // notes for why the rest are handed back rather than swept along with it.
        let (longest, covered, not_covered) =
            longest_reading_and_what_it_covers(members, mesh_edges_of_crease);
        let longest_crease = creases[longest].as_ref()?;

        Some(BevelSecondPassRunPlan {
            crease: longest_crease.clone(),
            linked_runs: None,
            edges: longest_crease.edges.clone(),
            covered,
            not_covered,
            edge_count: chain_edges(longest_crease).len() as i32,
        })
    }
}

/// Which member of a run reads the most of its crease, and which of the others that reading
/// answers for - the fallback taken where the members' readings will not join into one chain.
///
/// The split is what keeps the pass honest: a member is only ever marked rebuilt - which withdraws
/// the bead the FIRST pass built for it - when the chain about to be swept carries every edge that
/// member read. Everyone else is named in `not_covered` and keeps the bead they have, so no member
/// of a merged run can lose its fillet to a sweep that does not run along its crease.
///
/// Returns the member whose reading is swept, the members that reading covers (it first, so the
/// run is named by the row it was resolved from), and the members it does not. `members` must not
/// be empty (C# reads `members[0]`); a member with no reading reads as no edges.
pub fn longest_reading_and_what_it_covers(
    members: &[usize],
    mesh_edges_of_crease: &[Option<Vec<i32>>],
) -> (usize, Vec<usize>, Vec<usize>) {
    let mut longest = members[0];

    for &member in members {
        if reading(mesh_edges_of_crease, member).len()
            > reading(mesh_edges_of_crease, longest).len()
        {
            longest = member;
        }
    }

    let swept: HashSet<i32> = reading(mesh_edges_of_crease, longest)
        .iter()
        .copied()
        .collect();
    let mut covered = vec![longest];
    let mut not_covered = Vec::new();

    for &member in members {
        if member == longest {
            continue;
        }

        if is_within(reading(mesh_edges_of_crease, member), &swept) {
            covered.push(member);
        } else {
            not_covered.push(member);
        }
    }

    (longest, covered, not_covered)
}

/// Whether one member's reading is the whole union, which it is when it has as many edges.
fn covers(reading: &[i32], union_count: usize) -> bool {
    let distinct: HashSet<i32> = reading.iter().copied().collect();

    distinct.len() == union_count
}

/// Whether every edge of a reading is one the swept chain carries.
fn is_within(reading: &[i32], swept: &HashSet<i32>) -> bool {
    reading.iter().all(|mesh_edge| swept.contains(mesh_edge))
}
