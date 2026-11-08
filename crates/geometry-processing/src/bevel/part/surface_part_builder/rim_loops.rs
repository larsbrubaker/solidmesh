// The unit the surface construction drops and retries -
// one RIM LOOP of one group, rather than the whole group. The loops are the connected components
// of the group's own picked edges, which `bevel_chain_linker::link` walks them into; the builder's
// passes (`build`, `size_that_fits`) drop and retry them, and `level_line_rows` writes their groups' rows.

use std::collections::HashSet;
use std::hash::{Hash, Hasher};
use std::rc::Rc;

use crate::bevel::bevel_chain_linker;
use crate::bevel::{BevelAdvisory, BevelSweptChain};
use crate::topology::FeatureEdge;

use super::BevelSurfacePartBuilder;
use crate::bevel::part::refusal_place::the_edge_nearest_the_place_among;

/// One connected run of one group's picked edges, and which group picked
/// it.
///
/// A face group's rim is one closed loop per hole in the region it flooded - forty-five of them
/// on the barrel of "Many Holes.mcx", forty-one being port mouths - and those loops share no edge,
/// no face and no corner. They are independent bevels that happen to have been picked with one
/// click, so a refusal about one of them is no more the others' than one GROUP's refusal is the
/// next group's.
///
/// A loop is identified by its identity, not its edges (what is dropped is found by `contains`,
/// and loops key maps); the builder makes each loop once and hands out `Rc`s, and equality and hashing
/// here are that identity - the `id` the builder stamped it with.
#[derive(Debug)]
pub struct RimLoop {
    id: usize,

    /// The group that picked it.
    pub group_index: i32,

    pub edges: Vec<FeatureEdge>,

    /// The loop's own mesh edges, which is what a refusal naming an edge is matched against.
    pub mesh_edges: HashSet<i32>,
}

impl RimLoop {
    fn new(id: usize, group_index: i32, edges: Vec<FeatureEdge>) -> Self {
        let mesh_edges = edges.iter().map(|edge| edge.mesh_edge_index).collect();

        RimLoop {
            id,
            group_index,
            edges,
            mesh_edges,
        }
    }
}

impl PartialEq for RimLoop {
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id
    }
}

impl Eq for RimLoop {}

impl Hash for RimLoop {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.id.hash(state);
    }
}

impl BevelSurfacePartBuilder {
    /// Every contributing group's edges cut into the loops they link into, in
    /// the groups' own order.
    pub fn the_rim_loops_of(&mut self, building: &[i32]) -> Vec<Rc<RimLoop>> {
        let mut loops = Vec::new();

        for &index in building {
            loops.extend(self.the_rim_loops_of_the_group(index));
        }

        loops
    }

    /// One group's picked edges as loops, made once and remembered -
    /// because the retry asks for them again on every turn, and because the loops have to be the
    /// SAME objects from turn to turn, since what is dropped is identified by reference.
    pub fn the_rim_loops_of_the_group(&mut self, index: i32) -> Vec<Rc<RimLoop>> {
        if let Some(found) = self.rim_loops.get(&index) {
            return found.clone();
        }

        // Numbered across every group of this builder, so two loops of two groups never compare
        // equal.
        let mut next_id: usize = self.rim_loops.values().map(Vec::len).sum();

        let edges = &self.resolutions[index as usize].edges;
        let mut loops = Vec::new();

        for chain in bevel_chain_linker::link(edges) {
            loops.push(Rc::new(RimLoop::new(
                next_id,
                index,
                chain.edges.iter().map(|edge| edge.edge).collect(),
            )));
            next_id += 1;
        }

        if loops.is_empty() && !edges.is_empty() {
            // A group the linker found no run in is still one unit, so that it is dropped and
            // reported as a whole group.
            loops.push(Rc::new(RimLoop::new(next_id, index, edges.clone())));
        }

        self.rim_loops.insert(index, loops.clone());

        loops
    }

    /// Which groups these loops belong to, once each, sorted.
    pub fn the_groups_of(loops: &[Rc<RimLoop>]) -> Vec<i32> {
        let mut groups: Vec<i32> = Vec::new();

        for one in loops {
            if !groups.contains(&one.group_index) {
                groups.push(one.group_index);
            }
        }

        // Distinct integers, so the sort has no ties to order.
        groups.sort();

        groups
    }

    /// Every mesh edge these loops stand on.
    pub fn the_edges_of(loops: &[Rc<RimLoop>]) -> HashSet<i32> {
        let mut edges = HashSet::new();

        for one in loops {
            edges.extend(one.mesh_edges.iter().copied());
        }

        edges
    }

    /// The chains this attempt is to build - the ones every edge
    /// of which is still contributing.
    ///
    /// A chain the network linked across a dropped loop's edges goes with it, whole: a chain is
    /// planned and built as one thing, so half of one is not something this construction has a
    /// meaning for. Where nothing has been dropped this returns what it was given.
    pub fn only_the_contributing_loops(
        chains: &[BevelSweptChain],
        contributing_edges: &HashSet<i32>,
    ) -> Vec<BevelSweptChain> {
        chains
            .iter()
            .filter(|chain| {
                chain
                    .edges
                    .iter()
                    .all(|edge| contributing_edges.contains(&edge.edge.mesh_edge_index))
            })
            .cloned()
            .collect()
    }

    /// Which of these loops hold any of the mesh edges a refusal named - the
    /// loop-sized answer to the question the group-sized one used to be asked.
    pub fn the_loops_holding(
        contributing: &[Rc<RimLoop>],
        guilty: &HashSet<i32>,
    ) -> Vec<Rc<RimLoop>> {
        if guilty.is_empty() {
            return Vec::new();
        }

        contributing
            .iter()
            .filter(|one| !one.mesh_edges.is_disjoint(guilty))
            .cloned()
            .collect()
    }

    /// The picked mesh edge nearest a place, taken over
    /// LOOPS rather than over groups - two loops of one face's rim are as separate as two groups
    /// are, so the clear-winner margin has to be asked between them as well. `None` where there is
    /// no clear winner.
    pub fn the_loop_nearest_where_it_says_it_happened(
        &self,
        contributing: &[Rc<RimLoop>],
        reason: Option<&BevelAdvisory>,
    ) -> Option<i32> {
        let units: Vec<&[FeatureEdge]> = contributing
            .iter()
            .map(|one| one.edges.as_slice())
            .collect();

        the_edge_nearest_the_place_among(&units, reason, self.size)
    }
}
