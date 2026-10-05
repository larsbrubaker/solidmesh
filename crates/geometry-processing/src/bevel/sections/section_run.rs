//! One chain's worth of sections ([`SectionRun`], `Bevel/Sections/SectionRun.cs`): the section
//! standing at each vertex an edge leaves, together with the chain they were computed from, with
//! its constructor and accessors (`count`, `start_of`, `end_of`).

use std::rc::Rc;

use indexmap::IndexMap;

use super::{BevelSection, JunctionSection, SectionAbsorbedSplits};
use crate::bevel::surface::SurfaceBevelPlan;
use crate::topology::ChainEdge;

/// C#'s `Func<int, bool>` naming which mesh edges are features: the operation's own reading, the
/// predicate a run's offsets were walked along the surface with.
pub type IsAFeatureEdge = Rc<dyn Fn(i32) -> bool>;

/// C# `SectionRun` (a sealed class). There is no strip here, on purpose: edge `i`'s strip is the
/// loft between `start_of(i)` and `end_of(i)`, and on a closed run `end_of(i)` and
/// `start_of(i + 1)` are the SAME section - shared through `Rc`, so the seam closes by identity
/// rather than by two copies that have to be re-matched at every joint.
pub struct SectionRun {
    /// The sections in walk order: `sections[i]` stands at `edges[i].from_vertex`. On a closed run
    /// there are as many sections as edges.
    pub sections: Vec<Rc<BevelSection>>,
    /// Whether the run closes on itself, so that the last edge leads back into the first.
    pub is_closed: bool,
    /// The chain edges the sections were computed from, oriented along the walk.
    pub edges: Vec<ChainEdge>,
    /// The plan the run was built from, or `None` where it was built from a bare chain.
    pub plan: Option<Rc<SurfaceBevelPlan>>,
    /// The operation's own reading of which mesh edges are features, carried so the smooth-side
    /// cut walks the same surface the sections did; `None` where the run was built without one.
    pub is_a_feature_edge: Option<IsAFeatureEdge>,
    /// The run's split vertices read as points of its strips rather than as stations, or `None`
    /// where nothing is split or absorbing split vertices is off.
    pub absorbed: Option<SectionAbsorbedSplits>,
    /// The run's JUNCTION STATIONS by station index - the one place a station owns TWO sections.
    /// `sections[station]` holds the junction's leaving section, and `end_of` of the edge
    /// arriving there hands back its arriving section instead. Empty on every run without one
    /// (C#'s shared empty dictionary); iterated in insertion order.
    pub junctions: IndexMap<i32, JunctionSection>,
}

/// The refusal [`SectionRun::end_of`] gives an open run's last edge (C# throws
/// `InvalidOperationException`, with this message).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NoSectionPastTheLastEdge;

impl std::fmt::Display for NoSectionPastTheLastEdge {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("an open run's last edge has no section past it yet")
    }
}

impl std::error::Error for NoSectionPastTheLastEdge {}

impl SectionRun {
    /// C#'s constructor; its optional arguments (`isAFeatureEdge`, `absorbed`, `junctions`, each
    /// null by default, junctions then empty) are passed explicitly.
    pub fn new(
        sections: Vec<Rc<BevelSection>>,
        is_closed: bool,
        edges: Vec<ChainEdge>,
        plan: Option<Rc<SurfaceBevelPlan>>,
        is_a_feature_edge: Option<IsAFeatureEdge>,
        absorbed: Option<SectionAbsorbedSplits>,
        junctions: Option<IndexMap<i32, JunctionSection>>,
    ) -> Self {
        Self {
            sections,
            is_closed,
            edges,
            plan,
            is_a_feature_edge,
            absorbed,
            junctions: junctions.unwrap_or_default(),
        }
    }

    /// How many edges the run has.
    pub fn count(&self) -> i32 {
        self.edges.len() as i32
    }

    /// The section standing at the vertex edge `edge_index` leaves.
    pub fn start_of(&self, edge_index: i32) -> Rc<BevelSection> {
        Rc::clone(&self.sections[edge_index as usize])
    }

    /// The section standing at the vertex edge `edge_index` arrives at: the next vertex's, or a
    /// junction's ARRIVING section there. A junction's two sections differ only in how far past the
    /// arriving wall end they run, and the strip arriving wants the shorter one.
    pub fn end_of(&self, edge_index: i32) -> Result<Rc<BevelSection>, NoSectionPastTheLastEdge> {
        let count = self.sections.len() as i32;
        if !self.is_closed && edge_index + 1 >= count {
            return Err(NoSectionPastTheLastEdge);
        }

        let ahead = if self.is_closed {
            (edge_index + 1) % count
        } else {
            edge_index + 1
        };

        Ok(match self.junctions.get(&ahead) {
            Some(junction) => Rc::clone(&junction.arriving_section),
            None => Rc::clone(&self.sections[ahead as usize]),
        })
    }
}
