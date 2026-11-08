// The pass's side of the kernel's touching loops
// (`crate::bevel::part::touching_loops`) - naming the loops a touch leaves alone, and putting one back
// once the loop it touched has been dropped itself. A part of the builder rather than a type of
// its own because both halves read and rewrite the pass's private loops and rows. The level line
// (`level_line_build`) sets the touch it reads; the pass that drops loops (`build`) calls both.

use std::rc::Rc;

use crate::bevel::part::touching_loops;
use crate::bevel::{
    BevelAdvisory, BevelAdvisoryArg, BevelAdvisoryCode, BevelGroupResult, BevelLoopFailure,
};
use indexmap::IndexMap;
use solidmesh_math::Vector3;

use super::{BevelSurfacePartBuilder, RimLoop, TheChainCondemned};

/// A loop left alone's partner that stayed and the sentence it was given.
pub type LeftAlone = (Rc<RimLoop>, BevelAdvisory);

impl BevelSurfacePartBuilder {
    /// Whether the level line's last band refused the pick because two of its chains touch.
    pub fn the_band_refused_a_touch(&self) -> bool {
        self.the_band_refused_a_touch
    }

    /// This pass's loops left alone for touching, each with the partner that stayed and the
    /// sentence it was given, in the order they were left alone.
    pub fn left_alone_for_touching(&self) -> &IndexMap<Rc<RimLoop>, LeftAlone> {
        &self.left_alone_for_touching
    }

    /// Where the level line refused a many-loop rim because two of
    /// its closed loops touch, one loop of every touching pair named to be left alone, each with a
    /// sentence telling the user to pick it on its own (`touching_loops` says which of a pair
    /// stays). `None` where nothing is named.
    ///
    /// Not once the settling has been switched off: that is the pass saying these names could not
    /// be written as rows, and asking again would name the same loops forever. It then goes on
    /// down the older construction's path as it always did.
    ///
    /// The sentence does not suggest a second Bevel afterwards: that is unproven, and needs a test
    /// part whose touching loops each build alone.
    pub fn the_touching_loops_left_alone(
        &mut self,
        contributing: &[Rc<RimLoop>],
    ) -> Option<Vec<TheChainCondemned>> {
        if !self.the_band_refused_a_touch || self.the_settling_is_off {
            return None;
        }

        self.the_band_refused_a_touch = false;
        let loops: Vec<Vec<(i32, i32)>> = contributing
            .iter()
            .map(|one| {
                one.edges
                    .iter()
                    .map(|edge| (edge.start_vertex, edge.end_vertex))
                    .collect()
            })
            .collect();
        let leaving = touching_loops::the_ones_to_leave_alone(&loops);

        // Not gated on which chain the build a chain at a time gave up on. Only loops that really
        // touch are named here, so a loop failing for another reason is never given the touch's
        // sentence: it stays in the pass and is named for its own reason once the touch is gone.
        let mut condemned = Vec::new();

        for (one, partner, vertex) in leaving {
            let at = self.working_mesh.vertices[vertex as usize];
            let said = BevelAdvisory::new(
                BevelAdvisoryCode::LoopTouchesAnotherPickedLoop,
                vec![BevelAdvisoryArg::Vector3(Vector3::new(
                    at.x as f64,
                    at.y as f64,
                    at.z as f64,
                ))],
            );

            self.left_alone_for_touching.insert(
                contributing[one].clone(),
                (contributing[partner].clone(), said.clone()),
            );
            condemned.push(TheChainCondemned {
                mesh_edges: contributing[one].mesh_edges.clone(),
                reason: said,
                settled: None,
            });
        }

        if condemned.is_empty() {
            None
        } else {
            Some(condemned)
        }
    }

    /// Every loop left alone for touching whose partner has
    /// since been dropped itself - for its size or anything else - put back into the pass, and
    /// whether the pass has anybody left to build.
    ///
    /// The loop that stayed can still fail, and then the one left alone for it was dropped for
    /// nothing (Many Holes: both 70 edge loops kept over a 64 edge mouth, then refused by size).
    /// Put back, it loses its touch sentence and is asked again like any loop; if it still cannot
    /// build, it is named for whatever stops it then. Once only, so a loop cannot go round forever.
    pub fn put_back_the_loops_whose_partner_went(
        &mut self,
        picked: &[Rc<RimLoop>],
        contributing: &mut Vec<Rc<RimLoop>>,
        results: &[Option<BevelGroupResult>],
        refused_loops: &mut IndexMap<i32, Vec<BevelLoopFailure>>,
    ) -> bool {
        let snapshot: Vec<(Rc<RimLoop>, LeftAlone)> = self
            .left_alone_for_touching
            .iter()
            .map(|(one, held)| (one.clone(), held.clone()))
            .collect();

        for (one, (partner, said)) in snapshot {
            if contributing.contains(&partner) {
                continue;
            }

            // Order-keeping removal: the map's order is the order the loops were left alone in.
            self.left_alone_for_touching.shift_remove(&one);

            if contributing.contains(&one)
                || results[one.group_index as usize]
                    .as_ref()
                    .is_some_and(|row| row.failed())
                || !self.put_back_once.insert(one.clone())
            {
                continue;
            }

            // An advisory is a value, so the touch sentence is
            // matched by value - each touch sentence carries the vertex it is about, so no other
            // failure of that loop reads the same.
            if let Some(failures) = refused_loops.get_mut(&one.group_index) {
                let before = failures.len();
                failures.retain(|failure| failure.advisory != said);

                if failures.len() < before && failures.is_empty() {
                    // Removed in place; the map's order is the order rows were refused in.
                    refused_loops.shift_remove(&one.group_index);
                }
            }

            contributing.push(one);
        }

        // In the pick's own order, which is the order every reading of the pass is made in.
        // Stable, as the loops are distinct and so never tie (only loops absent from the pick
        // could tie, at `int.MaxValue`).
        let index_of = |one: &Rc<RimLoop>| -> usize {
            picked
                .iter()
                .position(|candidate| candidate == one)
                .unwrap_or(usize::MAX)
        };
        contributing.sort_by_key(index_of);

        !contributing.is_empty()
    }

    /// The loops already put back once in this rebuild.
    pub fn put_back_once(&self) -> &std::collections::HashSet<Rc<RimLoop>> {
        &self.put_back_once
    }
}
