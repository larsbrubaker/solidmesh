// A face rim built after its partner: where a pass's level-line build refused because a face rim's
// rail runs backward next to one of its cove-to-round corners (the cove-round offer's trigger,
// `cove_round_retry::the_flips_to_cut`) and the pick also holds groups that are not face rims, the
// pass is built IN TWO. The other groups go first, on the part as picked; the face rims go second,
// re-flooded from their face anchors on the first pass's output (`bevel_edge_group_resolver`), by a
// second level-line builder over that mesh. It is asked only after the pass built as it always was
// has refused a group (`build_the_groups_together`), so what blends today keeps its look, and its
// answer is taken only where it refuses fewer groups. The order follows one rule ([`goes_first`]): the group
// whose answer leaves the other group's face still readable goes first. A face rim re-floods from
// its face on any mesh; an open edge chain is found by its own edges, which a round laid first
// would eat. On "Unexpected Refusals.mcx" the tangent chain's round ends flush on the floor, and
// the floor's rim, re-read round the floor that leaves, then blends as one round. Every other
// pass's route is untouched. `build_the_groups_together` asks this. As for a pass the sections
// build in two, the two-pass mesh relies on each half's own route for closure and for not
// crossing itself (every level-line answer is read for both before it is published).

use std::rc::Rc;
use std::sync::Arc;

use crate::bevel::level_line::level_line_routing;
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode, BevelGroupResult, BevelLoopFailure};
use crate::topology::bevel_edge_group_resolver;
use crate::topology::{BevelEdgeGroup, BevelSelectionKind};
use indexmap::IndexMap;
use polygon_mesh::{CancelToken, Mesh, MeshEdgeGraph};
use vector_math::Matrix4X4;

use super::{BevelSurfacePartBuilder, RimLoop, SurfacePartBuildError};

/// Whether `group` goes in the first half of a pass built in two: the group whose answer leaves
/// the other group's face still readable goes first. A face rim (a face group) is re-flooded from
/// its face on whatever mesh it is built on, so it can go second; an edge chain is found by its own
/// edges, which a round laid before it would eat, so it goes first.
pub fn goes_first(group: &BevelEdgeGroup) -> bool {
    group.selection_kind != BevelSelectionKind::FaceGroup
}

/// What a pass built in two came to: not taken (the pass goes on as it would have), or its answer.
pub enum TheRimAfterItsPartner {
    /// The pick has no face rim to put second, or nothing to put first.
    NotTaken,
    /// The pass's mesh (boxed: a mesh is large beside the other variant), or `None` where
    /// neither half built.
    Built(Option<Box<Mesh>>),
}

impl BevelSurfacePartBuilder {
    /// The pass over `contributing` built in two, face rims second (see the file header).
    pub(super) fn the_rim_after_its_partner(
        &mut self,
        contributing: &[Rc<RimLoop>],
        results: &mut [Option<BevelGroupResult>],
        refused_loops: &mut IndexMap<i32, Vec<BevelLoopFailure>>,
        cancellation_token: &CancelToken,
    ) -> Result<TheRimAfterItsPartner, SurfacePartBuildError> {
        let (mut first, second): (Vec<Rc<RimLoop>>, Vec<Rc<RimLoop>>) = contributing
            .iter()
            .cloned()
            .partition(|one| goes_first(&self.resolutions[one.group_index as usize].group));
        if first.is_empty() || second.is_empty() {
            return Ok(TheRimAfterItsPartner::NotTaken);
        }

        // The first half on the part as picked; the trigger is this pass's, not the half's.
        self.the_rim_refused_at_its_flips = false;
        self.a_rim_refused_at_its_flips_in_the_pass = false;
        let mesh_a = self.the_one_pass(
            &mut first,
            results,
            refused_loops,
            cancellation_token,
            Self::PASS_A_END_ON_THE_BAR,
        )?;
        let Some(mesh_a) = mesh_a else {
            return Ok(TheRimAfterItsPartner::Built(None));
        };

        // The face rims re-flooded from their faces on the first half's output: read as the build
        // read them (the resolution's own group, in build space at the operation's bend
        // threshold), since the first half's mesh is in build space too. A face whose flood finds
        // no rim there is a rim the first half's round took: refused by name, as a lost loop of a
        // pass the sections built in two is (`pass_in_two`).
        let mut offered_b = vec![false; self.resolutions.len()];
        let mut rebound = self.resolutions.clone();
        let mut reflooded: Vec<i32> = Vec::new();
        for one in &second {
            let index = one.group_index as usize;
            if offered_b[index] || reflooded.contains(&(index as i32)) {
                continue;
            }
            let again = bevel_edge_group_resolver::resolve(&self.resolutions[index].group, &mesh_a);
            if again.is_resolved() {
                offered_b[index] = true;
                rebound[index] = again;
            } else {
                reflooded.push(index as i32);
                let advisory =
                    BevelAdvisory::new(BevelAdvisoryCode::SurfaceConstructionNotBuilt, vec![]);
                for lost in second
                    .iter()
                    .filter(|other| other.group_index == one.group_index)
                {
                    refused_loops
                        .entry(lost.group_index)
                        .or_default()
                        .push(BevelLoopFailure {
                            advisory: advisory.clone(),
                            // A size of zero, not a missing size: nothing fits.
                            largest_size_that_fits: Some(0.0),
                            edge_count: lost.edges.len() as i32,
                        });
                }
            }
        }
        if !offered_b.iter().any(|&offered| offered) {
            Self::write_what_the_loops_came_to(results, refused_loops);
            return Ok(TheRimAfterItsPartner::Built(Some(Box::new(mesh_a))));
        }

        let graph_a = MeshEdgeGraph::create(&mesh_a);
        let mesh_a = Arc::new(mesh_a);
        // NO memo, as for a pass built in two by the sections: the second half works on a mesh the
        // first rewrote at THIS size.
        let mut second_builder = BevelSurfacePartBuilder::new(
            self.groups.clone(),
            rebound,
            offered_b.clone(),
            self.advisories.clone(),
            mesh_a.clone(),
            graph_a,
            self.size,
            self.segments,
            self.round_profile,
            Matrix4X4::IDENTITY,
            self.source_color,
            None,
            Some(self.bar.the_rest_up_to(Self::PASSES_END_ON_THE_BAR)),
            None,
        );
        // Its sizes stand on the first half's mesh at the size asked: it offers no number.
        second_builder.offers_no_size = true;
        second_builder.size_search_budget_ms = self.size_search_budget_ms;

        let mut results_b: Vec<Option<BevelGroupResult>> = vec![None; self.resolutions.len()];
        let building_b: Vec<i32> = (0..offered_b.len() as i32)
            .filter(|&index| offered_b[index as usize])
            .collect();
        // The test seam that makes the second half refuse (`fail_the_second_half`): it builds
        // nothing and leaves its groups without rows, which the pass counts as refused.
        let mesh_b = if level_line_routing::the_second_half_fails() {
            None
        } else {
            second_builder.build_the_groups_together(
                &building_b,
                &mut results_b,
                cancellation_token,
            )?
        };
        drop(second_builder);

        Self::write_what_the_loops_came_to(results, refused_loops);
        self.merge_the_rows(results, &results_b, &building_b);

        Ok(TheRimAfterItsPartner::Built(Some(Box::new(match mesh_b {
            Some(mesh_b) => mesh_b,
            None => Arc::try_unwrap(mesh_a).unwrap_or_else(|shared| (*shared).clone()),
        }))))
    }
}
