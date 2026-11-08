// The half of the surface construction that says WHO
// builds a pass - the planning the whole pass shares, and, with `section_routing::use_sections`
// on, which of its loops the sections will take. The kernel's router answers for a WHOLE pass, so
// one plan it declines ("a chain is not closed" - a wall face's rim whose top edge belongs to
// another row) would send every loop of that pass to the legacy construction, which cannot close
// a raked hole mouth and drops it. Read per plan here (`section_routing::partition`) so the pass
// can be built IN TWO: the accepted loops through the sections, the rest by the legacy
// construction on their output, by a second builder over that mesh. The one pass that calls this
// is `build`; one attempt (`try_together`) shares the planning.

use std::collections::{HashMap, HashSet};
use std::rc::Rc;
use std::sync::Arc;

use crate::bevel::sections::section_routing;
use crate::bevel::surface::{
    surface_chain_join, surface_chain_opens_at_the_junction, surface_interrupted_crease,
    surface_shared_edge_network, SurfaceBevelPlan,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode, BevelGroupResult, BevelLoopFailure};
use crate::topology::{
    BevelEdgeGroupResolution, EdgeConvexity, FaceRegionCache, FeatureEdge, FeatureEdgeSet,
};
use indexmap::IndexMap;
use polygon_mesh::{CancelToken, Mesh, MeshEdgeGraph};
use vector_math::{Matrix4X4, Vector3Float};

use super::{BevelSurfacePartBuilder, RimLoop, SurfacePartBuildError};

/// What planning a pass came to: whether every chain was planned, and the plans.
#[derive(Default)]
pub struct PlannedPass {
    /// Whether every chain was planned.
    pub planned: bool,
    /// The flat-angle reading the plans were cut into, which the build needs too.
    pub regions: Option<Arc<FaceRegionCache>>,
    /// One plan per chain, in the order the chains were linked, joined and opened.
    pub plans: Vec<Rc<SurfaceBevelPlan>>,
    /// The picked edges a refusal is about.
    pub guilty: HashSet<i32>,
    /// Why the pass could not be planned, where it says.
    pub reason: Option<BevelAdvisory>,
}

/// An exact position key: the components' bits, with -0 folded onto 0, because the two zeros are
/// the same position.
fn position_key(position: Vector3Float) -> (u32, u32, u32) {
    let bits = |value: f32| {
        if value == 0.0 {
            0.0f32.to_bits()
        } else {
            value.to_bits()
        }
    };
    (bits(position.x), bits(position.y), bits(position.z))
}

impl BevelSurfacePartBuilder {
    /// Every contributing loop's chains linked, joined, opened and planned
    /// against the working mesh - the reading every later step of a pass is made of, and the one
    /// the router is asked about.
    pub fn try_plan_these_loops(
        &mut self,
        contributing: &[Rc<RimLoop>],
        trial_size: f64,
    ) -> PlannedPass {
        let mut planned = PlannedPass::default();

        // The operation's bend threshold, which every resolution was read at, so which group is
        // first here decides nothing.
        let group_indices = Self::the_groups_of(contributing);

        let Some(&first) = group_indices.first() else {
            return planned;
        };

        let contributing_edges = Self::the_edges_of(contributing);
        let features = self.resolutions[first as usize].features.clone();
        let regions = self.regions_at(features.flat_angle_degrees());
        planned.regions = Some(regions.clone());

        // The same mesh read as SURFACES rather than as flat faces, at the pick's own curve
        // tolerance: it answers the question at the END of a chain - whether the crease the chain
        // stops at carries on without it. Taken from the same group the flat angle is, so the two
        // readings are one pick's.
        let curve_regions = self.regions_at(
            self.resolutions[first as usize]
                .group
                .curve_tolerance_degrees,
        );

        // ONE NETWORK, not one selection at a time: two groups that name the same edge are linked
        // together. And only the loops still contributing: a loop dropped by an earlier turn of
        // the retry is taken out after the network has linked what is left.
        let selected: Vec<BevelEdgeGroupResolution> = group_indices
            .iter()
            .map(|&index| self.resolutions[index as usize].clone())
            .collect();
        let chains = Self::only_the_contributing_loops(
            &surface_shared_edge_network::link_together(&selected),
            &contributing_edges,
        );

        if chains.is_empty() {
            return planned;
        }

        // Two groups whose runs meet end to end are ONE run to bevel, so they are joined before
        // anything is planned; and a closed run that passes THROUGH a vertex where another run
        // ends is opened there, so the three picked edges meeting at it are three chain ends.
        let opened = surface_chain_opens_at_the_junction::open(surface_chain_join::join(&chains));

        // And TWO PIECES OF ONE CREASE: a crease something standing on it interrupts, picked both
        // sides of what interrupts it. A reading over all the chains together, refused for the
        // whole pass at the size whose bead would run UNDER it.
        if let Some(interrupted) = surface_interrupted_crease::try_read(
            &opened,
            &features,
            &regions,
            trial_size,
            self.round_profile,
        ) {
            planned.reason = Some(interrupted);
            return planned;
        }

        for chain in &opened {
            match SurfaceBevelPlan::try_create_with_curve_regions(
                chain,
                &regions,
                Some(&curve_regions),
                trial_size,
                self.round_profile,
            ) {
                Ok(plan) => planned.plans.push(plan),
                Err(refused) => {
                    // The CHAIN is what could not be planned, and a chain is made of picked
                    // edges, so the refusal has a set of them to be about even where the plan
                    // named none itself.
                    planned.reason = Some(refused);

                    for edge in &chain.edges {
                        planned.guilty.insert(edge.edge.mesh_edge_index);
                    }

                    planned.plans.clear();
                    return planned;
                }
            }
        }

        planned.planned = true;
        planned
    }

    /// The pass built in two - the loops the sections ACCEPT built by them
    /// first, and then the declined ones by the legacy construction on the mesh that leaves,
    /// fenced off the blend they laid.
    ///
    /// The second pass is a second builder over the first one's OUTPUT. Its edges are found again
    /// on that mesh by their endpoint POSITIONS rather than by re-flooding the face picks: the
    /// sections' publish copies an untouched vertex's position bit for bit. An endpoint or an edge
    /// that is NOT found is a loop the first pass's fillet ate, and it is refused by name rather
    /// than silently skipped. A loop dropped in pass A is not retried in pass B: its row already
    /// says what became of it.
    pub(super) fn build_in_two(
        &mut self,
        contributing: &[Rc<RimLoop>],
        for_legacy: &[Rc<RimLoop>],
        results: &mut [Option<BevelGroupResult>],
        refused_loops: &mut IndexMap<i32, Vec<BevelLoopFailure>>,
        cancellation_token: &CancelToken,
    ) -> Result<Option<Mesh>, SurfacePartBuildError> {
        let mut for_sections: Vec<Rc<RimLoop>> = contributing
            .iter()
            .filter(|one| !for_legacy.contains(one))
            .cloned()
            .collect();

        // THE SECTIONS HALF GOES FIRST, on the mesh the pick was made on: only the sections
        // publish the faces they laid, so the half that can be fenced is the one that must go
        // second.
        let mesh_a = self.the_one_pass(
            &mut for_sections,
            results,
            refused_loops,
            cancellation_token,
            Self::PASS_A_END_ON_THE_BAR,
        )?;
        let fence = self.blend_faces_of_the_last_build.clone();

        let Some(mesh_a) = mesh_a else {
            // Pass A's own refusal is already written against its rows, and there is no output to
            // build the rest on.
            return Ok(None);
        };

        // WHAT THE FENCE IS FOR: a fillet's facets turn less than the flat angle at each step, so
        // an unfenced flood that used to stop at the raked mouth's sharp rim runs over the blend
        // and out through the bore. Nothing else of pass B is re-read on this mesh, so the fence
        // has one place to be and that is every region cache the second builder makes.
        let graph_a = MeshEdgeGraph::create(&mesh_a);

        let rebound_resolutions = match self.the_same_loops_on(&mesh_a, &graph_a, for_legacy) {
            Ok(rebound) => rebound,
            Err(lost) => {
                for one in lost {
                    // Its edge is not on the first pass's mesh, which means the fillet built
                    // there took it.
                    let advisory =
                        BevelAdvisory::new(BevelAdvisoryCode::SurfaceConstructionNotBuilt, vec![]);

                    refused_loops
                        .entry(one.group_index)
                        .or_default()
                        .push(BevelLoopFailure {
                            advisory,
                            // A size of zero, not a missing size: nothing fits.
                            largest_size_that_fits: Some(0.0),
                            edge_count: one.edges.len() as i32,
                        });
                }

                Self::write_what_the_loops_came_to(results, refused_loops);

                return Ok(Some(mesh_a));
            }
        };

        let mut offered_b = vec![false; self.resolutions.len()];

        for one in for_legacy {
            offered_b[one.group_index as usize] = true;
        }

        let mesh_a = Arc::new(mesh_a);

        // NO memo, deliberately: pass B works on the mesh pass A rewrote at THIS size, so every
        // key it would write differs at every step of a drag. The builder gives itself an empty
        // one, which is the route as it was.
        let mut second = BevelSurfacePartBuilder::new(
            self.groups.clone(),
            rebound_resolutions,
            offered_b.clone(),
            self.advisories.clone(),
            mesh_a.clone(),
            graph_a,
            self.size,
            self.segments,
            self.round_profile,
            Matrix4X4::IDENTITY,
            self.source_color,
            Some(fence),
            Some(self.bar.the_rest_up_to(Self::PASSES_END_ON_THE_BAR)),
            None,
        );

        // Its sizes stand on pass A's mesh at the size asked, so none of them is one the user's
        // typed size would build: pass B offers no number.
        second.offers_no_size = true;
        // The same operation's search, so the same clock.
        second.size_search_budget_ms = self.size_search_budget_ms;

        let mut results_b: Vec<Option<BevelGroupResult>> = vec![None; self.resolutions.len()];
        let building_b: Vec<i32> = (0..offered_b.len() as i32)
            .filter(|&index| offered_b[index as usize])
            .collect();

        // By name, because these are the plans the router already declined on the source mesh:
        // asking it again would route them back here anyway.
        // Scoped to this build on this thread (and the compute it hands on), so a bevel building
        // beside it keeps its own setting and the switch comes back even if the build panics.
        let mesh_b = {
            let _without_sections = section_routing::SectionRoutingContext::current()
                .with_use_sections(false)
                .enter();
            second.build_the_groups_together(&building_b, &mut results_b, cancellation_token)
        };
        let mesh_b = mesh_b?;
        drop(second);

        Self::write_what_the_loops_came_to(results, refused_loops);
        self.merge_the_rows(results, &results_b, &building_b);

        Ok(Some(match mesh_b {
            Some(mesh_b) => mesh_b,
            None => Arc::try_unwrap(mesh_a).unwrap_or_else(|shared| (*shared).clone()),
        }))
    }

    /// The rows of both passes put together - a row that built in EITHER pass
    /// built, and the loop failures of both are the row's.
    pub(super) fn merge_the_rows(
        &self,
        results: &mut [Option<BevelGroupResult>],
        results_b: &[Option<BevelGroupResult>],
        building_b: &[i32],
    ) {
        for &index in building_b {
            let Some(mut from_b) = results_b[index as usize].clone() else {
                continue;
            };

            // Counted in the edges the USER picked, not in the pieces pass B re-found them as.
            from_b.edge_count = self.resolutions[index as usize].edges.len() as i32;

            let from_a = match results[index as usize].as_ref() {
                Some(from_a) if !from_a.failed() => from_a,
                _ => {
                    results[index as usize] = Some(from_b);
                    continue;
                }
            };

            if !from_b.failed() {
                continue;
            }

            // Pass A built part of this row and pass B refused the rest of it: the row BUILT, and
            // what pass B could not do is the loops' failure rather than the row's.
            let Some(advisory) = from_b.failure.clone() else {
                continue;
            };
            let mut failures = from_a.loop_failures.clone().unwrap_or_default();

            failures.push(BevelLoopFailure {
                advisory,
                largest_size_that_fits: from_b.largest_size_that_fits,
                edge_count: 0,
            });

            let mut merged = from_a.clone();
            merged.loop_failures = Some(failures);
            results[index as usize] = Some(merged);
        }
    }

    /// The loops of the first pass's leftovers found again on its OUTPUT
    /// mesh, by their endpoints' positions, as resolutions a second builder can be made over -
    /// or the loops an endpoint or an edge of which is not on that mesh (the lost loops).
    fn the_same_loops_on(
        &self,
        mesh_a: &Mesh,
        graph_a: &MeshEdgeGraph,
        for_legacy: &[Rc<RimLoop>],
    ) -> Result<Vec<BevelEdgeGroupResolution>, Vec<Rc<RimLoop>>> {
        let mut lost = Vec::new();

        let mut vertex_at: HashMap<(u32, u32, u32), i32> = HashMap::new();

        for (index, vertex) in mesh_a.vertices.iter().enumerate() {
            // Exact, because the publish copies an untouched vertex bit for bit. Two vertices at
            // one position would be a mesh that was never welded; the last index is kept.
            vertex_at.insert(position_key(*vertex), index as i32);
        }

        let mut edges_at_vertex: HashMap<i32, Vec<i32>> = HashMap::new();

        for edge in 0..graph_a.edge_count() {
            for end in 0..2 {
                let vertex = if end == 0 {
                    graph_a.get_vertex0(edge)
                } else {
                    graph_a.get_vertex1(edge)
                };

                edges_at_vertex.entry(vertex).or_default().push(edge);
            }
        }

        let mut edges_by_group: HashMap<i32, Vec<FeatureEdge>> = HashMap::new();

        for one in for_legacy {
            let mut found = Vec::new();
            let mut whole = true;
            let flat_angle_degrees = self.resolutions[one.group_index as usize]
                .features
                .flat_angle_degrees();

            for edge in &one.edges {
                let start = vertex_at.get(&position_key(Vector3Float::from_vector3(edge.start)));
                let end = vertex_at.get(&position_key(Vector3Float::from_vector3(edge.end)));
                let along_it = match (start, end) {
                    (Some(&start), Some(&end)) => {
                        Self::the_edges_from(mesh_a, graph_a, &edges_at_vertex, start, end)
                    }
                    _ => None,
                };

                let Some(along_it) = along_it else {
                    whole = false;
                    break;
                };

                for on_a in along_it {
                    let Some(classified) =
                        FeatureEdge::try_classify(mesh_a, graph_a, on_a, flat_angle_degrees)
                    else {
                        whole = false;
                        break;
                    };

                    if classified.convexity == EdgeConvexity::Flat {
                        // A piece of the picked edge the FIRST pass rounded away: no corner there
                        // to bevel and nothing to hand pass B.
                        continue;
                    }

                    found.push(classified);
                }

                if !whole {
                    break;
                }
            }

            if !whole {
                lost.push(one.clone());
                continue;
            }

            edges_by_group
                .entry(one.group_index)
                .or_default()
                .extend(found);
        }

        if !lost.is_empty() {
            return Err(lost);
        }

        Ok(self
            .resolutions
            .iter()
            .enumerate()
            .map(
                |(index, resolution)| match edges_by_group.remove(&(index as i32)) {
                    None => resolution.clone(),
                    // A succeeded resolution: no failure, no seed edge (-1), no chain or face group.
                    Some(edges) => BevelEdgeGroupResolution {
                        group: resolution.group.clone(),
                        features: Arc::new(FeatureEdgeSet::create_with_graph(
                            mesh_a,
                            graph_a,
                            resolution.features.flat_angle_degrees(),
                        )),
                        failure_reason: None,
                        seed_edge_index: -1,
                        edges,
                        chain: None,
                        face_group: None,
                    },
                },
            )
            .collect())
    }

    /// The mesh edges of the first pass's output that run from one of a picked
    /// edge's endpoints to the other - ONE of them where nothing touched that edge, and several
    /// where the first pass SPLIT it. The walk follows the straight line between the endpoints and
    /// refuses (`None`) rather than guesses where the step is not straight or overshoots.
    fn the_edges_from(
        mesh_a: &Mesh,
        graph_a: &MeshEdgeGraph,
        edges_at_vertex: &HashMap<i32, Vec<i32>>,
        start: i32,
        end: i32,
    ) -> Option<Vec<i32>> {
        let position = |vertex: i32| mesh_a.vertices[vertex as usize].as_vector3();
        let mut along_it = Vec::new();

        let target = position(end);
        let mut at = start;

        while at != end {
            let candidates = edges_at_vertex.get(&at)?;

            let here = position(at);
            let to_go = target - here;
            let left = to_go.length();

            if left == 0.0 {
                return None;
            }

            let heading = to_go / left;
            let mut stepped = -1;
            let mut stepped_length = f64::MAX;
            let mut next = -1;

            for &candidate in candidates {
                let other = if graph_a.get_vertex0(candidate) == at {
                    graph_a.get_vertex1(candidate)
                } else {
                    graph_a.get_vertex0(candidate)
                };
                let step = position(other) - here;
                let length = step.length();

                if length == 0.0 || length > left + (left * 1e-4) {
                    continue;
                }

                // Straight, to a tolerance a split vertex's own rounding cannot break.
                if step.dot(heading) / length < 0.99999 {
                    continue;
                }

                if stepped >= 0 {
                    if next == other {
                        // The same step twice - two faces' edges over one pair of vertices - is
                        // one step.
                        continue;
                    }

                    if length >= stepped_length {
                        // A longer way down the same line is the SPAN of the shorter one, so the
                        // SHORTEST step wins - it always lands on this line.
                        continue;
                    }
                }

                stepped = candidate;
                stepped_length = length;
                next = other;
            }

            if stepped < 0 {
                return None;
            }

            along_it.push(stepped);
            at = next;
        }

        if along_it.is_empty() {
            None
        } else {
            Some(along_it)
        }
    }

    /// The contributing loops whose plans the sections DECLINE,
    /// read one plan at a time on the working mesh. A loop is declined where ANY plan standing on
    /// one of its edges is declined. Empty with the flag off, and empty where nothing can be
    /// planned at all - in both cases the pass is built exactly as it is today.
    pub fn the_loops_the_sections_decline(
        &mut self,
        contributing: &[Rc<RimLoop>],
    ) -> Vec<Rc<RimLoop>> {
        let mut declined_loops = Vec::new();

        if !section_routing::use_sections() {
            return declined_loops;
        }

        let planned = self.try_plan_these_loops(contributing, self.size);

        if !planned.planned || planned.plans.is_empty() {
            return declined_loops;
        }

        let partition = section_routing::partition(
            &planned.plans,
            Some(&self.working_mesh),
            Some(&self.working_mesh_graph),
            None,
        );

        if partition.declined.is_empty() {
            return declined_loops;
        }

        let mut declined_edges = HashSet::new();

        for (plan, _) in &partition.declined {
            for edge in &planned.plans[*plan].edges {
                declined_edges.insert(edge.edge.edge.mesh_edge_index);
            }
        }

        for one in contributing {
            if !one.mesh_edges.is_disjoint(&declined_edges) {
                declined_loops.push(one.clone());
            }
        }

        declined_loops
    }
}
