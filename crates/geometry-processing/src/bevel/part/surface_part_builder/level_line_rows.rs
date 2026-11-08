// The level line's result rows - the row per group
// of a pass it built, the stop-square sentence for an end-capped run, and which chain (and so
// which loop) a level-line size refusal is about, with the refusal's room remembered as the
// pick's floor in the memo (`level_line_memo`). The level-line route (`level_line`) sets the pass's chains
// and calls these; the rows are the kernel's `BevelGroupResult`.

use std::collections::HashSet;

use crate::bevel::bevel_chain_linker::BevelSweptChain;
use crate::bevel::level_line::{BandGapResult, BandProfile, RoomReadingKind, RoomSource};
use crate::bevel::surface::{
    surface_chain_join, surface_shared_edge_network, SurfaceChainEndReading,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode, BevelGroupResult, BevelResultGroup};
use crate::topology::BevelEdgeGroupResolution;
use std::rc::Rc;

use super::level_line_memo::THE_ROUTES_SNAP;
use super::{BevelSurfacePartBuilder, RimLoop};

impl BevelSurfacePartBuilder {
    /// The row per group of a pass the level line built - the
    /// existing path's own tail. The short-run sentence is said only of an open run whose ends
    /// were closed by END-FACE caps, and only where its crease carries on past the pick
    /// ([`Self::the_run_stops_short`]). A closed loop has no free end, and a FADE does not stop
    /// square.
    pub fn write_the_rows_that_built(
        &mut self,
        contributing: &[Rc<RimLoop>],
        results: &mut [Option<BevelGroupResult>],
    ) {
        // Nothing is fenced off for a second pass over this output yet: the level line does not
        // mark the faces it laid, so it reports none rather than the previous build's.
        self.blend_faces_of_the_last_build = Vec::new();

        let group_indices = Self::the_groups_of(contributing);
        for &index in &group_indices {
            let edge_count = self.resolutions[index as usize].edges.len() as i32;
            let mut the_groups_mesh_edges = HashSet::new();
            for one in contributing {
                if one.group_index == index {
                    the_groups_mesh_edges.extend(one.mesh_edges.iter().copied());
                }
            }

            let said = self
                .the_run_stops_short(&group_indices, index, &the_groups_mesh_edges, edge_count)
                .unwrap_or_default();

            let mut row = BevelGroupResult::new(
                index,
                BevelResultGroup::Edge(self.groups[index as usize].clone()),
                edge_count,
                None,
                said,
            );
            row.loop_count = self.the_rim_loops_of_the_group(index).len() as i32;
            results[index as usize] = Some(row);
        }
    }

    /// The chain a size refusal is about, or -1, with the
    /// refusal's room remembered as the pick's floor - read first, so a refusal answered from that
    /// floor names what the floor's own pinch named.
    ///
    /// A PINCH names the side that got no loop. A build refused from a floor a pinch left names
    /// that pinch's chain again: read any other way, a drag that had pinched and dropped the short
    /// block's foot refused every loop of the rim at the next size. A room read off the width names
    /// a chain only where it is FOREIGN - one loop's own shortfall: dropped, the rest of the pick
    /// builds. A width reading of two chains closing on each other, or of a side several chains
    /// share, is a collision and stays the whole pass's.
    #[allow(clippy::too_many_arguments)]
    pub fn the_chain_named_with_its_room_remembered(
        &mut self,
        gap: &BandGapResult,
        room_set_by: RoomSource,
        pick: &[Vec<i32>],
        closed: &[bool],
        profile: BandProfile,
        known_room: Option<f64>,
        trial_size: f64,
    ) -> i32 {
        let key = self.the_memo_key_of(pick, closed, profile, THE_ROUTES_SNAP);
        let named = Self::the_chain_the_refusal_names(
            gap,
            &room_set_by,
            self.memo
                .the_pinched_chain_under_the_floor(&key, known_room, Some(trial_size)),
        );

        self.remember_a_smaller_room(
            pick,
            closed,
            profile,
            gap.room,
            room_set_by,
            Self::the_chain_the_pinch_named(gap),
            Some(trial_size),
        );

        named
    }

    /// The chain this build's own pinch named, or -1 where it did not
    /// pinch or its reading named nobody - never the handed-in room's chain, which did not pinch
    /// here.
    pub fn the_chain_the_pinch_named(gap: &BandGapResult) -> i32 {
        if gap.pinch_probes > 0
            && matches!(
                gap.room_set_by.kind,
                RoomReadingKind::SelfRoom | RoomReadingKind::ChainToChain
            )
        {
            gap.room_set_by.chain
        } else {
            -1
        }
    }

    /// The chain a refusal names - see [`Self::the_chain_named_with_its_room_remembered`].
    pub fn the_chain_the_refusal_names(
        gap: &BandGapResult,
        room_set_by: &RoomSource,
        the_floors_pinched_chain: i32,
    ) -> i32 {
        if gap.pinch_probes > 0 {
            return Self::the_chain_the_pinch_named(gap);
        }

        if gap.room_set_by.kind == RoomReadingKind::None && the_floors_pinched_chain >= 0 {
            return the_floors_pinched_chain;
        }

        if room_set_by.kind == RoomReadingKind::Foreign {
            room_set_by.chain
        } else {
            -1
        }
    }

    /// The mesh edges the last level-line size refusal is ABOUT -
    /// the named chain's, so the caller drops that one loop and builds the rest, and none where
    /// the refusal named no chain, which leaves it the whole pass's.
    pub fn the_edges_the_refusal_names(&self) -> HashSet<i32> {
        match &self.the_chain_edges_of_the_pass {
            Some(edges)
                if self.the_chain_the_refusal_names >= 0
                    && (self.the_chain_the_refusal_names as usize) < edges.len() =>
            {
                edges[self.the_chain_the_refusal_names as usize].clone()
            }
            _ => HashSet::new(),
        }
    }

    /// The existing construction's sentence for a run that stops SQUARE
    /// while its crease carries on past what was picked, said of the level line's end-capped run.
    /// The carrying on is read by the existing construction's own reading
    /// (`SurfaceChainEndReading`) on the pass's chains linked and joined as the existing
    /// construction links them, at the curve tolerance; a crease that ENDS at the end face (a
    /// cube's edge) is not a short cut. `None` where there is nothing to say.
    pub fn the_run_stops_short(
        &mut self,
        group_indices: &[i32],
        index: i32,
        the_groups_mesh_edges: &HashSet<i32>,
        edge_count: i32,
    ) -> Option<Vec<BevelAdvisory>> {
        let capped = self.the_end_capped_chain.as_ref()?;
        if capped.len() < 2 {
            return None;
        }

        let from = capped[0];
        let to = capped[capped.len() - 1];
        let picked: Vec<BevelEdgeGroupResolution> = group_indices
            .iter()
            .map(|&i| self.resolutions[i as usize].clone())
            .collect();
        let linked = surface_chain_join::join(&surface_shared_edge_network::link_together(&picked));
        let chain: &BevelSweptChain = linked.iter().find(|c| {
            !c.is_closed
                && c.edges
                    .iter()
                    .any(|e| the_groups_mesh_edges.contains(&e.edge.mesh_edge_index))
                && !c.edges.is_empty()
                && ((c.edges[0].from_vertex() == from
                    && c.edges[c.edges.len() - 1].to_vertex() == to)
                    || (c.edges[0].from_vertex() == to
                        && c.edges[c.edges.len() - 1].to_vertex() == from))
        })?;

        let tolerance = self.resolutions[index as usize]
            .group
            .curve_tolerance_degrees;
        let regions = self.regions_at(tolerance);
        let ends = SurfaceChainEndReading::for_regions(Some(&regions))?;

        let first = &chain.edges[0];
        let last = &chain.edges[chain.edges.len() - 1];
        let start_carries_on = ends.runs_out_at(first, first.from_vertex());
        let end_carries_on = ends.runs_out_at(last, last.to_vertex());
        if !start_carries_on && !end_carries_on {
            return None;
        }

        // One line for the run and not one per end, as SurfaceRunStopsShort says it.
        Some(vec![if start_carries_on && end_carries_on {
            BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceRunStopsSquareAtBothEnds,
                vec![first.from().into(), last.to().into(), edge_count.into()],
            )
        } else {
            BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceRunStopsSquareAtOneEnd,
                vec![
                    if start_carries_on {
                        first.from()
                    } else {
                        last.to()
                    }
                    .into(),
                    edge_count.into(),
                ],
            )
        }])
    }
}
