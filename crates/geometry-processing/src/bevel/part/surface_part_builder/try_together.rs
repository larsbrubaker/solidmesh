// ONE attempt
// at building a pass's loops together - the level line first where it is on, then the plans, the
// room and the one build - and, where it does not build, why not and which mesh edges of the pick
// are to blame. The one pass (`build`) drops and retries what this blames; the size search
// (`size_that_fits`) asks it at smaller sizes; the existing construction past the level line's
// room (`past_the_level_lines_room`) asks it again with the level line set aside. The planning it
// shares with the pass in two is `pass_in_two::try_plan_these_loops`.

use std::collections::HashSet;
use std::rc::Rc;

use crate::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, surface_room, surface_run_stops_short,
    SurfaceBandRunsOffOverAStretch, SurfaceBuildFailure, SurfaceBuildOptions,
};
use crate::bevel::{
    BevelAdvisory, BevelAdvisoryArg, BevelAdvisoryCode, BevelGroupResult, BevelResultGroup,
};
use polygon_mesh::{CancelToken, Mesh};

use super::{BevelSurfacePartBuilder, RimLoop, SurfacePartBuildError, TheChainCondemned};

/// What one attempt came to: the mesh where it built, and why not where it did not.
#[derive(Default)]
pub struct PassAttempt {
    /// The mesh where it built.
    pub mesh: Option<Mesh>,
    /// The mesh edges the refusal is ABOUT, which the caller maps back to loops. Empty where the
    /// refusal names none, which makes it the whole pass's.
    pub guilty: HashSet<i32>,
    /// Why it did not build, or `None` where it built or says nothing.
    pub reason: Option<BevelAdvisory>,
    /// Whether the refusal is one to put through `bevel_group_failures::diagnose`, which reads a
    /// size that does not fit as a coarser question about the pick.
    pub about_the_size: bool,
    /// The loops a settling (or a touch) named, where any were named.
    pub condemned: Option<Vec<TheChainCondemned>>,
}

impl BevelSurfacePartBuilder {
    /// One attempt at building these loops together at `trial_size`,
    /// which is the operation's own everywhere but in the size search - that asks this same
    /// question of one group at smaller and smaller numbers to find the largest one it answers yes
    /// to.
    pub fn try_these_groups_together(
        &mut self,
        contributing: &[Rc<RimLoop>],
        results: &mut [Option<BevelGroupResult>],
        trial_size: f64,
        cancellation_token: &CancelToken,
    ) -> Result<PassAttempt, SurfacePartBuildError> {
        let mut attempt = PassAttempt::default();

        // The level line first, where the opt-in is on and the pass's picked edges link into runs -
        // closed loops or open runs alike. It either builds the whole pass, refuses it by SIZE with
        // the room it measured - which, where the pick's own room was read off a feature edge
        // beyond its face, the existing construction is asked to build past - or says nothing at
        // all and the pass goes on below exactly as it does today.
        if self.routing.level_line && !self.the_level_line_is_set_aside {
            let answer = self.try_the_level_line(contributing, trial_size, cancellation_token)?;
            attempt.condemned = answer.condemned;

            if let Some(mut level_line_mesh) = answer.mesh {
                self.carry_the_colours_over(&mut level_line_mesh);
                self.write_the_rows_that_built(contributing, results);
                attempt.mesh = Some(level_line_mesh);

                return Ok(attempt);
            }

            if let Some(level_line_refusal) = answer.reason {
                attempt.guilty = self.the_edges_the_refusal_names();
                attempt.reason = Some(level_line_refusal);
                attempt.about_the_size = answer.about_the_size;

                if self.goes_past_the_level_lines_room(answer.about_the_size) {
                    attempt.mesh = self.the_existing_construction_past_the_level_lines_room(
                        contributing,
                        results,
                        trial_size,
                        cancellation_token,
                        &mut attempt.guilty,
                        &mut attempt.reason,
                    )?;
                }

                return Ok(attempt);
            }

            // And a many-loop rim the level line refused because two of its closed loops touch
            // leaves one of each touching pair alone here, rather than going on below to the older
            // construction: that one refuses a loop of its own choosing, unrelated to the touch,
            // and the round repeats until the pair happens to go (Many Holes: 21 innocent port
            // mouths dropped).
            if !attempt
                .condemned
                .as_ref()
                .is_some_and(|named| !named.is_empty())
            {
                attempt.condemned = self.the_touching_loops_left_alone(contributing);
            }

            if attempt.condemned.is_some() {
                // The settling named everybody who cannot stay at this size. The caller writes
                // their rows and drops them all at once, and the pass after that builds the rest.
                return Ok(attempt);
            }
        }

        let planned = self.try_plan_these_loops(contributing, trial_size);
        attempt.guilty = planned.guilty;
        attempt.reason = planned.reason;

        let Some(regions) = planned.regions.filter(|_| planned.planned) else {
            return Ok(attempt);
        };
        let plans = planned.plans;

        if plans.is_empty() {
            return Ok(attempt);
        }

        let group_indices = Self::the_groups_of(contributing);

        // ROOM, and the plan's rung 8: the size was asked for, so a size that does not fit is
        // refused with the size that would have - never quietly shrunk to it. Measured across the
        // regions the plans actually land on, with the chains sharing a region negotiating it
        // between them. Asked AFTER the plans are drawn because the plans are what say which
        // regions those are and at what wedge.
        //
        // The stretches a band runs off part of its landing over are taken back and HANDED TO THE
        // BUILD, which reads them again off the same plans and refuses the pass by name where the
        // two readings do not come to the same set.
        let mut stretches: Vec<SurfaceBandRunsOffOverAStretch> = Vec::new();
        let fit =
            surface_room::try_fit(&plans, trial_size, cancellation_token, Some(&mut stretches))?;

        if !fit.fits {
            attempt.about_the_size = true;

            if let Some(does_not_fit) = fit.refusal.as_ref() {
                if does_not_fit.mesh_edge_index() != BevelAdvisory::NO_MESH_EDGE {
                    attempt.guilty.insert(does_not_fit.mesh_edge_index());
                }
            }

            attempt.reason = fit.refusal;

            return Ok(attempt);
        }

        // The blend's own faces come back with the mesh so that a SECOND pass over this output can
        // fence its floods off them - an index list into this mesh's face list, which stays
        // meaningful because nothing on this route reorders or drops a face.
        let built = surface_bevel_builder::try_build(
            &self.working_mesh,
            &self.working_mesh_graph,
            &regions,
            &plans,
            self.segments,
            cancellation_token,
            SurfaceBuildOptions {
                crossings: Some(&fit.crossings),
                ran_off_over_a_stretch: Some(&stretches),
                use_sections: self.routing.sections,
                ..SurfaceBuildOptions::default()
            },
        );

        let built = match built {
            Ok(built) => built,
            Err(SurfaceBuildFailure::Cancelled) => return Err(SurfacePartBuildError::Cancelled),
            Err(SurfaceBuildFailure::NotANumber(_)) => {
                return Err(SurfacePartBuildError::NotANumber)
            }
            Err(failure) => {
                let build_refused = Self::the_refusal_a_build_gives(&failure);

                // Scoped exactly as the readings above are: a refusal raised while one STRIP was
                // being read is that strip's - the builder stamps it with the picked edge the
                // strip stands on - and a strip belongs to whoever picked its edge. What names no
                // edge but names a place is charged to the loop nearest it; what names neither is
                // everybody's.
                if build_refused.mesh_edge_index() != BevelAdvisory::NO_MESH_EDGE {
                    attempt.guilty.insert(build_refused.mesh_edge_index());
                } else if let Some(nearest) = self
                    .the_loop_nearest_where_it_says_it_happened(contributing, Some(&build_refused))
                {
                    attempt.guilty.insert(nearest);
                }

                attempt.reason = Some(build_refused);

                return Ok(attempt);
            }
        };

        // Read on the answer, because no step of the build can see it: a construction that shares
        // every seam vertex as it makes it either closes or has a defect in it, and a defect
        // published is a part with a crack or a fold in it. Refused here, and charged to the LOOP
        // NEAREST WHERE THE HOLE STANDS by the same clear-winner reading every other place-only
        // refusal is scoped by: a dropped loop is dropped and the pass RE-RUN without it, so if
        // what is left then closes the reading was right.
        let mut mesh = built.mesh;
        let (closed, open_at) = surface_mesh_closure::is_closed(&mesh);

        if !closed {
            let reason = BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceResultNotClosed,
                vec![BevelAdvisoryArg::Vector3(open_at)],
            );

            if let Some(nearest_to_the_hole) =
                self.the_loop_nearest_where_it_says_it_happened(contributing, Some(&reason))
            {
                attempt.guilty.insert(nearest_to_the_hole);
            }

            attempt.reason = Some(reason);

            return Ok(attempt);
        }

        self.blend_faces_of_the_last_build = built.blend_faces;

        self.carry_the_colours_over(&mut mesh);

        for index in group_indices {
            // Built, and with one thing left to say: a free end that stopped SQUARE because the
            // crease carried on past what was picked. Counted in the group's own edges, and read
            // about THIS group's own runs - of the loops of it still contributing - only.
            let edge_count = self.resolutions[index as usize].edges.len() as i32;
            let mut the_groups_mesh_edges = HashSet::new();

            for one in contributing {
                if one.group_index == index {
                    the_groups_mesh_edges.extend(one.mesh_edges.iter().copied());
                }
            }

            let said = surface_run_stops_short::read(
                Some(&plans),
                edge_count,
                Some(&the_groups_mesh_edges),
            )
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

        attempt.mesh = Some(mesh);

        Ok(attempt)
    }

    /// The source's per-face colours carried onto the built mesh, the
    /// same way the boolean path carries them: by nearest centroid. Read in the working mesh's own
    /// space, before the item matrix carries the result out of it, so the two sets of centroids
    /// are measured in one frame. An unpainted source has nothing to carry and costs nothing.
    pub fn carry_the_colours_over(&self, mesh: &mut Mesh) {
        let saved = self.working_mesh.save_face_centroid_colors();
        mesh.restore_face_colors_from_centroids(saved.as_deref());
    }

    /// The advisory a refused build is written with: its own, or - where the sections route
    /// declined with nothing named - the construction not building, which names no edge and no
    /// place, so it is everybody's: a decline must refuse the rows, never end the rebuild.
    pub fn the_refusal_a_build_gives(failure: &SurfaceBuildFailure) -> BevelAdvisory {
        failure.advisory().cloned().unwrap_or_else(|| {
            BevelAdvisory::new(BevelAdvisoryCode::SurfaceConstructionNotBuilt, vec![])
        })
    }
}
