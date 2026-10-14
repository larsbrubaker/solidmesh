//! The strips and their seams: the part of the surface bevel's build
//! between the walkback and the corners, split out of
//! [`super::surface_bevel_builder`] to keep each file under the length limit. In order: the
//! band corners per rail and per end ([`read_the_band_corners`]), one strip per edge the blend does
//! not swallow ([`try_create_the_strips`]), and each strip's two seams - the run the walkback left
//! on the landing face, the plane beyond where the band ran off the face whole, or nothing where a
//! mitre consumed that side - with the partial run-offs read beside them ([`try_pick_the_seams`]).

use std::collections::HashSet;
use std::rc::Rc;

use polygon_mesh::Mesh;

use super::surface_band_runs_off_the_face_beyond::{self as runs_off, BandSpan, ChainAtTheSpan};
use super::surface_bevel_builder::{is_swallowed, plan_of, the_strips_own_edge_is_to_blame};
use super::surface_mitre_joint_step::{SideBandCorners, SideSeams};
use super::surface_run_across::SurfaceRunAcrossRegionMapsCache;
use super::{
    surface_room_measure, surface_run_across_walk, surface_seam_over_the_edge,
    SurfaceBandRunsOffOverAStretch, SurfaceBevelPlan, SurfaceEdgePlan, SurfaceMitreConsumesTheRail,
    SurfaceStrip, SurfaceStripTrimmedByTheFaceBeyond, SurfaceVertexPool, SurfaceWalkbackResult,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// Which sides RAN OFF their landing face altogether, cut back along the
/// plane beyond it instead of sewn to a seam on it, indexed `[edge][side]`.
pub type SideRunOffs = [Option<Rc<SurfaceStripTrimmedByTheFaceBeyond>>; 2];

/// Which sides ran off only PART of their span, indexed `[edge][side]`.
pub type SideStretches = [Option<SurfaceBandRunsOffOverAStretch>; 2];

/// The walkback's band corners per rail AND PER END (`SurfaceWalkbackResult::band_corners`): a
/// mitre reads the corner of the foot's own end, so the two ends are kept apart here rather than
/// flattened and told apart afterwards by where they stand. The last index is 0 where the chain
/// ARRIVES at the edge and 1 where it LEAVES it.
///
/// Which of the cut's own ends is which is read off the end plane rather than off its position in
/// the cut's list, because an end the chain has none of - the free start of an open chain - is
/// simply absent and the one end a cut does have would then be read as the arriving one whatever
/// it is. A band end's Inward points from its corner INTO the chain, so it runs with the edge's
/// own direction at the arriving end and against it at the leaving one; at the mitre plane the dot
/// is (1 + cos turn) / |d1 + d2|, which only vanishes at the doubled-back corner TryMitre refuses
/// by name.
pub fn read_the_band_corners(
    walked: &SurfaceWalkbackResult,
    plans: &[Rc<SurfaceBevelPlan>],
    first_of_plan: &[i32],
    edges: &[SurfaceEdgePlan],
) -> Vec<SideBandCorners> {
    let count = edges.len();
    let mut band_corners: Vec<SideBandCorners> = vec![Default::default(); count];

    for (i, corners) in band_corners.iter_mut().enumerate() {
        let of_the_plans_edge = plan_of(first_of_plan, i as i32) as usize;

        for (side, corners) in corners.iter_mut().enumerate() {
            let Some(of_the_cut) = walked.band_corners.get(&(i as i32, side == 0)) else {
                continue;
            };

            let ends = plans[of_the_plans_edge].band_ends_of(
                (i as i32 - first_of_plan[of_the_plans_edge]) as usize,
                side == 0,
            );

            for end in 0..of_the_cut.len().min(ends.len()) {
                if of_the_cut[end].is_empty() {
                    continue;
                }

                let arriving = if ends[end].inward.dot(edges[i].edge.direction()) > 0.0 {
                    0
                } else {
                    1
                };

                match &mut corners[arriving] {
                    already @ None => *already = Some(of_the_cut[end].clone()),
                    Some(both) => {
                        // Two of the cut's ends reading as one end of the edge is not a shape
                        // anything here builds, but the corners are kept rather than dropped if it
                        // ever happens.
                        both.extend_from_slice(&of_the_cut[end]);
                    }
                }
            }
        }
    }

    band_corners
}

/// One strip per edge, `None` for an edge the blend SWALLOWS: it lies inside the surface that
/// replaces it, so it has no seam to pick either, and every stage after asks the same question once
/// (`SurfaceEdgeRoles`). Its band still comes off the faces: what stood there is under the blend's
/// own footprint.
pub fn try_create_the_strips(
    plans: &[Rc<SurfaceBevelPlan>],
    first_of_plan: &[i32],
    edges: &[SurfaceEdgePlan],
    segments_per_quarter: i32,
) -> Result<Vec<Option<Rc<SurfaceStrip>>>, BevelAdvisory> {
    let mut strips = vec![None; edges.len()];

    for (i, strip) in strips.iter_mut().enumerate() {
        if is_swallowed(plans, first_of_plan, i as i32) {
            continue;
        }

        *strip = Some(Rc::new(SurfaceStrip::try_create(
            &edges[i],
            segments_per_quarter,
        )?));
    }

    Ok(strips)
}

/// What the seam pick leaves for the corners and the ends.
pub struct PickedSeams {
    /// The seam each side is sewn to, `[edge][side]`.
    pub seams: Vec<SideSeams>,
    /// Which sides ran off their landing face altogether.
    pub ran_off: Vec<SideRunOffs>,
    /// Which sides ran off over part of their span.
    pub over_a_stretch: Vec<SideStretches>,
}

/// The two seams of every strip, each edge and each side in turn, after the run-out survey.
#[allow(clippy::too_many_arguments)]
pub fn try_pick_the_seams(
    mesh: &Mesh,
    pool: &mut SurfaceVertexPool,
    walked: &SurfaceWalkbackResult,
    plans: &[Rc<SurfaceBevelPlan>],
    first_of_plan: &[i32],
    edges: &[SurfaceEdgePlan],
    strips: &[Option<Rc<SurfaceStrip>>],
    consumed: &[Option<SurfaceMitreConsumesTheRail>],
) -> Result<PickedSeams, BevelAdvisory> {
    let count = edges.len();
    let mut seams: Vec<SideSeams> = vec![Default::default(); count];
    let mut ran_off: Vec<SideRunOffs> = vec![Default::default(); count];
    let mut over_a_stretch: Vec<SideStretches> = vec![Default::default(); count];

    // Which mesh edges this pass is beveling and where they end, for the readings that have to
    // tell the chain apart from everything else the bands reach - the run-off below is one, the
    // walkback's own standing-feature reading is the other.
    let mut beveled_mesh_edges = HashSet::new();
    let mut chain_vertices = HashSet::new();

    for plan in plans {
        for edge in &plan.edges {
            beveled_mesh_edges.insert(edge.edge.edge.mesh_edge_index);
            chain_vertices.insert(edge.edge.edge.start_vertex);
            chain_vertices.insert(edge.edge.edge.end_vertex);
        }
    }

    // One reading of each curved landing's facets, shared by every side that asks: the same cache
    // the room keeps, for the same question - is this landing one the closed forms answer for at
    // all.
    let mut across_curved_regions = SurfaceRunAcrossRegionMapsCache::new();

    for i in 0..count {
        if is_swallowed(plans, first_of_plan, i as i32) {
            continue;
        }

        let Some(strip) = strips[i].as_deref() else {
            continue;
        };
        let edge = &edges[i];

        for left in [true, false] {
            let side = if left { 0 } else { 1 };

            let landing = if left {
                &edge.left_region
            } else {
                &edge.right_region
            };
            let span = BandSpan {
                from: edge.from(),
                along: edge.edge.direction(),
                length: (edge.to() - edge.from()).length(),
                inward: if left {
                    edge.left_inward_at_the_edge
                } else {
                    edge.right_inward_at_the_edge
                }
                .get_normal(),
                setback: if left {
                    edge.left_setback
                } else {
                    edge.right_setback
                },
            };

            // A band that took the WHOLE of its landing face has no seam on it to pick: the region
            // was given up entire, and what bounds the strip on that side is the plane the face
            // runs off onto. Asked before the runs rather than after they come up empty, because
            // that is the honest order - the reading says there is nothing to sew to, and the trim
            // says what stands there instead.
            match runs_off::try_read(
                Some(mesh),
                Some(landing),
                &span,
                &beveled_mesh_edges,
                &chain_vertices,
            ) {
                Ok(Some(beyond)) => {
                    let trim = SurfaceStripTrimmedByTheFaceBeyond::try_read(
                        pool,
                        strip,
                        edge,
                        side as i32,
                        beyond.plane_point,
                        beyond.plane_normal,
                        beyond.trims_the_start,
                    )
                    .map_err(|failure| the_strips_own_edge_is_to_blame(edges, i, failure))?;

                    seams[i][side] = Some(trim.seam.clone());
                    ran_off[i][side] = Some(Rc::new(trim));
                    continue;
                }
                Ok(None) => {}
                Err(ran_off_why) => {
                    return Err(the_strips_own_edge_is_to_blame(edges, i, ran_off_why));
                }
            }

            let runs = walked.tangent_runs.get(&(i as i32, left));

            if let Some(reading) = &consumed[i] {
                if reading.side == side as i32 {
                    if runs.is_some_and(|runs| !runs.is_empty()) {
                        // The two readings are of one thing and disagree: the mitres have eaten
                        // this rail's whole stretch of the tangent line, and the walkback left a
                        // stretch of it standing on the face anyway.
                        let failure = BevelAdvisory::new(
                            BevelAdvisoryCode::SurfaceMitreConsumesTheStrip,
                            vec![edge.from().into()],
                        );
                        return Err(the_strips_own_edge_is_to_blame(edges, i, failure));
                    }

                    continue;
                }
            }

            // Windowed on the band's own ends rather than on the edge's corners: where the two
            // beads at a mitre set back by different amounts the mitre point stands past the
            // corner, and on a loop whose setback steps round it a band can lie clear of its own
            // chord entire (SurfaceSeamOverTheEdge).
            let of_the_plan = plan_of(first_of_plan, i as i32) as usize;
            let in_its_chain = (i as i32 - first_of_plan[of_the_plan]) as usize;
            let band_ends = plans[of_the_plan].band_ends_of(in_its_chain, left);

            let seam = surface_seam_over_the_edge::try_pick_the_run_over_the_edge(
                pool,
                strip,
                edge,
                runs.map(|runs| runs.as_slice()),
                Some(&band_ends),
                left,
            )
            .map_err(|refused_at| {
                // Reported where the reading really failed - the nearest end of the nearest run, or
                // the middle of the edge where the face gave up over all of it - rather than at the
                // edge's own first corner, which on a chain of several edges is a corner the run
                // has nothing to do with and pins every size's advisory to one arbitrary place.
                let failure = BevelAdvisory::new(
                    BevelAdvisoryCode::SurfaceStripSeamDidNotMeet,
                    vec![refused_at.into()],
                );
                the_strips_own_edge_is_to_blame(edges, i, failure)
            })?;

            seams[i][side] = Some(seam);

            // And where the band ran off only PART of its span, at one free end of the chain
            // (SurfaceBandRunsOffOverAStretch): the seam is the run the walkback left, which
            // simply stops where the face died away, and what ends the strip there is the part's
            // own HULL rather than any face of it. Read here beside the whole-span run-off and off
            // the same reading, and under the same gate the ROOM reads it under - a landing paced
            // out across its own curved surface is not one this closed form answers for on either
            // side of the pass (surface_room_measure::room_across).
            if surface_run_across_walk::try_across_a_curved_region(
                mesh,
                landing,
                edge.from(),
                edge.to(),
                edge.edge.direction(),
                span.inward,
                Some(&mut across_curved_regions),
            )
            .is_some()
            {
                continue;
            }

            let chain = ChainAtTheSpan {
                convexity: edge.convexity(),
                free_at_the_start: surface_room_measure::free_end(
                    &plans[of_the_plan],
                    in_its_chain,
                    true,
                ),
                free_at_the_end: surface_room_measure::free_end(
                    &plans[of_the_plan],
                    in_its_chain,
                    false,
                ),
            };

            match runs_off::try_read_a_stretch(
                Some(mesh),
                Some(landing),
                &span,
                chain,
                &beveled_mesh_edges,
                &chain_vertices,
            ) {
                // Stamped with the edge's index in its OWN chain, which is what the stretch says
                // it carries and what the room stamps: the two are compared after, and a pass of
                // several chains numbers its edges end to end here and from zero there.
                Ok(Some(over_the_stretch)) => {
                    over_a_stretch[i][side] = Some(over_the_stretch.on(
                        of_the_plan as i32,
                        in_its_chain as i32,
                        side as i32,
                    ));
                }
                Ok(None) => {}
                Err(stretch_why) => {
                    return Err(the_strips_own_edge_is_to_blame(edges, i, stretch_why));
                }
            }
        }
    }

    Ok(PickedSeams {
        seams,
        ran_off,
        over_a_stretch,
    })
}
