//! Where a strip STOPS: the ring at an open end and the notch it cuts in the face beyond, and the
//! gathering of every free end of every chain into the corners they stand at. Ports
//! `Bevel/Surface/SurfaceStripEnds.cs` (C# `internal static class`; a module of free functions
//! here): the one ruler every end reads its seam with ([`meets_the_seam`]), the gathering of every
//! free end into the corners it stands at ([`try_close_the_ends`]), the run-off stretch at an end
//! ([`try_the_stretch_at`]), the junction at a vertex ([`the_junction_at`]) and its closing
//! ([`try_close_the_junction`]), and the open end ([`try_open_end`]). The third way a strip stops -
//! against the next strip of its own chain - is `SurfaceStripMitre`.

use std::rc::Rc;

use vector_math::Vector3;

use super::surface_mitre_joint_step::SideSeams;
use super::{
    surface_corner_patch, surface_end_cut, surface_end_cut_above_the_crossing,
    surface_mixed_junction_patch, surface_strip_ends_at_a_landing_notch,
    surface_strip_ends_over_the_face_beyond, SurfaceBandRunsOffOverAStretch, SurfaceBevelPlan,
    SurfaceCornerPatchArrival, SurfaceEdgePlan, SurfaceMixedJunction,
    SurfaceMixedJunctionPatchArrival, SurfaceStrip, SurfaceStripEndsOnTheHull,
    SurfaceStripEndsOnTheHullSupportingPlane, SurfaceStripNotchedByAFeature, SurfaceStripNotches,
    SurfaceStripStation, SurfaceStripTrimmedAtTheCrossing, SurfaceStripTrimmedByTheFaceBeyond,
    SurfaceStripTrims, SurfaceVertexPool,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};
use crate::topology::EdgeConvexity;

/// C# `MeetsTheSeam`: how near two points of the seam have to come to be the same point - the
/// pool's own [`SurfaceVertexPool::near_enough`], which is ten times its weld because these are
/// float positions read back and crossed with planes, and follows the part's size because a float's
/// ulp does. A seam that is a rounding out is a crack either way.
pub fn meets_the_seam(pool: &SurfaceVertexPool) -> f64 {
    pool.near_enough()
}

/// One free end of one chain, as [`try_close_the_ends`] gathers them (C#'s tuple).
#[derive(Debug, Clone, Copy)]
struct ChainEndAt {
    /// Which plan of the pass the chain is.
    plan: usize,
    /// The build's own index of the edge whose strip closes this end.
    edge: usize,
    /// Whether the end is the chain's START.
    at_the_start: bool,
    /// The mesh's own index of the chain end's vertex - what the ends of a corner are gathered by.
    vertex: i32,
    /// Where that vertex stands.
    corner: Vector3,
    /// Whether the strip closing this end is a junction's cut further along the chain.
    the_cut_is_further_along: bool,
}

/// C# `TryCloseTheEnds`: every free end of every open chain, closed - alone against the face beyond
/// it, or three at a time into a corner patch. Every arrival at a vertex is a chain end, a chain
/// that ran through one having been cut there before it was planned.
///
/// The C# arrays map as: `strips` one per build edge (`None` where a blend swallowed it), `seams`
/// C#'s `[edge, side]` runs, `start_rings`/`end_rings` the stations each end is handed (`None`, C#'s
/// null, until closed), and `trimmed`/`ran_off`/`notched`/`over_a_stretch` C#'s `[edge, side]`
/// arrays, `None` where the build has none. Where C# would throw on a missing seam, the end is
/// refused as a corner not supported.
#[allow(clippy::too_many_arguments)]
pub fn try_close_the_ends(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    plans: &[Rc<SurfaceBevelPlan>],
    junctions: Option<&[Rc<SurfaceMixedJunction>]>,
    edges: &[SurfaceEdgePlan],
    first_of_plan: &[i32],
    strips: &[Option<Rc<SurfaceStrip>>],
    seams: &[SideSeams],
    start_rings: &mut [Option<SurfaceStripStation>],
    end_rings: &mut [Option<SurfaceStripStation>],
    trimmed: Option<&SurfaceStripTrims>,
    ran_off: Option<&[[Option<Rc<SurfaceStripTrimmedByTheFaceBeyond>>; 2]]>,
    notched: Option<&SurfaceStripNotches>,
    over_a_stretch: Option<&[[Option<SurfaceBandRunsOffOverAStretch>; 2]]>,
    hull: &[SurfaceStripEndsOnTheHullSupportingPlane],
) -> Result<(), BevelAdvisory> {
    // Gathered by the mesh's OWN vertex index rather than by how near two ends stand: the mesh the
    // plans were made on is welded, so two chains that end at one vertex of the part end at one
    // index, and that is exact. Grouping by position would not be - a nearness test is not
    // transitive, so which ends came out together would depend on which one seeded the group.
    let mut ends: Vec<ChainEndAt> = Vec::new();

    for (p, plan) in plans.iter().enumerate() {
        if plan.is_closed {
            continue;
        }

        let first = first_of_plan[p] as usize;
        let last = first + plan.edges.len() - 1;

        for at_the_start in [true, false] {
            let chain_end = if at_the_start { first } else { last };
            let mut end = chain_end;

            // An end edge a blend SWALLOWS has no strip to close. Where the blend is a JUNCTION's,
            // what stands at that end of the chain is the edge that really carries the corner's
            // ring - further along the chain with the swallowed chords between - so the strip that
            // closes here is asked of the roles rather than assumed to be the end edge.
            if strips[end].is_none() {
                if let Some(carries) = plan
                    .roles()
                    .try_the_edge_carrying_the_ring_at(SurfaceBevelPlan::junction_key(at_the_start))
                {
                    end = first + carries as usize;
                }
            }

            if strips[end].is_none() {
                continue;
            }

            // Named by the CHAIN's own end whatever strip closes it, because that is the vertex the
            // three ends of a corner are gathered by.
            ends.push(ChainEndAt {
                plan: p,
                edge: end,
                at_the_start,
                vertex: if at_the_start {
                    edges[first].edge.from_vertex()
                } else {
                    edges[last].edge.to_vertex()
                },
                corner: if at_the_start {
                    edges[first].from()
                } else {
                    edges[last].to()
                },
                the_cut_is_further_along: end != chain_end,
            });
        }
    }

    // An end a standing feature's extension NOTCHES is closed AFTER every other end, because the
    // cross section its strip stops at reads the strip's other end's own ring. A stable partition,
    // so nothing but the notched ends moves.
    let notched_at = |end: &ChainEndAt| {
        SurfaceStripNotchedByAFeature::standing_at(notched, end.edge as i32, end.at_the_start)
    };
    let (mut ends, notched_last): (Vec<ChainEndAt>, Vec<ChainEndAt>) =
        ends.into_iter().partition(|end| notched_at(end).is_none());
    ends.extend(notched_last);

    let mut closed = vec![false; ends.len()];

    // What the mesh looked like before ANY end was closed, so a reading that asks which faces stand
    // at a vertex asks of the part rather than of whatever the ends closed so far.
    let faces_before_the_ends = faces.len();

    let seam_end = |edge: usize, side: usize, at_the_start: bool| -> Option<i32> {
        let run = seams.get(edge)?[side].as_ref()?;
        let end = if at_the_start {
            run.first()
        } else {
            run.last()
        };
        end.map(|&(vertex, _)| vertex)
    };

    for at in 0..ends.len() {
        if closed[at] {
            continue;
        }

        let mut together = vec![at];
        for other in at + 1..ends.len() {
            if !closed[other] && ends[other].vertex == ends[at].vertex {
                together.push(other);
            }
        }

        for &one in &together {
            closed[one] = true;
        }

        if together.len() == 1 {
            let end = ends[at];
            let strip = strips[end.edge]
                .as_ref()
                .expect("an end is only gathered where its strip stands");
            let edge = &edges[end.edge];
            let edge_index = end.edge as i32;

            if end.the_cut_is_further_along {
                // A JUNCTION's own cut, several chords along the chain, under the chain end's own
                // vertex - which only the junction patch can close, and that wants all three ends.
                return Err(BevelAdvisory::new(
                    BevelAdvisoryCode::SurfaceCornerNotSupported,
                    vec![end.corner.into()],
                ));
            }

            // A groove that stops in the middle of a run closes against a wall of its own, facing
            // the way the chain goes at the start of it and back the way it came at the finish.
            let plan = &plans[end.plan];
            let runs_out = if end.at_the_start {
                plan.start_runs_out
            } else {
                plan.end_runs_out
            };
            let runs_out_facing = if !runs_out {
                Vector3::ZERO
            } else if end.at_the_start {
                strip.along
            } else {
                -strip.along
            };

            // A band another chain's RUN-OUT crosses closes on the crossing rather than on the face
            // beyond the corner (`SurfaceEndCutAboveTheCrossing`).
            let crossed = SurfaceStripTrimmedAtTheCrossing::standing_at(
                trimmed,
                edge_index,
                end.at_the_start,
            );

            // A band that RAN OFF its landing face closes on the trim rather than on a seam.
            let off_the_face = surface_strip_ends_over_the_face_beyond::try_the_run_off_on(
                ran_off, edge_index, end.corner,
            )?;

            let crossed_here = crossed
                .as_ref()
                .is_some_and(|crossed| crossed.at_the_start == end.at_the_start);

            if off_the_face.is_some() && crossed_here {
                // Two curves want this one end ring; named rather than built on whichever was asked
                // first.
                return Err(BevelAdvisory::new(
                    BevelAdvisoryCode::SurfaceRunOffMeetsACrossing,
                    vec![end.corner.into()],
                ));
            }

            // A band a standing FEATURE was undercut by closes on the notch that feature takes out
            // of its strip.
            let notch = notched_at(&end);

            // A band that ran off PART of its landing at THIS end is ended by the part's own hull.
            // Asked BEFORE the crossing is told it closed this end, so a refusal here leaves
            // nothing behind it.
            let on_the_hull =
                try_the_stretch_at(over_a_stretch, end.edge, end.at_the_start, end.corner)?;

            if on_the_hull.is_some() && (off_the_face.is_some() || notch.is_some() || crossed_here)
            {
                // Two readings for one end ring, named for the reading that really stands here.
                let code = if crossed_here {
                    BevelAdvisoryCode::SurfaceRunOffMeetsACrossing
                } else if notch.is_some() {
                    BevelAdvisoryCode::SurfaceStandingFeatureDoesNotExtend
                } else {
                    BevelAdvisoryCode::SurfaceStripRunsOffBothSides
                };
                return Err(BevelAdvisory::new(code, vec![end.corner.into()]));
            }

            if crossed_here {
                if let Some(crossed) = crossed.as_ref() {
                    crossed.closed_the_end.set(true);
                }
            }

            if notch.is_some() && crossed_here {
                // The run-out's crossing cuts the rails back from one side and the feature's trace
                // from the other; named rather than built on whichever was asked first.
                return Err(BevelAdvisory::new(
                    BevelAdvisoryCode::SurfaceStandingFeatureDoesNotExtend,
                    vec![end.corner.into()],
                ));
            }

            let crossed_at_the_other_end = SurfaceStripTrimmedAtTheCrossing::standing_at(
                trimmed,
                edge_index,
                !end.at_the_start,
            );
            let notched_at_the_other_end =
                SurfaceStripNotchedByAFeature::standing_at(notched, edge_index, !end.at_the_start);

            let station = if let Some(stretch) = on_the_hull.as_ref() {
                SurfaceStripEndsOnTheHull::try_close(
                    pool,
                    faces,
                    faces_before_the_ends,
                    strip,
                    edge,
                    seams,
                    edge_index,
                    end.corner,
                    end.at_the_start,
                    stretch,
                    hull,
                )?
            } else if let Some(off_the_face) = off_the_face.as_ref() {
                surface_strip_ends_over_the_face_beyond::try_close_over_the_face_beyond(
                    pool,
                    faces,
                    strip,
                    edge,
                    seams,
                    edge_index,
                    end.corner,
                    end.at_the_start,
                    runs_out_facing,
                    off_the_face,
                    notch.as_deref(),
                    crossed_at_the_other_end.as_deref(),
                    notched_at_the_other_end.as_deref(),
                )?
            } else if let Some(notch) = notch.as_ref() {
                let the_other_end = if end.at_the_start {
                    end_rings[end.edge].as_ref()
                } else {
                    start_rings[end.edge].as_ref()
                };
                surface_strip_ends_at_a_landing_notch::try_close_the_landing_end(
                    pool,
                    faces,
                    strip,
                    seams,
                    edge_index,
                    end.corner,
                    end.at_the_start,
                    runs_out_facing,
                    notch,
                    the_other_end,
                    crossed_at_the_other_end.as_deref(),
                    notched_at_the_other_end.as_deref(),
                )?
            } else if let (Some(crossed), true) = (crossed.as_ref(), crossed_here) {
                surface_end_cut_above_the_crossing::try_close(
                    pool,
                    faces,
                    strip,
                    edge,
                    seams,
                    edge_index,
                    end.corner,
                    end.at_the_start,
                    runs_out_facing,
                    crossed,
                )?
            } else {
                try_open_end(
                    pool,
                    faces,
                    strip,
                    edge,
                    seams,
                    edge_index,
                    end.corner,
                    end.at_the_start,
                    runs_out_facing,
                )?
            };

            if end.at_the_start {
                start_rings[end.edge] = Some(station);
            } else {
                end_rings[end.edge] = Some(station);
            }

            continue;
        }

        if together.len() != 3 {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceCornerNotSupported,
                vec![ends[at].corner.into()],
            ));
        }

        // Three ends with a CREASE among them are not the ball's octant: what closes them is the
        // one torus between the two cuts (`SurfaceMixedJunction`), read at the top of the build.
        if let Some(junction) = the_junction_at(junctions, ends[at].vertex) {
            try_close_the_junction(
                pool,
                faces,
                &junction,
                first_of_plan,
                strips,
                seams,
                start_rings,
                end_rings,
            )?;

            continue;
        }

        let mut arrivals = Vec::with_capacity(3);
        for &one in &together {
            let end = ends[one];
            let (Some(strip), Some(on_the_left), Some(on_the_right)) = (
                strips[end.edge].as_ref(),
                seam_end(end.edge, 0, end.at_the_start),
                seam_end(end.edge, 1, end.at_the_start),
            ) else {
                return Err(BevelAdvisory::new(
                    BevelAdvisoryCode::SurfaceCornerNotSupported,
                    vec![end.corner.into()],
                ));
            };

            arrivals.push(SurfaceCornerPatchArrival::new(
                strip.clone(),
                edges[end.edge].clone(),
                on_the_left,
                on_the_right,
            ));
        }

        // One size and one profile for the whole operation, so the first plan's are every plan's.
        let plan = &plans[ends[at].plan];
        let stations = surface_corner_patch::try_close(
            pool,
            faces,
            &arrivals,
            ends[at].corner,
            plan.size,
            plan.round_profile,
        )?;

        for (&one, station) in together.iter().zip(stations) {
            let end = ends[one];
            if end.at_the_start {
                start_rings[end.edge] = Some(station);
            } else {
                end_rings[end.edge] = Some(station);
            }
        }
    }

    // And every crossing has to have CLOSED the end it stands at. One that did not is standing at a
    // mitre or a junction, where there is no end cut to put the curve in - an open shell nobody
    // named. Named here instead.
    if let Some(trimmed) = trimmed {
        for crossing in trimmed.iter().flatten().flatten() {
            if !crossing.closed_the_end.get() {
                return Err(BevelAdvisory::new(
                    BevelAdvisoryCode::SurfaceCrossingMeetsAMitredBand,
                    vec![pool.position_of(crossing.foot).into()],
                ));
            }
        }
    }

    Ok(())
}

/// The one run-off stretch at edge `edge_index`'s start (or end) - from either side's entry of
/// `over_a_stretch` (`None`, C#'s null, is none) - or `None` where neither side runs off there.
/// Refused (at `corner`) where both sides run off at that end.
pub fn try_the_stretch_at(
    over_a_stretch: Option<&[[Option<super::SurfaceBandRunsOffOverAStretch>; 2]]>,
    edge_index: usize,
    at_the_start: bool,
    corner: vector_math::Vector3,
) -> Result<Option<super::SurfaceBandRunsOffOverAStretch>, crate::bevel::BevelAdvisory> {
    let Some(over_a_stretch) = over_a_stretch else {
        return Ok(None);
    };

    let mut stretch = None;

    for side in &over_a_stretch[edge_index] {
        let Some(here) = side
            .as_ref()
            .filter(|here| here.at_the_start == at_the_start)
        else {
            continue;
        };

        if stretch.is_some() {
            return Err(crate::bevel::BevelAdvisory::new(
                crate::bevel::BevelAdvisoryCode::SurfaceStripRunsOffBothSides,
                vec![corner.into()],
            ));
        }

        stretch = Some(here.clone());
    }

    Ok(stretch)
}

/// The junction standing at `vertex`, where one of `junctions` does.
pub fn the_junction_at(
    junctions: Option<&[std::rc::Rc<super::SurfaceMixedJunction>]>,
    vertex: i32,
) -> Option<std::rc::Rc<super::SurfaceMixedJunction>> {
    junctions?
        .iter()
        .find(|junction| junction.vertex == vertex)
        .cloned()
}

/// C# `TryCloseTheJunction`: closes a junction's three ends with the one torus patch between its
/// two cuts ([`surface_mixed_junction_patch::try_close`]), and hands each strip back its own end
/// ring. WHICH three ends they are is the junction's own answer
/// ([`SurfaceMixedJunction::the_arriving_ends`]), because it differs by kind.
#[allow(clippy::too_many_arguments)]
pub fn try_close_the_junction(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    junction: &SurfaceMixedJunction,
    first_of_plan: &[i32],
    strips: &[Option<Rc<SurfaceStrip>>],
    seams: &[SideSeams],
    start_rings: &mut [Option<SurfaceStripStation>],
    end_rings: &mut [Option<SurfaceStripStation>],
) -> Result<(), BevelAdvisory> {
    let ends = junction.the_arriving_ends();
    let mut at_the_start = [false; 3];
    let mut edge_index = [0usize; 3];
    let mut arriving = Vec::with_capacity(3);

    for (one, end) in ends.iter().enumerate() {
        at_the_start[one] = end.at_the_start;
        let index = first_of_plan[end.plan as usize] + end.edge;
        edge_index[one] = index as usize;

        // A strip a blend swallowed, standing where the corner's own cut should be. The reading
        // refuses such a corner, so this says the two have not drifted apart.
        let Some(strip) = strips.get(index as usize).and_then(Option::as_ref) else {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceCornerNotSupported,
                vec![junction.corner.into()],
            ));
        };

        arriving.push(SurfaceMixedJunctionPatchArrival::new(
            strip.clone(),
            index,
            !end.at_the_start,
        ));
    }

    let stations =
        surface_mixed_junction_patch::try_close(pool, faces, junction, &arriving, seams)?;

    for (one, station) in stations.into_iter().enumerate().take(3) {
        if at_the_start[one] {
            start_rings[edge_index[one]] = Some(station);
        } else {
            end_rings[edge_index[one]] = Some(station);
        }
    }

    Ok(())
}

/// C# `TryOpenEnd`: a strip that stops at a chain's free end - the ring where every rail crosses
/// the end plane through `corner` and the two seams' end vertices (rail 0 and the last standing on
/// those vertices), and the notch that ring cuts out of the face beyond
/// ([`surface_end_cut::try_cut`], with no die point). Returns the station the stitch sews to.
/// `seams` is C#'s `List<(int, double)>[,]` indexed by `edge_index`; a missing or empty run there,
/// where C# would throw, is refused as an end not supported.
#[allow(clippy::too_many_arguments)]
pub fn try_open_end(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    strip: &SurfaceStrip,
    edge: &SurfaceEdgePlan,
    seams: &[SideSeams],
    edge_index: i32,
    corner: Vector3,
    at_the_start: bool,
    runs_out_facing: Vector3,
) -> Result<SurfaceStripStation, BevelAdvisory> {
    let not_supported = || {
        BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceStripEndNotSupported,
            vec![corner.into()],
        )
    };
    let seam_end = |side: usize| -> Option<i32> {
        let run = seams.get(edge_index as usize)?[side].as_ref()?;
        let end = if at_the_start {
            run.first()
        } else {
            run.last()
        };
        end.map(|&(vertex, _)| vertex)
    };
    let (Some(on_the_left), Some(on_the_right)) = (seam_end(0), seam_end(1)) else {
        return Err(not_supported());
    };

    let normal =
        (pool.position_of(on_the_left) - corner).cross(pool.position_of(on_the_right) - corner);
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // NaN refuses, as the C#'s `!(x > 0)` does
    let no_plane = !(normal.length_squared() > 0.0);
    if no_plane {
        return Err(not_supported());
    }

    let normal = normal.get_normal();
    let rails = strip.rail_count();
    let mut ring = vec![0; rails.max(0) as usize];
    let mut meets_at = vec![0.0; rails.max(0) as usize];

    for rail in 0..rails {
        let Some(param) = strip.try_cross(rail, corner, normal) else {
            return Err(not_supported());
        };

        meets_at[rail as usize] = param;
        let at = strip.point_on(rail, param);

        if rail == 0 || rail == rails - 1 {
            let vertex = if rail == 0 { on_the_left } else { on_the_right };
            if (pool.position_of(vertex) - at).length() > meets_the_seam(pool) {
                return Err(BevelAdvisory::new(
                    BevelAdvisoryCode::SurfaceStripSeamDidNotMeet,
                    vec![at.into()],
                ));
            }

            ring[rail as usize] = vertex;
            continue;
        }

        ring[rail as usize] = pool.add(at);
    }

    // Which way the chain runs back from this corner.
    let into_the_chain = if at_the_start {
        edge.edge.direction()
    } else {
        -edge.edge.direction()
    };

    // Which end of the ring stands on a wall the bevel rolled along, if either.
    let rolled_wall_end = if edge.left_band.is_some() {
        0
    } else if edge.right_band.is_some() {
        ring.len() as i32 - 1
    } else {
        -1
    };

    // No die point: an end nothing crosses has a profile that is all rim.
    let along_the_profile = surface_end_cut::try_cut(
        pool,
        faces,
        &ring,
        corner,
        edge.convexity() == EdgeConvexity::Concave,
        into_the_chain,
        runs_out_facing,
        rolled_wall_end,
        -1,
    )?;

    Ok(SurfaceStripStation::new(ring, meets_at, along_the_profile))
}
