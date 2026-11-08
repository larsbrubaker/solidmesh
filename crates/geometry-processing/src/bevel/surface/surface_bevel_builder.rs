//! Phase 3 of the surface bevel, and the whole of it seen from outside
//! (free functions): a
//! mesh, a chain and a size in, the bevelled mesh out, with no kernel call anywhere between. The
//! walkback opens the gap, the strip fills it, and the end faces give up the section the profile
//! cuts out of them. [`try_build`] is the entry point the part builder and the kernel tests call.
//!
//! **Sewn, never welded.** Every vertex along the seam is one the walkback already made and named
//! (`SurfaceWalkbackResult::tangent_runs`), and the strip uses those indices rather than points of
//! its own at the same place. The strip's cross sections therefore stand where the tangent lines
//! have vertices, on both sides: a station is put at every INTERIOR tangent vertex of either line,
//! and the line that has no vertex of its own there carries the one it is still on, so the quad
//! collapses to a triangle instead of a new point landing part way along a face's boundary edge.
//!
//! **Ends and mitres are the same cut.** Both are a plane the rails are cut at: at a corner between
//! two chain edges it is the mitre plane through the corner, and at the open end of a chain it is
//! the plane of the face beyond the corner. Either way each rail meets it at its own parameter,
//! which is why the strip is kept as rails (`SurfaceStrip`) - a mitred corner of a cube's top rim
//! meets the top face's tangent line 3 mm before it meets the side face's, and no single cross
//! section could be at both.
//!
//! **What it refuses.** A seam that does not meet: two strips whose rails cross the mitre plane at
//! different points (a rim whose two sides are not one face each - a tessellated cylinder, rung 5),
//! or an end ring that does not land on the tangent vertices the walkback reported. These are
//! refusals with a place in them rather than a mesh with a crack in it, which is the plan's
//! no-fallback rule.
//!
//! The pass is split by stage for the file-length limit: the strips and their seams are
//! `super::surface_bevel_builder_seams`, the corner loop `super::surface_bevel_builder_corners`
//! and the last pass, the stitches, `super::surface_builder_bitten_strips`. The build is one
//! function taking [`SurfaceBuildOptions`] for what a caller may leave out, answering
//! `Result<SurfaceBuilt, SurfaceBuildFailure>`.

use std::cell::OnceCell;
use std::rc::Rc;

use solidmesh_math::Vector3Float;
use solidmesh_mesh::{CancelToken, Mesh, MeshEdgeGraph};

use super::surface_bevel_builder_corners::{self as corners, CornerLoopInputs};
use super::surface_bevel_builder_seams as seams;
use super::surface_strip_trimmed_at_the_crossing::SurfaceStripTrims;
use super::{
    surface_bevel_crossings, surface_builder_bitten_strips, surface_builder_mixed_corners,
    surface_builder_standing_features, surface_face_walkback, surface_run_off_agreement,
    surface_seam_re_read, surface_strip_ends, surface_two_corners_at_one_vertex,
    SurfaceBandRunsOffOverAStretch, SurfaceBevelPlan, SurfaceBevelPlanlessLoop, SurfaceEdgePlan,
    SurfaceMitreConsumesTheRail, SurfaceMixedJunction, SurfaceRoomCrossing,
    SurfaceStripEndsOnTheHull, SurfaceStripStation, SurfaceVertexPool,
};
use crate::bevel::sections::section_builder_walked_side::SectionBuildRefusal;
use crate::bevel::sections::section_routing::{self, SectionRouteAnswer};
use crate::bevel::sections::{section_assembly, NanSectionPoint};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};
use crate::topology::{FaceRegionCache, FeatureEdgeSet};

/// What a pass that built hands back: the bevelled mesh and the faces of its blend.
#[derive(Debug, Clone)]
pub struct SurfaceBuilt {
    /// The bevelled mesh.
    pub mesh: Mesh,
    /// The indices, into [`SurfaceBuilt::mesh`]'s own face list, of the faces the fillet is made
    /// of. A caller fences a later flood with these, so that a region picked on this mesh does not
    /// run over the blend - a fillet's facets turn less than a pick's flat angle, so an unfenced
    /// flood crosses them and comes out the far side.
    ///
    /// **Owed.** Only the SECTIONS route answers this; the legacy construction hands back an empty
    /// list, which reads as "no blend here" and leaves a fence built from it doing nothing. Its own
    /// strips, mitres, ends and corner patches are laid into a face list the same way and could be
    /// marked the same way, and until they are a pass that fell through to the legacy route cannot
    /// be fenced.
    pub blend_faces: Vec<i32>,
}

/// Why a pass built nothing: a refusal (named or not), a cancellation, or a point that came out not
/// a number.
#[derive(Debug, Clone)]
pub enum SurfaceBuildFailure {
    /// Refused, with the advisory the user is shown.
    Refused(BevelAdvisory),
    /// Refused with nothing named, which only the sections
    /// route answers (its assembly declining, or its room with no advisory).
    Declined,
    /// The caller's cancellation token was cancelled between two stages.
    Cancelled,
    /// A section point that is not a number.
    NotANumber(NanSectionPoint),
}

impl SurfaceBuildFailure {
    /// The advisory the user is shown (`None` for a decline, a cancel or a NaN).
    pub fn advisory(&self) -> Option<&BevelAdvisory> {
        match self {
            SurfaceBuildFailure::Refused(advisory) => Some(advisory),
            _ => None,
        }
    }
}

impl From<BevelAdvisory> for SurfaceBuildFailure {
    fn from(advisory: BevelAdvisory) -> Self {
        SurfaceBuildFailure::Refused(advisory)
    }
}

impl From<NanSectionPoint> for SurfaceBuildFailure {
    fn from(nan: NanSectionPoint) -> Self {
        SurfaceBuildFailure::NotANumber(nan)
    }
}

impl From<SectionBuildRefusal> for SurfaceBuildFailure {
    fn from(refusal: SectionBuildRefusal) -> Self {
        match refusal {
            SectionBuildRefusal::Refused(advisory) => SurfaceBuildFailure::Refused(advisory),
            SectionBuildRefusal::NotANumber(nan) => SurfaceBuildFailure::NotANumber(nan),
            SectionBuildRefusal::Declined => SurfaceBuildFailure::Declined,
        }
    }
}

/// What a caller may hand the build beyond the mesh, the chain and the size; all optional.
#[derive(Clone, Copy, Default)]
pub struct SurfaceBuildOptions<'a> {
    /// Where another room's band crosses one of these, when any does.
    pub crossings: Option<&'a [SurfaceRoomCrossing]>,
    /// The partial run-offs the ROOM read on these same plans (`SurfaceRoom::try_fit`), for this
    /// build to check its own reading against: the two are the same call and must answer the
    /// same, and where they do not the pass is refused by name rather than built with an end
    /// nobody closes (`surface_run_off_agreement`). `None` where the caller has no reading to
    /// offer, which leaves the build on its own.
    pub ran_off_over_a_stretch: Option<&'a [SurfaceBandRunsOffOverAStretch]>,
    /// Lifts THE LEAN GATE, for the kernel test that measures a leaning corner's own rows while
    /// the bites, the wall and the run-out laid on them are still the torus's. A seam of the ladder
    /// and not of the product: no caller outside the tests passes it.
    pub lay_the_oblique_rows_anyway: bool,
    /// Closed loops the planner would not plan, which only the sections route can build.
    pub planless_loops: &'a [SurfaceBevelPlanlessLoop],
    /// The feature set the pick was resolved in, carried down to the routed pass so a PLANNED run
    /// reads the part's own creases - see `section_routing::is_simple`. `None` leaves the routed
    /// pass with no reading.
    pub features: Option<&'a FeatureEdgeSet>,
    /// Whether a pass the sections can build is built by them (the caller's
    /// `BevelRouting::sections`); off, every pass is built by the construction below, as it is by
    /// default.
    pub use_sections: bool,
}

/// Whether every plan of the pass was made at the same size and the same profile, which is what the
/// rest of the pass assumes: a corner reads the size and the profile off whichever of the three
/// chains happened to seed it (`surface_strip_ends::try_close_the_ends`), so a pass carrying a 3 mm
/// fillet and a 1 mm chamfer would build one of them at the other's corner without ever saying so.
/// One operation is one size and one profile; a set that is not is refused here, at the top, rather
/// than found out at a vertex.
fn is_one_operation(plans: &[Rc<SurfaceBevelPlan>]) -> bool {
    plans.iter().all(|plan| {
        !(plan.round_profile != plans[0].round_profile
            || (plan.size - plans[0].size).abs() > plans[0].size.abs() * 1e-9)
    })
}

fn cancelled(cancellation_token: &CancelToken) -> Result<(), SurfaceBuildFailure> {
    if cancellation_token.is_cancelled() {
        Err(SurfaceBuildFailure::Cancelled)
    } else {
        Ok(())
    }
}

/// Bevels one planned chain.
pub fn try_build_one(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    regions: &FaceRegionCache,
    plan: Rc<SurfaceBevelPlan>,
    segments_per_quarter: i32,
    cancellation_token: &CancelToken,
) -> Result<Mesh, SurfaceBuildFailure> {
    try_build(
        mesh,
        graph,
        regions,
        &[plan],
        segments_per_quarter,
        cancellation_token,
        SurfaceBuildOptions::default(),
    )
    .map(|built| built.mesh)
}

/// Bevels every planned chain of one part in ONE
/// pass - every chain is planned on the original mesh, every cut of every chain is walked back
/// together, and then the strips, the mitres, the ends and the corner patches are stitched into the
/// one gap that leaves.
///
/// One pass is what makes more than one chain possible at all. A chain's tangent lines depend only
/// on the faces its own edges lie between, so the plans do not depend on each other; but the
/// WALKBACK does - two chains reaching one face each take a band off it, and a second walkback run
/// on the rewritten mesh would be planning against a chain that mesh no longer has. So the cuts are
/// gathered and applied together, and every strip is sewn to the runs that leaves.
///
/// Where chains MEET is the corner: every free end of every chain is gathered by the vertex it
/// stands on, and an end standing alone is notched into the face beyond it, while three ends at one
/// vertex hand over to the corner patch. TWO ends at one vertex are refused, because two runs
/// meeting end to end are one run and mitre inside a single chain - joining them is
/// `surface_chain_join`'s, done before anything is planned.
///
/// `mesh` is the welded mesh the plans were made on (it is not modified), `graph` its own edge
/// graph, `regions` the region cache the plans were made with. The cancellation token is checked
/// between the stages of the build - the walkback, the strips, the ends and the stitch - each of
/// which is a whole pass over the mesh.
pub fn try_build(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    regions: &FaceRegionCache,
    plans: &[Rc<SurfaceBevelPlan>],
    segments_per_quarter: i32,
    cancellation_token: &CancelToken,
    options: SurfaceBuildOptions<'_>,
) -> Result<SurfaceBuilt, SurfaceBuildFailure> {
    cancelled(cancellation_token)?;

    let planless_loops = options.planless_loops;
    let has_planless_loops = !planless_loops.is_empty();

    if plans.is_empty() && !has_planless_loops {
        return Err(BevelAdvisory::new(BevelAdvisoryCode::NoEdgesResolved, vec![]).into());
    }

    if !plans.is_empty() && !is_one_operation(plans) {
        return Err(
            BevelAdvisory::new(BevelAdvisoryCode::SurfaceMixedSizesInOnePass, vec![]).into(),
        );
    }

    // The sections construction takes WHOLE passes it can build - all the plans or none of them,
    // see SectionRouting's remarks - and everything else falls through to the construction below
    // untouched. Off unless the caller routes the sections (`options.use_sections`).
    section_routing::set_last_route(Some("legacy: the flag is off".to_string()));

    let mut routed_refusal = None;

    if options.use_sections {
        match section_routing::is_simple_with_planless(
            plans,
            planless_loops,
            Some(mesh),
            Some(graph),
            segments_per_quarter,
            options.features,
        )? {
            SectionRouteAnswer::Sections(mut route) => {
                // The room is the routed path's own reading and the two paths run out at different
                // numbers, so a chain with no room refuses the pass here rather than falling back
                // to a construction that would build what the panel has just said does not fit.
                let size = if !plans.is_empty() {
                    plans[0].size
                } else {
                    planless_loops[0].size
                };

                section_routing::has_room(mesh, graph, &route.runs, size).map_err(|failure| {
                    failure.map_or(SurfaceBuildFailure::Declined, SurfaceBuildFailure::Refused)
                })?;

                let (built, blend_faces) = section_assembly::try_assemble_with_blend(
                    mesh,
                    graph,
                    &route.runs,
                    &mut route.pool,
                )?;

                return Ok(SurfaceBuilt {
                    mesh: built,
                    blend_faces,
                });
            }
            SectionRouteAnswer::Legacy { refusal, .. } => routed_refusal = refusal,
        }
    }

    if has_planless_loops {
        // A loop with no plan has no legacy construction to fall through to - the planner refused
        // to make one. So the row says again what the planner said.
        return Err(routed_refusal
            .unwrap_or_else(|| planless_loops[0].planner_refusal.clone())
            .into());
    }

    let built = try_build_the_legacy_pass(
        mesh,
        graph,
        regions,
        plans,
        segments_per_quarter,
        cancellation_token,
        &options,
    )?;

    Ok(SurfaceBuilt {
        mesh: built,
        blend_faces: Vec::new(),
    })
}

/// The legacy construction: everything the build does after the routing.
fn try_build_the_legacy_pass(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    regions: &FaceRegionCache,
    plans: &[Rc<SurfaceBevelPlan>],
    segments_per_quarter: i32,
    cancellation_token: &CancelToken,
    options: &SurfaceBuildOptions<'_>,
) -> Result<Mesh, SurfaceBuildFailure> {
    // Every vertex where three or more picked edges meet with a crease among them, read as the
    // junction it is and handed to the plans that meet at it - or refused there by name
    // (SurfaceMixedJunction). It has to come before the walkback, because what it records is what
    // bounds the bands the walkback takes off.
    let junctions: Vec<Rc<SurfaceMixedJunction>> = SurfaceMixedJunction::try_read_all(
        plans,
        SurfaceVertexPool::near_enough_for(&mesh.vertices),
    )?
    .into_iter()
    .map(Rc::new)
    .collect();

    // And a vertex TWO plans both turn a corner at is no corner anything here builds, so it is
    // named at the vertex now rather than found out as a seam that would not meet.
    surface_two_corners_at_one_vertex::try_survey(plans)?;

    let walked = surface_face_walkback::try_apply(mesh, graph, regions, plans)?;

    cancelled(cancellation_token)?;

    let mut pool = SurfaceVertexPool::new(&walked.mesh.vertices);
    let mut faces: Vec<(i32, i32, i32)> = Vec::with_capacity(walked.mesh.faces.len());

    for face in walked.mesh.faces.iter() {
        faces.push((face.v0, face.v1, face.v2));
    }

    // Every chain's edges, end to end, under the one numbering the walkback keyed its tangent runs
    // by: chain p's edge i is edge first_of_plan[p] + i here and everywhere downstream.
    let mut edges: Vec<SurfaceEdgePlan> = Vec::new();
    let mut first_of_plan = vec![0; plans.len()];

    for (p, plan) in plans.iter().enumerate() {
        first_of_plan[p] = edges.len() as i32;
        edges.extend(plan.edges.iter().cloned());
    }

    let count = edges.len() as i32;

    let band_corners = seams::read_the_band_corners(&walked, plans, &first_of_plan, &edges);

    let strips = seams::try_create_the_strips(plans, &first_of_plan, &edges, segments_per_quarter)?;

    // Which chords are so short that their own two mitres have crossed over on some rails, read
    // before any seam is picked because a seam on a consumed side is not a run of the tangent line
    // at all but the single corner the two NEIGHBOURS' lines already meet at.
    let consumed: Vec<Option<SurfaceMitreConsumesTheRail>> =
        SurfaceMitreConsumesTheRail::try_survey(&pool, plans, &first_of_plan, &edges, &strips)?;

    // And which leaning mixed corners' run-outs would die PAST the end of the crease they die on,
    // read HERE rather than in the corner loop: the same overshoot that refuses the corner also
    // takes the tangent run a neighbouring seam would have been sewn to, so a seam picked first
    // answers first and names the boolean's triangulation instead of the size
    // (surface_builder_mixed_corners::try_survey_the_run_outs).
    surface_builder_mixed_corners::try_survey_the_run_outs(
        Some(regions),
        plans,
        &first_of_plan,
        count,
        options.crossings,
        &consumed,
        options.lay_the_oblique_rows_anyway,
    )?;

    let seams::PickedSeams {
        seams: mut the_seams,
        ran_off,
        over_a_stretch,
    } = seams::try_pick_the_seams(
        mesh,
        &mut pool,
        &walked,
        plans,
        &first_of_plan,
        &edges,
        &strips,
        &consumed,
    )?;

    surface_run_off_agreement::the_same_on_both_readings(
        &first_of_plan,
        &over_a_stretch,
        options.ran_off_over_a_stretch,
        &edges,
    )?;

    SurfaceMitreConsumesTheRail::try_fill_the_seams(&pool, &strips, &mut the_seams, &consumed)?;

    // And what a feature STANDING inside a band's footprint takes out of the strip - a closed hole
    // where it stands part way along, an open notch where it stands at an end, and its own faces
    // carried out over the corner the band took (SurfaceBuilderStandingFeatures). Read here, with
    // the seams picked and the run-off trims standing, because the notch has to know which rails
    // the run-off already collapsed - and before the ends are closed, which is what the notch
    // changes.
    let (notched, bitten) = surface_builder_standing_features::try_read(
        &mut pool,
        &mut faces,
        &walked,
        plans,
        Some(&junctions),
        &edges,
        &first_of_plan,
        &strips,
        &mut the_seams,
        &ran_off,
        count,
    )?;

    // Where the seams were read: everything numbered from here on is a vertex the corners and the
    // ends are about to make, and any of them landing on a seam has to be taken into it before the
    // stitch (SurfaceSeamReRead).
    let before_the_corners = pool.count();

    cancelled(cancellation_token)?;

    let mut start_rings: Vec<Option<SurfaceStripStation>> = vec![None; edges.len()];
    let mut end_rings: Vec<Option<SurfaceStripStation>> = vec![None; edges.len()];

    let hull_planes = OnceCell::new();
    let near_enough = pool.near_enough();
    let the_hull = corners::the_hull_once(&hull_planes, || {
        SurfaceStripEndsOnTheHull::supporting_planes_of(&walked.mesh, near_enough)
    });

    // Where the room reading found a mixed corner's RUN-OUT crossing another chain's BAND, the
    // band's strip is cut short at the curve the two meet in rather than either of them being
    // refused (SurfaceRoomCrossing). The two are in one scope only here, which is why the crossing
    // is built here: the corner reads the curve off the strip, and the strip closes its end on the
    // curve.
    let mut trimmed: SurfaceStripTrims = vec![[None, None]; edges.len()];

    corners::try_close_the_corners(
        &mut pool,
        &mut faces,
        &CornerLoopInputs {
            regions,
            plans,
            first_of_plan: &first_of_plan,
            edges: &edges,
            strips: &strips,
            seams: &the_seams,
            band_corners: &band_corners,
            consumed: &consumed,
            crossings: options.crossings,
            lay_the_oblique_rows_anyway: options.lay_the_oblique_rows_anyway,
            hull: &the_hull,
        },
        &mut trimmed,
        &mut start_rings,
        &mut end_rings,
    )?;

    // The planes the part is bounded by, read ONCE for the pass and only where an end really needs
    // them: a strip that ran off part of its landing is ended on them (SurfaceStripEndsOnTheHull),
    // and every other end is closed on a face as it always was.
    let hull = if the_hull_is_needed(&over_a_stretch) {
        the_hull()
    } else {
        Vec::new()
    };

    surface_strip_ends::try_close_the_ends(
        &mut pool,
        &mut faces,
        plans,
        Some(&junctions),
        &edges,
        &first_of_plan,
        &strips,
        &the_seams,
        &mut start_rings,
        &mut end_rings,
        Some(&trimmed),
        Some(&ran_off),
        Some(&notched),
        Some(&over_a_stretch),
        &hull,
    )?;

    cancelled(cancellation_token)?;

    // The seams as the mesh now stands rather than as the walkback left them: a corner bite of one
    // chain can cut a face out to ANOTHER chain's tangent line, and the vertices it lands there are
    // sewn into the face but were not in the run the strip was going to sew to.
    let later_splits =
        surface_seam_re_read::later_splits_carried_by_a_face(&faces, before_the_corners);

    surface_seam_re_read::take_in_the_later_splits(&pool, &later_splits, &strips, &mut the_seams);

    // And the seams themselves cut back where a crossing took the tangent line: past the crossing
    // that line has the cap gone on BOTH sides of it - the walkback took one and the run-out's own
    // sector the other - so the vertices the walkback left there bound nothing, and a stitch told to
    // reach them would draw the strip back over the surface that replaced it.
    surface_bevel_crossings::trim_the_seams_at_the_crossings(&strips, &trimmed, &mut the_seams);

    // THE ROWS FIRST, THEN THE FEATURES, THEN THE STITCHES - three passes where there used to be one
    // (SurfaceBuilderBittenStrips).
    surface_builder_bitten_strips::try_lay_them_and_stitch(
        &mut pool,
        &mut faces,
        plans,
        &first_of_plan,
        &strips,
        &bitten,
        &the_seams,
        &start_rings,
        &end_rings,
        &later_splits,
        count,
    )?;

    Ok(assemble(pool.vertices(), &faces))
}

/// Whether the blend swallows edge `edge_index` of the one numbering.
pub fn is_swallowed(
    plans: &[Rc<SurfaceBevelPlan>],
    first_of_plan: &[i32],
    edge_index: i32,
) -> bool {
    let plan = plan_of(first_of_plan, edge_index);

    plan >= 0
        && plans[plan as usize]
            .roles()
            .is_swallowed(edge_index - first_of_plan[plan as usize])
}

/// Whether any side of any strip ran off part of its landing, which is the
/// only reading that asks what the part's hull planes are - and they cost a pass of the mesh per
/// distinct face plane, so a pass with no such end does not pay for them.
fn the_hull_is_needed(over_a_stretch: &[seams::SideStretches]) -> bool {
    over_a_stretch
        .iter()
        .flatten()
        .any(|stretch| stretch.is_some())
}

/// A refusal raised while one strip was being read, told which
/// picked edge that strip is. A refusal that already named an edge keeps the one it named.
///
/// Reporting only: nothing about what is built or refused changes here, only WHOSE the answer is.
/// A pass builds every group's chains out of one mesh together and so refuses once, and without a
/// name on that refusal the only honest thing the caller could do with it was write it against
/// every group in the pass - which is how a chain round one end of a part came to carry a sentence
/// about a strip at the other end that it never touched ("Chain and Face Should Both work.mcx").
/// The strip loop is per picked edge, so the edge is simply to hand; the caller maps it back to the
/// group that picked it and drops that group alone, so the groups it did not touch still build.
pub fn the_strips_own_edge_is_to_blame(
    edges: &[SurfaceEdgePlan],
    edge_index: usize,
    failure: BevelAdvisory,
) -> BevelAdvisory {
    if failure.mesh_edge_index() == BevelAdvisory::NO_MESH_EDGE {
        failure.on_mesh_edge(edges[edge_index].edge.edge.mesh_edge_index)
    } else {
        failure
    }
}

/// Which plan of the pass an edge of the one numbering belongs to, or -1 where none
/// does.
pub fn plan_of(first_of_plan: &[i32], edge_index: i32) -> i32 {
    for p in (0..first_of_plan.len()).rev() {
        if edge_index >= first_of_plan[p] {
            return p as i32;
        }
    }

    -1
}

/// The new mesh, with the vertices no face uses any more dropped and everything
/// renumbered - the corner the end cut took off is nobody's now. No `CleanAndMerge`: every vertex
/// shared was shared by index as it was made.
fn assemble(pool_vertices: &[Vector3Float], faces: &[(i32, i32, i32)]) -> Mesh {
    let mut renumbered = vec![-1; pool_vertices.len()];
    let mut vertices: Vec<Vector3Float> = Vec::with_capacity(pool_vertices.len());

    for &(v0, v1, v2) in faces {
        for vertex in [v0, v1, v2] {
            if renumbered[vertex as usize] < 0 {
                renumbered[vertex as usize] = vertices.len() as i32;
                vertices.push(pool_vertices[vertex as usize]);
            }
        }
    }

    let mut mesh = Mesh::new();

    for &(v0, v1, v2) in faces {
        mesh.faces.add(
            renumbered[v0 as usize],
            renumbered[v1 as usize],
            renumbered[v2 as usize],
            &vertices,
        );
    }

    mesh.vertices = vertices;
    mesh
}
