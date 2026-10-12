//! Everything one resolved group of selected edges turns into (`Bevel/BevelGroupTools.cs`, C#
//! `internal static class`, free functions here): the runs it links up to
//! ([`super::bevel_chain_linker`]), the tools swept along them ([`super::bevel_chain_sweep`],
//! [`super::bevel_tapered_sweep`], [`super::bevel_cap_clip`], [`super::bevel_partnered_bead`]), and
//! the trims those tools carry with them ([`BevelTool`]).
//!
//! Split out of `BevelFeatureMeshBuilder` because the two answer different questions. This module
//! answers "what solids does THIS group want", reading nothing but the group's own resolution and
//! the handovers the corner stages worked out beforehand; the builder answers "what does the whole
//! operation do with all of them" - which batch each tool joins, how many booleans that adds up to,
//! and how the progress bar is shared out over them. The seam between the two is [`try_build`]'s
//! return: a list of [`BevelTool`], each already carrying its wedge, its tube and its cap clips, and
//! no knowledge at all of the order the batches run in.
//!
//! It takes the profile as a parameter rather than reading the operation, which is what lets a test
//! build one group's tools from a bare mesh and assert on the solid - most of the bevel suite enters
//! here rather than through a rebuild.

use std::cell::OnceCell;
use std::collections::HashSet;
use std::rc::Rc;

use indexmap::IndexMap;
use polygon_mesh::Mesh;
use vector_math::Vector3;

use super::bevel_chain_end_cap::ResolveOptions;
use super::{
    bevel_bead_exit, bevel_cap_clip, bevel_chain_linker, bevel_chain_sweep,
    bevel_edge_tool_builder, bevel_pair_corner_reach, bevel_pair_corner_trim, bevel_partnered_bead,
    bevel_tapered_sweep,
};
use super::{
    BevelAdvisory, BevelAdvisoryCode, BevelChainEnds, BevelCornerVoids, BevelEndCap,
    BevelFaceRunOut, BevelRoundFaces, BevelRunTaper, BevelSurfaceLabels, BevelSweptChain,
    BevelSweptSection, BevelTool,
};
use crate::topology::{
    BevelEdgeGroupResolution, BevelSelectionKind, ChainEdge, EdgeConvexity, FeatureEdge,
};

/// C# `sweepTaper`: how to sweep a run that has grown a finite run-out, told the taper, the ends
/// and which section to draw; `None` from it is a taper the kernel refused.
pub type BevelSweepTaper<'f> =
    &'f mut dyn FnMut(&BevelRunTaper, &BevelChainEnds, BevelSweptSection) -> Option<Mesh>;

/// [`try_build`]'s optional arguments, at C#'s defaults by `Default` (every one `None`, `false`,
/// except `allow_taper`, which is true). Each field's meaning is C#'s parameter of the same name:
///
/// - `selected_across_groups`: every edge every group selected, which is what an end may overlap
///   into rather than cap against (plan 6's cross-group corner); `None` reads this group's own.
/// - `patched_corners`: where `BevelCornerPatches` has put a ball, by vertex.
/// - `blend_handovers` / `blend_capped_corners`: the plane a convex chain ending at a blended corner
///   stops on, and which face a concave one stops on there.
/// - `filled_corners`: where `BevelCornerPatches` has put a ball out in the VOID.
/// - `corner_voids`: the rebuild's reading of the void each 5.3 corner opens.
/// - `surface_labels`: the rebuild's reading of which faces are one surface, asked at THIS group's
///   own curve tolerance - and only by the corner that needs it.
/// - `patch_seam_rings`: the points the corner patches have already put on each run's ball-centre
///   plane.
/// - `concave_runs_across_groups`: every concave run of the whole rebuild.
/// - `run_out`: how far past its crease each run may ride.
/// - `linked_runs`: a shared sweep's prelinked runs, which own every frame and endpoint.
/// - `sweep_taper`: injectable for the branch no fixture reaches - a taper the kernel refuses;
///   `None` is `BevelTaperedSweep.TryCreate` itself.
/// - `rounds`: which faces of the part are facets of a ROUND.
pub struct BevelGroupToolsOptions<'a, 'm> {
    pub selected_across_groups: Option<&'a [FeatureEdge]>,
    pub patched_corners: Option<&'a IndexMap<i32, Vector3>>,
    pub blend_handovers: Option<&'a IndexMap<(i32, i32), BevelEndCap>>,
    pub blend_capped_corners: Option<&'a IndexMap<(i32, i32), Vector3>>,
    pub filled_corners: Option<&'a IndexMap<i32, Vector3>>,
    pub may_clip_caps: bool,
    pub corner_voids: Option<&'a BevelCornerVoids>,
    pub surface_labels: Option<&'a mut BevelSurfaceLabels<'m>>,
    pub runs_may_drive_through: Option<&'a HashSet<i32>>,
    pub patch_seam_rings: Option<&'a IndexMap<(i32, i32), Vec<Vector3>>>,
    pub concave_runs_across_groups: Option<&'a [BevelSweptChain]>,
    pub run_out: Option<Rc<BevelFaceRunOut>>,
    pub linked_runs: Option<&'a [BevelSweptChain]>,
    pub allow_taper: bool,
    pub sweep_taper: Option<BevelSweepTaper<'a>>,
    pub rounds: Option<&'a BevelRoundFaces>,
}

impl Default for BevelGroupToolsOptions<'_, '_> {
    fn default() -> Self {
        BevelGroupToolsOptions {
            selected_across_groups: None,
            patched_corners: None,
            blend_handovers: None,
            blend_capped_corners: None,
            filled_corners: None,
            may_clip_caps: false,
            corner_voids: None,
            surface_labels: None,
            runs_may_drive_through: None,
            patch_seam_rings: None,
            concave_runs_across_groups: None,
            run_out: None,
            linked_runs: None,
            allow_taper: true,
            sweep_taper: None,
            rounds: None,
        }
    }
}

/// What a group that could be applied turns into: its tools, and what it did that the user should
/// know about but which did not stop it (`None` when there was nothing to say - never empty).
#[derive(Debug, Clone)]
pub struct BevelGroupToolsBuilt {
    pub tools: Vec<BevelTool>,
    pub warnings: Option<Vec<BevelAdvisory>>,
}

/// Why a group could not be applied, with whatever it had already said on the way (`None` when
/// nothing).
#[derive(Debug, Clone)]
pub struct BevelGroupToolsRefusal {
    pub reason: BevelAdvisory,
    pub warnings: Option<Vec<BevelAdvisory>>,
}

/// C# `TryBuild`: the tools for one resolved group, or the reason the group cannot be applied. All
/// or nothing: a group that could only be half applied would leave a part that looks beveled and is
/// not.
///
/// One tool per RUN of connected edges, not one per edge: a run of two or more is swept as a single
/// mitred solid (`BevelChainSweep`), which is what makes the joints between them meet instead of
/// overlapping. Every run's free ends - a one-edge run's as much as a long chain's - are capped or
/// overshot as `BevelChainEnds` reads the topology at them. A run is cut into same-convexity pieces
/// first (`BevelChainLinker.SplitByConvexity`), because one sweep cannot carry both.
///
/// A one-edge run is swept like any other: the extruded prism it used to keep landed its cut plane
/// on Clipper's 0.001 mm grid, up to a micron off the sweep's exact one
/// (`BevelToolPlaneAccuracyTests`).
///
/// Capping a single edge does NOT get in the way of plan 5.3's corner: `BevelChainEnds` only ever
/// caps against faces of edges the SELECTION did not take at this convexity, so two chamfers
/// meeting at a corner still overlap inside it - and the selection it reads is every group's
/// (`selected_across_groups`).
pub fn try_build(
    resolution: &BevelEdgeGroupResolution,
    round_profile: bool,
    size: f64,
    segments: i32,
    options: BevelGroupToolsOptions<'_, '_>,
) -> Result<BevelGroupToolsBuilt, BevelGroupToolsRefusal> {
    let BevelGroupToolsOptions {
        selected_across_groups,
        patched_corners,
        blend_handovers,
        blend_capped_corners,
        filled_corners,
        may_clip_caps,
        corner_voids,
        surface_labels,
        runs_may_drive_through,
        patch_seam_rings,
        concave_runs_across_groups,
        run_out,
        linked_runs,
        allow_taper,
        mut sweep_taper,
        rounds,
    } = options;

    let mut group_tools: Vec<BevelTool> = Vec::new();

    if resolution.edges.is_empty() {
        return Err(BevelGroupToolsRefusal {
            reason: BevelAdvisory::new(BevelAdvisoryCode::NoEdgesResolved, vec![]),
            warnings: None,
        });
    }

    // Collected here and published only on the way out, so a caller reads either the whole list
    // or None - never an empty one, which is what "nothing to say" has always been.
    let mut said: Vec<BevelAdvisory> = Vec::new();
    let published = |said: Vec<BevelAdvisory>| (!said.is_empty()).then_some(said);

    // A face region whose boundary is not all bevelable. The rim really is the whole of what a
    // bevel can be built on here and the group is not refused for it, but the row has to say how
    // much was left out, because a face beveled on three sides of four looks like a bug from
    // across the room and is not one.
    if let Some(face_group) = &resolution.face_group {
        let open = face_group.open_boundary_mesh_edges().len();

        if open > 0 {
            said.push(BevelAdvisory::new(
                BevelAdvisoryCode::FaceRegionRimIsPartial,
                vec![(open as i32).into()],
            ));
        }
    }

    // Asked for by the corner that needs it rather than read here: labelling the part is a flood
    // over every face, and a rebuild whose corners are all settled by their normals - every flat
    // and every prismatic one - must not pay for one. See BevelPairCornerCrease. The labels are
    // read once and held for the whole group, so every caller is handed the same slice.
    let tolerance = resolution.group.curve_tolerance_degrees;
    let has_labels = surface_labels.is_some();
    let mut labels = surface_labels;
    let surfaces: OnceCell<Vec<i32>> = OnceCell::new();
    let mut surface_of_face = || -> Option<&[i32]> {
        let labels = labels.as_mut()?;
        Some(
            surfaces
                .get_or_init(|| labels.at(tolerance).to_vec())
                .as_slice(),
        )
    };

    let selected = selected_across_groups.unwrap_or(&resolution.edges);

    // The selection as a set, for the same "is this edge one of ours" question BevelChainEnds asks
    // with one of its own. Built at the first chain that sweeps a tube and not before, because most
    // groups never sweep one at all.
    let mut selected_mesh_edges: Option<HashSet<i32>> = None;

    // A shared sweep uses one contributor for its common feature-set/curve context. Its prelinked
    // runs, rather than that contributor's subset, own every frame and endpoint.
    let linked_list: Vec<BevelSweptChain> = match linked_runs {
        Some(linked) => linked.to_vec(),
        None => bevel_chain_linker::link_resolution(resolution),
    };

    for linked in linked_list {
        // Two cuts, and they are not the same cut: SplitByConvexity where one profile cannot be
        // carried on, then SplitWhereTheMitreCannotFit where one MITRE cannot. What comes out of
        // the pair is swept whole, and every corner they cut at is a run end for the cap and blend
        // machinery below to answer.
        let mut pieces: Vec<(BevelSweptChain, f64)> = Vec::new();

        for uniform in bevel_chain_linker::split_by_convexity(linked) {
            let (split, depth) = bevel_chain_linker::split_where_the_mitre_cannot_fit_with_depth(
                uniform,
                size,
                segments,
                round_profile,
            );

            pieces.extend(split.into_iter().map(|piece| (piece, depth)));
        }

        for (chain, back_depth) in pieces {
            let is_bead = chain.edges[0].edge.convexity == EdgeConvexity::Concave;
            let mut cap_clips = None;

            // Every run gets its ends read, however short it is. A single edge dies at a corner
            // exactly as often as a chain does - more often, since it has no neighbours in the
            // selection to run into.
            let ends = BevelChainEnds::resolve(
                &chain.edges,
                chain.is_closed,
                size,
                Some(&resolution.features),
                Some(selected),
                Some(&mut said),
                ResolveOptions {
                    patched_corners,
                    blend_handovers,
                    blend_capped_corners,
                    filled_corners,
                    round_profile,
                    surface_of_face: if has_labels {
                        Some(&mut surface_of_face)
                    } else {
                        None
                    },
                    runs_may_drive_through,
                    patch_seam_rings,
                    concave_runs: if is_bead {
                        concave_runs_across_groups
                    } else {
                        None
                    },
                },
            );

            // A concave run that has a straight partner at BOTH ends, and a run-out that reaches
            // them, is not swept at all: it is built station by station as one solid, crease and
            // run-outs together (BevelPartneredBead). Such a bead needs no void wedge, no bead clip
            // and no end cap - it already ends where the finished part's material ends. The gate is
            // the builder's own refusal rather than a test here.
            let try_partnered_bead = || -> Option<Mesh> {
                match (
                    is_bead && round_profile,
                    concave_runs_across_groups,
                    &run_out,
                ) {
                    (true, Some(concave), Some(run_out)) => bevel_partnered_bead::try_build(
                        &chain,
                        concave,
                        size,
                        segments,
                        Some(Rc::clone(run_out)),
                        None,
                    )
                    .ok(),
                    _ => None,
                }
            };

            let lone_unlinked_edge = chain.edges.len() == 1 && linked_runs.is_none();
            let partnered = if lone_unlinked_edge {
                None
            } else {
                try_partnered_bead()
            };

            if let Some(partnered) = partnered {
                group_tools.push(partnered_tool(partnered));
                continue;
            }

            // Any OPEN run with no shared corner/clip ownership may grow a temporary runout at its
            // free ends - read PER END, by way of a reach of zero at an end something else stands
            // at - and only in a group that ASKED for edges (a tangent chain that came back as its
            // seed edge is diagnosed instead, BevelGroupFailures.Diagnose). A concave run only
            // where the PARTNERED builder would not have it; a LONE concave edge in a group that
            // was not prelinked keeps taper first, partnered only if the taper is refused
            // (BevelSingleEdgeTaperTests). See BevelRunTaperTests for the per-end reading.
            let mut taper: Option<BevelRunTaper> = None;

            if allow_taper
                && !may_clip_caps
                && has_labels
                && resolution.face_group.is_none()
                && resolution.group.selection_kind == BevelSelectionKind::Edge
                && !chain.is_closed
            {
                // The run's two ends, which for a lone edge are that edge's own two vertices and for
                // a merged rim are the outer vertices of its first and last facet.
                let first_edge = &chain.edges[0];
                let last_edge = &chain.edges[chain.edges.len() - 1];
                let owned = |v: i32| {
                    patched_corners.is_some_and(|c| c.contains_key(&v))
                        || filled_corners.is_some_and(|c| c.contains_key(&v))
                        || blend_handovers.is_some_and(|c| c.keys().any(|k| k.0 == v))
                        || blend_capped_corners.is_some_and(|c| c.keys().any(|k| k.0 == v))
                        || patch_seam_rings.is_some_and(|c| c.keys().any(|k| k.0 == v))
                };
                let mut start_is_free = ends.start_pair_corner.is_none()
                    && ends.start_overshoot_corner.is_none()
                    && !owned(first_edge.from_vertex());
                let mut end_is_free = ends.end_pair_corner.is_none()
                    && ends.end_overshoot_corner.is_none()
                    && !owned(last_edge.to_vertex());

                // A BEAD keeps the run-wide reading it always had: a bead carries a void wedge and
                // a clip swept along the very same run, and at a 5.3 corner those are built out of
                // two beads' overshoots lying over each other (BevelTwoBeadCornerClipTests). Only
                // a CUTTER, whose tool is the whole of what it contributes, is read per end.
                if is_bead && !(start_is_free && end_is_free) {
                    start_is_free = false;
                    end_is_free = false;
                }

                if start_is_free || end_is_free {
                    // Each end reaches by ITS OWN edge's tangent distance: the two ends of a run
                    // need not meet their faces at the same angle.
                    let reach = |end: &ChainEdge| {
                        bevel_edge_tool_builder::tangent_distance(
                            size,
                            180.0 - end.edge.turn_angle_degrees.abs(),
                            round_profile,
                        )
                    };

                    taper = BevelRunTaper::create(
                        &chain,
                        &resolution.features,
                        surface_of_face(),
                        selected,
                        tolerance,
                        if start_is_free {
                            reach(first_edge)
                        } else {
                            0.0
                        },
                        if end_is_free { reach(last_edge) } else { 0.0 },
                    );

                    // A runout is a reach PAST the run's own end, and it takes the same bound every
                    // other such reach takes: half the END EDGE that has to pay for it
                    // (BevelChainEnds.StartMaximumOvershoot, BevelChainEndBudget.MaximumOvershoot).
                    // On a tessellated rim - a 1.57 mm facet asked for a 2 mm fillet - the size
                    // alone would run the runout clean over the rim vertex next door
                    // (BevelShortRunOvershootTests).
                    if let Some(found) = taper.as_mut() {
                        found.start_length = found
                            .start_length
                            .min((first_edge.to() - first_edge.from()).length() / 2.0);
                        found.end_length = found
                            .end_length
                            .min((last_edge.to() - last_edge.from()).length() / 2.0);
                    }
                }
            }

            // The lone unlinked edge's own turn at the partnered builder, taken only where its
            // taper was refused - which is the order that case has always been decided in.
            if taper.is_none() && lone_unlinked_edge {
                if let Some(partnered) = try_partnered_bead() {
                    group_tools.push(partnered_tool(partnered));
                    continue;
                }
            }

            // A cutter goes through BevelCapClip rather than straight to the sweep: where two end
            // caps lean into each other over a short run the sweep cannot carry them, and the
            // plane comes off the finished solid as a half space instead. The wedge takes the FULL
            // back: only the tool is swept with the shallower one the mitre split asked for.
            // Neither is handed the rounds (the second pass rebuilds beads alone).
            let mut sweep_tapered = |tapered: &BevelRunTaper,
                                     taped_ends: &BevelChainEnds,
                                     section: BevelSweptSection,
                                     depth: f64|
             -> Option<Mesh> {
                match sweep_taper.as_mut() {
                    Some(sweep) => sweep(tapered, taped_ends, section),
                    None => bevel_tapered_sweep::try_create(
                        tapered,
                        taped_ends,
                        size,
                        segments,
                        round_profile,
                        section,
                        depth,
                    )
                    .ok(),
                }
            };

            let mut failure_reason: Option<BevelAdvisory> = None;
            let sweep_plain = |said: &mut Vec<BevelAdvisory>| {
                bevel_chain_sweep::try_create(
                    &chain.edges,
                    chain.is_closed,
                    Some(&ends),
                    size,
                    segments,
                    round_profile,
                    BevelSweptSection::Tool,
                    0.0,
                    0.0,
                    Some(said),
                    back_depth,
                    rounds,
                )
            };

            let mut tool: Option<Mesh> = if let Some(tapered) = taper.as_ref() {
                sweep_tapered(tapered, &ends, BevelSweptSection::Tool, back_depth)
            } else if is_bead || !may_clip_caps {
                match sweep_plain(&mut said) {
                    Ok(mesh) => Some(mesh),
                    Err(reason) => {
                        failure_reason = Some(reason);
                        None
                    }
                }
            } else {
                match bevel_cap_clip::try_create(
                    &chain.edges,
                    chain.is_closed,
                    Some(&ends),
                    size,
                    segments,
                    round_profile,
                    Some(&mut said),
                    back_depth,
                ) {
                    Ok((mesh, clips)) => {
                        // C# hands back null clips where the caps were swept.
                        cap_clips = (!clips.is_empty()).then_some(clips);
                        Some(mesh)
                    }
                    Err(reason) => {
                        failure_reason = Some(reason);
                        None
                    }
                }
            };

            // A RUN-OUT IS AN IMPROVEMENT ON THE SWEEP, NOT A PRECONDITION FOR IT. The plain sweep
            // of this same chain was buildable before the taper existed and still is, so the run
            // falls back to the tool it would have had with the taper turned off. The taper's own
            // reason is dropped rather than warned about; clearing the taper here keeps the void
            // wedge below swept the same way as the tool it trims.
            if tool.is_none() && taper.is_some() {
                taper = None;
                match sweep_plain(&mut said) {
                    Ok(mesh) => tool = Some(mesh),
                    Err(reason) => failure_reason = Some(reason),
                }
            }

            let Some(tool) = tool else {
                // Every path that leaves no tool has set its reason (the tapered sweep's refusal is
                // always followed by the plain sweep's), so the fallback below is never taken.
                return Err(BevelGroupToolsRefusal {
                    reason: failure_reason.unwrap_or_else(|| {
                        BevelAdvisory::new(BevelAdvisoryCode::SurfaceConstructionNotBuilt, vec![])
                    }),
                    warnings: published(said),
                });
            };

            // A bead carries the void it may fill along with it, swept along this same chain with
            // the same frames, joints and ends. It is swept AFTER the tool and its failure is
            // swallowed: a wedge is a trim, not a feature. With ONE end treatment of its own, and
            // it is the only place the bead and its wedge part company: a 5.3 corner (see
            // BevelCornerVoids).
            let mut void_wedge = None;
            let mut fallback_wedge = None;
            let mut chain_corner_voids: Option<Vec<Rc<Mesh>>> = None;

            if is_bead {
                let first_edge = &chain.edges[0];
                let last_edge = &chain.edges[chain.edges.len() - 1];

                let mut void_at_start = corner_voids.and_then(|voids| {
                    voids.try_read(
                        ends.start_overshoot_corner,
                        first_edge,
                        first_edge.from(),
                        size,
                        segments,
                        round_profile,
                    )
                });
                let mut void_at_end = corner_voids.and_then(|voids| {
                    voids.try_read(
                        ends.end_overshoot_corner,
                        last_edge,
                        last_edge.to(),
                        size,
                        segments,
                        round_profile,
                    )
                });

                // Only an end whose corner CAN be read is squared off. An end whose corner cannot
                // keeps the wedge it had - never a wedge squared with nothing put back in its
                // place, which would saw the corner the two beads meet in.
                let squaring = void_at_start.is_some() || void_at_end.is_some();
                let squared_ends = squaring.then(|| {
                    ends.squared_at(&chain.edges, void_at_start.is_some(), void_at_end.is_some())
                });

                let swept = sweep_void_wedges(
                    &ends,
                    squared_ends.as_ref(),
                    |wedge_ends: &BevelChainEnds, squaring_a_corner: bool| {
                        // The walls of the wedge itself are buried in the material - EXCEPT on the
                        // squared sweep, the only wedge the clip ever unions with a 5.3 corner's
                        // void, whose walls lie on the same two faces
                        // (BevelSweptSection.VoidWedgeOnTheFaces).
                        let wedge_section = if squaring_a_corner {
                            BevelSweptSection::VoidWedgeOnTheFaces
                        } else {
                            BevelSweptSection::VoidWedge
                        };

                        match taper.as_ref() {
                            Some(tapered) => sweep_tapered(tapered, wedge_ends, wedge_section, 0.0),
                            None => bevel_chain_sweep::try_create(
                                &chain.edges,
                                chain.is_closed,
                                Some(wedge_ends),
                                size,
                                segments,
                                round_profile,
                                wedge_section,
                                0.0,
                                0.0,
                                None,
                                0.0,
                                rounds,
                            )
                            .ok(),
                        }
                    },
                );

                void_wedge = swept.0;
                fallback_wedge = swept.1;

                if !swept.2 {
                    void_at_start = None;
                    void_at_end = None;
                }

                if void_at_start.is_some() || void_at_end.is_some() {
                    let mut corners: Vec<Rc<Mesh>> = Vec::with_capacity(2);

                    if let Some(at_start) = &void_at_start {
                        corners.push(Rc::clone(at_start));
                    }

                    if let Some(at_end) = &void_at_end {
                        if !void_at_start
                            .as_ref()
                            .is_some_and(|at_start| Rc::ptr_eq(at_start, at_end))
                        {
                            corners.push(Rc::clone(at_end));
                        }
                    }

                    chain_corner_voids = Some(corners);
                }
            }

            // And a CUTTER that overshoots into another selected run carries the tube its own
            // rolling ball leaves, for the same kind of reason and swept the same way. Only at a
            // pair corner: everywhere else the union of the cutters is already the answer. Its
            // failure is swallowed like the wedge's - a capped end leaning off square drags the far
            // side of the wider ring back past the last segment, and a corner whose tube would not
            // sweep is left as plan 5.3 left it. Read for a bead too, and recorded on the tool,
            // purely for symmetry with BevelChainEnds.StartPairCorner.
            let pair_corners = bevel_pair_corner_trim::corners_of(Some(&ends));

            let wants_tube = !is_bead && round_profile && pair_corners.is_some();

            // The tube's own two ends, which are the ONLY thing swept here that does not stop where
            // the cutter beside it stops: it has to reach the partner's arc axis
            // (BevelPairCornerReach).
            if wants_tube && selected_mesh_edges.is_none() {
                selected_mesh_edges = Some(self::selected_mesh_edges(selected));
            }

            let (to_start_partner, to_end_partner) = if wants_tube {
                (
                    bevel_pair_corner_reach::to_partner_axis(
                        ends.start_pair_corner,
                        &chain.edges[0],
                        true,
                        Some(&resolution.features),
                        selected_mesh_edges.as_ref(),
                        size,
                    ),
                    bevel_pair_corner_reach::to_partner_axis(
                        ends.end_pair_corner,
                        &chain.edges[chain.edges.len() - 1],
                        false,
                        Some(&resolution.features),
                        selected_mesh_edges.as_ref(),
                        size,
                    ),
                )
            } else {
                (0.0, 0.0)
            };

            let ball_sweep = if wants_tube {
                bevel_chain_sweep::try_create(
                    &chain.edges,
                    chain.is_closed,
                    Some(&ends),
                    size,
                    segments,
                    round_profile,
                    BevelSweptSection::Ball,
                    to_start_partner,
                    to_end_partner,
                    None,
                    0.0,
                    rounds,
                )
                .ok()
            } else {
                None
            };

            // Only a BEAD is held inside the faces its run leaves through: a cutter that overshoots
            // takes material away out there and stopping it early is what leaves the gouge
            // unfinished (BevelBeadExit says which direction each is wrong in).
            let mut built = BevelTool::new(tool, is_bead);
            built.void_wedge = void_wedge;
            built.ball_sweep = ball_sweep;
            built.pair_corners = pair_corners;
            built.cap_clips = cap_clips;
            built.corner_voids = chain_corner_voids;
            built.fallback_wedge = fallback_wedge;
            built.exit_planes = if is_bead {
                bevel_bead_exit::of(Some(&ends))
            } else {
                None
            };
            group_tools.push(built);
        }
    }

    Ok(BevelGroupToolsBuilt {
        tools: group_tools,
        warnings: published(said),
    })
}

/// A partnered bead's tool: a bead already bounded where it was built.
fn partnered_tool(mesh: Mesh) -> BevelTool {
    let mut tool = BevelTool::new(mesh, true);
    tool.bounded_at_sweep = true;
    tool
}

/// The selection's mesh edges as a set, which is how `BevelPairCornerReach` asks whether an edge
/// arriving at a corner is one of the runs it has to reach past.
fn selected_mesh_edges(selected: &[FeatureEdge]) -> HashSet<i32> {
    selected.iter().map(|edge| edge.mesh_edge_index).collect()
}

/// C# `SweepVoidWedges`: one bead's void wedge swept onto its 5.3 ends squared off where the corner
/// out there could be read, and - where that happened - the same wedge swept onto the ends it
/// always had, for the clip to fall back to.
///
/// `ends` are the ends the BEAD itself was swept onto, overshoots and all. `squared_ends` are the
/// same ends with the readable 5.3 corners capped square (`BevelChainEnds.SquaredAt`), or `None`
/// where there was nothing to square - C# reads that as `SquaredAt` handing back the very same
/// object. `sweep` sweeps the wedge onto a given set of ends, told whether they are the SQUARED
/// ones; it is injected because what this function IS is the choice between two sweeps, and the
/// branches that matter are the ones where a sweep comes back `None`, which no fixture produces.
///
/// Returns the wedge to clip with, the one to fall back to where the first will not do (`None`
/// where there is none), and whether the corners were squared off at all - the caller's cue to keep
/// or drop the cones that go with them.
///
/// Two sweeps rather than one, and the second is not waste: it is the clip's fallback, swept here
/// where sweeping is cheap. The other way a squared sweep can be lost is the fold test: squaring an
/// end SHORTENS the strip that test measures its drag across, so a chain whose OTHER end sits on a
/// leaning cap over a short segment could sweep its wedge with the overshoot and not without it.
pub fn sweep_void_wedges(
    ends: &BevelChainEnds,
    squared_ends: Option<&BevelChainEnds>,
    mut sweep: impl FnMut(&BevelChainEnds, bool) -> Option<Mesh>,
) -> (Option<Mesh>, Option<Mesh>, bool) {
    let Some(squared_ends) = squared_ends else {
        // Nothing was squared, so there is one wedge and nothing to fall back from.
        return (sweep(ends, false), None, false);
    };

    let squared = sweep(squared_ends, true);
    let overshot = sweep(ends, false);

    match squared {
        Some(squared) => (Some(squared), overshot, true),
        None => (overshot, None, false),
    }
}
