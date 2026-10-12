//! The kernel's own bead sequence as several C# bevel test classes each carry a private copy of
//! it (`BevelRecessApexRunOutTests.BeadTheCreaseAsync`, `BevelBlockedCreaseEndTests`,
//! `BevelCenterBottomCornerTests`, `BevelCentreFlowsThroughTests`, ...), shared here once: the
//! tools each group builds - split into the solids that go on whole and the beads the clip trims -
//! and the clip and union that finishes the part (`BevelBeadClip.ApplyAllAsync`, then the union).

use std::collections::HashSet;
use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::bevel_bead_clip::{self, Clippable, Tally};
use geometry_processing::bevel::bevel_corner_patch;
use geometry_processing::bevel::bevel_face_fit::DEFAULT_SURFACE_TOLERANCE_DEGREES;
use geometry_processing::bevel::bevel_group_tools::{self, BevelGroupToolsOptions};
use geometry_processing::bevel::bevel_partnered_bead_partners::concave_runs_across_groups;
use geometry_processing::bevel::{
    bevel_mixed_blends, bevel_shared_edges, BevelCornerVoids, BevelFaceFit, BevelFaceRunOut,
    BevelRebuildAdvisories, BevelSurfaceLabels, BevelTool,
};
use geometry_processing::topology::bevel_edge_group_resolver;
use geometry_processing::topology::{BevelEdgeGroup, FeatureEdge};
use polygon_mesh::csg::{BooleanError, CsgModes};
use polygon_mesh::Mesh;
use vector_math::Matrix4X4;

use crate::kernel_cut_helpers::boolean;

/// What the groups built: the part and every solid going on whole, and the beads the clip trims.
pub struct Built {
    pub solids: Vec<Mesh>,
    pub clippable: Vec<Clippable>,
}

/// The light sequence (`BevelBlockedCreaseEndTests`, `BevelCenterBottomCornerTests`,
/// `BevelCentreFlowsThroughTests`): resolve, the corner patches, the concave runs and the run-out,
/// and the tools. `None` where a group did not build. The clippable beads carry no exit planes,
/// as C#'s four-argument `Clippable` there.
pub fn light_tools(
    part: &Mesh,
    groups: &[BevelEdgeGroup],
    radius: f64,
    segments: i32,
) -> Option<Built> {
    let (resolutions, _) = bevel_edge_group_resolver::resolve_all(groups, part);
    let eligible: Vec<bool> = resolutions.iter().map(|r| r.is_resolved()).collect();
    let patches = bevel_corner_patch::create(&resolutions, &eligible, true, radius, segments);
    let selected: Vec<FeatureEdge> = resolutions
        .iter()
        .flat_map(|r| r.edges.iter().copied())
        .collect();
    let concave_runs = concave_runs_across_groups(&resolutions, Some(&eligible));
    let run_out = Rc::new(BevelFaceRunOut::create(
        Arc::new(part.clone()),
        None,
        DEFAULT_SURFACE_TOLERANCE_DEGREES,
    ));
    let drive_through: HashSet<i32> = patches.runs_may_drive_through.iter().copied().collect();

    let mut solids = vec![part.clone()];
    let mut clippable = Vec::new();

    for resolution in &resolutions {
        let built = bevel_group_tools::try_build(
            resolution,
            true,
            radius,
            segments,
            BevelGroupToolsOptions {
                selected_across_groups: Some(&selected),
                patched_corners: Some(&patches.ball_centres),
                filled_corners: Some(&patches.fill_ball_centres),
                runs_may_drive_through: Some(&drive_through),
                patch_seam_rings: Some(&patches.seam_rings),
                concave_runs_across_groups: Some(&concave_runs),
                run_out: Some(run_out.clone()),
                ..BevelGroupToolsOptions::default()
            },
        )
        .ok()?;

        sort(built.tools, &mut solids, &mut clippable, false);
    }

    Some(Built { solids, clippable })
}

/// The full sequence `BevelFeatureMeshBuilder` runs (`BevelRecessApexRunOutTests`,
/// `BevelTwoBeadCornerClipTests`): resolve, award, face fit, the cross-group readings (corner
/// voids, surface labels, corner patches, mixed blends, concave runs, run-out) and the tools, the
/// clippable beads carrying their exit planes. Panics, as C# throws, where a group does not fit or
/// does not build.
pub fn full_tools(part: &Mesh, groups: &[BevelEdgeGroup], radius: f64, segments: i32) -> Built {
    let (resolutions, graph) = bevel_edge_group_resolver::resolve_all(groups, part);
    let (resolutions, _) = bevel_shared_edges::award_once(resolutions);
    let graph = Rc::new(graph.unwrap_or_else(|| part.get_mesh_edge_graph()));
    let shared_part = Arc::new(part.clone());
    let face_fit = BevelFaceFit::create(
        shared_part.clone(),
        Rc::new(resolutions.clone()),
        true,
        radius,
        DEFAULT_SURFACE_TOLERANCE_DEGREES,
        Some(graph.clone()),
    );

    let mut eligible = vec![false; resolutions.len()];
    let mut trimmed_by_a_wall = vec![false; resolutions.len()];

    for i in 0..resolutions.len() {
        let check = face_fit.try_check_group_fully(i);

        if let Some(does_not_fit) = &check.failure {
            panic!("group {i} does not fit: {:?}", does_not_fit.code());
        }

        trimmed_by_a_wall[i] = check.trimmed_by_a_wall_it_ends_on;
        eligible[i] = true;
    }

    let corner_voids = BevelCornerVoids::new(shared_part.clone(), Some(graph.clone()));
    let mut surface_labels = BevelSurfaceLabels::new(part, Some(&graph));
    let selected: Vec<FeatureEdge> = resolutions
        .iter()
        .flat_map(|r| r.edges.iter().copied())
        .collect();
    let patches = bevel_corner_patch::create(&resolutions, &eligible, true, radius, segments);
    let mixed_blends = bevel_mixed_blends::create(&resolutions, &eligible, true, radius, segments);
    let concave_runs = concave_runs_across_groups(&resolutions, Some(&eligible));
    let run_out = Rc::new(BevelFaceRunOut::create(
        shared_part,
        Some(graph.clone()),
        DEFAULT_SURFACE_TOLERANCE_DEGREES,
    ));
    let drive_through: HashSet<i32> = patches.runs_may_drive_through.iter().copied().collect();

    let mut solids = vec![part.clone()];
    let mut clippable = Vec::new();

    for (i, resolution) in resolutions.iter().enumerate() {
        let built = bevel_group_tools::try_build(
            resolution,
            true,
            radius,
            segments,
            BevelGroupToolsOptions {
                selected_across_groups: Some(&selected),
                patched_corners: Some(&patches.ball_centres),
                blend_handovers: Some(mixed_blends.convex_handovers()),
                blend_capped_corners: Some(mixed_blends.concave_cap_normals()),
                filled_corners: Some(&patches.fill_ball_centres),
                may_clip_caps: trimmed_by_a_wall[i],
                corner_voids: Some(&corner_voids),
                surface_labels: Some(&mut surface_labels),
                runs_may_drive_through: Some(&drive_through),
                patch_seam_rings: Some(&patches.seam_rings),
                concave_runs_across_groups: Some(&concave_runs),
                run_out: Some(run_out.clone()),
                ..BevelGroupToolsOptions::default()
            },
        )
        .unwrap_or_else(|refusal| panic!("group {i} did not build: {:?}", refusal.reason.code()));

        sort(built.tools, &mut solids, &mut clippable, true);
    }

    Built { solids, clippable }
}

/// Every bead with a void wedge goes to the clip; everything else goes on whole.
fn sort(
    tools: Vec<BevelTool>,
    solids: &mut Vec<Mesh>,
    clippable: &mut Vec<Clippable>,
    with_exit_planes: bool,
) {
    for tool in tools {
        if tool.is_bead {
            if let Some(wedge) = tool.void_wedge {
                clippable.push(Clippable::new(
                    tool.mesh,
                    wedge,
                    tool.corner_voids,
                    tool.fallback_wedge,
                    if with_exit_planes {
                        tool.exit_planes
                    } else {
                        None
                    },
                ));
                continue;
            }
        }

        solids.push(tool.mesh);
    }
}

/// `BevelBeadClip.ApplyAllAsync` over the clippable solids, then the union of everything: the
/// finished part and the clip's tally.
pub fn clip_and_union(
    part: &Mesh,
    clippable: &[Clippable],
    mut solids: Vec<Mesh>,
) -> (Mesh, Tally) {
    let combine = |operands: &[(Mesh, Matrix4X4)], mode: CsgModes| -> Result<Mesh, BooleanError> {
        let meshes: Vec<&Mesh> = operands.iter().map(|(mesh, _)| mesh).collect();
        Ok(boolean(&meshes, mode))
    };
    let mut advisories = BevelRebuildAdvisories::new(None::<Vec<_>>);
    let (clipped, tally) = bevel_bead_clip::apply_all(
        part,
        Matrix4X4::IDENTITY,
        clippable,
        &mut solids,
        &mut advisories,
        &combine,
    )
    .expect("not cancelled");

    solids.extend(clipped);

    (
        boolean(&solids.iter().collect::<Vec<_>>(), CsgModes::Union),
        tally,
    )
}
