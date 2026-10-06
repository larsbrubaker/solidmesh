//! The plane a chain END is capped on (`Bevel/BevelChainEndPlane.cs`, a static class): handed
//! over to a corner patch or blend where one owns the vertex; otherwise the face beyond the end
//! that agrees best with the way the run leaves, the run running on or dying buried past an
//! interrupted crease, a valley fold's mitre, or no plane at all.

use std::collections::HashSet;

use indexmap::IndexMap;
use vector_math::Vector3;

use super::bevel_interrupted_crease::{Interruption, Leaning};
use super::bevel_pair_corner_crease::PairFaces;
use super::{
    bevel_bead_exit, bevel_corner_handover, bevel_corner_seam, bevel_pair_corner_crease,
    bevel_valley_fold_mitre, BevelAdvisory, BevelAdvisoryCode, BevelEndCap, BevelSweptChain,
};
use crate::topology::{FeatureEdge, FeatureEdgeSet};

/// What C# `ResolveEnd` hands back besides the cap: the pair corner the end stands at, whether a
/// selected run meets it, the interruption it runs on past, and the faces a bead leaves through.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ResolvedEnd {
    /// The capping plane - the default (no plane) where the end is not capped here.
    pub cap: BevelEndCap,
    /// The vertex, where it is a pair corner (a concave crease between two selected runs).
    pub pair_corner_vertex: Option<i32>,
    /// Whether another selected run meets this end.
    pub overlaps_selected_run: bool,
    /// The interrupted crease a bead runs on past.
    pub interruption: Interruption,
    /// The faces a bead leaves through, where it squares or dies buried.
    pub exit_planes: Option<Vec<BevelEndCap>>,
}

/// The corner patches' and blends' readings `ResolveEnd` hands over to, all optional.
#[derive(Debug, Clone, Copy, Default)]
pub struct EndHandovers<'a> {
    /// C# `patchedCorners`.
    pub patched_corners: Option<&'a IndexMap<i32, Vector3>>,
    /// C# `blendHandovers`.
    pub blend_handovers: Option<&'a IndexMap<(i32, i32), BevelEndCap>>,
    /// C# `blendCappedCorners`.
    pub blend_capped_corners: Option<&'a IndexMap<(i32, i32), Vector3>>,
    /// C# `runsMayDriveThrough`.
    pub runs_may_drive_through: Option<&'a HashSet<i32>>,
    /// C# `patchSeamRings`.
    pub patch_seam_rings: Option<&'a IndexMap<(i32, i32), Vec<Vector3>>>,
}

/// C# `BevelChainEndPlane.ResolveEnd`.
#[allow(clippy::too_many_arguments)]
pub fn resolve_end<'s>(
    vertex: i32,
    position: Vector3,
    outward: Vector3,
    own_face0: i32,
    own_face1: i32,
    own_normal0: Vector3,
    own_normal1: Vector3,
    own_edge: i32,
    is_bead: bool,
    minimum_agreement: f64,
    reach: f64,
    features: &FeatureEdgeSet,
    selected: &HashSet<i32>,
    mut warnings: Option<&mut Vec<BevelAdvisory>>,
    handovers: EndHandovers,
    mut surface_of_face: Option<&mut dyn FnMut() -> Option<&'s [i32]>>,
    concave_runs: Option<&[BevelSweptChain]>,
) -> ResolvedEnd {
    let mut resolved = ResolvedEnd::default();

    if let Some(handover) = bevel_corner_handover::try_hand_over(
        vertex,
        position,
        outward,
        own_edge,
        is_bead,
        handovers.patched_corners,
        handovers.blend_handovers,
        handovers.blend_capped_corners,
        handovers.runs_may_drive_through,
        handovers.patch_seam_rings,
    ) {
        resolved.cap = handover;
        return resolved;
    }

    let incident = features.edges_at_vertex(vertex);
    let exits = || {
        if is_bead {
            bevel_bead_exit::faces_the_run_leaves_through(
                incident, features, position, outward, own_face0, own_face1,
            )
        } else {
            None
        }
    };

    let mut saw_candidate = false;
    let mut saw_selected_neighbour = false;
    let mut leaning = Leaning::default();
    let mut saw_concave_crease_between_runs = false;
    let mut best_agreement = 0.0;
    let mut best_normal = Vector3::ZERO;
    let mut best_closes_over = false;
    let mut neighbour_normals: Vec<Vector3> = Vec::new();
    let mut neighbour_faces: Vec<i32> = Vec::new();
    let mut neighbour_edges: Vec<FeatureEdge> = Vec::new();

    for &i in incident {
        let edge = &features.edges()[i as usize];
        if !selected.contains(&edge.mesh_edge_index) || edge.mesh_edge_index == own_edge {
            continue;
        }

        saw_selected_neighbour = true;
        resolved.overlaps_selected_run = true;
        neighbour_normals.push(edge.normal0);
        neighbour_normals.push(edge.normal1);
        neighbour_faces.push(edge.face0);
        neighbour_faces.push(edge.face1);
        neighbour_edges.push(*edge);
    }

    for &i in incident {
        let edge = &features.edges()[i as usize];
        if selected.contains(&edge.mesh_edge_index) {
            continue;
        }

        if saw_selected_neighbour
            && bevel_pair_corner_crease::lies_between_selected_runs(
                edge,
                own_normal0,
                own_normal1,
                &neighbour_normals,
                PairFaces {
                    own_face0,
                    own_face1,
                    neighbour_faces: Some(&neighbour_faces),
                },
                match surface_of_face {
                    // Reborrowed for this one reading, the closure's own lifetime shortened to it.
                    Some(ref mut ask) => Some(&mut **ask as &mut dyn FnMut() -> Option<&'s [i32]>),
                    None => None,
                },
            )
        {
            saw_concave_crease_between_runs |=
                bevel_pair_corner_crease::takes_the_partners_ball(edge);
            continue;
        }

        for side in 0..2 {
            let face = if side == 0 { edge.face0 } else { edge.face1 };
            if face == own_face0 || face == own_face1 {
                continue;
            }

            saw_candidate = true;
            let normal = if side == 0 {
                edge.normal0
            } else {
                edge.normal1
            };
            let along_outward = normal.dot(outward);
            let agreement = along_outward.abs();
            leaning.saw(along_outward);
            if agreement < minimum_agreement || agreement <= best_agreement {
                continue;
            }

            best_agreement = agreement;
            best_normal = if along_outward < 0.0 { -normal } else { normal };
            best_closes_over = along_outward < 0.0;
        }
    }

    if best_agreement > 0.0 {
        resolved.cap = BevelEndCap::new(
            best_normal,
            best_normal.dot(position)
                + bevel_corner_seam::stand_off(is_bead && !best_closes_over, position),
        );
        return resolved;
    }

    if is_bead && !saw_selected_neighbour {
        resolved.interruption = leaning.runs_on(
            saw_candidate,
            incident,
            features,
            own_edge,
            outward,
            own_normal0,
            own_normal1,
            position,
            concave_runs,
        );
        if resolved.interruption.runs_on() {
            return resolved;
        }

        if leaning.dies_buried(
            saw_candidate,
            incident,
            features,
            own_edge,
            outward,
            position,
            reach,
        ) {
            resolved.exit_planes = exits();
            return resolved;
        }

        if saw_candidate {
            if let Some(warnings) = warnings.as_mut() {
                warnings.push(BevelAdvisory::new(
                    BevelAdvisoryCode::BeadSquaredAtCorner,
                    vec![position.into()],
                ));
            }
        }

        resolved.exit_planes = exits();
        resolved.cap = BevelEndCap::new(outward, outward.dot(position));
        return resolved;
    }

    if is_bead && neighbour_edges.len() == 1 {
        if let Some(valley_mitre) = bevel_valley_fold_mitre::try_mitre(
            vertex,
            position,
            outward,
            own_normal0,
            own_normal1,
            &neighbour_edges[0],
        ) {
            if saw_concave_crease_between_runs {
                resolved.pair_corner_vertex = Some(vertex);
            }
            resolved.cap = valley_mitre;
            return resolved;
        }
    }

    if saw_candidate && !is_bead {
        if let Some(warnings) = warnings.as_mut() {
            warnings.push(BevelAdvisory::new(
                BevelAdvisoryCode::CutRunsPastCorner,
                vec![position.into()],
            ));
        }
    }

    if saw_concave_crease_between_runs {
        resolved.pair_corner_vertex = Some(vertex);
    }

    resolved
}
