//! The corner where a filleted CONVEX edge runs into a filleted CONCAVE one - the L bracket's
//! ends - blended rather than left with a fin of bead standing beside a rounded rim. Ports
//! `Bevel/BevelMixedBlends.cs` (C# `internal static class`; a module of free functions here).
//!
//! This half reads topology - which edges met, which faces they share, whether the corner leans
//! further than the budget - and decides which corners get a tool and what each refusal says. The
//! trigonometry that draws the tool is [`super::bevel_mixed_blend_tool`]; the record it fills is
//! [`BevelMixedBlendSet`], and the corners themselves are gathered by [`BevelMixedCornerEnds`].
//!
//! The blend is a continuation of the CUT, not of the bead: after the concave bead lands, the rim
//! of the cap face is a line, then the arc where the bead's cylinder cuts it, then a line, all
//! tangent; the blend is the fillet of that arc - a ball of the same radius tangent to the cap face
//! and rolling on the concave cylinder from the material side, swept round into a torus.

use indexmap::IndexMap;
use polygon_mesh::csg::boolean_processing::{classify_boolean_operand, BooleanOperandVerdict};
use polygon_mesh::Mesh;
use vector_math::{math_helper, Matrix4X4, Vector3};

use super::bevel_corner_face_ring;
use super::bevel_mixed_blend_tool;
use super::bevel_mixed_corners;
use super::{
    BevelAdvisory, BevelAdvisoryArg, BevelAdvisoryCode, BevelEndCap, BevelMixedBlend,
    BevelMixedBlendSet, BevelMixedCornerEnds,
};
use crate::topology::{BevelEdgeGroupResolution, EdgeConvexity, FeatureEdge};

/// One cut running into one bead is the simplest corner this blends; one is not a corner at all.
const EDGES_IN_A_MIXED_PAIR: usize = 2;

/// And three is the most, because at four a corner patch claims the vertex too. Three is a ring
/// of exactly three faces or it is nothing.
const EDGES_IN_A_MIXED_RING: usize = 3;

/// A one segment fillet is a chamfer chord, and there is no arc on the rim to blend along.
const MINIMUM_SEGMENTS_FOR_A_BLEND: i32 = 2;

/// How far off square to the cap face the concave edge may lean, as a multiple of HALF the rim's
/// own chord step - so the budget tightens as the tessellation gets finer.
///
/// Measured, not derived: the algebra predicts one half step (the residual `r (1 - cos(phi))`
/// reaches the seam's own miter ledge `r (1 / cos(step / 2) - 1)` at `phi = step / 2`), but the
/// tool does better; its lean excess crosses the ledge at 1.85 half steps at 2 segments, 2.3 at 4
/// and about 2.7 by 32. So 1.5, the coarsest tessellation's own crossing less a fifth. At a
/// right-angle rim that is 8.4 degrees of lean at 8 segments, 4.2 at 16 and 2.1 at 32.
const OFF_SQUARE_BUDGET_IN_HALF_STEPS: f64 = 1.5;

/// How nearly the concave edge may lie IN the cap face before there is no depth to measure the
/// blend in at all - a different refusal from the lean budget, and the one that keeps every
/// division by `along_edge . cap_face` finite. It holds exactly when the two picked edges are
/// collinear, which no well formed mesh presents; it stays because a division should be guarded
/// where it is made.
const MINIMUM_CAP_FACE_DEPTH: f64 = 1e-6;

/// How far from square to the convex run the seam plane may lean before that run cannot be
/// stopped on it, in degrees - `BevelChainEnds.MaximumCapPlaneAngleDegrees`'s number and its
/// reason. Unreachable through the lean gate by the same algebra as [`MINIMUM_CAP_FACE_DEPTH`];
/// it stays because `BevelEndCap::slide_to_plane` divides by exactly this quantity.
const MAXIMUM_SEAM_LEAN_DEGREES: f64 = 80.0;

/// How far round the rim has to turn before there is an arc worth blending along, in degrees.
const MINIMUM_TURN_DEGREES: f64 = 1.0;

/// The handover planes, keyed by (mesh vertex, mesh EDGE), as C#'s `Dictionary` in fill order.
type Handovers = IndexMap<(i32, i32), BevelEndCap>;

/// The concave cap normals, keyed the same way.
type CapNormals = IndexMap<(i32, i32), Vector3>;

/// C# `List<BevelAdvisory>[]`: one optional list of warnings per group.
type Warnings = Vec<Option<Vec<BevelAdvisory>>>;

/// The blend tools for a whole selection, read across every group at once - a vertex does not
/// care which group each edge was picked into, and a cut and a bead meeting at one are usually
/// two groups.
///
/// `eligible` says which resolutions resolved and fit; `round_profile` is false for a chamfer,
/// which has no rolling ball to carry round; `radius` and `segments` are the operation's one bevel
/// size and segment count.
pub fn create(
    resolutions: &[BevelEdgeGroupResolution],
    eligible: &[bool],
    round_profile: bool,
    radius: f64,
    segments: i32,
) -> BevelMixedBlendSet {
    let mut warnings: Warnings = vec![None; resolutions.len()];
    let mut blends = Vec::new();
    let mut convex_handovers = Handovers::new();
    let mut concave_cap_normals = CapNormals::new();

    // A chamfer has no rolling ball to carry round a rim, and a one segment fillet is a chamfer
    // chord, so neither has an arc to blend along. The corners are still WALKED in that case
    // rather than returned past: a three-edge mixed corner that gets no blend is the plan's
    // capped-with-a-warning case and has to say so, and this is the only place that knows one was
    // reached.
    let can_blend = round_profile && segments >= MINIMUM_SEGMENTS_FOR_A_BLEND;

    for corner in BevelMixedCornerEnds::collect(resolutions, eligible) {
        blend(
            &corner,
            can_blend,
            radius,
            segments,
            &mut blends,
            &mut convex_handovers,
            &mut concave_cap_normals,
            &mut warnings,
        );
    }

    BevelMixedBlendSet::new(blends, convex_handovers, concave_cap_normals, warnings)
}

/// One corner's blends: one for a mixed pair, one for a corner of two cuts and a bead, and two for
/// a corner of one cut and two beads.
#[allow(clippy::too_many_arguments)]
fn blend(
    corner: &BevelMixedCornerEnds,
    can_blend: bool,
    radius: f64,
    segments: i32,
    blends: &mut Vec<BevelMixedBlend>,
    convex_handovers: &mut Handovers,
    concave_cap_normals: &mut CapNormals,
    warnings: &mut Warnings,
) {
    let edge_count = corner.edges().len();

    if edge_count != EDGES_IN_A_MIXED_PAIR && edge_count != EDGES_IN_A_MIXED_RING {
        // One edge is not a corner; four or more is BevelMixedCorners' warning and its sharp reason.
        return;
    }

    let Some((convex, concave)) = try_get_ends(corner) else {
        return;
    };

    // A THREE edge mixed corner that ends up with no tool is the plan's capped-with-a-warning case
    // and may not be shipped in silence, whatever stood the blend down. Two edges are not that
    // case: a mixed PAIR lands whole either way, each run capped on the other's faces.
    let must_speak = edge_count == EDGES_IN_A_MIXED_RING;

    if !can_blend {
        if must_speak {
            warn(
                corner,
                warnings,
                bevel_mixed_corners::capped_corner(edge_count as i32, corner.position),
            );
        }

        return;
    }

    // Both runs of one convexity here have to be able to STOP at this corner, and a run only stops
    // at a vertex it ENDS at. Two edges of one convexity picked into one group are joined into a
    // single run straight through this corner, so neither takes its handover - and the sweep then
    // crosses the tool's own surface tangentially. Read off the SELECTION rather than off the
    // linked runs, which are not built yet.
    if (convex.len() == 2 && corner.shares_a_group(&convex[0], &convex[1]))
        || (concave.len() == 2 && corner.shares_a_group(&concave[0], &concave[1]))
    {
        // Not BevelMixedCorners' capped line, which says every edge STOPS at the corner: here the
        // point is that one of them does not.
        warn(
            corner,
            warnings,
            advisory(BevelAdvisoryCode::BlendSameGroupNotBlended, corner.position),
        );

        return;
    }

    if edge_count == EDGES_IN_A_MIXED_RING
        && bevel_corner_face_ring::try_order(corner.edges()).is_none()
    {
        // More faces than there are pairings to spend them on, so which face each blend dies on is
        // not a question the corner answers.
        warn(
            corner,
            warnings,
            advisory(BevelAdvisoryCode::BlendMoreThanThreeFaces, corner.position),
        );

        return;
    }

    // Two cuts and a bead are ONE tool with a handover at each end of its rim; every other
    // admitted corner is one tool per bead, each handing over on the single cut.
    let far = if convex.len() == 2 {
        Some(&convex[1])
    } else {
        None
    };

    let mut built = false;
    let mut spoke = false;

    for bead in &concave {
        blend_pair(
            corner,
            &convex[0],
            bead,
            far,
            must_speak,
            radius,
            segments,
            blends,
            convex_handovers,
            concave_cap_normals,
            warnings,
            &mut built,
            &mut spoke,
        );
    }

    if must_speak && !built && !spoke {
        // Nothing was built here and nothing has been said, so the plain count line is both the
        // truth and the whole of it.
        warn(
            corner,
            warnings,
            bevel_mixed_corners::capped_corner(edge_count as i32, corner.position),
        );
    }
}

/// One pair's blend, published only once the tool itself is in hand: a chain told to hand its end
/// over to a blend that then failed to build would stop short around a hole.
///
/// `far` is the second CUT of a two-cut corner, which stops on the far end of this same tool's
/// rim. `built` is set where a tool was added, `spoke` where a refusal found its own words.
#[allow(clippy::too_many_arguments)]
fn blend_pair(
    corner: &BevelMixedCornerEnds,
    convex: &FeatureEdge,
    concave: &FeatureEdge,
    far: Option<&FeatureEdge>,
    must_speak: bool,
    radius: f64,
    segments: i32,
    blends: &mut Vec<BevelMixedBlend>,
    convex_handovers: &mut Handovers,
    concave_cap_normals: &mut CapNormals,
    warnings: &mut Warnings,
    built: &mut bool,
    spoke: &mut bool,
) {
    let tool = match try_build(corner, convex, concave, far, must_speak, radius, segments) {
        Ok(tool) => tool,
        Err(warning) => {
            if let Some(warning) = warning {
                warn(corner, warnings, warning);

                *spoke = true;
            }

            return;
        }
    };

    *built = true;

    blends.push(BevelMixedBlend::new(
        tool.mesh,
        corner.groups().to_vec(),
        tool.cap_face_reach,
        tool.void_face_reach,
        tool.runout_reach,
    ));

    // TryAdd rather than an assignment: at a corner of one cut and two beads both tools hand over
    // on that one cut, on planes that are the same one where the corner is square. Keeping the
    // first is not a preference between them.
    convex_handovers
        .entry((corner.vertex, convex.mesh_edge_index))
        .or_insert(tool.handover);

    if let Some(far) = far {
        convex_handovers
            .entry((corner.vertex, far.mesh_edge_index))
            .or_insert(tool.far_handover);
    }

    concave_cap_normals.insert((corner.vertex, concave.mesh_edge_index), tool.cap_normal);
}

/// C# `Warn`: adds `warning` to every group meeting at the corner.
fn warn(corner: &BevelMixedCornerEnds, warnings: &mut Warnings, warning: BevelAdvisory) {
    for &group in corner.groups() {
        warnings[group as usize]
            .get_or_insert_with(Vec::new)
            .push(warning.clone());
    }
}

/// An advisory naming one point, as every refusal here does.
fn advisory(code: BevelAdvisoryCode, at: Vector3) -> BevelAdvisory {
    BevelAdvisory::new(code, vec![BevelAdvisoryArg::Vector3(at)])
}

/// What [`try_build`] hands back for a tool that built: C#'s `out` parameters.
struct BuiltBlend {
    mesh: Mesh,
    cap_face_reach: f64,
    void_face_reach: f64,
    runout_reach: f64,
    handover: BevelEndCap,
    far_handover: BevelEndCap,
    cap_normal: Vector3,
}

/// One corner's blend, or the reason there is none: `Err(None)` is a corner this was never asked
/// to blend, `Err(Some(warning))` a refusal with words.
///
/// `must_speak` is whether the two DEGENERACY refusals - too little turn, and a corner whose own
/// faces will not solve for a ball path - have to find words. They are silent at a two-edge
/// corner, which lands whole capped either way.
fn try_build(
    corner: &BevelMixedCornerEnds,
    convex: &FeatureEdge,
    concave: &FeatureEdge,
    far: Option<&FeatureEdge>,
    must_speak: bool,
    radius: f64,
    segments: i32,
) -> Result<BuiltBlend, Option<BevelAdvisory>> {
    // The three faces the corner is made of. Both edges' own copies of the shared one agree, but
    // the bead is swept from the CONCAVE edge's normals, so the axis below has to be read from
    // those or the tool would meet the bead a rounding error off its chords.
    let Some((shared_face, cap_face, void_face)) = try_get_faces(convex, concave) else {
        return Err(None);
    };

    let along_edge = away(concave, corner.vertex);

    // How square the edge runs to the cap face, as the cosine of its lean. Everything below
    // measures depth along the edge and lands it on the cap face by dividing by this, so it is
    // read - and refused - before any of that arithmetic runs.
    let squareness = -along_edge.dot(cap_face);

    if squareness < MINIMUM_CAP_FACE_DEPTH {
        // D9: an edge lying IN the face the rim ends on leaves no rim at all.
        return Err(Some(advisory(
            BevelAdvisoryCode::BlendRimRunsAlongFace,
            corner.position,
        )));
    }

    let determinant = shared_face.dot(void_face.cross(along_edge));

    if determinant.abs() < 1e-6 {
        // The basis the ball's axis is solved in has collapsed, so there is no rolling ball path
        // to draw the tool about at all.
        return Err(must_speak.then(|| {
            advisory(
                BevelAdvisoryCode::BlendFacesTooNearlyAligned,
                corner.position,
            )
        }));
    }

    // Where the ball that rolls in the concave corner sits, solved in the basis dual to the two
    // face normals and the edge: the point a radius OUTSIDE both faces and square to the edge,
    // which is exactly the centre BevelEdgeToolBuilder draws the bead's arc about.
    let axis_point = corner.position
        + ((void_face.cross(along_edge) + along_edge.cross(shared_face)) * (radius / determinant));

    let from_shared = -shared_face;
    let from_void = -void_face;

    let turn = from_shared.dot(from_void).clamp(-1.0, 1.0).acos();

    if turn < math_helper::degrees_to_radians(MINIMUM_TURN_DEGREES) {
        // A fold this slight leaves no arc on the rim for the cut to be carried round.
        return Err(must_speak.then(|| {
            advisory(
                BevelAdvisoryCode::BlendCornerFoldsTooLittle,
                corner.position,
            )
        }));
    }

    // The lean budget, which is the rim's own chord step and so is only knowable once the turn is.
    let step = turn / segments as f64;

    if squareness < (OFF_SQUARE_BUDGET_IN_HALF_STEPS * step / 2.0).cos() {
        return Err(Some(advisory(
            BevelAdvisoryCode::BlendCornerNotSquareEnough,
            corner.position,
        )));
    }

    // The far end of the rim RUNS OUT unless a second selected cut is waiting there to take it
    // over.
    let run_out_past_the_rim = far.is_none();

    let tool = bevel_mixed_blend_tool::build(
        corner.position,
        axis_point,
        along_edge,
        cap_face,
        from_shared,
        from_void,
        turn,
        radius,
        segments,
        run_out_past_the_rim,
    );

    if tool.mesh.is_some()
        && (!can_stop_on(&tool.handover, convex, corner.vertex)
            || far.is_some_and(|far| !can_stop_on(&tool.far_handover, far, corner.vertex)))
    {
        // The seam is no use to the run that has to stop on it. Refused here rather than left for
        // BevelChainEnds, because a blend whose handover the chain then could not take would cut
        // where no fillet ever ran up to it.
        return Err(Some(advisory(
            BevelAdvisoryCode::BlendRimTooNearlyAlongCorner,
            corner.position,
        )));
    }

    let mesh = match tool.mesh {
        Some(mesh)
            if classify_boolean_operand(&mesh, Matrix4X4::IDENTITY, false)
                == BooleanOperandVerdict::Clean =>
        {
            mesh
        }
        _ => {
            return Err(Some(advisory(
                BevelAdvisoryCode::BlendNotSolidToCut,
                corner.position,
            )));
        }
    };

    Ok(BuiltBlend {
        mesh,
        cap_face_reach: tool.cap_face_reach,
        void_face_reach: tool.void_face_reach,
        runout_reach: tool.runout_reach,
        handover: tool.handover,
        far_handover: tool.far_handover,
        cap_normal: cap_face,
    })
}

/// Whether the run that ends on a published seam can actually be stopped on it - the plane has to
/// exist and it has to be square enough to that run to say where it ends.
fn can_stop_on(seam: &BevelEndCap, run: &FeatureEdge, vertex: i32) -> bool {
    seam.is_capped()
        && seam.plane_normal.dot(away(run, vertex)).abs()
            >= math_helper::degrees_to_radians(MAXIMUM_SEAM_LEAN_DEGREES).cos()
}

/// The selected ends at this corner sorted by convexity (convex, concave), or `None` where they
/// are not at least one of each.
fn try_get_ends(corner: &BevelMixedCornerEnds) -> Option<(Vec<FeatureEdge>, Vec<FeatureEdge>)> {
    let mut convex = Vec::with_capacity(2);
    let mut concave = Vec::with_capacity(2);

    for end in corner.edges() {
        if end.convexity == EdgeConvexity::Convex {
            convex.push(*end);
        } else if end.convexity == EdgeConvexity::Concave {
            concave.push(*end);
        }
    }

    (!convex.is_empty()
        && !concave.is_empty()
        && convex.len() + concave.len() == corner.edges().len())
    .then_some((convex, concave))
}

/// The corner's three surfaces (shared, cap, void): the one both edges share, the one the convex
/// edge's fillet dies on, and the one the bead's far side lands on. `None` where the two edges do
/// not share exactly one surface. Matched by NORMAL rather than by face index: one flat surface is
/// as many triangles as the tessellation happened to split it into.
fn try_get_faces(
    convex: &FeatureEdge,
    concave: &FeatureEdge,
) -> Option<(Vector3, Vector3, Vector3)> {
    let mut shared_face = Vector3::ZERO;
    let mut cap_face = Vector3::ZERO;
    let mut void_face = Vector3::ZERO;

    let mut shared = 0;

    for i in 0..2 {
        let convex_normal = if i == 0 {
            convex.normal0
        } else {
            convex.normal1
        };

        for j in 0..2 {
            let concave_normal = if j == 0 {
                concave.normal0
            } else {
                concave.normal1
            };

            if convex_normal.dot(concave_normal) <= FeatureEdge::COPLANAR_FACE_DOT {
                continue;
            }

            shared += 1;

            shared_face = concave_normal;
            cap_face = if i == 0 {
                convex.normal1
            } else {
                convex.normal0
            };
            void_face = if j == 0 {
                concave.normal1
            } else {
                concave.normal0
            };
        }
    }

    (shared == 1).then_some((shared_face, cap_face, void_face))
}

/// The direction an edge runs away from one of its own ends.
fn away(edge: &FeatureEdge, vertex: i32) -> Vector3 {
    (if vertex == edge.start_vertex {
        edge.end - edge.start
    } else {
        edge.start - edge.end
    })
    .get_normal()
}
