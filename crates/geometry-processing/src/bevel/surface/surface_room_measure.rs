//! Pacing out the room a cut has on its region (`Bevel/Surface/SurfaceRoomMeasure.cs`, a static
//! class: free functions here). The rulers and the exact offset of a rim segment in front of the
//! chain edge land here first, with the bands a pass takes off a region, the free end, the corner
//! a wall shares and its measure, and the room across one region ([`room_across`]); the whole
//! measure (`Measure`) lands with a later package.

use std::collections::{HashMap, HashSet};
use std::sync::LazyLock;

use vector_math::{cs_math, Vector3};

use polygon_mesh::Mesh;

use super::surface_band_runs_off_the_face_beyond::{self, BandSpan, ChainAtTheSpan};
use super::surface_run_across::SurfaceRunAcrossRegionMapsCache;
use super::{surface_room_standing_feature, surface_run_across_walk};
use super::{SurfaceBandRunsOffOverAStretch, SurfaceBevelPlan};
use crate::bevel::BevelAdvisory;
use crate::topology::EdgeConvexity;
use crate::topology::FaceGroup;

/// How near a rim edge has to come to the chain edge's own line before it is read as being that
/// line rather than in front of it, and how much of the span two segments must share before they
/// are facing each other at all. Relative to the chain edge's length, with a floor of one unit so
/// that a tiny edge on a large part is not measured against a tolerance finer than its own
/// arithmetic.
pub const RELATIVE_TOLERANCE: f64 = 1e-7;

/// How near two points have to be to be the same corner of the rim. Looser than
/// [`RELATIVE_TOLERANCE`], and relative to the chain edge for the same reason, because a mesh
/// keeps its vertices as floats: two readings of one corner differ in the last bit or two of a
/// float, which at 20 mm is already coarser than a tenth of a millionth.
pub const VERTEX_TOLERANCE: f64 = 1e-5;

/// What a region no band of this pass lands on has, so that the reading needs no null case: one
/// shared empty set, as C#'s static instance is.
static NO_BANDS: LazyLock<HashSet<i32>> = LazyLock::new(HashSet::new);

/// Which edges the pass is beveling take a band off `region`, or an empty set where none does.
/// Those are the partners a band on that region negotiates its share of the face with rather than
/// the walls it stops at. Looked up only, so the `HashMap` order cannot reach an output.
pub fn bands_on_the_region<'a>(
    bands_on_region: &'a HashMap<i32, HashSet<i32>>,
    region: &FaceGroup,
) -> &'a HashSet<i32> {
    bands_on_region
        .get(&SurfaceBevelPlan::key_of(region))
        .unwrap_or(&NO_BANDS)
}

/// The smallest offset along `inward` at which the segment `a`..`b` stands in front of the chain
/// edge's own span (from `from`, `length` along unit `along`), or infinity where it does not
/// stand in front of it at all.
///
/// Exact rather than sampled. Both segments are read in the chain edge's (along, inward) frame;
/// the other segment is clipped to the span the chain edge occupies along that frame's first axis,
/// and the offset is linear along it, so its smallest value over the clipped part is at one of the
/// two clipped ends. Two answers are deliberately not offsets: a segment sharing no more than a
/// corner's width of the span, and a segment lying ON the edge's line, which are the chain edge
/// itself and anything collinear with it.
pub fn offset_to_segment(
    from: Vector3,
    along: Vector3,
    length: f64,
    inward: Vector3,
    a: Vector3,
    b: Vector3,
) -> f64 {
    let tolerance = cs_math::max_f64(length, 1.0) * RELATIVE_TOLERANCE;

    // How much of the span the two have to SHARE before they face each other at all is a CORNER's
    // width and not a rounding: a rim that leaves the chain edge's own corner is written in
    // floats, and the stretch of it below the corner overlaps the span by a millionth or so
    // without standing in front of anything. Read at the finer ruler that overlap is a wall the
    // width of a corner, which on the picture hanger's gusset says a 20 mm face is 0.4 mm wide.
    let sharing = cs_math::max_f64(length, 1.0) * VERTEX_TOLERANCE;

    let along_a = (a - from).dot(along);
    let along_b = (b - from).dot(along);
    let offset_a = (a - from).dot(inward);
    let offset_b = (b - from).dot(inward);

    // C#'s Math.Max/Math.Min: NaN in either argument gives NaN.
    let low = cs_math::max_f64(cs_math::min_f64(along_a, along_b), 0.0);
    let high = cs_math::min_f64(cs_math::max_f64(along_a, along_b), length);

    let shared = high - low;
    if shared <= sharing || shared.is_nan() {
        return f64::INFINITY;
    }

    let nearest = cs_math::min_f64(
        offset_at(along_a, offset_a, along_b, offset_b, low),
        offset_at(along_a, offset_a, along_b, offset_b, high),
    );

    if nearest > tolerance {
        nearest
    } else {
        f64::INFINITY
    }
}

/// The offset of the segment at one place along the frame's first axis, interpolated.
pub fn offset_at(along_a: f64, offset_a: f64, along_b: f64, offset_b: f64, at: f64) -> f64 {
    let span = along_b - along_a;

    if span == 0.0 {
        return cs_math::min_f64(offset_a, offset_b);
    }

    offset_a + ((offset_b - offset_a) * ((at - along_a) / span))
}

/// Whether `plan`'s edge `edge_index` ends the chain at its start (or end) with no junction there:
/// a FREE end.
pub fn free_end(plan: &SurfaceBevelPlan, edge_index: usize, at_the_start: bool) -> bool {
    if plan.is_closed
        || (if at_the_start {
            edge_index != 0
        } else {
            edge_index != plan.edges.len() - 1
        })
    {
        return false;
    }

    plan.try_junction_at(at_the_start).is_none()
}

/// Whether a wall's offset is read as the room: not where the room is paced out across the
/// surface, and not where a standing feature hides the wall's far end.
pub fn read_the_offset(
    region: &FaceGroup,
    wall: &crate::topology::FeatureEdge,
    far_end: Vector3,
    corner: f64,
    across_the_surface: bool,
    beveled_mesh_edges: &HashSet<i32>,
    chain_vertices: &HashSet<i32>,
) -> bool {
    !across_the_surface
        && !super::surface_room_standing_feature::hides_the_end_of_a_wall(
            region,
            wall,
            far_end,
            corner,
            beveled_mesh_edges,
            chain_vertices,
        )
}

/// Whether `rim` meets the chain edge at `corner` (within `tolerance`), and its leg out of it.
pub fn shares_the_corner(
    rim: &crate::topology::FeatureEdge,
    corner: Vector3,
    tolerance: f64,
) -> Option<Vector3> {
    if (rim.start - corner).length() <= tolerance {
        return Some(rim.end - corner);
    }

    if (rim.end - corner).length() <= tolerance {
        return Some(rim.start - corner);
    }

    None
}

/// Measures a wall leg meeting the chain edge at its corner: a leg running in front of the band
/// (`inward`) and past the corner's own reach `corner` along the edge raises the share it `taken`
/// (reach over offset), and lowers the `nearest` room where its offset is read.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x > 0)`, which refuses a NaN as well.
pub fn measure_corner(
    leg: Vector3,
    along: Vector3,
    inward: Vector3,
    read_the_offset: bool,
    corner: f64,
    nearest: &mut f64,
    taken: &mut f64,
) {
    let offset = leg.dot(inward);

    if !(offset > 0.0) {
        return;
    }

    let reach = leg.dot(along);

    if !(reach > corner) {
        return;
    }

    *taken = cs_math::max_f64(*taken, reach / offset);

    if read_the_offset {
        *nearest = cs_math::min_f64(*nearest, offset);
    }
}

/// C# `RoomAcross`: how far a cut along the chain edge `from`-`to` can reach into `region` before
/// it meets a wall - infinite where the band runs OFF the region whole; paced out along a curved
/// landing's own surface where that walk answers; the rim it runs off over a stretch charged only
/// over the rest of the span; every other rim read as a corner wall at either end (summed into what
/// the two mitres take from the edge's length) or as a wall straight across. Refusals the run-off
/// readings name are added to `named` on `mesh_edge_index`. Returns the room and the stretch the
/// band runs off over, if any (C#'s `out ranOffOverAStretch`).
#[allow(clippy::too_many_arguments)]
pub fn room_across(
    mesh: &Mesh,
    from: Vector3,
    to: Vector3,
    along: Vector3,
    length: f64,
    inward: Vector3,
    setback: f64,
    region: &FaceGroup,
    beveled_mesh_edges: &HashSet<i32>,
    chain_vertices: &HashSet<i32>,
    bands_on_this_region: &HashSet<i32>,
    region_maps: Option<&mut SurfaceRunAcrossRegionMapsCache>,
    mut named: Option<&mut Vec<BevelAdvisory>>,
    mesh_edge_index: i32,
    chain_convexity: EdgeConvexity,
    free_at_the_start: bool,
    free_at_the_end: bool,
) -> (f64, Option<SurfaceBandRunsOffOverAStretch>) {
    let mut ran_off_over_a_stretch = None;
    let corner = cs_math::max_f64(length, 1.0) * VERTEX_TOLERANCE;
    let span = BandSpan {
        from,
        along,
        length,
        inward,
        setback,
    };

    // A band that takes the WHOLE of this region over its own span runs OFF its convex rim.
    match surface_band_runs_off_the_face_beyond::try_read(
        Some(mesh),
        Some(region),
        &span,
        beveled_mesh_edges,
        chain_vertices,
    ) {
        Ok(Some(_)) => return (f64::INFINITY, None),
        Ok(None) => {}
        Err(why) => {
            // It runs off onto something that is not one plane: carried out by name.
            if let Some(named) = named.as_deref_mut() {
                named.push(why.on_mesh_edge(mesh_edge_index));
            }
        }
    }

    // A CURVED landing is paced out along its own surface instead.
    let run = surface_run_across_walk::try_across_a_curved_region(
        mesh,
        region,
        from,
        to,
        along,
        inward,
        region_maps,
    );
    let across_the_surface = run.is_some();
    let mut nearest = run.unwrap_or(f64::INFINITY);

    // The run-off over PART of the span; not on a landing paced out across its own surface.
    let mut goes_over_these_rims: Option<std::sync::Arc<HashSet<i32>>> = None;
    let mut rest_of_the_span_from = from;
    let mut rest_of_the_span = length;

    if !across_the_surface {
        let chain = ChainAtTheSpan {
            convexity: chain_convexity,
            free_at_the_start,
            free_at_the_end,
        };
        match surface_band_runs_off_the_face_beyond::try_read_a_stretch(
            Some(mesh),
            Some(region),
            &span,
            chain,
            beveled_mesh_edges,
            chain_vertices,
        ) {
            Ok(Some(stretch)) => {
                goes_over_these_rims = Some(stretch.rims_it_runs_off.clone());

                // The part of the span the band does NOT run off over: one interval.
                rest_of_the_span = cs_math::max_f64(length - stretch.until, 0.0);
                rest_of_the_span_from = if stretch.at_the_start {
                    from + (along * stretch.until)
                } else {
                    from
                };
                ran_off_over_a_stretch = Some(stretch);
            }
            Ok(None) => {}
            Err(why) => {
                if let Some(named) = named {
                    named.push(why.on_mesh_edge(mesh_edge_index));
                }
            }
        }
    }

    // What the mitre at either end takes out of the edge's own length, per unit of setback.
    let mut taken_at_from = 0.0;
    let mut taken_at_to = 0.0;

    for rim in region.boundary_edges() {
        if goes_over_these_rims
            .as_ref()
            .is_some_and(|rims| rims.contains(&rim.mesh_edge_index))
        {
            // The knife edge the band runs OFF over its stretch: a wall over the REST of the span.
            if rest_of_the_span > 0.0 {
                let over_the_rest = offset_to_segment(
                    rest_of_the_span_from,
                    along,
                    rest_of_the_span,
                    inward,
                    rim.start,
                    rim.end,
                );
                if over_the_rest < nearest {
                    nearest = over_the_rest;
                }
            }

            continue;
        }

        let leg_at_from = shares_the_corner(rim, from, corner);
        let leg_at_to = if leg_at_from.is_none() {
            shares_the_corner(rim, to, corner)
        } else {
            None
        };
        let (at_from, at_to) = (leg_at_from.is_some(), leg_at_to.is_some());

        if bands_on_this_region.contains(&rim.mesh_edge_index) {
            // The chain edge itself, and every other edge taking a band off THIS region.
            continue;
        }

        if (at_from || at_to) && beveled_mesh_edges.contains(&rim.mesh_edge_index) {
            // A corner the pass is beveling that lands on some OTHER region.
            continue;
        }

        if surface_room_standing_feature::stands_on(region, rim, beveled_mesh_edges, chain_vertices)
        {
            // Not a wall at all: a feature HANGING over the region.
            continue;
        }

        if let Some(leg) = leg_at_from {
            let offset = read_the_offset(
                region,
                rim,
                from + leg,
                corner,
                across_the_surface,
                beveled_mesh_edges,
                chain_vertices,
            );
            measure_corner(
                leg,
                along,
                inward,
                offset,
                corner,
                &mut nearest,
                &mut taken_at_from,
            );
        } else if let Some(leg) = leg_at_to {
            let offset = read_the_offset(
                region,
                rim,
                to + leg,
                corner,
                across_the_surface,
                beveled_mesh_edges,
                chain_vertices,
            );
            measure_corner(
                leg,
                -along,
                inward,
                offset,
                corner,
                &mut nearest,
                &mut taken_at_to,
            );
        } else if !across_the_surface {
            let offset = offset_to_segment(from, along, length, inward, rim.start, rim.end);
            if offset < nearest {
                nearest = offset;
            }
        }
    }

    let taken = taken_at_from + taken_at_to;
    if taken > 0.0 {
        nearest = cs_math::min_f64(nearest, length / taken);
    }

    (nearest, ran_off_over_a_stretch)
}
