//! Which runs bound a partnered bead at each end (`Bevel/BevelPartneredBeadPartners.cs`, a file of
//! the C# `static partial class BevelPartneredBead`): every other concave run meeting this one at
//! an end, as the chorded tube its own bead is built from.

use vector_math::Vector3;

use super::bevel_partnered_bead::{
    try_end_on_the_across_plane, BevelPartneredBeadEnd, BevelPartneredBeadRun,
};
use super::bevel_partnered_bead_stations::spacing;
use super::bevel_partnered_bead_tolerances::{
    meeting_within, partner_overlap, reaches_the_face, RUN_OUT_LIMIT_IN_SIZES,
};
use super::bevel_run_partners::try_ball_centre_line;
use super::{BevelPartnerTube, BevelRunOutBound, BevelSweptChain};
use crate::topology::ChainEdge;

/// C# `TryReadPartners`: this end's partner tubes, read from every run in `concave_runs` (other
/// than this one) that meets it here - or the refusal that says the end cannot be bounded.
pub fn try_read_partners(
    chain: &BevelSweptChain,
    concave_runs: &[BevelSweptChain],
    run: &BevelPartneredBeadRun,
    end: &mut BevelPartneredBeadEnd,
) -> Result<(), String> {
    let size = run.size;
    end.partners = Vec::with_capacity(2);

    for other in concave_runs {
        if other.edges.is_empty() || is_the_same_run(other, chain) {
            continue;
        }

        if !meets_here(run, other, end) {
            continue;
        }

        let Some((axis_point, direction)) = try_ball_centre_line(other, size) else {
            return Err("a run meeting this one at its end turns, so it has no tube to be bounded by and this end has to be capped the way it is capped today".to_string());
        };

        let (from, to) = span_of(run, other, axis_point, direction);

        end.partners.push(BevelPartnerTube::around_the_chorded_arc(
            axis_point,
            direction,
            size - partner_overlap(size),
            other.edges[0].edge.normal0,
            other.edges[0].edge.normal1,
            run.arc_facets,
            from,
            to,
        ));
    }

    if end.partners.is_empty() {
        if end.has_wall {
            return Ok(());
        }

        if !end.has_across_plane || !try_end_on_the_across_plane(run, end) {
            return Err("no other run meets this one at this end, so there is nothing to bound the bead where its crease stops".to_string());
        }

        return Ok(());
    }

    if !end.has_wall
        && !rides_the_same_face(run, end)
        && (!end.has_across_plane || !try_end_on_the_across_plane(run, end))
    {
        return Err("the run meeting this one at this end rides a different face and no wall stops the bead there, so nothing bounds its run-out and the bead is the one the sweep has always built".to_string());
    }

    Ok(())
}

/// C# `RidesTheSameFace`: whether any partner's tube runs along the riding face, one size off it.
pub fn rides_the_same_face(run: &BevelPartneredBeadRun, end: &BevelPartneredBeadEnd) -> bool {
    let slack = reaches_the_face(run);
    let on_the_face = run.edges[0].from();

    end.partners.iter().any(|partner| {
        partner.axis_direction.dot(run.riding_normal).abs() <= slack / run.size
            && ((partner.axis_point - on_the_face).dot(run.riding_normal) - run.size).abs() <= slack
    })
}

/// C# `SpanOf`: how far along its own axis a partner's tube reaches - its crease plus each end's
/// run-out - as `(from, to)`.
pub fn span_of(
    run: &BevelPartneredBeadRun,
    other: &BevelSweptChain,
    axis_point: Vector3,
    direction: Vector3,
) -> (f64, f64) {
    let along = |point: Vector3| (point - axis_point).dot(direction);

    let spacing = spacing(run);
    let first = &other.edges[0];
    let last = &other.edges[other.edges.len() - 1];

    let from = along(first.from())
        - reach_of(
            run,
            first,
            first.from(),
            -direction,
            axis_point,
            direction,
            spacing,
        );
    let to = along(last.to())
        + reach_of(
            run,
            last,
            last.to(),
            direction,
            axis_point,
            direction,
            spacing,
        );

    (from, to)
}

/// C# `ReachOf`: how far past `vertex` a partner's own bead runs out, read the way this run's
/// ends are - one spacing more against a wall.
#[allow(clippy::too_many_arguments)]
pub fn reach_of(
    run: &BevelPartneredBeadRun,
    edge: &ChainEdge,
    vertex: Vector3,
    outward: Vector3,
    axis_point: Vector3,
    direction: Vector3,
    spacing: f64,
) -> f64 {
    let ball_centre = axis_point + (direction * (vertex - axis_point).dot(direction));
    let limit = RUN_OUT_LIMIT_IN_SIZES * run.size;

    let left = run.run_out.try_measure(
        edge.left_face(),
        ball_centre - (edge.left_normal() * run.size),
        outward,
        limit,
    );
    let right = run.run_out.try_measure(
        edge.right_face(),
        ball_centre - (edge.right_normal() * run.size),
        outward,
        limit,
    );

    if left.is_none() && right.is_none() {
        return 0.0;
    }

    let rides_left = match (left, right) {
        (Some(l), Some(r)) => l.distance >= r.distance,
        (Some(_), None) => true,
        _ => false,
    };
    let riding = if rides_left { left } else { right }.unwrap_or_default();

    if riding.bound == BevelRunOutBound::ConcaveEdge {
        riding.distance + spacing
    } else {
        riding.distance
    }
}

/// C# `MeetsHere`: whether `other` starts or ends at this end - by vertex index, or within the
/// part's own meeting tolerance of its position.
pub fn meets_here(
    run: &BevelPartneredBeadRun,
    other: &BevelSweptChain,
    end: &BevelPartneredBeadEnd,
) -> bool {
    let first = &other.edges[0];
    let last = &other.edges[other.edges.len() - 1];

    first.from_vertex() == end.vertex_index
        || last.to_vertex() == end.vertex_index
        || (first.from() - end.vertex).length() < meeting_within(run)
        || (last.to() - end.vertex).length() < meeting_within(run)
}

/// C# `IsTheSameRun`: the same mesh edges in the same order.
pub fn is_the_same_run(one: &BevelSweptChain, other: &BevelSweptChain) -> bool {
    one.edges.len() == other.edges.len()
        && one
            .edges
            .iter()
            .zip(&other.edges)
            .all(|(a, b)| a.edge.mesh_edge_index == b.edge.mesh_edge_index)
}
