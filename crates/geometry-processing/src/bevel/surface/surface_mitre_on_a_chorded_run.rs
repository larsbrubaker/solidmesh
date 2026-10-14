//! A mitre landing on a CHORDED run of a chain - several collinear pieces of one crease. The
//! neighbour walks, the collinearity test, the starvation test, and the point itself
//! ([`try_the_point`], [`where_the_neighbours_plane_crosses`]).
//!
//! **The corner is the answer, and only the corner, wherever the corner really divides the band.**
//! Two collinear pieces have ONE tangent line between them, so there is no crossing to read the
//! plane's point off and the corner stands in. **What that misses, measured on a raked bore's
//! mouth:** a band's stretch of its own tangent line is bounded by its neighbours' mitre points,
//! which at a shallow turn with stepping setbacks stand clear of the chord - split such a chord
//! and the corner lands past the pair's own stretch, leaving the second piece a window of nothing
//! (`SurfaceStripSeamDidNotMeet`). **So the plane divides the RUN's band as the corner divides the
//! run**, and only where the corner stands outside the stretch.

use vector_math::{cs_math, Vector3};

use super::{surface_mitre_seam, surface_plan_band_ends, SurfaceEdgePlan, SurfaceLine};

/// How far along a mitre plane a tangent line has to run to be read as crossing it, as a cosine.
pub const ALONG_ENOUGH_TO_CROSS: f64 = 1e-6;

/// Whether a piece of the run between `low` and `high` (along a run `whole` long, with the joint
/// at `at_the_joint` between `before_the_joint` and `after_the_joint`) stands outside the run and
/// so starves one of the two pieces either side of the joint of any length at all.
#[allow(clippy::too_many_arguments, clippy::neg_cmp_op_on_partial_ord)] // `!(x < y)`: a NaN passes.
pub fn starves_a_piece_by_standing_outside_the_run(
    low: f64,
    high: f64,
    before_the_joint: f64,
    at_the_joint: f64,
    after_the_joint: f64,
    whole: f64,
    near_enough: f64,
) -> bool {
    if !(low < -near_enough) && !(high > whole + near_enough) {
        return false;
    }

    // These min / max pass a NaN through rather than picking the other side.
    cs_math::min_f64(at_the_joint, high) <= cs_math::max_f64(before_the_joint, low)
        || cs_math::min_f64(after_the_joint, high) <= cs_math::max_f64(at_the_joint, low)
}

/// Whether edges `arrived_on` and `goes_on` are pieces of one straight crease: their tangent
/// lines are one line on a face they share.
pub fn are_collinear(
    edges: &[SurfaceEdgePlan],
    arrived_on: usize,
    goes_on: usize,
    near_enough: f64,
) -> bool {
    surface_mitre_seam::the_lines_are_one(&edges[arrived_on], &edges[goes_on], near_enough)
}

/// The edge after `edge`, wrapping round a closed chain of more than one edge; `None` past the
/// end of an open one.
pub fn try_the_next(edges: &[SurfaceEdgePlan], is_closed: bool, edge: usize) -> Option<usize> {
    let next = edge + 1;

    if next < edges.len() {
        return Some(next);
    }

    (is_closed && edges.len() > 1).then_some(0)
}

/// The edge before `edge`, wrapping round a closed chain of more than one edge; `None` before
/// the start of an open one.
pub fn try_the_previous(edges: &[SurfaceEdgePlan], is_closed: bool, edge: usize) -> Option<usize> {
    if edge >= 1 {
        return Some(edge - 1);
    }

    (is_closed && edges.len() > 1).then(|| edges.len() - 1)
}

/// The point the mitre plane between `leaving` (the edge arriving at the joint) and the edge after
/// it passes through, where the two are collinear AND the corner between them stands off the pair's
/// own band - `None` everywhere else, which leaves every other corner reading exactly as it did.
/// `near_enough` is the PART's ruler for two tangent lines being one line.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)` on purpose: it refuses a NaN as well.
pub fn try_the_point(
    edges: &[SurfaceEdgePlan],
    is_closed: bool,
    leaving: usize,
    near_enough: f64,
) -> Option<Vector3> {
    let arriving = try_the_next(edges, is_closed, leaving)?;

    if !are_collinear(edges, leaving, arriving, near_enough) {
        return None;
    }

    // The whole run of collinear pieces the joint is inside, which is one chord as far as its two
    // ends go. Bounded by the edge count so a chain whose every edge is collinear walks round once
    // and stops.
    let mut first = leaving;
    let mut last = arriving;

    for _ in 0..edges.len() {
        match try_the_previous(edges, is_closed, first) {
            Some(before) if before != last && are_collinear(edges, before, first, near_enough) => {
                first = before;
            }
            _ => break,
        }
    }

    for _ in 0..edges.len() {
        match try_the_next(edges, is_closed, last) {
            Some(after) if after != first && are_collinear(edges, last, after, near_enough) => {
                last = after;
            }
            _ => break,
        }
    }

    let from = edges[first].from();
    let direction = edges[first].edge.direction();

    // The stretch the run's band really covers, read on EACH of its two tangent lines - kept side
    // by side, because whether a piece is starved is a question about one side, and taken together
    // as well, since the one point the plane is put through has to stand inside both stretches. The
    // two sides do not agree on their own: a neighbour's mitre plane leans by half the turn.
    let mut lowest = f64::NEG_INFINITY;
    let mut highest = f64::INFINITY;
    let (mut left_low, mut left_high, mut right_low, mut right_high) = (0.0, 0.0, 0.0, 0.0);

    for left in [true, false] {
        let line = if left {
            edges[first].left_tangent_line
        } else {
            edges[first].right_tangent_line
        };

        let starts = where_the_neighbours_plane_crosses(
            edges, is_closed, first, true, from, direction, line,
        );
        let finishes = where_the_neighbours_plane_crosses(
            edges, is_closed, last, false, from, direction, line,
        );
        let low = cs_math::min_f64(starts, finishes);
        let high = cs_math::max_f64(starts, finishes);

        if left {
            (left_low, left_high) = (low, high);
        } else {
            (right_low, right_high) = (low, high);
        }

        lowest = cs_math::max_f64(lowest, low);
        highest = cs_math::min_f64(highest, high);
    }

    // How far along the run the joint's own corner stands, and how long the whole run is - the two
    // lengths the band is to be divided in the ratio of.
    let at_the_joint = (edges[arriving].from() - from).dot(direction);
    let whole = (edges[last].to() - from).dot(direction);

    if !(whole > 0.0) || at_the_joint <= 0.0 || at_the_joint >= whole || !(highest > lowest) {
        return None;
    }

    // Where the joint's own two pieces stand along the run, read from the edges themselves.
    let before_the_joint = (edges[leaving].from() - from).dot(direction);
    let after_the_joint = (edges[arriving].to() - from).dot(direction);

    if !starves_a_piece_by_standing_outside_the_run(
        left_low,
        left_high,
        before_the_joint,
        at_the_joint,
        after_the_joint,
        whole,
        near_enough,
    ) && !starves_a_piece_by_standing_outside_the_run(
        right_low,
        right_high,
        before_the_joint,
        at_the_joint,
        after_the_joint,
        whole,
        near_enough,
    ) {
        return None;
    }

    Some(from + (direction * (lowest + ((highest - lowest) * (at_the_joint / whole)))))
}

/// Where the plane bounding one end of the run (at the start of `edge` when `at_the_start`, else at
/// its end) crosses one of its tangent lines, as a parameter along `direction` from `from` - the
/// run's own corner where there is no neighbour to be bounded by, or where that neighbour's plane
/// runs along the line.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > tolerance)` on purpose: a NaN passes.
pub fn where_the_neighbours_plane_crosses(
    edges: &[SurfaceEdgePlan],
    is_closed: bool,
    edge: usize,
    at_the_start: bool,
    from: Vector3,
    direction: Vector3,
    line: SurfaceLine,
) -> f64 {
    let corner = (if at_the_start {
        edges[edge].from()
    } else {
        edges[edge].to()
    } - from)
        .dot(direction);

    let found = if at_the_start {
        try_the_previous(edges, is_closed, edge)
    } else {
        try_the_next(edges, is_closed, edge)
    };

    let Some(neighbour) = found else {
        return corner;
    };

    let (arrived_on, goes_on) = if at_the_start {
        (neighbour, edge)
    } else {
        (edge, neighbour)
    };

    let normal = surface_plan_band_ends::mitre_normal(
        edges[arrived_on].edge.direction(),
        edges[goes_on].edge.direction(),
        edges[goes_on].edge.direction(),
    );

    let point = surface_mitre_seam::the_point_or(
        &edges[arrived_on],
        &edges[goes_on],
        surface_plan_band_ends::crosses_within(&edges[arrived_on], &edges[goes_on]),
        edges[goes_on].from(),
    );

    let slope = line.direction.dot(normal);

    if !(slope.abs() > ALONG_ENOUGH_TO_CROSS) {
        return corner;
    }

    let crossing = line.point + (line.direction * ((point - line.point).dot(normal) / slope));

    (crossing - from).dot(direction)
}
