//! A mitre landing on a CHORDED run of a chain - several collinear pieces of one crease
//! (`Bevel/Surface/SurfaceMitreOnAChordedRun.cs`, a static class: free functions here). The
//! neighbour walks, the collinearity test and the starvation test land here first; reading the
//! point itself (`TryThePoint`, `WhereTheNeighboursPlaneCrosses`) lands with a later package.

use super::{surface_mitre_seam, SurfaceEdgePlan};

/// How far along a mitre plane a tangent line has to run to be read as crossing it, as a cosine.
pub const ALONG_ENOUGH_TO_CROSS: f64 = 1e-6;

/// Whether a piece of the run between `low` and `high` (along a run `whole` long, with the joint
/// at `at_the_joint` between `before_the_joint` and `after_the_joint`) stands outside the run and
/// so starves one of the two pieces either side of the joint of any length at all.
#[allow(clippy::too_many_arguments, clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x < y)`.
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

    // C#'s Math.Min / Math.Max, which pass a NaN through.
    cs_min(at_the_joint, high) <= cs_max(before_the_joint, low)
        || cs_min(after_the_joint, high) <= cs_max(at_the_joint, low)
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

fn cs_min(a: f64, b: f64) -> f64 {
    if a.is_nan() || b.is_nan() {
        f64::NAN
    } else {
        a.min(b)
    }
}

fn cs_max(a: f64, b: f64) -> f64 {
    if a.is_nan() || b.is_nan() {
        f64::NAN
    } else {
        a.max(b)
    }
}
