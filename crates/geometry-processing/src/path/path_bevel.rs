//! Rounds (bevels) the corners of a 2D path, sharing edge space fairly between corners that
//! compete for it. Ports `Path/PathBevel.cs` (`PathBevelSizeMode`, `PathBevel`,
//! `PathBevel.Contour`).
//!
//! Ported in C# from NodeDesigner's bevel-path-utils.js: candidates chosen by a selection
//! predicate and a bend threshold, per-corner radii settled by damped relaxation, then one
//! inscribed tangent arc per surviving corner. Open contours are handled (a polyline's ends are
//! never rounded), and the relaxation solves a whole pass at a time so the radii depend on the
//! shape and not on where its vertex numbering starts (see [`relax_radii`]).
//!
//! The size is read either as a radius (a fillet) or as a setback along each edge (a chamfer,
//! [`PathBevelSizeMode::TangentDistance`]); a setback `d` is turned into the radius
//! `r = d * tan(theta / 2)` before anything else runs.
//!
//! C#'s `Func<int, int, Vector2, bool> vertexSelection` (null selects all) is
//! `Option<&dyn Fn(usize, usize, Vector2) -> bool>`: contour ordinal, index within that contour,
//! position. A null `source`/`contour` is not representable here, so those C# guards go.

// C#'s "the long way round": `!(x > 0)` is true for NaN where `x <= 0` is not, and that is
// how a NaN size, radius or angle declines a corner instead of slipping through.
#![allow(clippy::neg_cmp_op_on_partial_ord)]

use agg_path::{FlagsAndCommand, FlattenCurves, VertexSource, VertexStorage};
use vector_math::{cs_math, math_helper, Vector2};

use super::path_bevel_arcs;

/// What the size handed to a bevel measures (C# `PathBevelSizeMode`).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub enum PathBevelSizeMode {
    /// The radius of the arc inscribed in each corner: every corner comes out the same
    /// roundness, and how far back it reaches depends on how sharp it is.
    #[default]
    Radius,
    /// The setback: how far back along each of its edges the corner is cut, which is what a CAD
    /// chamfer is dimensioned by.
    TangentDistance,
}

/// A vertex selection: contour ordinal, index within that contour and position (C#
/// `Func<int, int, Vector2, bool>`).
pub type VertexSelection<'a> = Option<&'a dyn Fn(usize, usize, Vector2) -> bool>;

/// One contour of a path, plus whether it closed back on itself (C# `PathBevel.Contour`).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Contour {
    /// The points of the contour, in the order the path draws them.
    pub points: Vec<Vector2>,
    /// Whether the contour closed back on itself. An open contour has two ends, which have one
    /// edge each and so no corner for an arc to sit in.
    pub is_closed: bool,
}

/// The relaxation is a fixed-point solve, so it wants only enough iterations to settle.
const MAX_ITERATIONS: usize = 50;

const CONVERGENCE_THRESHOLD: f64 = 0.0001;

/// Blend half the new value with half the old, so two corners on one short edge slide together
/// onto their shares rather than oscillating.
const RELAXATION_FACTOR: f64 = 0.5;

/// Shorter than this and an edge has no direction, so the corners at its ends have no angle.
pub const MINIMUM_EDGE_LENGTH: f64 = 1e-10;

/// An interior angle below this is a cusp, with no arc inscribed in it.
pub const MINIMUM_INTERIOR_ANGLE: f64 = 1e-6;

/// How finely a curved source is flattened before it is beveled: 1000, as the rest of the path
/// pipeline uses, rather than agg's default of 1 (half a millimetre of chord error).
const FLATTENING_RESOLUTION_SCALE: f64 = 1000.0;

/// Bevels the corners of every contour in the source (C# `PathBevel.Bevel`).
///
/// `size` is the arc radius in Radius mode and the setback in TangentDistance mode; corners get
/// less where there is not room. `segments` is line segments per corner (one gives the chamfer
/// chord).
pub fn bevel(
    source: &dyn VertexSource,
    size: f64,
    segments: i32,
    bend_threshold_degrees: f64,
    vertex_selection: VertexSelection<'_>,
    size_mode: PathBevelSizeMode,
) -> VertexStorage {
    let mut output = VertexStorage::new();

    let contours = extract_contours(source);

    for (contour_index, contour) in contours.iter().enumerate() {
        // a zero size still comes through here and is rebuilt as it was (flattened)
        let beveled = bevel_contour(
            &contour.points,
            contour.is_closed,
            size,
            segments,
            bend_threshold_degrees,
            vertex_selection,
            size_mode,
            contour_index,
        );

        if beveled.is_empty() {
            continue;
        }

        output.move_to_vector(beveled[0]);

        for point in &beveled[1..] {
            output.line_to_vector(*point);
        }

        if contour.is_closed {
            output.close_polygon();
        }
    }

    output
}

/// Bevels one contour (C# `PathBevel.BevelContour`). `closed` false is a polyline, whose two
/// ends are never rounded. `contour_index` is only passed on to the selection.
#[allow(clippy::too_many_arguments)]
pub fn bevel_contour(
    contour: &[Vector2],
    closed: bool,
    size: f64,
    segments: i32,
    bend_threshold_degrees: f64,
    vertex_selection: VertexSelection<'_>,
    size_mode: PathBevelSizeMode,
    contour_index: usize,
) -> Vec<Vector2> {
    // the long way round so that a NaN size passes the path through rather than filling it
    // with NaN
    if !(size > 0.0) {
        return contour.to_vec();
    }

    // one segment is the chord between the tangent points; zero would divide by zero
    let arc_segments = segments.max(1) as usize;

    let working = clean_contour(contour, closed);
    let vertex_count = working.len();

    if vertex_count < 3 {
        return contour.to_vec();
    }

    let is_candidate = identify_bevel_candidates(
        &working,
        closed,
        bend_threshold_degrees,
        vertex_selection,
        contour_index,
    );

    // how much path each corner has on either side of it, and who it is competing with for it
    let mut prev_candidate = vec![-1i64; vertex_count];
    let mut next_candidate = vec![-1i64; vertex_count];
    let mut run_to_prev = vec![0.0; vertex_count];
    let mut run_to_next = vec![0.0; vertex_count];

    for i in 0..vertex_count {
        if !is_candidate[i] {
            continue;
        }

        let (prev, prev_run) =
            path_bevel_arcs::find_neighbor_candidate(&working, closed, &is_candidate, i, -1);
        prev_candidate[i] = prev;
        run_to_prev[i] = prev_run;
        let (next, next_run) =
            path_bevel_arcs::find_neighbor_candidate(&working, closed, &is_candidate, i, 1);
        next_candidate[i] = next;
        run_to_next[i] = next_run;
    }

    // tan(interiorAngle / 2) converts a radius into the length of path the arc eats.
    // Non-candidates keep 1, which is never read but keeps the divisor safe.
    let corner_tan_half_angle: Vec<f64> = (0..vertex_count)
        .map(|i| {
            if is_candidate[i] {
                (interior_angle(&working, i, closed) / 2.0).tan()
            } else {
                1.0
            }
        })
        .collect();

    // a setback becomes the radius that produces it: r = d * tan(angle / 2)
    let mut requested_radius = vec![0.0; vertex_count];
    for i in 0..vertex_count {
        if !is_candidate[i] {
            continue;
        }

        requested_radius[i] = if size_mode == PathBevelSizeMode::TangentDistance {
            size * corner_tan_half_angle[i]
        } else {
            size
        };
    }

    let effective_radius = relax_radii(
        &is_candidate,
        &run_to_prev,
        &run_to_next,
        &prev_candidate,
        &next_candidate,
        &corner_tan_half_angle,
        &requested_radius,
    );

    // the arcs are built first, because each reports which vertices it reached past
    let mut eaten = vec![false; vertex_count];
    let mut arcs: Vec<Option<Vec<Vector2>>> = vec![None; vertex_count];

    for i in 0..vertex_count {
        // the long way round, so a NaN radius declines the corner
        if !is_candidate[i] || !(effective_radius[i] >= 1e-6) {
            continue;
        }

        // each corner's share of a run is in proportion to the two tangent distances, so the
        // shares never overlap even where the run is curved
        let tangent_dist = path_bevel_arcs::tangent_distance_of(
            &effective_radius,
            &corner_tan_half_angle,
            i as i64,
        );

        let allowance_to_prev = share_of_run(
            run_to_prev[i],
            tangent_dist,
            path_bevel_arcs::tangent_distance_of(
                &effective_radius,
                &corner_tan_half_angle,
                prev_candidate[i],
            ),
        );

        let allowance_to_next = share_of_run(
            run_to_next[i],
            tangent_dist,
            path_bevel_arcs::tangent_distance_of(
                &effective_radius,
                &corner_tan_half_angle,
                next_candidate[i],
            ),
        );

        arcs[i] = Some(path_bevel_arcs::generate_bevel_arc(
            &working,
            closed,
            &is_candidate,
            i,
            effective_radius[i],
            arc_segments,
            allowance_to_prev,
            allowance_to_next,
            &mut eaten,
        ));
    }

    let mut output = Vec::new();

    for i in 0..vertex_count {
        if let Some(arc) = &arcs[i] {
            output.extend_from_slice(arc);
            continue;
        }

        if !eaten[i] {
            output.push(working[i]);
        }
    }

    output
}

/// How much of a run belongs to the corner at one end of it, given what both ends settled for.
/// The two shares add up to the whole run.
fn share_of_run(run_length: f64, tangent_dist: f64, neighbor_tangent_dist: f64) -> f64 {
    let both = tangent_dist + neighbor_tangent_dist;

    // the long way round: NaN tangent distances leave the whole run
    if both > 0.0 {
        run_length * (tangent_dist / both)
    } else {
        run_length
    }
}

/// The contours of a path exactly as a bevel sees them: flattened, with the vertices that carry
/// no geometry taken out (C# `PathBevel.CleanedContours`). One entry per contour, including
/// those too short to have a corner, because the position in this list is the contour ordinal
/// the selection is told.
pub fn cleaned_contours(source: &dyn VertexSource) -> Vec<Contour> {
    extract_contours(source)
        .into_iter()
        .map(|contour| Contour {
            points: clean_contour(&contour.points, contour.is_closed),
            is_closed: contour.is_closed,
        })
        .collect()
}

/// The contour without zero length edges and, on a closed contour only, without a repeated
/// first point at the end. An open contour that ends on its first point keeps it: only a close
/// vertex closes a contour.
fn clean_contour(contour: &[Vector2], closed: bool) -> Vec<Vector2> {
    let mut cleaned: Vec<Vector2> = Vec::with_capacity(contour.len());

    for point in contour {
        if let Some(last) = cleaned.last() {
            if (*point - *last).length() < MINIMUM_EDGE_LENGTH {
                continue;
            }
        }

        cleaned.push(*point);
    }

    if closed {
        // SVG and font paths commonly repeat the first point to close
        while cleaned.len() >= 2
            && (cleaned[cleaned.len() - 1] - cleaned[0]).length() < MINIMUM_EDGE_LENGTH
        {
            cleaned.pop();
        }
    }

    cleaned
}

/// Settles a radius for every candidate by repeatedly asking each corner what fits given what
/// its neighbours currently take (C# `PathBevel.RelaxRadii`).
///
/// Every corner is solved against the previous pass's radii and the whole array replaced at the
/// end of the pass, so equal demands stay equal and the answer is a function of the shape alone
/// (the JavaScript updated in place and depended on vertex numbering).
fn relax_radii(
    is_candidate: &[bool],
    run_to_prev: &[f64],
    run_to_next: &[f64],
    prev_candidate: &[i64],
    next_candidate: &[i64],
    corner_tan_half_angle: &[f64],
    requested_radius: &[f64],
) -> Vec<f64> {
    let vertex_count = is_candidate.len();
    let mut effective_radius = vec![0.0; vertex_count];

    // start optimistic: everyone gets what they asked for, then gives ground
    for i in 0..vertex_count {
        if is_candidate[i] {
            effective_radius[i] = requested_radius[i];
        }
    }

    let mut next_radius = vec![0.0; vertex_count];

    for _ in 0..MAX_ITERATIONS {
        let mut total_change = 0.0;

        for i in 0..vertex_count {
            if !is_candidate[i] {
                next_radius[i] = effective_radius[i];
                continue;
            }

            let tan_half = measurable_or_none(corner_tan_half_angle[i]);

            let room_to_prev = measurable_or_none(run_to_prev[i]);
            let room_to_next = measurable_or_none(run_to_next[i]);

            let desired = measurable_or_none(requested_radius[i])
                .min((room_to_prev * tan_half).min(room_to_next * tan_half));

            let neighbor_prev_tangent_dist =
                measurable_or_none(path_bevel_arcs::tangent_distance_of(
                    &effective_radius,
                    corner_tan_half_angle,
                    prev_candidate[i],
                ));

            let neighbor_next_tangent_dist =
                measurable_or_none(path_bevel_arcs::tangent_distance_of(
                    &effective_radius,
                    corner_tan_half_angle,
                    next_candidate[i],
                ));

            let max_radius_from_prev_space = (room_to_prev - neighbor_prev_tangent_dist) * tan_half;
            let max_radius_from_next_space = (room_to_next - neighbor_next_tangent_dist) * tan_half;

            // one arc, so it has to fit on both of its edges
            let target_radius = cs_math::max_f64(
                0.0,
                cs_math::min_f64(
                    desired,
                    cs_math::min_f64(max_radius_from_prev_space, max_radius_from_next_space),
                ),
            );

            let new_radius = measurable_or_none(
                effective_radius[i] * (1.0 - RELAXATION_FACTOR) + target_radius * RELAXATION_FACTOR,
            );

            total_change += (new_radius - effective_radius[i]).abs();
            next_radius[i] = new_radius;
        }

        std::mem::swap(&mut effective_radius, &mut next_radius);

        if total_change < CONVERGENCE_THRESHOLD {
            break;
        }
    }

    effective_radius
}

/// The value where it is a number the relaxation can work with, and zero where it is not, so
/// one bad number costs only the corners it touches.
fn measurable_or_none(value: f64) -> f64 {
    if value.is_finite() {
        value
    } else {
        0.0
    }
}

/// Flags the vertices that pass the selection and bend enough to be worth rounding (C#
/// `PathBevel.IdentifyBevelCandidates`, shared with [`crate::topology::PathCornerSet`]).
pub(crate) fn identify_bevel_candidates(
    working: &[Vector2],
    closed: bool,
    bend_threshold_degrees: f64,
    vertex_selection: VertexSelection<'_>,
    contour_index: usize,
) -> Vec<bool> {
    let bend_threshold_radians = math_helper::degrees_to_radians(bend_threshold_degrees);
    let vertex_count = working.len();
    let mut candidates = vec![false; vertex_count];

    for i in 0..vertex_count {
        if !closed && (i == 0 || i == vertex_count - 1) {
            // an end of a polyline has one edge, and no corner to inscribe an arc in
            continue;
        }

        if let Some(selection) = vertex_selection {
            if !selection(contour_index, i, working[i]) {
                continue;
            }
        }

        let interior = interior_angle(working, i, closed);

        if interior < MINIMUM_INTERIOR_ANGLE {
            // a cusp
            continue;
        }

        // how far the path turns at this vertex: 0 running straight on, PI at a cusp
        let sweep_angle = std::f64::consts::PI - interior;

        candidates[i] = sweep_angle > bend_threshold_radians;
    }

    candidates
}

/// The angle at vertex `i` between the edge back to its previous vertex and the edge on to its
/// next one.
pub(crate) fn interior_angle(working: &[Vector2], i: usize, closed: bool) -> f64 {
    let vertex_count = working.len();

    let to_prev = normalize(working[prev_index(i, vertex_count, closed) as usize] - working[i]);
    let to_next = normalize(working[next_index(i, vertex_count, closed) as usize] - working[i]);

    cs_clamp(to_prev.dot(to_next), -1.0, 1.0).acos()
}

/// C# `Math.Clamp`, which passes NaN through.
fn cs_clamp(value: f64, min: f64, max: f64) -> f64 {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}

pub(crate) fn normalize(vector: Vector2) -> Vector2 {
    let length = vector.length();

    if length < 1e-10 {
        return Vector2::ZERO;
    }

    vector / length
}

/// The previous index; -1 off the start of an open contour.
pub(crate) fn prev_index(i: usize, vertex_count: usize, closed: bool) -> i64 {
    if closed {
        ((i + vertex_count - 1) % vertex_count) as i64
    } else {
        i as i64 - 1
    }
}

/// The next index; `vertex_count` off the end of an open contour.
pub(crate) fn next_index(i: usize, vertex_count: usize, closed: bool) -> i64 {
    if closed {
        ((i + 1) % vertex_count) as i64
    } else {
        i as i64 + 1
    }
}

/// Splits the path into contours, keeping for each whether it was closed. Flattened at
/// [`FLATTENING_RESOLUTION_SCALE`], which decides what a corner is as much as smoothness: the
/// bends of a finely flattened curve fall below any reasonable bend threshold.
fn extract_contours(source: &dyn VertexSource) -> Vec<Contour> {
    let mut contours: Vec<Contour> = Vec::new();

    let mut flattened = FlattenCurves::new(Box::new(VertexStorage::from_source(source)));
    flattened.set_resolution_scale(FLATTENING_RESOLUTION_SCALE);

    // the index of the contour being added to, if any
    let mut current: Option<usize> = None;

    for vertex_data in flattened.vertices() {
        if vertex_data.command == FlagsAndCommand::MOVE_TO || vertex_data.is_line_to() {
            if vertex_data.command == FlagsAndCommand::MOVE_TO {
                // a MoveTo always starts a new contour
                current = None;
            }

            let index = match current {
                Some(index) => index,
                None => {
                    contours.push(Contour::default());
                    contours.len() - 1
                }
            };
            current = Some(index);

            contours[index].points.push(vertex_data.position);
        } else if vertex_data.command != FlagsAndCommand::FLAG_NONE {
            // the close vertex is the only thing that says the contour was closed
            if vertex_data.is_close() {
                if let Some(index) = current {
                    contours[index].is_closed = true;
                }
            }

            current = None;
        }
    }

    contours
}
