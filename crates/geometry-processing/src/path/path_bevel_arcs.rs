//! The arc-generation half of the path bevel: how much room a corner has along the path, and the
//! arc inscribed in it once the relaxation has settled how much of that room it may take.
//!
//! The fillet center is searched for along the offset of the forward run (its segments pushed
//! over by the radius, joined by arcs of the radius about its vertices) for the place where the
//! distance back to the other run is the radius too. Where both runs are straight that is the
//! textbook bisector point; where a run is curved the center rides the curve's own offset, so the
//! arc lands tangent to the curve. If no center of the requested radius fits in what the corner
//! was given, the radius is halved down to the largest that does.

// The long way round: `!(x > 0)` is true for NaN where `x <= 0` is not, and that is
// how a NaN size, radius or angle declines a corner instead of slipping through.
#![allow(clippy::neg_cmp_op_on_partial_ord)]

use std::cell::Cell;
use std::f64::consts::PI;

use vector_math::{cs_math, Vector2};

use super::path_bevel::{interior_angle, next_index, prev_index};

/// How far a corner must bend before an arc can be built in it at all, in radians: below this
/// the fillet center runs off to infinity along the bisector.
const MINIMUM_ARC_SWEEP_ANGLE: f64 = 1e-6;

/// Halvings of the search for the center along the forward run's offset.
const CENTER_SEARCH_ITERATIONS: usize = 60;

/// Halvings of the search for a radius that fits, once the requested one has not.
const FIT_SEARCH_ITERATIONS: usize = 20;

/// How close to the largest fitting radius the fit search stops early, as a fraction of the
/// radius asked for.
const FIT_SEARCH_PRECISION: f64 = 1e-6;

/// How far past its share of a run a tangent point may land before the radius is a bad fit.
const ALLOWANCE_SLACK: f64 = 1e-9;

/// How far the wrong way round the center a sweep may read before it is taken as really going
/// that way rather than as a zero sweep rounding pushed off its sign.
const SWEEP_DIRECTION_TOLERANCE: f64 = 1e-9;

thread_local! {
    /// Per thread, how many places on a run the
    /// construction has measured against since the last reset, so a test can read the growth of
    /// the work instead of the time it took.
    static RUN_PLACES_MEASURED: Cell<u64> = const { Cell::new(0) };
}

fn count_run_place() {
    RUN_PLACES_MEASURED.with(|count| count.set(count.get() + 1));
}

/// How many run places this thread's constructions have measured since the last reset.
pub fn run_places_measured() -> u64 {
    RUN_PLACES_MEASURED.with(Cell::get)
}

/// Zeroes this thread's [`run_places_measured`].
pub fn reset_run_places_measured() {
    RUN_PLACES_MEASURED.with(|count| count.set(0));
}

/// The points of the arc that rounds the corner at `vertex`, marking the vertices it reaches past
/// in `eaten`. A single point back at the vertex when no arc can be built.
#[allow(clippy::too_many_arguments)]
pub(crate) fn generate_bevel_arc(
    working: &[Vector2],
    closed: bool,
    is_candidate: &[bool],
    vertex: usize,
    radius: f64,
    segments: usize,
    allowance_to_prev: f64,
    allowance_to_next: f64,
    eaten: &mut [bool],
) -> Vec<Vector2> {
    let corner = working[vertex];
    let no_arc = vec![corner];

    let sweep_angle = PI - interior_angle(working, vertex, closed);

    // the long way round, so a NaN interior angle reads as a corner that cannot be measured
    if !(sweep_angle >= MINIMUM_ARC_SWEEP_ANGLE) {
        return no_arc;
    }

    if !(radius > 0.0) {
        return no_arc;
    }

    let mut run_to_prev = collect_run(working, closed, is_candidate, vertex, -1);
    let mut run_to_next = collect_run(working, closed, is_candidate, vertex, 1);

    if run_to_prev.segment_count() < 1 || run_to_next.segment_count() < 1 {
        return no_arc;
    }

    // which side of the corner the center falls on: the side the path turns towards
    let turn = cross(-run_to_prev.directions[0], run_to_next.directions[0]);

    if !(turn.abs() > 0.0) {
        return no_arc;
    }

    let interior_sign = if turn > 0.0 { 1.0 } else { -1.0 };
    run_to_next.fill_interior_normals(interior_sign);

    let fillet = match try_find_center(
        &mut run_to_prev,
        &mut run_to_next,
        radius,
        allowance_to_prev,
        allowance_to_next,
    )
    .or_else(|| {
        try_largest_radius_that_fits(
            &mut run_to_prev,
            &mut run_to_next,
            radius,
            allowance_to_prev,
            allowance_to_next,
        )
    }) {
        Some(fillet) => fillet,
        None => return no_arc,
    };

    eat(&run_to_prev, fillet.reach_to_prev, eaten);
    eat(&run_to_next, fillet.reach_to_next, eaten);

    sample_arc(&fillet, interior_sign, segments)
}

/// Where the fillet of one corner ended up. The interior sign is
/// passed to [`sample_arc`] alongside rather than stored.
#[derive(Debug, Clone, Copy, Default)]
struct Fillet {
    center: Vector2,
    touch_on_prev: Vector2,
    touch_on_next: Vector2,
    reach_to_prev: f64,
    reach_to_next: f64,
    radius: f64,
}

/// The center of the fillet of `radius` touching both runs, halved for along the forward run's
/// offset. `None` when no such center lies within what the corner was allowed.
fn try_find_center(
    run_to_prev: &mut PathRun,
    run_to_next: &mut PathRun,
    radius: f64,
    allowance_to_prev: f64,
    allowance_to_next: f64,
) -> Option<Fillet> {
    if !(radius > 0.0) {
        return None;
    }

    let reach_limit = cs_math::min_f64(
        run_to_next.length(),
        allowance_to_next + slack(allowance_to_next),
    );

    // the run behind is only this corner's as far as its share of it
    let prev_limit = cs_math::min_f64(
        run_to_prev.length(),
        allowance_to_prev + slack(allowance_to_prev),
    );

    let boundaries = run_to_next.offset_boundaries(radius, reach_limit);

    if boundaries.is_empty() {
        return None;
    }

    // at the corner the center is always nearer the run behind than a radius, unless the
    // geometry is too degenerate to measure
    if !(gap(run_to_prev, run_to_next, radius, 0.0, prev_limit) < 0.0) {
        return None;
    }

    let mut low = 0.0;
    let mut high = 0.0;
    let mut bracketed = false;
    let mut no_crossing_before = 0.0;

    // the first element whose far end is far enough from the run behind is the tangency
    // nearest the corner
    for boundary in boundaries {
        count_run_place();

        if boundary < no_crossing_before {
            // a gap of g rules out a crossing for the next g of parameter
            low = boundary;
            continue;
        }

        let measured = gap(run_to_prev, run_to_next, radius, boundary, prev_limit);

        if measured >= 0.0 {
            high = boundary;
            bracketed = true;
            break;
        }

        low = boundary;
        no_crossing_before = boundary - measured;
    }

    if !bracketed {
        return None;
    }

    for _ in 0..CENTER_SEARCH_ITERATIONS {
        let middle = (low + high) / 2.0;

        if gap(run_to_prev, run_to_next, radius, middle, prev_limit) < 0.0 {
            low = middle;
        } else {
            high = middle;
        }
    }

    let (center, touch_on_next, reach_to_next) =
        run_to_next.evaluate_offset(radius, (low + high) / 2.0);
    let (distance_to_prev, touch_on_prev, reach_to_prev) =
        run_to_prev.closest_point(center, prev_limit);

    if !(reach_to_prev < prev_limit) {
        // the tangency landed on the end of this corner's share of the run behind it
        return None;
    }

    if !((distance_to_prev - radius).abs() < 1e-6) {
        // the halving did not converge onto a tangency
        return None;
    }

    Some(Fillet {
        center,
        touch_on_prev,
        touch_on_next,
        reach_to_prev,
        reach_to_next,
        radius,
    })
}

/// The largest radius no bigger than `radius` whose fillet fits in what the corner was allowed,
/// found by halving. The negotiated radius is a ceiling: a corner may take less, never more.
fn try_largest_radius_that_fits(
    run_to_prev: &mut PathRun,
    run_to_next: &mut PathRun,
    radius: f64,
    allowance_to_prev: f64,
    allowance_to_next: f64,
) -> Option<Fillet> {
    if !(radius > 0.0) {
        return None;
    }

    let mut low = 0.0;
    let mut high = radius;
    let mut found = None;

    for _ in 0..FIT_SEARCH_ITERATIONS {
        if high - low < radius * FIT_SEARCH_PRECISION {
            break;
        }

        let middle = (low + high) / 2.0;

        match try_find_center(
            run_to_prev,
            run_to_next,
            middle,
            allowance_to_prev,
            allowance_to_next,
        ) {
            Some(smaller) => {
                found = Some(smaller);
                low = middle;
            }
            None => high = middle,
        }
    }

    found
}

/// How much further than the radius the center at `offset_parameter` along the forward run's
/// offset is from the run behind the corner.
fn gap(
    run_to_prev: &PathRun,
    run_to_next: &mut PathRun,
    radius: f64,
    offset_parameter: f64,
    prev_limit: f64,
) -> f64 {
    let (center, _, _) = run_to_next.evaluate_offset(radius, offset_parameter);
    run_to_prev.closest_point(center, prev_limit).0 - radius
}

fn slack(allowance: f64) -> f64 {
    (allowance.abs() * ALLOWANCE_SLACK) + ALLOWANCE_SLACK
}

/// Marks the vertices of `run` the arc reached past. Never the corner the run ends at.
fn eat(run: &PathRun, reach: f64, eaten: &mut [bool]) {
    for i in 1..run.points.len().saturating_sub(1) {
        if run.cumulative[i] <= reach + ALLOWANCE_SLACK {
            eaten[run.indices[i]] = true;
        }
    }
}

/// The fillet sampled into `segments` chords from its touch on the run behind to its touch on the
/// run ahead, going round the center the way the corner turns (not the short way round, which
/// fails exactly at the half-circle limit).
fn sample_arc(fillet: &Fillet, interior_sign: f64, segments: usize) -> Vec<Vector2> {
    let start_angle =
        (fillet.touch_on_prev.y - fillet.center.y).atan2(fillet.touch_on_prev.x - fillet.center.x);
    let end_angle =
        (fillet.touch_on_next.y - fillet.center.y).atan2(fillet.touch_on_next.x - fillet.center.x);
    let mut arc_sweep = end_angle - start_angle;

    // wrapped into the turn direction, leaving a zero sweep alone
    while arc_sweep * interior_sign < -SWEEP_DIRECTION_TOLERANCE {
        arc_sweep += 2.0 * PI * interior_sign;
    }

    while arc_sweep * interior_sign > 2.0 * PI {
        arc_sweep -= 2.0 * PI * interior_sign;
    }

    let mut arc_points = Vec::with_capacity(segments + 1);
    // placed rather than sampled, so the arc starts and ends exactly where the path does
    arc_points.push(fillet.touch_on_prev);

    for i in 1..segments {
        let current_angle = start_angle + (arc_sweep * (i as f64 / segments as f64));
        arc_points.push(Vector2::new(
            fillet.center.x + (fillet.radius * current_angle.cos()),
            fillet.center.y + (fillet.radius * current_angle.sin()),
        ));
    }

    arc_points.push(fillet.touch_on_next);
    arc_points
}

/// The stretch of path a corner has to itself in one direction, with the arc length to each
/// vertex along the way.
#[derive(Debug, Default)]
struct PathRun {
    indices: Vec<usize>,
    points: Vec<Vector2>,
    /// The arc length from the corner to each point.
    cumulative: Vec<f64>,
    /// The unit direction of each segment, pointing away from the corner.
    directions: Vec<Vector2>,
    /// The unit normal of each segment towards the fillet center; only for the run ahead.
    normals: Vec<Vector2>,
    /// The turn from each segment's normal to the next one's.
    vertex_turns: Vec<f64>,
    /// The offset parameter at the start of each element of the offset, with its end appended.
    element_starts: Vec<f64>,
    tabulated_radius: f64,
}

impl PathRun {
    fn segment_count(&self) -> usize {
        self.points.len().saturating_sub(1)
    }

    fn length(&self) -> f64 {
        self.cumulative[self.cumulative.len() - 1]
    }

    fn fill_interior_normals(&mut self, interior_sign: f64) {
        self.normals.clear();
        self.vertex_turns.clear();
        self.tabulated_radius = f64::NAN;

        for direction in &self.directions {
            // the left normal of the direction, flipped when the corner turns the other way
            self.normals
                .push(Vector2::new(-direction.y, direction.x) * interior_sign);
        }

        for segment in 0..self.segment_count().saturating_sub(1) {
            self.vertex_turns.push(signed_angle_between(
                self.normals[segment],
                self.normals[segment + 1],
            ));
        }
    }

    /// Lays out where each element of the offset of `radius` starts.
    fn tabulate_offset(&mut self, radius: f64) {
        if self.tabulated_radius == radius {
            return;
        }

        self.tabulated_radius = radius;
        self.element_starts.clear();
        let mut total = 0.0;

        for segment in 0..self.segment_count() {
            self.element_starts.push(total);
            total += self.cumulative[segment + 1] - self.cumulative[segment];

            if segment + 1 < self.segment_count() {
                self.element_starts.push(total);
                total += self.vertex_turns[segment].abs() * radius;
            }
        }

        self.element_starts.push(total);
    }

    /// The element of the offset that `parameter` falls in.
    fn element_at(&self, parameter: f64) -> usize {
        let mut low = 0;
        let mut high = self.element_starts.len() - 2;

        while low < high {
            let middle = (low + high).div_ceil(2);

            if self.element_starts[middle] <= parameter {
                low = middle;
            } else {
                high = middle - 1;
            }
        }

        low
    }

    /// The center of a fillet of `radius` sitting `offset_parameter` along this run's offset,
    /// the point of the run it touches, and how far along the run that point is.
    fn evaluate_offset(&mut self, radius: f64, offset_parameter: f64) -> (Vector2, Vector2, f64) {
        self.tabulate_offset(radius);

        let end = self.element_starts[self.element_starts.len() - 1];
        let parameter = cs_clamp(offset_parameter, 0.0, end);
        let element = self.element_at(parameter);
        let segment = element / 2;
        let into = parameter - self.element_starts[element];

        if element.is_multiple_of(2) {
            let along = into.min(self.cumulative[segment + 1] - self.cumulative[segment]);
            let touch = self.points[segment] + (self.directions[segment] * along);
            let center = touch + (self.normals[segment] * radius);
            return (center, touch, self.cumulative[segment] + along);
        }

        // rolling round the vertex between two segments: every center here touches that vertex
        let turn = self.vertex_turns[segment];
        let vertex_arc_length = turn.abs() * radius;
        let fraction = if vertex_arc_length > 0.0 {
            (into / vertex_arc_length).min(1.0)
        } else {
            0.0
        };
        let touch = self.points[segment + 1];
        let center = touch + (rotate(self.normals[segment], turn * fraction) * radius);
        (center, touch, self.cumulative[segment + 1])
    }

    /// The offset parameter at the end of each element of the offset, up to the one that touches
    /// `path_limit` along the run.
    fn offset_boundaries(&self, radius: f64, path_limit: f64) -> Vec<f64> {
        let mut boundaries = Vec::new();
        let mut total = 0.0;

        for segment in 0..self.segment_count() {
            count_run_place();

            if path_limit < self.cumulative[segment + 1] {
                boundaries.push(total + (path_limit - self.cumulative[segment]).max(0.0));
                return boundaries;
            }

            total += self.cumulative[segment + 1] - self.cumulative[segment];
            boundaries.push(total);

            if segment + 1 < self.segment_count() {
                total += self.vertex_turns[segment].abs() * radius;
                boundaries.push(total);
            }
        }

        boundaries
    }

    /// How far `from` is from the first `limit` of arc length of this run, with the nearest
    /// point and how far along the run it sits.
    fn closest_point(&self, from: Vector2, limit: f64) -> (f64, Vector2, f64) {
        let mut closest = self.points[0];
        let mut reach = 0.0;
        let mut best = f64::MAX;

        for segment in 0..self.segment_count() {
            if self.cumulative[segment] >= limit {
                break;
            }

            count_run_place();

            let segment_length = self.cumulative[segment + 1].min(limit) - self.cumulative[segment];
            let along = cs_clamp(
                (from - self.points[segment]).dot(self.directions[segment]),
                0.0,
                segment_length,
            );
            let on_segment = self.points[segment] + (self.directions[segment] * along);
            let distance = (from - on_segment).length();

            // a NaN distance loses to every real one
            if distance < best {
                best = distance;
                closest = on_segment;
                reach = self.cumulative[segment] + along;
            }
        }

        (best, closest, reach)
    }
}

/// The run from `from` in `direction` to the next candidate corner, or to the end of an open
/// contour.
fn collect_run(
    working: &[Vector2],
    closed: bool,
    is_candidate: &[bool],
    from: usize,
    direction: i32,
) -> PathRun {
    let vertex_count = working.len();
    let mut current = from;
    let mut run = PathRun {
        tabulated_radius: f64::NAN,
        ..PathRun::default()
    };
    run.indices.push(current);
    run.points.push(working[current]);
    run.cumulative.push(0.0);

    loop {
        let next = if direction > 0 {
            next_index(current, vertex_count, closed)
        } else {
            prev_index(current, vertex_count, closed)
        };

        if next < 0 || next >= vertex_count as i64 {
            return run;
        }

        let next = next as usize;
        let chord = working[next] - working[current];
        let chord_length = chord.length();

        // the long way round: a NaN vertex ends the run here
        if !(chord_length > 0.0) {
            return run;
        }

        let previous_total = run.cumulative[run.cumulative.len() - 1];
        run.indices.push(next);
        run.points.push(working[next]);
        run.cumulative.push(previous_total + chord_length);
        run.directions.push(chord / chord_length);
        current = next;

        if is_candidate[current] {
            return run;
        }
    }
}

/// Walks from `from` in `direction` to the next candidate corner, returning that corner (or -1
/// off the end of an open contour or at a vertex that cannot be measured) and the arc length
/// covered.
pub(crate) fn find_neighbor_candidate(
    working: &[Vector2],
    closed: bool,
    is_candidate: &[bool],
    from: usize,
    direction: i32,
) -> (i64, f64) {
    let vertex_count = working.len();
    let mut current = from;
    let mut run_length = 0.0;

    loop {
        let next = if direction > 0 {
            next_index(current, vertex_count, closed)
        } else {
            prev_index(current, vertex_count, closed)
        };

        if next < 0 || next >= vertex_count as i64 {
            // the end of an open contour: the rest of the path is this corner's
            return (-1, run_length);
        }

        let next = next as usize;
        let chord_length = (working[next] - working[current]).length();

        if !chord_length.is_finite() || !(run_length + chord_length).is_finite() {
            // a vertex that cannot be measured is a wall
            return (-1, run_length);
        }

        run_length += chord_length;
        current = next;

        if is_candidate[current] {
            return (current as i64, run_length);
        }
    }
}

/// How far back along the path the corner at `candidate` currently reaches; zero for -1.
pub(crate) fn tangent_distance_of(
    effective_radius: &[f64],
    corner_tan_half_angle: &[f64],
    candidate: i64,
) -> f64 {
    if candidate < 0 {
        0.0
    } else {
        effective_radius[candidate as usize] / corner_tan_half_angle[candidate as usize]
    }
}

fn cross(a: Vector2, b: Vector2) -> f64 {
    (a.x * b.y) - (a.y * b.x)
}

/// The turn from one unit vector to the other, in radians, between -PI and PI.
fn signed_angle_between(from: Vector2, to: Vector2) -> f64 {
    cross(from, to).atan2(from.dot(to))
}

fn rotate(vector: Vector2, angle: f64) -> Vector2 {
    let cos = angle.cos();
    let sin = angle.sin();
    Vector2::new(
        (vector.x * cos) - (vector.y * sin),
        (vector.x * sin) + (vector.y * cos),
    )
}

/// Clamps `value` into `[min, max]`, passing a NaN through (where `f64::clamp` would panic on a
/// NaN bound) so the NaN guards downstream still see it.
fn cs_clamp(value: f64, min: f64, max: f64) -> f64 {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}
