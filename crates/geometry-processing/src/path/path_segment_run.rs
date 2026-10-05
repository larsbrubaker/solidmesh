//! Grows a seed segment of a polyline contour into the run of neighbouring segments that are
//! curved but close. Ports `Path/PathSegmentRun.cs` (`RunStopReason`, `SegmentRun`,
//! `PathSegmentRun`).
//!
//! The 2D prototype of a mesh walk rule between `FeatureEdgeChain::propagate_collinear` and
//! `FeatureEdgeChain::propagate`: each END of the run gets its own TOTAL turning budget, spent on
//! the absolute bends it crosses (total turning does not depend on how finely a curve was
//! flattened), and a single vertex bending by the corner angle stops an end whatever budget is
//! left. The ends are grown turn and turn about, so a closed contour's last segments divide
//! evenly between them.
//!
//! C#'s `ArgumentNullException`/`ArgumentException`/`ArgumentOutOfRangeException` are
//! [`PathSegmentRunError`]; a null `points` is not representable.

use std::fmt;

use vector_math::{math_helper, Vector2};

/// Why a segment run stopped growing at one of its ends (C# `RunStopReason`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RunStopReason {
    /// The next vertex bends by at least the corner angle all on its own.
    Corner,
    /// Taking the next segment would push this end's turning past the budget.
    BudgetSpent,
    /// There is no next segment: the end of an open contour.
    End,
    /// The two ends met: the run is the whole closed contour.
    Loop,
}

/// A run of contour segments that are curved but close (C# `SegmentRun`): [`SegmentRun::count`]
/// of them walking forward from [`SegmentRun::first_segment`], wrapping on a closed contour.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SegmentRun {
    /// The first segment of the run. Segment i joins point i to point i + 1.
    pub first_segment: usize,
    /// How many segments are in the run.
    pub count: usize,
    /// How many segments the whole contour has: one per point closed, one fewer open.
    pub segment_count: usize,
    /// The total absolute turning of the run in degrees (360 for a simple closed loop).
    pub turning_degrees: f64,
    /// The turning with the sign of each bend kept: positive turning left.
    pub signed_turning_degrees: f64,
    /// Why the run stopped growing backwards.
    pub backward_stop: RunStopReason,
    /// Why the run stopped growing forwards.
    pub forward_stop: RunStopReason,
}

impl SegmentRun {
    /// Whether the run is the whole closed contour.
    pub fn is_loop(&self) -> bool {
        self.backward_stop == RunStopReason::Loop
    }

    /// The run's segments in walk order, wrapped into range.
    pub fn segment_indices(&self) -> impl Iterator<Item = usize> + '_ {
        (0..self.count).map(move |i| (self.first_segment + i) % self.segment_count)
    }
}

/// Why [`walk`] refused its arguments.
#[derive(Debug, Clone, PartialEq)]
pub enum PathSegmentRunError {
    /// C# `ArgumentException`: fewer than two points.
    TooFewPoints,
    /// C# `ArgumentOutOfRangeException` for `seedSegmentIndex`.
    SeedOutOfRange {
        /// The seed given.
        seed_segment_index: i32,
        /// How many segments the contour has.
        segment_count: usize,
    },
    /// C# `ArgumentOutOfRangeException` for an angle that is negative or NaN.
    NotAnAngle {
        /// The parameter, by its C# name.
        parameter: &'static str,
        /// The value given.
        degrees: f64,
    },
}

impl fmt::Display for PathSegmentRunError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PathSegmentRunError::TooFewPoints => write!(
                f,
                "A contour needs at least two points to have a segment to walk."
            ),
            PathSegmentRunError::SeedOutOfRange {
                seed_segment_index,
                segment_count,
            } => write!(
                f,
                "Segment {seed_segment_index} is not one of this contour's {segment_count} segments."
            ),
            PathSegmentRunError::NotAnAngle { .. } => write!(
                f,
                "An angle in degrees must be a number and cannot be negative."
            ),
        }
    }
}

impl std::error::Error for PathSegmentRunError {}

/// How far over the budget an end may still count as within it, in degrees.
const BUDGET_SLACK: f64 = 1e-9;

/// A segment shorter than this has no direction to measure.
const ZERO_LENGTH: f64 = 1e-9;

/// Grows the seed segment into the run of segments around it that are curved but close (C#
/// `PathSegmentRun.Walk`). Each end may turn `turning_budget_degrees` away from the seed; a
/// vertex bending `corner_degrees` or more stops an end.
pub fn walk(
    points: &[Vector2],
    closed: bool,
    seed_segment_index: i32,
    turning_budget_degrees: f64,
    corner_degrees: f64,
) -> Result<SegmentRun, PathSegmentRunError> {
    if points.len() < 2 {
        return Err(PathSegmentRunError::TooFewPoints);
    }

    throw_if_not_an_angle(turning_budget_degrees, "turningBudgetDegrees")?;
    throw_if_not_an_angle(corner_degrees, "cornerDegrees")?;

    let point_count = points.len() as i64;
    let segment_count = if closed { point_count } else { point_count - 1 };

    let seed = i64::from(seed_segment_index);

    if seed < 0 || seed >= segment_count {
        return Err(PathSegmentRunError::SeedOutOfRange {
            seed_segment_index,
            segment_count: segment_count as usize,
        });
    }

    let mut ends = Ends::default();
    let mut backwards_turn = true;

    while ends.backward_stop.is_none() || ends.forward_stop.is_none() {
        let count = 1 + ends.backward_count + ends.forward_count;
        let first_segment = wrap(seed - ends.backward_count, segment_count);
        let last_segment = (seed + ends.forward_count) % segment_count;

        // before any bend is asked for: the bend lookup wraps whatever the contour is
        if !closed {
            if ends.backward_stop.is_none() && first_segment == 0 {
                ends.backward_stop = Some(RunStopReason::End);
                continue;
            }

            if ends.forward_stop.is_none() && last_segment == segment_count - 1 {
                ends.forward_stop = Some(RunStopReason::End);
                continue;
            }
        }

        let backward_vertex = first_segment as usize;
        let forward_vertex = ((last_segment + 1) % point_count) as usize;

        if closed && count == segment_count {
            // every segment is in and the two frontiers meet on one last joint
            close_on_last_joint(
                points,
                backward_vertex,
                turning_budget_degrees,
                corner_degrees,
                &mut ends,
            );
            break;
        }

        if closed
            && count == segment_count - 1
            && ends.backward_stop.is_none()
            && ends.forward_stop.is_none()
        {
            // one segment left and both ends want it: ask each end about its own joint
            let backward_refusal = refuse(
                abs_bend_degrees_at(points, backward_vertex),
                ends.backward_turning,
                turning_budget_degrees,
                corner_degrees,
            );
            let forward_refusal = refuse(
                abs_bend_degrees_at(points, forward_vertex),
                ends.forward_turning,
                turning_budget_degrees,
                corner_degrees,
            );

            match (backward_refusal, forward_refusal) {
                (Some(backward), Some(forward)) => {
                    ends.backward_stop = Some(backward);
                    ends.forward_stop = Some(forward);
                    break;
                }
                (None, None) => {
                    cross(
                        points,
                        backward_vertex,
                        &mut ends.backward_turning,
                        &mut ends.signed_turning,
                    );
                    cross(
                        points,
                        forward_vertex,
                        &mut ends.forward_turning,
                        &mut ends.signed_turning,
                    );
                    // the one segment left joins at both ends; counted backwards
                    ends.backward_count += 1;
                    ends.backward_stop = Some(RunStopReason::Loop);
                    ends.forward_stop = Some(RunStopReason::Loop);
                    break;
                }
                (None, Some(forward)) => {
                    cross(
                        points,
                        backward_vertex,
                        &mut ends.backward_turning,
                        &mut ends.signed_turning,
                    );
                    ends.backward_count += 1;
                    ends.forward_stop = Some(forward);
                }
                (Some(backward), None) => {
                    cross(
                        points,
                        forward_vertex,
                        &mut ends.forward_turning,
                        &mut ends.signed_turning,
                    );
                    ends.forward_count += 1;
                    ends.backward_stop = Some(backward);
                }
            }

            continue;
        }

        let step_backward =
            ends.backward_stop.is_none() && (ends.forward_stop.is_some() || backwards_turn);
        backwards_turn = !step_backward;

        if step_backward {
            if let Some(refusal) = refuse(
                abs_bend_degrees_at(points, backward_vertex),
                ends.backward_turning,
                turning_budget_degrees,
                corner_degrees,
            ) {
                ends.backward_stop = Some(refusal);
                continue;
            }

            cross(
                points,
                backward_vertex,
                &mut ends.backward_turning,
                &mut ends.signed_turning,
            );
            ends.backward_count += 1;
        } else {
            if let Some(refusal) = refuse(
                abs_bend_degrees_at(points, forward_vertex),
                ends.forward_turning,
                turning_budget_degrees,
                corner_degrees,
            ) {
                ends.forward_stop = Some(refusal);
                continue;
            }

            cross(
                points,
                forward_vertex,
                &mut ends.forward_turning,
                &mut ends.signed_turning,
            );
            ends.forward_count += 1;
        }
    }

    Ok(SegmentRun {
        first_segment: wrap(seed - ends.backward_count, segment_count) as usize,
        count: (1 + ends.backward_count + ends.forward_count) as usize,
        segment_count: segment_count as usize,
        turning_degrees: ends.backward_turning + ends.forward_turning,
        signed_turning_degrees: ends.signed_turning,
        backward_stop: ends.backward_stop.unwrap_or(RunStopReason::End),
        forward_stop: ends.forward_stop.unwrap_or(RunStopReason::End),
    })
}

/// The walk's state: what each end has taken and spent, and why it stopped.
#[derive(Debug, Default)]
struct Ends {
    backward_count: i64,
    forward_count: i64,
    backward_turning: f64,
    forward_turning: f64,
    signed_turning: f64,
    backward_stop: Option<RunStopReason>,
    forward_stop: Option<RunStopReason>,
}

/// Settles the one joint left when the run holds every segment of a closed contour.
fn close_on_last_joint(
    points: &[Vector2],
    vertex: usize,
    turning_budget_degrees: f64,
    corner_degrees: f64,
    ends: &mut Ends,
) {
    let bend = abs_bend_degrees_at(points, vertex);

    if ends.backward_stop.is_none() {
        match refuse(
            bend,
            ends.backward_turning,
            turning_budget_degrees,
            corner_degrees,
        ) {
            None => {
                cross(
                    points,
                    vertex,
                    &mut ends.backward_turning,
                    &mut ends.signed_turning,
                );
                ends.backward_stop = Some(RunStopReason::Loop);
                ends.forward_stop = Some(RunStopReason::Loop);
                return;
            }
            Some(refusal) => ends.backward_stop = Some(refusal),
        }
    }

    if ends.forward_stop.is_none() {
        match refuse(
            bend,
            ends.forward_turning,
            turning_budget_degrees,
            corner_degrees,
        ) {
            None => {
                cross(
                    points,
                    vertex,
                    &mut ends.forward_turning,
                    &mut ends.signed_turning,
                );
                // a loop has no ends, so an end that stopped short is reported as the loop
                ends.backward_stop = Some(RunStopReason::Loop);
                ends.forward_stop = Some(RunStopReason::Loop);
            }
            Some(refusal) => ends.forward_stop = Some(refusal),
        }
    }
}

/// Why crossing a joint that bends this far would be refused, or `None` if this end may.
fn refuse(
    bend_degrees: f64,
    spent_degrees: f64,
    turning_budget_degrees: f64,
    corner_degrees: f64,
) -> Option<RunStopReason> {
    if bend_degrees >= corner_degrees {
        return Some(RunStopReason::Corner);
    }

    if spent_degrees + bend_degrees > turning_budget_degrees + BUDGET_SLACK {
        return Some(RunStopReason::BudgetSpent);
    }

    None
}

/// Charges a joint to the end that crossed it, and to the run's signed turning.
fn cross(points: &[Vector2], vertex: usize, end_turning: &mut f64, signed_turning: &mut f64) {
    let signed = signed_bend_degrees_at(points, vertex);
    *end_turning += signed.abs();
    *signed_turning += signed;
}

fn abs_bend_degrees_at(points: &[Vector2], vertex_index: usize) -> f64 {
    signed_bend_degrees_at(points, vertex_index).abs()
}

/// How far the contour's direction turns at a vertex, in degrees, positive turning left. Wraps
/// its neighbour lookup whatever the contour, so never ask about an open contour's end.
fn signed_bend_degrees_at(points: &[Vector2], vertex_index: usize) -> f64 {
    let (Some(incoming), Some(outgoing)) = (
        try_direction_to(points, vertex_index, -1),
        try_direction_to(points, vertex_index, 1),
    ) else {
        // every point sits on this one
        return 0.0;
    };

    // atan2 of the cross against the dot rather than acos, which is flat near 1
    let cross = incoming.cross(outgoing);
    let dot = incoming.dot(outgoing);
    math_helper::radians_to_degrees(cross.atan2(dot))
}

/// The direction of the contour on one side of a vertex, from the nearest neighbour far enough
/// away to have one, pointing along the walk.
fn try_direction_to(points: &[Vector2], vertex_index: usize, step: i64) -> Option<Vector2> {
    let point_count = points.len() as i64;
    let at = points[vertex_index];

    for taken in 1..point_count {
        let index = wrap(vertex_index as i64 + (step * taken), point_count) as usize;
        let offset = points[index] - at;

        if offset.length() >= ZERO_LENGTH {
            return Some(if step > 0 { offset } else { -offset });
        }
    }

    None
}

/// An index brought into range, for negatives as well as overruns.
fn wrap(index: i64, count: i64) -> i64 {
    ((index % count) + count) % count
}

/// Refuses an angle that is negative or not a number.
fn throw_if_not_an_angle(degrees: f64, parameter: &'static str) -> Result<(), PathSegmentRunError> {
    if degrees.is_nan() || degrees < 0.0 {
        return Err(PathSegmentRunError::NotAnAngle { parameter, degrees });
    }

    Ok(())
}
