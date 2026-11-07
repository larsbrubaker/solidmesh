//! Spinning a 2D profile into a solid: the sweep's angles (snapped onto a full turn when they are
//! within a hair of one) and the profile's clean-up. The profile arrives as integer polygons at
//! `DEFAULT_SCALING` (`outline_to_mesh`); its winding is normalized and anything left of the axis
//! is mirrored onto the right before it is swept, then it is prepared as loops a solid can be built
//! on (`processors/ring_loops.rs`). The solid itself is built from those integer loops in
//! `revolve/revolved_mesh.rs`.

use std::fmt;

use clipper2_rust::{FillRule, Path64, Paths64};
use clipper_library::{get_corrected_winding, int_point, ClipperError, PolygonsExtensions};
use vector_math::math_helper::{range_0_to_tau, TAU};
use vector_math::MathError;

use crate::mesh::Mesh;
use crate::processors::ring_loops::simple_loops;

mod revolved_mesh;

use revolved_mesh::revolved_mesh;

/// What [`revolve`] can fail with: an angle that cannot be brought into `[0, Tau]`, or a
/// clipper failure in the profile's clean-up.
#[derive(Clone, Debug, PartialEq)]
pub enum RevolveError {
    Angle(MathError),
    Clipper(ClipperError),
}

impl From<MathError> for RevolveError {
    fn from(error: MathError) -> Self {
        RevolveError::Angle(error)
    }
}

impl From<ClipperError> for RevolveError {
    fn from(error: ClipperError) -> Self {
        RevolveError::Clipper(error)
    }
}

impl fmt::Display for RevolveError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            RevolveError::Angle(error) => error.fmt(f),
            RevolveError::Clipper(error) => error.fmt(f),
        }
    }
}

impl std::error::Error for RevolveError {}

/// Snaps an angle within 0.001 of 0 or Tau onto it.
fn fix_close_angles(angle: f64) -> f64 {
    if angle.abs() < 0.001 {
        return 0.0;
    }

    if angle < TAU + 0.001 && angle > TAU - 0.001 {
        return TAU;
    }

    angle
}

/// The sweep's start and end in `[0, Tau]`, an end within 0.001 of 0 or Tau snapped onto it so
/// a sweep meant as a full turn closes. A sweep narrower than the snap distance keeps its own
/// angles: snapping both of its ends onto the same place would sweep nothing.
fn sweep_angles(angle_start: f64, angle_end: f64) -> Result<(f64, f64), MathError> {
    let angle_start = range_0_to_tau(angle_start)?;
    let angle_end = range_0_to_tau(angle_end)?;
    let (snapped_start, snapped_end) = (fix_close_angles(angle_start), fix_close_angles(angle_end));
    if snapped_start == snapped_end {
        return Ok((angle_start, angle_end));
    }
    Ok((snapped_start, snapped_end))
}

/// Sweeps the profile (integer polygons at `DEFAULT_SCALING`, x out from the axis, y along it) from
/// `angle_start` to `angle_end` in `angle_steps` strips (fewer where a strip would be narrower than
/// the profile's precision), capping both ends of a partial sweep. A profile that encloses no
/// area, or a sweep of no angle, revolves to an empty mesh.
///
/// The angles are brought into `[0, Tau]` first; one that cannot be (below `-Tau`, or NaN) is
/// an error; a clipper failure is the other.
pub fn revolve(
    polygons: &Paths64,
    angle_steps: i32,
    angle_start: f64,
    angle_end: f64,
    revolve_around_z: bool,
) -> Result<Mesh, RevolveError> {
    let mut angle_steps = angle_steps;
    let (angle_start, angle_end) = sweep_angles(angle_start, angle_end)?;
    // nothing is swept, so there is no solid
    if angle_start == angle_end {
        return Ok(Mesh::new());
    }

    let fewest_steps = if angle_start == 0.0 && angle_end == TAU {
        3
    } else {
        1
    };
    angle_steps = angle_steps.max(fewest_steps);

    // the profile is cleaned up in a copy of its own
    let mut polygons = polygons.clone();

    // A profile enclosing no area - flat, or with no points at all - is what a primitive's
    // editor holds between one keystroke and the next. It cannot make a solid, so it is an
    // empty one.
    let profile_bounds = polygons.get_bounds();
    if profile_bounds.width() <= 0.0 || profile_bounds.height() <= 0.0 {
        return Ok(Mesh::new());
    }

    // Every profile needs its winding normalized, whichever side of the axis it is on: a
    // clockwise one revolves inside out.
    polygons = get_corrected_winding(&polygons)?;
    if polygons.is_empty() {
        // winding correction drops what encloses no area
        return Ok(Mesh::new());
    }

    // Only the profiles with material left of the axis need clipping and mirroring.
    if polygons.iter().any(|poly| poly.iter().any(|pos| pos.x < 0)) {
        let mut bounds = polygons.get_bounds();
        bounds.inflate(10.0);
        // clip against x=0 left and right
        let left_clip: Path64 = vec![
            int_point(0.0, bounds.bottom),
            int_point(0.0, bounds.top),
            int_point(bounds.left, bounds.top),
            int_point(bounds.left, bounds.bottom),
        ];
        let right_stuff = polygons.subtract_polygon(&left_clip)?;

        let right_clip: Path64 = vec![
            int_point(0.0, bounds.top),
            int_point(0.0, bounds.bottom),
            int_point(bounds.right, bounds.bottom),
            int_point(bounds.right, bounds.top),
        ];
        let left_stuff = polygons.subtract_polygon(&right_clip)?;
        // mirror left material across the origin
        let left_add = left_stuff.scale(-1.0, 1.0);
        polygons = if !left_add.is_empty() {
            if !right_stuff.is_empty() {
                right_stuff.union(&left_add, FillRule::EvenOdd)?
            } else {
                left_add
            }
        } else {
            // there is nothing on the left
            right_stuff
        };
        polygons = get_corrected_winding(&polygons)?;
    }

    // The profile is placed to the nearest clipper unit, so a strip less than a unit wide at the
    // profile's outermost point adds no shape, only slivers thinner than the mesh's f32 positions
    // can tell apart. Only a sweep of a tiny angle or a tiny profile ever has strips that narrow.
    let outermost = polygons
        .iter()
        .flatten()
        .map(|point| point.x)
        .max()
        .unwrap_or(0);
    // `as` saturates; the arc is far inside i32 for any profile clipper accepts
    let widest_strips = (outermost as f64 * (angle_end - angle_start).abs()) as i32;
    angle_steps = angle_steps.min(widest_strips.max(fewest_steps));

    // loops a column can be built on: where loops touch, each pass through a point gets its own
    // vertex, so every strip edge has exactly two faces
    let polygons = simple_loops(polygons);
    let has_start_and_end_faces = angle_start > 0.000001 || angle_end < TAU - 0.000001;
    let wraps = !has_start_and_end_faces;
    Ok(revolved_mesh(
        &polygons,
        &column_angles(angle_start, angle_end, angle_steps, wraps),
        wraps,
        revolve_around_z,
    ))
}

/// The angle of every column of the sweep: `angle_steps` strips from `angle_start`, accumulated
/// one step at a time. A full turn has a column per strip, its last strip closing onto the first
/// column; a partial sweep has one more, where its end cap sits (the accumulated angle, which is
/// only approximately `angle_end`).
fn column_angles(angle_start: f64, angle_end: f64, angle_steps: i32, wraps: bool) -> Vec<f64> {
    let angle_delta = (angle_end - angle_start) / f64::from(angle_steps);
    let columns = if wraps { angle_steps } else { angle_steps + 1 };
    let mut angles = Vec::with_capacity(columns.max(0) as usize);
    let mut current_angle = angle_start;
    for _ in 0..columns {
        angles.push(current_angle);
        current_angle += angle_delta;
    }
    angles
}
