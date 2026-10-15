//! The circle arithmetic behind a fitted source round: the Kasa algebraic circle fit and its 3 by 3
//! solve, the chord count a surface was drawn with, and how much of the circle its points go round;
//! `bevel_source_rounds.rs` is the reader that decides what to fit.

use std::f64::consts::TAU;

use vector_math::{ieee_math, Vector2, Vector3};

use super::bevel_source_rounds::ONE_NORMAL_RADIANS;

/// How far off the fitted circle a vertex may sit, as a fraction of the fitted radius.
const CIRCLE_RESIDUAL_FRACTION: f64 = 1e-3;

/// The circle these points lie on (centre, radius), by the Kasa algebraic fit - which is linear,
/// so there is nothing to iterate and nothing to seed. `None` where the fit is degenerate or where
/// any point is off the circle by more than [`CIRCLE_RESIDUAL_FRACTION`], which is what says these
/// points were not a circle in the first place.
pub(super) fn try_fit_circle(points: &[Vector2]) -> Option<(Vector2, f64)> {
    // Kasa minimises the error in x^2 + y^2 - 2ax - 2by - c, which is linear in (a, b, c), so the
    // normal equations are this 3 by 3.
    let (mut sx, mut sy, mut sxx, mut syy, mut sxy, mut sz, mut sxz, mut syz) =
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    let n = points.len() as f64;

    for point in points {
        let z = (point.x * point.x) + (point.y * point.y);

        sx += point.x;
        sy += point.y;
        sxx += point.x * point.x;
        syy += point.y * point.y;
        sxy += point.x * point.y;
        sz += z;
        sxz += point.x * z;
        syz += point.y * z;
    }

    let m = [
        [2.0 * sxx, 2.0 * sxy, sx],
        [2.0 * sxy, 2.0 * syy, sy],
        [2.0 * sx, 2.0 * sy, n],
    ];

    let rhs = [sxz, syz, sz];

    let solution = try_solve3(m, rhs)?;

    let squared = solution[2] + (solution[0] * solution[0]) + (solution[1] * solution[1]);

    if squared.is_nan() || squared <= 0.0 {
        return None;
    }

    let centre = Vector2::new(solution[0], solution[1]);
    let radius = squared.sqrt();

    for &point in points {
        if ((point - centre).length() - radius).abs() > radius * CIRCLE_RESIDUAL_FRACTION {
            return None;
        }
    }

    Some((centre, radius))
}

/// A 3 by 3 solve by Gaussian elimination with partial pivoting.
fn try_solve3(mut a: [[f64; 3]; 3], mut b: [f64; 3]) -> Option<[f64; 3]> {
    let mut solution = [0.0; 3];

    for column in 0..3 {
        let mut pivot = column;

        for row in (column + 1)..3 {
            if a[row][column].abs() > a[pivot][column].abs() {
                pivot = row;
            }
        }

        if a[pivot][column].abs() < 1e-12 {
            return None;
        }

        if pivot != column {
            a.swap(column, pivot);
            b.swap(column, pivot);
        }

        for row in (column + 1)..3 {
            let factor = a[row][column] / a[column][column];

            let pivot_row = a[column];

            for (k, value) in a[row].iter_mut().enumerate().skip(column) {
                *value -= factor * pivot_row[k];
            }

            b[row] -= factor * b[column];
        }
    }

    for row in (0..3).rev() {
        let mut sum = b[row];

        for k in (row + 1)..3 {
            sum -= a[row][k] * solution[k];
        }

        solution[row] = sum / a[row][row];
    }

    Some(solution)
}

/// How many chords the whole cylinder would be drawn with at this surface's tessellation: a turn
/// of the circle divided by the MEDIAN turn between neighbouring facet normals. The median rather
/// than the mean because a boolean leaves slivers, and a sliver's two normals sit on top of each
/// other.
pub(super) fn count_the_chords(normals: &[Vector3], e1: Vector3, e2: Vector3) -> i32 {
    let mut angles: Vec<f64> = normals
        .iter()
        .map(|normal| normal.dot(e2).atan2(normal.dot(e1)))
        .collect();

    // Equal angles are interchangeable, so the sort's tie order cannot reach the gaps.
    angles.sort_by(f64::total_cmp);

    let mut gaps: Vec<f64> = angles
        .windows(2)
        .map(|pair| pair[1] - pair[0])
        .filter(|&gap| gap > ONE_NORMAL_RADIANS)
        .collect();

    if gaps.is_empty() {
        return 0;
    }

    gaps.sort_by(f64::total_cmp);

    let median = gaps[gaps.len() / 2];

    if median > 0.0 {
        // Banker's rounding; the count of a real tessellation is far inside i32, so `as` never
        // saturates.
        (TAU / median).round_ties_even() as i32
    } else {
        0
    }
}

/// How much of the fitted circle the points the fit was made through actually go round, in
/// radians.
///
/// Measured as a turn LESS the widest gap between neighbouring points rather than as the greatest
/// angle between any two, so a surface that goes the whole way round - where the two extremes sit
/// either side of the branch cut of atan2 and are next to each other on the part - reads as nearly
/// a full turn rather than as whatever the branch cut happened to leave.
pub(super) fn angular_span(points: &[Vector2], centre: Vector2) -> f64 {
    let mut angles: Vec<f64> = points
        .iter()
        .map(|&point| {
            let relative = point - centre;
            relative.y.atan2(relative.x)
        })
        .collect();

    angles.sort_by(f64::total_cmp);

    let mut widest = (angles[0] + TAU) - angles[angles.len() - 1];

    for pair in angles.windows(2) {
        widest = ieee_math::max_f64(widest, pair[1] - pair[0]);
    }

    ieee_math::max_f64(0.0, TAU - widest)
}
