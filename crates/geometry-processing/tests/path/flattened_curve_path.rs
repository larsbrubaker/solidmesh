//! An open path of lines and cubic Béziers, flattened as it is drawn: what a caller with curves
//! hands the path bevel and the segment-run walk (both take contours already flattened). Each
//! cubic becomes [`SEGMENTS_PER_CUBIC`] chords at even steps of its parameter. What these tests
//! measure against the exact curve (a fillet's radius and reach where it lands on a curved run)
//! carries an error of about one chord length, so the chords are kept a few hundredths of a
//! millimetre long on the arcs drawn here (radius 20 or less, a quarter turn or less per cubic).

#![allow(dead_code)]

use geometry_processing::path::Contour;
use vector_math::Vector2;

/// Chords per cubic: 256 makes a 4 mm radius eighth turn's chords 12 microns long (64 left the
/// largest fillet on such a run 8.5 microns over the exact one, past its 5 micron tolerance).
pub const SEGMENTS_PER_CUBIC: usize = 256;

/// An open path being drawn, held as its flattened points.
#[derive(Debug, Clone, Default)]
pub struct FlattenedCurvePath {
    points: Vec<Vector2>,
}

impl FlattenedCurvePath {
    /// A path starting at (x, y).
    pub fn starting_at(x: f64, y: f64) -> FlattenedCurvePath {
        FlattenedCurvePath {
            points: vec![Vector2::new(x, y)],
        }
    }

    /// A straight line on to (x, y).
    pub fn line_to(&mut self, x: f64, y: f64) {
        self.points.push(Vector2::new(x, y));
    }

    /// A cubic Bézier on from the current point through the two control points to `end`.
    pub fn curve4_vector(&mut self, control_1: Vector2, control_2: Vector2, end: Vector2) {
        let start = *self.points.last().expect("a path starts with a point");
        for step in 1..=SEGMENTS_PER_CUBIC {
            let t = step as f64 / SEGMENTS_PER_CUBIC as f64;
            let u = 1.0 - t;
            let point = (start * (u * u * u))
                + (control_1 * (3.0 * u * u * t))
                + (control_2 * (3.0 * u * t * t))
                + (end * (t * t * t));
            self.points.push(point);
        }
    }

    /// The path as the one open contour it is.
    pub fn contour(&self) -> Contour {
        Contour {
            points: self.points.clone(),
            is_closed: false,
        }
    }
}
