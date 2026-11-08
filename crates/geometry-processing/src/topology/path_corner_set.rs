//! Every corner of a path that a bevel would round, at one bend threshold ([`PathCornerSet`] of
//! [`PathCorner`]s).
//!
//! The 2D analog of [`super::FeatureEdgeSet`]. Built from
//! [`crate::path::path_bevel::cleaned_contours`] and decided by the bevel's own candidate test,
//! so the corners offered in the viewport are by construction exactly the vertices the bevel
//! will consider. [`super::bevel_corner_group_resolver`] matches stored corner groups in it.
//! It is built from the same flattened contours the bevel is handed (the app flattens its paths
//! for both in mattercad-geometry).

use vector_math::{ieee_math, math_helper, Vector2};

use crate::path::path_bevel::{self, Contour};

/// One corner of a path: where it is, and the unit directions of the two edges that meet there.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct PathCorner {
    /// Which contour the corner is on, indexing [`PathCornerSet::contours`].
    pub contour_index: usize,
    /// Where the corner sits in its contour: the index a selection predicate is handed. Not
    /// unique across the path.
    pub vertex_index: usize,
    /// Where the corner is, in the path's own 2D space.
    pub position: Vector2,
    /// The unit direction back along the edge to the vertex before it.
    pub to_previous: Vector2,
    /// The unit direction on along the edge to the vertex after it.
    pub to_next: Vector2,
}

impl PathCorner {
    /// How far the path turns at this corner, in degrees: 0 running straight on, 180 at a cusp.
    pub fn bend_degrees(&self) -> f64 {
        180.0
            - math_helper::radians_to_degrees(libm::acos(clamp_passing_nan(
                self.to_previous.dot(self.to_next),
                -1.0,
                1.0,
            )))
    }
}

/// The corners of a path at a bend threshold.
#[derive(Debug, Clone, PartialEq)]
pub struct PathCornerSet {
    /// The threshold the corners were selected at, in degrees.
    pub bend_threshold_degrees: f64,
    /// The contours the corners were found on, cleaned as the bevel cleans them.
    pub contours: Vec<Contour>,
    /// The corners, in contour then vertex order.
    pub corners: Vec<PathCorner>,
    /// The diagonal of the whole path's bounding box, over every point of every contour: the
    /// scale a match judges drift against.
    pub extent: f64,
}

impl PathCornerSet {
    /// How many corners there are.
    pub fn count(&self) -> usize {
        self.corners.len()
    }

    /// Finds the corners of a path's contours (flattened, as [`path_bevel::bevel`] is handed
    /// them) at a bend threshold.
    pub fn from_contours(contours: &[Contour], bend_threshold_degrees: f64) -> PathCornerSet {
        let contours = path_bevel::cleaned_contours(contours);
        let mut corners = Vec::new();
        let mut minimum = Vector2::new(f64::MAX, f64::MAX);
        let mut maximum = Vector2::new(f64::MIN, f64::MIN);
        let mut any_point = false;

        for (contour_index, contour) in contours.iter().enumerate() {
            let points = &contour.points;

            for point in points {
                minimum = Vector2::new(
                    ieee_math::min_f64(minimum.x, point.x),
                    ieee_math::min_f64(minimum.y, point.y),
                );
                maximum = Vector2::new(
                    ieee_math::max_f64(maximum.x, point.x),
                    ieee_math::max_f64(maximum.y, point.y),
                );
                any_point = true;
            }

            // fewer than three and the bevel hands the contour back untouched, but it keeps its
            // place in the list, because that place is the contour ordinal
            if points.len() < 3 {
                continue;
            }

            // asked with no selection: this set answers "which vertices are corners"
            let is_candidate = path_bevel::identify_bevel_candidates(
                points,
                contour.is_closed,
                bend_threshold_degrees,
                None,
                0,
            );

            let count = points.len();
            for i in 0..count {
                if !is_candidate[i] {
                    continue;
                }

                // candidates are never the ends of an open contour, so i - 1 and i + 1 exist
                let previous = if contour.is_closed {
                    points[(i + count - 1) % count]
                } else {
                    points[i - 1]
                };
                let next = if contour.is_closed {
                    points[(i + 1) % count]
                } else {
                    points[i + 1]
                };

                corners.push(PathCorner {
                    contour_index,
                    vertex_index: i,
                    position: points[i],
                    to_previous: (previous - points[i]).get_normal(),
                    to_next: (next - points[i]).get_normal(),
                });
            }
        }

        let extent = if any_point {
            (maximum - minimum).length()
        } else {
            0.0
        };

        PathCornerSet {
            bend_threshold_degrees,
            contours,
            corners,
            extent,
        }
    }
}

/// Clamps `value` into `[min, max]`, passing a NaN through (where `f64::clamp` would panic on a
/// NaN bound) so the NaN guards downstream still see it.
fn clamp_passing_nan(value: f64, min: f64, max: f64) -> f64 {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}
