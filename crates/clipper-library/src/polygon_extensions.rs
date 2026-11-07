//! The extension methods on one ClipperLib polygon (`List<IntPoint>`, clipper2-rust's `Path64`)
//! that MatterCAD and PolygonMesh call; the many-polygon helpers are in
//! [`super::polygons_extensions`].

use clipper2_rust::{ClipType, FillRule, JoinType, Path64, Paths64, Point64};

use super::clipper_error::ClipperError;
use super::clipper_statics::execute;
use super::int_point_extensions::{calc_intersection, get_intersection, int_point, Intersection};
use super::polygons_extensions::PolygonsExtensions;
use vector_math::rectangle_double::RectangleDouble;

/// Helpers on a single polygon.
pub trait PolygonExtensions {
    /// The bounds of every point ([`RectangleDouble::ZERO_INTERSECTION`] when empty).
    fn get_bounds(&self) -> RectangleDouble;

    /// Every edge of the closed polygon the segment `start`-`end` meets: the edge's start
    /// index, how it meets, and where. Where the lines are parallel the
    /// position is the origin.
    fn get_intersections(
        &self,
        start: Point64,
        end: Point64,
    ) -> Vec<(usize, Intersection, Point64)>;

    /// 1 if counter-clockwise (y up), -1 if clockwise or degenerate, from the signed area.
    /// Callers keep the polygons that return 1 as outer outlines and drop the holes.
    fn get_winding_direction(&self) -> i32;

    /// The sum of the edge lengths (with the closing edge when
    /// `is_closed`): the perimeter, not the root of
    /// [`PolygonExtensions::length_squared`].
    fn length(&self, is_closed: bool) -> f64;

    /// The sum of each edge's squared length (with the closing
    /// edge when `is_closed`).
    fn length_squared(&self, is_closed: bool) -> f64;

    /// Offset by `distance` clipper units with round joins, the arc
    /// tolerance cut by the polygon's own size ([`PolygonsExtensions::arc_tolerance_for`]).
    fn offset(&self, distance: f64) -> Result<Paths64, ClipperError>;

    /// Round joins at a fixed arc tolerance.
    fn offset_with_arc_tolerance(
        &self,
        distance: f64,
        arc_tolerance: f64,
    ) -> Result<Paths64, ClipperError>;

    /// Each point rotated about the origin and rounded back to the grid.
    fn rotate(&self, radians: f64) -> Path64;

    fn scale(&self, scale_x: f64, scale_y: f64) -> Path64;

    /// Even-odd difference.
    fn subtract(&self, other: &Paths64) -> Result<Paths64, ClipperError>;

    fn subtract_polygon(&self, other: &Path64) -> Result<Paths64, ClipperError>;

    /// Each point moved by `(x, y) * scale`.
    fn translate(&self, x: f64, y: f64, scale: f64) -> Path64;
}

impl PolygonExtensions for Path64 {
    fn get_bounds(&self) -> RectangleDouble {
        let mut bounds = RectangleDouble::ZERO_INTERSECTION;
        for point in self {
            bounds.expand_to_include_xy(point.x as f64, point.y as f64);
        }
        bounds
    }

    fn get_intersections(
        &self,
        start: Point64,
        end: Point64,
    ) -> Vec<(usize, Intersection, Point64)> {
        let mut intersections = Vec::new();
        for i in 0..self.len() {
            let edge_start = self[i];
            let edge_end = self[(i + 1) % self.len()];

            let intersection = get_intersection(start, end, edge_start, edge_end);
            if intersection != Intersection::None {
                let position = calc_intersection(start, end, edge_start, edge_end)
                    .unwrap_or_else(|| Point64::new(0, 0));
                intersections.push((i, intersection, position));
            }
        }
        intersections
    }

    fn get_winding_direction(&self) -> i32 {
        // The signed (shoelace) area tells the orientation. A sum of
        // `(x2 + x1) * (y2 + y1)` would not: it is the same for a polygon and its reverse.
        if super::clipper_statics::area(self) > 0.0 {
            1
        } else {
            -1
        }
    }

    fn length(&self, is_closed: bool) -> f64 {
        // Sum the edge lengths: the root of the summed squared edges is not the path's length
        // once it has more than one edge.
        let mut length = 0.0;
        if self.len() > 1 {
            let mut previous_point = if is_closed {
                self[self.len() - 1]
            } else {
                self[0]
            };
            for &current_point in &self[if is_closed { 0 } else { 1 }..] {
                let dx = (previous_point.x - current_point.x) as f64;
                let dy = (previous_point.y - current_point.y) as f64;
                length += (dx * dx + dy * dy).sqrt();
                previous_point = current_point;
            }
        }
        length
    }

    fn length_squared(&self, is_closed: bool) -> f64 {
        let mut length = 0.0;
        if self.len() > 1 {
            let mut previous_point = self[0];
            if is_closed {
                previous_point = self[self.len() - 1];
            }
            for &current_point in &self[if is_closed { 0 } else { 1 }..] {
                let dx = previous_point.x - current_point.x;
                let dy = previous_point.y - current_point.y;
                length += (dx * dx + dy * dy) as f64;
                previous_point = current_point;
            }
        }
        length
    }

    fn offset(&self, distance: f64) -> Result<Paths64, ClipperError> {
        let arc_tolerance = vec![self.clone()]
            .arc_tolerance_for(super::polygons_extensions::DEFAULT_CLIPPER_UNITS_PER_UNIT);
        self.offset_with_arc_tolerance(distance, arc_tolerance)
    }

    fn offset_with_arc_tolerance(
        &self,
        distance: f64,
        arc_tolerance: f64,
    ) -> Result<Paths64, ClipperError> {
        vec![self.clone()].offset_with_arc_tolerance(distance, JoinType::Round, arc_tolerance)
    }

    fn rotate(&self, radians: f64) -> Path64 {
        let cos = radians.cos();
        let sin = radians.sin();
        self.iter()
            .map(|point| {
                let (x, y) = (point.x as f64, point.y as f64);
                int_point(x * cos - y * sin, y * cos + x * sin)
            })
            .collect()
    }

    fn scale(&self, scale_x: f64, scale_y: f64) -> Path64 {
        self.iter()
            .map(|point| int_point(point.x as f64 * scale_x, point.y as f64 * scale_y))
            .collect()
    }

    fn subtract(&self, other: &Paths64) -> Result<Paths64, ClipperError> {
        execute(
            ClipType::Difference,
            &vec![self.clone()],
            other,
            FillRule::EvenOdd,
        )
    }

    fn subtract_polygon(&self, other: &Path64) -> Result<Paths64, ClipperError> {
        self.subtract(&vec![other.clone()])
    }

    fn translate(&self, x: f64, y: f64, scale: f64) -> Path64 {
        self.iter()
            .map(|point| int_point(point.x as f64 + x * scale, point.y as f64 + y * scale))
            .collect()
    }
}
