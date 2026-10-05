//! The pieces of agg-sharp's `clipper_library` that PolygonMesh's slicing and stitching call,
//! on clipper2-rust's integer polygons (`Point64`/`Path64`/`Paths64` stand in for ClipperLib's
//! `IntPoint`/`Polygon`/`Polygons`): segment intersection (`IntPointExtensions`),
//! `Polygon.GetIntersections` (`PolygonExtensions`), `PolygonsExtensions.CreateFromString` and
//! the `Union` helper, and agg-sharp's modified Clipper 6.4.2 `CleanPolygon(s)` (`clipper.cs`,
//! Boost Software License 1.0), which Clipper2 does not have.

use std::cmp::Ordering;

use clipper2_rust::{union_64, FillRule, Path64, Paths64, Point64};

/// How two segments meet (C# `ClipperLib.Intersection`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Intersection {
    None,
    Colinear,
    Intersect,
}

/// ClipperLib's `new IntPoint(double, double)`: each coordinate rounded half to even
/// (`Math.Round`).
pub fn int_point(x: f64, y: f64) -> Point64 {
    Point64::new(x.round_ties_even() as i64, y.round_ties_even() as i64)
}

fn sub(a: Point64, b: Point64) -> Point64 {
    Point64::new(a.x - b.x, a.y - b.y)
}

fn dot(a: Point64, b: Point64) -> i64 {
    a.x * b.x + a.y * b.y
}

/// C# `IntPoint.LengthSquared()`.
pub fn length_squared(point: Point64) -> i64 {
    point.x * point.x + point.y * point.y
}

/// C# `IntPoint.Length()`: the length truncated to a whole unit.
pub fn length(point: Point64) -> i64 {
    (length_squared(point) as f64).sqrt() as i64
}

/// The orientation of the ordered triplet: 0 collinear, 1 clockwise, 2 counterclockwise.
pub fn orientation(start: Point64, end: Point64, test: Point64) -> i32 {
    let val = (end.y - start.y) * (test.x - end.x) - (end.x - start.x) * (test.y - end.y);
    if val == 0 {
        return 0;
    }
    if val > 0 {
        1
    } else {
        2
    }
}

/// Whether `test_position` lies on the segment `start`-`end`, measured in whole units
/// (C# `IntPoint.OnSegment`).
pub fn on_segment(test_position: Point64, start: Point64, end: Point64) -> bool {
    if start == end {
        return test_position == start;
    }

    let segment_delta = sub(end, start);
    let segment_length = length(segment_delta);
    let point_rel_start = sub(test_position, start);
    let distance_from_start = dot(segment_delta, point_rel_start) / segment_length;

    if distance_from_start >= 0 && distance_from_start <= segment_length {
        let segment_delta_left = Point64::new(-segment_delta.y, segment_delta.x);
        let distance_from_start_left = dot(segment_delta_left, point_rel_start) / segment_length;
        if distance_from_start_left == 0 {
            return true;
        }
    }

    false
}

/// How segment A meets segment B (C# `IntPointExtensions.GetIntersection`).
pub fn get_intersection(
    start_a: Point64,
    end_a: Point64,
    start_b: Point64,
    end_b: Point64,
) -> Intersection {
    let o1 = orientation(start_a, end_a, start_b);
    let o2 = orientation(start_a, end_a, end_b);
    let o3 = orientation(start_b, end_b, start_a);
    let o4 = orientation(start_b, end_b, end_a);

    // The collinear special cases: an end of one segment lies on the other.
    if o1 == 0 && on_segment(start_b, start_a, end_a) {
        return Intersection::Colinear;
    }
    if o2 == 0 && on_segment(end_b, start_a, end_a) {
        return Intersection::Colinear;
    }
    if o3 == 0 && on_segment(start_a, start_b, end_b) {
        return Intersection::Colinear;
    }
    if o4 == 0 && on_segment(end_a, start_b, end_b) {
        return Intersection::Colinear;
    }

    // General case
    if o1 != o2 && o3 != o4 {
        return Intersection::Intersect;
    }

    Intersection::None
}

/// Where the lines through A and B cross, or `None` when they are parallel to within a unit
/// (C# `IntPointExtensions.CalcIntersection`; integer division truncates as in C#).
pub fn calc_intersection(a1: Point64, a2: Point64, b1: Point64, b2: Point64) -> Option<Point64> {
    let intersection_epsilon = 1;
    let num = (a1.y - b1.y) * (b2.x - b1.x) - (a1.x - b1.x) * (b2.y - b1.y);
    let den = (a2.x - a1.x) * (b2.y - b1.y) - (a2.y - a1.y) * (b2.x - b1.x);
    if den.abs() < intersection_epsilon {
        return None;
    }

    Some(Point64::new(
        a1.x + (a2.x - a1.x) * num / den,
        a1.y + (a2.y - a1.y) * num / den,
    ))
}

/// Every edge of the closed `polygon` the segment `start`-`end` meets: the edge's start index,
/// how it meets, and where (C# `Polygon.GetIntersections`). Where the lines are parallel the
/// position is C#'s `default(IntPoint)`, the origin.
pub fn get_intersections(
    polygon: &Path64,
    start: Point64,
    end: Point64,
) -> Vec<(usize, Intersection, Point64)> {
    let mut intersections = Vec::new();
    for i in 0..polygon.len() {
        let edge_start = polygon[i];
        let edge_end = polygon[(i + 1) % polygon.len()];

        let intersection = get_intersection(start, end, edge_start, edge_end);
        if intersection != Intersection::None {
            let position = calc_intersection(start, end, edge_start, edge_end)
                .unwrap_or_else(|| Point64::new(0, 0));
            intersections.push((i, intersection, position));
        }
    }
    intersections
}

/// Parses `"x:1000, y:0,x:0, y:1000,|..."` into polygons, scaling each value
/// (C# `PolygonsExtensions.CreateFromString`). Each value is read after an optional `x:`/`y:`
/// label; a value that is not a number is an error, as C#'s `double.Parse` throws.
pub fn create_from_string(
    polygons_packed_string: &str,
    scale: f64,
) -> Result<Paths64, std::num::ParseFloatError> {
    let mut output = Paths64::new();
    for polygon_string in polygons_packed_string.split('|') {
        let mut poly = Path64::new();
        let int_point_data: Vec<&str> = polygon_string.split(',').collect();
        let mut i = 0;
        while i + 1 < int_point_data.len() {
            poly.push(int_point(
                get_double(int_point_data[i], scale)?,
                get_double(int_point_data[i + 1], scale)?,
            ));
            i += 2;
        }
        if !poly.is_empty() {
            output.push(poly);
        }
    }
    Ok(output)
}

/// C# `PolygonsExtensions.GetDouble`: the number after the `:`, times `scale`.
fn get_double(number_string: &str, scale: f64) -> Result<f64, std::num::ParseFloatError> {
    let value = match number_string.find(':') {
        Some(colon) => &number_string[colon + 1..],
        None => number_string,
    };
    Ok(value.trim().parse::<f64>()? * scale)
}

/// `a.Union(b, fillType)`: `a` as subject, `b` as clip, one union pass.
pub fn union(a: &Paths64, b: &Paths64, fill_rule: FillRule) -> Paths64 {
    union_64(a, b, fill_rule)
}

/// agg-sharp's `Clipper.CleanPolygons`: [`clean_polygon`] on each polygon.
pub fn clean_polygons(polys: &Paths64, distance: f64) -> Paths64 {
    polys
        .iter()
        .map(|poly| clean_polygon(poly, distance))
        .collect()
}

/// agg-sharp's `Clipper.CleanPolygon`: drops vertices within `distance` of a neighbour, then
/// collapses runs of near-collinear vertices - testing each new end against every vertex
/// already accumulated, so a finely tessellated curve is not flattened one triple at a time.
/// A result of fewer than three vertices is empty. The C# linked `OutPt` ring is an index ring
/// here.
pub fn clean_polygon(path: &Path64, distance: f64) -> Path64 {
    let mut cnt = path.len();
    if cnt == 0 {
        return Path64::new();
    }

    let pt: Vec<Point64> = path.clone();
    let mut next: Vec<usize> = (0..cnt).map(|i| (i + 1) % cnt).collect();
    let mut prev: Vec<usize> = (0..cnt).map(|i| (i + cnt - 1) % cnt).collect();
    let mut idx = vec![0u8; cnt];

    let dist_sqrd = distance * distance;
    let mut op = 0usize;
    while idx[op] == 0 && next[op] != prev[op] {
        if points_are_close(pt[op], pt[prev[op]], dist_sqrd) {
            op = exclude_op(op, &mut next, &mut prev, &mut idx);
            cnt -= 1;
        } else if points_are_close(pt[prev[op]], pt[next[op]], dist_sqrd) {
            exclude_op(next[op], &mut next, &mut prev, &mut idx);
            op = exclude_op(op, &mut next, &mut prev, &mut idx);
            cnt -= 2;
        } else {
            idx[op] = 1;
            op = next[op];
        }
    }

    // Collapse collinear runs, checking each new end against everything accumulated so a
    // tessellated curve is not flattened into a long flat.
    let mut remove_points: Vec<usize> = Vec::new();
    let loop_start = op;
    let mut first = true;
    while (first || op != loop_start) && next[op] != prev[op] {
        if slopes_near_collinear(pt[prev[op]], pt[op], pt[next[op]], dist_sqrd) {
            let start = prev[op];
            let first_check = op;
            let mut current_check = first_check;
            let mut end_check = next[op];
            let mut found_end = false;
            while !found_end && end_check != start {
                while current_check != end_check && (first || current_check != loop_start) {
                    let collinear = slopes_near_collinear(
                        pt[start],
                        pt[current_check],
                        pt[end_check],
                        dist_sqrd,
                    );
                    if collinear && end_check != loop_start {
                        if !remove_points.contains(&current_check) {
                            remove_points.push(current_check);
                        }
                        current_check = next[current_check];
                    } else {
                        // add all the points we found and we are done
                        found_end = true;
                        break;
                    }
                }

                if !found_end {
                    current_check = first_check;
                    end_check = next[end_check];
                }
            }

            op = end_check;
        } else {
            op = next[op];
        }

        first = false;
    }

    // remove all the points that were collinear, in the order they were found (a C# HashSet
    // with no removals enumerates in insertion order)
    for remove in remove_points {
        op = exclude_op(remove, &mut next, &mut prev, &mut idx);
        cnt -= 1;
    }

    if cnt < 3 {
        cnt = 0;
    }
    let mut result = Path64::with_capacity(cnt);
    for _ in 0..cnt {
        result.push(pt[op]);
        op = next[op];
    }
    result
}

/// Unlinks `op` from the ring and returns its predecessor (C# `ExcludeOp`).
fn exclude_op(op: usize, next: &mut [usize], prev: &mut [usize], idx: &mut [u8]) -> usize {
    let result = prev[op];
    next[result] = next[op];
    prev[next[op]] = result;
    idx[result] = 0;
    result
}

fn points_are_close(pt1: Point64, pt2: Point64, dist_sqrd: f64) -> bool {
    let dx = pt1.x as f64 - pt2.x as f64;
    let dy = pt1.y as f64 - pt2.y as f64;
    (dx * dx) + (dy * dy) <= dist_sqrd
}

fn distance_from_line_sqrd(pt: Point64, ln1: Point64, ln2: Point64) -> f64 {
    let a = ln1.y as f64 - ln2.y as f64;
    let b = ln2.x as f64 - ln1.x as f64;
    let mut c = a * ln1.x as f64 + b * ln1.y as f64;
    c = a * pt.x as f64 + b * pt.y as f64 - c;
    (c * c) / (a * a + b * b)
}

/// Whether the three points are collinear to within `dist_sqrd`, testing the distance of the
/// point that is geometrically between the other two.
fn slopes_near_collinear(pt1: Point64, pt2: Point64, pt3: Point64, dist_sqrd: f64) -> bool {
    let delta_x = pt1.x as f64 - pt2.x as f64;
    let delta_y = pt1.y as f64 - pt2.y as f64;
    if delta_x.abs() > delta_y.abs() {
        if (pt1.x > pt2.x) == (pt1.x < pt3.x) {
            distance_from_line_sqrd(pt1, pt2, pt3) < dist_sqrd
        } else if (pt2.x > pt1.x) == (pt2.x < pt3.x) {
            distance_from_line_sqrd(pt2, pt1, pt3) < dist_sqrd
        } else {
            distance_from_line_sqrd(pt3, pt1, pt2) < dist_sqrd
        }
    } else if (pt1.y > pt2.y) == (pt1.y < pt3.y) {
        distance_from_line_sqrd(pt1, pt2, pt3) < dist_sqrd
    } else if (pt2.y > pt1.y) == (pt2.y < pt3.y) {
        distance_from_line_sqrd(pt2, pt1, pt3) < dist_sqrd
    } else {
        distance_from_line_sqrd(pt3, pt1, pt2) < dist_sqrd
    }
}

/// Orders by x only, as C#'s `IndexAndPositionSorter`.
pub(crate) fn compare_x(a: &Point64, b: &Point64) -> Ordering {
    a.x.cmp(&b.x)
}
