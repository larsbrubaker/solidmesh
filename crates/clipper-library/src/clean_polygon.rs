//! A modified Clipper 6.4.2 `Clipper.CleanPolygon(s)` (Boost Software License 1.0), which
//! Clipper2 does not have; the other `Clipper` statics MatterCAD calls are in [`super::clipper_statics`].

use clipper2_rust::{Path64, Paths64, Point64};

/// The usual clean distance, just over sqrt(2) so diagonal grid neighbours merge: points closer
/// than this merge.
pub const DEFAULT_CLEAN_DISTANCE: f64 = 1.415;

/// [`clean_polygon`] on each polygon.
pub fn clean_polygons(polys: &Paths64, distance: f64) -> Paths64 {
    polys
        .iter()
        .map(|poly| clean_polygon(poly, distance))
        .collect()
}

/// Drops vertices within `distance` of a neighbour, then
/// collapses runs of near-collinear vertices - testing each new end against every vertex
/// already accumulated, so a finely tessellated curve is not flattened one triple at a time.
/// A result of fewer than three vertices is empty. The ring is linked by `next`/`prev` index
/// arrays.
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

    // remove all the points that were collinear, in the order they were found, so the ring's
    // start point is deterministic
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

/// Unlinks `op` from the ring and returns its predecessor.
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
