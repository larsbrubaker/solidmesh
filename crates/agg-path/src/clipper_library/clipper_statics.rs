//! The static `Clipper` members MatterCAD calls, from agg-sharp's modified Clipper 6.4.2
//! (`clipper_library/clipper.cs`, Boost Software License 1.0): `Area`, `Orientation`,
//! `PointInPolygon`, `GetBounds`, `SimplifyPolygon(s)`, `ClosedPathsFromPolyTree`, and the
//! `new Clipper()` + `Execute` pattern the extension methods share. The arithmetic ones are
//! ported from Clipper 6 line for line, so their results match the C# exactly; the boolean ones
//! run on clipper2-rust (docs/CSHARP_DIVERGENCES.md 220). `CleanPolygon(s)` is in
//! [`super::clean_polygon`].

use clipper2_rust::{
    poly_tree_to_paths64, ClipType, Clipper64, FillRule, Path64, Paths64, PolyTree64, Rect64,
};

/// `new Clipper()`: Clipper 6 drops collinear vertices from its output unless asked
/// (`PreserveCollinear` defaults to false), where Clipper2 keeps them by default.
pub fn new_clipper() -> Clipper64 {
    let mut clipper = Clipper64::new();
    clipper.set_preserve_collinear(false);
    clipper
}

/// `clipper.AddPaths(subjects, ptSubject, true); clipper.AddPaths(clips, ptClip, true);
/// clipper.Execute(clipType, solution, fillType)`.
pub fn execute(
    clip_type: ClipType,
    subjects: &Paths64,
    clips: &Paths64,
    fill_rule: FillRule,
) -> Paths64 {
    let mut clipper = new_clipper();
    clipper.add_subject(subjects);
    clipper.add_clip(clips);
    let mut solution = Paths64::new();
    clipper.execute(clip_type, fill_rule, &mut solution, None);
    solution
}

/// As [`execute`], into a `PolyTree`.
pub fn execute_tree(
    clip_type: ClipType,
    subjects: &Paths64,
    clips: &Paths64,
    fill_rule: FillRule,
) -> PolyTree64 {
    let mut clipper = new_clipper();
    clipper.add_subject(subjects);
    clipper.add_clip(clips);
    let mut tree = PolyTree64::new();
    let mut open = Paths64::new();
    clipper.execute_tree(clip_type, fill_rule, &mut tree, &mut open);
    tree
}

/// C# `Clipper.Area`: the signed area, positive when the polygon is counter-clockwise (y up).
/// Fewer than three points is 0.
pub fn area(poly: &Path64) -> f64 {
    let cnt = poly.len();
    if cnt < 3 {
        return 0.0;
    }
    let mut a = 0.0;
    let mut j = cnt - 1;
    for i in 0..cnt {
        a += (poly[j].x as f64 + poly[i].x as f64) * (poly[j].y as f64 - poly[i].y as f64);
        j = i;
    }
    -a * 0.5
}

/// C# `Clipper.Orientation`: true when [`area`] is not negative.
pub fn orientation(poly: &Path64) -> bool {
    area(poly) >= 0.0
}

/// C# `Clipper.PointInPolygon`: 0 outside, +1 inside, -1 on the boundary (Hormann & Agathos,
/// "The Point in Polygon Problem for Arbitrary Polygons").
pub fn point_in_polygon(pt: clipper2_rust::Point64, path: &Path64) -> i32 {
    let mut result = 0;
    let cnt = path.len();
    if cnt < 3 {
        return 0;
    }
    let mut ip = path[0];
    for i in 1..=cnt {
        let ip_next = if i == cnt { path[0] } else { path[i] };
        if ip_next.y == pt.y
            && (ip_next.x == pt.x || (ip.y == pt.y && ((ip_next.x > pt.x) == (ip.x < pt.x))))
        {
            return -1;
        }
        if (ip.y < pt.y) != (ip_next.y < pt.y) {
            if ip.x >= pt.x {
                if ip_next.x > pt.x {
                    result = 1 - result;
                } else {
                    let d = (ip.x - pt.x) as f64 * (ip_next.y - pt.y) as f64
                        - (ip_next.x - pt.x) as f64 * (ip.y - pt.y) as f64;
                    if d == 0.0 {
                        return -1;
                    } else if (d > 0.0) == (ip_next.y > ip.y) {
                        result = 1 - result;
                    }
                }
            } else if ip_next.x > pt.x {
                let d = (ip.x - pt.x) as f64 * (ip_next.y - pt.y) as f64
                    - (ip_next.x - pt.x) as f64 * (ip.y - pt.y) as f64;
                if d == 0.0 {
                    return -1;
                } else if (d > 0.0) == (ip_next.y > ip.y) {
                    result = 1 - result;
                }
            }
        }
        ip = ip_next;
    }
    result
}

/// C# `Clipper.GetBounds(Paths)`: Clipper's `IntRect` (`top` is the smallest y), all zero when
/// every path is empty.
pub fn get_bounds(paths: &Paths64) -> Rect64 {
    let mut points = paths.iter().flatten();
    let Some(first) = points.next() else {
        return Rect64 {
            left: 0,
            top: 0,
            right: 0,
            bottom: 0,
        };
    };
    let mut result = Rect64 {
        left: first.x,
        top: first.y,
        right: first.x,
        bottom: first.y,
    };
    for point in points {
        if point.x < result.left {
            result.left = point.x;
        } else if point.x > result.right {
            result.right = point.x;
        }
        if point.y < result.top {
            result.top = point.y;
        } else if point.y > result.bottom {
            result.bottom = point.y;
        }
    }
    result
}

/// C# `Clipper.SimplifyPolygon(poly, fillType)`: the polygon unioned with itself under
/// `fill_rule`, which splits a self-intersecting polygon into simple ones. Clipper 6 also sets
/// `StrictlySimple`, which Clipper2 does not have (docs/CSHARP_DIVERGENCES.md 221).
pub fn simplify_polygon(poly: &Path64, fill_rule: FillRule) -> Paths64 {
    simplify_polygons(&vec![poly.clone()], fill_rule)
}

/// C# `Clipper.SimplifyPolygons(polys, fillType)`: as [`simplify_polygon`] for many.
pub fn simplify_polygons(polys: &Paths64, fill_rule: FillRule) -> Paths64 {
    execute(ClipType::Union, polys, &Paths64::new(), fill_rule)
}

/// C# `Clipper.ClosedPathsFromPolyTree`: every non-empty contour, parents before children.
pub fn closed_paths_from_poly_tree(poly_tree: &PolyTree64) -> Paths64 {
    poly_tree_to_paths64(poly_tree)
}
