//! The static `Clipper` members MatterCAD calls, from a modified Clipper 6.4.2 (Boost Software
//! License 1.0), and the `new Clipper()` +
//! `Execute` pattern the extension methods share. Everything runs on the Clipper 6 port in
//! [`crate::clipper6`]; this module adapts clipper2-rust's types (`Path64`, `FillRule`,
//! `ClipType`, `PolyTree64`), which agg-path's polygon API speaks, to it. `CleanPolygon(s)` is
//! in [`mod@super::clean_polygon`].

use clipper2_rust::{ClipType, FillRule, JoinType, Path64, Paths64, PolyTree64, Rect64};

use super::clipper_error::ClipperError;
use crate::clipper6;

/// Clipper2's fill rule as Clipper 6's `PolyFillType` (the same four rules).
pub fn poly_fill_type(fill_rule: FillRule) -> clipper6::PolyFillType {
    match fill_rule {
        FillRule::EvenOdd => clipper6::PolyFillType::EvenOdd,
        FillRule::NonZero => clipper6::PolyFillType::NonZero,
        FillRule::Positive => clipper6::PolyFillType::Positive,
        FillRule::Negative => clipper6::PolyFillType::Negative,
    }
}

/// Clipper2's join type as Clipper 6's. Clipper 6 has no bevel join; its square join (the
/// corner cut off square to the bisector) is the nearest, and no caller asks for a bevel.
pub fn clipper6_join_type(join_type: JoinType) -> clipper6::JoinType {
    match join_type {
        JoinType::Square | JoinType::Bevel => clipper6::JoinType::Square,
        JoinType::Round => clipper6::JoinType::Round,
        JoinType::Miter => clipper6::JoinType::Miter,
    }
}

/// Clipper2's clip type as Clipper 6's; `None` for Clipper2's `NoClip`, which clips nothing.
fn clipper6_clip_type(clip_type: ClipType) -> Option<clipper6::ClipType> {
    match clip_type {
        ClipType::NoClip => None,
        ClipType::Intersection => Some(clipper6::ClipType::Intersection),
        ClipType::Union => Some(clipper6::ClipType::Union),
        ClipType::Difference => Some(clipper6::ClipType::Difference),
        ClipType::Xor => Some(clipper6::ClipType::Xor),
    }
}

/// `new Clipper()` with `subjects` and `clips` added as closed paths. Clipper 6 throws for
/// coordinates beyond +/-2^62 (no flattened path reaches them: a 1000-units-per-mm grid would
/// be 4.6 billion km across); that comes back as the error.
fn new_clipper(subjects: &Paths64, clips: &Paths64) -> Result<clipper6::Clipper, ClipperError> {
    let mut clipper = clipper6::Clipper::new();
    clipper.add_paths(subjects, clipper6::PolyType::Subject, true)?;
    clipper.add_paths(clips, clipper6::PolyType::Clip, true)?;
    Ok(clipper)
}

/// `clipper.AddPaths(subjects, ptSubject, true); clipper.AddPaths(clips, ptClip, true);
/// clipper.Execute(clipType, solution, fillType)`. Clipper2's `NoClip` (Clipper 6 has no
/// counterpart) clips nothing and gives an empty solution.
pub fn execute(
    clip_type: ClipType,
    subjects: &Paths64,
    clips: &Paths64,
    fill_rule: FillRule,
) -> Result<Paths64, ClipperError> {
    let mut solution = Paths64::new();
    let Some(clip_type) = clipper6_clip_type(clip_type) else {
        return Ok(solution);
    };
    let mut clipper = new_clipper(subjects, clips)?;
    let fill = poly_fill_type(fill_rule);
    clipper.execute(clip_type, &mut solution, fill, fill)?;
    Ok(solution)
}

/// As [`execute`], into a `PolyTree`: Clipper 6's tree, copied node for node (children in
/// Clipper 6's order) into clipper2-rust's `PolyTree64`.
pub fn execute_tree(
    clip_type: ClipType,
    subjects: &Paths64,
    clips: &Paths64,
    fill_rule: FillRule,
) -> Result<PolyTree64, ClipperError> {
    let mut tree = clipper6::PolyTree::new();
    if let Some(clip_type) = clipper6_clip_type(clip_type) {
        let mut clipper = new_clipper(subjects, clips)?;
        let fill = poly_fill_type(fill_rule);
        clipper.execute_tree(clip_type, &mut tree, fill, fill)?;
    }
    let mut result = PolyTree64::new();
    copy_children(&tree, clipper6::PolyTree::ROOT, &mut result, 0);
    Ok(result)
}

fn copy_children(tree: &clipper6::PolyTree, node: usize, result: &mut PolyTree64, parent: usize) {
    for &child in tree.childs(node) {
        let index = result.add_child(parent, tree.contour(child).clone());
        copy_children(tree, child, result, index);
    }
}

/// `new ClipperOffset(miterLimit, arcTolerance)`, `AddPaths(paths, joinType,
/// etClosedPolygon)`, `Execute(ref solution, delta)`. As with [`execute`], Clipper 6's
/// exceptions come back as the error.
pub fn offset_closed_polygons(
    paths: &Paths64,
    delta: f64,
    join_type: JoinType,
    miter_limit: f64,
    arc_tolerance: f64,
) -> Result<Paths64, ClipperError> {
    let mut offseter = clipper6::ClipperOffset::new(miter_limit, arc_tolerance);
    offseter.add_paths(
        paths,
        clipper6_join_type(join_type),
        clipper6::EndType::ClosedPolygon,
    );
    let mut solution = Paths64::new();
    offseter.execute(&mut solution, delta)?;
    Ok(solution)
}

/// The signed area, positive when the polygon is counter-clockwise (y up).
/// Fewer than three points is 0.
pub fn area(poly: &Path64) -> f64 {
    clipper6::area(poly)
}

/// True when [`area`] is not negative.
pub fn orientation(poly: &Path64) -> bool {
    clipper6::orientation(poly)
}

/// 0 outside, +1 inside, -1 on the boundary (Hormann & Agathos,
/// "The Point in Polygon Problem for Arbitrary Polygons").
pub fn point_in_polygon(pt: clipper2_rust::Point64, path: &Path64) -> i32 {
    clipper6::point_in_polygon(pt, path)
}

/// Clipper's `IntRect` (`top` is the smallest y), all zero when
/// every path is empty.
pub fn get_bounds(paths: &Paths64) -> Rect64 {
    let r = clipper6::get_bounds(paths);
    Rect64 {
        left: r.left,
        top: r.top,
        right: r.right,
        bottom: r.bottom,
    }
}

/// The polygon unioned with itself under
/// `fill_rule` with `StrictlySimple` set, which splits a self-intersecting polygon, or one that
/// touches itself at a vertex, into simple ones.
pub fn simplify_polygon(poly: &Path64, fill_rule: FillRule) -> Result<Paths64, ClipperError> {
    clipper6::simplify_polygon(poly, poly_fill_type(fill_rule)).map_err(ClipperError)
}

/// As [`simplify_polygon`] for many.
pub fn simplify_polygons(polys: &Paths64, fill_rule: FillRule) -> Result<Paths64, ClipperError> {
    clipper6::simplify_polygons(polys, poly_fill_type(fill_rule)).map_err(ClipperError)
}

/// Every non-empty contour, parents before children.
pub fn closed_paths_from_poly_tree(poly_tree: &PolyTree64) -> Paths64 {
    let mut result = Paths64::new();
    add_node_paths(poly_tree, 0, &mut result);
    result
}

/// Every non-empty contour under `node` (every node of a closed clip is closed).
fn add_node_paths(poly_tree: &PolyTree64, node: usize, paths: &mut Paths64) {
    let polygon = poly_tree.nodes[node].polygon();
    if !polygon.is_empty() {
        paths.push(polygon.clone());
    }
    for &child in poly_tree.nodes[node].children() {
        add_node_paths(poly_tree, child, paths);
    }
}
