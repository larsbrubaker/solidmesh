//! The extension methods on a list of ClipperLib polygons (`List<List<IntPoint>>`,
//! clipper2-rust's `Paths64`) that MatterCAD and PolygonMesh call: the booleans (`CombinePolygons`, `Union`,
//! `Subtract`, `Intersect`), the size-aware offsets (`ArcToleranceFor`, `Offset`), the
//! transforms, `CreateFromString`, and the PolyTree walks that split polygons into outlines
//! with their holes. The booleans and offsets run on the Clipper 6 port ([`crate::clipper6`])
//! and return its failures as [`ClipperError`].

use clipper2_rust::{ClipType, FillRule, JoinType, Path64, Paths64, PolyTree64};

use super::clipper_error::ClipperError;
use super::clipper_statics::{execute, execute_tree, offset_closed_polygons};
use super::int_point_extensions::int_point;
use super::polygon_extensions::PolygonExtensions;
use crate::curve_tolerance;
use crate::rectangle_double::RectangleDouble;

/// Clipper's own default arc tolerance, in clipper units - what these helpers always used.
pub const DEFAULT_ARC_TOLERANCE: f64 = 0.25;

/// The clipper units in one unit of the shape: agg converts paths at 1000 (a micron in mm).
pub const DEFAULT_CLIPPER_UNITS_PER_UNIT: f64 = 1000.0;

/// Clipper 6's `ClipperOffset` default miter limit.
pub const DEFAULT_MITER_LIMIT: f64 = 2.0;

/// One polygon of [`PolygonsExtensions::describe_all_polygons`].
#[derive(Clone, Debug, PartialEq)]
pub struct PolygonDescription {
    pub polygon: Path64,
    /// Which top-level branch of the union's tree the polygon is in.
    pub group: i32,
    /// The child count of the outline the polygon belongs to.
    pub count: i32,
    /// The polygon's index within its outline-and-holes run.
    pub path: i32,
    pub depth: i32,
}

/// Helpers on a list of polygons.
pub trait PolygonsExtensions {
    /// `self` as subject, `b` as clip.
    fn combine_polygons(
        &self,
        b: &Paths64,
        clip_type: ClipType,
        fill_rule: FillRule,
    ) -> Result<Paths64, ClipperError>;

    /// The union of every polygon's bounds ([`RectangleDouble::ZERO_INTERSECTION`] when empty).
    fn get_bounds(&self) -> RectangleDouble;

    /// The sum of each polygon's [`PolygonExtensions::length`].
    fn length(&self, are_closed: bool) -> f64;

    /// [`DEFAULT_ARC_TOLERANCE`] at or under
    /// [`curve_tolerance::DESK_SIZE`], grown in proportion above it. No points leaves the
    /// bounds inverted, a negative size, which reads as desk size.
    fn arc_tolerance_for(&self, clipper_units_per_unit: f64) -> f64;

    /// Offset by `distance` clipper units with the arc
    /// tolerance cut by the polygons' own size.
    fn offset(&self, distance: f64, join_type: JoinType) -> Result<Paths64, ClipperError>;

    fn offset_with_arc_tolerance(
        &self,
        distance: f64,
        join_type: JoinType,
        arc_tolerance: f64,
    ) -> Result<Paths64, ClipperError>;

    fn rotate(&self, radians: f64) -> Paths64;

    fn scale(&self, scale_x: f64, scale_y: f64) -> Paths64;

    /// The even-odd union, one set per outline (the outline, its
    /// holes), a hole's islands' sets before their outline's.
    fn separate_polygon_groups(&self) -> Result<Vec<Paths64>, ClipperError>;

    /// Every polygon of the even-odd union, outline then holes.
    fn describe_all_polygons(&self) -> Result<Vec<PolygonDescription>, ClipperError>;

    /// The even-odd union as outlines, each
    /// followed by its holes.
    fn separate_into_outlines_and_contained_holes(&self) -> Result<Vec<Paths64>, ClipperError>;

    /// Even-odd difference.
    fn subtract(&self, other: &Paths64) -> Result<Paths64, ClipperError>;

    fn subtract_polygon(&self, other: &Path64) -> Result<Paths64, ClipperError>;

    fn translate(&self, x: f64, y: f64, scale: f64) -> Paths64;

    /// The union with `other` under `fill_rule`.
    fn union(&self, other: &Paths64, fill_rule: FillRule) -> Result<Paths64, ClipperError>;

    /// Even-odd.
    fn union_polygon(&self, other: &Path64) -> Result<Paths64, ClipperError>;

    /// The intersection with `other` under `fill_rule`.
    fn intersect(&self, other: &Paths64, fill_rule: FillRule) -> Result<Paths64, ClipperError>;
}

impl PolygonsExtensions for Paths64 {
    fn combine_polygons(
        &self,
        b: &Paths64,
        clip_type: ClipType,
        fill_rule: FillRule,
    ) -> Result<Paths64, ClipperError> {
        execute(clip_type, self, b, fill_rule)
    }

    fn get_bounds(&self) -> RectangleDouble {
        let mut bounds = RectangleDouble::ZERO_INTERSECTION;
        for poly in self {
            bounds.expand_to_include_rect(poly.get_bounds());
        }
        bounds
    }

    fn length(&self, are_closed: bool) -> f64 {
        let mut length = 0.0;
        for polygon in self {
            length += polygon.length(are_closed);
        }
        length
    }

    fn arc_tolerance_for(&self, clipper_units_per_unit: f64) -> f64 {
        let bounds = self.get_bounds();
        curve_tolerance::for_size(
            DEFAULT_ARC_TOLERANCE,
            vector_math::ieee_math::max_f64(bounds.width(), bounds.height())
                / clipper_units_per_unit,
        )
    }

    fn offset(&self, distance: f64, join_type: JoinType) -> Result<Paths64, ClipperError> {
        let arc_tolerance = self.arc_tolerance_for(DEFAULT_CLIPPER_UNITS_PER_UNIT);
        self.offset_with_arc_tolerance(distance, join_type, arc_tolerance)
    }

    fn offset_with_arc_tolerance(
        &self,
        distance: f64,
        join_type: JoinType,
        arc_tolerance: f64,
    ) -> Result<Paths64, ClipperError> {
        offset_closed_polygons(
            self,
            distance,
            join_type,
            DEFAULT_MITER_LIMIT,
            arc_tolerance,
        )
    }

    fn rotate(&self, radians: f64) -> Paths64 {
        self.iter().map(|poly| poly.rotate(radians)).collect()
    }

    fn scale(&self, scale_x: f64, scale_y: f64) -> Paths64 {
        self.iter()
            .map(|poly| poly.scale(scale_x, scale_y))
            .collect()
    }

    fn separate_polygon_groups(&self) -> Result<Vec<Paths64>, ClipperError> {
        let poly_tree = union_tree(self)?;
        let mut ret = Vec::new();
        poly_tree_to_polygon_sets(&poly_tree, 0, &mut ret);
        Ok(ret)
    }

    fn describe_all_polygons(&self) -> Result<Vec<PolygonDescription>, ClipperError> {
        let poly_tree = union_tree(self)?;
        let mut output = Vec::new();
        describe_all_polygons_in(&poly_tree, 0, 0, 0, 0, &mut output);
        Ok(output)
    }

    fn separate_into_outlines_and_contained_holes(&self) -> Result<Vec<Paths64>, ClipperError> {
        let poly_tree = union_tree(self)?;
        let mut ret = Vec::new();
        poly_tree_to_outlines_and_contained_holes(&poly_tree, 0, &mut ret);
        Ok(ret)
    }

    fn subtract(&self, other: &Paths64) -> Result<Paths64, ClipperError> {
        self.combine_polygons(other, ClipType::Difference, FillRule::EvenOdd)
    }

    fn subtract_polygon(&self, other: &Path64) -> Result<Paths64, ClipperError> {
        self.subtract(&vec![other.clone()])
    }

    fn translate(&self, x: f64, y: f64, scale: f64) -> Paths64 {
        self.iter()
            .map(|poly| poly.translate(x, y, scale))
            .collect()
    }

    fn union(&self, other: &Paths64, fill_rule: FillRule) -> Result<Paths64, ClipperError> {
        self.combine_polygons(other, ClipType::Union, fill_rule)
    }

    fn union_polygon(&self, other: &Path64) -> Result<Paths64, ClipperError> {
        self.union(&vec![other.clone()], FillRule::EvenOdd)
    }

    fn intersect(&self, other: &Paths64, fill_rule: FillRule) -> Result<Paths64, ClipperError> {
        self.combine_polygons(other, ClipType::Intersection, fill_rule)
    }
}

/// `clipper.AddPaths(polygons, ptSubject, true); clipper.Execute(ctUnion, polyTree)`.
fn union_tree(polygons: &Paths64) -> Result<PolyTree64, ClipperError> {
    execute_tree(
        ClipType::Union,
        polygons,
        &Paths64::new(),
        FillRule::EvenOdd,
    )
}

/// For each child, the child and its children's
/// contours as one set, pushed after the sets of those grandchildren's subtrees.
pub fn poly_tree_to_polygon_sets(poly_tree: &PolyTree64, node: usize, ret: &mut Vec<Paths64>) {
    for &child in poly_tree.nodes[node].children() {
        let mut output_polygons = vec![poly_tree.nodes[child].polygon().clone()];
        for &grandchild in poly_tree.nodes[child].children() {
            output_polygons.push(poly_tree.nodes[grandchild].polygon().clone());
            poly_tree_to_polygon_sets(poly_tree, grandchild, ret);
        }
        ret.push(output_polygons);
    }
}

/// Every polygon under `node`, outline then holes. `group`, `path` and `depth` are passed by
/// value: a child's counting does not reach its siblings. Every node of a closed union is
/// closed, so no node is skipped as open.
pub fn describe_all_polygons_in(
    poly_tree: &PolyTree64,
    node: usize,
    mut group: i32,
    mut path: i32,
    mut depth: i32,
    output: &mut Vec<PolygonDescription>,
) {
    let poly_node = &poly_tree.nodes[node];
    let child_count = poly_node.children().len() as i32;

    // if this is not a hole
    if !poly_node.polygon().is_empty() && !poly_tree.is_hole(node) {
        output.push(PolygonDescription {
            polygon: poly_node.polygon().clone(),
            group,
            count: child_count,
            path,
            depth,
        });
        path += 1;

        if child_count > 0 {
            depth += 1;
        }

        // add all the children that are holes as holes
        for &child in poly_node.children() {
            let child_polygon = poly_tree.nodes[child].polygon();
            if !child_polygon.is_empty() && poly_tree.is_hole(child) {
                output.push(PolygonDescription {
                    polygon: child_polygon.clone(),
                    group,
                    count: child_count,
                    path,
                    depth,
                });
                path += 1;
            }
        }
    }

    for &child in poly_node.children() {
        describe_all_polygons_in(poly_tree, child, group, path, depth, output);
        group += 1;
    }
}

/// Each outline (a non-hole node) with its hole
/// children, outlines in tree order.
pub fn poly_tree_to_outlines_and_contained_holes(
    poly_tree: &PolyTree64,
    node: usize,
    outlines_and_holes: &mut Vec<Paths64>,
) {
    let poly_node = &poly_tree.nodes[node];

    // if this is not a hole
    if !poly_node.polygon().is_empty() && !poly_tree.is_hole(node) {
        // create a new shape for the polygon data
        let mut outline_and_holes = vec![poly_node.polygon().clone()];

        // add all the children that are holes as holes
        for &child in poly_node.children() {
            let child_polygon = poly_tree.nodes[child].polygon();
            if !child_polygon.is_empty() && poly_tree.is_hole(child) {
                outline_and_holes.push(child_polygon.clone());
            }
        }

        outlines_and_holes.push(outline_and_holes);
    }

    for &child in poly_node.children() {
        poly_tree_to_outlines_and_contained_holes(poly_tree, child, outlines_and_holes);
    }
}

/// Parses `"x:1000, y:0,x:0, y:1000,|..."` into polygons, scaling each value.
/// Each value is read after an optional `x:`/`y:` label; a value that is not a number is an
/// error rather than a silent zero.
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

/// The number after the `:`, times `scale`.
fn get_double(number_string: &str, scale: f64) -> Result<f64, std::num::ParseFloatError> {
    let value = match number_string.find(':') {
        Some(colon) => &number_string[colon + 1..],
        None => number_string,
    };
    Ok(value.trim().parse::<f64>()? * scale)
}
