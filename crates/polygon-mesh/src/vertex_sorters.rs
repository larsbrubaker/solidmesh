//! Orderings of f64 positions used to find nearby vertices by binary search: along X, along
//! all three axes after a rotation, and by distance from a point.
//!
//! The distance sorter cannot find nearby vertices (positions at about the same distance from
//! its point can be far apart), so [`VertexSorter`] is just the ordering and
//! `find_vertices` is an inherent method on the two sorters that support it.

use std::cmp::Ordering;

use vector_math::{Matrix4X4, Vector3};

/// An ordering of positions.
pub trait VertexSorter {
    /// Negative, zero or positive as `a` sorts before, with or after `b`.
    fn compare(&self, a: Vector3, b: Vector3) -> i32;
}

/// The index of a match, or the bitwise complement of the insertion point. The midpoint is
/// `lo + (hi - lo) / 2`, a fixed rule, so among equal entries the same one is always found.
fn binary_search(vertices: &[Vector3], value: Vector3, sorter: &dyn VertexSorter) -> i32 {
    let mut lo: i32 = 0;
    let mut hi: i32 = vertices.len() as i32 - 1;
    while lo <= hi {
        let i = lo + ((hi - lo) >> 1);
        let order = sorter.compare(vertices[i as usize], value);
        if order == 0 {
            return i;
        }
        if order < 0 {
            lo = i + 1;
        } else {
            hi = i - 1;
        }
    }
    !lo
}

/// A total order on doubles as an i32 (NaN sorts first, so sorting never fails on one).
fn compare_f64(a: f64, b: f64) -> i32 {
    match a.partial_cmp(&b) {
        Some(Ordering::Less) => -1,
        Some(Ordering::Greater) => 1,
        Some(Ordering::Equal) => 0,
        None => match (a.is_nan(), b.is_nan()) {
            (true, true) => 0,
            (true, false) => -1,
            _ => 1,
        },
    }
}

fn add_to_list_if_same_enough(
    vertices: &[Vector3],
    position: Vector3,
    find_list: &mut Vec<Vector3>,
    max_distance_squared: f64,
    i: usize,
) {
    if vertices[i] == position || (vertices[i] - position).length_squared() <= max_distance_squared
    {
        find_list.push(vertices[i]);
    }
}

/// The vertices of `vertices` (sorted by `sorter`) within `max_distance` of `position`: walk up
/// from the binary-search index, then down, stopping each way once `key` (the sort axis) is
/// further than `max_distance` away.
fn find_vertices_along(
    sorter: &dyn VertexSorter,
    key: impl Fn(Vector3) -> f64,
    vertices: &[Vector3],
    position: Vector3,
    max_distance: f64,
) -> Vec<Vector3> {
    let mut found_vertices = Vec::new();
    let mut index = binary_search(vertices, position, sorter);
    if index < 0 {
        index = !index;
    }
    let index = index as usize;

    // we have the starting index now get all the vertices that are close enough from here
    let max_distance_squared = max_distance * max_distance;
    let position_key = key(position);
    for i in index..vertices.len() {
        if (key(vertices[i]) - position_key).abs() > max_distance {
            // we are too far away on the axis, we are done with this direction
            break;
        }
        add_to_list_if_same_enough(
            vertices,
            position,
            &mut found_vertices,
            max_distance_squared,
            i,
        );
    }
    for i in (0..index).rev() {
        if (key(vertices[i]) - position_key).abs() > max_distance {
            break;
        }
        add_to_list_if_same_enough(
            vertices,
            position,
            &mut found_vertices,
            max_distance_squared,
            i,
        );
    }
    found_vertices
}

/// Orders positions by X.
#[derive(Debug, Clone, Copy, Default)]
pub struct VertexXAxisSorter;

impl VertexSorter for VertexXAxisSorter {
    fn compare(&self, a: Vector3, b: Vector3) -> i32 {
        compare_f64(a.x, b.x)
    }
}

impl VertexXAxisSorter {
    /// The vertices (sorted by this sorter) within `max_distance_to_consider_vertex_as_same`
    /// of `position`.
    pub fn find_vertices(
        &self,
        vertices: &[Vector3],
        position: Vector3,
        max_distance_to_consider_vertex_as_same: f64,
    ) -> Vec<Vector3> {
        find_vertices_along(
            self,
            |v| v.x,
            vertices,
            position,
            max_distance_to_consider_vertex_as_same,
        )
    }
}

/// Orders positions by X, then Y, then Z after a fixed rotation, so axis-aligned geometry does
/// not pile up on equal keys.
#[derive(Debug, Clone, Copy)]
pub struct VertexXYZAxisWithRotation {
    rotation_to_use: Matrix4X4,
}

impl Default for VertexXYZAxisWithRotation {
    fn default() -> Self {
        VertexXYZAxisWithRotation {
            rotation_to_use: Matrix4X4::create_rotation(Vector3::new(0.224374, 0.805696, 0.383724)),
        }
    }
}

impl VertexXYZAxisWithRotation {
    fn transform_vector(&self, vec: Vector3) -> Vector3 {
        let mat = &self.rotation_to_use;
        Vector3::new(
            vec.x * mat.row0.x + vec.y * mat.row1.x + vec.z * mat.row2.x,
            vec.x * mat.row0.y + vec.y * mat.row1.y + vec.z * mat.row2.y,
            vec.x * mat.row0.z + vec.y * mat.row1.z + vec.z * mat.row2.z,
        )
    }

    /// The vertices (sorted by this sorter) within `max_distance_to_consider_vertex_as_same`
    /// of `position`.
    pub fn find_vertices(
        &self,
        vertices: &[Vector3],
        position: Vector3,
        max_distance_to_consider_vertex_as_same: f64,
    ) -> Vec<Vector3> {
        find_vertices_along(
            self,
            |v| self.transform_vector(v).x,
            vertices,
            position,
            max_distance_to_consider_vertex_as_same,
        )
    }
}

impl VertexSorter for VertexXYZAxisWithRotation {
    fn compare(&self, a_vertex: Vector3, b_vertex: Vector3) -> i32 {
        let a = self.transform_vector(a_vertex);
        let b = self.transform_vector(b_vertex);
        // Anything not "less" and not "equal" (NaN included) is 1.
        let step = |a: f64, b: f64| {
            if a < b {
                Some(-1)
            } else if a == b {
                None
            } else {
                Some(1)
            }
        };
        step(a.x, b.x)
            .or_else(|| step(a.y, b.y))
            .or_else(|| step(a.z, b.z))
            .unwrap_or(0)
    }
}

/// Orders positions by their distance from a fixed, arbitrary point.
#[derive(Debug, Clone, Copy, Default)]
pub struct VertexDistanceFromPointSorter;

impl VertexSorter for VertexDistanceFromPointSorter {
    fn compare(&self, a: Vector3, b: Vector3) -> i32 {
        let position_to_measure_from = Vector3::new(0.224374, 0.805696, 0.383724);
        let dist_to_a_squared = (a - position_to_measure_from).length_squared();
        let dist_to_b_squared = (b - position_to_measure_from).length_squared();
        compare_f64(dist_to_a_squared, dist_to_b_squared)
    }
}
