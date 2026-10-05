//! Slicing a mesh by a plane into closed integer polygons. Port of agg-sharp `Csg/SliceLayer.cs`
//! with its helpers `Csg/Segment.cs`, `Csg/SortedIntPoint.cs` and `Csg/IndexAndPosition*.cs`;
//! ClipperLib's `IntPoint` polygons are clipper2-rust's `Point64`/`Path64`/`Paths64`.
//!
//! Not here: the `IBvhItem` accelerator overloads of `CreateSlice`/`GetUnorderdSegments` (and
//! their `includeBehindThePlane` filter, which only the accelerated path reads) join with the
//! ray tracer (2d). `IntPointPolygon(s)Extensions.AsVertices` turn polygons back into agg-path
//! vertices.

use std::collections::HashMap;

use agg_path::shape_path::FlagsAndCommand;
use agg_path::vertex_data::VertexData;
use clipper2_rust::{FillRule, Path64, Paths64, Point64};
use vector_math::cs_sort::introspective_sort;
use vector_math::{Matrix4X4, Plane, Quaternion, Vector2, Vector3};

use crate::clipper_extensions::{self, compare_x, int_point, length, length_squared};
use crate::Mesh;

/// One cut through a face, in the plane's scaled XY space (C# `Segment`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Segment {
    pub start: Point64,
    pub end: Point64,
}

impl Segment {
    pub fn new(start: Point64, end: Point64) -> Self {
        Self { start, end }
    }
}

/// The slice of `mesh` (placed by `matrix`) at z = 0 of world space, as polygons in XY
/// (C# `GetPolygonXYLoopsAt0`; its `outputScale` parameter is unused there and is not carried).
pub fn get_polygon_xy_loops_at_0(mesh: &Mesh, matrix: Matrix4X4) -> Paths64 {
    let slice_plane = Plane::new(Vector3::UNIT_Z, 0.0);

    // transform our plane to the mesh
    let to_mesh_matrix = matrix.inverted();
    let plane_in_mesh_space = Plane::from_normal_and_point(
        slice_plane.normal.transform_normal(to_mesh_matrix),
        (slice_plane.normal * slice_plane.distance_from_origin).transform(to_mesh_matrix),
    );

    create_slice(mesh, plane_in_mesh_space, 1000)
}

/// The closed polygons where `plane` cuts `mesh`, scaled by `output_scale` into integer XY
/// (C# `CreateSlice(mesh, plane, outputScale)`).
pub fn create_slice(mesh: &Mesh, plane: Plane, output_scale: i32) -> Paths64 {
    let plane_transform_to_xy = get_transform_to_xy(plane, output_scale);
    create_slice_with_transform(mesh, plane, plane_transform_to_xy)
}

/// [`create_slice`] with the plane-to-XY transform supplied.
pub fn create_slice_with_transform(
    mesh: &Mesh,
    plane: Plane,
    plane_transform_to_xy: Matrix4X4,
) -> Paths64 {
    let unordered_segments =
        get_unorderd_segments_with_transform(mesh, plane, plane_transform_to_xy);

    // connect all the segments together into polygons
    find_closed_polygons(&unordered_segments)
}

/// Unions the polygons, non-zero, the first as subject and the rest as clip
/// (C# `UnionClosedPolygons`).
pub fn union_closed_polygons(closed_polygons: Paths64) -> Paths64 {
    if closed_polygons.len() > 1 {
        return clipper_extensions::union(
            &closed_polygons[0..1].to_vec(),
            &closed_polygons[1..].to_vec(),
            FillRule::NonZero,
        );
    }

    closed_polygons
}

/// The transform from the plane to a polygon at z = 0, scaled up by `output_scale`
/// (C# `GetTransformToXy`).
pub fn get_transform_to_xy(plane: Plane, output_scale: i32) -> Matrix4X4 {
    let flattened_matrix = get_flattened_matrix(plane);
    flattened_matrix * Matrix4X4::create_scale_uniform(f64::from(output_scale))
}

/// The rotation taking the plane's normal to +Z, then the shift putting the plane at z = 0
/// (C# `GetFlattenedMatrix`).
pub fn get_flattened_matrix(cut_plane: Plane) -> Matrix4X4 {
    let rotation = Quaternion::from_directions(cut_plane.normal, Vector3::UNIT_Z);
    let mut flattened_matrix = Matrix4X4::create_rotation_quaternion(rotation);
    flattened_matrix =
        flattened_matrix * Matrix4X4::create_translation(0.0, 0.0, -cut_plane.distance_from_origin);
    flattened_matrix
}

/// Every face's cut by `plane`, in the default (x1000) XY space (C# `GetUnorderdSegments`).
pub fn get_unorderd_segments(mesh: &Mesh, plane: Plane) -> Vec<Segment> {
    get_unorderd_segments_with_transform(mesh, plane, get_transform_to_xy(plane, 1000))
}

/// Every face's cut by `plane`, carried into z = 0 space by `mesh_to_0_plane`.
pub fn get_unorderd_segments_with_transform(
    mesh: &Mesh,
    plane: Plane,
    mesh_to_0_plane: Matrix4X4,
) -> Vec<Segment> {
    let mut unordered_segments = Vec::new();
    for face in mesh.faces.iter() {
        if let Some((start, end)) = face.get_cut_line(&mesh.vertices, plane) {
            let start_at_z0 = start.transform(mesh_to_0_plane);
            let end_at_z0 = end.transform(mesh_to_0_plane);
            unordered_segments.push(Segment::new(
                int_point(start_at_z0.x, start_at_z0.y),
                int_point(end_at_z0.x, end_at_z0.y),
            ));
        }
    }

    unordered_segments
}

/// Chains the segments into polygons: exact end-to-start joins first, then the open chains are
/// linked closing the smallest gaps first; slivers under .01 perimeter are dropped and the
/// result is cleaned at 1 unit (C# `FindClosedPolygons`).
pub fn find_closed_polygons(unordered_segments: &[Segment]) -> Paths64 {
    let start_indexes = create_fast_index_lookup(unordered_segments);

    let mut segment_has_been_added = vec![false; unordered_segments.len()];

    let mut open_polygon_list: Vec<Path64> = Vec::new();
    let mut closed_polygons: Paths64 = Vec::new();

    for starting_segment_index in 0..unordered_segments.len() {
        if segment_has_been_added[starting_segment_index] {
            continue;
        }

        let mut poly = Path64::new();
        // We start by adding the start, as we will add ends from now on.
        let polygon_start_position = unordered_segments[starting_segment_index].start;
        poly.push(polygon_start_position);

        let mut segment_index_being_added = starting_segment_index as i32;
        let mut can_close;

        loop {
            can_close = false;
            segment_has_been_added[segment_index_being_added as usize] = true;
            let added_segment_end_point =
                unordered_segments[segment_index_being_added as usize].end;

            poly.push(added_segment_end_point);
            segment_index_being_added = get_touching_segment_index(
                unordered_segments,
                &start_indexes,
                &segment_has_been_added,
                added_segment_end_point,
            );
            if segment_index_being_added == -1 {
                // if we have looped back around to where we started
                if added_segment_end_point == polygon_start_position {
                    can_close = true;
                }

                break;
            }
            // C# also sets canClose here when the found segment starts back at the polygon's
            // start, but the next pass through the loop always resets it first, so it is not
            // carried.
        }

        if can_close {
            closed_polygons.push(poly);
        } else {
            open_polygon_list.push(poly);
        }
    }

    // Remove all polygons from the open polygon list that have 0 points or all the same point
    open_polygon_list.retain(|poly| match poly.first() {
        None => false,
        Some(first) => poly.iter().any(|point| point != first),
    });

    let mut start_sorter = SortedIntPoint::default();
    for (i, poly) in open_polygon_list.iter().enumerate() {
        start_sorter.add(i as i32, poly[0]);
    }
    start_sorter.sort();

    let mut end_sorter = SortedIntPoint::default();
    for (i, poly) in open_polygon_list.iter().enumerate() {
        end_sorter.add(i as i32, poly[poly.len() - 1]);
    }
    end_sorter.sort();

    // Link up all the missing ends, closing up the smallest gaps first. This is an inefficient
    // implementation which can run in O(n*n*n) time.
    loop {
        let mut best_score = f64::MAX;
        let mut best_a = -1i32;
        let mut best_b = -1i32;
        let mut reversed = false;
        for (polygon_a_index, a_polygon) in open_polygon_list.iter().enumerate() {
            if a_polygon.is_empty() {
                continue;
            }

            let a_end_position = a_polygon[a_polygon.len() - 1];
            // find the closestStartFromEnd
            let (b_start_index, distance_to_start_sqrd) =
                start_sorter.find_closet_index(a_end_position, -1);
            if distance_to_start_sqrd < best_score {
                best_score = distance_to_start_sqrd;
                best_a = polygon_a_index as i32;
                best_b = b_start_index;
                reversed = false;

                if best_score == 0.0 {
                    // found a perfect match stop looking
                    break;
                }
            }

            // find the closestStartFromStart
            let (b_end_index, distance_to_end_sqrd) =
                end_sorter.find_closet_index(a_end_position, polygon_a_index as i32);
            if distance_to_end_sqrd < best_score {
                best_score = distance_to_end_sqrd;
                best_a = polygon_a_index as i32;
                best_b = b_end_index;
                reversed = true;

                if best_score == 0.0 {
                    // found a perfect match stop looking
                    break;
                }
            }

            if best_score == 0.0 {
                // found a perfect match stop looking
                break;
            }
        }

        if best_score >= f64::MAX {
            // we could not find any points to connect this to
            break;
        }

        let (a, b) = (best_a as usize, best_b as usize);
        if a == b {
            // This loop connects to itself, close the polygon.
            closed_polygons.push(open_polygon_list[a].clone());
            open_polygon_list[a].clear(); // B is cleared as it is A
            end_sorter.remove(best_a);
            start_sorter.remove(best_a);
        } else if reversed {
            if open_polygon_list[a].len() > open_polygon_list[b].len() {
                let reversed_b: Path64 = open_polygon_list[b].iter().rev().copied().collect();
                open_polygon_list[a].extend(reversed_b);
                open_polygon_list[b].clear();
                end_sorter.remove(best_b);
                start_sorter.remove(best_b);
            } else {
                let reversed_a: Path64 = open_polygon_list[a].iter().rev().copied().collect();
                open_polygon_list[b].extend(reversed_a);
                open_polygon_list[a].clear();
                end_sorter.remove(best_a);
                start_sorter.remove(best_a);
            }
        } else {
            let polygon_b = std::mem::take(&mut open_polygon_list[b]);
            open_polygon_list[a].extend(polygon_b);
            end_sorter.remove(best_b);
            start_sorter.remove(best_b);
        }
    }

    let minimum_perimeter = 0.01;
    closed_polygons.retain(|polygon| {
        let mut perimeter_length = 0.0;
        for int_point_index in 1..polygon.len() {
            let delta = Point64::new(
                polygon[int_point_index].x - polygon[int_point_index - 1].x,
                polygon[int_point_index].y - polygon[int_point_index - 1].y,
            );
            perimeter_length += length(delta) as f64;
            if perimeter_length > minimum_perimeter {
                break;
            }
        }
        perimeter_length >= minimum_perimeter
    });

    clipper_extensions::clean_polygons(&closed_polygons, 1.0)
}

/// The segments by exact start position (C# `CreateFastIndexLookup`).
pub fn create_fast_index_lookup(unordered_segments: &[Segment]) -> HashMap<(i64, i64), Vec<i32>> {
    let mut start_indexes: HashMap<(i64, i64), Vec<i32>> = HashMap::new();

    for (starting_segment_index, segment) in unordered_segments.iter().enumerate() {
        let position_key = (segment.start.x, segment.start.y);
        start_indexes
            .entry(position_key)
            .or_default()
            .push(starting_segment_index as i32);
    }

    start_indexes
}

/// The last not-yet-added segment starting exactly at `added_segment_end_point`, or -1.
fn get_touching_segment_index(
    unordered_segments: &[Segment],
    start_indexes: &HashMap<(i64, i64), Vec<i32>>,
    segment_has_been_added: &[bool],
    added_segment_end_point: Point64,
) -> i32 {
    let mut lookup_segment_index = -1;
    let position_key = (added_segment_end_point.x, added_segment_end_point.y);
    if let Some(indexes) = start_indexes.get(&position_key) {
        for &index in indexes {
            if !segment_has_been_added[index as usize]
                && unordered_segments[index as usize].start == added_segment_end_point
            {
                lookup_segment_index = index;
            }
        }
    }

    lookup_segment_index
}

/// C# `IntPointPolygonsExtensions.AsVertices(polygons, outputScale = 1000)`: each polygon of
/// more than two points as a path, divided by `output_scale`.
pub fn polygons_as_vertices(polygons: &Paths64, output_scale: f64) -> Vec<VertexData> {
    polygons
        .iter()
        .filter(|polygon| polygon.len() > 2)
        .flat_map(|polygon| as_vertices(polygon, output_scale))
        .collect()
}

/// C# `IntPointPolygonExtensions.AsVertices(polygon, outputScale = 1000)`: a move-to the last
/// point, then a line-to every point in order (so the path ends where it began; no close).
pub fn as_vertices(polygon: &Path64, output_scale: f64) -> Vec<VertexData> {
    let scaled = |p: &Point64| Vector2::new(p.x as f64 / output_scale, p.y as f64 / output_scale);
    let Some(last) = polygon.last() else {
        return Vec::new();
    };
    std::iter::once(VertexData::new(FlagsAndCommand::MOVE_TO, scaled(last)))
        .chain(
            polygon
                .iter()
                .map(|p| VertexData::new(FlagsAndCommand::LINE_TO, scaled(p))),
        )
        .collect()
}

/// The area of a polygon, positive for counter-clockwise (C# `IntPointPolygonExtensions.Area`).
pub fn area(polygon: &Path64) -> f64 {
    let count = polygon.len();
    if count < 3 {
        return 0.0;
    }

    let mut a = 0.0;
    let mut last_point = count - 1;
    for i in 0..count {
        a += (polygon[last_point].x as f64 + polygon[i].x as f64)
            * (polygon[last_point].y as f64 - polygon[i].y as f64);
        last_point = i;
    }

    -a * 0.5
}

/// Positions sorted by x for a nearest-point search that walks outwards from a binary-search
/// hit (C# `SortedIntPoint`).
#[derive(Default)]
struct SortedIntPoint {
    sorted: Vec<(i32, Point64)>,
}

impl SortedIntPoint {
    fn add(&mut self, index: i32, position: Point64) {
        self.sorted.push((index, position));
    }

    /// The index of the closest position (skipping `index_to_skip`) and its squared distance,
    /// `f64::MAX` when none qualifies.
    fn find_closet_index(&self, position: Point64, index_to_skip: i32) -> (i32, f64) {
        let count = self.sorted.len() as i64;
        let mut index = (count - 1).min(self.binary_search(position));
        if index < 0 {
            index = (count - 1).min(!index);
        }
        let index = index as usize;

        let mut best_index = index;
        let mut best_distance_squared = f64::MAX;
        if self.sorted[best_index].0 != index_to_skip {
            best_distance_squared = distance_squared(self.sorted[index].1, position);
        }

        // we have the starting index now get all the vertices that are close enough
        for i in 0..self.sorted.len() {
            let mut checked_x = false;
            let current_index = index + i;
            let prev_index = index as i64 - i as i64 - 1;
            if current_index < self.sorted.len()
                && ((self.sorted[current_index].1.x - position.x).abs() as f64).powi(2)
                    <= best_distance_squared
            {
                checked_x = true;
                let dist_squared = distance_squared(self.sorted[current_index].1, position);
                if dist_squared < best_distance_squared
                    && self.sorted[current_index].0 != index_to_skip
                {
                    best_distance_squared = dist_squared;
                    best_index = current_index;
                }
            }

            if prev_index >= 0 {
                let prev_index = prev_index as usize;
                if ((self.sorted[prev_index].1.x - position.x).abs() as f64).powi(2)
                    <= best_distance_squared
                {
                    checked_x = true;
                    let dist_squared = distance_squared(self.sorted[prev_index].1, position);
                    if dist_squared < best_distance_squared
                        && self.sorted[prev_index].0 != index_to_skip
                    {
                        best_distance_squared = dist_squared;
                        best_index = prev_index;
                    }
                }
            }

            if best_distance_squared == 0.0 || !checked_x {
                break;
            }
        }

        (self.sorted[best_index].0, best_distance_squared)
    }

    /// .NET `List.BinarySearch` by x: the index of a match, or the complement of the insertion
    /// point.
    fn binary_search(&self, position: Point64) -> i64 {
        let mut lo = 0i64;
        let mut hi = self.sorted.len() as i64 - 1;
        while lo <= hi {
            let i = lo + ((hi - lo) >> 1);
            match compare_x(&self.sorted[i as usize].1, &position) {
                std::cmp::Ordering::Equal => return i,
                std::cmp::Ordering::Less => lo = i + 1,
                std::cmp::Ordering::Greater => hi = i - 1,
            }
        }
        !lo
    }

    fn remove(&mut self, index: i32) {
        let position = self
            .sorted
            .iter()
            .position(|(i, _)| *i == index)
            .expect("a polygon being linked is always in both sorters");
        self.sorted.remove(position);
    }

    fn sort(&mut self) {
        introspective_sort(&mut self.sorted, &mut |a, b| compare_x(&a.1, &b.1));
    }
}

fn distance_squared(a: Point64, b: Point64) -> f64 {
    length_squared(Point64::new(a.x - b.x, a.y - b.y)) as f64
}
