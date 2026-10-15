//! Cutting one face by a plane: the split, the cut line and the edge-symmetric clip of an edge at
//! the plane, plus the [`SplitData`] the optional clip predicate receives.

use std::cmp::Ordering;

use vector_math::{Plane, Vector3, Vector3Float};

use crate::face::Face;

/// What a `clip_face` predicate sees: the face and its three signed plane distances.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SplitData {
    pub face: Face,
    pub dist: [f64; 3],
}

/// Orders two positions x, then y, then z, so an edge has one canonical direction whichever face
/// looks at it. NaN components compare as "after".
fn compare_positions(a: Vector3Float, b: Vector3Float) -> Ordering {
    for axis in 0..3 {
        if a[axis] != b[axis] {
            return if a[axis] < b[axis] {
                Ordering::Less
            } else {
                Ordering::Greater
            };
        }
    }
    Ordering::Equal
}

/// The point where the edge `a`-`b` crosses a plane, interpolated from the lexicographically
/// smaller end so both faces sharing the edge get bit-identical results. Pass the ORIGINAL edge
/// endpoints, never already-subdivided ones.
pub fn clip_edge_at_plane(
    a: Vector3Float,
    b: Vector3Float,
    dist_a: f64,
    dist_b: f64,
) -> Vector3Float {
    let (start, end, start_dist, end_dist) = if compare_positions(b, a) == Ordering::Less {
        (b, a, dist_b, dist_a)
    } else {
        (a, b, dist_a, dist_b)
    };

    let total_distance = start_dist.abs() + end_dist.abs();
    let ratio_to_start = start_dist.abs() / total_distance;
    start + (end - start) * ratio_to_start
}

/// Whether the largest distance is above `on_plane_distance`, with NaN treated as in `linq_max`.
fn max_above(values: &[f64; 3], on_plane_distance: f64) -> bool {
    linq_max(values) > on_plane_distance
}

/// The largest value, where NaN only wins when every value is NaN, so one bad distance does not
/// decide which side a face is on.
fn linq_max(values: &[f64; 3]) -> f64 {
    let mut max = values[0];
    for &v in &values[1..] {
        if v > max || max.is_nan() {
            max = v;
        }
    }
    max
}

impl Face {
    /// Splits the face at `plane`, appending the pieces to `new_faces` and their corners to
    /// `new_vertices` (indices into `new_vertices`, not the mesh). Returns true when the face
    /// crosses the plane (or, with `discard_faces_on_negative_side`, when it is wholly on the
    /// negative side and so is dropped). `clip_face`, when given, can veto a two-edge split.
    #[allow(clippy::too_many_arguments)]
    pub fn split(
        &self,
        face_vertices: &[Vector3Float],
        plane: Plane,
        new_faces: &mut Vec<Face>,
        new_vertices: &mut Vec<Vector3Float>,
        on_plane_distance: f64,
        clip_face: Option<&dyn Fn(&SplitData) -> bool>,
        discard_faces_on_negative_side: bool,
    ) -> bool {
        let v = [
            face_vertices[self.v0 as usize],
            face_vertices[self.v1 as usize],
            face_vertices[self.v2 as usize],
        ];

        // get the distance from the crossing plane
        let dist = v.map(|a| plane.get_distance_from_plane_f32(a));

        // bool if each point is clipped
        let clip_point = dist.map(|a| a.abs() > on_plane_distance);

        // bool if there is a clip on a line segment (between points)
        let clip_segment: [bool; 3] = std::array::from_fn(|i| {
            let next_i = (i + 1) % 3;
            clip_point[i]
                && clip_point[next_i]
                && ((dist[i] < 0.0 && dist[next_i] > 0.0) || (dist[i] > 0.0 && dist[next_i] < 0.0))
        });

        let segments_clipped = clip_segment.iter().filter(|&&c| c).count();

        let clip_edge = |vi0: usize, new_vertices: &mut Vec<Vector3Float>| {
            let vi1 = (vi0 + 1) % 3;
            new_vertices.push(clip_edge_at_plane(v[vi0], v[vi1], dist[vi0], dist[vi1]));
        };

        match segments_clipped {
            // if 2 sides are clipped we will add 2 new vertices and 3 polygons
            2 => {
                let accepted = match clip_face {
                    Some(f) => f(&SplitData { face: *self, dist }),
                    None => true,
                };
                if accepted {
                    // find the side we are not going to clip
                    let vi0 = if clip_segment[0] && clip_segment[1] {
                        2
                    } else if clip_segment[0] && clip_segment[2] {
                        1
                    } else {
                        0
                    };
                    let vi1 = (vi0 + 1) % 3;
                    let vi2 = (vi0 + 2) % 3;
                    // NaN counts as not positive.
                    let vi0_positive = dist[vi0] > 0.0;
                    let vertex_start = new_vertices.len() as i32;
                    new_vertices.push(v[vi0]);
                    new_vertices.push(v[vi1]);
                    new_vertices.push(v[vi2]);
                    clip_edge(vi1, new_vertices);
                    clip_edge(vi2, new_vertices);
                    if !discard_faces_on_negative_side || vi0_positive {
                        new_faces.push(Face::new(
                            vertex_start,
                            vertex_start + 1,
                            vertex_start + 3,
                            new_vertices,
                        ));
                        new_faces.push(Face::new(
                            vertex_start,
                            vertex_start + 3,
                            vertex_start + 4,
                            new_vertices,
                        ));
                    }
                    if !discard_faces_on_negative_side || !vi0_positive {
                        new_faces.push(Face::new(
                            vertex_start + 3,
                            vertex_start + 2,
                            vertex_start + 4,
                            new_vertices,
                        ));
                    }
                    return true;
                }
            }
            // if 1 side is clipped we will add 1 new vertex and 2 polygons
            1 => {
                let vi0 = if clip_segment[0] {
                    0
                } else if clip_segment[1] {
                    1
                } else {
                    2
                };
                let vi1 = (vi0 + 1) % 3;
                let vi2 = (vi0 + 2) % 3;
                // NaN counts as not positive.
                let vi0_positive = dist[vi0] > 0.0;
                let vertex_start = new_vertices.len() as i32;
                new_vertices.push(v[vi0]);
                new_vertices.push(v[vi1]);
                new_vertices.push(v[vi2]);
                clip_edge(vi0, new_vertices);
                if !discard_faces_on_negative_side || vi0_positive {
                    new_faces.push(Face::new(
                        vertex_start,
                        vertex_start + 3,
                        vertex_start + 2,
                        new_vertices,
                    ));
                }
                if !discard_faces_on_negative_side || !vi0_positive {
                    new_faces.push(Face::new(
                        vertex_start + 3,
                        vertex_start + 1,
                        vertex_start + 2,
                        new_vertices,
                    ));
                }
                return true;
            }
            // This face doesn't cross the plane (NaN maxima count as not above it).
            0 if discard_faces_on_negative_side && !max_above(&dist, on_plane_distance) => {
                return true;
            }
            _ => {}
        }

        false
    }

    /// The segment where `plane` cuts the face, wound to respect the face's winding. `None` when
    /// the face does not cross the plane.
    pub fn get_cut_line(
        &self,
        face_vertices: &[Vector3Float],
        plane: Plane,
    ) -> Option<(Vector3, Vector3)> {
        let v = [
            face_vertices[self.v0 as usize],
            face_vertices[self.v1 as usize],
            face_vertices[self.v2 as usize],
        ];

        // get the distance from the crossing plane
        let dist = v.map(|a| plane.get_distance_from_plane_f32(a));

        let clip_edge = |vi0: usize| {
            let vi1 = (vi0 + 1) % 3;
            let total_distance = dist[vi0].abs() + dist[vi1].abs();
            let ratio_to_dist0 = dist[vi0].abs() / total_distance;
            let new_point = v[vi0] + (v[vi1] - v[vi0]) * ratio_to_dist0;
            Vector3::from_vector3_float(new_point)
        };

        let (d0, d1, d2) = (dist[0], dist[1], dist[2]);
        if d0 < 0.0 && d1 >= 0.0 && d2 >= 0.0 {
            Some((clip_edge(2), clip_edge(0)))
        } else if d0 >= 0.0 && d1 < 0.0 && d2 < 0.0 {
            Some((clip_edge(0), clip_edge(2)))
        } else if d0 >= 0.0 && d1 < 0.0 && d2 >= 0.0 {
            Some((clip_edge(0), clip_edge(1)))
        } else if d0 < 0.0 && d1 >= 0.0 && d2 < 0.0 {
            Some((clip_edge(1), clip_edge(0)))
        } else if d0 >= 0.0 && d1 >= 0.0 && d2 < 0.0 {
            Some((clip_edge(1), clip_edge(2)))
        } else if d0 < 0.0 && d1 < 0.0 && d2 >= 0.0 {
            Some((clip_edge(2), clip_edge(1)))
        } else {
            None
        }
    }
}
