//! Turns "the user clicked here" into "they meant this edge". Ports `Topology/EdgePicking.cs`
//! (`FeatureEdgePick`, `EdgePicking`). Picks against a [`FeatureEdgeSet`] built from the mesh the
//! ray was cast at.

use polygon_mesh::Mesh;
use vector_math::{Matrix4X4, Vector3};

use super::error::TopologyError;
use super::feature_edge::FeatureEdge;
use super::feature_edge_set::FeatureEdgeSet;

/// Which feature edge a pick landed on, or that it landed on none. `Default` is a miss, so a
/// caller that forgets to check `is_hit` reads an edge index of -1 rather than edge zero.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct FeatureEdgePick {
    is_hit: bool,
    edge_index: i32,
    distance_to_edge: f64,
    edge: FeatureEdge,
    closest_point_on_edge: Vector3,
}

impl FeatureEdgePick {
    /// A pick that found nothing.
    pub fn missed() -> Self {
        Self::default()
    }

    /// Whether a feature edge was found within the tolerance.
    pub fn is_hit(&self) -> bool {
        self.is_hit
    }

    /// The edge that was picked, as an index into the [`FeatureEdgeSet`] the pick ran against, or
    /// -1 on a miss.
    pub fn edge_index(&self) -> i32 {
        if self.is_hit {
            self.edge_index
        } else {
            -1
        }
    }

    /// The edge that was picked. Default on a miss.
    pub fn edge(&self) -> FeatureEdge {
        self.edge
    }

    /// How far the pick point was from the edge segment, in the space the pick measured in -
    /// mesh-local unless the caller supplied a measurement matrix. Positive infinity on a miss.
    pub fn distance_to_edge(&self) -> f64 {
        if self.is_hit {
            self.distance_to_edge
        } else {
            f64::INFINITY
        }
    }

    /// The point on the edge nearest the pick point, in mesh-local space - where a hover
    /// highlight or a size handle wants to sit. Zero on a miss.
    pub fn closest_point_on_edge(&self) -> Vector3 {
        self.closest_point_on_edge
    }

    pub(crate) fn hit(
        edge_index: i32,
        edge: FeatureEdge,
        distance_to_edge: f64,
        closest_point_on_edge: Vector3,
    ) -> Self {
        FeatureEdgePick {
            is_hit: true,
            edge_index,
            distance_to_edge,
            edge,
            closest_point_on_edge,
        }
    }
}

/// Finds the feature edge a pick point on a face meant, measuring in mesh-local space. See
/// [`pick_feature_edge_measured`].
pub fn pick_feature_edge(
    mesh: &Mesh,
    features: &FeatureEdgeSet,
    hit_face_index: i32,
    hit_position_local: Vector3,
    max_distance: f64,
) -> Result<FeatureEdgePick, TopologyError> {
    pick_feature_edge_measured(
        mesh,
        features,
        hit_face_index,
        hit_position_local,
        max_distance,
        Matrix4X4::IDENTITY,
    )
}

/// Finds the feature edge a pick point on a face meant, or reports that it meant none, with the
/// distances measured after `measurement_space` (usually the object's world matrix) has been
/// applied to the hit point and to each candidate. A pick tolerance is really a number of
/// pixels, a world-space measure; converting it to local units by one number breaks on an
/// unevenly scaled object. Only the comparison moves: the edge and point that come back are in
/// mesh-local space, carried across by the along-the-edge fraction, which an affine map keeps.
///
/// The candidates are the feature edges incident to the hit face's three VERTICES rather than
/// the face's own edges: a face's own edge may be a triangulation diagonal and not in the set,
/// and a fan-triangulated flat region has interior triangles whose edges are all diagonals. What
/// that cannot see is a feature edge passing near the point while sharing no vertex with the hit
/// face (Steiner vertices, slivers); widening to the one-ring is deferred until a part needs it.
/// Ties keep the first candidate in the incidence lists' order, so the same input always picks
/// the same edge.
///
/// Errors when `hit_face_index` is not a face of `mesh` (a stale BVH or the wrong mesh rather
/// than a miss), or when `hit_position_local` or `max_distance` is not finite: every comparison
/// against NaN is false, so the loop would report a confident hit at a distance of NaN.
pub fn pick_feature_edge_measured(
    mesh: &Mesh,
    features: &FeatureEdgeSet,
    hit_face_index: i32,
    hit_position_local: Vector3,
    max_distance: f64,
    measurement_space: Matrix4X4,
) -> Result<FeatureEdgePick, TopologyError> {
    let face_count = mesh.faces.len();
    if hit_face_index < 0 || hit_face_index as usize >= face_count {
        return Err(TopologyError::FaceIndexOutOfRange {
            parameter: "hitFaceIndex",
            index: hit_face_index,
            face_count,
        });
    }

    if !is_finite(hit_position_local) {
        return Err(TopologyError::NotFinite {
            parameter: "hitPositionLocal",
            message: format!("The hit position {hit_position_local} is not a finite point."),
        });
    }

    if !max_distance.is_finite() {
        return Err(TopologyError::NotFinite {
            parameter: "maxDistance",
            message: format!("A pick tolerance of {max_distance} is not a distance."),
        });
    }

    let face = mesh.faces[hit_face_index as usize];
    let face_vertices = [face.v0, face.v1, face.v2];

    let hit_position_measured = hit_position_local.transform(measurement_space);

    let mut best = FeatureEdgePick::missed();
    let mut best_distance = f64::INFINITY;

    for &vertex in &face_vertices {
        // An edge along the face is reached from both of its vertices and so is measured twice.
        // Harmless: the second measurement ties with the first and the strict less-than keeps
        // the first.
        for &candidate_index in features.edges_at_vertex(vertex) {
            let candidate = features.edges()[candidate_index as usize];

            let start_measured = candidate.start.transform(measurement_space);
            let end_measured = candidate.end.transform(measurement_space);

            let along_fraction =
                closest_fraction_along_segment(start_measured, end_measured, hit_position_measured);
            let distance = (hit_position_measured
                - lerp_along(start_measured, end_measured, along_fraction))
            .length();

            if distance > max_distance || distance >= best_distance {
                continue;
            }

            best_distance = distance;
            best = FeatureEdgePick::hit(
                candidate_index,
                candidate,
                distance,
                lerp_along(candidate.start, candidate.end, along_fraction),
            );
        }
    }

    Ok(best)
}

fn is_finite(position: Vector3) -> bool {
    position.x.is_finite() && position.y.is_finite() && position.z.is_finite()
}

/// How far along a segment its nearest point to another point lies, as a fraction from 0 at the
/// start to 1 at the end. Clamped to the segment, which keeps an edge whose LINE runs close to
/// the pick from stealing it. A segment with no length is its own nearest point.
fn closest_fraction_along_segment(start: Vector3, end: Vector3, point: Vector3) -> f64 {
    let along_edge = end - start;
    let length_squared = along_edge.length_squared();

    if length_squared.is_nan() || length_squared <= 0.0 {
        return 0.0;
    }

    ((point - start).dot(along_edge) / length_squared).clamp(0.0, 1.0)
}

fn lerp_along(start: Vector3, end: Vector3, along_fraction: f64) -> Vector3 {
    start + ((end - start) * along_fraction)
}
