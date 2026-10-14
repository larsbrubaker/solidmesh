//! The reading of the MATERIAL at a chain's end corner: whether one of the two faces the bevel
//! walks back carries straight on past that corner, so that the groove has to be closed by a wall
//! of its own rather than notched into a face beyond it. Its nested reading
//! ([`SurfaceFaceCarriesOnWayOn`]), the reading ([`past`]) and where the way on falls in one facet
//! ([`where_the_way_on_falls`]).

/// Where the way on past a corner falls against one
/// facet of the region standing there.
use std::collections::HashSet;

use polygon_mesh::Mesh;
use vector_math::Vector3;

use super::SurfaceEdgePlan;
use crate::topology::FaceRegionCache;

/// How near the facet's edge the way on may run and still be read as running ALONG it.
const ALONG_THE_FACETS_EDGE: f64 = 1e-9;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum SurfaceFaceCarriesOnWayOn {
    /// Outside the facet, or not a facet at this corner at all.
    #[default]
    Outside = 0,

    /// Along one of the facet's own two edges at the corner.
    AlongAnEdge,

    /// Into the facet's material.
    Inside,
}

/// Whether one of the two faces `end` walks back carries straight on past its corner (vertex
/// `corner_vertex` at `corner`) the way the chain runs out (`outward`): some facet of either region
/// at the corner has the way on INSIDE it, or two of its facets both run along an edge to the same
/// far vertex. `None` regions (no region cache) carry nothing on.
pub fn past(
    regions: Option<&FaceRegionCache>,
    end: &SurfaceEdgePlan,
    corner_vertex: i32,
    corner: Vector3,
    outward: Vector3,
) -> bool {
    let Some(regions) = regions else {
        return false;
    };
    let mesh = regions.mesh();

    for region in [&end.left_region, &end.right_region] {
        let mut claimed = HashSet::new();

        for &face_index in region.faces() {
            match where_the_way_on_falls(mesh, face_index, corner_vertex, corner, outward) {
                (SurfaceFaceCarriesOnWayOn::Inside, _) => return true,
                (SurfaceFaceCarriesOnWayOn::AlongAnEdge, far_vertex)
                    if !claimed.insert(far_vertex) =>
                {
                    return true
                }
                _ => {}
            }
        }
    }

    false
}

/// Where the way on (`outward` squared into the facet's plane) falls in facet `face_index` at its
/// corner `corner_vertex`: outside it, inside it, or along one of its two edges there - with the
/// far vertex of that edge (-1 otherwise).
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)` refuses a NaN as well.
pub fn where_the_way_on_falls(
    mesh: &Mesh,
    face_index: i32,
    corner_vertex: i32,
    corner: Vector3,
    outward: Vector3,
) -> (SurfaceFaceCarriesOnWayOn, i32) {
    let outside = (SurfaceFaceCarriesOnWayOn::Outside, -1);
    let face = mesh.faces[face_index as usize];
    let corners = [face.v0, face.v1, face.v2];
    let Some(at) = corners.iter().position(|&v| v == corner_vertex) else {
        return outside;
    };

    let first_vertex = corners[(at + 1) % 3];
    let second_vertex = corners[(at + 2) % 3];
    let first = Vector3::from_vector3_float(mesh.vertices[first_vertex as usize]) - corner;
    let second = Vector3::from_vector3_float(mesh.vertices[second_vertex as usize]) - corner;
    let normal = first.cross(second);

    if !(normal.length_squared() > 0.0) {
        return outside;
    }

    let normal = normal.get_normal();

    let way_on = outward - (normal * outward.dot(normal));

    if !(way_on.length_squared() > 0.0) {
        return outside;
    }

    let way_on = way_on.get_normal();

    let off_the_first = first.get_normal().cross(way_on).dot(normal);
    let off_the_second = way_on.cross(second.get_normal()).dot(normal);

    if off_the_first < -ALONG_THE_FACETS_EDGE || off_the_second < -ALONG_THE_FACETS_EDGE {
        return outside;
    }

    if off_the_first > ALONG_THE_FACETS_EDGE && off_the_second > ALONG_THE_FACETS_EDGE {
        return (SurfaceFaceCarriesOnWayOn::Inside, -1);
    }

    (
        SurfaceFaceCarriesOnWayOn::AlongAnEdge,
        if off_the_first <= off_the_second {
            first_vertex
        } else {
            second_vertex
        },
    )
}
