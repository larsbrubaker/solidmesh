//! How far an OFFSET side's boundary may reach across a smooth side, read along the surface
//! (a free function beside [`super::section_room`]'s region readings). Where the
//! side's region is one plane the ray across the face is exact ([`section_room::reach_across_the_face`]);
//! across a curved region it is the surface walk ([`super::section_surface_walk_try::try_walk`]).

use indexmap::IndexMap;
use solidmesh_math::Vector3;
use solidmesh_mesh::{Mesh, MeshEdgeGraph};

use super::section_fan_walk::EdgeAtPair;
use super::section_room::{self, Region};
use super::section_surface_walk_faces::try_face_holding;
use super::section_surface_walk_try::try_walk;
use super::SurfaceWalkRefusal;

/// How far the boundary from `station` along `away` may reach before it
/// leaves the part, for the side whose `fan` of faces stands at `vertex`.
///
/// ONE PLANE (or nothing asked): the ray drawn in the seed face's plane is a path along the part, and
/// [`section_room::reach_across_the_face`] answers out of the pass's region cache. Across a CURVED
/// region the walk is the reading: a whole `asked_reach` walked is no limit (the asked reach comes
/// back); a walk stopped by a crease or a boundary gives how far it travelled; a walk the surface
/// itself could not make names no limit at all (`f64::MAX`), and the room's number comes from
/// elsewhere.
#[allow(clippy::too_many_arguments, clippy::neg_cmp_op_on_partial_ord)]
pub fn reach_of_an_offset_side(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    regions_by_face: &mut IndexMap<i32, Region>,
    fan: &[i32],
    vertex: i32,
    station: Vector3,
    away: Vector3,
    is_a_feature_edge: Option<&dyn Fn(i32) -> bool>,
    asked_reach: f64,
) -> f64 {
    let region = section_room::region_of_the_face(
        mesh,
        graph,
        edge_at_pair,
        regions_by_face,
        fan[0],
        is_a_feature_edge,
    );

    if region.is_one_plane || !(asked_reach > 0.0) {
        return section_room::reach_across_the_face(
            mesh, graph, &region, fan[0], vertex, station, away,
        );
    }

    let Ok((face, in_face)) = try_face_holding(mesh, vertex, away, fan) else {
        return f64::MAX;
    };

    match try_walk(
        mesh,
        graph,
        edge_at_pair,
        station,
        face,
        in_face,
        asked_reach,
        is_a_feature_edge,
    ) {
        // A whole span of the part spent and nothing of the part in the way: no limit arises here.
        Ok(_) => asked_reach,
        Err(walk)
            if walk.refusal == SurfaceWalkRefusal::CreaseReached
                || walk.refusal == SurfaceWalkRefusal::BoundaryReached =>
        {
            walk.travelled
        }
        // A walk the surface itself could not make is no reading at all.
        Err(_) => f64::MAX,
    }
}
