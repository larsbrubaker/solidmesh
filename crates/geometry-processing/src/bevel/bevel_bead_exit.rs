//! Where a BEAD's run leaves its end vertex through a face of the part it does not own: the exit
//! planes a bead is clipped to so it adds no material past them. Reading the exits off the faces at
//! the end vertex and gathering a run's two ends' exits land here, with the half-space inside an
//! exit plane ([`inside`]).

use vector_math::{math_helper, Vector3};

use polygon_mesh::Mesh;
use vector_math::AxisAlignedBoundingBox;

use super::bevel_cap_clip;
use super::{BevelChainEnds, BevelEndCap};
use crate::topology::FeatureEdgeSet;

/// How square to the run's way out a face has to stand to be an exit, as a cosine: the steepest
/// cap plane a run may stop on ([`BevelChainEnds::MAXIMUM_CAP_PLANE_ANGLE_DEGREES`]).
pub fn minimum_agreement() -> f64 {
    math_helper::degrees_to_radians(BevelChainEnds::MAXIMUM_CAP_PLANE_ANGLE_DEGREES).cos()
}

/// How near two exit planes' normals have to be to be one plane.
const SAME_PLANE: f64 = 1e-9;

/// The faces the run leaves its end vertex (at `position`, running `outward`) through: every face
/// of the `incident` feature edges that is neither of the run's own two faces and faces the way out
/// by more than [`minimum_agreement`], each plane once. `None` where there are none.
pub fn faces_the_run_leaves_through(
    incident: &[i32],
    features: &FeatureEdgeSet,
    position: Vector3,
    outward: Vector3,
    own_face0: i32,
    own_face1: i32,
) -> Option<Vec<BevelEndCap>> {
    let mut exits: Option<Vec<BevelEndCap>> = None;

    for &index in incident {
        let edge = features.edges()[index as usize];

        for side in 0..2 {
            let face = if side == 0 { edge.face0 } else { edge.face1 };

            if face == own_face0 || face == own_face1 {
                continue;
            }

            let normal = if side == 0 {
                edge.normal0
            } else {
                edge.normal1
            };

            if normal.dot(outward) <= minimum_agreement() || already_have(exits.as_deref(), normal)
            {
                continue;
            }

            exits
                .get_or_insert_with(Vec::new)
                .push(BevelEndCap::new(normal, normal.dot(position)));
        }
    }

    exits
}

/// Whether `exits` already has a plane facing `normal`.
pub fn already_have(exits: Option<&[BevelEndCap]>, normal: Vector3) -> bool {
    exits.is_some_and(|exits| {
        exits
            .iter()
            .any(|exit| exit.plane_normal.dot(normal) > 1.0 - SAME_PLANE)
    })
}

/// Both ends' exit planes together (start first), or `None` where neither end has any or `ends` is
/// `None`.
pub fn of(ends: Option<&BevelChainEnds>) -> Option<Vec<BevelEndCap>> {
    let ends = ends?;

    let start = ends.start_exit_planes.as_ref().filter(|s| !s.is_empty());
    let end = ends.end_exit_planes.as_ref().filter(|e| !e.is_empty());

    match (start, end) {
        (None, None) => None,
        (None, Some(end)) => Some(end.clone()),
        (Some(start), None) => Some(start.clone()),
        (Some(start), Some(end)) => {
            let mut both = Vec::with_capacity(start.len() + end.len());
            both.extend_from_slice(start);
            both.extend_from_slice(end);
            Some(both)
        }
    }
}

/// The half-space INSIDE an exit plane - the cap clip's box for the plane turned round.
pub fn inside(plane: &BevelEndCap, bounds: &AxisAlignedBoundingBox) -> Mesh {
    bevel_cap_clip::half_space(
        &BevelEndCap::new(-plane.plane_normal, -plane.plane_distance),
        bounds,
    )
}
