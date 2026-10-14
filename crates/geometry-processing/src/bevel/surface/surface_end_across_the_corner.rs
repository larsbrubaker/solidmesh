//! A CONCAVE bead's end cut off by the plane of the face standing across its corner
//! (free functions). Its two
//! rulers and reading the plane ([`try_read`]).

use std::collections::HashSet;

use vector_math::{math_helper, Vector3};

use super::{SurfaceBandEnd, SurfaceEdgePlan};
use crate::bevel::BevelChainEnds;
use crate::topology::{EdgeConvexity, FaceRegionCache};

/// How far two faces' normals may differ and still be read as one plane, as one less the cosine
/// between them - the same millionth `SurfaceEndCut` reads an end plane by, so a plane named here
/// is one that step can find again.
pub const FACES_AGREE: f64 = 1e-6;

/// How far from square to the run the face across the corner may lean and still be read as the
/// plane that cuts the bead off, as the cosine of
/// [`BevelChainEnds::MAXIMUM_CAP_PLANE_ANGLE_DEGREES`]: the same 80 degrees the boolean
/// construction caps on.
pub fn square_enough() -> f64 {
    math_helper::degrees_to_radians(BevelChainEnds::MAXIMUM_CAP_PLANE_ANGLE_DEGREES).cos()
}

/// The plane the bead of chain end `end` is cut off by at `corner` (mesh vertex
/// `corner_vertex`), where one stands - the single face there that is neither of the two the bevel
/// walks back, read as the half space its material is on. `None` where `end` is not a concave bead,
/// there are no `regions`, no such face stands there, two surfaces do (cutting the bead off on
/// either would leave it open against the other), or the face is not square ENOUGH to `outward`,
/// the way the chain would go on past the corner.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > y)` refuses a NaN as well.
pub fn try_read(
    regions: Option<&FaceRegionCache>,
    end: &SurfaceEdgePlan,
    corner_vertex: i32,
    corner: Vector3,
    outward: Vector3,
) -> Option<SurfaceBandEnd> {
    let mesh = regions?.mesh();

    if end.convexity() != EdgeConvexity::Concave {
        return None;
    }

    let being_walked_back: HashSet<i32> = [&end.left_region, &end.right_region]
        .iter()
        .flat_map(|region| region.faces().iter().copied())
        .collect();

    let mut across: Option<Vector3> = None;

    for (face_index, face) in mesh.faces.iter().enumerate() {
        if (face.v0 != corner_vertex && face.v1 != corner_vertex && face.v2 != corner_vertex)
            || being_walked_back.contains(&(face_index as i32))
        {
            continue;
        }

        let normal = Vector3::from_vector3_float(face.normal);

        if !(normal.length_squared() > 0.0) {
            continue;
        }

        let normal = normal.get_normal();

        match across {
            None => across = Some(normal),
            Some(first) => {
                if first.dot(normal) < 1.0 - FACES_AGREE {
                    // Two surfaces stand across this corner. Refused rather than guessed at.
                    return None;
                }
            }
        }
    }

    let across = across?;

    // And the chain itself has to be on the material side, square ENOUGH: a face standing nearly
    // along the run passes the sign test while cutting the bead off at a grazing angle the end ring
    // cannot be sewn to.
    if !(outward.dot(across) > square_enough()) {
        return None;
    }

    Some(SurfaceBandEnd::new(corner, -across))
}
