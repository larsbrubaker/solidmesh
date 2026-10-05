//! A CONCAVE bead's end cut off by the plane of the face standing across its corner
//! (`Bevel/Surface/SurfaceEndAcrossTheCorner.cs`, a static class: free functions here). Its two
//! rulers land here first; reading the plane lands with a later package.

use vector_math::math_helper;

use crate::bevel::BevelChainEnds;

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
