//! A chain vertex where the chain turns SHARPLY - a corner of the outline, where the two edges' canals
//! cannot share one section by a smooth sweep (`Bevel/Sections/SectionCorner.cs`, a C# `internal
//! static class`; a module of free functions here). What is here are the corner's tolerances and the
//! reading of a turn ([`turn_degrees`], [`is_sharp_turn`]); the corner's construction (`TryBuild`) and
//! its mitre plane land with later runs of this package.

use vector_math::{math_helper, Vector3};

use crate::topology::{ChainEdge, FaceGroup};

/// C# `SameToWithin`: how nearly two wedges, two slide distances or two directions must agree to be
/// read as the same. The corner is a SYMMETRIC construction; a pair that is only nearly symmetric is
/// refused rather than split the difference.
pub const SAME_TO_WITHIN: f64 = 1e-9;

/// C# `SameWedgeToWithin`: how nearly two wedges - or two slide distances down the fold - must agree
/// to be read as the same wedge. Only the CHEAP SCREEN the router can ask off the two chain edges
/// alone; wider than [`SAME_TO_WITHIN`] because both are read off FACE NORMALS of float vertices.
pub const SAME_WEDGE_TO_WITHIN: f64 = 1e-4;

/// C# `ProfilesMeetToWithinThisMuchOfTheRadius`: how far apart the two edges' profiles, each slid onto
/// the bisector plane, may stand and still be read as ONE curve, as a fraction of the radius.
pub const PROFILES_MEET_TO_WITHIN_THIS_MUCH_OF_THE_RADIUS: f64 = 1e-3;

/// C# `SmoothSurfaceDot`: how nearly two faces must line up to be read as ONE SMOOTH SURFACE rather than
/// two sides of a feature - the cosine of the same curve tolerance [`is_sharp_turn`] reads turns at.
pub fn smooth_surface_dot() -> f64 {
    math_helper::degrees_to_radians(FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES).cos()
}

/// C# `IsSharpTurn`: whether the chain turns past the curve tolerance between two consecutive edges,
/// which is what makes the vertex a corner rather than a point of a curve. The builder and the router
/// read the same question here so that neither can route what the other would not draw.
pub fn is_sharp_turn(arriving: &ChainEdge, leaving: &ChainEdge) -> bool {
    turn_degrees(arriving, leaving) > FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES
}

/// C# `TurnDegrees`: the turn between two consecutive chain edges, in degrees.
pub fn turn_degrees(arriving: &ChainEdge, leaving: &ChainEdge) -> f64 {
    let cosine = direction(arriving)
        .get_normal()
        .dot(direction(leaving).get_normal());

    math_helper::radians_to_degrees(cosine.clamp(-1.0, 1.0).acos())
}

/// A chain edge's walk direction.
fn direction(edge: &ChainEdge) -> Vector3 {
    edge.direction()
}
