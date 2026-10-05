//! A chain vertex where the chain turns SHARPLY - a corner of the outline, where the two edges' canals
//! cannot share one section by a smooth sweep (`Bevel/Sections/SectionCorner.cs`, a C# `internal
//! static class`; a module of free functions here). What is here are the corner's tolerances and the
//! reading of a turn ([`turn_degrees`], [`is_sharp_turn`]); the mitre plane ([`try_the_mitre_plane`]);
//! the reading of the two sides ([`try_read_the_sides`]) and the slid profile each edge's canal cuts
//! in the bisector ([`slid_profile`]). The corner's construction (`TryBuild`) lands with a later run
//! of this package.

use vector_math::{cs_math, math_helper, Vector3};

use super::{section_builder, SectionSide};
use crate::topology::{ChainEdge, FaceGroup, FeatureEdge};

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

/// C# `TryTheMitrePlane`: THE PLANE THE MITRE IS CUT BY, for two edges standing at any two wedges on
/// their shared flat - square to the sum of the two edges' directions, through the point where their
/// canal axes come nearest (the midpoint of the two nearest points) - as `(through, normal)`. `None`
/// where the chain turns straight back or the two axes run parallel.
pub fn try_the_mitre_plane(
    arriving: &ChainEdge,
    leaving: &ChainEdge,
    radius: f64,
) -> Option<(Vector3, Vector3)> {
    let along0 = direction(arriving).get_normal();
    let along1 = direction(leaving).get_normal();
    let summed = along0 + along1;

    if summed.length() < SAME_TO_WITHIN {
        return None;
    }

    let normal = summed.get_normal();

    let station = leaving.from();
    let lean = along0.dot(along1);
    let denominator = 1.0 - (lean * lean);

    if denominator < SAME_TO_WITHIN {
        // Parallel axes never meet, so there is no one point to take the plane through.
        return None;
    }

    let from0 = the_axis_at(arriving, station, radius);
    let from1 = the_axis_at(leaving, station, radius);
    let between = from1 - from0;
    let step0 = (between.dot(along0) - (lean * between.dot(along1))) / denominator;
    let step1 = ((lean * between.dot(along0)) - between.dot(along1)) / denominator;

    let through = ((from0 + (along0 * step0)) + (from1 + (along1 * step1))) / 2.0;

    Some((through, normal))
}

/// C# `TheAxisAt`: the point of an edge's canal axis level with `station` - a radius clear of both
/// of its faces.
pub fn the_axis_at(edge: &ChainEdge, station: Vector3, radius: f64) -> Vector3 {
    let one = edge.left_normal().get_normal();
    let other = edge.right_normal().get_normal();

    station - ((one + other) * radius / (1.0 + one.dot(other)))
}

/// C# `LooksLikeACorner`: whether two edges' sides read as a corner the section can be shared across
/// ([`try_read_the_sides`]).
pub fn looks_like_a_corner(arriving: &ChainEdge, leaving: &ChainEdge) -> bool {
    try_read_the_sides(arriving, leaving).is_some()
}

/// C# `SharedKind`: a side that rides a shared fold slides; one that does not offsets.
pub fn shared_kind(shared_fold: i32) -> SectionSide {
    if shared_fold >= 0 {
        SectionSide::Slide
    } else {
        SectionSide::Offset
    }
}

/// C# `TryReadTheSides`: which side two edges SHARE (the same surface either side of the turn - read
/// at the curve tolerance, since a tessellated wall hands it a different facet each side) and the
/// wedge they stand at, as `(flat_on_the_left, cosine, wedge)`. `None` where both sides or neither are
/// shared, or the two wedges differ past [`SAME_WEDGE_TO_WITHIN`], or the wedge is flat or folds
/// back.
pub fn try_read_the_sides(arriving: &ChainEdge, leaving: &ChainEdge) -> Option<(bool, f64, f64)> {
    let smooth = smooth_surface_dot();
    let left_is_shared = arriving
        .left_normal()
        .get_normal()
        .dot(leaving.left_normal().get_normal())
        > smooth;
    let right_is_shared = arriving
        .right_normal()
        .get_normal()
        .dot(leaving.right_normal().get_normal())
        > smooth;

    if left_is_shared == right_is_shared {
        // Both sides shared is a smooth vertex drawn as a corner; neither shared is a corner where
        // all four faces differ, which has no one profile to share.
        return None;
    }

    let arriving_cosine = arriving
        .left_normal()
        .get_normal()
        .dot(arriving.right_normal().get_normal());
    let leaving_cosine = leaving
        .left_normal()
        .get_normal()
        .dot(leaving.right_normal().get_normal());

    if (arriving_cosine - leaving_cosine).abs() > SAME_WEDGE_TO_WITHIN
        || arriving_cosine > FeatureEdge::COPLANAR_FACE_DOT
        || arriving_cosine < -0.999999
    {
        return None;
    }

    Some((
        left_is_shared,
        arriving_cosine,
        arriving_cosine.clamp(-1.0, 1.0).acos(),
    ))
}

/// C# `AwayInTheWall`: the direction across the WALL face away from the edge - in the wall's plane,
/// square to the edge - for an edge whose faces meet at `cosine`.
pub fn away_in_the_wall(edge: &ChainEdge, wall_on_the_left: bool, cosine: f64) -> Vector3 {
    let (wall, flat) = if wall_on_the_left {
        (edge.left_normal(), edge.right_normal())
    } else {
        (edge.right_normal(), edge.left_normal())
    };
    let across = cs_math::max_f64(1.0 - (cosine * cosine), 0.0).sqrt();

    ((wall.get_normal() * cosine) - flat.get_normal()) / across
}

/// C# `SlidProfile`: an edge's own profile of `points` samples at `station`, each slid along the edge
/// onto the bisector plane through the station - the curve that edge's canal cuts in that plane.
pub fn slid_profile(
    edge: &ChainEdge,
    station: Vector3,
    bisector: Vector3,
    radius: f64,
    points: usize,
) -> Vec<Vector3> {
    let along = direction(edge).get_normal();
    let lean = along.dot(bisector);
    let one = edge.left_normal().get_normal();
    let other = edge.right_normal().get_normal();
    let cosine = one.dot(other);
    let wedge = cosine.clamp(-1.0, 1.0).acos();
    let centre = station - ((one + other) * radius / (1.0 + cosine));

    (0..points)
        .map(|k| {
            let swept = centre
                + (section_builder::slerp(one, other, k as f64 / (points - 1) as f64, wedge)
                    * radius);

            swept - (along * ((swept - station).dot(bisector) / lean))
        })
        .collect()
}
