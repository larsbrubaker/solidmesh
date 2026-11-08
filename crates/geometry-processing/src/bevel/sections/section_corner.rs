//! A chain vertex where the chain turns SHARPLY - a corner of the outline, where the two edges' canals
//! cannot share one section by a smooth sweep. What is here are the corner's tolerances and the
//! reading of a turn ([`turn_degrees`], [`is_sharp_turn`]); the mitre plane ([`try_the_mitre_plane`]);
//! the reading of the two sides ([`try_read_the_sides`]) and the slid profile each edge's canal cuts
//! in the bisector ([`slid_profile`]); the fold and shared side the section ends on
//! ([`try_the_fold`], [`try_the_shared_side`]); and the corner's construction ([`try_build`]).

use solidmesh_math::{ieee_math, math_helper, Vector3};

use solidmesh_mesh::{Mesh, MeshEdgeGraph};

use super::section_fan_walk::EdgeAtPair;
use super::{
    section_builder, section_junction_build, BevelSection, NanSectionPoint, SectionKind,
    SectionPool, SectionSide,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};
use crate::topology::{ChainEdge, FaceGroup, FeatureEdge};

/// How nearly two wedges, two slide distances or two directions must agree to be
/// read as the same. The corner is a SYMMETRIC construction; a pair that is only nearly symmetric is
/// refused rather than split the difference.
pub const SAME_TO_WITHIN: f64 = 1e-9;

/// How nearly two wedges - or two slide distances down the fold - must agree
/// to be read as the same wedge. Only the CHEAP SCREEN the router can ask off the two chain edges
/// alone; wider than [`SAME_TO_WITHIN`] because both are read off FACE NORMALS of float vertices.
pub const SAME_WEDGE_TO_WITHIN: f64 = 1e-4;

/// How far apart the two edges' profiles, each slid onto
/// the bisector plane, may stand and still be read as ONE curve, as a fraction of the radius.
pub const PROFILES_MEET_TO_WITHIN_THIS_MUCH_OF_THE_RADIUS: f64 = 1e-3;

/// How nearly two faces must line up to be read as ONE SMOOTH SURFACE rather than
/// two sides of a feature - the cosine of the same curve tolerance [`is_sharp_turn`] reads turns at.
pub fn smooth_surface_dot() -> f64 {
    math_helper::degrees_to_radians(FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES).cos()
}

/// Whether the chain turns past the curve tolerance between two consecutive edges,
/// which is what makes the vertex a corner rather than a point of a curve. The builder and the router
/// read the same question here so that neither can route what the other would not draw.
pub fn is_sharp_turn(arriving: &ChainEdge, leaving: &ChainEdge) -> bool {
    turn_degrees(arriving, leaving) > FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES
}

/// The turn between two consecutive chain edges, in degrees.
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

/// THE PLANE THE MITRE IS CUT BY, for two edges standing at any two wedges on
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

/// The point of an edge's canal axis level with `station` - a radius clear of both
/// of its faces.
pub fn the_axis_at(edge: &ChainEdge, station: Vector3, radius: f64) -> Vector3 {
    let one = edge.left_normal().get_normal();
    let other = edge.right_normal().get_normal();

    station - ((one + other) * radius / (1.0 + one.dot(other)))
}

/// Whether two edges' sides read as a corner the section can be shared across
/// ([`try_read_the_sides`]).
pub fn looks_like_a_corner(arriving: &ChainEdge, leaving: &ChainEdge) -> bool {
    try_read_the_sides(arriving, leaving).is_some()
}

/// A side that rides a shared fold slides; one that does not offsets.
pub fn shared_kind(shared_fold: i32) -> SectionSide {
    if shared_fold >= 0 {
        SectionSide::Slide
    } else {
        SectionSide::Offset
    }
}

/// Which side two edges SHARE (the same surface either side of the turn - read
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

/// The direction across the WALL face away from the edge - in the wall's plane,
/// square to the edge - for an edge whose faces meet at `cosine`.
pub fn away_in_the_wall(edge: &ChainEdge, wall_on_the_left: bool, cosine: f64) -> Vector3 {
    let (wall, flat) = if wall_on_the_left {
        (edge.left_normal(), edge.right_normal())
    } else {
        (edge.right_normal(), edge.left_normal())
    };
    let across = ieee_math::max_f64(1.0 - (cosine * cosine), 0.0).sqrt();

    ((wall.get_normal() * cosine) - flat.get_normal()) / across
}

/// An edge's own profile of `points` samples at `station`, each slid along the edge
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

/// The one crease the WALL side's fan crosses at a corner - the fold the wall
/// boundary slides down - checked to lean out along both edges' walls by the same slide distance
/// (within [`SAME_WEDGE_TO_WITHIN`] of the setback). No fold, several, or a fold the two edges would
/// slide down different distances is refused.
#[allow(clippy::too_many_arguments)]
pub fn try_the_fold(
    vertex: i32,
    arriving: &ChainEdge,
    leaving: &ChainEdge,
    wall_on_the_left: bool,
    cosine: f64,
    setback: f64,
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    station: Vector3,
) -> Result<i32, BevelAdvisory> {
    let not_supported = || {
        BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceCornerNotSupported,
            vec![station.into()],
        )
    };

    let creases = section_builder::try_creases_in_the_fan(
        vertex,
        arriving,
        leaving,
        wall_on_the_left,
        mesh,
        graph,
        edge_at_pair,
    )?;

    if creases.len() != 1 {
        // No fold at all, or several: the wall boundary has no one edge to ride and the corner would
        // have to fan.
        return Err(not_supported());
    }

    let fold = creases[0];

    let other = if graph.get_vertex0(fold) == vertex {
        graph.get_vertex1(fold)
    } else {
        graph.get_vertex0(fold)
    };
    let down = (Vector3::from_vector3_float(mesh.vertices[other as usize]) - station).get_normal();

    let arriving_lean = down.dot(away_in_the_wall(arriving, wall_on_the_left, cosine));
    let leaving_lean = down.dot(away_in_the_wall(leaving, wall_on_the_left, cosine));

    if arriving_lean < SAME_TO_WITHIN
        || leaving_lean < SAME_TO_WITHIN
        || ((setback / arriving_lean) - (setback / leaving_lean)).abs()
            > SAME_WEDGE_TO_WITHIN * ieee_math::max_f64(1.0, setback)
    {
        return Err(not_supported());
    }

    Ok(fold)
}

/// The shared FLAT side's fan at a corner - no crease (an offset), or one
/// SMOOTH fold of a tessellated surface to slide along - as the shared fold or -1. Several creases, or
/// one that is a real feature, are refused.
#[allow(clippy::too_many_arguments)]
pub fn try_the_shared_side(
    vertex: i32,
    arriving: &ChainEdge,
    leaving: &ChainEdge,
    flat_on_the_left: bool,
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    station: Vector3,
) -> Result<i32, BevelAdvisory> {
    let creases = section_builder::try_creases_in_the_fan(
        vertex,
        arriving,
        leaving,
        flat_on_the_left,
        mesh,
        graph,
        edge_at_pair,
    )?;

    if creases.len() > 1 || (creases.len() == 1 && !is_a_smooth_fold(mesh, graph, creases[0])) {
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceCornerNotSupported,
            vec![station.into()],
        ));
    }

    Ok(creases.first().copied().unwrap_or(-1))
}

/// Whether a manifold edge's two faces line up within the curve tolerance - one
/// smooth surface folded by its tessellation rather than a feature of the part.
pub fn is_a_smooth_fold(mesh: &Mesh, graph: &MeshEdgeGraph, edge: i32) -> bool {
    if graph.get_face_count(edge) != 2 {
        return false;
    }

    let one = Vector3::from_vector3_float(mesh.faces[graph.get_face(edge, 0) as usize].normal)
        .get_normal();
    let other = Vector3::from_vector3_float(mesh.faces[graph.get_face(edge, 1) as usize].normal)
        .get_normal();

    one.dot(other) > smooth_surface_dot()
}

/// Why [`try_build`] drew no corner: a refusal with an advisory, or - at a JUNCTION, which
/// `SectionJunction` draws - a decline with NO advisory ("handled elsewhere", never a refusal to
/// publish), or a point that is not a number.
#[derive(Debug, Clone, PartialEq)]
pub enum CornerRefusal {
    /// The station is a junction: not this rule's to draw, and nothing named.
    AJunction,
    /// The corner this rule cannot draw, named.
    Refused(BevelAdvisory),
    /// A profile point that is not a number.
    NotANumber(NanSectionPoint),
}

impl From<NanSectionPoint> for CornerRefusal {
    fn from(nan: NanSectionPoint) -> Self {
        CornerRefusal::NotANumber(nan)
    }
}

/// The mitre section at the vertex `leaving` starts from - the arriving edge's own
/// section slid onto the bisector plane, shared by both strips - or why there is none. The
/// operation's `is_a_feature_edge` is read ONLY to tell a junction from a corner this rule cannot
/// draw; the mitre's own fold reading keeps its fallback.
#[allow(clippy::too_many_arguments)]
pub fn try_build(
    arriving: &ChainEdge,
    leaving: &ChainEdge,
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    pool: &mut SectionPool,
    radius: f64,
    segments: i32,
    is_a_feature_edge: Option<&dyn Fn(i32) -> bool>,
) -> Result<BevelSection, CornerRefusal> {
    let vertex = leaving.from_vertex();
    let station = leaving.from();
    let refused = || {
        CornerRefusal::Refused(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceCornerNotSupported,
            vec![station.into()],
        ))
    };

    let Some((flat_on_the_left, cosine, wedge)) = try_read_the_sides(arriving, leaving) else {
        // Either the two edges share no flat side, or they meet it at different wedges. Both mean the
        // two canals do not cut the bisector plane in the same curve.
        if section_junction_build::is_a_junction_station(
            mesh,
            graph,
            arriving,
            leaving,
            vertex,
            is_a_feature_edge,
            Some(edge_at_pair),
        ) {
            // ONE SHARED FLAT SIDE AT TWO WEDGES is not a corner this rule failed at - it is a
            // JUNCTION, and `SectionJunction` draws it; no, and NOTHING named.
            return Err(CornerRefusal::AJunction);
        }

        return Err(refused());
    };

    let along_in = arriving.direction().get_normal();
    let along_out = leaving.direction().get_normal();
    let mut bisector = along_in + along_out;

    if bisector.length() < SAME_TO_WITHIN {
        // The chain doubles back on itself; there is no corner to bisect.
        return Err(refused());
    }

    bisector = bisector.get_normal();

    let lean = along_in.dot(bisector);

    if lean < SAME_TO_WITHIN {
        return Err(refused());
    }

    let setback = radius * (wedge / 2.0).tan();

    let fold = try_the_fold(
        vertex,
        arriving,
        leaving,
        !flat_on_the_left,
        cosine,
        setback,
        mesh,
        graph,
        edge_at_pair,
        station,
    )
    .map_err(CornerRefusal::Refused)?;
    let shared_fold = try_the_shared_side(
        vertex,
        arriving,
        leaving,
        flat_on_the_left,
        mesh,
        graph,
        edge_at_pair,
        station,
    )
    .map_err(CornerRefusal::Refused)?;

    let points = (segments.max(1) + 1) as usize;

    // The arriving edge's own straight-edge section at the vertex, slid onto the bisector plane - and
    // the LEAVING edge's, drawn only to be measured against it. By the reflection the two are one
    // curve when the corner is symmetric; where the shared side is a smooth wall rather than one plane
    // they part by a little, and how much is the whole question of whether one section serves both
    // strips.
    let drawn = slid_profile(arriving, station, bisector, radius, points);
    let seen_from_the_other_side = slid_profile(leaving, station, bisector, radius, points);
    let mut apart = 0.0;

    for k in 0..points {
        apart = ieee_math::max_f64(apart, (drawn[k] - seen_from_the_other_side[k]).length());
    }

    if apart > PROFILES_MEET_TO_WITHIN_THIS_MUCH_OF_THE_RADIUS * radius {
        // The two canals do not cut the bisector plane in the same curve, so there is no one section
        // to share and drawing the arriving edge's for both would leave the leaving strip off its own
        // canal by this much.
        return Err(refused());
    }

    let left = arriving.left_normal().get_normal();
    let right = arriving.right_normal().get_normal();
    let centre = station - ((left + right) * radius / (1.0 + cosine));

    let mut profile = Vec::with_capacity(points);

    for &at in &drawn {
        profile.push(pool.add(at)?);
    }

    let mitre_centre = centre - (along_in * ((centre - station).dot(bisector) / lean));
    let (left_kind, right_kind) = if flat_on_the_left {
        (shared_kind(shared_fold), SectionSide::Slide)
    } else {
        (SectionSide::Slide, shared_kind(shared_fold))
    };
    let (left_slide_edge, right_slide_edge) = if flat_on_the_left {
        (shared_fold, fold)
    } else {
        (fold, shared_fold)
    };

    let section = BevelSection::new(
        vertex,
        station,
        bisector,
        profile[0],
        profile[points - 1],
        profile,
        mitre_centre,
        mitre_centre,
        left_kind,
        right_kind,
        left_slide_edge,
        right_slide_edge,
        setback,
    )
    // The profile has at least two points and runs from its own first to its own last by
    // construction, so the constructor's refusal cannot be met.
    .map_err(|_| refused())?;

    Ok(BevelSection {
        kind: SectionKind::Corner,
        ..section
    })
}
