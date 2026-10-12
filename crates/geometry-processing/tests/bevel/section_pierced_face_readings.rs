//! The private helpers of the partial class `SectionPiercedFaceTests` (`SectionPiercedFaceTests.cs`
//! and its `.Junction`/`.Pass`/`.Seam`/`.Takeover` parts): the top face's pick and the plans it
//! opens into through the sections (`ThePlans`, `TheTopFace`, `TheTopFaceAt`), the junction's
//! geometric readings (`Wedge`, `TheFoldAt`, `TheCreaseAt`, `TheAxisAt`, `DistanceToLine`,
//! `SlidOntoThePlane`, `MeetOfLines`), the junction section built through the production rule
//! (`TheJunctionAt`), and the face readings the junction parts share (`TheOperationsFeatureReading`,
//! `TheNormalOf`, `TheTriangleHolds`). One copy here, so every part of the class reads the pick the
//! same way, as C#'s one class does. `TheSharpTurnsOf` is [`crate::section_turn_readings`]'s.

use std::collections::HashSet;
use std::rc::Rc;

use geometry_processing::bevel::sections::section_builder;
use geometry_processing::bevel::sections::section_junction_build;
use geometry_processing::bevel::sections::{section_routing, JunctionSection, SectionPool};
use geometry_processing::bevel::surface::SurfaceBevelPlan;
use geometry_processing::bevel::BevelAdvisory;
use geometry_processing::topology::{
    bevel_edge_group_resolver, BevelEdgeGroup, ChainEdge, FaceGroup, FeatureEdge, FeatureEdgeSet,
};
use polygon_mesh::{CancelToken, Mesh, MeshEdgeGraph};
use vector_math::math_helper::radians_to_degrees;
use vector_math::Vector3;

use crate::inner_face_test_mesh;
use crate::section_turn_readings::the_sharp_turns_of;
use crate::surface_test_build::{pinned_to_the_sections, try_bevel, TryBevelOptions};

/// C# `Radius`: the radius the junction is read at - half the 1 mm the reading is quoted for.
pub const RADIUS: f64 = 0.5;

/// C# `ThePlans`: the plans the top face's pick opens into, built through the sections, with the
/// route the router took and the advisory the build named. `up` picks the top face of a moved part.
pub fn the_plans(
    part: &Mesh,
    up: Option<Vector3>,
) -> (
    Vec<Rc<SurfaceBevelPlan>>,
    Option<String>,
    Option<BevelAdvisory>,
) {
    let _sections = pinned_to_the_sections();

    let outcome = try_bevel(
        part,
        &[the_top_face(part, up)],
        2.0 * RADIUS,
        inner_face_test_mesh::SEGMENTS,
        &CancelToken::new(),
        TryBevelOptions {
            round_profile: true,

            // The junction screen is only readable with the OPERATION's own feature reading: on
            // this tessellated bore the coplanarity fallback calls every facet fold a crease, the
            // wall-side fan then reads TWO at vertex 19, and the junction is missed - so the router
            // keeps the wedge screen and the pass goes legacy. Measured: without this the route is
            // "legacy: the chain turns 86.81 degrees at vertex 19"; with it, "sections".
            carry_the_feature_reading: true,
        },
    );

    let reason = outcome.reason().cloned();
    let route = section_routing::last_route();

    (outcome.plans_built, route, reason)
}

/// C# `TheTopFace`: the cube's TOP face as a face-group pick, flooded the way a click on it would
/// be.
pub fn the_top_face(part: &Mesh, up: Option<Vector3>) -> BevelEdgeGroup {
    let graph = part.get_mesh_edge_graph();
    let features = FeatureEdgeSet::create_with_graph(
        part,
        &graph,
        inner_face_test_mesh::BEND_THRESHOLD_DEGREES,
    );
    let up = up.unwrap_or(Vector3::UNIT_Z);

    for (face, drawn) in part.faces.iter().enumerate() {
        if Vector3::from_vector3_float(drawn.normal)
            .get_normal()
            .dot(up)
            < 0.999
        {
            continue;
        }

        let region =
            FaceGroup::create_with_graph(part, &graph, face as i32, features.flat_angle_degrees())
                .expect("the face is a face of the part");

        if region.boundary_edges().is_empty() {
            continue;
        }

        return bevel_edge_group_resolver::create_from_face_group(
            &region,
            &region.boundary_edges()[0],
            features.flat_angle_degrees(),
        )
        .expect("the anchor bounds the region it was read from");
    }

    panic!("the part has no top face");
}

/// The sharp turn whose leaving edge starts at `vertex` - C#'s `TheSharpTurnsOf(plans).Single(...)`.
pub fn the_turn_at(plans: &[Rc<SurfaceBevelPlan>], vertex: i32) -> (ChainEdge, ChainEdge) {
    let turns: Vec<_> = the_sharp_turns_of(plans)
        .into_iter()
        .filter(|(_, leaving)| leaving.from_vertex() == vertex)
        .collect();

    assert_eq!(turns.len(), 1, "one sharp turn at vertex {vertex}");

    turns[0]
}

/// C# `Wedge`: the wedge one chain edge stands at, in degrees.
pub fn wedge(edge: &ChainEdge) -> f64 {
    radians_to_degrees(
        edge.left_normal()
            .get_normal()
            .dot(edge.right_normal().get_normal())
            .clamp(-1.0, 1.0)
            .acos(),
    )
}

/// C# `TheFoldAt`: the one crease leaving the junction that is not the chain - the mouth's
/// continuation down the -X wall - as a unit direction pointing away from the vertex.
pub fn the_fold_at(part: &Mesh, arriving: &ChainEdge, leaving: &ChainEdge) -> Vector3 {
    let station = leaving.from();
    let crease = the_crease_at(part, arriving, leaving);
    let down = crease.direction.get_normal();

    if (Vector3::from_vector3_float(part.vertices[crease.start_vertex as usize]) - station).length()
        > 1e-6
    {
        -down
    } else {
        down
    }
}

/// C# `TheCreaseAt`: the fold as the part's own feature edge, which carries the TWO FACES it joins:
/// the cube's -X wall and the bore facet that abuts it, which are the two planes bounding the part
/// at the junction. The bore facet the mouth CHORD stands on is a different one, 6.36 degrees
/// round.
pub fn the_crease_at(part: &Mesh, arriving: &ChainEdge, leaving: &ChainEdge) -> FeatureEdge {
    let features = FeatureEdgeSet::create_with_graph(
        part,
        &part.get_mesh_edge_graph(),
        inner_face_test_mesh::BEND_THRESHOLD_DEGREES,
    );

    let creases: Vec<&FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| {
            (edge.start_vertex == leaving.from_vertex() || edge.end_vertex == leaving.from_vertex())
                && edge.start_vertex != arriving.from_vertex()
                && edge.end_vertex != arriving.from_vertex()
                && edge.start_vertex != leaving.to_vertex()
                && edge.end_vertex != leaving.to_vertex()
        })
        .collect();

    assert_eq!(creases.len(), 1, "one crease leaves the junction");

    *creases[0]
}

/// C# `TheAxisAt`: where one edge's canal axis stands abreast of the vertex.
pub fn the_axis_at(edge: &ChainEdge, station: Vector3, radius: f64) -> Vector3 {
    let one = edge.left_normal().get_normal();
    let other = edge.right_normal().get_normal();

    station - (radius * (one + other) / (1.0 + one.dot(other)))
}

/// C# `DistanceToLine`: how far a point stands off a line.
pub fn distance_to_line(point: Vector3, from: Vector3, along: Vector3) -> f64 {
    let offset = point - from;

    (offset - (offset.dot(along) * along)).length()
}

/// C# `SlidOntoThePlane`: one edge's own straight-edge profile at the station, slid along that edge
/// onto a plane - `SectionCorner.SlidProfile`'s rule, taken through a point that need not be the
/// vertex.
pub fn slid_onto_the_plane(
    edge: &ChainEdge,
    station: Vector3,
    through: Vector3,
    normal: Vector3,
    radius: f64,
    points: usize,
) -> Vec<Vector3> {
    let along = edge.direction().get_normal();
    let lean = along.dot(normal);
    let one = edge.left_normal().get_normal();
    let other = edge.right_normal().get_normal();
    let cosine = one.dot(other);
    let wedge = cosine.clamp(-1.0, 1.0).acos();
    let centre = station - (radius * (one + other) / (1.0 + cosine));

    (0..points)
        .map(|k| {
            let swept = centre
                + (radius
                    * section_builder::slerp(one, other, k as f64 / (points - 1) as f64, wedge));

            swept - (((swept - through).dot(normal) / lean) * along)
        })
        .collect()
}

/// C# `MeetOfLines`: the point two lines meet at, taken as the midpoint of their closest approach,
/// with C#'s `out skew` - how far apart the two closest points stand.
pub fn meet_of_lines(
    from0: Vector3,
    along0: Vector3,
    from1: Vector3,
    along1: Vector3,
) -> (Vector3, f64) {
    let lean = along0.dot(along1);
    let denominator = 1.0 - (lean * lean);
    let between = from1 - from0;
    let step0 = (between.dot(along0) - (lean * between.dot(along1))) / denominator;
    let step1 = ((lean * between.dot(along0)) - between.dot(along1)) / denominator;
    let one = from0 + (step0 * along0);
    let other = from1 + (step1 * along1);

    ((one + other) / 2.0, (one - other).length())
}

/// C# `TheJunctionAt`: the junction section at one of the two vertices, built through the
/// production rule with the fold read off the part the way the builder will have to read it, and
/// the pool it was minted into.
pub fn the_junction_at(
    part: &Mesh,
    plans: &[Rc<SurfaceBevelPlan>],
    vertex: i32,
    radius: f64,
    segments: i32,
) -> (JunctionSection, SectionPool) {
    let (arriving, leaving) = the_turn_at(plans, vertex);

    let mut pool = SectionPool::new(&part.vertices);

    let section = section_junction_build::try_build(
        &arriving,
        &leaving,
        the_fold_at(part, &arriving, &leaving),
        &mut pool,
        radius,
        segments,
        None,
    )
    .unwrap_or_else(|failure| panic!("the junction at {vertex} was refused: {failure:?}"));

    (section, pool)
}

/// C# `TheOperationsFeatureReading`: the OPERATION's own reading of which mesh edges are features,
/// which is what the junction's fold has to be read with - on this part's tessellated bore the
/// fallback coplanarity test calls a facet fold a crease, and the wall-side fan then reads two.
pub fn the_operations_feature_reading(
    part: &Mesh,
    graph: &MeshEdgeGraph,
) -> Rc<dyn Fn(i32) -> bool> {
    let features = FeatureEdgeSet::create_with_graph(
        part,
        graph,
        inner_face_test_mesh::BEND_THRESHOLD_DEGREES,
    );
    let edges: HashSet<i32> = features
        .edges()
        .iter()
        .map(|one| one.mesh_edge_index)
        .collect();

    Rc::new(move |edge| edges.contains(&edge))
}

/// C# `TheNormalOf`: one face's plane normal, taken from its own corners rather than the stored
/// float.
pub fn the_normal_of(part: &Mesh, face: i32) -> Vector3 {
    let drawn = &part.faces[face as usize];
    let a = Vector3::from_vector3_float(part.vertices[drawn.v0 as usize]);

    (Vector3::from_vector3_float(part.vertices[drawn.v1 as usize]) - a)
        .cross(Vector3::from_vector3_float(part.vertices[drawn.v2 as usize]) - a)
        .get_normal()
}

/// C# `TheTriangleHolds`: whether a point's projection onto a face's plane stands inside that
/// face's triangle, to within a DISTANCE - the reading the carry-out itself makes, asked here of
/// the part.
pub fn the_triangle_holds(part: &Mesh, face: i32, at: Vector3, slack: f64) -> bool {
    let drawn = &part.faces[face as usize];
    let corners = [
        Vector3::from_vector3_float(part.vertices[drawn.v0 as usize]),
        Vector3::from_vector3_float(part.vertices[drawn.v1 as usize]),
        Vector3::from_vector3_float(part.vertices[drawn.v2 as usize]),
    ];

    let normal = the_normal_of(part, face);

    for k in 0..3 {
        let a = corners[k];
        let along = corners[(k + 1) % 3] - a;
        let length = along.length();

        if length < 1e-300 {
            return false;
        }

        if along.cross(at - a).dot(normal) / length < -slack {
            return false;
        }
    }

    true
}
