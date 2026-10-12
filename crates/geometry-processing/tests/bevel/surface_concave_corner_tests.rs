//! Port of `SurfaceConcaveCornerTests.cs`: the inside corner of a pocket, THREE concave edges
//! meeting at one vertex, all three selected. No junction reading sees it - `SurfaceMixedJunction`
//! passes over a vertex with no convex arm to press a ball against - so the three chain ends arrive
//! together at the corner patch, which reads it as the ball at REST on the three faces and closes
//! the spherical triangle between its three end rings.
//!
//! **What the corner really is.** The ball's centre is the convex octant's reflected to the void
//! side - `n_i.d = +r` instead of `-r` - which puts the three end rings' pairwise meetings on the
//! very tangent points the walkback already left on the faces ((10, 12, 12), (12, 10, 12) and
//! (12, 12, 10) on this block at r 2).
//!
//! **Both profiles, and only three concave arms.** A chamfer's three planes meet at a point
//! wherever they lean, so the same three arms MITRE and close without a patch at all. And a corner
//! with a crease among convex arms never reaches this patch: it is read as a mixed junction, which
//! is what the third case pins. C# `[Arguments]` rows are a loop.

use std::sync::Arc;

use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, SurfaceBevelPlan, SurfaceBuildFailure,
    SurfaceBuildOptions, SurfaceBuilt, SurfaceVertexPool,
};
use geometry_processing::bevel::{bevel_chain_linker, bevel_sliver_collapse, BevelAdvisoryCode};
use geometry_processing::topology::{EdgeConvexity, FaceRegionCache, FeatureEdge, FeatureEdgeSet};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

use crate::bevel_around_center_tip_dihedrals::worst_dihedral_degrees;
use crate::mesh_measurements::signed_volume;
use crate::surface_test_build::{assert_sound, expect_built, TryBevelOutcome};
use crate::topology_test_meshes::create_corner_pocket_block;

/// The block's own size, and the bite taken out of its far corner.
const BLOCK_SIZE: f64 = 20.0;

const POCKET_SIZE: f64 = 10.0;

/// The fillet radius, small enough that each bead has the whole of its 10 mm face to sit on.
const RADIUS: f64 = 2.0;

const SEGMENTS: i32 = 8;

/// The block less the bite - 8000 of solid with a 10 cube taken out of one corner.
const THE_PLAIN_BLOCK: f64 = 7000.0;

/// The pocket's inside corner, where the three concave edges meet.
fn corner() -> Vector3 {
    Vector3::new(10.0, 10.0, 10.0)
}

/// The ball at rest on the pocket's three faces: the convex octant's centre (8, 8, 8) reflected
/// through [`corner`], which is a radius out on the void side of each face.
fn the_ball_centre() -> Vector3 {
    Vector3::new(12.0, 12.0, 12.0)
}

/// How near a point read back OUT OF THE POOL stands to where the construction put it: the pool
/// keeps its vertices as floats, so a coordinate around 12 is held to about a ten-millionth.
const AS_NEAR_AS_A_FLOAT_HOLDS: f64 = 1e-6;

/// How far a rebuilt surface may fall from the smooth one it stands for -
/// `BevelControlMeshComparison.ChordTolerance` in the application's own test suite.
const THE_CHORD_TOLERANCE: f64 = 0.05;

/// Where that ball touches the three faces, which is where the end rings meet in pairs.
fn the_tangent_points() -> [Vector3; 3] {
    [
        Vector3::new(10.0, 12.0, 12.0),
        Vector3::new(12.0, 10.0, 12.0),
        Vector3::new(12.0, 12.0, 10.0),
    ]
}

/// The far ends of the three concave edges that run out of [`corner`].
fn the_three_arms() -> [Vector3; 3] {
    [
        Vector3::new(20.0, 10.0, 10.0),
        Vector3::new(10.0, 20.0, 10.0),
        Vector3::new(10.0, 10.0, 20.0),
    ]
}

/// All three beads, both profiles: the same three arms close either way, and each way to its OWN
/// closed form. The chamfer's solid is the cube corner's closed form mirrored: each of the three
/// fills `(L - d) * d^2 / 2` beyond the corner cell, and the cell gains `d^3 - d^3 / 4` - 7054
/// exactly at L 10 and d 2.
#[test]
fn three_concave_arms_close_for_a_fillet_and_mitre_for_a_chamfer() {
    for round_profile in [true, false] {
        let block = create_corner_pocket_block(BLOCK_SIZE, POCKET_SIZE);
        let features = FeatureEdgeSet::create(&block);

        for arm in the_three_arms() {
            assert_eq!(
                edge_between(&features, corner(), arm).convexity,
                EdgeConvexity::Concave,
                "the pocket is a bite out of the block, so every edge of its inside corner is a \
                 bead's crease - {arm:?} is not"
            );
        }

        let mesh = expect_built(
            try_bevel(corner(), &the_three_arms(), round_profile, RADIUS, SEGMENTS),
            "an inside corner of three arms closes at both profiles",
        );

        assert_sound(&mesh);

        if !round_profile {
            let filled = (3.0 * (POCKET_SIZE - RADIUS) * (RADIUS * RADIUS / 2.0))
                + (RADIUS * RADIUS * RADIUS)
                - (RADIUS * RADIUS * RADIUS / 4.0);
            let volume = signed_volume(&mesh);

            assert!(
                (volume - (THE_PLAIN_BLOCK + filled)).abs() <= THE_PLAIN_BLOCK * 1e-6,
                "three prisms beyond the corner cell, and the cell gaining what stands under all \
                 three chamfer planes - {} against {volume}",
                THE_PLAIN_BLOCK + filled
            );

            continue;
        }

        let rounded = THE_PLAIN_BLOCK
            + (3.0 * the_beads_section(RADIUS, SEGMENTS) * (POCKET_SIZE - RADIUS))
            + (RADIUS * RADIUS * RADIUS)
            - (std::f64::consts::PI * RADIUS * RADIUS * RADIUS / 6.0);
        let volume = signed_volume(&mesh);

        assert!(
            (volume - rounded).abs() <= 0.4 * (RADIUS / 3.0).powi(3),
            "three prisms of the bead's own section, and the cell filled round the ball at rest in \
             it - {rounded:.4} against {volume}"
        );

        assert!(
            rounded < 7054.0,
            "an arc fills less of a pocket corner than the chord across it does, which is the \
             chamfer's own number above"
        );
    }
}

/// The pocket corner CLOSES. Three concave beads hand over to the ball at REST on their three
/// faces, and the spherical triangle between their three end rings is laid as rows wound the other
/// way about. The patch is a polyhedron INSCRIBED in the ball, so the reading stands a shade ABOVE
/// the closed form, held to the convex rung's 0.4 scaled as `r^3`; the worst bend across it is
/// under twice the arc step.
#[test]
fn three_concave_beads_close_in_a_spherical_triangle() {
    for radius in [2.0, 3.0] {
        let centre = corner() + Vector3::new(radius, radius, radius);

        let mesh = expect_built(
            try_bevel(corner(), &the_three_arms(), true, radius, SEGMENTS),
            "the ball at rest on the pocket's three faces closes the corner between its three rings",
        );

        let (open, open_at) = surface_mesh_closure::open_edge_count(&mesh);

        assert_eq!(
            open, 0,
            "nothing may be left open, and something is at {open_at:?}"
        );
        assert_sound(&mesh);

        // The patch and nothing else lies in the corner cell.
        let patch = faces_inside_the_corner_cell(&mesh, centre);

        assert_eq!(
            patch.len() as i32,
            (4 * SEGMENTS * SEGMENTS) - (3 * SEGMENTS),
            "the grid on its arcs' own sample count is n squared triangles, and one level of \
             INTERIOR refinement cuts each of them in four but for the edges of the RIM - 4n \
             squared less 3n (radius {radius})"
        );

        let on_the_patch = distinct_corners(&patch);

        assert_eq!(
            on_the_patch.len() as i32,
            (2 * SEGMENTS * SEGMENTS) + 1,
            "the grid's own (n+1)(n+2)/2 points and one midpoint per INTERIOR edge, which is 2n \
             squared + 1 (radius {radius})"
        );

        for &vertex in &on_the_patch {
            let at = Vector3::from_vector3_float(mesh.vertices[vertex as usize]);

            assert!(
                ((at - centre).length() - radius).abs() < AS_NEAR_AS_A_FLOAT_HOLDS,
                "every point of the patch stands a radius from the ball at rest - {at:?} is {} \
                 from {centre:?}",
                (at - centre).length()
            );
        }

        for tangent in the_tangent_points_around(centre, radius) {
            assert_eq!(
                count_at(&mesh, tangent),
                1,
                "the ball touches each face at one point, minted once and looked up after - \
                 {tangent:?} is not one vertex of the published mesh"
            );
        }

        // No sliver: the construction shares vertices rather than welding afterwards.
        let weld = SurfaceVertexPool::weld_distance_for(&mesh.vertices);
        let (thinnest, where_) = the_thinnest_face_in(&mesh);

        assert!(
            thinnest > weld,
            "nothing published may be thinner than the weld distance {weld:e} - instead \
             {thinnest:e} at {where_}"
        );

        let section = the_beads_section(radius, SEGMENTS);
        let ball = std::f64::consts::PI * radius * radius * radius / 6.0;
        let expected =
            THE_PLAIN_BLOCK + (3.0 * section * (POCKET_SIZE - radius)) + (radius * radius * radius)
                - ball;

        // The construction is similar about the centre, so the inscribed deficit's bound scales
        // with r^3.
        let inscribed_octant = 0.4 * (radius / 3.0).powi(3);
        let volume = signed_volume(&mesh);

        assert!(
            (volume - expected).abs() <= inscribed_octant,
            "three exact prisms and one cell, the cell filled round an octant the patch can only \
             inscribe - {expected:.4} against {volume}"
        );

        assert!(
            volume > expected,
            "an inscribed patch leaves LESS void in the cell than the smooth ball does, so a pocket \
             can only be filled a shade too full"
        );

        let (worst, _edges, offenders) =
            worst_dihedral_degrees(&mesh, inside_the_corner_cell(centre));

        assert!(
            worst < 2.0 * 90.0 / SEGMENTS as f64,
            "the patch is one sphere chorded {SEGMENTS} times a quarter, and nothing on it may fold \
             - {offenders}"
        );
    }
}

/// No triangle of the patch lies further inside the ball than the chord tolerance the fixtures are
/// read against. A triangle inscribed in a sphere lies deepest inside it at the centre of its
/// MIN-CONTAINMENT circle, `r - sqrt(r^2 - rho^2)`. Radius 5, the scale the fixtures the tolerance
/// was written for are bevelled at.
#[test]
fn every_patch_triangle_lies_within_the_chord_tolerance_of_the_ball() {
    for segments in [8, 16] {
        let radius = 5.0;

        let centre = corner() + Vector3::new(radius, radius, radius);

        let mesh = expect_built(
            try_bevel(corner(), &the_three_arms(), true, radius, segments),
            &format!("the pocket corner closes at r {radius} as it does at any other"),
        );

        let patch = faces_inside_the_corner_cell(&mesh, centre);

        assert!(
            !patch.is_empty(),
            "the corner cell holds the patch, and an empty cell means there is nothing here to \
             measure"
        );

        let mut worst = 0.0;
        let mut where_ = String::from("(none)");
        let mut over = 0;

        for &(v0, v1, v2) in &patch {
            let corners = [v0, v1, v2].map(|v| mesh.vertices[v as usize]);
            let sag = the_deepest_it_lies_inside_the_ball(
                radius,
                Vector3::from_vector3_float(corners[0]),
                Vector3::from_vector3_float(corners[1]),
                Vector3::from_vector3_float(corners[2]),
            );

            if sag > THE_CHORD_TOLERANCE {
                over += 1;
            }

            if sag > worst {
                worst = sag;
                where_ = format!("{corners:?}");
            }
        }

        assert_eq!(
            over,
            0,
            "nothing published may fall further than {THE_CHORD_TOLERANCE} inside the ball it is \
             cut on - {over} of {} faces do, the worst by {worst:.4} at {where_} ({segments} \
             segments)",
            patch.len()
        );
    }
}

/// TWO of the same three creases, filleted: they are ONE chain through the corner rather than two
/// ends arriving at it, so the corner is an interior MITRE and the patch is never asked.
#[test]
fn two_of_the_same_creases_mitre_through_the_corner_and_close() {
    let mesh = expect_built(
        try_bevel(corner(), &the_three_arms()[..2], true, RADIUS, SEGMENTS),
        "two creases sharing a vertex link into one run, whose corner mitres like any other",
    );

    assert_sound(&mesh);

    assert!(
        signed_volume(&mesh) > THE_PLAIN_BLOCK,
        "a bead FILLS an inside corner, so the two of them can only leave more material than the \
         block had"
    );
}

/// A three-arm corner with a CREASE among convex arms is not read here: it never reaches the
/// patch. The same block's OTHER pocket corner serves: at (20, 10, 10) the pocket floor's crease
/// arrives from (10, 10, 10) and the two rims of the bite on the block's own `x = 20` face arrive
/// convex. Filleted at the same 2 mm it builds.
#[test]
fn a_three_arm_corner_with_a_convex_arm_is_read_as_a_junction_and_not_named_here() {
    let block = create_corner_pocket_block(BLOCK_SIZE, POCKET_SIZE);
    let features = FeatureEdgeSet::create(&block);
    let mixed_corner = Vector3::new(20.0, 10.0, 10.0);

    let arms = [
        Vector3::new(10.0, 10.0, 10.0),
        Vector3::new(20.0, 20.0, 10.0),
        Vector3::new(20.0, 10.0, 20.0),
    ];

    let convexities: Vec<EdgeConvexity> = arms
        .iter()
        .map(|&arm| edge_between(&features, mixed_corner, arm).convexity)
        .collect();

    assert_eq!(
        convexities
            .iter()
            .filter(|&&at| at == EdgeConvexity::Concave)
            .count(),
        1,
        "one arm of this corner is the pocket's own floor crease - instead {convexities:?}"
    );
    assert_eq!(
        convexities
            .iter()
            .filter(|&&at| at == EdgeConvexity::Convex)
            .count(),
        2,
        "and the other two are the bite's rims on the outside of the block - instead \
         {convexities:?}"
    );

    let outcome = try_bevel(mixed_corner, &arms, true, RADIUS, SEGMENTS);

    assert_ne!(
        outcome.reason().map(|reason| reason.code()),
        Some(BevelAdvisoryCode::SurfaceJunctionOfThreeCreases),
        "a corner with a convex arm is not three creases, whatever else it may be - {:?}",
        outcome.reason()
    );

    let mesh = expect_built(
        outcome,
        "one crease and two cuts at a vertex is the mixed junction, which builds",
    );

    assert_sound(&mesh);
}

/// The three END RINGS, read off the PUBLISHED mesh: each vertex a radius from the ball at rest,
/// and each pair of rings meeting at one of the three points the ball touches the faces at. The rim
/// is the patch's own vertices standing on one of the cell's walls, `3 * (segments + 1) - 3` of
/// them, counted as mesh INDICES, which is what says the sharing is real.
#[test]
fn the_three_end_rings_stand_on_the_void_ball() {
    let mesh = expect_built(
        try_bevel(corner(), &the_three_arms(), true, RADIUS, SEGMENTS),
        "the corner closes on the ball at rest in the pocket",
    );

    for tangent in the_tangent_points() {
        assert_eq!(
            count_at(&mesh, tangent),
            1,
            "the ball touches the three faces at {tangent:?}, and the two rings that end there \
             meet at ONE vertex of the mesh"
        );
    }

    let on_the_rim: Vec<i32> =
        distinct_corners(&faces_inside_the_corner_cell(&mesh, the_ball_centre()))
            .into_iter()
            .filter(|&vertex| {
                on_a_wall_of_the_corner_cell(
                    Vector3::from_vector3_float(mesh.vertices[vertex as usize]),
                    the_ball_centre(),
                )
            })
            .collect();

    assert_eq!(
        on_the_rim.len() as i32,
        (3 * (SEGMENTS + 1)) - 3,
        "three rings of {} rails sharing their six ends in pairs",
        SEGMENTS + 1
    );

    for &vertex in &on_the_rim {
        let at = Vector3::from_vector3_float(mesh.vertices[vertex as usize]);

        assert!(
            ((at - the_ball_centre()).length() - RADIUS).abs() < AS_NEAR_AS_A_FLOAT_HOLDS,
            "every ring vertex is cut on that one ball - {at:?} is {} from it",
            (at - the_ball_centre()).length()
        );
    }
}

/// How many vertices of the mesh stand at one point.
fn count_at(mesh: &Mesh, at: Vector3) -> usize {
    mesh.vertices
        .iter()
        .filter(|&&vertex| {
            (Vector3::from_vector3_float(vertex) - at).length() < AS_NEAR_AS_A_FLOAT_HOLDS
        })
        .count()
}

/// The faces' corners, each once, in the order C#'s `SelectMany(...).Distinct()` meets them.
fn distinct_corners(faces: &[(i32, i32, i32)]) -> Vec<i32> {
    let mut seen = Vec::new();

    for &(v0, v1, v2) in faces {
        for vertex in [v0, v1, v2] {
            if !seen.contains(&vertex) {
                seen.push(vertex);
            }
        }
    }

    seen
}

/// Whether a point stands on one of the three walls of the corner cell, which is where a ring is.
fn on_a_wall_of_the_corner_cell(at: Vector3, centre: Vector3) -> bool {
    const SLACK: f64 = 1e-5;

    (at.x - centre.x).abs() < SLACK
        || (at.y - centre.y).abs() < SLACK
        || (at.z - centre.z).abs() < SLACK
}

/// The faces lying wholly inside the CORNER CELL - the cube between [`corner`] and the ball centre.
fn faces_inside_the_corner_cell(mesh: &Mesh, centre: Vector3) -> Vec<(i32, i32, i32)> {
    let inside = inside_the_corner_cell(centre);
    let at = |v: i32| Vector3::from_vector3_float(mesh.vertices[v as usize]);

    mesh.faces
        .iter()
        .filter(|face| inside(at(face.v0)) && inside(at(face.v1)) && inside(at(face.v2)))
        .map(|face| (face.v0, face.v1, face.v2))
        .collect()
}

/// Whether a point is in the corner cell, read with a float's own slack so that a ring vertex ON
/// one of the cell's walls counts as in it.
fn inside_the_corner_cell(centre: Vector3) -> impl Fn(Vector3) -> bool {
    const SLACK: f64 = 1e-5;
    let corner = corner();

    move |at| {
        at.x >= corner.x - SLACK
            && at.x <= centre.x + SLACK
            && at.y >= corner.y - SLACK
            && at.y <= centre.y + SLACK
            && at.z >= corner.z - SLACK
            && at.z <= centre.z + SLACK
    }
}

/// The three points the ball at rest touches the pocket's faces at, which are the rings' own ends.
fn the_tangent_points_around(centre: Vector3, radius: f64) -> [Vector3; 3] {
    [
        centre - Vector3::new(radius, 0.0, 0.0),
        centre - Vector3::new(0.0, radius, 0.0),
        centre - Vector3::new(0.0, 0.0, radius),
    ]
}

/// One bead's cross section at a right-angled crease: the kite between the crease and its two
/// tangent lines less the ball's sector, plus what an inscribed chording of the arc adds.
fn the_beads_section(radius: f64, segments: i32) -> f64 {
    const WEDGE: f64 = std::f64::consts::PI / 2.0;

    let kite = (radius * radius / (WEDGE / 2.0).tan())
        - (radius * radius * (std::f64::consts::PI - WEDGE) / 2.0);
    let excess =
        radius * radius / 2.0 * (WEDGE - (segments as f64 * (WEDGE / segments as f64).sin()));

    kite + excess
}

/// The deepest a triangle inscribed in a ball of `radius` lies inside it: `r - sqrt(r^2 - rho^2)`
/// with rho the radius of the smallest circle containing the triangle.
fn the_deepest_it_lies_inside_the_ball(radius: f64, a: Vector3, b: Vector3, c: Vector3) -> f64 {
    let opposite_a = (b - c).length();
    let opposite_b = (c - a).length();
    let opposite_c = (a - b).length();

    let longest = opposite_a.max(opposite_b.max(opposite_c));
    let twice_the_area = (b - a).cross(c - a).length();

    // Obtuse when the longest edge's square beats the other two together, and then the
    // circumcentre stands outside the triangle and it is the edge itself that contains it.
    let acute = twice_the_area > 0.0
        && (2.0 * longest * longest)
            <= (opposite_a * opposite_a) + (opposite_b * opposite_b) + (opposite_c * opposite_c);

    let containing = if acute {
        opposite_a * opposite_b * opposite_c / (2.0 * twice_the_area)
    } else {
        longest / 2.0
    };

    if containing >= radius {
        radius
    } else {
        radius - ((radius * radius) - (containing * containing)).sqrt()
    }
}

/// The smallest altitude any face of the mesh has, and where it is, for a failure message.
fn the_thinnest_face_in(mesh: &Mesh) -> (f64, String) {
    let mut thinnest = f64::INFINITY;
    let mut where_ = String::from("(no faces)");

    for face in mesh.faces.iter() {
        let corners = [face.v0, face.v1, face.v2].map(|v| mesh.vertices[v as usize]);
        let altitude = bevel_sliver_collapse::altitude_of(corners[0], corners[1], corners[2]);

        if altitude < thinnest {
            thinnest = altitude;
            where_ = format!("{corners:?}");
        }
    }

    (thinnest, where_)
}

/// The pocket block with the edges running from `corner` to the given points bevelled by the
/// surface construction, as one chain per linked run.
fn try_bevel(
    corner: Vector3,
    arms: &[Vector3],
    round_profile: bool,
    radius: f64,
    segments: i32,
) -> TryBevelOutcome {
    let block = create_corner_pocket_block(BLOCK_SIZE, POCKET_SIZE);
    let features = FeatureEdgeSet::create(&block);
    let graph = block.get_mesh_edge_graph();
    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(block.clone()),
        Some(graph.clone()),
        features.flat_angle_degrees(),
    ));

    let edges: Vec<FeatureEdge> = arms
        .iter()
        .map(|&arm| edge_between(&features, corner, arm))
        .collect();
    let chains = bevel_chain_linker::link(&edges);
    let mut plans = Vec::with_capacity(chains.len());

    for chain in &chains {
        plans.push(
            SurfaceBevelPlan::try_create(chain, &regions, radius, round_profile)
                .unwrap_or_else(|reason| panic!("the plan refused the chain: {reason}")),
        );
    }

    // A chamfer's profile is one chord, so its strips are sampled once whatever the fillet's count.
    let sampled_at = if round_profile { segments } else { 1 };

    let result: Result<SurfaceBuilt, SurfaceBuildFailure> = surface_bevel_builder::try_build(
        &block,
        &graph,
        &regions,
        &plans,
        sampled_at,
        &CancelToken::new(),
        SurfaceBuildOptions::default(),
    );

    TryBevelOutcome {
        plans_built: Vec::new(),
        result,
    }
}

/// The one feature edge running between two given points, either way about.
fn edge_between(features: &FeatureEdgeSet, from: Vector3, to: Vector3) -> FeatureEdge {
    let found: Vec<&FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| {
            ((edge.start - from).length() < 1e-6 && (edge.end - to).length() < 1e-6)
                || ((edge.start - to).length() < 1e-6 && (edge.end - from).length() < 1e-6)
        })
        .collect();

    assert_eq!(found.len(), 1, "one edge runs from {from:?} to {to:?}");

    *found[0]
}
