//! Rung 3 of the polygon-space bevel plan, bevels that meet at a
//! VERTEX of the part. Two edges of a cube corner first, which is one chain mitred at its interior
//! corner, and then all three, which is three chains handing over to one corner patch.
//!
//! **Two edges is a mitre and nothing else, and the pair corner is that mitre.** Measured in the
//! corner's own coordinates the boundary of the two chamfers' union is the bisector plane, which is
//! the mitre; the fillet says the same. So NO patch belongs at a two-edge corner and no face has its
//! corner clipped twice.
//!
//! **Three edges is where a patch is owed.** There the three strips cannot meet each other pairwise
//! and close, so each strip ends on the plane a radius in from the third face and the spherical
//! octant about the ball centre fills what is left. The parameter rows are a loop.

use std::sync::Arc;

use geometry_processing::bevel::bevel_seam_ring;
use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_corner_mitre, surface_corner_patch_rows, surface_mesh_closure,
    SurfaceBevelPlan, SurfaceBuildFailure, SurfaceBuildOptions, SurfaceVertexPool,
};
use geometry_processing::bevel::{bevel_chain_linker, BevelAdvisoryCode, BevelSweptChain};
use geometry_processing::topology::{FaceRegionCache, FeatureEdge, FeatureEdgeSet};
use polygon_mesh::solids::platonic_solids::create_cube;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::{Vector2, Vector3, Vector3Float};

use crate::mesh_measurements::{shell_count, signed_volume};
use crate::surface_test_chains::join_and_open;

const CUBE_SIZE: f64 = 20.0;

const SIZE: f64 = 3.0;

const SEGMENTS: i32 = 8;

/// Where the two or three bevelled edges meet, in the cube's own coordinates.
fn corner() -> Vector3 {
    Vector3::new(CUBE_SIZE / 2.0, -CUBE_SIZE / 2.0, CUBE_SIZE / 2.0)
}

fn assert_within(actual: f64, expected: f64, tolerance: f64, because: &str) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual} is not {expected} within {tolerance}: {because}"
    );
}

/// Two edges of one cube corner, chamfered as one open chain of two edges: the interior corner
/// takes the same mitre a closed rim's corner takes, and the two far ends are ordinary end notches.
/// `8000 - (2 * 20 * 4.5 - 9) = 7829` by inclusion and exclusion on the two prisms.
#[test]
fn two_cube_edges_at_a_corner_chamfer_into_one_mitre() {
    let built = build(two_edges_at_the_corner, SIZE, false, 1);

    assert_it_is_a_sound_solid(&built);

    let removed = (2.0 * CUBE_SIZE * (SIZE * SIZE / 2.0)) - (SIZE * SIZE * SIZE / 3.0);

    assert_within(
        signed_volume(&built),
        8000.0 - removed,
        8000.0 * 1e-6,
        "two swept sections less the one corner the mitre stops being counted twice",
    );

    // The side faces are each cut by ONE strip, which is the whole of what the pair corner comes
    // to: 20 wide by 20 - d deep, with no second lens taken out of it anywhere.
    assert_within(
        area_on(&built, |at| (at.x as f64 - (CUBE_SIZE / 2.0)).abs() < 1e-5),
        CUBE_SIZE * (CUBE_SIZE - SIZE),
        1e-6,
        "the x side face",
    );
    assert_within(
        area_on(&built, |at| (at.y as f64 + (CUBE_SIZE / 2.0)).abs() < 1e-5),
        CUBE_SIZE * (CUBE_SIZE - SIZE),
        1e-6,
        "the y side face",
    );
}

/// The same two edges filleted. The closed form is the chamfer's worked over the chorded profile,
/// which is exact for the tessellation the strip really builds.
#[test]
fn two_cube_edges_at_a_corner_fillet_into_one_mitre() {
    let built = build(two_edges_at_the_corner, SIZE, true, SEGMENTS);

    assert_it_is_a_sound_solid(&built);

    let profile = fillet_profile(SIZE, std::f64::consts::PI / 2.0, SEGMENTS);
    let removed = (2.0 * CUBE_SIZE * section_area(&profile)) - mitred_corner_overlap(&profile);

    assert_within(
        signed_volume(&built),
        8000.0 - removed,
        8000.0 * 1e-6,
        "the mitre is exact on the chorded profile as much as on the chamfer's one chord",
    );

    // Each side face keeps everything below its own tangent line and nothing else.
    assert_within(
        area_on(&built, |at| (at.x as f64 - (CUBE_SIZE / 2.0)).abs() < 1e-5),
        CUBE_SIZE * (CUBE_SIZE - SIZE),
        1e-6,
        "the x side face",
    );
    assert_within(
        area_on(&built, |at| (at.y as f64 + (CUBE_SIZE / 2.0)).abs() < 1e-5),
        CUBE_SIZE * (CUBE_SIZE - SIZE),
        1e-6,
        "the y side face",
    );
}

/// THE MITRE PLANE'S OWN GUARD: where the two beads set back by the SAME amount on the face they
/// share, the plane through the crossing of their two tangent lines is the plane through the corner
/// vertex, so a reading that moves to the crossing must leave this fixture alone to the last bit.
/// The snapshot is every vertex of the fillet build on `x + y = 0`, at full precision; the
/// tolerance is 1e-9 on purpose.
#[test]
fn an_equal_setback_mitre_is_unchanged() {
    let built = build(two_edges_at_the_corner, SIZE, true, SEGMENTS);

    let snapshot = [
        Vector3::new(-10.0, 10.0, -10.0),
        Vector3::new(10.0, -10.0, -10.0),
        Vector3::new(10.0, -10.0, 7.0),
        Vector3::new(9.94235610961914, -9.94235610961914, 7.585270881652832),
        Vector3::new(9.771638870239258, -9.771638870239258, 8.148050308227539),
        Vector3::new(9.49440860748291, -9.49440860748291, 8.66671085357666),
        Vector3::new(9.121320724487305, -9.121320724487305, 9.121320724487305),
        Vector3::new(8.66671085357666, -8.66671085357666, 9.49440860748291),
        Vector3::new(8.148050308227539, -8.148050308227539, 9.771638870239258),
        Vector3::new(7.585270881652832, -7.585270881652832, 9.94235610961914),
        Vector3::new(-10.0, 10.0, 10.0),
        Vector3::new(0.0, 0.0, 10.0),
        Vector3::new(7.0, -7.0, 10.0),
    ];

    // First occurrences kept, then a stable sort by z then x, so ties keep their walked order.
    let mut on_the_plane: Vec<Vector3> = Vec::new();
    for vertex in built
        .vertices
        .iter()
        .map(|&v| Vector3::from_vector3_float(v))
    {
        if (vertex.x + vertex.y).abs() < 1e-9 && !on_the_plane.contains(&vertex) {
            on_the_plane.push(vertex);
        }
    }
    on_the_plane.sort_by(|a, b| a.z.total_cmp(&b.z).then(a.x.total_cmp(&b.x)));

    assert_eq!(
        on_the_plane.len(),
        snapshot.len(),
        "the mitre ring and the corners it shares its plane with are these and no others - found {on_the_plane:?}"
    );

    for (found, expected) in on_the_plane.iter().zip(snapshot.iter()) {
        assert!(
            (*found - *expected).length() < 1e-9,
            "the equal setback mitre may not move: {found:?} stands where {expected:?} did"
        );
    }
}

/// All three edges of one cube corner, chamfered: three chains, all planned on the original mesh
/// and walked back together. Three chamfers mitre to the one point `(d/2, d/2, d/2)` and take no
/// patch; the solid is 7750.25, the boolean construction's own reading of the same corner.
#[test]
fn three_cube_edges_at_a_corner_chamfer_mitre_to_one_point() {
    let built = build(three_edges_at_the_corner, SIZE, false, 1);

    assert_it_is_a_sound_solid(&built);

    let removed = (3.0 * (CUBE_SIZE - SIZE) * (SIZE * SIZE / 2.0)) + (SIZE * SIZE * SIZE)
        - (SIZE * SIZE * SIZE / 4.0);

    assert_within(
        signed_volume(&built),
        8000.0 - removed,
        8000.0 * 1e-6,
        "three prisms beyond the corner cell, and the cell keeping what stands under all three chamfer planes",
    );

    assert_within(
        8000.0 - removed,
        7750.25,
        1e-9,
        "the closed form is the boolean construction's own reading of the same corner",
    );

    let meeting_at = corner() + Vector3::new(-SIZE / 2.0, SIZE / 2.0, -SIZE / 2.0);
    let in_the_cell = faces_in_the_corner_cell(&built);

    assert_eq!(
        in_the_cell.len(),
        3,
        "what stands in the cell is the three strips' own bent ends, one triangle each, and no patch at all"
    );

    for face in &in_the_cell {
        assert!(
            face.iter().any(|&at| (at - meeting_at).length() < 1e-5),
            "every one of them runs to the point the three planes meet at, and {meeting_at:?} is in none of {face:?}"
        );
    }

    for touch in touch_points() {
        let sharing = in_the_cell
            .iter()
            .filter(|face| face.iter().any(|&at| (at - touch).length() < 1e-5))
            .count();
        assert_eq!(
            sharing, 2,
            "each touch point is the far end of one chord, shared by the two strips that meet there, and {touch:?} is not"
        );
    }
}

/// The same corner filleted: the patch is the octant of the ball that stands a radius from all three
/// faces, and every vertex of it is exactly a radius from that centre. The octant is the one inexact
/// term of the volume - the patch is INSCRIBED in the ball - and the reading is held to 0.4 for it.
#[test]
fn three_cube_edges_at_a_corner_fillet_into_a_sphere_octant() {
    let built = build(three_edges_at_the_corner, SIZE, true, SEGMENTS);

    assert_it_is_a_sound_solid(&built);

    let pi = std::f64::consts::PI;
    let section = section_area(&fillet_profile(SIZE, pi / 2.0, SEGMENTS));
    let removed = (3.0 * (CUBE_SIZE - SIZE) * section) + (SIZE * SIZE * SIZE)
        - (pi * SIZE * SIZE * SIZE / 6.0);

    assert_within(
        signed_volume(&built),
        8000.0 - removed,
        0.4,
        "three exact prisms and one cell, the cell keeping an octant the patch can only inscribe",
    );

    assert!(
        signed_volume(&built) < 8000.0 - removed,
        "an inscribed patch and inscribed chords can only take MORE than the smooth solid, never less"
    );

    let centre = corner() + Vector3::new(-SIZE, SIZE, -SIZE);
    let patch = faces_in_the_corner_cell(&built);

    assert_eq!(
        patch.len() as i32,
        (4 * SEGMENTS * SEGMENTS) - (3 * SEGMENTS),
        "the grid on its own arcs' sample count is n squared triangles, and one level of INTERIOR \
         refinement cuts each of them in four but for the edges of the RIM - 4T less R, which for three \
         rings of n chords is 4n squared less 3n"
    );

    for face in &patch {
        for &at in face {
            assert_within(
                (at - centre).length(),
                SIZE,
                1e-6,
                &format!("every point of the patch stands a radius from the ball centre, and {at:?} does not"),
            );
        }
    }
}

/// A row whose two rim ends stand OPPOSITE on the ball refuses the whole corner instead of being
/// laid on a great circle that does not exist. The rim is built by hand as a lune between two
/// meridians, so that what is measured is the opposition and not the shape.
#[test]
fn a_row_whose_ends_stand_opposite_on_the_ball_refuses_the_corner() {
    for (colatitude_degrees, emits) in [(89.995, false), (45.0, true)] {
        let centre = Vector3::new(0.0, 0.0, 0.0);
        let mid = colatitude_degrees * std::f64::consts::PI / 180.0;
        let far = 2.0 * mid;

        // The lune: a pole, two meridians down to colatitude 2 * mid, and a base between their far
        // ends.
        let apex = Vector3::new(0.0, 0.0, 1.0);
        let on_the_right = Vector3::new(mid.sin(), 0.0, mid.cos());
        let right_far = Vector3::new(far.sin(), 0.0, far.cos());
        let on_the_left = Vector3::new(-mid.sin(), 0.0, mid.cos());
        let left_far = Vector3::new(-far.sin(), 0.0, far.cos());

        let mut vertices: Vec<Vector3Float> = Vec::new();
        let mut at = |direction: Vector3| {
            vertices.push(Vector3Float::from_vector3(direction.get_normal()));
            vertices.len() as i32 - 1
        };

        let apex_at = at(apex);
        let right_at = at(on_the_right);
        let right_far_at = at(right_far);
        let along_the_base = [
            right_far_at,
            at(bevel_seam_ring::slerp(right_far, left_far, 1.0 / 3.0)),
            at(bevel_seam_ring::slerp(right_far, left_far, 2.0 / 3.0)),
        ];
        let left_far_at = at(left_far);
        let left_at = at(on_the_left);

        let sides = vec![
            vec![apex_at, right_at, right_far_at],
            vec![
                along_the_base[0],
                along_the_base[1],
                along_the_base[2],
                left_far_at,
            ],
            vec![left_far_at, left_at, apex_at],
        ];

        let mut pool = SurfaceVertexPool::new(&vertices);
        let mut faces = Vec::new();

        let laid =
            surface_corner_patch_rows::try_emit(&mut pool, &mut faces, &sides, centre, 1.0, false);

        assert_eq!(
            laid,
            emits,
            "the row runs between two points {} degrees apart on the ball, {}",
            (2.0 * colatitude_degrees * 1000.0).round_ties_even() / 1000.0,
            if emits {
                format!("which is one arc and lays {} triangles", faces.len())
            } else {
                "which is a half turn and no arc at all - it must refuse rather than lid with rounding"
                    .to_string()
            }
        );

        for &(v0, v1, v2) in &faces {
            for vertex in [v0, v1, v2] {
                let at = pool.position_of(vertex);

                assert!(
                    !(at.x.is_nan() || at.y.is_nan() || at.z.is_nan()),
                    "nothing published may be a coordinate that is not a number - {at:?}"
                );
            }
        }
    }
}

/// The cube corner filleted at this class's own size and sample count -
/// the SQUARE corner, whose patch is a sphere octant - for the oblique corner's rows to be held to
/// leaving it exactly as it was.
pub fn the_filleted_cube_corner() -> Mesh {
    build(three_edges_at_the_corner, SIZE, true, SEGMENTS)
}

/// The faces of the built mesh lying wholly inside the corner cell - the cube of the bevel's own
/// size at the corner - which is where the patch is and where nothing else can be.
fn faces_in_the_corner_cell(mesh: &Mesh) -> Vec<[Vector3; 3]> {
    let least = corner() + Vector3::new(-SIZE, 0.0, -SIZE);
    let most = corner() + Vector3::new(0.0, SIZE, 0.0);

    mesh.faces
        .iter()
        .map(|face| {
            [
                Vector3::from_vector3_float(mesh.vertices[face.v0 as usize]),
                Vector3::from_vector3_float(mesh.vertices[face.v1 as usize]),
                Vector3::from_vector3_float(mesh.vertices[face.v2 as usize]),
            ]
        })
        .filter(|corners| {
            corners.iter().all(|at| {
                at.x >= least.x - 1e-5
                    && at.x <= most.x + 1e-5
                    && at.y >= least.y - 1e-5
                    && at.y <= most.y + 1e-5
                    && at.z >= least.z - 1e-5
                    && at.z <= most.z + 1e-5
            })
        })
        .collect()
}

/// Where the ball of the bevel's size touches each of the corner's three faces.
fn touch_points() -> [Vector3; 3] {
    let centre = corner() + Vector3::new(-SIZE, SIZE, -SIZE);

    [
        centre + Vector3::new(SIZE, 0.0, 0.0),
        centre + Vector3::new(0.0, -SIZE, 0.0),
        centre + Vector3::new(0.0, 0.0, SIZE),
    ]
}

/// The three edges of the cube meeting at the corner.
fn three_edges_at_the_corner(features: &FeatureEdgeSet) -> Vec<FeatureEdge> {
    let mut found = two_edges_at_the_corner(features);

    found.push(edge_between(
        features,
        corner(),
        Vector3::new(CUBE_SIZE / 2.0, -CUBE_SIZE / 2.0, -CUBE_SIZE / 2.0),
    ));

    found
}

/// The two edges picked ONE AT A TIME, which is how the app gets them, are joined back into one
/// chain before anything is planned, and bevel to the same solid as the two picked together.
#[test]
fn two_edges_picked_separately_are_joined_into_one_run_to_bevel() {
    let cube = create_cube(CUBE_SIZE, CUBE_SIZE, CUBE_SIZE);
    let features = FeatureEdgeSet::create(&cube);

    let mut separately: Vec<BevelSweptChain> = Vec::new();

    for edge in two_edges_at_the_corner(&features) {
        separately.extend(bevel_chain_linker::link(&[edge]));
    }

    assert_eq!(
        separately.len(),
        2,
        "one group, one edge, one chain - twice"
    );

    let joined = join_and_open(&separately);

    assert_eq!(joined.len(), 1);
    assert_eq!(joined[0].edges.len(), 2);
    assert!(!joined[0].is_closed);
    assert_eq!(
        joined[0].edges[0].to_vertex(),
        joined[0].edges[1].from_vertex(),
        "a joined run is head to tail, which is what the mitre between its two edges is built on"
    );

    let built = build(two_edges_at_the_corner, SIZE, false, 1);
    let removed = (2.0 * CUBE_SIZE * (SIZE * SIZE / 2.0)) - (SIZE * SIZE * SIZE / 3.0);

    assert_within(
        signed_volume(&built),
        8000.0 - removed,
        8000.0 * 1e-6,
        "the volume",
    );
}

/// One pass is one operation: three chains at a corner, two of them planned at 3 mm and one at 1 mm,
/// is refused at the top of the build, naming the pass of mixed sizes.
#[test]
fn plans_of_two_different_sizes_in_one_pass_are_refused() {
    let cube = create_cube(CUBE_SIZE, CUBE_SIZE, CUBE_SIZE);
    let (regions, features) = regions_of(&cube);

    let chains = bevel_chain_linker::link(&three_edges_at_the_corner(&features));
    let mut plans = Vec::with_capacity(chains.len());

    for chain in &chains {
        let size = if plans.is_empty() { SIZE } else { 1.0 };

        let plan = SurfaceBevelPlan::try_create(chain, &regions, size, false)
            .unwrap_or_else(|reason| panic!("the plan refused the chain: {reason}"));

        plans.push(plan);
    }

    assert_eq!(
        plans.len(),
        3,
        "three edges at one vertex link into three chains, which is what gives the pass more than one size to disagree about"
    );

    let built = surface_bevel_builder::try_build(
        &cube,
        &cube.get_mesh_edge_graph(),
        &regions,
        &plans,
        1,
        &CancelToken::new(),
        SurfaceBuildOptions::default(),
    );

    let Err(SurfaceBuildFailure::Refused(reason)) = built else {
        panic!("a pass carrying two sizes has no one answer at the vertex they meet at");
    };
    assert_eq!(
        reason.code(),
        BevelAdvisoryCode::SurfaceMixedSizesInOnePass,
        "it is the pass that is refused, at its top, for the one thing wrong with it"
    );
}

/// The mitre's own rim check: the three strips' six rail ends have to be three vertices shared two
/// at a time, read by index.
#[test]
fn a_mitre_wants_its_three_strips_to_share_their_ends_by_index() {
    assert!(
        surface_corner_mitre::rail_ends_pair_up(&[4, 7, 7, 9, 9, 4]),
        "three strips chained end to end are what a closed corner looks like"
    );

    assert!(
        !surface_corner_mitre::rail_ends_pair_up(&[4, 7, 7, 9, 9, 5]),
        "the third strip ends at a vertex nothing else stands on, which is a seam with two vertices in it"
    );

    assert!(
        !surface_corner_mitre::rail_ends_pair_up(&[4, 4, 7, 7, 9, 9]),
        "three strips each pairing with themselves close nothing, however the count comes out"
    );
}

fn assert_it_is_a_sound_solid(built: &Mesh) {
    assert!(
        built.is_manifold(),
        "a bevel of a closed solid stays a closed solid"
    );
    let (closed, open_at) = surface_mesh_closure::is_closed(built);
    assert!(
        closed,
        "nothing may be left open, and something is at {open_at:?}"
    );
    assert_eq!(shell_count(built), 1);
}

/// The top front and top right edges of the cube, which meet at the corner.
fn two_edges_at_the_corner(features: &FeatureEdgeSet) -> Vec<FeatureEdge> {
    vec![
        edge_between(
            features,
            corner(),
            Vector3::new(-CUBE_SIZE / 2.0, -CUBE_SIZE / 2.0, CUBE_SIZE / 2.0),
        ),
        edge_between(
            features,
            corner(),
            Vector3::new(CUBE_SIZE / 2.0, CUBE_SIZE / 2.0, CUBE_SIZE / 2.0),
        ),
    ]
}

/// The one feature edge running between two given points, either way about.
fn edge_between(features: &FeatureEdgeSet, from: Vector3, to: Vector3) -> FeatureEdge {
    let found: Vec<_> = features
        .edges()
        .iter()
        .filter(|edge| {
            ((edge.start - from).length() < 1e-6 && (edge.end - to).length() < 1e-6)
                || ((edge.start - to).length() < 1e-6 && (edge.end - from).length() < 1e-6)
        })
        .cloned()
        .collect();
    assert_eq!(found.len(), 1);

    found.into_iter().next().expect("one edge")
}

fn regions_of(cube: &Mesh) -> (Arc<FaceRegionCache>, FeatureEdgeSet) {
    let features = FeatureEdgeSet::create(cube);
    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(cube.clone()),
        Some(cube.get_mesh_edge_graph()),
        features.flat_angle_degrees(),
    ));

    (regions, features)
}

/// The cube with the named edges bevelled by the surface construction, as one chain per linked run.
fn build(
    picked: fn(&FeatureEdgeSet) -> Vec<FeatureEdge>,
    size: f64,
    round_profile: bool,
    segments_per_quarter: i32,
) -> Mesh {
    let cube = create_cube(CUBE_SIZE, CUBE_SIZE, CUBE_SIZE);
    let (regions, features) = regions_of(&cube);

    let chains = bevel_chain_linker::link(&picked(&features));
    let mut plans = Vec::with_capacity(chains.len());

    for chain in &chains {
        plans.push(
            SurfaceBevelPlan::try_create(chain, &regions, size, round_profile)
                .unwrap_or_else(|reason| panic!("the plan refused the chain: {reason}")),
        );
    }

    surface_bevel_builder::try_build(
        &cube,
        &cube.get_mesh_edge_graph(),
        &regions,
        &plans,
        segments_per_quarter,
        &CancelToken::new(),
        SurfaceBuildOptions::default(),
    )
    .unwrap_or_else(|reason| panic!("the build refused the corner: {reason:?}"))
    .mesh
}

/// The fillet's removed section as the tessellation cuts it, in the (across, down) frame whose
/// origin is the corner the bevel takes off.
// The rotation of (0, -1) is written term by term, multiplications by -1 included, so the
// profile's floats round the same way on every platform.
#[allow(clippy::neg_multiply)]
pub fn fillet_profile(radius: f64, wedge_radians: f64, segments: i32) -> Vec<Vector2> {
    let sweep = std::f64::consts::PI - wedge_radians;
    let centre = Vector2::new(
        radius / (wedge_radians / 2.0).tan(),
        radius / (wedge_radians / 2.0).tan(),
    );

    (0..=segments)
        .map(|i| {
            let turn = -sweep * i as f64 / segments as f64;

            centre
                + Vector2::new(
                    (0.0 * turn.cos()) - (-1.0 * turn.sin()),
                    (0.0 * turn.sin()) + (-1.0 * turn.cos()),
                ) * radius
        })
        .collect()
}

/// How much one edge's bevel takes per unit length: the area between the corner and the profile.
pub fn section_area(profile: &[Vector2]) -> f64 {
    let mut twice = 0.0;

    let mut ring = vec![Vector2::new(0.0, 0.0)];
    ring.extend_from_slice(profile);

    for i in 0..ring.len() {
        let here = ring[i];
        let next = ring[(i + 1) % ring.len()];

        twice += (here.x * next.y) - (next.x * here.y);
    }

    f64::abs(twice) / 2.0
}

/// What two bevels meeting at one corner would each take twice if the mitre did not stop them.
pub fn mitred_corner_overlap(profile: &[Vector2]) -> f64 {
    let mut overlap = 0.0;

    for i in 0..profile.len().saturating_sub(1) {
        let depth = profile[i + 1].y - profile[i].y;
        let first = profile[i].x;
        let second = profile[i + 1].x;

        overlap += depth / 3.0 * ((first * first) + (first * second) + (second * second));
    }

    overlap
}

/// The area of every triangle all three of whose corners stand on one plane.
fn area_on(mesh: &Mesh, on_the_plane: impl Fn(Vector3Float) -> bool) -> f64 {
    let mut area = 0.0;

    for face in mesh.faces.iter() {
        if on_the_plane(mesh.vertices[face.v0 as usize])
            && on_the_plane(mesh.vertices[face.v1 as usize])
            && on_the_plane(mesh.vertices[face.v2 as usize])
        {
            let first = Vector3::from_vector3_float(mesh.vertices[face.v0 as usize]);

            area += (Vector3::from_vector3_float(mesh.vertices[face.v1 as usize]) - first)
                .cross(Vector3::from_vector3_float(mesh.vertices[face.v2 as usize]) - first)
                .length()
                / 2.0;
        }
    }

    area
}
