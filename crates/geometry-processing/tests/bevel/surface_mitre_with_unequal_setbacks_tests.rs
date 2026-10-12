//! Port of `SurfaceMitreWithUnequalSetbacksTests.cs`: a mitre between two beads of ONE radius that
//! set back by DIFFERENT amounts on the face they share - the right knife tip of the nose-in-notch
//! part, where the tip's own 45 degree vertical edge is mitred against the 90 degree top edge along
//! x = 20.
//!
//! **Why this is not the mitre every other fixture has.** Where both beads set back by the same
//! amount the mitre plane passes through the CORNER VERTEX. Here the tip's wedge is 45 degrees, so
//! its bead stands its tangent line `r / tan(22.5) = r (sqrt 2 + 1)` = 8.2083 up the shared face
//! from the corner, while the top edge's stands at r = 3.4. The plane through the crossing of the
//! two tangent lines - `y + z = 24.8083` - is the seam.
//!
//! **How the rails pair.** By the angle from the SHARED face: both strips chord at the same step
//! (135/12 = 90/8 = 11.25 degrees), the tip's rail at theta crosses the seam at
//! `z = 16.6 + 3.4 sin theta`, exactly where the top edge's own rail at theta stands. The tip's four
//! rails past 90 degrees have no partner and end on the top face z = 20, so its end ring is a
//! STAIRCASE turning at rail 8, (16.6, 4.8083, 20).

use geometry_processing::bevel::surface::surface_mesh_closure;
use geometry_processing::topology::{
    bevel_edge_group_resolver, BevelEdgeGroup, BevelSelectionKind, EdgeConvexity, FaceGroup,
    FeatureEdgeSet,
};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::{math_helper, Vector3};

use crate::mesh_measurements::{shell_count, signed_volume};
use crate::nose_in_notch_cut::anchor;
use crate::nose_in_notch_test_meshes as nose;
use crate::surface_test_build::{expect_built, try_bevel, TryBevelOptions};
use crate::topology_test_meshes::create_raked_l_block;

const RADIUS: f64 = 3.4;

const SEGMENTS: i32 = 8;

/// The cube's edge, and the two planes the tip's own bead is bounded by.
const SIZE: f64 = nose::SIZE;

/// Both axes stand one radius in from the face x = 20 they share.
const AXIS_X: f64 = SIZE - RADIUS;

/// How far up the shared face the TIP's tangent line stands: its wedge is 45 degrees, so
/// `r / tan(22.5) = r (sqrt 2 + 1)`. The top edge's is a plain r.
fn tip_setback() -> f64 {
    RADIUS * (1.0 + 2.0_f64.sqrt())
}

/// The seam: the plane through the crossing of the two tangent lines, as `y + z`.
fn seam_plane() -> f64 {
    tip_setback() + AXIS_X
}

/// The step both strips chord at - per quarter turn and per operation, so 135/12 = 90/8.
const STEP_DEGREES: f64 = 90.0 / SEGMENTS as f64;

/// A place named on the mesh is three floats, so an exact vertex arrives about 1e-7 off it.
const AS_NEAR_AS_A_FLOAT_HOLDS: f64 = 1e-4;

/// How far the raked L's second wall leans under the part, which makes its wedge 75 degrees.
const RAKE_DEGREES: f64 = 15.0;

/// The raked L's own size and segments, the numbers the Boolean row measures it at.
const RAKED_RADIUS: f64 = 1.0;

const RAKED_SEGMENTS: i32 = 8;

/// The size the corpus fixture's fourth group carries, and the segments its operation does.
const THE_RIMS_RADIUS: f64 = 2.0;

const THE_RIMS_SEGMENTS: i32 = 16;

#[test]
fn a_mitre_of_unequal_setbacks_is_cut_on_the_plane_through_the_tangent_lines_crossing() {
    let part = nose::create();
    let features = FeatureEdgeSet::create(&part);

    let the_tip = anchor(
        &features,
        Vector3::new(SIZE, 0.0, 0.0),
        Vector3::new(SIZE, 0.0, SIZE),
    );
    let the_top_edge = anchor(
        &features,
        Vector3::new(SIZE, 0.0, SIZE),
        Vector3::new(SIZE, SIZE, SIZE),
    );

    let groups = vec![the_tip.clone(), the_top_edge.clone()];

    let mesh = build(
        &part,
        &groups,
        RADIUS,
        SEGMENTS,
        "two beads of unequal setback on one shared face mitre on the crossing of their tangent lines",
    );

    assert_it_is_a_sound_solid(&mesh);

    // THE NINE SHARED CROSSINGS, rail by rail, from the foot on the shared face x = 20 round to the
    // turn point on the top face. Every one of them lies on y + z = 24.8083 by construction.
    for rail in 0..=SEGMENTS {
        let at = the_tip_rail_crosses_the_seam_at(rail as f64 * STEP_DEGREES);

        assert!(
            (at.y + at.z - seam_plane()).abs() < 1e-9,
            "rail {rail}'s crossing {at:?} is on the seam plane before the mesh is even asked"
        );

        assert!(
            carries(&mesh, at),
            "rail {rail} of the tip and rail {rail} of the top edge meet at {at:?}, one vertex and \
             not two"
        );
    }

    // Rail 0 is the crossing of the two tangent lines itself, and rail 8 is the turn point.
    assert!(
        carries(&mesh, Vector3::new(SIZE, tip_setback(), AXIS_X)),
        "rail 0 of the mitre IS where the two tangent lines cross on the shared face"
    );

    assert!(
        carries(&mesh, Vector3::new(AXIS_X, tip_setback() - RADIUS, SIZE)),
        "rail 8 turns the staircase, on both planes at once: 8.2083 - 3.4 = 4.8083 and 24.8083 - \
         4.8083 = 20"
    );

    // AND THE FOUR EXITS. Past 90 degrees the tip's rails stand at x < 16.6, where the top edge's
    // bead never reached and the part's own top face still stands, so they end on z = 20.
    for rail in (SEGMENTS + 1)..13 {
        let theta = math_helper::degrees_to_radians(rail as f64 * STEP_DEGREES);
        let at = Vector3::new(
            AXIS_X + (RADIUS * theta.cos()),
            tip_setback() - (RADIUS * theta.sin()),
            SIZE,
        );

        assert!(
            carries(&mesh, at),
            "rail {rail} has no partner and leaves on the top face at {at:?}"
        );
    }

    // The last exit is the tip bead's own foot on the slope, where it meets the slope's top edge.
    assert!(
        carries(&mesh, Vector3::new(14.19584, 5.80416, SIZE)),
        "the staircase's far end is the tip bead's foot on the slope, on the top face"
    );

    // AND NOTHING STANDS OUTSIDE THE PART. Near the right tip the solid is bounded by x <= 20,
    // z <= 20 and the slope x + y >= 20.
    for vertex in mesh
        .vertices
        .iter()
        .map(|&vertex| Vector3::from_vector3_float(vertex))
        .filter(|vertex| vertex.x > 12.0)
    {
        assert!(
            vertex.x <= SIZE + AS_NEAR_AS_A_FLOAT_HOLDS,
            "{vertex:?} stands outside the face x = 20"
        );
        assert!(
            vertex.z <= SIZE + AS_NEAR_AS_A_FLOAT_HOLDS,
            "{vertex:?} stands above the top face"
        );
        assert!(
            vertex.x + vertex.y >= SIZE - AS_NEAR_AS_A_FLOAT_HOLDS,
            "{vertex:?} stands in front of the right slope, where the notch took the material away"
        );
    }

    // AND WHAT IT TOOK OFF, between the only two bounds that can be stated without a control: the
    // pair stands BELOW either bead alone, and ABOVE the part less both removals added up whole.
    let just_the_tip = the_volume_of_just(&the_tip);
    let just_the_top_edge = the_volume_of_just(&the_top_edge);
    let both = signed_volume(&mesh);

    assert!(
        both < just_the_tip.min(just_the_top_edge),
        "two beads take more off than either alone - {both} against {just_the_tip} and \
         {just_the_top_edge}"
    );

    assert!(
        both > just_the_tip + just_the_top_edge - nose::VOLUME,
        "the two grooves overlap at the mitre, so {both} may not be as low as counting both \
         removals whole"
    );
}

/// THE SAME CORNER ON THE OTHER SIDE, and the part is symmetric about x = 10, so the two builds
/// have to be one shape under the mirror. The reflection swaps LEFT for RIGHT everywhere the
/// construction has a side, so comparing the two meshes under the mirror asks that of every vertex
/// at once.
#[test]
fn the_left_mitre_is_the_rights_mirror() {
    let part = nose::create();
    let features = FeatureEdgeSet::create(&part);

    let groups = vec![
        anchor(
            &features,
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, SIZE),
        ),
        anchor(
            &features,
            Vector3::new(0.0, 0.0, SIZE),
            Vector3::new(0.0, SIZE, SIZE),
        ),
    ];

    let mesh = build(
        &part,
        &groups,
        RADIUS,
        SEGMENTS,
        "the left tip mitres against the top edge along x = 0 exactly as the right one does",
    );

    assert_it_is_a_sound_solid(&mesh);

    // THE NINE SHARED CROSSINGS AND THE FOUR EXITS, mirrored.
    for rail in 0..13 {
        let at = mirrored(if rail <= SEGMENTS {
            the_tip_rail_crosses_the_seam_at(rail as f64 * STEP_DEGREES)
        } else {
            the_tip_rail_leaves_on_the_top_face_at(rail as f64 * STEP_DEGREES)
        });

        assert!(
            carries(&mesh, at),
            "rail {rail} of the left mitre stands at {at:?}, the right one's own rail {rail} \
             reflected"
        );
    }

    // AND THE WHOLE MESH, vertex for vertex.
    let right = the_right_mitre();

    assert_eq!(
        mesh.vertices.len(),
        right.vertices.len(),
        "one shape reflected has one vertex count"
    );
    assert_eq!(
        mesh.faces.len(),
        right.faces.len(),
        "and one triangle count"
    );

    for vertex in mesh
        .vertices
        .iter()
        .map(|&vertex| Vector3::from_vector3_float(vertex))
    {
        assert!(
            carries(&right, mirrored(vertex)),
            "{vertex:?} of the left build has no partner at {:?} in the right one",
            mirrored(vertex)
        );
    }

    // Held to the float the mesh stores rather than to the double the volume is summed in.
    let left_volume = signed_volume(&mesh);
    let right_volume = signed_volume(&right);

    assert!(
        (left_volume - right_volume).abs() <= nose::VOLUME * 1e-7,
        "a reflection takes no material off - {left_volume} against {right_volume}"
    );
}

/// THE SAME PAIR WALKED THE OTHER WAY, so the WIDER bead ARRIVES at the mitre instead of leaving
/// it. The corner is the same corner and the part is the same part, so the two builds have to be
/// ONE MESH.
#[test]
fn the_mitre_is_the_same_when_the_wider_bead_arrives() {
    let part = nose::create();
    let features = FeatureEdgeSet::create(&part);

    // The top edge FIRST, so the walk leaves along it and arrives along the tip - the wider bead.
    let groups = vec![
        anchor(
            &features,
            Vector3::new(SIZE, 0.0, SIZE),
            Vector3::new(SIZE, SIZE, SIZE),
        ),
        anchor(
            &features,
            Vector3::new(SIZE, 0.0, 0.0),
            Vector3::new(SIZE, 0.0, SIZE),
        ),
    ];

    let mesh = build(
        &part,
        &groups,
        RADIUS,
        SEGMENTS,
        "the mitre is the same corner whichever way the chain walks it",
    );

    assert_it_is_a_sound_solid(&mesh);

    let leaving = the_right_mitre();

    assert_eq!(
        mesh.vertices.len(),
        leaving.vertices.len(),
        "one corner built twice has one vertex count"
    );

    for vertex in mesh
        .vertices
        .iter()
        .map(|&vertex| Vector3::from_vector3_float(vertex))
    {
        assert!(
            carries(&leaving, vertex),
            "{vertex:?} of the arriving walk stands nowhere in the leaving one"
        );
    }

    let arriving_volume = signed_volume(&mesh);
    let leaving_volume = signed_volume(&leaving);

    assert!(
        (arriving_volume - leaving_volume).abs() <= nose::VOLUME * 1e-7,
        "and the two walks take the same material off - {arriving_volume} against {leaving_volume}"
    );
}

/// A CLOSED RIM that meets the same two widths twice ("Center Right and Left Side.mcx"'s fourth
/// group): the whole rim of the part's x = 0 face, four edges walked as one closed chain. The knife
/// edge is the WIDE one, so the walk leaves by it at the bottom corner and arrives along it at the
/// top corner, and both mitres have to stand before anything closes.
#[test]
fn the_closed_rim_mitres_the_wide_bead_leaving_and_arriving() {
    let part = nose::create();
    let features = FeatureEdgeSet::create(&part);

    let corners = [
        Vector3::new(0.0, 0.0, SIZE),
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(0.0, SIZE, 0.0),
        Vector3::new(0.0, SIZE, SIZE),
    ];
    let groups: Vec<BevelEdgeGroup> = (0..4)
        .map(|i| anchor(&features, corners[i], corners[(i + 1) % 4]))
        .collect();

    let mesh = build(
        &part,
        &groups,
        THE_RIMS_RADIUS,
        THE_RIMS_SEGMENTS,
        "the rim's wide bead leaves at one corner and arrives at the other",
    );

    assert_it_is_a_sound_solid(&mesh);

    assert!(
        signed_volume(&mesh) < nose::VOLUME,
        "a bevel takes material off"
    );
}

/// THE SAME UNEQUAL MITRE AT A REFLEX CORNER: the raked L's two top edges, which meet at the L's
/// INSIDE corner over an unselected concave crease. Here the narrower edge's far landing face is the
/// square run's own WALL x = 0, which the PARTNER bead has already banded down, so the sector has
/// to be clipped by the partner band's end line as well. The surface is asserted in closed form:
/// where the two runs overshoot into each other a rolling ball leaves the HIGHER of their two
/// cylinders.
#[test]
fn the_reflex_corner_of_the_raked_l_mitres_its_unequal_setbacks() {
    let part = create_raked_l_block(RAKE_DEGREES);
    let features = FeatureEdgeSet::create(&part);

    let rim: Vec<usize> = features
        .edges()
        .iter()
        .enumerate()
        .filter(|(_, edge)| {
            edge.convexity == EdgeConvexity::Convex
                && edge.start.z.abs() < 1e-6
                && edge.end.z.abs() < 1e-6
                && (edge.start.length() < 1e-6 || edge.end.length() < 1e-6)
        })
        .map(|(index, _)| index)
        .collect();

    assert_eq!(
        rim.len(),
        2,
        "the raked L has exactly two top edges running out of its inner corner"
    );

    let groups: Vec<BevelEdgeGroup> = rim
        .iter()
        .map(|&index| {
            bevel_edge_group_resolver::create_from_edge(
                &features,
                index as i32,
                BevelSelectionKind::Edge,
                FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
            )
            .expect("the seed is an edge")
        })
        .collect();

    let mesh = build(
        &part,
        &groups,
        RAKED_RADIUS,
        RAKED_SEGMENTS,
        "the reflex corner's two unequal beads mitre exactly as the tip's do",
    );

    assert_it_is_a_sound_solid(&mesh);

    let rake = math_helper::degrees_to_radians(RAKE_DEGREES);

    // The raked run's axis: a radius inside the top face z = 0 and inside the leaning wall.
    let raked_axis_z = -RAKED_RADIUS;
    let raked_axis_y = -RAKED_RADIUS * (1.0 + rake.sin()) / rake.cos();

    // The same slack the Boolean row allows: six sagittas of one step of the coarser profile.
    let tolerance = 6.0
        * RAKED_RADIUS
        * (1.0
            - math_helper::degrees_to_radians(
                (180.0 - (90.0 - RAKE_DEGREES)) / 2.0 / RAKED_SEGMENTS as f64,
            )
            .cos());

    let samples: Vec<(Vector3, f64)> = mesh
        .vertices
        .iter()
        .map(|&vertex| Vector3::from_vector3_float(vertex))
        .filter(|vertex| {
            vertex.x >= -RAKED_RADIUS
                && vertex.x <= 0.0
                && vertex.y >= raked_axis_y
                && vertex.y <= raked_axis_y + (RAKED_RADIUS * rake.cos())
                && vertex.z >= -RAKED_RADIUS
        })
        .map(|vertex| {
            let square = -RAKED_RADIUS
                + 0.0_f64
                    .max(
                        (RAKED_RADIUS * RAKED_RADIUS)
                            - ((vertex.x + RAKED_RADIUS) * (vertex.x + RAKED_RADIUS)),
                    )
                    .sqrt();
            let raked = raked_axis_z
                + 0.0_f64
                    .max(
                        (RAKED_RADIUS * RAKED_RADIUS)
                            - ((vertex.y - raked_axis_y) * (vertex.y - raked_axis_y)),
                    )
                    .sqrt();

            (vertex, vertex.z - square.max(raked))
        })
        .collect();

    assert!(
        !samples.is_empty(),
        "no built vertex landed in the overlap band at all, so the check below looked at nothing"
    );

    let mut offenders: Vec<&(Vector3, f64)> = samples
        .iter()
        .filter(|sample| sample.1.abs() > tolerance)
        .collect();
    offenders.sort_by(|a, b| b.1.abs().total_cmp(&a.1.abs()));

    assert_eq!(
        offenders.len(),
        0,
        "the corner is the HIGHER of the two runs' cylinders, held to {tolerance:.4} - but the \
         surface is off it at: {:?}",
        offenders.iter().take(6).collect::<Vec<_>>()
    );
}

fn build(part: &Mesh, groups: &[BevelEdgeGroup], size: f64, segments: i32, because: &str) -> Mesh {
    expect_built(
        try_bevel(
            part,
            groups,
            size,
            segments,
            &CancelToken::new(),
            TryBevelOptions::default(),
        ),
        because,
    )
}

/// Closed, manifold, one shell, and no triangle with two corners at one vertex.
fn assert_it_is_a_sound_solid(mesh: &Mesh) {
    let (open, open_at) = surface_mesh_closure::open_edge_count(mesh);

    assert_eq!(
        open, 0,
        "nothing may be left open, and something is at {open_at:?}"
    );
    assert!(mesh.is_manifold());
    assert_eq!(shell_count(mesh), 1);
    assert_eq!(
        two_corners_at_one_point(mesh),
        0,
        "a triangle with two corners at one point covers nothing and shades as a crack"
    );
}

/// The right pair built on a part of its own, to compare the left one against.
fn the_right_mitre() -> Mesh {
    let part = nose::create();
    let features = FeatureEdgeSet::create(&part);

    let groups = vec![
        anchor(
            &features,
            Vector3::new(SIZE, 0.0, 0.0),
            Vector3::new(SIZE, 0.0, SIZE),
        ),
        anchor(
            &features,
            Vector3::new(SIZE, 0.0, SIZE),
            Vector3::new(SIZE, SIZE, SIZE),
        ),
    ];

    build(
        &part,
        &groups,
        RADIUS,
        SEGMENTS,
        "the right mitre has to build before the left can be compared to it",
    )
}

/// Reflected in the part's own plane of symmetry, `x = 10`.
fn mirrored(at: Vector3) -> Vector3 {
    Vector3::new(SIZE - at.x, at.y, at.z)
}

/// What ONE of the two groups measures when it is built on its own, on a part of its own.
fn the_volume_of_just(group: &BevelEdgeGroup) -> f64 {
    let mesh = build(
        &nose::create(),
        std::slice::from_ref(group),
        RADIUS,
        SEGMENTS,
        "the bead has to build on its own before it can be mitred",
    );

    signed_volume(&mesh)
}

/// Where the tip bead's rail at `theta_degrees` from the shared face x = 20 crosses the seam plane
/// - which is also where the top edge's own rail at the same angle stands.
fn the_tip_rail_crosses_the_seam_at(theta_degrees: f64) -> Vector3 {
    let theta = math_helper::degrees_to_radians(theta_degrees);

    Vector3::new(
        AXIS_X + (RADIUS * theta.cos()),
        tip_setback() - (RADIUS * theta.sin()),
        AXIS_X + (RADIUS * theta.sin()),
    )
}

/// Where the tip bead's rail PAST the turn leaves on the top face - the four with no partner.
fn the_tip_rail_leaves_on_the_top_face_at(theta_degrees: f64) -> Vector3 {
    let theta = math_helper::degrees_to_radians(theta_degrees);

    Vector3::new(
        AXIS_X + (RADIUS * theta.cos()),
        tip_setback() - (RADIUS * theta.sin()),
        SIZE,
    )
}

/// Whether a mesh carries a vertex at one place.
fn carries(mesh: &Mesh, at: Vector3) -> bool {
    mesh.vertices.iter().any(|&vertex| {
        (Vector3::from_vector3_float(vertex) - at).length() < AS_NEAR_AS_A_FLOAT_HOLDS
    })
}

/// How many of a mesh's triangles have two corners at one vertex.
fn two_corners_at_one_point(mesh: &Mesh) -> usize {
    mesh.faces
        .iter()
        .filter(|face| face.v0 == face.v1 || face.v1 == face.v2 || face.v2 == face.v0)
        .count()
}
