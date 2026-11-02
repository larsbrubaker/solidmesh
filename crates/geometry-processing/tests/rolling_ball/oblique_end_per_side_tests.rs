//! An open chain whose sides differ, ending on end faces a few degrees off square that carry an
//! end ray: the one-wall-bending wedge, and the groove it is the V of, sheared along the chain by 8
//! degrees each way, built with a 30 degree sharp turn (the threshold under which the oblique end
//! reading gives each end its ray). The ball takes the pass, each side's rail runs on to the end
//! plane at its own side's distance from the chain's line, and the filled part is closed and does
//! not cross itself. On the groove the round adds material, so each leaning end face grows over it
//! and is capped between the round's last row, its section in that face, and the end run; the
//! level line's own build of the groove (the switch off) caps the same way.

use std::collections::HashMap;

use geometry_processing::bevel::level_line::band_gap_entry_points::{
    build_pick, BandGapBuildOptions,
};
use geometry_processing::bevel::level_line::gap_fill::{self, DEFAULT_FILLET_SEGMENTS};
use geometry_processing::bevel::level_line::{
    BandGapRefusal, BandGapResult, BandProfile, GapFillRefusal, GapFillResult, LevelLineMesh,
};
use geometry_processing::bevel::rolling_ball::rolling_ball_routing::{
    last_route, reset_to_the_default, set_last_route, set_use_rolling_ball,
};
use polygon_mesh::csg::boolean_processing::{classify_boolean_operand, BooleanOperandVerdict};
use polygon_mesh::Mesh;
use vector_math::{Matrix4X4, Vector3, Vector3Float};

use crate::band_route_tests::creases;
use crate::band_route_tests::Triangles;
use crate::band_setback_tests::{
    a_groove_with_one_wall_bending_steeper, a_wedge_with_one_wall_bending_steeper,
    on_the_steeper_walls_edge, on_the_straight_walls_edge,
    the_balls_centre_in_the_one_sided_groove, the_contact_distances_in_the_one_sided_groove,
    STEEP_DEGREES,
};

/// The chain: the apex (or the groove's floor) from (0, 0, 0) to (0, 6, 0).
const LENGTH: f64 = 6.0;

/// The fillet's size in every test here.
const RADIUS: f64 = 2.0;

/// How near a point must stand to a hand answer: the field's root precision.
const NEAR: f64 = 1e-6;

/// Whether the triangles cross themselves, by the boolean operand check.
fn crosses_itself(positions: &[Vector3], triangles: &[(i32, i32, i32)]) -> bool {
    let mut mesh = Mesh::new();
    for p in positions {
        mesh.vertices.push(Vector3Float::from_vector3(*p));
    }
    for &(a, b, c) in triangles {
        mesh.faces.add(a, b, c, &mesh.vertices);
    }
    classify_boolean_operand(&mesh, Matrix4X4::IDENTITY, false)
        == BooleanOperandVerdict::SelfIntersecting
}

/// `part` leaning `degrees` off square at both ends (each vertex moved along the chain by its x
/// times the lean's tangent), with the lean's slope.
fn leaning(part: (Vec<Vector3>, Triangles), degrees: f64) -> (Vec<Vector3>, Triangles, f64) {
    let slope = degrees.to_radians().tan();
    let (positions, triangles) = part;
    let positions = positions
        .iter()
        .map(|p| Vector3::new(p.x, p.y + (slope * p.x), p.z))
        .collect();
    (positions, triangles, slope)
}

/// `chain` of `positions` filleted at [`RADIUS`] with the switch `on` or off: the band, the route
/// it took and the fill.
fn filleted(
    positions: &[Vector3],
    triangles: &Triangles,
    chain: [i32; 2],
    on: bool,
) -> (BandGapResult, Option<String>, GapFillResult) {
    set_use_rolling_ball(on);
    set_last_route(None);
    let band = build_pick(
        positions,
        triangles,
        &[chain.to_vec()],
        &[false],
        Some(creases(positions, triangles)),
        RADIUS,
        BandProfile::Fillet,
        &BandGapBuildOptions {
            sharp_turn_degrees: 30.0,
            ..BandGapBuildOptions::default()
        },
    )
    .expect("the pick's arguments are good");
    let route = last_route();
    let fill = gap_fill::fill(&band, BandProfile::Fillet, DEFAULT_FILLET_SEGMENTS);
    reset_to_the_default();
    (band, route, fill.expect("the fill runs"))
}

/// The filled `mesh` is closed (every directed edge of a live triangle run once, and once back)
/// and does not cross itself; each end face, leaning by `slope`, stays one flat face (no vertex
/// stands just off it, as a row bowed out of it would).
fn closed_flat_ended_and_not_crossing(mesh: &LevelLineMesh, slope: f64) {
    for v in 0..mesh.vertex_count() {
        let p = mesh.position(v);
        for at in [0.0, LENGTH] {
            let off = (p.y - (at + (slope * p.x))).abs();
            assert!(
                off <= NEAR || off >= 0.01,
                "{p:?} stands {off} off the end face through y = {at}"
            );
        }
    }

    let mut directed: HashMap<(i32, i32), i32> = HashMap::new();
    let mut live = Vec::new();
    for t in 0..mesh.triangle_count() {
        if mesh.is_removed(t) {
            continue;
        }
        let face = mesh.triangles()[t as usize];
        live.push((face.a, face.b, face.c));
        for (a, b) in [(face.a, face.b), (face.b, face.c), (face.c, face.a)] {
            *directed.entry((a, b)).or_default() += 1;
        }
    }
    for (&(a, b), &count) in &directed {
        assert_eq!(count, 1, "edge {a}-{b} is run {count} times one way");
        assert_eq!(
            directed.get(&(b, a)),
            Some(&1),
            "edge {a}-{b} has no triangle running it back: the mesh is open"
        );
    }
    let all: Vec<Vector3> = (0..mesh.vertex_count()).map(|v| mesh.position(v)).collect();
    assert!(
        !crosses_itself(&all, &live),
        "the filleted part crosses itself"
    );
}

/// Every vertex `fill` added on an end face (leaning by `slope`) stands `radius` from the axis
/// (x, z) = `centre`, read in (x, z): the lean slides points along the chain only, so a row that is
/// the round's section in the end face keeps the square section's (x, z). Answers how many there
/// were at each end.
fn rows_on_the_end_faces_stand_on(
    mesh: &LevelLineMesh,
    fill: &GapFillResult,
    slope: f64,
    centre: (f64, f64),
    radius: f64,
) -> [i32; 2] {
    let mut on_the_section = [0; 2];
    for &v in &fill.added_vertices {
        let p = mesh.position(v);
        for (end, at) in [(0, 0.0), (1, LENGTH)] {
            if (p.y - (at + (slope * p.x))).abs() > NEAR {
                continue;
            }
            on_the_section[end] += 1;
            let from_the_axis = (p.x - centre.0).hypot(p.z - centre.1);
            assert!(
                (from_the_axis - radius).abs() < NEAR,
                "end {end}'s row vertex {p:?} stands {from_the_axis} from the axis, not {radius}"
            );
        }
    }
    on_the_section
}

/// `part` leaning `degrees` off square at both ends, its `chain` filleted at radius 2 with the
/// switch on. Pins the route, the end rays, each rail's end on each end face at its own side's
/// distance from the chain's line (2.34 on the straight wall, 2.72 on the steeper one, worked out
/// by hand), the filled part closed, flat-ended and not crossing itself, and how many vertices the
/// fill lays on each end face, every one on the ball's cylinder about the centre worked out by
/// hand: where the round adds material (the groove) each end's last row lies on the end face as
/// the round's own section in it (drawn as an arc in the leaning plane by that plane's own angle,
/// the groove's row stood up to 0.11 inside the cylinder); where it takes material away (the
/// wedge) the end run the cut left is the last row and the fill lays nothing there.
fn each_rail_ends_on_the_leaning_end_face_at_its_own_distance(
    part: (Vec<Vector3>, Triangles),
    chain: [i32; 2],
    degrees: f64,
    rows_laid_on_the_end_faces: i32,
) {
    let (positions, triangles, slope) = leaning(part, degrees);
    let (straight, steeper) = the_contact_distances_in_the_one_sided_groove(RADIUS);
    let (ball, route, fill) = filleted(&positions, &triangles, chain, true);

    assert_eq!(route.as_deref(), Some("rolling-ball"));
    assert_eq!(ball.refusal, BandGapRefusal::None);
    assert_eq!(
        ball.chains[0].end_rays,
        (true, true),
        "both ends carry a ray"
    );
    assert_eq!(fill.refusal, GapFillRefusal::None, "{:?}", fill.open_edge);

    let mesh = fill.mesh.as_ref().expect("the filled mesh").borrow();
    let mut met = [[0; 2]; 2];
    for v in 0..mesh.vertex_count() {
        let p = mesh.position(v);
        let from_the_line = p.x.hypot(p.z);
        for (end, at) in [(0, 0.0), (1, LENGTH)] {
            if (p.y - (at + (slope * p.x))).abs() > NEAR || from_the_line <= 0.5 {
                continue;
            }
            let walls = [
                (on_the_straight_walls_edge(p, NEAR), straight),
                (on_the_steeper_walls_edge(p, NEAR), steeper),
            ];
            for (wall, (on, distance)) in walls.into_iter().enumerate() {
                // The round's rail on the wall's edge with the end face; the wall runs on past it.
                if on && from_the_line < distance + 0.5 {
                    met[end][wall] += 1;
                    assert!(
                        (from_the_line - distance).abs() < NEAR,
                        "end {end}'s rail on wall {wall} at {p:?} stands {from_the_line} from \
                         the chain's line, not {distance}"
                    );
                }
            }
        }
    }
    assert_eq!(
        met,
        [[1, 1], [1, 1]],
        "each rail ends once on each end face"
    );

    closed_flat_ended_and_not_crossing(&mesh, slope);
    let centre = the_balls_centre_in_the_one_sided_groove(RADIUS);
    assert_eq!(
        rows_on_the_end_faces_stand_on(&mesh, &fill, slope, centre, RADIUS),
        [rows_laid_on_the_end_faces; 2],
        "the vertices each end's row lays on its end face"
    );
}

/// Leaning one way: the straight wall's end of each end face leans back past the end vertex.
#[test]
fn a_wedge_whose_sides_differ_leaning_one_way_ends_each_rail_on_the_end_face_at_its_own_distance() {
    each_rail_ends_on_the_leaning_end_face_at_its_own_distance(
        a_wedge_with_one_wall_bending_steeper(),
        [0, 4],
        8.0,
        0,
    );
}

/// Leaning the other way: the steeper wall's end of each end face leans back past the end vertex.
#[test]
fn a_wedge_whose_sides_differ_leaning_the_other_way_ends_each_rail_on_the_end_face_at_its_own_distance(
) {
    each_rail_ends_on_the_leaning_end_face_at_its_own_distance(
        a_wedge_with_one_wall_bending_steeper(),
        [0, 4],
        -8.0,
        0,
    );
}

/// The groove leaning one way: the chain is concave, so the round adds material and each leaning
/// end face grows over it; the cap is the end face's own region between the round's last row and
/// the end run. Its two rails end at chain parameters 0.24 apart, and the row drawn about the
/// chain's point at their mean stood off the face at one end, so that cap was not flat and was
/// refused `CornerCapNotStarShaped`.
#[test]
fn a_groove_whose_sides_differ_leaning_one_way_caps_each_leaning_end_face_over_the_round() {
    each_rail_ends_on_the_leaning_end_face_at_its_own_distance(
        a_groove_with_one_wall_bending_steeper(),
        [0, 8],
        8.0,
        DEFAULT_FILLET_SEGMENTS - 1,
    );
}

/// The groove leaning the other way.
#[test]
fn a_groove_whose_sides_differ_leaning_the_other_way_caps_each_leaning_end_face_over_the_round() {
    each_rail_ends_on_the_leaning_end_face_at_its_own_distance(
        a_groove_with_one_wall_bending_steeper(),
        [0, 8],
        -8.0,
        DEFAULT_FILLET_SEGMENTS - 1,
    );
}

/// The level line's build of the groove (the switch off) leaning `degrees`: both sides set back by
/// the angle's 2, so its rails end 2 from the floor's line, the straight one at (-sqrt 2, sqrt 2)
/// and the steeper one where its wall, from (1, 1) at 65 degrees, stands 2 from the line; its rows
/// are drawn about the end vertex, so in (x, z) each end's last row is the circle tangent to the
/// lines from the end vertex to the two rail ends, at those ends.
fn the_level_lines_groove_caps_each_leaning_end_face(degrees: f64) {
    let (positions, triangles, slope) = leaning(a_groove_with_one_wall_bending_steeper(), degrees);
    let (band, route, fill) = filleted(&positions, &triangles, [0, 8], false);
    assert_eq!(route, None, "the switch off takes the level line");
    assert_eq!(band.refusal, BandGapRefusal::None);
    assert_eq!(fill.refusal, GapFillRefusal::None, "{:?}", fill.open_edge);

    // The steeper rail end (1 + s cos 65, 1 + s sin 65) at distance 2: s^2 + 2 s (cos + sin) + 2
    // = 4.
    let steep = STEEP_DEGREES.to_radians();
    let sum = steep.cos() + steep.sin();
    let s = -sum + ((sum * sum) + 2.0).sqrt();
    let q = (1.0 + (s * steep.cos()), 1.0 + (s * steep.sin()));
    let p = (-std::f64::consts::SQRT_2, std::f64::consts::SQRT_2);
    // Both legs are 2, so the circle's centre is on their bisector, 2 / cos(half the angle
    // between them) from the end vertex, and its radius 2 tan(half that angle).
    let cosine = ((p.0 * q.0) + (p.1 * q.1)) / 4.0;
    let half = cosine.acos() / 2.0;
    let bisector = (p.0 + q.0, p.1 + q.1);
    let length = bisector.0.hypot(bisector.1);
    let reach = 2.0 / half.cos();
    let centre = (reach * bisector.0 / length, reach * bisector.1 / length);

    let mesh = fill.mesh.as_ref().expect("the filled mesh").borrow();
    closed_flat_ended_and_not_crossing(&mesh, slope);
    assert_eq!(
        rows_on_the_end_faces_stand_on(&mesh, &fill, slope, centre, 2.0 * half.tan()),
        [DEFAULT_FILLET_SEGMENTS - 1; 2],
        "the vertices each end's row lays on its end face"
    );
}

/// The switch off, leaning one way: the level line caps the groove's leaning end faces the same way.
#[test]
fn the_level_lines_groove_leaning_one_way_caps_each_leaning_end_face_over_the_round() {
    the_level_lines_groove_caps_each_leaning_end_face(8.0);
}

/// The switch off, leaning the other way.
#[test]
fn the_level_lines_groove_leaning_the_other_way_caps_each_leaning_end_face_over_the_round() {
    the_level_lines_groove_caps_each_leaning_end_face(-8.0);
}
