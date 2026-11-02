//! An open chain whose sides differ, ending on end faces a few degrees off square that carry an
//! end ray: the one-wall-bending wedge sheared along its apex by 8 degrees each way, built with a
//! 30 degree sharp turn (the threshold under which the oblique end reading gives each end its
//! ray). The ball takes the pass, each side's rail runs on to the end plane at its own side's
//! distance from the chain's line, and the filled part is closed and does not cross itself.

use std::collections::HashMap;

use geometry_processing::bevel::level_line::band_gap_entry_points::{
    build_pick, BandGapBuildOptions,
};
use geometry_processing::bevel::level_line::gap_fill::{self, DEFAULT_FILLET_SEGMENTS};
use geometry_processing::bevel::level_line::{BandGapRefusal, BandProfile, GapFillRefusal};
use geometry_processing::bevel::rolling_ball::rolling_ball_routing::{
    last_route, reset_to_the_default, set_last_route, set_use_rolling_ball,
};
use polygon_mesh::csg::boolean_processing::{classify_boolean_operand, BooleanOperandVerdict};
use polygon_mesh::Mesh;
use vector_math::{Matrix4X4, Vector3, Vector3Float};

use crate::band_route_tests::creases;
use crate::band_setback_tests::{
    a_wedge_with_one_wall_bending_steeper, on_the_steeper_walls_edge, on_the_straight_walls_edge,
    the_contact_distances_in_the_one_sided_groove,
};

/// The chain: the apex from (0, 0, 0) to (0, 6, 0).
const LENGTH: f64 = 6.0;

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

/// The wedge leaning `degrees` off square at both ends (each vertex moved along the chain by its x
/// times the lean's tangent), its apex filleted at radius 2 with the switch on. Pins the route,
/// the end rays, each rail's end on each end face at its own side's distance from the chain's
/// line (2.34 on the straight wall, 2.72 on the steeper one, worked out by hand), and the filled
/// part closed and not crossing itself.
fn each_rail_ends_on_the_leaning_end_face_at_its_own_distance(degrees: f64) {
    let slope = degrees.to_radians().tan();
    let (positions, triangles) = a_wedge_with_one_wall_bending_steeper();
    let positions: Vec<Vector3> = positions
        .iter()
        .map(|p| Vector3::new(p.x, p.y + (slope * p.x), p.z))
        .collect();
    let radius = 2.0;
    let (straight, steeper) = the_contact_distances_in_the_one_sided_groove(radius);

    set_use_rolling_ball(true);
    set_last_route(None);
    let ball = build_pick(
        &positions,
        &triangles,
        &[vec![0, 4]],
        &[false],
        Some(creases(&positions, &triangles)),
        radius,
        BandProfile::Fillet,
        &BandGapBuildOptions {
            sharp_turn_degrees: 30.0,
            ..BandGapBuildOptions::default()
        },
    )
    .expect("the pick's arguments are good");
    let route = last_route();
    let fill = gap_fill::fill(&ball, BandProfile::Fillet, DEFAULT_FILLET_SEGMENTS);
    reset_to_the_default();

    assert_eq!(route.as_deref(), Some("rolling-ball"));
    assert_eq!(ball.refusal, BandGapRefusal::None);
    assert_eq!(
        ball.chains[0].end_rays,
        (true, true),
        "both ends carry a ray"
    );
    let fill = fill.expect("the fill runs");
    assert_eq!(fill.refusal, GapFillRefusal::None, "{:?}", fill.open_edge);

    let mesh = fill.mesh.as_ref().expect("the filled mesh").borrow();
    let near = 1e-6;
    let mut met = [[0; 2]; 2];
    for v in 0..mesh.vertex_count() {
        let p = mesh.position(v);
        let from_the_line = p.x.hypot(p.z);
        for (end, at) in [(0, 0.0), (1, LENGTH)] {
            if (p.y - (at + (slope * p.x))).abs() > near || from_the_line <= 0.5 {
                continue;
            }
            let walls = [
                (on_the_straight_walls_edge(p, near), straight),
                (on_the_steeper_walls_edge(p, near), steeper),
            ];
            for (wall, (on, distance)) in walls.into_iter().enumerate() {
                // The round's rail on the wall's edge with the end face; the wall runs on past it.
                if on && from_the_line < distance + 0.5 {
                    met[end][wall] += 1;
                    assert!(
                        (from_the_line - distance).abs() < near,
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
        "the filleted wedge crosses itself"
    );
}

/// Leaning one way: the straight wall's end of each end face leans back past the end vertex.
#[test]
fn a_wedge_whose_sides_differ_leaning_one_way_ends_each_rail_on_the_end_face_at_its_own_distance() {
    each_rail_ends_on_the_leaning_end_face_at_its_own_distance(8.0);
}

/// Leaning the other way: the steeper wall's end of each end face leans back past the end vertex.
#[test]
fn a_wedge_whose_sides_differ_leaning_the_other_way_ends_each_rail_on_the_end_face_at_its_own_distance(
) {
    each_rail_ends_on_the_leaning_end_face_at_its_own_distance(-8.0);
}
