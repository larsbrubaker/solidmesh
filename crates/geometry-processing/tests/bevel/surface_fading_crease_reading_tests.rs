//! Port of `SurfaceFadingCreaseReadingTests.cs`: what an L prism whose inner corner fades into a
//! cone above its apex (`topology_test_meshes::create_l_prism_with_fading_inner_corner`) reads as -
//! ONE 90 degree crease up to the apex, a flood that crosses the corner because every turn round the
//! apex is under the 10 degree rung, a strip that narrows linearly to nothing where the wall's rim
//! reaches the rail, and a closing ring that welds only on the blend's own chords.

use geometry_processing::bevel::bevel_edge_tool_builder;
use geometry_processing::bevel::surface::SurfaceVertexPool;
use geometry_processing::topology::{EdgeConvexity, FeatureEdgeSet};
use polygon_mesh::Mesh;
use vector_math::math_helper;
use vector_math::Vector3;

use crate::topology_test_meshes::create_l_prism_with_fading_inner_corner;

/// The picker's smooth-wall rung, the same one the refusal test floods at.
const FLAT_ANGLE_DEGREES: f64 = 10.0;

const TOP_RADIUS: f64 = 1.0;

const BLEND_FACETS: i32 = 12;

const HEIGHT: f64 = 10.0;

/// Where the sharp half of the inner corner ends and the cone takes over.
fn apex() -> Vector3 {
    Vector3::new(10.0, 10.0, HEIGHT / 2.0)
}

fn the_prism() -> Mesh {
    create_l_prism_with_fading_inner_corner(TOP_RADIUS, BLEND_FACETS, HEIGHT)
}

#[test]
fn the_crease_does_not_open_at_all_it_stops_at_the_cones_apex() {
    let prism = the_prism();
    let features = FeatureEdgeSet::create_with_angle(&prism, FLAT_ANGLE_DEGREES);

    let concave: Vec<_> = features
        .edges()
        .iter()
        .filter(|edge| edge.convexity == EdgeConvexity::Concave)
        .collect();

    assert_eq!(
        concave.len(),
        1,
        "the fade leaves exactly one crease, the sharp half of the inner corner"
    );

    let crease = concave[0];

    assert!((crease.start - Vector3::new(10.0, 10.0, 0.0)).length() < 1e-9);
    assert!((crease.end - apex()).length() < 1e-9);

    let wedge = bevel_edge_tool_builder::profile_wedge_angle_degrees(crease);
    assert!(
        (wedge - 90.0).abs() <= 1e-9,
        "ONE edge, one wedge - the dihedral never opens, so there is no per-station setback to \
         taper and the r / tan(theta / 2) reading of the fade does not apply here ({wedge})"
    );
}

#[test]
fn the_flood_crosses_the_corner_because_every_turn_round_the_apex_is_under_the_threshold() {
    let prism = the_prism();
    let graph = prism.get_mesh_edge_graph();

    let mut turns: Vec<f64> = Vec::new();

    for e in 0..graph.edge_count() {
        let a = Vector3::from_vector3_float(prism.vertices[graph.get_vertex0(e) as usize]);
        let b = Vector3::from_vector3_float(prism.vertices[graph.get_vertex1(e) as usize]);

        // only the blend's own fan - the crease itself and the bottom face's fan diagonals also
        // reach the apex, and neither is a step the flood walks over the corner on
        let far = if (a - apex()).length() < 1e-9 {
            b
        } else if (b - apex()).length() < 1e-9 {
            a
        } else {
            Vector3::default()
        };

        if far == Vector3::default() || far.z < HEIGHT - 1e-9 {
            continue;
        }

        let n0 = Vector3::from_vector3_float(prism.faces[graph.get_face(e, 0) as usize].normal);
        let n1 = Vector3::from_vector3_float(prism.faces[graph.get_face(e, 1) as usize].normal);

        turns.push(math_helper::radians_to_degrees(
            n0.dot(n1).clamp(-1.0, 1.0).acos(),
        ));
    }

    assert_eq!(
        turns.len(),
        (BLEND_FACETS + 1) as usize,
        "twelve sectors leave thirteen generators, two of them the wall-to-cone creases"
    );

    let min = turns.iter().copied().fold(f64::INFINITY, f64::min);
    let max = turns.iter().copied().fold(f64::NEG_INFINITY, f64::max);

    assert!(
        (min - 3.8241).abs() <= 1e-3,
        "the two wall-to-cone creases turn half a facet step ({min})"
    );

    assert!(
        (max - 7.6074).abs() <= 1e-3,
        "and the sectors turn a facet step - every one of them under the 10 degree rung, which is \
         why the two walls come back as ONE region and the plan refuses today ({max})"
    );
}

fn the_strips_width_falls_linearly_to_zero_where_the_walls_own_rim_arrives_at_the_rail(
    radius: f64,
    expected_last_station_height: f64,
) {
    let prism = the_prism();

    let setback = bevel_edge_tool_builder::tangent_distance(radius, 90.0, true);

    assert!(
        (setback - radius).abs() <= 1e-12,
        "r / tan(45 degrees) is r, at every station, because the wedge never changes ({setback})"
    );

    let rail_x = 10.0 + setback;

    let mut previous = f64::MAX;

    let mut z = 0.0;
    while z <= expected_last_station_height + 1e-9 {
        let width = rail_x - wall_reach_at(&prism, z);

        assert!(
            width <= previous + 1e-12,
            "the strip only ever narrows along the run - at z {z} it is {width}"
        );

        assert!(
            width >= -1e-12,
            "and never crosses its own rail - at z {z} it is {width}"
        );

        if z <= HEIGHT / 2.0 {
            assert!(
                (width - radius).abs() <= 1e-12,
                "below the apex the corner is square, so the strip is the full setback wide ({width})"
            );
        } else {
            assert!(
                (width - (radius - ((z - (HEIGHT / 2.0)) / 5.0))).abs() <= 1e-12,
                "above it the wall stops at the cone's radius, which grows a fifth a millimetre per \
                 millimetre of height ({width} at z {z})"
            );
        }

        previous = width;
        z += 0.125;
    }

    let last = rail_x - wall_reach_at(&prism, expected_last_station_height);
    assert!(
        last.abs() <= 1e-12,
        "so the last station is z {expected_last_station_height}, where the wall's receding rim \
         meets the rail at ({rail_x}, 10, {expected_last_station_height}) and the strip has no \
         width left to close ({last})"
    );
}

#[test]
fn the_strips_width_falls_linearly_to_zero_where_the_walls_own_rim_arrives_at_the_rail_0_25_6_25() {
    the_strips_width_falls_linearly_to_zero_where_the_walls_own_rim_arrives_at_the_rail(0.25, 6.25);
}

#[test]
fn the_strips_width_falls_linearly_to_zero_where_the_walls_own_rim_arrives_at_the_rail_0_5_7_5() {
    the_strips_width_falls_linearly_to_zero_where_the_walls_own_rim_arrives_at_the_rail(0.5, 7.5);
}

#[test]
fn at_the_radius_the_refusal_tests_use_there_is_no_fade_to_build() {
    let prism = the_prism();

    let setback = bevel_edge_tool_builder::tangent_distance(2.0, 90.0, true);
    let width = 10.0 + setback - wall_reach_at(&prism, HEIGHT);

    assert!(
        (width - 1.0).abs() <= 1e-12,
        "at r 2 the blend's own radius never catches the rail - the strip is still a millimetre \
         wide where it reaches the top face, so what this pick wants at that size is a run off the \
         rim, not a fade ({width})"
    );
}

#[test]
fn the_closing_ring_is_zero_width_only_on_the_blends_own_chords() {
    let prism = the_prism();

    let pool = SurfaceVertexPool::near_enough_for(&prism.vertices);

    assert!(
        (pool - 3.0e-4).abs() <= 1e-12,
        "a thirty millimetre diagonal, a millionth of it welded, ten of those near enough ({pool})"
    );

    const RADIUS: f64 = 0.5;

    // at the last station the blend's section and the bead's circle share a centre and a radius,
    // so the section's VERTICES are on the bead exactly and its chords sag inside it by the versine
    let sag =
        RADIUS * (1.0 - math_helper::degrees_to_radians(90.0 / BLEND_FACETS as f64 / 2.0).cos());

    assert!((sag - 1.0706e-3).abs() <= 1e-7, "{sag}");

    assert!(
        sag / pool > 3.0,
        "{:.2} pool rulers - the cap closes with no width only if the bead's last ring is \
         tessellated on the blend's own twelve chords rather than on the bead's own segment count, \
         because the sag between them is far too wide to weld shut",
        sag / pool
    );
}

/// The smallest x the wall plane y = 10 still covers at height z, read off the mesh - which is the
/// rim the strip's far rail has to land on.
fn wall_reach_at(mesh: &Mesh, z: f64) -> f64 {
    let mut reach = f64::MAX;

    for face in mesh.faces.iter() {
        let corners = [
            Vector3::from_vector3_float(mesh.vertices[face.v0 as usize]),
            Vector3::from_vector3_float(mesh.vertices[face.v1 as usize]),
            Vector3::from_vector3_float(mesh.vertices[face.v2 as usize]),
        ];

        if corners.iter().any(|v| (v.y - 10.0).abs() > 1e-9) {
            continue;
        }

        for i in 0..3 {
            let a = corners[i];
            let b = corners[(i + 1) % 3];

            if (a.z - z) * (b.z - z) > 0.0 {
                continue;
            }

            if (a.z - b.z).abs() < 1e-12 {
                reach = reach.min(a.x.min(b.x));
                continue;
            }

            reach = reach.min(a.x + ((b.x - a.x) * ((z - a.z) / (b.z - a.z))));
        }
    }

    reach
}
