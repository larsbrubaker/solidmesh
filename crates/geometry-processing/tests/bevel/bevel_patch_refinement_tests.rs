//! Port of `BevelPatchRefinementTests.cs`: the corner patch refined on the inside and untouched on
//! its boundary. The boundary is the SEAM the runs ending at that corner carry point for point, so a
//! refinement that moved it, or put a point between two of its points, would leave two solids
//! meeting along two different polylines; the interior belongs to nobody else and is where the
//! chord error lives.

use polygon_mesh::solids::platonic_solids;
use polygon_mesh::Mesh;
use vector_math::{math_helper, Vector3, Vector3Float};

use geometry_processing::bevel::{bevel_corner_patch, bevel_patch_refinement};
use geometry_processing::topology::{
    bevel_edge_group_resolver, BevelEdgeGroup, BevelSelectionKind, FaceGroup, FeatureEdgeSet,
};

use crate::topology_test_meshes;

const RADIUS: f64 = 3.4;
const SEGMENTS: i32 = 8;
const APEX_DEGREES: f64 = 45.0;
const LENGTH: f64 = 20.0;
const HEIGHT: f64 = 20.0;

/// Every point the refinement adds is ON the sphere, in doubles, and no boundary point moves or
/// gains a point between it and its neighbour.
#[test]
fn every_point_it_adds_is_on_the_sphere_and_no_boundary_point_moves() {
    let mut directions = vec![
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
    ];
    let patch = [(0, 1, 2)];
    let boundary = [vec![1, 2]];
    let before = directions.clone();

    let refined = bevel_patch_refinement::refine(&mut directions, &patch, Some(&boundary));

    for i in 0..before.len() {
        assert_eq!(
            directions[i], before[i],
            "a refinement may add directions and may not move one"
        );
    }

    for direction in &directions {
        assert!(
            (direction.length() - 1.0).abs() <= 1e-9,
            "every point of this surface is a direction on the ball"
        );
    }

    assert_eq!(
        directions.len(),
        5,
        "two of the three edges are interior and take a midpoint each; the boundary edge does not"
    );
    assert_eq!(
        refined.len(),
        3,
        "a triangle with two of its edges split comes out as three"
    );

    let carrying = refined
        .iter()
        .filter(|&&(a, b, c)| {
            [(a, b), (b, c), (c, a)]
                .iter()
                .any(|&edge| edge == (1, 2) || edge == (2, 1))
        })
        .count();
    assert_eq!(carrying, 1, "the boundary edge has to survive WHOLE");

    let would_be_midpoint = (directions[1] + directions[2]).get_normal();
    assert!(
        !directions
            .iter()
            .any(|&direction| (direction - would_be_midpoint).length() < 1e-9),
        "and nothing was added ON it"
    );
}

#[test]
fn the_built_patch_keeps_its_seam_and_puts_every_other_vertex_on_the_ball_cube_corner() {
    the_built_patch_keeps_its_seam_and_puts_every_other_vertex_on_the_ball("cube corner");
}

#[test]
fn the_built_patch_keeps_its_seam_and_puts_every_other_vertex_on_the_ball_knife_tip() {
    the_built_patch_keeps_its_seam_and_puts_every_other_vertex_on_the_ball("knife tip");
}

/// The same thing said of a real corner patch: its boundary vertices are the seam ring, to the bit,
/// and everything else it carries is on the ball. (C#'s one test with two `[Arguments]`.)
fn the_built_patch_keeps_its_seam_and_puts_every_other_vertex_on_the_ball(fixture: &str) {
    let (part, picks) = part(fixture);
    let features = FeatureEdgeSet::create(&part);
    let groups: Vec<BevelEdgeGroup> = picks
        .iter()
        .map(|&(from, to)| group(&features, from, to))
        .collect();

    let (resolutions, _) = bevel_edge_group_resolver::resolve_all(&groups, &part);
    let eligible: Vec<bool> = resolutions.iter().map(|r| r.is_resolved()).collect();
    let patches = bevel_corner_patch::create(&resolutions, &eligible, true, RADIUS, SEGMENTS);

    assert_eq!(patches.patches.len(), 1);
    assert_eq!(patches.ball_centres.len(), 1);

    let centre = *patches.ball_centres.values().next().unwrap();
    let mesh = &patches.patches[0].mesh;
    let carried: Vec<Vector3> = mesh
        .vertices
        .iter()
        .map(|&vertex| Vector3::from_vector3_float(vertex))
        .collect();

    for seam in patches.seam_rings.values() {
        for &point in seam {
            let stored = Vector3::from_vector3_float(Vector3Float::from_vector3(point));
            assert!(
                carried.contains(&stored),
                "the seam point {point:?} has to still be a vertex of the patch after the interior is refined"
            );
        }
    }

    let on_the_ball: Vec<Vector3> = carried
        .iter()
        .copied()
        .filter(|&vertex| ((vertex - centre).length() - RADIUS).abs() < 0.5 * RADIUS)
        .collect();

    assert!(
        on_the_ball.len() > 3 * (SEGMENTS as usize + 1),
        "refining the interior has to leave MORE points on the ball than the seam alone"
    );

    for vertex in &on_the_ball {
        assert!(
            ((*vertex - centre).length() - RADIUS).abs() <= 1e-5,
            "these are the ball's own points"
        );
    }

    let outer_radius = carried
        .iter()
        .map(|&vertex| (vertex - centre).length())
        .fold(f64::NEG_INFINITY, f64::max);
    let on_the_ball_faces = faces_at(mesh, centre, RADIUS);
    let on_the_cap_faces = faces_at(mesh, centre, outer_radius);

    assert!(
        on_the_ball_faces >= 3 * on_the_cap_faces,
        "the ball surface carries {on_the_ball_faces} triangles against the outer cap's {on_the_cap_faces}"
    );
}

/// How many of a mesh's triangles have all three corners at the given distance from a point.
fn faces_at(mesh: &Mesh, centre: Vector3, distance: f64) -> usize {
    let at = |v: i32| Vector3::from_vector3_float(mesh.vertices[v as usize]);
    mesh.faces
        .iter()
        .filter(|face| {
            [face.v0, face.v1, face.v2]
                .iter()
                .all(|&v| ((at(v) - centre).length() - distance).abs() < 1e-4)
        })
        .count()
}

fn part(fixture: &str) -> (Mesh, Vec<(Vector3, Vector3)>) {
    if fixture == "cube corner" {
        let corner = Vector3::new(10.0, 10.0, 10.0);

        return (
            platonic_solids::create_cube(20.0, 20.0, 20.0),
            vec![
                (corner, Vector3::new(-10.0, 10.0, 10.0)),
                (corner, Vector3::new(10.0, -10.0, 10.0)),
                (corner, Vector3::new(10.0, 10.0, -10.0)),
            ],
        );
    }

    let top = HEIGHT / 2.0;
    let spread = LENGTH * (math_helper::degrees_to_radians(APEX_DEGREES) / 2.0).tan();
    let tip = Vector3::new(0.0, 0.0, top);

    (
        topology_test_meshes::create_isosceles_prism(APEX_DEGREES, LENGTH, HEIGHT),
        vec![
            (tip, Vector3::new(0.0, 0.0, -top)),
            (tip, Vector3::new(LENGTH, spread, top)),
            (tip, Vector3::new(LENGTH, -spread, top)),
        ],
    )
}

fn group(features: &FeatureEdgeSet, from: Vector3, to: Vector3) -> BevelEdgeGroup {
    for (i, edge) in features.edges().iter().enumerate() {
        if ((edge.start - from).length() < 1e-5 && (edge.end - to).length() < 1e-5)
            || ((edge.start - to).length() < 1e-5 && (edge.end - from).length() < 1e-5)
        {
            return bevel_edge_group_resolver::create_from_edge(
                features,
                i as i32,
                BevelSelectionKind::Edge,
                FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
            )
            .expect("the seed is a feature edge");
        }
    }

    panic!("the fixture has no feature edge from {from:?} to {to:?}");
}
