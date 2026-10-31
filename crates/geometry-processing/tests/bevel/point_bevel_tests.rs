//! Point mode's kernel - which vertices are offered (`point_candidates`) and the one-hole build at
//! a picked point (`point_bevel`) - a cone's tip and a cube's corner, filleted and chamfered across
//! the room, each held to the requirement: closed, consistently oriented, one shell, no zero-area
//! triangle, material removed; the fillet near the exact ball, the chamfer's cap flat, a size past
//! the room refused - and a fillet at a point where sharp edges meet (a cube corner) refused by
//! name. Parameter rows are loops over the same-named test.

use std::collections::{HashMap, HashSet};

use geometry_processing::bevel::level_line::point_bevel::{
    self, PointBevelKind, PointBevelRefusal,
};
use geometry_processing::bevel::level_line::point_candidates;
use geometry_processing::bevel::level_line::point_star::PointStar;
use geometry_processing::bevel::level_line::{LevelLineMesh, LevelLineTriangle};
use vector_math::number_format::format_f64_max_decimals;
use vector_math::{Matrix4X4, Vector3};

const SIDES: i32 = 40;
const RADIUS: f64 = 1.0;
const HEIGHT: f64 = 2.0;
const THRESHOLD: f64 = 20.0;

fn half_angle() -> f64 {
    RADIUS.atan2(HEIGHT)
}

type Triangles = Vec<(i32, i32, i32)>;

#[test]
fn a_cones_apex_is_a_candidate_and_its_rim_and_sides_are_not() {
    let (positions, triangles) = cone(false);
    let candidates = point_candidates::find(&positions, &triangles, THRESHOLD);
    assert_eq!(candidates, vec![0]);
    let apex = point_candidates::angle_defect(&positions, &triangles, 0);
    let rim = point_candidates::angle_defect(&positions, &triangles, 1);
    println!("cone apex defect {apex:.2}, base rim {rim:.2}");
    assert!(apex > 195.0 && apex < 205.0, "{apex}");
    assert!(rim > 10.0 && rim < 16.0, "{rim}");
}

#[test]
fn every_cube_corner_is_a_candidate_with_defect_ninety() {
    let (positions, triangles) = cube(2.0);
    assert_eq!(
        point_candidates::find(&positions, &triangles, THRESHOLD).len(),
        8
    );
    for v in 0..8 {
        assert!((point_candidates::angle_defect(&positions, &triangles, v) - 90.0).abs() < 1e-9);
    }
}

#[test]
fn a_cylinders_rim_is_not_a_candidate() {
    let (positions, triangles) = cylinder();
    let rim = point_candidates::angle_defect(&positions, &triangles, 1);
    println!("cylinder rim defect {rim:.2}");
    assert!(rim > 8.0 && rim < 10.0, "{rim}");
    assert!(point_candidates::find(&positions, &triangles, THRESHOLD).is_empty());
}

#[test]
fn a_flat_faces_interior_vertex_is_not_a_candidate() {
    let mut positions = Vec::new();
    let mut triangles = Vec::new();
    for y in 0..3 {
        for x in 0..3 {
            positions.push(Vector3::new(x as f64, y as f64, 0.0));
        }
    }

    for y in 0..2 {
        for x in 0..2 {
            let a = y * 3 + x;
            triangles.push((a, a + 1, a + 4));
            triangles.push((a, a + 4, a + 3));
        }
    }

    assert!(point_candidates::angle_defect(&positions, &triangles, 4).abs() < 1e-9);
    assert!(!point_candidates::find(&positions, &triangles, THRESHOLD).contains(&4));
}

#[test]
fn a_cones_apex_builds_across_its_room() {
    for kind in [PointBevelKind::Fillet, PointBevelKind::Chamfer] {
        for fraction in [0.1, 0.5, 0.9, 1.0] {
            let (positions, triangles) = cone(true);
            let apex = positions[0];
            let room = point_bevel::room(&positions, &triangles, 0, kind, THRESHOLD);
            let size = room * fraction;
            let result =
                point_bevel::build_one(&positions, &triangles, 0, size, kind, 8, THRESHOLD)
                    .expect("no fault");
            assert_eq!(result.refusal, None, "{kind:?} at {fraction}");
            let mesh_rc = result.mesh.expect("built");
            let mesh = mesh_rc.borrow();
            let removed = volume_of(&positions, &triangles) - volume(&mesh);
            println!("cone {kind:?} at {fraction} of room {room:.4}: volume removed {removed}");
            assert_solid(&mesh);
            assert!(removed > 0.0);

            if kind == PointBevelKind::Fillet {
                // the exact sphere tangent to the true cone: centre on the axis r / sin(a) below
                // the apex
                let centre = apex - Vector3::new(0.0, 0.0, size / half_angle().sin());
                let setback = size / half_angle().tan();
                let worst = patch_vertices(&mesh, apex, setback)
                    .into_iter()
                    .map(|v| ((v - centre).length() - size).abs())
                    .fold(f64::NEG_INFINITY, f64::max);
                println!(
                    "  fillet patch worst deviation from the exact ball {:.2}% of r",
                    100.0 * worst / size
                );
                assert!(worst <= 0.05 * size, "{fraction}: {worst}");
            } else {
                let cap = patch_vertices(&mesh, apex, size);
                let max = cap.iter().map(|v| v.z).fold(f64::NEG_INFINITY, f64::max);
                let min = cap.iter().map(|v| v.z).fold(f64::INFINITY, f64::min);
                assert!(
                    max - min < 1e-9,
                    "the chamfer's cap is one plane square to the axis"
                );
            }
        }
    }
}

#[test]
fn a_cones_apex_refuses_past_its_room() {
    for kind in [PointBevelKind::Fillet, PointBevelKind::Chamfer] {
        let (positions, triangles) = cone(true);
        let room = point_bevel::room(&positions, &triangles, 0, kind, THRESHOLD);
        let result =
            point_bevel::build_one(&positions, &triangles, 0, room * 1.1, kind, 8, THRESHOLD)
                .expect("no fault");
        assert!(!result.is_built());
        assert_eq!(result.failure, PointBevelRefusal::TooLarge);
        let refusal = result.refusal.expect("refused");
        assert!(
            refusal.contains(&format_f64_max_decimals(room, 3)),
            "{refusal}"
        );
    }
}

#[test]
fn a_cube_corner_chamfer_removes_exactly_the_tetrahedron() {
    let (positions, triangles) = cube(2.0);
    let room = point_bevel::room(
        &positions,
        &triangles,
        1,
        PointBevelKind::Chamfer,
        THRESHOLD,
    );
    assert!(
        (room - 2.0).abs() < 1e-12,
        "the plane reaches the neighbouring corners"
    );
    const D: f64 = 0.8;
    let result = point_bevel::build_one(
        &positions,
        &triangles,
        1,
        D,
        PointBevelKind::Chamfer,
        8,
        THRESHOLD,
    )
    .expect("no fault");
    assert_eq!(result.refusal, None);
    let mesh_rc = result.mesh.expect("built");
    let mesh = mesh_rc.borrow();
    assert_solid(&mesh);
    let removed = volume_of(&positions, &triangles) - volume(&mesh);
    assert!((removed - D * D * D / 6.0).abs() < 1e-12, "{removed}");
}

#[test]
fn two_corners_of_one_edge_chamfer_in_one_call() {
    // corners 0 and 1 share an edge of length 2: at 0.8 each ring stops 0.4 short of the other's
    let (positions, triangles) = cube(2.0);
    const D: f64 = 0.8;
    let built = point_bevel::build(
        &positions,
        &triangles,
        &[(0, D), (1, D)],
        PointBevelKind::Chamfer,
        8,
        THRESHOLD,
    )
    .expect("no fault");
    assert!(
        built.points.iter().all(|p| p.is_built()),
        "{:?}",
        built.points.iter().map(|p| &p.refusal).collect::<Vec<_>>()
    );
    let mesh_rc = built.mesh.expect("built");
    let mesh = mesh_rc.borrow();
    assert_solid(&mesh);
    let removed = volume_of(&positions, &triangles) - volume(&mesh);
    assert!((removed - 2.0 * D * D * D / 6.0).abs() < 1e-12, "{removed}");
}

#[test]
fn two_corners_whose_rings_would_meet_both_refuse_naming_the_room_beside_the_other() {
    // at 1.2 each ring reaches 1.2 along the shared edge of length 2, so each has room for
    // 2 - 1.2 = 0.8
    let (positions, triangles) = cube(2.0);
    let built = point_bevel::build(
        &positions,
        &triangles,
        &[(0, 1.2), (1, 1.2)],
        PointBevelKind::Chamfer,
        8,
        THRESHOLD,
    )
    .expect("no fault");
    assert!(built.mesh.is_none());
    for point in &built.points {
        assert_eq!(point.failure, PointBevelRefusal::TooLarge);
        assert!((point.room - 0.8).abs() < 1e-12, "{}", point.room);
        assert!(point.refusal.as_deref().unwrap_or("").contains("0.8"));
    }
}

#[test]
fn a_cube_corner_refuses_a_fillet_naming_its_sharp_edges() {
    let (positions, triangles) = cube(2.0);
    let result = point_bevel::build_one(
        &positions,
        &triangles,
        1,
        0.5,
        PointBevelKind::Fillet,
        8,
        THRESHOLD,
    )
    .expect("no fault");
    assert!(!result.is_built());
    assert_eq!(result.failure, PointBevelRefusal::PointHasSharpEdges);
    assert!(result
        .refusal
        .as_deref()
        .unwrap_or("")
        .contains("fillet the edges instead"));
}

#[test]
fn every_candidate_takes_a_chamfer_and_only_a_point_without_sharp_edges_a_fillet() {
    let (cone_positions, cone_triangles) = cone(true);
    let cone_candidates = point_candidates::describe(&cone_positions, &cone_triangles, THRESHOLD);
    assert_eq!(
        cone_candidates.iter().map(|c| c.vertex).collect::<Vec<_>>(),
        vec![0]
    );
    assert!(
        cone_candidates[0].allows_fillet,
        "no sharp edge meets a cone's tip"
    );

    let (cube_positions, cube_triangles) = cube(2.0);
    let cube_candidates = point_candidates::describe(&cube_positions, &cube_triangles, THRESHOLD);
    assert_eq!(cube_candidates.len(), 8);
    assert!(
        cube_candidates.iter().all(|c| !c.allows_fillet),
        "three sharp edges meet every cube corner"
    );
}

#[test]
fn each_candidates_sharp_edges_read_round_it_match_its_whole_mesh_star() {
    // A cone at a threshold low enough to offer its base rim too, beside a cube: points with and
    // without sharp edges in one mesh, each answered from its own triangles.
    let (mut positions, mut triangles) = cone(false);
    let (cube_positions, cube_triangles) = cube(2.0);
    let offset = positions.len() as i32;
    positions.extend(
        cube_positions
            .iter()
            .map(|p| *p + Vector3::new(5.0, 0.0, 0.0)),
    );
    triangles.extend(
        cube_triangles
            .iter()
            .map(|&(a, b, c)| (a + offset, b + offset, c + offset)),
    );
    let threshold = 10.0;

    let described = point_candidates::describe(&positions, &triangles, threshold);

    assert_eq!(
        described.len(),
        1 + SIDES as usize + 8,
        "the apex, the base rim and the cube's corners"
    );
    assert!(described.iter().any(|c| c.allows_fillet));
    assert!(described.iter().any(|c| !c.allows_fillet));
    for candidate in &described {
        let star = PointStar::new(&positions, &triangles, candidate.vertex, threshold);
        assert_eq!(
            candidate.allows_fillet,
            !star.has_sharp_edges(),
            "vertex {}",
            candidate.vertex
        );
    }
}

#[test]
fn a_cone_fillet_is_the_same_under_a_rigid_motion() {
    let (positions, triangles) = cone(true);
    let motion = Matrix4X4::create_rotation(Vector3::new(0.4, -0.7, 0.2))
        * Matrix4X4::create_translation(3.0, -5.0, 11.0);
    let moved: Vec<Vector3> = positions.iter().map(|p| p.transform(motion)).collect();
    let r = 0.5 * point_bevel::room(&positions, &triangles, 0, PointBevelKind::Fillet, THRESHOLD);
    let here = point_bevel::build_one(
        &positions,
        &triangles,
        0,
        r,
        PointBevelKind::Fillet,
        8,
        THRESHOLD,
    )
    .expect("no fault");
    let there = point_bevel::build_one(
        &moved,
        &triangles,
        0,
        r,
        PointBevelKind::Fillet,
        8,
        THRESHOLD,
    )
    .expect("no fault");
    assert_eq!(here.refusal, None);
    assert_eq!(there.refusal, None);
    assert!((there.room - here.room).abs() < 1e-9);
    let here_mesh_rc = here.mesh.expect("built");
    let here_mesh = here_mesh_rc.borrow();
    let there_mesh_rc = there.mesh.expect("built");
    let there_mesh = there_mesh_rc.borrow();
    assert_eq!(
        live_triangles(&there_mesh).len(),
        live_triangles(&here_mesh).len()
    );
    assert!((volume(&there_mesh) - volume(&here_mesh)).abs() < 1e-9);

    // compared as point sets: the refinement may number the same vertices differently once moved
    let there_live: Vec<Vector3> = live_vertices(&there_mesh)
        .into_iter()
        .map(|v| there_mesh.position(v))
        .collect();
    let worst = live_vertices(&here_mesh)
        .into_iter()
        .map(|v| here_mesh.position(v).transform(motion))
        .map(|p| {
            there_live
                .iter()
                .map(|&q| (p - q).length())
                .fold(f64::INFINITY, f64::min)
        })
        .fold(f64::NEG_INFINITY, f64::max);
    assert!(
        worst < 1e-8,
        "the moved build is the build moved (coordinates near 15, so round-off near 1e-9): {worst}"
    );
}

/// The live triangles' vertices, each once, in first-seen order.
fn live_vertices(mesh: &LevelLineMesh) -> Vec<i32> {
    let mut seen = HashSet::new();
    let mut vertices = Vec::new();
    for t in live_triangles(mesh) {
        for v in [t.a, t.b, t.c] {
            if seen.insert(v) {
                vertices.push(v);
            }
        }
    }

    vertices
}

/// Live vertices no farther than `reach` from the point - the ring and the fill.
fn patch_vertices(mesh: &LevelLineMesh, point: Vector3, reach: f64) -> Vec<Vector3> {
    live_vertices(mesh)
        .into_iter()
        .map(|v| mesh.position(v))
        .filter(|p| (*p - point).length() <= reach * (1.0 + 1e-9))
        .collect()
}

fn live_triangles(mesh: &LevelLineMesh) -> Vec<LevelLineTriangle> {
    (0..mesh.triangle_count())
        .filter(|&t| !mesh.is_removed(t))
        .map(|t| mesh.triangles()[t as usize])
        .collect()
}

fn assert_solid(mesh: &LevelLineMesh) {
    let live = live_triangles(mesh);
    let mut directed = HashSet::new();
    for t in &live {
        for e in [(t.a, t.b), (t.b, t.c), (t.c, t.a)] {
            assert!(directed.insert(e), "edge {e:?} is walked once each way");
        }

        let p = mesh.position(t.a);
        let area = (mesh.position(t.b) - p)
            .cross(mesh.position(t.c) - p)
            .length()
            / 2.0;
        assert!(area > 1e-12);
    }

    for &(a, b) in &directed {
        assert!(
            directed.contains(&(b, a)),
            "edge {a}-{b} is closed and consistently oriented"
        );
    }

    // one shell: every live triangle reached from the first across shared edges
    let mut by_edge = HashMap::new();
    for (i, t) in live.iter().enumerate() {
        by_edge.insert((t.a, t.b), i);
        by_edge.insert((t.b, t.c), i);
        by_edge.insert((t.c, t.a), i);
    }

    let mut seen = vec![false; live.len()];
    let mut stack = vec![0usize];
    seen[0] = true;
    let mut count = 1;
    while let Some(i) = stack.pop() {
        let t = live[i];
        for (a, b) in [(t.a, t.b), (t.b, t.c), (t.c, t.a)] {
            let next = by_edge[&(b, a)];
            if !seen[next] {
                seen[next] = true;
                count += 1;
                stack.push(next);
            }
        }
    }

    assert_eq!(count, live.len(), "the part is one shell");
}

fn volume(mesh: &LevelLineMesh) -> f64 {
    live_triangles(mesh)
        .iter()
        .map(|t| {
            mesh.position(t.a)
                .dot(mesh.position(t.b).cross(mesh.position(t.c)))
        })
        .sum::<f64>()
        / 6.0
}

fn volume_of(positions: &[Vector3], triangles: &[(i32, i32, i32)]) -> f64 {
    triangles
        .iter()
        .map(|&(a, b, c)| {
            positions[a as usize].dot(positions[b as usize].cross(positions[c as usize]))
        })
        .sum::<f64>()
        / 6.0
}

/// Builds the cone's apex (at half its room) or the cube's corner 1 (at 0.8)
/// as the tests above do, for SliverSplitGuardTests' meter to read whether the sliver guard fires
/// on a point bevel.
pub(crate) fn build_for_meter(is_cone: bool, kind: PointBevelKind) {
    let (positions, triangles) = if is_cone { cone(true) } else { cube(2.0) };
    let corner = if is_cone { 0 } else { 1 };
    let size = if is_cone {
        0.5 * point_bevel::room(&positions, &triangles, 0, kind, THRESHOLD)
    } else {
        0.8
    };
    let _ = point_bevel::build_one(&positions, &triangles, corner, size, kind, 8, THRESHOLD);
}

/// A right cone (apex vertex 0, rim 1..Sides at z = 0) closed by a base disk, or standing on a
/// cylinder.
fn cone(on_cylinder: bool) -> (Vec<Vector3>, Triangles) {
    let mut positions = vec![Vector3::new(0.0, 0.0, HEIGHT)];
    let mut triangles = Vec::new();
    positions.extend(ring(0.0));
    let rim = |i: i32| 1 + (i % SIDES);
    for i in 0..SIDES {
        triangles.push((0, rim(i), rim(i + 1)));
    }

    let mut base_ring = rim(0);
    if on_cylinder {
        positions.extend(ring(-1.0));
        let bottom = |i: i32| 1 + SIDES + (i % SIDES);
        for i in 0..SIDES {
            triangles.push((rim(i), bottom(i), bottom(i + 1)));
            triangles.push((rim(i), bottom(i + 1), rim(i + 1)));
        }

        base_ring = bottom(0);
    }

    let centre = positions.len() as i32;
    positions.push(Vector3::new(0.0, 0.0, if on_cylinder { -1.0 } else { 0.0 }));
    for i in 0..SIDES {
        triangles.push((centre, base_ring + (i + 1) % SIDES, base_ring + i));
    }

    (positions, triangles)
}

/// A closed 40-sided cylinder: top centre 0, top ring 1..Sides, bottom ring, bottom centre.
fn cylinder() -> (Vec<Vector3>, Triangles) {
    let mut positions = vec![Vector3::new(0.0, 0.0, 0.0)];
    positions.extend(ring(0.0));
    positions.extend(ring(-1.0));
    positions.push(Vector3::new(0.0, 0.0, -1.0));
    let mut triangles = Vec::new();
    for i in 0..SIDES {
        let (t0, t1) = (1 + i, 1 + (i + 1) % SIDES);
        let (b0, b1) = (t0 + SIDES, t1 + SIDES);
        triangles.push((0, t0, t1));
        triangles.push((t0, b0, b1));
        triangles.push((t0, b1, t1));
        triangles.push((2 * SIDES + 1, b1, b0));
    }

    (positions, triangles)
}

fn ring(z: f64) -> Vec<Vector3> {
    (0..SIDES)
        .map(|i| {
            let a = 2.0 * std::f64::consts::PI * i as f64 / SIDES as f64;
            Vector3::new(RADIUS * a.cos(), RADIUS * a.sin(), z)
        })
        .collect()
}

/// A cube [0, s]^3, corner index x + 2y + 4z, each face split on one diagonal - corner 1 has its
/// own diagonal on one face only, so the cut there crosses triangles that do not touch the point.
fn cube(s: f64) -> (Vec<Vector3>, Triangles) {
    let positions = (0..8)
        .map(|i| Vector3::new((i & 1) as f64, ((i >> 1) & 1) as f64, ((i >> 2) & 1) as f64) * s)
        .collect();
    let mut triangles = Vec::new();
    for (a, b, c, d) in [
        (0, 2, 3, 1),
        (4, 5, 7, 6),
        (0, 1, 5, 4),
        (2, 6, 7, 3),
        (0, 4, 6, 2),
        (1, 3, 7, 5),
    ] {
        triangles.push((a, b, c));
        triangles.push((a, c, d));
    }

    (positions, triangles)
}
