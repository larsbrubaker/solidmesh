//! A round cut flat in its end face's plane reaches nothing past it: two cavities back to back in a
//! block, on a web 1 thick, every wall facing -x one region and every wall facing +y another, so
//! the far cavity's corner is a crease between the chain's own two sides, 1 from the chain's floor
//! end straight through the web's solid. The near cavity's corner, picked with the rolling ball's
//! sides, must not read its room there.

use std::collections::HashMap;
use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{
    build_pick_with_side_regions, BandGapBuildOptions,
};
use geometry_processing::bevel::level_line::{
    BandGapRefusal, BandGapResult, BandProfile, RoomReadingKind,
};
use geometry_processing::bevel::rolling_ball::rolling_ball_routing::{
    last_route, reset_to_the_default, set_last_route, set_use_rolling_ball, ROLLING_BALL_ROUTE,
};
use vector_math::Vector3;

use super::band_route_tests::{creases, Triangles};

const XS: [f64; 4] = [-4.0, -2.0, 2.0, 4.0];
const YS: [f64; 4] = [-4.0, -2.0, 2.0, 4.0];
/// The bottom cavity from -4 to -0.5, the web from -0.5 to 0.5, the top cavity from 0.5 to 4.
const ZS: [f64; 6] = [-6.0, -4.0, -0.5, 0.5, 4.0, 6.0];

fn solid(i: i32, j: i32, k: i32) -> bool {
    let inside = (0..3).contains(&i) && (0..3).contains(&j) && (0..5).contains(&k);
    inside && !(i == 1 && j == 1 && (k == 1 || k == 3))
}

/// The block as the boundary of its solid cells, each cell face two triangles wound outward, every
/// corner a grid point shared by every face that has it. Each triangle's region: 1 for a wall
/// facing -x at x = 2, 2 for a wall facing +y at y = -2, and otherwise one per face direction and
/// plane.
fn back_to_back_cavities() -> ((Vec<Vector3>, Triangles), Vec<u32>) {
    let mut positions = Vec::new();
    let mut index: HashMap<(usize, usize, usize), i32> = HashMap::new();
    let mut vertex = |p: (usize, usize, usize)| -> i32 {
        *index.entry(p).or_insert_with(|| {
            positions.push(Vector3::new(XS[p.0], YS[p.1], ZS[p.2]));
            positions.len() as i32 - 1
        })
    };
    let mut triangles: Triangles = Vec::new();
    let mut regions: Vec<u32> = Vec::new();
    for k in 0..5usize {
        for j in 0..3usize {
            for i in 0..3usize {
                if !solid(i as i32, j as i32, k as i32) {
                    continue;
                }
                // (axis, sign): the face's outward direction.
                for (axis, sign) in [(0, -1), (0, 1), (1, -1), (1, 1), (2, -1), (2, 1)] {
                    let mut n = [i as i32, j as i32, k as i32];
                    n[axis] += sign;
                    if solid(n[0], n[1], n[2]) {
                        continue;
                    }
                    let plane = if sign > 0 {
                        [i, j, k][axis] + 1
                    } else {
                        [i, j, k][axis]
                    };
                    let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
                    let corner = |du: usize, dv: usize| {
                        let mut c = [i, j, k];
                        c[axis] = plane;
                        c[u] += du;
                        c[v] += dv;
                        (c[0], c[1], c[2])
                    };
                    let q = [corner(0, 0), corner(1, 0), corner(1, 1), corner(0, 1)];
                    let q: Vec<i32> = q.iter().map(|&c| vertex(c)).collect();
                    // (u, v, axis) is right-handed, so u then v winds about +axis.
                    let (a, b, c, d) = if sign > 0 {
                        (q[0], q[1], q[2], q[3])
                    } else {
                        (q[0], q[3], q[2], q[1])
                    };
                    let region = match (axis, sign, plane) {
                        (0, -1, 2) => 1,
                        (1, 1, 1) => 2,
                        _ => 10 + (axis as u32) * 20 + if sign > 0 { 10 } else { 0 } + plane as u32,
                    };
                    triangles.push((a, b, c));
                    triangles.push((a, c, d));
                    regions.push(region);
                    regions.push(region);
                }
            }
        }
    }
    ((positions, triangles), regions)
}

/// The top cavity's corner at x = 2, y = -2 (floor 0.5 to ceiling 4) filleted at `size` with the
/// rolling ball's switch on. Its three other vertical corners are no feature edges, so the band's
/// own floods join its two sides round the cavity and the band is tagged from the ball's sides.
fn the_top_cavitys_corner(size: f64) -> (BandGapResult, Option<String>) {
    let ((positions, triangles), regions) = back_to_back_cavities();
    let at = |p: Vector3| {
        positions
            .iter()
            .position(|&q| (q - p).length() < 1e-12)
            .expect("a grid point") as i32
    };
    let chain = vec![
        at(Vector3::new(2.0, -2.0, 0.5)),
        at(Vector3::new(2.0, -2.0, 4.0)),
    ];
    let sharp = creases(&positions, &triangles);
    let corners = [(2.0, 2.0), (-2.0, 2.0), (-2.0, -2.0)];
    let held = positions.clone();
    let blended = move |a: i32, b: i32| {
        let (p, q) = (held[a as usize], held[b as usize]);
        corners.iter().any(|&(x, y)| {
            [p, q]
                .iter()
                .all(|r| r.x == x && r.y == y && r.z >= 0.5 && r.z <= 4.0)
        })
    };
    set_use_rolling_ball(true);
    set_last_route(None);
    let result = build_pick_with_side_regions(
        &positions,
        &triangles,
        &[chain],
        &[false],
        Some(Rc::new(move |a, b| sharp(a, b) && !blended(a, b))),
        size,
        BandProfile::Fillet,
        &BandGapBuildOptions::default(),
        Some(&regions),
    )
    .expect("the pick's arguments are good");
    let route = last_route();
    reset_to_the_default();
    (result, route)
}

/// The ball takes the top cavity's corner, its sides the walls facing -x and +y of both cavities.
/// The far cavity's corner, and its walls' creases onto its ceiling, stand 1 from the chain's floor
/// end, but straight through the web: the round is cut flat in the floor's plane, so none of them
/// sets the room. By hand: the ball between two walls at a right angle sets back exactly its
/// radius, and each wall runs 4 from the chain to the next corner, where the walls leave the
/// side's region, so the room is that reach, 4 less the kept strip's margin; read through the web
/// it was 1 less the margin.
#[test]
fn a_crease_past_the_end_face_through_the_solid_sets_no_room() {
    let (ball, route) = the_top_cavitys_corner(0.5);
    assert_eq!(route.as_deref(), Some(ROLLING_BALL_ROUTE));
    assert!(
        ball.room > 1.0,
        "the far cavity set the room: {}",
        ball.room
    );
    assert!(ball.room < 4.0 && ball.room > 4.0 - 1e-6, "{}", ball.room);
    assert_eq!(ball.room_set_by.kind, RoomReadingKind::Foreign);
}

/// At 2, past the far cavity's reach through the web (1) and under the room (4), the top cavity's
/// corner builds its band, and nothing of the far cavity is cut or split: its walls, still the
/// chain's two regions and within the field's straight reach from the floor end, left the sides
/// with the floor's plane, since the round is cut flat in it (kept in them, the cut runs its level
/// line across them).
#[test]
fn a_band_wider_than_the_web_leaves_the_far_cavity_uncut() {
    let (ball, route) = the_top_cavitys_corner(2.0);
    assert_eq!(route.as_deref(), Some(ROLLING_BALL_ROUTE));
    assert_eq!(
        ball.refusal,
        BandGapRefusal::None,
        "{:?}",
        ball.rolling_ball_refusal
    );
    let mesh = ball.mesh.expect("a built band has its mesh");
    let mesh = mesh.borrow();
    let mut cut = 0;
    for (i, t) in mesh.triangles().iter().enumerate() {
        if !mesh.is_removed(i as i32) {
            continue;
        }
        cut += 1;
        for v in [t.a, t.b, t.c] {
            let p = mesh.position(v);
            assert!(p.z >= 0.5 - 1e-9, "the band cut the far cavity at {p:?}");
        }
    }
    assert!(cut > 0, "the band was cut");
    // Nor is a level line run across it: below the web every vertex is still a grid point.
    for v in 0..mesh.vertex_count() {
        let p = mesh.position(v);
        assert!(
            p.z >= 0.0 || ZS[..3].contains(&p.z),
            "the cut ran across the far cavity at {p:?}"
        );
    }
}
