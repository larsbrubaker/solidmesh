//! The level line's band built with its setbacks from the rolling ball
//! (`level_line::rolling_ball_setbacks`, behind `rolling_ball::rolling_ball_routing`), held to the
//! level line's own build where the two must agree: on a flat corner the ball's contacts stand
//! r tan(phi / 2) from the edge, exactly the setback the level line works out from the angle, so
//! the band comes out the same on both routes. And a pass the ball cannot take whole keeps the
//! level line's setbacks for every chain, with the reason in the route.

use std::collections::HashSet;
use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{
    build_pick, BandGapBuildOptions,
};
use geometry_processing::bevel::level_line::{BandGapRefusal, BandGapResult, BandProfile};
use geometry_processing::bevel::rolling_ball::rolling_ball_routing::{
    last_route, reset_to_the_default, set_last_route, set_use_rolling_ball,
    the_running_app_wants_it_on, ROLLING_BALL_ROUTE,
};
use vector_math::Vector3;

/// How far a vertex of the two routes' meshes may stand apart: the ball's solve is exact to about
/// 1e-9 of the part's size, and a cut point moves no further than the setback it was cut to.
const SAME_MESH: f64 = 1e-8;

pub type Triangles = Vec<(i32, i32, i32)>;

/// A convex solid's triangles, each wound to face away from the solid's centre.
fn outward(positions: &[Vector3], triangles: &[(i32, i32, i32)]) -> Triangles {
    let mut centre = Vector3::ZERO;
    for p in positions {
        centre += *p;
    }
    centre *= 1.0 / positions.len() as f64;
    triangles
        .iter()
        .map(|&(a, b, c)| {
            let (pa, pb, pc) = (
                positions[a as usize],
                positions[b as usize],
                positions[c as usize],
            );
            let normal = (pb - pa).cross(pc - pa);
            if normal.dot(pa - centre) < 0.0 {
                (a, c, b)
            } else {
                (a, b, c)
            }
        })
        .collect()
}

/// A box from `min` to `max`: bottom corners 0 to 3, top 4 to 7, counterclockwise from (-, -).
pub fn a_box(min: Vector3, max: Vector3) -> (Vec<Vector3>, Triangles) {
    let positions = vec![
        Vector3::new(min.x, min.y, min.z),
        Vector3::new(max.x, min.y, min.z),
        Vector3::new(max.x, max.y, min.z),
        Vector3::new(min.x, max.y, min.z),
        Vector3::new(min.x, min.y, max.z),
        Vector3::new(max.x, min.y, max.z),
        Vector3::new(max.x, max.y, max.z),
        Vector3::new(min.x, max.y, max.z),
    ];
    let quads = [
        [0, 1, 2, 3],
        [4, 5, 6, 7],
        [0, 1, 5, 4],
        [1, 2, 6, 5],
        [2, 3, 7, 6],
        [3, 0, 4, 7],
    ];
    let mut triangles = Vec::new();
    for q in quads {
        triangles.push((q[0], q[1], q[2]));
        triangles.push((q[0], q[2], q[3]));
    }
    let triangles = outward(&positions, &triangles);
    (positions, triangles)
}

/// A prism of length 2 along z on an equilateral triangle of side 2: every edge along its length
/// is a 60 degree corner. Its bottom corners are 0 to 2, (0, 0), (2, 0) and (1, sqrt 3); its top
/// corners 3 to 5 above them.
pub fn a_sixty_degree_prism() -> (Vec<Vector3>, Triangles) {
    let corner = [(0.0, 0.0), (2.0, 0.0), (1.0, 3.0_f64.sqrt())];
    let mut positions = Vec::new();
    for z in [0.0, 2.0] {
        for (x, y) in corner {
            positions.push(Vector3::new(x, y, z));
        }
    }
    let mut triangles = vec![(0, 1, 2), (3, 4, 5)];
    for i in 0..3 {
        let j = (i + 1) % 3;
        triangles.push((i, j, j + 3));
        triangles.push((i, j + 3, i + 3));
    }
    let triangles = outward(&positions, &triangles);
    (positions, triangles)
}

/// The mesh's creases: every edge whose two triangles' normals turn by more than 30 degrees.
pub fn creases(
    positions: &[Vector3],
    triangles: &[(i32, i32, i32)],
) -> Rc<dyn Fn(i32, i32) -> bool> {
    let normal = |t: (i32, i32, i32)| {
        let a = positions[t.0 as usize];
        (positions[t.1 as usize] - a)
            .cross(positions[t.2 as usize] - a)
            .get_normal()
    };
    let key = |a: i32, b: i32| if a < b { (a, b) } else { (b, a) };
    let mut on: Vec<((i32, i32), Vector3)> = Vec::new();
    for &t in triangles {
        for (a, b) in [(t.0, t.1), (t.1, t.2), (t.2, t.0)] {
            on.push((key(a, b), normal(t)));
        }
    }
    let mut sharp = HashSet::new();
    for (i, (edge, n)) in on.iter().enumerate() {
        for (other, m) in &on[i + 1..] {
            if edge == other && n.dot(*m) < (30.0_f64).to_radians().cos() {
                sharp.insert(*edge);
            }
        }
    }
    Rc::new(move |a, b| sharp.contains(&key(a, b)))
}

/// The pick built with the rolling ball's switch as `on`, and the route the pass wrote.
pub fn built(
    (positions, triangles): &(Vec<Vector3>, Triangles),
    chains: &[Vec<i32>],
    size: f64,
    on: bool,
) -> (BandGapResult, Option<String>) {
    set_use_rolling_ball(on);
    set_last_route(None);
    let closed = vec![false; chains.len()];
    let result = build_pick(
        positions,
        triangles,
        chains,
        &closed,
        Some(creases(positions, triangles)),
        size,
        BandProfile::Fillet,
        &BandGapBuildOptions::default(),
    )
    .expect("the pick's arguments are good");
    let route = last_route();
    reset_to_the_default();
    (result, route)
}

/// Both builds' meshes vertex for vertex within [`SAME_MESH`], their triangles the same, and their
/// fields' setbacks the same within the ball's precision.
fn assert_same_band(ball: &BandGapResult, level_line: &BandGapResult) {
    assert_eq!(ball.refusal, BandGapRefusal::None);
    assert_eq!(level_line.refusal, BandGapRefusal::None);
    let ball_mesh = ball
        .mesh
        .as_ref()
        .expect("the ball's band came out")
        .borrow();
    let line_mesh = level_line
        .mesh
        .as_ref()
        .expect("the band came out")
        .borrow();
    assert_eq!(ball_mesh.vertex_count(), line_mesh.vertex_count());
    for (v, (p, q)) in ball_mesh
        .vertices()
        .iter()
        .zip(line_mesh.vertices())
        .enumerate()
    {
        assert!(
            (*p - *q).length() < SAME_MESH,
            "vertex {v}: {p:?} against {q:?}"
        );
    }
    assert_eq!(ball_mesh.triangles(), line_mesh.triangles());
    for t in 0..ball_mesh.triangle_count() {
        assert_eq!(
            ball_mesh.is_removed(t),
            line_mesh.is_removed(t),
            "triangle {t}"
        );
    }
    for (a, b) in ball.chains.iter().zip(&level_line.chains) {
        for (x, y) in a.setbacks.iter().zip(&b.setbacks) {
            assert!((x - y).abs() < SAME_MESH, "setback {x} against {y}");
        }
    }
}

/// A cube's edge, a square corner: the ball's contacts stand the radius from the edge, the level
/// line's setback, so the band is the same on both routes and the route names the ball.
#[test]
fn a_square_corner_bands_the_same_on_the_rolling_ball_as_on_the_level_line() {
    let part = a_box(Vector3::new(-1.0, -1.0, -1.0), Vector3::new(1.0, 1.0, 1.0));
    let chains = [vec![4, 5]];
    let (ball, route) = built(&part, &chains, 0.25, true);
    assert_eq!(route.as_deref(), Some(ROLLING_BALL_ROUTE));
    let (level_line, off_route) = built(&part, &chains, 0.25, false);
    assert_eq!(
        off_route, None,
        "a pass with the switch off writes no route"
    );
    assert_same_band(&ball, &level_line);
    for setback in &ball.chains[0].setbacks {
        assert!((setback - 0.25).abs() < SAME_MESH, "{setback}");
    }
}

/// A 60 degree corner: the contacts stand r tan(60 degrees) = r sqrt 3 from the edge on both
/// routes.
#[test]
fn a_sixty_degree_corner_bands_the_same_on_the_rolling_ball_as_on_the_level_line() {
    let part = a_sixty_degree_prism();
    let chains = [vec![0, 3]];
    let (ball, route) = built(&part, &chains, 0.2, true);
    assert_eq!(route.as_deref(), Some(ROLLING_BALL_ROUTE));
    let (level_line, _) = built(&part, &chains, 0.2, false);
    assert_same_band(&ball, &level_line);
    for setback in &ball.chains[0].setbacks {
        assert!(
            (setback - 0.2 * 3.0_f64.sqrt()).abs() < SAME_MESH,
            "{setback}"
        );
    }
}

/// A slab 0.5 thick with two picked edges at radius 0.4: the ball walks the short upright edge
/// (two broad faces either side, the top and bottom its end faces) but not the long top edge,
/// where a ball 0.4 below the top reaches through the bottom. The whole pass keeps the level
/// line's setbacks - the upright edge's too - and the route names the chain that refused.
#[test]
fn a_pass_the_rolling_ball_cannot_walk_whole_keeps_the_level_line_for_every_chain() {
    let part = a_box(Vector3::new(-5.0, -5.0, 0.0), Vector3::new(5.0, 5.0, 0.5));
    let chains = [vec![0, 4], vec![6, 7]];
    let (ball, route) = built(&part, &chains, 0.4, true);
    let route = route.expect("a pass with the switch on writes its route");
    assert!(
        route.starts_with("level-line: chain 1: the ball has no walk"),
        "{route}"
    );
    let (level_line, _) = built(&part, &chains, 0.4, false);
    assert_eq!(level_line.refusal, BandGapRefusal::None);
    assert_eq!(ball.refusal, BandGapRefusal::None);
    assert_eq!(level_line.chains.len(), 2);
    assert_eq!(ball.chains.len(), level_line.chains.len());
    for (a, b) in ball.chains.iter().zip(&level_line.chains) {
        assert_eq!(
            a.setbacks, b.setbacks,
            "the level line's own setbacks, bit for bit"
        );
    }
}

/// The developer variable turns the ball on only where it plainly says so: unset, "0", "false" or
/// anything else leaves the level line's own setbacks, so no user's fillet changes by accident.
#[test]
fn the_rolling_ball_is_off_unless_the_developer_variable_says_on() {
    assert!(!the_running_app_wants_it_on(None));
    assert!(!the_running_app_wants_it_on(Some("0")));
    assert!(!the_running_app_wants_it_on(Some("false")));
    assert!(!the_running_app_wants_it_on(Some("yes")));
    assert!(the_running_app_wants_it_on(Some("1")));
    assert!(the_running_app_wants_it_on(Some("TRUE")));
}
