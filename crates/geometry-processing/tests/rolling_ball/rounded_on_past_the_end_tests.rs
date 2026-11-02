//! A round picked on to a corner already rounded past its ends: an L-shaped bar whose inner corner
//! is a quarter round of radius 1 for one unit at each end and sharp between, the sharp stretch
//! picked just under that radius. Each end face (the flat curved triangle where the rounded stretch
//! meets the sharp one) lies all but a thin crescent inside the round's end section, so the cap
//! cuts it away up to its edge with the rounded corner, and the round flows on into the rounded
//! corner. The rounded corner is in the wall's region, so it is a side of the ball, as the T
//! Handle's web hole wall is the pocket walls'; the floor past each end face is a face of its own.

use std::collections::HashMap;

use geometry_processing::bevel::level_line::band_gap_entry_points::{
    build_pick_with_side_regions, BandGapBuildOptions,
};
use geometry_processing::bevel::level_line::gap_fill::{self, DEFAULT_FILLET_SEGMENTS};
use geometry_processing::bevel::level_line::{
    BandGapRefusal, BandGapResult, BandProfile, GapFillRefusal,
};
use geometry_processing::bevel::rolling_ball::rolling_ball_routing::{
    last_route, reset_to_the_default, set_last_route, set_use_rolling_ball, ROLLING_BALL_ROUTE,
};
use polygon_mesh::csg::boolean_processing::{classify_boolean_operand, BooleanOperandVerdict};
use polygon_mesh::Mesh;
use vector_math::{Matrix4X4, Vector3, Vector3Float};

use super::band_route_tests::{creases, Triangles};

/// The rounded stretches' radius.
const R: f64 = 1.0;
/// How far the bar's slabs reach below the floor and behind the wall.
const SLAB: f64 = 1.0;
/// How far the floor and the wall run out from the corner.
const REACH: f64 = 3.0;
/// The sharp stretch's length.
const LENGTH: f64 = 4.0;
/// The rounded stretch's length at each end.
const ROUNDED: f64 = 1.0;
/// Segments on each rounded stretch's quarter circle.
const ARC: usize = 8;

/// The bar's cross-section in (x, z), counterclockwise, the corner at the origin sharp or rounded.
/// The sharp section also carries the points where the round's arc meets the floor and the wall,
/// so the two sections share them.
fn section(rounded: bool) -> Vec<(f64, f64)> {
    let mut ring = vec![(-SLAB, -SLAB), (REACH, -SLAB), (REACH, 0.0)];
    if rounded {
        for i in 0..=ARC {
            let theta = std::f64::consts::FRAC_PI_2 * (i as f64 / ARC as f64);
            ring.push((R - (R * theta.sin()), R - (R * theta.cos())));
        }
    } else {
        ring.extend([(R, 0.0), (0.0, 0.0), (0.0, R)]);
    }
    ring.extend([(0.0, REACH), (-SLAB, REACH)]);
    ring
}

/// The bar as a closed mesh, every triangle wound outward, with each triangle's region: 1 the
/// floor beside the sharp stretch (z = 0), 2 the wall (x = 0) and the rounded corner, 3 the floor
/// past the end faces, every other face its own.
fn bar() -> ((Vec<Vector3>, Triangles), Vec<u32>) {
    let mut positions: Vec<Vector3> = Vec::new();
    let mut index: HashMap<(i64, i64, i64), i32> = HashMap::new();
    let mut vertex = |x: f64, y: f64, z: f64| -> i32 {
        // Keys at a billionth: the two sections' shared points are the same arithmetic.
        let key = |v: f64| (v * 1e9).round_ties_even() as i64;
        *index.entry((key(x), key(y), key(z))).or_insert_with(|| {
            positions.push(Vector3::new(x, y, z));
            positions.len() as i32 - 1
        })
    };
    let ys = [-ROUNDED, 0.0, LENGTH, LENGTH + ROUNDED];
    let mut triangles: Triangles = Vec::new();
    // The three prisms' sides: rounded, sharp, rounded.
    for (k, rounded) in [(0, true), (1, false), (2, true)] {
        let ring = section(rounded);
        for i in 0..ring.len() {
            let (p, q) = (ring[i], ring[(i + 1) % ring.len()]);
            let (a, b) = (vertex(p.0, ys[k], p.1), vertex(q.0, ys[k], q.1));
            let (c, d) = (vertex(q.0, ys[k + 1], q.1), vertex(p.0, ys[k + 1], p.1));
            triangles.push((a, d, c));
            triangles.push((a, c, b));
        }
    }
    // The bar's two ends: fans from the slabs' outer corner, which sees the whole section.
    let ring = section(true);
    for (y, outward) in [(ys[0], -1.0), (ys[3], 1.0)] {
        let hub = vertex(ring[0].0, y, ring[0].1);
        for i in 1..ring.len() - 1 {
            let (b, c) = (
                vertex(ring[i].0, y, ring[i].1),
                vertex(ring[i + 1].0, y, ring[i + 1].1),
            );
            triangles.push(if outward > 0.0 {
                (hub, c, b)
            } else {
                (hub, b, c)
            });
        }
    }
    // The end faces: the rounded section less the sharp one, a fan from the corner, facing the
    // sharp stretch.
    let arc: Vec<(f64, f64)> = section(true)[3..3 + ARC + 1].to_vec();
    for (y, toward_plus_y) in [(0.0, true), (LENGTH, false)] {
        let corner = vertex(0.0, y, 0.0);
        for pair in arc.windows(2) {
            let (b, c) = (
                vertex(pair[0].0, y, pair[0].1),
                vertex(pair[1].0, y, pair[1].1),
            );
            triangles.push(if toward_plus_y {
                (corner, c, b)
            } else {
                (corner, b, c)
            });
        }
    }
    // Every triangle faces away from the solid: a closed mesh wound outward has a positive volume.
    let volume: f64 = triangles
        .iter()
        .map(|&(a, b, c)| {
            positions[a as usize].dot(positions[b as usize].cross(positions[c as usize])) / 6.0
        })
        .sum();
    assert!(volume > 0.0, "the bar is wound inward: {volume}");
    let regions = triangles
        .iter()
        .map(|&(a, b, c)| {
            let (pa, pb, pc) = (
                positions[a as usize],
                positions[b as usize],
                positions[c as usize],
            );
            let n = (pb - pa).cross(pc - pa).get_normal();
            let all = |f: &dyn Fn(Vector3) -> bool| f(pa) && f(pb) && f(pc);
            if n.z > 1.0 - 1e-9 && all(&|p| p.z.abs() < 1e-12) {
                if all(&|p| p.y > -1e-12 && p.y < LENGTH + 1e-12) {
                    1
                } else {
                    3
                }
            } else if (n.x > 1.0 - 1e-9 && all(&|p| p.x.abs() < 1e-12))
                || all(&|p| p.x > -1e-12 && p.x <= R + 1e-12 && p.z > -1e-12 && p.z <= R + 1e-12)
                    && n.y.abs() < 1e-9
            {
                2
            } else {
                // Every other face by its normal and its plane's offset.
                let d = n.dot(pa);
                let key = |v: f64| ((v * 1e6).round_ties_even() as i64).rem_euclid(97) as u32;
                10 + key(n.x) + (97 * key(n.y)) + (97 * 97 * key(n.z)) + (97 * 97 * 97 * key(d))
            }
        })
        .collect();
    ((positions, triangles), regions)
}

/// The bar's own volume, read off its triangles.
fn bar_volume() -> f64 {
    let ((positions, triangles), _) = bar();
    triangles
        .iter()
        .map(|&(a, b, c)| {
            positions[a as usize].dot(positions[b as usize].cross(positions[c as usize])) / 6.0
        })
        .sum()
}

/// The sharp stretch of the corner picked at `size` with the rolling ball's switch on.
fn the_sharp_stretch(size: f64) -> (BandGapResult, Option<String>) {
    let ((positions, triangles), regions) = bar();
    let at = |p: Vector3| {
        positions
            .iter()
            .position(|&q| (q - p).length() < 1e-12)
            .expect("a section point") as i32
    };
    let chain = vec![
        at(Vector3::new(0.0, 0.0, 0.0)),
        at(Vector3::new(0.0, LENGTH, 0.0)),
    ];
    let sharp = creases(&positions, &triangles);
    set_use_rolling_ball(true);
    set_last_route(None);
    let result = build_pick_with_side_regions(
        &positions,
        &triangles,
        &[chain],
        &[false],
        Some(sharp),
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

/// Each end face is cut away right up to its edge with the rounded corner, a side's surface the
/// side's own band never reaches: that edge is the end's run, not the wall's rail, so each side
/// leaves one open run and each end one.
#[test]
fn an_end_face_cut_up_to_a_sides_surface_leaves_that_edge_as_the_ends_run() {
    let (ball, route) = the_sharp_stretch(0.9);
    assert_eq!(route.as_deref(), Some(ROLLING_BALL_ROUTE));
    assert_eq!(
        ball.refusal,
        BandGapRefusal::None,
        "{:?}",
        ball.rolling_ball_refusal
    );
    // Both ends stand on end faces: neither is a free end's nose.
    let mesh = ball.mesh.as_ref().expect("a built band has its mesh");
    assert_eq!(mesh.borrow().ball_free_ends, [false, false]);
    let runs = |side: i32| -> Vec<(bool, usize)> {
        ball.loops
            .iter()
            .filter(|l| l.side == side)
            .map(|l| (l.is_closed, l.vertices.len()))
            .collect()
    };
    for side in 1..=4 {
        let runs = runs(side);
        assert!(
            runs.len() == 1 && !runs[0].0,
            "side {side} leaves {runs:?}; every run {:?}",
            (1..=4)
                .map(|s| (s, ball.loops.iter().filter(|l| l.side == s).count()))
                .collect::<Vec<_>>()
        );
    }
}

/// What the fill made of a built band: whether the filled mesh is closed (every directed edge of
/// its live triangles run once and once back), whether the Boolean kernel reads it as crossing
/// itself (taken in single precision, as the app publishes it), its volume and the vertices it
/// minted. `Err` with the fill's refusal where it refused.
pub struct Filled {
    pub closed: bool,
    pub crosses_itself: bool,
    pub volume: f64,
    /// Where each vertex the fill minted stands.
    pub minted: Vec<Vector3>,
}

/// The fill of `ball` (see [`Filled`]).
pub fn filled(ball: &BandGapResult) -> Result<Filled, String> {
    let fill = gap_fill::fill(ball, BandProfile::Fillet, DEFAULT_FILLET_SEGMENTS)
        .map_err(|error| format!("{error:?}"))?;
    if fill.refusal != GapFillRefusal::None {
        return Err(format!("{:?} at {:?}", fill.refusal, fill.open_edge));
    }
    let mesh = fill.mesh.as_ref().expect("the filled mesh").borrow();
    let mut directed: HashMap<(i32, i32), i32> = HashMap::new();
    let mut answer = Mesh::new();
    answer.vertices.extend(
        mesh.vertices()
            .iter()
            .map(|&p| Vector3Float::from_vector3(p)),
    );
    let mut volume = 0.0;
    for t in 0..mesh.triangle_count() {
        if mesh.is_removed(t) {
            continue;
        }
        let face = mesh.triangles()[t as usize];
        for edge in [(face.a, face.b), (face.b, face.c), (face.c, face.a)] {
            *directed.entry(edge).or_default() += 1;
        }
        answer.faces.add(face.a, face.b, face.c, &answer.vertices);
        let (a, b, c) = (
            mesh.position(face.a),
            mesh.position(face.b),
            mesh.position(face.c),
        );
        volume += a.dot(b.cross(c)) / 6.0;
    }
    let closed = directed
        .iter()
        .all(|(&(a, b), &count)| count == 1 && directed.get(&(b, a)) == Some(&1));
    let crosses_itself = classify_boolean_operand(&answer, Matrix4X4::IDENTITY, false)
        == BooleanOperandVerdict::SelfIntersecting;
    let minted = fill
        .added_vertices
        .iter()
        .map(|&v| mesh.position(v))
        .collect();
    Ok(Filled {
        closed,
        crosses_itself,
        volume,
        minted,
    })
}

/// At 0.9 the round's end section stands 0.1 inside the rounded stretches' circle at each end face,
/// so the cap is a crescent between the two, cut from the end face up to its edge with the rounded
/// corner: with that edge the end's run, the fill closes the round at both ends, the mesh is closed
/// and clean, and the round adds its section's volume. (At the radius itself each side's contact
/// lands on the end of its edge with the end face, which the ball refuses as a contact on a rim.)
#[test]
fn a_round_just_under_the_rounded_corners_radius_fills_closed_into_it() {
    {
        let size = 0.9;
        let (ball, route) = the_sharp_stretch(size);
        assert_eq!(route.as_deref(), Some(ROLLING_BALL_ROUTE), "{size}");
        assert_eq!(ball.refusal, BandGapRefusal::None, "{size}");
        let filled = filled(&ball).unwrap_or_else(|why| panic!("{size}: {why}"));
        assert!(filled.closed, "{size}");
        assert!(!filled.crosses_itself, "{size}");
        // By hand: the round adds the quarter square less the round's section along the sharp
        // stretch, and its end caps lie flat in the end faces. The rows are chords of the arc, n
        // of them over its quarter turn, so the section is n triangles of area size^2 sin(pi / 2n)
        // / 2 about the ball's centre, and the round adds (1 - (n / 2) sin(pi / 2n)) size^2 LENGTH
        // (0.7602 at 0.9 with 4 rows' segments).
        let added = filled.volume - bar_volume();
        let n = f64::from(DEFAULT_FILLET_SEGMENTS);
        let section = (n / 2.0) * (std::f64::consts::FRAC_PI_2 / n).sin();
        let by_hand = (1.0 - section) * size * size * LENGTH;
        assert!(
            (added - by_hand).abs() < 1e-3 * by_hand,
            "{size}: {added} against {by_hand}"
        );
    }
}

/// At 0.99 the end run, the rounded corner's circle in 8 chords, stands within the last row's
/// sagitta of the round's end section (its 4 chords bow 0.019 inside the arc, which stands 0.01
/// inside the circle). Minted as the angle's or the ball's arc, the last row crossed the run and
/// the cap zipped between them folded over itself in the end face. That run lies on the end
/// station's circle and turns one way about its centre, so it IS the last row: nothing is minted in
/// either end face, the cell before it is zipped from the row before onto the run, no cap is laid,
/// and the closed answer does not cross itself. This is the T Handle's web end in small.
#[test]
fn a_round_whose_end_run_lies_on_its_end_section_takes_that_run_as_its_last_row() {
    let (ball, route) = the_sharp_stretch(0.99);
    assert_eq!(route.as_deref(), Some(ROLLING_BALL_ROUTE));
    assert_eq!(ball.refusal, BandGapRefusal::None);
    let filled = filled(&ball).expect("the fill closes");
    assert!(filled.closed);
    assert!(!filled.crosses_itself);
    for p in &filled.minted {
        assert!(
            p.y.abs() > 1e-9 && (p.y - LENGTH).abs() > 1e-9,
            "a fill vertex is minted in an end face: {p:?}"
        );
    }
}
