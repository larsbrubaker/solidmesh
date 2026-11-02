//! An end's run as the open fill's last row, on the rolling ball's route.
//!
//! At an end face the round's last row is the end station's section: the ball's arc about the end
//! station's centre O from one rail's end S to the other's T. Where the end face is cut away up to
//! a side's own surface (a rounded corner the round flows into, the T Handle's hole through its
//! web) the end's run already lies on that arc to within a row's chord sagitta, as a polyline of
//! its own vertices. Minted as a row of its own, the last row's chords and the run's cross one
//! another, and the cap zipped between them folds over itself in the end face. So where the run
//! IS the section, it is taken as the last row ([`end_run_rows`]): nothing is minted in the end
//! face, no cap is laid, and the cell before it is a ladder from the row before onto the run
//! ([`lay_ladder`]), every triangle on vertices the two already share, so the fill stays closed by
//! construction.
//!
//! A run is the last row only when all of these hold: the fill is on the ball's route (the chain's
//! field carries the ball's centres) with end runs (an end-face fill, not a fade), no ray rows, and
//! that end is not clipped, a hull end or carried; the run has at least 3 vertices and shares both
//! ends with the last row; every interior vertex stands within the row's chord sagitta plus 8 snaps
//! of the end station's circle (radius r about O, in the plane of O, S and T), and so does each
//! edge's midpoint; the run turns one way about O, strictly from S to T; and the fill has at least
//! 3 rows, the cell before the last advancing both rails. Every other end keeps its minted row and
//! its cap.

use vector_math::Vector3;

use super::open_gap_fill::OpenGapFillOpenRail;
use super::{corner_fill, ChainField, LevelLineMesh, LevelLineMeshError};

/// How many snaps past the row's chord sagitta an end run's vertex may stand off the end station's
/// circle.
const SNAPS_OFF_THE_CIRCLE: f64 = 8.0;

/// An end run taken as the last row.
#[derive(Clone, Debug, PartialEq)]
pub struct EndRunRow {
    /// The run as the end-face fill hands it over (end 0's from the last row's right end to its
    /// left, end 1's from left to right).
    pub run: Vec<i32>,
    /// Each run vertex's share of the turn about the end station's centre, from the run's first
    /// vertex (0) to its last (1), strictly increasing.
    pub turn: Vec<f64>,
}

/// What [`end_run_rows`] reads besides the mesh, the chain and the rails.
#[derive(Clone, Copy, Debug)]
pub struct EndRunAsk<'a> {
    /// The spokes the rows stand on, in rail order.
    pub spokes: &'a [(i32, i32)],
    /// The end runs at end 0 and end 1.
    pub end_runs: &'a [Vec<i32>],
    /// Which ends keep their own construction: clipped, hull or carried.
    pub kept_ends: [bool; 2],
    /// Whether the rows stand on the end ray.
    pub rows_on_the_ray: bool,
    /// How many segments each row has.
    pub segments: i32,
}

/// Per end (0, 1), the end run taken as that end's last row, or `None` where the end keeps its
/// minted row and cap (the file's header lists when a run is taken).
pub fn end_run_rows(
    mesh: &LevelLineMesh,
    chain: &ChainField,
    left: &OpenGapFillOpenRail,
    right: &OpenGapFillOpenRail,
    ask: EndRunAsk,
) -> [Option<EndRunRow>; 2] {
    let mut rows = [None, None];
    let Some(ball) = chain.ball.as_deref() else {
        return rows;
    };
    let spokes = ask.spokes;
    let count = spokes.len();
    if ask.rows_on_the_ray || ask.segments < 2 || count < 3 || ask.end_runs.len() != 2 {
        return rows;
    }
    if ball.centres.is_empty() || ball.centres.len() != chain.points.len() {
        return rows;
    }
    for (e, row) in rows.iter_mut().enumerate() {
        if ask.kept_ends[e] {
            continue;
        }
        let (last, before) = if e == 0 {
            (0, 1)
        } else {
            (count - 1, count - 2)
        };
        let (end, inner) = (spokes[last], spokes[before]);
        if end.0 == inner.0 || end.1 == inner.1 {
            continue;
        }
        let (l, r) = (
            left.vertices[end.0 as usize],
            right.vertices[end.1 as usize],
        );
        // End 0's run goes from the row's right end to its left, end 1's from left to right.
        let (s, t) = if e == 0 { (r, l) } else { (l, r) };
        let run = &ask.end_runs[e];
        if run.len() < 3 || run[0] != s || run[run.len() - 1] != t {
            continue;
        }
        let centre = if e == 0 {
            ball.centres[0]
        } else {
            ball.centres[ball.centres.len() - 1]
        };
        let points: Vec<Vector3> = run.iter().map(|&v| mesh.position(v)).collect();
        let tolerance_snaps = SNAPS_OFF_THE_CIRCLE * mesh.same_place();
        if let Some(turn) =
            turn_on_the_circle(centre, ball.radius, &points, ask.segments, tolerance_snaps)
        {
            *row = Some(EndRunRow {
                run: run.clone(),
                turn,
            });
        }
    }
    rows
}

/// Each point's share of the turn about `centre` from the first point to the last, where every
/// interior point and every edge's midpoint stands within the chord sagitta of a row of `segments` plus `snaps` of the circle
/// of `radius` about `centre` in the plane of the centre and the two end points, and the turn
/// strictly increases; `None` otherwise.
pub fn turn_on_the_circle(
    centre: Vector3,
    radius: f64,
    points: &[Vector3],
    segments: i32,
    snaps: f64,
) -> Option<Vec<f64>> {
    let (first, last) = (*points.first()?, *points.last()?);
    let (to_s, to_t) = (first - centre, last - centre);
    let normal = to_s.cross(to_t);
    let (normal_length, s_length) = (normal.length(), to_s.length());
    if normal_length.is_nan() || normal_length <= 0.0 || s_length <= 0.0 {
        return None;
    }
    let n = normal * (1.0 / normal_length);
    let e1 = to_s * (1.0 / s_length);
    let e2 = n.cross(e1);
    let angle = |p: Vector3| {
        let d = p - centre;
        d.dot(e2).atan2(d.dot(e1))
    };
    let whole = angle(last);
    if whole.is_nan() || whole <= 0.0 {
        return None;
    }
    let sagitta = radius * (1.0 - (whole / (2.0 * f64::from(segments))).cos());
    let tolerance = sagitta + snaps;
    // How far a point stands from the circle: its height off the plane and its radial miss in it.
    let off = |p: Vector3| {
        let d = p - centre;
        let h = d.dot(n);
        let radial = (d - (n * h)).length() - radius;
        (h * h + radial * radial).sqrt()
    };
    // Each edge's midpoint as well as each vertex: a run whose vertices sit on the circle can
    // still cut across it along an edge (the chord itself, its vertices just off its ends).
    if points
        .windows(2)
        .any(|edge| off((edge[0] + edge[1]) * 0.5) > tolerance)
    {
        return None;
    }
    let mut turn = Vec::with_capacity(points.len());
    turn.push(0.0);
    let mut previous = 0.0;
    for &p in &points[1..points.len() - 1] {
        if off(p) > tolerance {
            return None;
        }
        let at = angle(p);
        if at.is_nan() || at <= previous || at >= whole {
            return None;
        }
        turn.push(at / whole);
        previous = at;
    }
    turn.push(1.0);
    Some(turn)
}

/// Lays the cell between the row `u` (from the cell's first rail vertex to its second, `n + 1`
/// vertices) and the end run `run` taken as the next row, whose turn shares are `turn`: `first` is
/// the rail's stretch from `u[0]` to `run[0]`, `last` the other rail's from `u[n]` to the run's
/// last vertex. The first stretch is fanned onto `u[1]`, the last onto `u[n - 1]`, and between
/// them the row's interior and the run are zipped by their share of the turn (the row's vertex `i`
/// at `i / n`). Each triangle runs `u` forward and the stretches and the run backward, the winding
/// the open fill's quads use.
pub fn lay_ladder(
    mesh: &mut LevelLineMesh,
    u: &[i32],
    first: &[i32],
    run: &[i32],
    turn: &[f64],
    last: &[i32],
    laid: &mut Vec<i32>,
) -> Result<(), LevelLineMeshError> {
    let n = u.len() - 1;
    for k in 0..first.len() - 1 {
        corner_fill::lay(mesh, first[k], u[1], first[k + 1], laid)?;
    }
    let m = run.len() - 1;
    let (mut i, mut j) = (1, 0);
    while i < n - 1 || j < m {
        let row_next = (i + 1) as f64 / n as f64;
        if j == m || (i < n - 1 && row_next <= turn[j + 1]) {
            corner_fill::lay(mesh, run[j], u[i], u[i + 1], laid)?;
            i += 1;
        } else {
            corner_fill::lay(mesh, run[j], u[i], run[j + 1], laid)?;
            j += 1;
        }
    }
    for k in (1..last.len()).rev() {
        corner_fill::lay(mesh, last[k], u[n - 1], last[k - 1], laid)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A quarter circle's points in the plane z = 0 about the origin, radius 2.
    fn quarter(count: usize) -> Vec<Vector3> {
        (0..=count)
            .map(|k| {
                let a = std::f64::consts::FRAC_PI_2 * (k as f64 / count as f64);
                Vector3::new(2.0 * a.cos(), 2.0 * a.sin(), 0.0)
            })
            .collect()
    }

    /// Points on the circle read as their share of the turn: k / count for evenly spaced points.
    #[test]
    fn points_on_the_circle_turn_evenly() {
        let turn = turn_on_the_circle(Vector3::ZERO, 2.0, &quarter(8), 4, 0.0).expect("on it");
        for (k, share) in turn.iter().enumerate() {
            assert!((share - (k as f64 / 8.0)).abs() < 1e-12, "{k}: {share}");
        }
    }

    /// A point off the circle by more than the sagitta (2 (1 - cos(pi / 16)) = 0.0384 at 4
    /// segments), or one that turns back, is no row.
    #[test]
    fn a_point_off_the_circle_or_turning_back_is_no_row() {
        let mut off = quarter(8);
        // 0.03 off: inside the sagitta.
        off[3] *= 1.015;
        assert!(turn_on_the_circle(Vector3::ZERO, 2.0, &off, 4, 0.0).is_some());
        // 0.1 off: outside it.
        off[3] *= 1.05 / 1.015;
        assert!(turn_on_the_circle(Vector3::ZERO, 2.0, &off, 4, 0.0).is_none());
        let mut back = quarter(8);
        back.swap(3, 4);
        assert!(turn_on_the_circle(Vector3::ZERO, 2.0, &back, 4, 0.0).is_none());
    }

    /// A run that is the straight chord S T, with its two interior vertices just off its ends,
    /// has those vertices 0.0199 off the circle (inside the sagitta, 0.0384) and turning one way,
    /// but its middle edge runs along the chord, 2 (1 - cos(pi / 4)) = 0.586 inside the arc
    /// (0.29 r): no row of the round. Each edge's midpoint is held to the circle too.
    #[test]
    fn a_run_along_the_chord_is_no_row() {
        let (s, t) = (Vector3::new(2.0, 0.0, 0.0), Vector3::new(0.0, 2.0, 0.0));
        let chord = [s, s + ((t - s) * 0.01), t - ((t - s) * 0.01), t];
        assert_eq!(turn_on_the_circle(Vector3::ZERO, 2.0, &chord, 4, 0.0), None);
    }
}
