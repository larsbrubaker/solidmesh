//! A fillet row's control point drawn about the rolling ball's centre, so the round between the two
//! rails is the ball's own arc rather than the arc the chain's angle assumes.
//!
//! A row is a rational quadratic from a rail vertex P on one side to Q on the other through a
//! control point C (`gap_fill::arc_point`); it is the circle tangent to CP at P and CQ at Q where
//! |CP| = |CQ|. The level line takes C as the chain's own point E, which is the right control point
//! only where the round's centre stands square off the chain at the chain's angle (two flat faces).
//! On the ball's route the round is the ball's: P and Q are its contacts, both the radius r from
//! its centre O, so the tangents at P and Q meet at `C = O + r (u_P + u_Q) / (1 + u_P . u_Q)`, with
//! u_P and u_Q the unit directions from O to P and Q; then |CP| = |CQ| = r tan(theta / 2) and the
//! row is the ball's arc. On two flat faces that C is E itself (to the walk's precision), so the
//! snap below keeps such rows exactly as the level line draws them.
//!
//! The chain point is kept wherever the ball does not describe the row (the fallbacks, in the order
//! they are tested; the ray rows by `open_gap_fill::arcs` before it asks): no ball on the chain
//! (the switch off, or a field the ball's route did not make); the row past a chain end or on an
//! end ray (a nose or a past-end row, swept round the end vertex); a tie or mitre spoke, and a row
//! on a segment that touches a sharp joint, whose centre is a mean of two legs' and no ball's; a
//! rail point that stands off the ball's sphere by more than the row's chord sagitta (a rail cut
//! short of the contact by an end face, a rested rim, a side that narrows), where the ball's
//! tangents are not the band's; a ball centre read so far along the chain from the row's own chain
//! point that it alone stands a square row's rails off the sphere by more than that sagitta (the
//! ball read there is not that row's); a degenerate pair (P and Q opposite or on one ray from O);
//! and a chain end's row whose ball control point leaves the plane of the row the level line draws
//! (P, E, Q), which is the end face where E is the end vertex (the face that row must close flat
//! in). A ball control point within the mesh's `same_place` of the chain point is the chain point,
//! so a flat corner's rows do not move by the solve's noise.
//!
//! **A free end's taper fades.** Where a free end's taper (`rolling_ball_free_ends`) shrinks the
//! round's setback below the ball's, the rails are pulled in while the centres stay, so a row there
//! is neither the ball's arc nor the chain point's; drawn as the one or the other, a ball row beside
//! a taper row turned 41 degrees against 90 on the step and the round stood a ridge between them
//! (its middle moving 1.2 times as far as its quarter points). So a row whose mean, or either rail
//! point read along the chain from the row's chain point, lies where the taper has cut the setback
//! takes its control point
//! between the ball's and the chain point's by [`taper_weight`] of the share of the ball's setback
//! the taper leaves there ([`BallRows::taper`], read at the least of the three): the ball's where
//! the round is whole, the chain point's where it has closed onto the end vertex, smoothly
//! between. The ball's control point there is taken whatever the rails' distance off the sphere
//! (the taper puts them off it), and the along-chain fallback still holds.
//!
//! **Why the taper's own share, not the rails' distance off the sphere.** That distance cannot tell
//! a taper from the other ways a rail stands off the ball, and fading by it drew wrong rows in
//! each: a centre misplaced along the chain (a cube turn, radius 3: three from the chain's end the
//! centre read is (-7.45, 7, 7), 0.45 along from its square row's ball at (-7, 7, 7), the rails
//! 0.034 off, so a fade by distance gave the row a control point 0.42 out of its plane and the
//! fill removed 77.71 against the closed form 76.38); rails cut by an end face (the T Handle's
//! junction at R12, beside the web: rails up to 0.6 off a ball whose arc's sagitta is 3.5, so a
//! fade by distance gave them weights near 1 and control points 4 to 5 from the chain point, and
//! both picks crossed themselves); and the taper itself, the one place the round is meant to move
//! from the ball's arc to the end vertex. The taper's share is 1 everywhere but a free end's
//! taper, so every other row keeps the rule above exactly. Beside a sharp joint (no ball centre on
//! that segment) the round still steps from the ball's rows to the chain point's.
//!
//! [`BallRows`] is what the ball's route carries onto the chain's own field
//! (`band_gap_setback_fields::fields_at_the_size`); [`ChainField::ball_centre_at`] reads it, and
//! `open_gap_fill::arcs` asks [`row_control`] for each row.

use vector_math::{ieee_math, Vector3};

use super::ChainField;

/// How near u_P . u_Q may come to -1 before the pair is read as opposite (no tangents meet).
const OPPOSITE: f64 = 1e-9;

/// How near u_P . u_Q may come to 1 before the pair is read as one direction (no arc between).
const SAME_DIRECTION: f64 = 1e-12;

/// The rolling ball's centres along a chain at the size asked, one per chain vertex, and its
/// radius: where each station's round is centred, carried onto the chain's own field so the fill
/// can draw its rows about them.
#[derive(Clone, Debug, PartialEq)]
pub struct BallRows {
    /// The ball's centre at each chain vertex, absolute (not less the chain point).
    pub centres: Vec<Vector3>,
    /// The ball's radius: the size the pass was built at.
    pub radius: f64,
    /// The share of the ball's setback a free end's taper leaves at each chain vertex, the smaller
    /// side's: 1 away from a free end, falling to 0 at its end vertex
    /// (`rolling_ball_setbacks::SideSetbacks::taper`).
    pub taper: Vec<f64>,
}

/// Everything about one row that decides its control point besides its two rail points.
#[derive(Clone, Copy, Debug)]
pub struct RowAsked {
    /// The mean of the two rails' chain parameters, unclamped: outside [0, length] the row is past
    /// a chain end.
    pub mean: f64,
    /// The chain's own point the level line draws the row about (the end vertex past an end).
    pub chain_point: Vector3,
    /// How many segments the row has.
    pub segments: i32,
    /// Whether either rail vertex is a tie or mitre vertex standing for a jump of parameters.
    pub tie_or_mitre: bool,
    /// Whether the row is the first or last of an open fill, at a chain end.
    pub end_row: bool,
    /// How near two points must stand to be one (the mesh's `same_place`).
    pub snap: f64,
    /// The chain's unit direction at the row's mean (zero where the chain has none there): how far
    /// along the chain the ball's centre stands from the row's chain point is read along it.
    pub tangent: Vector3,
}

/// The control point of the row from `p` to `q`: the ball's ([`ball_control`]) where the ball
/// describes the row, faded toward the chain point by [`taper_weight`] in a free end's taper, and
/// the chain point everywhere else (the file's header lists the fallbacks).
pub fn row_control(chain: &ChainField, p: Vector3, q: Vector3, row: RowAsked) -> Vector3 {
    let e = row.chain_point;
    let Some(ball) = chain.ball.as_deref() else {
        return e;
    };
    if row.tie_or_mitre || row.segments < 2 {
        return e;
    }
    let Some(centre) = chain.ball_centre_at(row.mean) else {
        return e;
    };
    let along = (centre - e).dot(row.tangent);
    let Some((ball_c, on_the_ball)) = ball_control(centre, ball.radius, p, q, row.segments, along)
    else {
        return e;
    };
    // Where along the chain the row and each rail point stand, the rail points by their offset
    // along the chain from the row's chain point: a rail point the last whole station's sphere
    // bends in (the field reads it as that station's) still stands in the taper beside it.
    let taper = [
        row.mean,
        row.mean + (p - e).dot(row.tangent),
        row.mean + (q - e).dot(row.tangent),
    ]
    .into_iter()
    .map(|parameter| chain.ball_taper_at(parameter))
    .fold(1.0, ieee_math::min_f64);
    let c = if taper < 1.0 {
        // In a free end's taper: part way from the ball's control point to the chain point.
        e + ((ball_c - e) * taper_weight(taper))
    } else if on_the_ball {
        ball_c
    } else {
        return e;
    };
    if (c - e).length() <= row.snap {
        return e;
    }
    if row.end_row && !in_the_end_plane(c, e, p, q, row.snap) {
        return e;
    }
    c
}

/// Where the tangents to the ball of `radius` about `centre` at `p` and `q` meet, in the plane of
/// the three: `centre + radius (u_P + u_Q) / (1 + u_P . u_Q)`, and whether both points lie on the
/// ball as a row of `segments` segments can tell: within the row's chord sagitta
/// `radius (1 - cos(theta / (2 segments)))` of the sphere, theta the angle the arc subtends.
/// `along` is how far the centre stands along the chain from the row's own chain point. `None`
/// where the ball does not describe the row at all: a centre so far along the chain that it alone
/// stands a square row's rails off the sphere by more than the chord sagitta,
/// `sqrt(radius^2 + along^2) - radius` (the file's header says why); or the two directions
/// opposite, the same, or not directions.
pub fn ball_control(
    centre: Vector3,
    radius: f64,
    p: Vector3,
    q: Vector3,
    segments: i32,
    along: f64,
) -> Option<(Vector3, bool)> {
    let (to_p, to_q) = (p - centre, q - centre);
    let (rho_p, rho_q) = (to_p.length(), to_q.length());
    if !(rho_p > 0.0 && rho_q > 0.0 && radius > 0.0 && segments > 0) {
        return None;
    }
    let (u_p, u_q) = (to_p * (1.0 / rho_p), to_q * (1.0 / rho_q));
    let cos = u_p.dot(u_q).clamp(-1.0, 1.0);
    if 1.0 + cos <= OPPOSITE || 1.0 - cos <= SAME_DIRECTION {
        return None;
    }
    let sagitta = radius * (1.0 - (cos.acos() / (2.0 * f64::from(segments))).cos());
    let along_off = ((radius * radius) + (along * along)).sqrt() - radius;
    if along_off.is_nan() || along_off > sagitta {
        return None;
    }
    let on_the_ball = (rho_p - radius).abs() <= sagitta && (rho_q - radius).abs() <= sagitta;
    let control = centre + ((u_p + u_q) * (radius / (1.0 + cos)));
    if !(control.x.is_finite() && control.y.is_finite() && control.z.is_finite()) {
        return None;
    }
    Some((control, on_the_ball))
}

/// How much of the ball's control point a row takes where a free end's taper leaves `taper` of the
/// ball's setback: the smoothstep `3 t^2 - 2 t^3` of the share, so 1 where the round is whole, 0
/// where it has closed onto the end vertex, and level at both, so the round fades in and out of the
/// taper with no kink.
pub fn taper_weight(taper: f64) -> f64 {
    let t = taper.clamp(0.0, 1.0);
    t * t * (3.0 - (2.0 * t))
}

/// Whether `c` lies within `snap` of the plane of the row the level line draws (`p`, `e`, `q`),
/// which is the end face where `e` is the end vertex: the plane an end row must stay in so the
/// end face closes flat. A degenerate plane (the three on one line) holds nothing.
fn in_the_end_plane(c: Vector3, e: Vector3, p: Vector3, q: Vector3, snap: f64) -> bool {
    let normal = (p - e).cross(q - e);
    let length = normal.length();
    // Zero, or not a number: no plane.
    if length.is_nan() || length <= 0.0 {
        return false;
    }
    ((c - e).dot(normal) / length).abs() <= snap
}

#[cfg(test)]
mod tests {
    use super::*;

    /// sqrt(1.75): how far the step's ball stands in from its edge (radius 2, riser 0.5).
    fn step_a() -> f64 {
        1.75f64.sqrt()
    }

    /// A square corner's ball control point is the corner itself, at any turn of the corner: the
    /// tangents at the two contacts meet on the edge.
    #[test]
    fn a_square_corners_ball_control_is_its_chain_point() {
        let r = 2.0;
        for k in 0..12 {
            let angle = f64::from(k) * 0.5;
            let (s, c) = angle.sin_cos();
            let e = Vector3::new(0.3, -1.1, 0.7);
            // Two unit directions square to each other in the plane y = const, turned by angle.
            let along_a = Vector3::new(c, 0.0, s);
            let along_b = Vector3::new(-s, 0.0, c);
            let centre = e - (along_a * r) - (along_b * r);
            let p = centre + (along_b * r);
            let q = centre + (along_a * r);
            let (control, on_the_ball) =
                ball_control(centre, r, p, q, 4, 0.0).expect("the ball describes the row");
            assert!(on_the_ball);
            assert!((control - e).length() < 1e-12, "{control:?} against {e:?}");
        }
    }

    /// The step: the ball of radius 2 about (-a, 0, -2) rests on the top at (-a, 0, 0) and on the
    /// rim at (0, 0, -0.5). u_P = (0, 0, 1), u_Q = (a, 0, 1.5) / 2, their dot 0.75, so
    /// C = (-a, 0, -2) + 2 (a / 2, 0, 1.75) / 1.75 = (-a 0.75 / 1.75, 0, 0) = (-0.566947, 0, 0).
    #[test]
    fn the_steps_ball_control_is_where_its_tangents_meet() {
        let a = step_a();
        let centre = Vector3::new(-a, 0.0, -2.0);
        let p = Vector3::new(-a, 0.0, 0.0);
        let q = Vector3::new(0.0, 0.0, -0.5);
        let (control, on_the_ball) =
            ball_control(centre, 2.0, p, q, 4, 0.0).expect("the ball describes the row");
        assert!(on_the_ball);
        let expected = Vector3::new(-a * 0.75 / 1.75, 0.0, 0.0);
        assert!((control - expected).length() < 1e-12, "{control:?}");
        assert!((control.x + 0.566947).abs() < 1e-6, "{control:?}");
        // The two tangent lengths agree, so the row is the circle.
        assert!(((control - p).length() - (control - q).length()).abs() < 1e-12);
    }

    /// A rail point off the sphere by more than the row's chord sagitta is no contact of the ball
    /// (rule d): the step's sagitta at 4 segments is 2 (1 - cos(acos(0.75) / 8)) = 0.00815. The
    /// control point is still answered, for a free end's taper to fade by.
    #[test]
    fn a_rail_point_off_the_ball_by_more_than_the_sagitta_is_no_contact() {
        let a = step_a();
        let centre = Vector3::new(-a, 0.0, -2.0);
        let q = Vector3::new(0.0, 0.0, -0.5);
        let sagitta = 2.0 * (1.0 - (0.75f64.acos() / 8.0).cos());
        assert!((sagitta - 0.00815).abs() < 1e-5, "{sagitta}");
        let on = |p: Vector3| ball_control(centre, 2.0, p, q, 4, 0.0).map(|b| b.1);
        assert_eq!(on(Vector3::new(-a, 0.0, 0.5 * sagitta)), Some(true));
        assert_eq!(on(Vector3::new(-a, 0.0, 2.0 * sagitta)), Some(false));
        assert_eq!(on(Vector3::new(-a, 0.0, -2.0 * sagitta)), Some(false));
    }

    /// The cube turn's row three from the chain's end (radius 3, 8 segments): its rails at
    /// (-7, 7, 10) and (-7, 10, 7) are square to the chain, whose ball is centred at (-7, 7, 7),
    /// but the centre read there stands at (-7.45, 7, 7), 0.45 along the chain from the row. That
    /// puts the rails 0.034 off the sphere, which a taper could too, but it is a centre misplaced along the
    /// chain, not a taper: off by sqrt(9 + 0.45^2) - 3 = 0.0336 against the chord sagitta
    /// 3 (1 - cos(pi / 32)) = 0.0144, the ball does not describe the row. The centre in the row's
    /// own plane does, fully.
    #[test]
    fn a_centre_misplaced_along_the_chain_keeps_the_chain_point() {
        let (p, q) = (Vector3::new(-7.0, 7.0, 10.0), Vector3::new(-7.0, 10.0, 7.0));
        let misplaced = Vector3::new(-7.45, 7.0, 7.0);
        assert_eq!(ball_control(misplaced, 3.0, p, q, 8, 0.45), None);
        let square = Vector3::new(-7.0, 7.0, 7.0);
        let (control, on_the_ball) =
            ball_control(square, 3.0, p, q, 8, 0.0).expect("the ball's row");
        assert!(on_the_ball);
        assert!(
            (control - Vector3::new(-7.0, 10.0, 10.0)).length() < 1e-12,
            "{control:?}"
        );
        // Along by less than the sagitta allows is the ball's still.
        let (_, near) = ball_control(square, 3.0, p, q, 8, 0.2).expect("the ball's row");
        assert!(near);
    }

    /// The taper's weight is 1 where the round is whole, 0 where it has closed, one half half way,
    /// level at both ends, and never falls as the share grows.
    #[test]
    fn the_taper_weight_runs_smoothly_from_the_end_vertex_to_the_whole_round() {
        assert_eq!(taper_weight(1.0), 1.0);
        assert_eq!(taper_weight(0.0), 0.0);
        assert_eq!(taper_weight(0.5), 0.5);
        assert_eq!(taper_weight(-0.25), 0.0);
        assert_eq!(taper_weight(1.25), 1.0);
        assert!(1.0 - taper_weight(0.999) < 1e-5 && taper_weight(0.001) < 1e-5);
        let mut last = 0.0;
        for k in 0..=100 {
            let weight = taper_weight(f64::from(k) / 100.0);
            assert!(weight >= last, "{k}: {weight}");
            last = weight;
        }
    }

    /// Contacts opposite across the ball, on one ray from its centre, or at the centre itself have
    /// no meeting tangents (rule e).
    #[test]
    fn degenerate_contacts_keep_the_chain_point() {
        let centre = Vector3::new(1.0, 2.0, 3.0);
        let up = Vector3::new(0.0, 0.0, 1.0);
        let opposite = ball_control(centre, 1.0, centre + up, centre - up, 4, 0.0);
        assert_eq!(opposite, None);
        let same = ball_control(centre, 1.0, centre + up, centre + up, 4, 0.0);
        assert_eq!(same, None);
        let at_the_centre = ball_control(centre, 1.0, centre, centre + up, 4, 0.0);
        assert_eq!(at_the_centre, None);
    }

    /// The end-row plane: a point off the plane of E, P and Q by more than the snap is out, one
    /// within it in, and a degenerate plane holds nothing.
    #[test]
    fn an_end_rows_control_stays_in_the_plane_of_its_end() {
        let e = Vector3::ZERO;
        let p = Vector3::new(-1.0, 0.0, 0.0);
        let q = Vector3::new(0.0, 0.0, -1.0);
        assert!(in_the_end_plane(
            Vector3::new(-0.4, 1e-10, 0.0),
            e,
            p,
            q,
            1e-9
        ));
        assert!(!in_the_end_plane(
            Vector3::new(-0.4, 1e-6, 0.0),
            e,
            p,
            q,
            1e-9
        ));
        let on_a_line = Vector3::new(-2.0, 0.0, 0.0);
        assert!(!in_the_end_plane(Vector3::ZERO, e, p, on_a_line, 1e-9));
    }
}
