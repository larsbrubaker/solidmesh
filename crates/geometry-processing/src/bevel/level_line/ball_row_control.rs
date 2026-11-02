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
//! rail point that stands off the ball's sphere by more than the row's own chord sagitta (a free
//! end's taper, a rail cut short of the contact), where the ball's tangents are not the band's; a
//! degenerate pair (P and Q opposite or on one ray from O); and a chain end's row whose ball
//! control point leaves the plane of the row the level line draws (P, E, Q), which is the end face
//! where E is the end vertex (the face that row must close flat in). A ball control point within
//! the mesh's `same_place` of the chain point is the chain point, so a flat corner's rows do not
//! move by the solve's noise.
//!
//! Nothing blends a ball row into a fallback row beside it, so the round steps between them: next
//! to a free end's taper (which pulls P and Q in while the centres stay) up to 0.086 at r = 2
//! between rows 0.25 apart, and the same beside a sharp joint. Smoothing that step (the taper read
//! into the centre, or C blended toward E where a rail point leaves the sagitta) is owed.
//!
//! [`BallRows`] is what the ball's route carries onto the chain's own field
//! (`band_gap_setback_fields::fields_at_the_size`); [`ChainField::ball_centre_at`] reads it, and
//! `open_gap_fill::arcs` asks [`row_control`] for each row.

use vector_math::Vector3;

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
}

/// The control point of the row from `p` to `q`: the ball's ([`ball_control`]) where the ball
/// describes the row, and the chain point everywhere else (the file's header lists the fallbacks).
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
    let Some(c) = ball_control(centre, ball.radius, p, q, row.segments) else {
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
/// the three: `centre + radius (u_P + u_Q) / (1 + u_P . u_Q)`. `None` where the ball does not
/// describe a row of `segments` segments from `p` to `q`: either point stands off the sphere by
/// more than the row's chord sagitta, `radius (1 - cos(theta / (2 segments)))` with theta the angle
/// the arc subtends, or the two directions are opposite, the same, or not directions at all.
pub fn ball_control(
    centre: Vector3,
    radius: f64,
    p: Vector3,
    q: Vector3,
    segments: i32,
) -> Option<Vector3> {
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
    if (rho_p - radius).abs() > sagitta || (rho_q - radius).abs() > sagitta {
        return None;
    }
    let control = centre + ((u_p + u_q) * (radius / (1.0 + cos)));
    if !(control.x.is_finite() && control.y.is_finite() && control.z.is_finite()) {
        return None;
    }
    Some(control)
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
            let control = ball_control(centre, r, p, q, 4).expect("the ball describes the row");
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
        let control = ball_control(centre, 2.0, p, q, 4).expect("the ball describes the row");
        let expected = Vector3::new(-a * 0.75 / 1.75, 0.0, 0.0);
        assert!((control - expected).length() < 1e-12, "{control:?}");
        assert!((control.x + 0.566947).abs() < 1e-6, "{control:?}");
        // The two tangent lengths agree, so the row is the circle.
        assert!(((control - p).length() - (control - q).length()).abs() < 1e-12);
    }

    /// A rail point off the sphere by more than the row's chord sagitta is no contact of the ball
    /// (rule d): the step's sagitta at 4 segments is 2 (1 - cos(acos(0.75) / 8)) = 0.00815.
    #[test]
    fn a_rail_point_off_the_ball_by_more_than_the_sagitta_keeps_the_chain_point() {
        let a = step_a();
        let centre = Vector3::new(-a, 0.0, -2.0);
        let q = Vector3::new(0.0, 0.0, -0.5);
        let sagitta = 2.0 * (1.0 - (0.75f64.acos() / 8.0).cos());
        assert!((sagitta - 0.00815).abs() < 1e-5, "{sagitta}");
        let near = Vector3::new(-a, 0.0, 0.5 * sagitta);
        assert!(ball_control(centre, 2.0, near, q, 4).is_some());
        let off = Vector3::new(-a, 0.0, 2.0 * sagitta);
        assert_eq!(ball_control(centre, 2.0, off, q, 4), None);
        let short = Vector3::new(-a, 0.0, -2.0 * sagitta);
        assert_eq!(ball_control(centre, 2.0, short, q, 4), None);
    }

    /// Contacts opposite across the ball, on one ray from its centre, or at the centre itself have
    /// no meeting tangents (rule e).
    #[test]
    fn degenerate_contacts_keep_the_chain_point() {
        let centre = Vector3::new(1.0, 2.0, 3.0);
        let up = Vector3::new(0.0, 0.0, 1.0);
        let opposite = ball_control(centre, 1.0, centre + up, centre - up, 4);
        assert_eq!(opposite, None);
        let same = ball_control(centre, 1.0, centre + up, centre + up, 4);
        assert_eq!(same, None);
        let at_the_centre = ball_control(centre, 1.0, centre, centre + up, 4);
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
