//! The growing crease runs' pierce room read from the rolling ball's own centres ([`BallPath`]),
//! where the ball takes a pass of one chain (`one_chain_room` hands the path to
//! `band_room::room_of_cut_mesh`, which reads it in place of
//! `GrowingCreaseRuns::fillet_pierce_room`).
//!
//! **What the pierce room guards.** A growing neighbour's ridge (a crease rising out of the near
//! end of a growing crease run, `GrowingCreaseRuns::ridges`) is where the neighbour's two planes
//! meet. The round runs on virtually past the growing crease and the neighbour's planes clip its
//! rows (`VirtualRail`, `StripClip`), so what the clip meets is the planes' line of intersection,
//! the ridge's whole line and not only its segment: the size at which that line leaves the round's
//! axis by more than the round's radius (as built of `fillet_segments` chords, the cosine of half a
//! chord's angle of it) bounds the room. On the rib of the strip clip tests (a box's top front
//! edge picked past a wedge rib) that is 6 / (3 - sqrt 5) = 7.854; read as the ridge's segment
//! against the ball instead, no room is set, the band is cut at 7.9 and the fill refuses it
//! (`GrowingNeighbourClipRefused`). The bound is not yet exact there: the fill already refuses at
//! 7.8 on both routes, so the clip stops closing somewhat below it.
//!
//! **Why the ball's centres.** `fillet_pierce_room` places the axis by the angle formula: a point
//! of the chain's leg plus the size times one side's in-face direction scaled by the larger side's
//! setback and that side's normal (`t1 u + m1`). Where a side curves away from the edge the ball's
//! centre is not there. Here each leg's axis runs through the ball's own centres at its two ends.
//! Where the sides are flat the two agree (the rib reads the same room either way).
//!
//! **Which legs and vertices, and how far each axis runs.** A ridge is read against every chain
//! leg and inner vertex standing within its reach of the foot of its start on its nearest leg: the
//! ridge's own length plus the size asked times one plus the largest centre per unit, how far
//! from the chain the round reaches at any size up to the size asked ([`BallPath::pierce_room`]).
//! On a finely faceted chain a ridge leaning along it comes nearest the round several legs from
//! its start's. The reach excludes the far side of a ring (which a ridge's infinite line can pass
//! over) only where it is shorter than the ring: on the 256-facet rim of radius 20 in the tests it
//! is about 30, so most of the ring is read there.
//!
//! The round's axis is the polyline of the ball's centres, and a line's nearest approach to a
//! polyline lies within a segment or at a vertex. Each leg's axis at size s is the line through
//! a + s c_a and b + s c_b, tilting with the size where the leg's two centres differ; the size
//! where the ridge's line leaves it is the one division of a linear distance where they agree
//! (`parallel_exit`) and a root of a quartic where they differ (`tilted_exit`), counted only where
//! the ridge's line comes nearest it within the leg ([`BallPath::meets_the_leg`]). Each inner
//! vertex (every vertex of a closed chain) is read as the point c_v(s) = v + s c_v, a quadratic
//! (`vertex_exit`), counted only where the line comes nearest both legs at the vertex past it, so
//! the vertex is the polyline's nearest point. No leg's straight axis is carried past a turn: at a
//! sharp one (a square rim's corner) that would be a round running straight on into the air.
//!
//! Past an open chain's end a leg is carried only where the fill carries the round on past it: an
//! end with an end ray (`virtual_rail_past_the_end`), whose rows run on until the whole row has
//! left the end plane, plus `PAST_THE_LAST_TOUCH` setbacks, before the growing end face's planes
//! clip them (`strip_clip_neighbour`). A row's points stand within the centre's offset of the
//! chain, so for an end face leaning up to 45 degrees off square the row has left the plane within
//! s |c| past the end; a setback is never more than s |c|, so the carry is read up to
//! s |c| (1 + 0.25) past it (c that end's centre per unit). An end face leaning further carries its
//! rows farther than this reads. A square or unrayed end face is cut flat in its plane, a free end
//! tapers onto its end vertex and a hull end is clipped by the part's planes: no round runs past
//! any of them, so no ridge is read past them. A ridge all but parallel to a leg has no nearest
//! approach worth reading on that leg.
//!
//! **Exact at the size asked, an estimate below it.** At the size asked the centres are the
//! ball's own, so whether the ridge's line has left the round there is exact, given that the round
//! between two stations runs straight between their centres and that the legs, vertices and
//! carries read are the ones above. The size a root names below or above the size asked is an estimate: the axis there is
//! the same scaled estimate the rest of the ball's room reads (`one_chain_room`), each chain point
//! plus the size times its centre's offset per unit of the size asked, and a contact's distance,
//! so a centre, is not linear in the size on a curved side.
//!
//! **One centre for both sides.** The angle formula has to pick a side to place its centre from,
//! and reading the larger side's setback for every run is what made it conservative. The ball's
//! centre is one point standing a radius from both sides, so a run on either side is read against
//! the same centre: reading a run against the side it lies on IS reading it against the ball.

use vector_math::{ieee_math, Vector3};

use super::virtual_rail_past_the_end::PAST_THE_LAST_TOUCH;
use super::GrowingCreaseRuns;

/// The rolling ball's centres along a chain, per unit of the size it was walked at.
#[derive(Clone, Copy, Debug)]
pub struct BallPath<'a> {
    /// Each chain vertex, in chain order.
    pub chain_points: &'a [Vector3],
    /// The ball's centre less its chain vertex at each chain vertex, per unit of size
    /// (`SideSetbacks::centres`).
    pub unit_centres: &'a [Vector3],
    /// Whether the chain closes on itself (a segment from its last vertex back to its first).
    pub closed: bool,
    /// Whether the fill carries the round on past the chain's (first, last) end: only an open
    /// chain's end that carries an end ray (`PassSetbacks::end_rays`), whose rows the fill runs on
    /// virtually past it (`virtual_rail_past_the_end`) for the growing neighbour's planes to clip.
    pub carried: [bool; 2],
    /// The size the ball was walked at: the size the build asks.
    pub size: f64,
}

impl BallPath<'_> {
    /// The path when it holds one centre per chain vertex and at least one leg, `None` otherwise.
    pub fn checked(self) -> Option<Self> {
        let usable = self.chain_points.len() >= 2
            && self.chain_points.len() == self.unit_centres.len()
            && self.size.is_finite()
            && self.size > 0.0;
        usable.then_some(self)
    }

    /// The leg of the chain nearest `point` (the first of a tie) as (its first vertex, the
    /// parameter of `point`'s foot on it, clamped to the leg).
    fn nearest_leg(&self, point: Vector3) -> (usize, f64) {
        let n = self.chain_points.len();
        let legs = if self.closed { n } else { n - 1 };
        let mut best = (0, 0.0);
        let mut best_distance = f64::INFINITY;
        for j in 0..legs {
            let (a, b) = (self.chain_points[j], self.chain_points[(j + 1) % n]);
            let ab = b - a;
            let t = if ab.length_squared() > 0.0 {
                ((point - a).dot(ab) / ab.length_squared()).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let distance = (point - (a + (ab * t))).length();
            if distance < best_distance {
                best_distance = distance;
                best = (j, t);
            }
        }
        best
    }

    /// How many legs the chain has: one per pair of neighbouring vertices, and one more from the
    /// last back to the first on a closed chain.
    fn leg_count(&self) -> usize {
        let n = self.chain_points.len();
        if self.closed {
            n
        } else {
            n - 1
        }
    }

    /// The legs whose span stands within `reach` of `foot` (the ridge's start's foot on its
    /// nearest leg, `nearest`, always among them), in chain order.
    fn legs_within_reach(&self, nearest: usize, foot: Vector3, reach: f64) -> Vec<usize> {
        let n = self.chain_points.len();
        (0..self.leg_count())
            .filter(|&leg| {
                if leg == nearest {
                    return true;
                }
                let (a, b) = (self.chain_points[leg], self.chain_points[(leg + 1) % n]);
                let ab = b - a;
                let t = if ab.length_squared() > 0.0 {
                    ((foot - a).dot(ab) / ab.length_squared()).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                (foot - (a + (ab * t))).length() <= reach
            })
            .collect()
    }

    /// How far from the ridge's start's foot a leg can stand and still be where the ridge's line
    /// comes nearest the round at a size up to the size asked: the ridge's own length, plus the
    /// round's reach from the chain (a centre's offset and the radius, at the largest centre).
    fn reach(&self, start: Vector3, far: Vector3) -> f64 {
        let largest = self
            .unit_centres
            .iter()
            .fold(0.0, |most: f64, centre| most.max(centre.length()));
        (far - start).length() + (self.size * (1.0 + largest))
    }

    /// The largest size up to which no ridge of `ridges` meets the round as built of
    /// `fillet_segments` chords (0 for the true round): the least size, over the legs and inner
    /// vertices the header names, at which a ridge's line leaves the round about the polyline of
    /// centres by more than its radius where it comes nearest that leg or vertex, as
    /// `GrowingCreaseRuns::fillet_pierce_room` reads it with the angle's centre; infinity where
    /// none does.
    pub fn pierce_room(&self, ridges: &[(Vector3, Vector3)], fillet_segments: i32) -> f64 {
        let inscribed = if fillet_segments > 0 {
            (std::f64::consts::PI / (2.0 * f64::from(fillet_segments))).cos()
        } else {
            1.0
        };
        let n = self.chain_points.len();
        let mut room = f64::INFINITY;
        for &(start, far) in ridges {
            let (nearest, t) = self.nearest_leg(start);
            let (a, b) = (
                self.chain_points[nearest],
                self.chain_points[(nearest + 1) % n],
            );
            let foot = a + ((b - a) * t);
            for leg in self.legs_within_reach(nearest, foot, self.reach(start, far)) {
                let next = (leg + 1) % n;
                let leg_length = (self.chain_points[next] - self.chain_points[leg]).length();
                let floor = GrowingCreaseRuns::ROOT_FLOOR * leg_length;
                let exit = if self.unit_centres[leg] == self.unit_centres[next] {
                    self.parallel_exit(leg, start, far, inscribed, floor)
                } else {
                    self.tilted_exit(leg, start, far, inscribed, floor)
                };
                room = ieee_math::min_f64(room, exit);
            }
            let reach = self.reach(start, far);
            let inner = if self.closed { 0..n } else { 1..(n - 1) };
            for v in inner {
                if (self.chain_points[v] - foot).length() <= reach {
                    room = ieee_math::min_f64(room, self.vertex_exit(v, start, far, inscribed));
                }
            }
        }
        room
    }

    /// The least exit root on leg `j` where its two centres agree: the axis runs along the leg,
    /// the ridge's distance from it is linear in the size, and each root is one division.
    fn parallel_exit(
        &self,
        j: usize,
        start: Vector3,
        far: Vector3,
        inscribed: f64,
        floor: f64,
    ) -> f64 {
        let k = (j + 1) % self.chain_points.len();
        let a = self.chain_points[j];
        let along = (self.chain_points[k] - a).get_normal();
        let ridge = (start - far).get_normal();
        let common = ridge.cross(along);
        if common.length() < 1e-12 {
            return f64::INFINITY;
        }
        let common = common.get_normal();
        let c0 = (a - start).dot(common);
        let c1 = self.unit_centres[j].dot(common);
        let mut least = f64::INFINITY;
        for sign in [1.0, -1.0] {
            if (sign * c1) - inscribed <= GrowingCreaseRuns::GRAZING_MARGIN {
                continue;
            }
            // A root at the scale of rounding is a ridge that starts ON the chain line.
            let s = c0 / ((sign * inscribed) - c1);
            if s > floor && self.meets_the_leg(j, start, ridge, s) {
                least = ieee_math::min_f64(least, s);
            }
        }
        least
    }

    /// The least exit root on leg `j` where its two centres differ: the axis at size s runs
    /// through a + s c_a and b + s c_b, so it tilts with the size. With the ridge's unit direction
    /// r, the axis point's offset from the ridge's start p0 + s p1 and the axis direction
    /// d0 + s d1, the signed distance is h(s) = N(s) / sqrt(M(s)), N = (p0 + s p1) . (n0 + s n1)
    /// and M = |n0 + s n1|^2 with n = r x d. Every size where |h| = k s is a root of the quartic
    /// N^2 - k^2 s^2 M; each is found by bisection between the roots of its derivatives
    /// ([`real_roots`]), and the least whose |h| grows past k s there, by the same margin the
    /// linear reading asks (sign h' - k above `GRAZING_MARGIN`), and where the ridge's line comes
    /// nearest the leg ([`Self::meets_the_leg`]), is the exit.
    fn tilted_exit(
        &self,
        j: usize,
        start: Vector3,
        far: Vector3,
        inscribed: f64,
        floor: f64,
    ) -> f64 {
        let k = (j + 1) % self.chain_points.len();
        let ridge = (start - far).get_normal();
        let (a, b) = (self.chain_points[j], self.chain_points[k]);
        let (p0, p1) = (a - start, self.unit_centres[j]);
        let (n0, n1) = (
            ridge.cross(b - a),
            ridge.cross(self.unit_centres[k] - self.unit_centres[j]),
        );
        let numerator = [p0.dot(n0), p0.dot(n1) + p1.dot(n0), p1.dot(n1)];
        let norm = [n0.dot(n0), 2.0 * n0.dot(n1), n1.dot(n1)];
        let k2 = inscribed * inscribed;
        let quartic = [
            numerator[0] * numerator[0],
            2.0 * numerator[0] * numerator[1],
            (numerator[1] * numerator[1]) + (2.0 * numerator[0] * numerator[2]) - (k2 * norm[0]),
            (2.0 * numerator[1] * numerator[2]) - (k2 * norm[1]),
            (numerator[2] * numerator[2]) - (k2 * norm[2]),
        ];
        let Some(bound) = cauchy_bound(&quartic) else {
            return f64::INFINITY;
        };
        for s in real_roots(&quartic, floor, ieee_math::max_f64(bound, floor)) {
            let m = evaluate(&norm, s);
            if m <= 0.0 {
                continue;
            }
            let root_m = m.sqrt();
            let n_at = evaluate(&numerator, s);
            let h = n_at / root_m;
            let n_slope = numerator[1] + (2.0 * numerator[2] * s);
            let m_slope = norm[1] + (2.0 * norm[2] * s);
            let h_slope = (n_slope / root_m) - ((n_at * m_slope) / (2.0 * m * root_m));
            let sign = if h < 0.0 { -1.0 } else { 1.0 };
            if (sign * h_slope) - inscribed > GrowingCreaseRuns::GRAZING_MARGIN
                && self.meets_the_leg(j, start, ridge, s)
            {
                return s;
            }
        }
        f64::INFINITY
    }

    /// Where the ridge's line, through `start` along `ridge`, comes nearest leg `j`'s axis at size
    /// `s`: the axis's parameter from the centre at `j` (0) to the centre at the next vertex (1),
    /// with the axis's length; `None` for lines all but parallel, which have no nearest approach
    /// worth reading.
    fn nearest_along(
        &self,
        j: usize,
        start: Vector3,
        ridge: Vector3,
        s: f64,
    ) -> Option<(f64, f64)> {
        let k = (j + 1) % self.chain_points.len();
        let from = self.chain_points[j] + (self.unit_centres[j] * s);
        let to = self.chain_points[k] + (self.unit_centres[k] * s);
        let axis = to - from;
        let offset = start - from;
        let (a, b, c) = (ridge.dot(ridge), ridge.dot(axis), axis.dot(axis));
        let (d, e) = (ridge.dot(offset), axis.dot(offset));
        let denominator = (a * c) - (b * b);
        // Lines all but parallel (or a NaN) have no nearest approach worth reading.
        if denominator.is_nan() || denominator <= 1e-24 * a * c {
            return None;
        }
        Some((((a * e) - (b * d)) / denominator, c.sqrt()))
    }

    /// Whether the ridge's line comes nearest leg `j`'s axis at size `s` where the fill builds that
    /// leg's round: within the leg (parameter 0 to 1, [`Self::nearest_along`]), and at an open
    /// chain's end past it only where the end is carried ([`BallPath::carried`]), by up to
    /// s |c| (1 + `PAST_THE_LAST_TOUCH`), with c that end's centre per unit (the header says why).
    /// A nearest approach past an inner vertex is that vertex's reading ([`Self::vertex_exit`]).
    pub fn meets_the_leg(&self, j: usize, start: Vector3, ridge: Vector3, s: f64) -> bool {
        let Some((along, axis_length)) = self.nearest_along(j, start, ridge, s) else {
            return false;
        };
        let n = self.chain_points.len();
        let legs = self.leg_count();
        // How far past a carried end the rows run, as a share of the axis; nothing past any other.
        let carry = |end: usize, vertex: usize| {
            if !self.closed && self.carried[end] && axis_length > 0.0 {
                s * self.unit_centres[vertex].length() * (1.0 + PAST_THE_LAST_TOUCH) / axis_length
            } else {
                0.0
            }
        };
        let before = if j == 0 { carry(0, 0) } else { 0.0 };
        let after = if j + 1 == legs { carry(1, n - 1) } else { 0.0 };
        along >= -before && along <= 1.0 + after
    }

    /// The least exit root at inner vertex `v` (every vertex of a closed chain): the ridge's line
    /// against the vertex's centre c_v(s) = v + s c_v, whose distance |(c_v(s) - start) x r| is
    /// |A + s B| with A = (v - start) x r and B = c_v x r, so |A + s B| = k s is the quadratic
    /// (B.B - k^2) s^2 + 2 (A.B) s + A.A = 0. A root counts where the distance grows past k s
    /// (its slope above k by `GRAZING_MARGIN`, as the legs' readings ask) and where the line
    /// comes nearest both legs at that vertex past it (the leg in at or past its end, the leg out
    /// at or before its start): there the vertex is the centres' nearest point to the line. An
    /// all but parallel leg's every point is as near as the vertex, so it does not forbid it.
    fn vertex_exit(&self, v: usize, start: Vector3, far: Vector3, inscribed: f64) -> f64 {
        let n = self.chain_points.len();
        let legs = self.leg_count();
        let (into, out) = (if v == 0 { legs - 1 } else { v - 1 }, v % legs);
        let shorter = ieee_math::min_f64(
            (self.chain_points[v] - self.chain_points[into]).length(),
            (self.chain_points[(v + 1) % n] - self.chain_points[v]).length(),
        );
        let floor = GrowingCreaseRuns::ROOT_FLOOR * shorter;
        let ridge = (start - far).get_normal();
        let a = (self.chain_points[v] - start).cross(ridge);
        let b = self.unit_centres[v].cross(ridge);
        let k2 = inscribed * inscribed;
        let (q2, q1, q0) = (b.dot(b) - k2, 2.0 * a.dot(b), a.dot(a));
        let mut roots: Vec<f64> = if q2 == 0.0 {
            if q1 == 0.0 {
                Vec::new()
            } else {
                vec![-q0 / q1]
            }
        } else {
            let discriminant = (q1 * q1) - (4.0 * q2 * q0);
            if discriminant < 0.0 {
                Vec::new()
            } else {
                let root = discriminant.sqrt();
                vec![(-q1 - root) / (2.0 * q2), (-q1 + root) / (2.0 * q2)]
            }
        };
        roots.sort_by(f64::total_cmp);
        for s in roots {
            if s <= floor || !s.is_finite() {
                continue;
            }
            let distance = (a + (b * s)).length();
            if distance <= 0.0 {
                continue;
            }
            let slope = (a.dot(b) + (s * b.dot(b))) / distance;
            if slope - inscribed <= GrowingCreaseRuns::GRAZING_MARGIN {
                continue;
            }
            let past_in = self
                .nearest_along(into, start, ridge, s)
                .is_none_or(|(along, _)| along >= 1.0);
            let past_out = self
                .nearest_along(out, start, ridge, s)
                .is_none_or(|(along, _)| along <= 0.0);
            if past_in && past_out {
                return s;
            }
        }
        f64::INFINITY
    }
}

/// The polynomial with coefficients `c` (lowest power first) at `x`, by Horner's rule.
fn evaluate(c: &[f64], x: f64) -> f64 {
    c.iter()
        .rev()
        .fold(0.0, |sum, &coefficient| (sum * x) + coefficient)
}

/// Every real root's magnitude is below this (Cauchy's bound), or `None` for a polynomial with no
/// power of the size in it.
fn cauchy_bound(c: &[f64]) -> Option<f64> {
    let top = c.iter().rposition(|&coefficient| coefficient != 0.0)?;
    if top == 0 {
        return None;
    }
    let lead = c[top].abs();
    let largest = c[..top].iter().fold(0.0, |most: f64, &coefficient| {
        most.max(coefficient.abs() / lead)
    });
    Some(1.0 + largest)
}

/// The real roots of the polynomial `c` (lowest power first) in (`lo`, `hi`], ascending: its
/// derivative's roots split the range into stretches where it is monotone, and each stretch whose
/// ends differ in sign holds one root, found by halving until the halves stop shrinking.
fn real_roots(c: &[f64], lo: f64, hi: f64) -> Vec<f64> {
    if c.len() < 2 {
        return Vec::new();
    }
    if c.len() == 2 {
        if c[1] == 0.0 {
            return Vec::new();
        }
        let x = -c[0] / c[1];
        return if x > lo && x <= hi {
            vec![x]
        } else {
            Vec::new()
        };
    }
    let derivative: Vec<f64> = c[1..]
        .iter()
        .enumerate()
        .map(|(i, &coefficient)| coefficient * (i + 1) as f64)
        .collect();
    let mut ends = vec![lo];
    ends.extend(real_roots(&derivative, lo, hi));
    ends.push(hi);
    let mut roots: Vec<f64> = Vec::new();
    for pair in ends.windows(2) {
        let (mut x0, mut x1) = (pair[0], pair[1]);
        let (v0, v1) = (evaluate(c, x0), evaluate(c, x1));
        if v1 == 0.0 {
            if x1 > lo && roots.last() != Some(&x1) {
                roots.push(x1);
            }
            continue;
        }
        if v0 == 0.0 || (v0 < 0.0) == (v1 < 0.0) {
            continue;
        }
        let rising = v1 > 0.0;
        loop {
            let middle = 0.5 * (x0 + x1);
            if middle <= x0 || middle >= x1 {
                break;
            }
            if (evaluate(c, middle) > 0.0) == rising {
                x1 = middle;
            } else {
                x0 = middle;
            }
        }
        if roots.last() != Some(&x1) {
            roots.push(x1);
        }
    }
    roots
}
