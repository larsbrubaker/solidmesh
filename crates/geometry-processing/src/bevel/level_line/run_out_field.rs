//! The field of one `CreaseRunOut` ([`RunOutField`]). Ports the type of
//! `Bevel/LevelLine/RunOutField.cs`, with its constructors (a bare cone, the field of a run-out,
//! the wedge under a contact profile), its reading, the golden-section least and its field
//! queries ([`ScalarField`], [`ChainDistanceField`]).

use std::cmp::Ordering;

use vector_math::{cs_math, Vector3};

use super::{
    ChainDistanceField, CreaseRunOut, FieldMaximum, FieldMinimum, LevelLineWork, ScalarField,
    WorkError,
};

/// The steepest taper taken - a guard, not a tuning: it keeps a degenerate reading (a contact
/// sample almost at P*) from turning the sliver into a fan.
pub const MAX_SLOPE: f64 = 1.0;

/// Golden-section steps: each keeps 0.618 of the span, so 80 of them reach the precision of the
/// ends.
pub const SEARCH_STEPS: i32 = 80;

/// The golden-section ratio, (sqrt 5 - 1) / 2 (C# `static readonly Golden`).
pub fn golden() -> f64 {
    (5f64.sqrt() - 1.0) / 2.0
}

/// C# `RunOutField`: the field of one `CreaseRunOut` - negative in a thin tapering wedge along the
/// unpicked crease, from the pick corner to P*, zero width at P* - the sliver the round's void ball
/// takes off both crease faces as it runs out. Read as the least of it and the pick's own field
/// (`BandGroupFields`), so the band cut carries the round on past the crease the side flood stopped
/// at.
///
/// **The shape.** The wedge is the solid of revolution about the crease's line under the wall
/// contact T(theta)'s profile: at distance s along the crease it reaches as far off the line as the
/// contact does (`Of` takes the upper concave hull of the sampled contact and P*). The hull's last
/// edge, falling to P*, is a CONE with its apex there ([`RunOutField::slope`], exact past the
/// apex); every earlier edge is one more line `off <= A + B s`; [`RunOutField::width`] caps the
/// peak; and the plane through the corner square to the crease closes it.
///
/// **What the seam needs.** Every part is convex and 1-Lipschitz (a hull line `off - A - B s` has
/// gradient sqrt(1 + B^2), since off and s change along orthogonal directions, so dividing by
/// exactly that makes it 1-Lipschitz - the normalisation is load-bearing), so the max is
/// 1-Lipschitz and CONVEX. Convex makes every query simple: the greatest over a segment or triangle
/// stands at a corner, and the least is found by golden-section search.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct RunOutField {
    /// The pick corner the wedge starts at.
    pub corner: Vector3,
    /// The unit direction along the crease, away from the corner.
    pub direction: Vector3,
    /// How far along the crease the wedge ends, P* (C# `Reach`).
    pub reach: f64,
    /// `sqrt(1 + slope^2)`: what the cone's reading is divided by to be 1-Lipschitz.
    pub norm: f64,
    /// The centre of the ball `NothingReachesBelow` tests against, halfway to P*; its radius
    /// depends on the margin.
    pub ball_centre: Vector3,
    /// The concave profile's rising and shoulder lines, each `off <= a + b s` with its own
    /// normalisation `sqrt(1 + b^2)`, before the cone's own falling generator; empty for a bare
    /// cone.
    pub lines: Vec<(f64, f64, f64)>,
    /// The taper: the wedge's radius per unit still to go to P*.
    pub slope: f64,
    /// The widest the wedge stands off the crease: the widest the wall contact does. Infinite for a
    /// bare cone.
    pub width: f64,
}

impl RunOutField {
    /// C#'s constructor: a wedge from `corner` along the unit `direction` to P* at `reach`, with
    /// the concave profile's lines `off <= a + b s` (each kept with its normalisation sqrt(1 + b^2)).
    pub fn new(
        corner: Vector3,
        direction: Vector3,
        reach: f64,
        slope: f64,
        width: f64,
        lines: &[(f64, f64)],
    ) -> Self {
        Self {
            lines: lines
                .iter()
                .map(|&(a, b)| (a, b, (1.0 + (b * b)).sqrt()))
                .collect(),
            corner,
            direction,
            reach,
            slope,
            width,
            norm: (1.0 + (slope * slope)).sqrt(),
            // The centre of the ball the nothing-reaches test uses; its radius depends on the margin.
            ball_centre: corner + (direction * (reach / 2.0)),
        }
    }

    /// A bare wedge from `corner` along the unit `direction` to P* at `reach` (C#'s optional
    /// `width` is +infinity there).
    pub fn cone(corner: Vector3, direction: Vector3, reach: f64, slope: f64, width: f64) -> Self {
        Self::new(corner, direction, reach, slope, width, &[])
    }

    /// The run-out's field, its taper read from the wall contact. The taper is steepest just short
    /// of P* (0.647 on the Top Center Face nose), so the bare cone stood 2.97 wide at the corner
    /// where the contact is never wider than 0.66; the widest contact caps the radius.
    pub fn of(run_out: &CreaseRunOut) -> Self {
        let contact: Vec<(f64, f64)> = run_out
            .wall_contact
            .iter()
            .map(|&contact| {
                (
                    (contact - run_out.corner_point).dot(run_out.crease_direction),
                    run_out.distance_to_crease_line(contact),
                )
            })
            .collect();
        Self::from_profile(
            run_out.corner_point,
            run_out.crease_direction,
            run_out.reach,
            &contact,
        )
    }

    /// The wedge under a contact profile given as (s along the crease, off it) samples, dying at
    /// P* = `reach`.
    pub fn from_profile(
        corner: Vector3,
        direction: Vector3,
        reach: f64,
        contact: &[(f64, f64)],
    ) -> Self {
        // The contact's (s, off) samples and P* itself, where it dies on the crease. Samples closer
        // in s than a hair of the reach are one sample, the widest of them - a hull edge between
        // them would stand vertical - and none may crowd P*, which keeps its zero width.
        let hair = 1e-9 * reach;
        let mut kept: Vec<(f64, f64)> = contact
            .iter()
            .copied()
            .filter(|p| p.0 < reach - hair)
            .collect();
        // C#'s OrderBy: a stable sort (the filter has let no NaN s through).
        kept.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(Ordering::Equal));
        let mut profile: Vec<(f64, f64)> = Vec::new();
        for sample in kept {
            match profile.last_mut() {
                Some(last) if sample.0 - last.0 <= hair => {
                    *last = (last.0, cs_math::max_f64(last.1, sample.1));
                }
                _ => profile.push(sample),
            }
        }

        if profile.is_empty() {
            // No contact short of P*: nothing to fit, so the thinnest wedge - the crease itself up
            // to P*.
            return Self::cone(corner, direction, reach, 0.0, 0.0);
        }

        profile.push((reach, 0.0));
        let hull = Self::upper_hull(&profile);
        // C#'s `Enumerable.Max`: a NaN is displaced by any number.
        let width = hull.iter().map(|p| p.1).fold(f64::NAN, |best, x| {
            if x > best || best.is_nan() {
                x
            } else {
                best
            }
        });

        // The last hull edge falls to P* and becomes the cone's generator (exact past the apex);
        // every other edge is one more line the wedge stays under. Their least is the hull, concave
        // in s, so the field stays convex.
        let last = hull.len() - 2;
        let slope = (hull[last].1 - hull[last + 1].1) / (hull[last + 1].0 - hull[last].0);
        let mut lines = Vec::new();
        for i in 0..last {
            let b = (hull[i + 1].1 - hull[i].1) / (hull[i + 1].0 - hull[i].0);
            lines.push((hull[i].1 - (b * hull[i].0), b));
        }

        Self::new(
            corner,
            direction,
            reach,
            cs_math::min_f64(slope, MAX_SLOPE),
            width,
            &lines,
        )
    }

    /// The least concave function over the samples: the upper hull, left to right. Concave is what
    /// keeps the wedge a convex solid.
    pub fn upper_hull(points: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let mut hull: Vec<(f64, f64)> = Vec::new();
        for &p in points {
            while hull.len() >= 2 {
                let (b, a) = (hull[hull.len() - 1], hull[hull.len() - 2]);
                if ((b.0 - a.0) * (p.1 - a.1)) - ((b.1 - a.1) * (p.0 - a.0)) >= 0.0 {
                    hull.pop();
                } else {
                    break;
                }
            }

            hull.push(p);
        }

        hull
    }

    /// The field's value at `point`: the greatest of the cone (relative to its apex at P*), the
    /// width cylinder, the closing plane through the corner and each hull line.
    pub fn read(&self, point: Vector3) -> f64 {
        let from = point - self.corner;
        let along = from.dot(self.direction);
        let off = (from - (self.direction * along)).length();

        // In the half-plane through the axis, relative to the apex at P*: x along (negative before
        // P*), y off it. Past the generator's foot the nearest point of the cone is the apex itself.
        let x = along - self.reach;
        let cone = if (-x) + (self.slope * off) >= 0.0 {
            ((self.slope * x) + off) / self.norm
        } else {
            ((x * x) + (off * off)).sqrt()
        };
        let mut value = cs_math::max_f64(cs_math::max_f64(cone, off - self.width), -along);
        for &(a, b, line_norm) in &self.lines {
            value = cs_math::max_f64(value, (off - a - (b * along)) / line_norm);
        }

        value
    }

    /// Where a convex function of one parameter is least over [lo, hi], by golden-section search
    /// with both ends read too.
    pub fn least_of(f: &dyn Fn(f64) -> f64, lo: f64, hi: f64) -> f64 {
        if hi.is_nan() || lo.is_nan() || hi <= lo {
            return lo;
        }

        let golden = golden();
        let mut from = lo;
        let mut to = hi;
        let mut x1 = to - (golden * (to - from));
        let mut x2 = from + (golden * (to - from));
        let mut f1 = f(x1);
        let mut f2 = f(x2);
        for _ in 0..SEARCH_STEPS {
            if f1 <= f2 {
                to = x2;
                x2 = x1;
                f2 = f1;
                x1 = to - (golden * (to - from));
                f1 = f(x1);
            } else {
                from = x1;
                x1 = x2;
                f1 = f2;
                x2 = from + (golden * (to - from));
                f2 = f(x2);
            }
        }

        // The bracket never quite reaches the ends, and a piece wholly to one side of the wedge is
        // least at an end - the usual case - so the ends are read too and the least of the three
        // is taken.
        let mut best = (from + to) / 2.0;
        let mut best_value = f(best);
        for end in [lo, hi] {
            let value = f(end);
            if value < best_value {
                best = end;
                best_value = value;
            }
        }

        best
    }
}

impl RunOutField {
    /// The greater of the field's values at the two ends - convex, so the greatest over a segment
    /// or triangle stands at a corner.
    pub fn greatest(&self, a: Vector3, b: Vector3) -> Result<FieldMaximum, WorkError> {
        let va = self.value_at(a)?;
        let vb = self.value_at(b)?;
        Ok(if va >= vb {
            FieldMaximum::new(va, a)
        } else {
            FieldMaximum::new(vb, b)
        })
    }
}

impl ScalarField for RunOutField {
    fn value_at(&self, point: Vector3) -> Result<f64, WorkError> {
        LevelLineWork::charge(1)?;
        Ok(self.read(point))
    }
}

impl ChainDistanceField for RunOutField {
    fn min_along_segment(&self, a: Vector3, b: Vector3) -> Result<FieldMinimum, WorkError> {
        LevelLineWork::charge(1)?;
        let t = Self::least_of(&|u| self.read(a + ((b - a) * u)), 0.0, 1.0);
        let at = a + ((b - a) * t);
        Ok(FieldMinimum::new(self.read(at), at))
    }

    fn min_over_triangle(
        &self,
        a: Vector3,
        b: Vector3,
        c: Vector3,
    ) -> Result<FieldMinimum, WorkError> {
        LevelLineWork::charge(1)?;
        let at = |u: f64, v: f64| a + ((b - a) * u) + ((c - a) * v);
        let least_across = |u: f64| {
            self.read(at(
                u,
                Self::least_of(&|v| self.read(at(u, v)), 0.0, 1.0 - u),
            ))
        };
        let best_u = Self::least_of(&least_across, 0.0, 1.0);
        let best_v = Self::least_of(&|v| self.read(at(best_u, v)), 0.0, 1.0 - best_u);
        let point = at(best_u, best_v);
        Ok(FieldMinimum::new(self.read(point), point))
    }

    /// True when the box misses a ball holding every point the field reads at or below `margin`.
    /// The ball must hold the SUBLEVEL set `{f <= m}`, not the solid: with `m = max(margin, 0)`,
    /// over `s` in `[-m, reach + m]` the sublevel set lies in a cylinder of half-length
    /// `reach / 2 + m` about the centre and radius `slope (reach + m) + m norm`; the ball through
    /// its rims holds it, and the hull lines and the width cap only shrink it further.
    fn nothing_reaches_below(&self, min: Vector3, max: Vector3, margin: f64) -> bool {
        let m = cs_math::max_f64(margin, 0.0);
        let half_length = (self.reach / 2.0) + m;
        let rim = (self.slope * (self.reach + m)) + (m * self.norm);
        let radius = ((half_length * half_length) + (rim * rim)).sqrt();
        let nearest = Vector3::component_max(min, Vector3::component_min(max, self.ball_centre));
        (nearest - self.ball_centre).length() > radius
    }

    fn max_along_segment(
        &self,
        a: Vector3,
        b: Vector3,
        _tolerance: f64,
    ) -> Result<FieldMaximum, WorkError> {
        self.greatest(a, b)
    }

    fn max_along_segment_at_least(
        &self,
        a: Vector3,
        b: Vector3,
        _tolerance: f64,
        at_least: f64,
    ) -> Result<FieldMaximum, WorkError> {
        let greatest = self.greatest(a, b)?;
        Ok(if greatest.value > at_least {
            greatest
        } else {
            FieldMaximum::new(at_least, a)
        })
    }

    fn max_over_triangle(
        &self,
        a: Vector3,
        b: Vector3,
        c: Vector3,
        _tolerance: f64,
    ) -> Result<FieldMaximum, WorkError> {
        let first = self.greatest(a, b)?;
        self.greatest(first.point, c)
    }

    fn max_over_triangle_bounded(
        &self,
        a: Vector3,
        b: Vector3,
        c: Vector3,
        tolerance: f64,
        _stop_above: f64,
        at_least: f64,
    ) -> Result<FieldMaximum, WorkError> {
        let greatest = self.max_over_triangle(a, b, c, tolerance)?;
        Ok(if greatest.value > at_least {
            greatest
        } else {
            FieldMaximum::new(at_least, a)
        })
    }
}

impl RunOutField {
    /// How far along the crease the wedge ends, P* (C#'s `Reach`).
    pub fn reach(&self) -> f64 {
        self.reach
    }
}
