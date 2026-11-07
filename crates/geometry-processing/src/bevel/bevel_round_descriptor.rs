//! A CYLINDER in the part: where its axis runs, how big it is, and which part of its surface is
//! really there. It is the analytic surface behind a tessellated round, so a pass can roll its ball
//! on the round itself rather than on the plane of whichever facet an edge of the crease happens to
//! lie in.
//!
//! **Two kinds of round, one descriptor.** A round the FIRST PASS CUT is described from its own
//! cutter and read by the second pass across the crease it re-reads. A round the part ARRIVED with
//! (a modelled bore or boss) is FITTED off the source mesh (`BevelSourceRounds`) and read by the
//! first pass. The difference shows only in the explicit-span constructor
//! ([`BevelRoundDescriptor::with_span`]), the axial extent, and
//! `BevelRoundFaces.ReadFromTheSourceMesh`.
//!
//! **Where a CUT round's numbers come from.** A convex edge's cutter inscribes an arc of the size
//! in the corner, tangent to both faces: the centre is `from - size * (n0 + n1) / (1 + n0 . n1)`,
//! and the surface is tangent to face i at `axis_point + size * n_i`, so n0 and n1 are also the two
//! ends of its angular SPAN.
//!
//! **The spine.** A ball of radius r rolling in the concave crease between this round and a planar
//! slope keeps its centre r from the slope and r + R from the axis, so the spine is the
//! intersection of an offset plane with a radius R + r cylinder - an ellipse arc, which is what
//! [`BevelRoundDescriptor::try_find_station`] walks. The spine's tangent is square to the radial at
//! the contact by construction, so a section drawn square to the tangent is an ORDINARY two-plane
//! bevel section.

use std::f64::consts::{PI, TAU};

use vector_math::{ieee_math, math_helper, Vector3};

/// How far past either end of its span a point's angle may be and still count, in degrees.
const SPAN_SLACK_DEGREES: f64 = 5.0;

/// How nearly a slope has to run along the axis before the scan's answer is replaced by the
/// closed form: the scan cannot bracket a spine it has slid off to infinity.
const NEARLY_ALONG_THE_AXIS: f64 = 1e-3;

/// Where a ball rolling between a round and a slope stands for one section
/// ([`BevelRoundDescriptor::try_find_station`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RoundStation {
    /// The angle about the axis, in the round's own basis.
    pub phi: f64,
    /// The ball's centre.
    pub centre: Vector3,
    /// The spine's unit tangent there, which is the section plane's normal.
    pub tangent: Vector3,
    /// Where the ball touches the round.
    pub contact: Vector3,
}

/// One round of a part: its axis, radius, chord count, and the span it covers along and about the
/// axis.
#[derive(Debug, Clone, PartialEq)]
pub struct BevelRoundDescriptor {
    /// A point of the axis.
    pub axis_point: Vector3,
    /// The axis direction (unit).
    pub axis_direction: Vector3,
    /// The round's radius.
    pub radius: f64,
    /// How many chords the round is drawn with, at least one.
    pub segments: i32,
    /// Where the round starts along the axis, from `axis_point`.
    pub axial_from: f64,
    /// Where it ends.
    pub axial_to: f64,
    first: Vector3,
    second: Vector3,
    span_low: f64,
    span_high: f64,
}

impl BevelRoundDescriptor {
    /// A CUT round: the span runs between the two radials (the two faces' outward normals). The
    /// angular basis is the FIRST face's radial, so one end of the span is always zero and the
    /// other is the signed turn to the second - which cannot wrap, because a convex edge's two
    /// outward normals are less than half a turn apart.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        axis_point: Vector3,
        axis_direction: Vector3,
        radius: f64,
        first_radial: Vector3,
        second_radial: Vector3,
        axial_from: f64,
        axial_to: f64,
        segments: i32,
    ) -> Self {
        let second = axis_direction.cross(first_radial).get_normal();
        let to = second_radial
            .dot(second)
            .atan2(second_radial.dot(first_radial));

        Self {
            axis_point,
            axis_direction,
            radius,
            segments: segments.max(1),
            axial_from,
            axial_to,
            first: first_radial,
            second,
            span_low: ieee_math::min_f64(0.0, to),
            span_high: ieee_math::max_f64(0.0, to),
        }
    }

    /// A round with an explicit span about `first_radial` - what a FITTED round needs, because its
    /// span may go the whole way round.
    #[allow(clippy::too_many_arguments)]
    pub fn with_span(
        axis_point: Vector3,
        axis_direction: Vector3,
        radius: f64,
        first_radial: Vector3,
        span_low: f64,
        span_high: f64,
        axial_from: f64,
        axial_to: f64,
        segments: i32,
    ) -> Self {
        Self {
            axis_point,
            axis_direction,
            radius,
            segments: segments.max(1),
            axial_from,
            axial_to,
            first: first_radial,
            second: axis_direction.cross(first_radial).get_normal(),
            span_low,
            span_high,
        }
    }

    fn goes_more_than_half_way_round(&self) -> bool {
        self.span_high - self.span_low > PI
    }

    /// The unit radial at an angle of the round's own basis.
    pub fn radial(&self, phi: f64) -> Vector3 {
        (self.first * phi.cos()) + (self.second * phi.sin())
    }

    /// Whether another descriptor is this same round (same radius, axis and span ends) to within
    /// `tolerance`.
    pub fn is_the_same_round_as(&self, other: &BevelRoundDescriptor, tolerance: f64) -> bool {
        if (self.radius - other.radius).abs() > tolerance
            || self.axis_direction.dot(other.axis_direction) < 1.0 - 1e-9
        {
            return false;
        }

        // A span that goes more than half way round has no two ends to match: both of them are
        // the same radial. Two source rounds of one cylinder each already carry the whole
        // surface's axial extent, so there is nothing a merge would add.
        if self.goes_more_than_half_way_round() || other.goes_more_than_half_way_round() {
            return false;
        }

        let between = other.axis_point - self.axis_point;

        if (between - (self.axis_direction * between.dot(self.axis_direction))).length() > tolerance
        {
            return false;
        }

        let low = self.radial(self.span_low);
        let high = self.radial(self.span_high);
        let other_low = other.radial(other.span_low);
        let other_high = other.radial(other.span_high);

        (low.dot(other_low) > 1.0 - 1e-9 && high.dot(other_high) > 1.0 - 1e-9)
            || (low.dot(other_high) > 1.0 - 1e-9 && high.dot(other_low) > 1.0 - 1e-9)
    }

    /// The same round with its axial extent widened to cover another's. Rebuilt through the
    /// EXPLICIT-span constructor, carrying this round's own span across whole.
    pub fn merged(&self, other: &BevelRoundDescriptor) -> BevelRoundDescriptor {
        let shift = (other.axis_point - self.axis_point).dot(self.axis_direction);

        BevelRoundDescriptor::with_span(
            self.axis_point,
            self.axis_direction,
            self.radius,
            self.first,
            self.span_low,
            self.span_high,
            ieee_math::min_f64(self.axial_from, other.axial_from + shift),
            ieee_math::max_f64(self.axial_to, other.axial_to + shift),
            self.segments,
        )
    }

    /// Whether a point of the cut part lies on this round's surface. The radius band is one sided
    /// on purpose: the round is TESSELLATED, so a chord vertex is at the radius but anything on a
    /// facet's interior is inside it, by as much as the sagitta.
    pub fn contains(&self, point: Vector3, tolerance: f64, axial_tolerance: f64) -> bool {
        let relative = point - self.axis_point;
        let along = relative.dot(self.axis_direction);

        if along < self.axial_from - axial_tolerance || along > self.axial_to + axial_tolerance {
            return false;
        }

        let radial = relative - (self.axis_direction * along);
        let distance = radial.length();

        // Floored at half the radius: the sagitta allowance is only ever meant to let a facet's
        // INTERIOR count, and at two chords or fewer cos(pi / segments) is zero or negative, which
        // would read the axis itself as being on the round. BevelRoundFaces refuses to describe a
        // round drawn with fewer than four chords at all, so the floor is a belt to that brace
        // rather than the thing doing the work.
        let inside = ieee_math::max_f64(
            self.radius / 2.0,
            self.radius * (PI / self.segments as f64).cos(),
        );

        if distance < inside - tolerance || distance > self.radius + tolerance || distance < 1e-9 {
            return false;
        }

        let phi = radial.dot(self.second).atan2(radial.dot(self.first));
        let slack = math_helper::degrees_to_radians(SPAN_SLACK_DEGREES);

        phi >= self.span_low - slack && phi <= self.span_high + slack
    }

    /// Whether a contact point has rolled off the plane of the crease's own facet by more than
    /// the chording could explain (twice the sagitta).
    pub fn contact_has_rolled_off_the_creases_facet(
        &self,
        contact: Vector3,
        crease_point: Vector3,
        facet_normal: Vector3,
    ) -> bool {
        let sagitta = self.radius * (1.0 - (PI / self.segments as f64).cos());

        (contact - crease_point).dot(facet_normal).abs() > 2.0 * sagitta
    }

    /// Whether a triangle (`a`, `b`, `c` with unit `normal`) lies on the plane tangent to this
    /// round along either end of its span.
    #[allow(clippy::too_many_arguments)]
    pub fn is_tangent_plane(
        &self,
        normal: Vector3,
        a: Vector3,
        b: Vector3,
        c: Vector3,
        normal_dot: f64,
        tolerance: f64,
        axial_tolerance: f64,
    ) -> bool {
        self.touches_the_tangent_line(
            self.span_low,
            normal,
            [a, b, c],
            normal_dot,
            tolerance,
            axial_tolerance,
        ) || self.touches_the_tangent_line(
            self.span_high,
            normal,
            [a, b, c],
            normal_dot,
            tolerance,
            axial_tolerance,
        )
    }

    fn touches_the_tangent_line(
        &self,
        at: f64,
        normal: Vector3,
        corners: [Vector3; 3],
        normal_dot: f64,
        tolerance: f64,
        axial_tolerance: f64,
    ) -> bool {
        let radial = self.radial(at);

        if normal.dot(radial) < normal_dot {
            return false;
        }

        corners
            .iter()
            .any(|&point| self.on_the_tangent_line(point, radial, tolerance, axial_tolerance))
    }

    fn on_the_tangent_line(
        &self,
        point: Vector3,
        radial: Vector3,
        tolerance: f64,
        axial_tolerance: f64,
    ) -> bool {
        let relative = point - self.axis_point;
        let along = relative.dot(self.axis_direction);

        if along < self.axial_from - axial_tolerance || along > self.axial_to + axial_tolerance {
            return false;
        }

        ((relative - (self.axis_direction * along)) - (radial * self.radius)).length() <= tolerance
    }

    /// Where a ball of `ball_radius` rolling between this round and the planar slope through
    /// `slope_point` (outward unit `slope_normal`) stands in the section through `through`: the
    /// station on the spine whose normal plane passes through the anchor. `None` where there is no
    /// such station on the span.
    ///
    /// The scan takes the FIRST sign change from `span_low` (on a closed span, the one nearest the
    /// anchor's own angle) and bisects it; the root is kept only while the anchor is inside the
    /// spine's tightest radius of curvature, where the distance is monotone - otherwise the scan
    /// may have found one of three roots and there is no saying which.
    ///
    /// A NaN distance refuses the station (`None`): only NaN input geometry produces one, and every
    /// reader already handles "no station" as a refusal.
    pub fn try_find_station(
        &self,
        through: Vector3,
        slope_point: Vector3,
        slope_normal: Vector3,
        ball_radius: f64,
    ) -> Option<RoundStation> {
        let along_the_axis = self.axis_direction.dot(slope_normal);

        if along_the_axis.abs() < 1e-6 {
            return self.try_find_station_along_the_axis(
                through,
                slope_point,
                slope_normal,
                ball_radius,
            );
        }

        let spine_radius = self.radius + ball_radius;
        let offset = (self.axis_point - slope_point).dot(slope_normal);

        // Where the ball's centre is at an angle of the span: out on the spine cylinder, slid
        // along the axis by whatever it takes to stand one radius off the slope plane.
        let centre_at = |at: f64| -> Vector3 {
            let radial = self.radial(at);
            let slide =
                (ball_radius - offset - (spine_radius * radial.dot(slope_normal))) / along_the_axis;

            self.axis_point + (radial * spine_radius) + (self.axis_direction * slide)
        };

        let tangent_at = |at: f64| -> Vector3 {
            let derivative = (self.first * -at.sin()) + (self.second * at.cos());
            let slide = -spine_radius * derivative.dot(slope_normal) / along_the_axis;

            ((derivative * spine_radius) + (self.axis_direction * slide)).get_normal()
        };

        // The signed distance from the section's anchor to the spine's own normal plane at that
        // angle.
        let distance_to_the_plane = |at: f64| (centre_at(at) - through).dot(tangent_at(at));

        // The tightest the spine ever curves: the smallest radius of curvature of the ellipse the
        // slope plane cuts from the radius R + r cylinder.
        let tightest_spine_turn = spine_radius * along_the_axis.abs();

        const SAMPLES: i32 = 128;
        let mut low = self.span_low;
        let mut high = self.span_high;
        let mut previous = distance_to_the_plane(self.span_low);
        let mut bracketed = false;

        // Only a SOURCE round ever closes on itself; a cut round's span is bounded by the two faces
        // its cutter ran out onto.
        let closed_span = self.goes_more_than_half_way_round();
        let mut anchor_radial = through - self.axis_point;
        anchor_radial =
            anchor_radial - self.axis_direction * anchor_radial.dot(self.axis_direction);
        let anchor_angle = anchor_radial
            .dot(self.second)
            .atan2(anchor_radial.dot(self.first));
        let mut nearest = f64::MAX;

        for i in 1..=SAMPLES {
            let at = self.span_low + ((self.span_high - self.span_low) * i as f64 / SAMPLES as f64);
            let here = distance_to_the_plane(at);

            if sign_or_none(here)? != sign_or_none(previous)? {
                let from = self.span_low
                    + ((self.span_high - self.span_low) * (i - 1) as f64 / SAMPLES as f64);

                if closed_span {
                    // A CLOSED span has no "first" end: span_low and span_high are the same
                    // radial, so the one wanted is the station nearest the anchor's own angle about
                    // the axis, which is the branch of the spine the crease is rolling along.
                    let turn = ((from + at) / 2.0) - anchor_angle;
                    let how_far = (turn - (TAU * (turn / TAU).round_ties_even())).abs();

                    if how_far < nearest {
                        nearest = how_far;
                        low = from;
                        high = at;
                        bracketed = true;
                    }

                    previous = here;

                    continue;
                }

                // The FIRST sign change, which is the one nearest span_low - deliberately, where a
                // tight spine has left more than one.
                low = from;
                high = at;
                bracketed = true;
                break;
            }

            previous = here;
        }

        if !bracketed {
            // NEARLY parallel, which the hard gate above only catches when it is exactly so.
            if along_the_axis.abs() < NEARLY_ALONG_THE_AXIS {
                return self.try_find_station_along_the_axis(
                    through,
                    slope_point,
                    slope_normal,
                    ball_radius,
                );
            }

            return None;
        }

        let mut at_low = distance_to_the_plane(low);

        let mut i = 0;
        while i < 80 && high - low > 1e-12 {
            let middle = (low + high) / 2.0;
            let here = distance_to_the_plane(middle);

            if sign_or_none(here)? == sign_or_none(at_low)? {
                low = middle;
                at_low = here;
            } else {
                high = middle;
            }

            i += 1;
        }

        let station = (low + high) / 2.0;
        let centre_there = centre_at(station);

        // THE ROOT HAS TO BE A RISING ONE, or the scan has found one of three and there is no
        // saying which: an anchor further from the spine than the spine's own tightest turn is
        // exactly the case where the distance is no longer monotone.
        if (centre_there - through).length() >= tightest_spine_turn {
            // A nearly parallel slope makes tightest_spine_turn nearly nothing, so this is where
            // such a slope's scan lands when it does bracket something. The closed form is the
            // answer there.
            if along_the_axis.abs() < NEARLY_ALONG_THE_AXIS {
                return self.try_find_station_along_the_axis(
                    through,
                    slope_point,
                    slope_normal,
                    ball_radius,
                );
            }

            return None;
        }

        let centre = centre_there;

        Some(RoundStation {
            phi: station,
            centre,
            tangent: tangent_at(station),
            contact: centre - (self.radial(station) * ball_radius),
        })
    }

    /// The closed form for a slope whose normal is square to the axis: the spine is a straight
    /// line parallel to the axis, and the ball slides freely along it.
    fn try_find_station_along_the_axis(
        &self,
        through: Vector3,
        slope_point: Vector3,
        slope_normal: Vector3,
        ball_radius: f64,
    ) -> Option<RoundStation> {
        // The normal squared to the axis. It is the normal itself to within the 1e-6 that got us
        // here, but taking the component keeps the two-dimensional solve exactly two dimensional.
        let mut square =
            slope_normal - (self.axis_direction * self.axis_direction.dot(slope_normal));
        let square_length = square.length();

        if square_length < 0.5 {
            return None;
        }

        square /= square_length;

        let spine_radius = self.radius + ball_radius;

        // How far the centre stands from the axis ALONG that normal: the plane is at
        // (p - slope_point) . n = ball_radius, and n differs from the in-plane direction only by
        // its length.
        let along =
            ((slope_point - self.axis_point).dot(slope_normal) + ball_radius) / square_length;
        let mut across = (spine_radius * spine_radius) - (along * along);

        if across < 0.0 {
            // The wall does not reach the cylinder at all with a ball of this size between them.
            return None;
        }

        across = across.sqrt();

        let sideways = self.axis_direction.cross(square).get_normal();
        let anchor_relative = through - self.axis_point;
        let anchor_across = anchor_relative.dot(sideways);

        // The root on the crease's side. Both roots are real walls of the same slab; the crease
        // this section is anchored at picks which of the two the ball is actually rolling in.
        let radial = ((square * along)
            + (sideways
                * if anchor_across >= 0.0 {
                    across
                } else {
                    -across
                }))
            / spine_radius;

        let station = radial.dot(self.second).atan2(radial.dot(self.first));
        let slack = math_helper::degrees_to_radians(SPAN_SLACK_DEGREES);

        if station < self.span_low - slack || station > self.span_high + slack {
            return None;
        }

        // The ball slides freely along the axis, so the station is the one whose normal plane -
        // square to the axis - passes through the anchor.
        let foot =
            self.axis_point + (self.axis_direction * anchor_relative.dot(self.axis_direction));

        Some(RoundStation {
            phi: station,
            centre: foot + (radial * spine_radius),
            tangent: self.axis_direction,
            contact: foot + (radial * self.radius),
        })
    }
}

/// -1, 0 or 1, and no answer for NaN, which has no sign.
fn sign_or_none(value: f64) -> Option<i32> {
    if value.is_nan() {
        None
    } else if value > 0.0 {
        Some(1)
    } else if value < 0.0 {
        Some(-1)
    } else {
        Some(0)
    }
}
