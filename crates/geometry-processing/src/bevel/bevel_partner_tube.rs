//! The volume a PARTNER run's rolling ball keeps empty: the cylinder of the bevel's radius about
//! that run's ball-centre line, over the SPAN that run's own bead covers. A run that shares a chain
//! end with this one has to keep its own bead out of it, which is how two beads meeting at a chain
//! end come to bound each other. Ports `Bevel/BevelPartnerTube.cs`.
//!
//! Measured off the Onshape control in "5mm Bevel Around Center.mcx": every one of its 136 faces
//! lies on a plane of the part or on one of four radius 5 cylinders, and where two beads overlap the
//! surface between them is simply the two cylinders crossing.
//!
//! - **Partners are runs that share a chain END, and only those** - not proximity: on that fixture
//!   792 of 1531 sampled points of the left fill lie inside a right-hand cylinder that does not
//!   bound it.
//! - **The tube is FINITE.** An endless tube clipped from every station ate the four beads down to
//!   34.28 mm^3 against the control's 64.458. The ball only keeps a place empty where it can roll,
//!   so the tube ends where the partner's bead ends; past either cap the depth reads +infinity.
//! - **Straight runs only.** A partner that curves has no single cylinder; the caller gets nothing
//!   and caps the bead at that end.
//! - **And the surface is CHORDED where the partner's really is** - see
//!   [`BevelPartnerTube::around_the_chorded_arc`].

use vector_math::{cs_math, Vector3};

/// C# `BevelPartnerTube` (a readonly struct).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BevelPartnerTube {
    /// Any point on the ball-centre line.
    pub axis_point: Vector3,
    /// The ball-centre line's direction, normalized.
    pub axis_direction: Vector3,
    /// The rolling ball's radius, which is the bevel's size.
    pub radius: f64,
    /// Where the partner's own bead starts along the axis, from `axis_point`.
    pub from: f64,
    /// Where it ends.
    pub to: f64,
    /// How many chords the partner's arc is drawn with, or zero for the smooth cylinder.
    facets: i32,
    /// The angle of the arc's FIRST vertex, in the basis `across` hands back.
    first_angle: f64,
    /// The angle one chord spans, which is the arc's whole sweep shared out over the facets.
    step: f64,
}

impl BevelPartnerTube {
    /// How far off a surface a point may be and still be called ON it, in millimetres.
    pub const ON_THE_SURFACE: f64 = 1e-9;

    /// The smooth tube, endless in both directions (C#'s defaults for `from`/`to`): what every
    /// caller that only asks about points beside the partner's own crease means.
    pub fn new(axis_point: Vector3, axis_direction: Vector3, radius: f64) -> Self {
        Self::spanning(
            axis_point,
            axis_direction,
            radius,
            f64::NEG_INFINITY,
            f64::INFINITY,
        )
    }

    /// The smooth tube over the partner's own bead, `from` to `to` along the axis from
    /// `axis_point`. `axis_direction` is normalized here.
    pub fn spanning(
        axis_point: Vector3,
        axis_direction: Vector3,
        radius: f64,
        from: f64,
        to: f64,
    ) -> Self {
        Self::with_facets(axis_point, axis_direction, radius, from, to, 0, 0.0, 0.0)
    }

    #[allow(clippy::too_many_arguments)]
    fn with_facets(
        axis_point: Vector3,
        axis_direction: Vector3,
        radius: f64,
        from: f64,
        to: f64,
        facets: i32,
        first_angle: f64,
        step: f64,
    ) -> Self {
        Self {
            axis_point,
            axis_direction: axis_direction.get_normal(),
            radius,
            from,
            to,
            facets,
            first_angle,
            step,
        }
    }

    /// Whether this is the partner's CHORDED surface rather than the smooth cylinder.
    pub fn is_chorded(&self) -> bool {
        self.facets > 0
    }

    /// The partner's surface as it is really built: the prism its arc is drawn as, inscribed in
    /// the cylinder, with the partner's own chord count and phase.
    ///
    /// Every chord lies inside the cylinder by the sagitta, so a seam clipped against the smooth
    /// cylinder lands OUTSIDE the partner's surface between its vertices and the two surfaces cross
    /// once per chord, shingling the junction (161.55 degree worst bend over 3199 edges on "5mm
    /// Bevel Around Center.mcx", against the control's 112.62 over 87). Given the prism, the seam
    /// lands ON the partner's facet planes.
    ///
    /// The phase is the arc's own, taken from the two face normals: the arc runs between the
    /// tangent points in the directions `-normal0` and `-normal1` from the ball-centre line. Outside
    /// the arc's sweep the surface is the cylinder again. `facets` below one - or a wedge with no
    /// arc to speak of - hands back the smooth cylinder.
    // The negated comparisons are C#'s `!(x > 1e-9)`, which refuse a NaN as well.
    #[allow(clippy::too_many_arguments, clippy::neg_cmp_op_on_partial_ord)]
    pub fn around_the_chorded_arc(
        axis_point: Vector3,
        axis_direction: Vector3,
        radius: f64,
        first_face_normal: Vector3,
        second_face_normal: Vector3,
        facets: i32,
        from: f64,
        to: f64,
    ) -> Self {
        let smooth = Self::spanning(axis_point, axis_direction, radius, from, to);

        if facets < 1 {
            return smooth;
        }

        let first = square_to_the_axis(-first_face_normal, smooth.axis_direction);
        let second = square_to_the_axis(-second_face_normal, smooth.axis_direction);

        if !(first.length() > 1e-9) || !(second.length() > 1e-9) {
            // A face whose normal runs along the crease is not a face this arc was drawn against.
            return smooth;
        }

        let mut from_angle = smooth.angle_of(axis_point + first);
        let mut sweep = smooth.angle_of(axis_point + second) - from_angle;

        while sweep > std::f64::consts::PI {
            sweep -= std::f64::consts::PI * 2.0;
        }

        while sweep < -std::f64::consts::PI {
            sweep += std::f64::consts::PI * 2.0;
        }

        if sweep < 0.0 {
            // The vertices are the same set either way round, so the arc is stated from whichever
            // tangent point makes the sweep positive and every reading below can take that for
            // granted.
            from_angle += sweep;
            sweep = -sweep;
        }

        let chord = sweep / facets as f64;

        if !(chord > 1e-9) || chord >= std::f64::consts::PI {
            // Coplanar faces leave no arc, and one chord across a half turn is not an inscribed
            // prism.
            return smooth;
        }

        Self::with_facets(
            axis_point,
            axis_direction,
            radius,
            from,
            to,
            facets,
            from_angle,
            chord,
        )
    }

    /// How far a point stands OUTSIDE the tube. Negative inside it, zero on its surface.
    ///
    /// Positively infinite past either end of the span: there is no surface out there to be a
    /// distance from. On a chorded tube it is the depth into the convex region the disk and the
    /// facets' half-planes cut out; only the facet a point stands in FRONT of can be the largest
    /// of those bounds, so one is enough to read.
    pub fn depth(&self, point: Vector3) -> f64 {
        let offset = point - self.axis_point;
        let along = offset.dot(self.axis_direction);

        if along < self.from || along > self.to {
            return f64::INFINITY;
        }

        let across = (offset - (self.axis_direction * along)).length();
        let depth = across - self.radius;

        if self.facets < 1 {
            return depth;
        }

        let angle = self.angle_of(point);

        let Some(middle) = self.try_facet(angle) else {
            return depth;
        };

        cs_math::max_f64(
            depth,
            (across * (angle - middle).cos()) - self.facet_distance(),
        )
    }

    /// Where the tube's surface at one angle round the axis crosses a plane, or `None` where the
    /// axis runs parallel to that plane and there is no single crossing. `angle` is in the basis
    /// [`Self::angle_of`] uses; `plane_normal` is the station plane's unit normal.
    ///
    /// This is what makes the seam between two beads exact rather than sampled: the curve the
    /// partner cuts in a station's plane is a conic - or, chorded, the polyline its facets cut -
    /// and every point of it is the crossing of this plane with one of the tube's own rulings.
    pub fn try_cross_plane(
        &self,
        angle: f64,
        plane_point: Vector3,
        plane_normal: Vector3,
    ) -> Option<Vector3> {
        let along = self.axis_direction.dot(plane_normal);

        if along.abs() < 1e-6 {
            // The ruling runs along the plane, so it either misses it or lies in it - neither is a
            // point.
            return None;
        }

        let (first, second) = self.across();

        let on_the_surface = self.axis_point
            + (((first * angle.cos()) + (second * angle.sin())) * self.radius_at(angle));

        Some(
            on_the_surface
                + (self.axis_direction
                    * ((plane_point - on_the_surface).dot(plane_normal) / along)),
        )
    }

    /// The angle round the axis of a point, in the same basis [`Self::try_cross_plane`] uses.
    pub fn angle_of(&self, point: Vector3) -> f64 {
        let (first, second) = self.across();

        let offset = point - self.axis_point;

        offset.dot(second).atan2(offset.dot(first))
    }

    /// How far the surface stands off the axis at one angle: the chord's own reach, or the radius.
    fn radius_at(&self, angle: f64) -> f64 {
        if self.facets > 0 {
            if let Some(middle) = self.try_facet(angle) {
                return self.facet_distance() / (angle - middle).cos();
            }
        }

        self.radius
    }

    /// How far each chord's own plane stands off the axis - the radius, less the sagitta.
    fn facet_distance(&self) -> f64 {
        self.radius * (self.step / 2.0).cos()
    }

    /// The angle of the middle of the facet an angle stands in front of, or `None` where it stands
    /// past either end of the arc and the bound out there is the cylinder itself.
    fn try_facet(&self, angle: f64) -> Option<f64> {
        let mut turned = angle - self.first_angle;

        while turned < 0.0 {
            turned += std::f64::consts::PI * 2.0;
        }

        while turned >= std::f64::consts::PI * 2.0 {
            turned -= std::f64::consts::PI * 2.0;
        }

        // C# `(int)Math.Floor(...)`. .NET 9+ converts saturating with NaN to 0, exactly as Rust's
        // `as` does, so even a NaN angle reads the same facet here as in MatterCAD.
        let facet = (turned / self.step).floor() as i32;

        if facet < 0 || facet >= self.facets {
            return None;
        }

        Some(self.first_angle + ((facet as f64 + 0.5) * self.step))
    }

    /// Two unit directions square to the axis and to each other, chosen the same way every time
    /// so that an angle means the same thing to every caller of the same tube.
    fn across(&self) -> (Vector3, Vector3) {
        let seed = if self.axis_direction.x.abs() < 0.9 {
            Vector3::UNIT_X
        } else {
            Vector3::UNIT_Y
        };

        let first = self.axis_direction.cross(seed).get_normal();
        let second = self.axis_direction.cross(first).get_normal();

        (first, second)
    }
}

/// The part of a direction square to the axis, which is where an arc's tangent point stands.
fn square_to_the_axis(direction: Vector3, axis_direction: Vector3) -> Vector3 {
    direction - (axis_direction * direction.dot(axis_direction))
}
