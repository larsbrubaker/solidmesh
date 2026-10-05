//! Where a rolling ball's CENTRE runs between a mixed corner's two strips, and where it touches
//! the face it rides and the cylinder it is pressed against as it goes - the one source of all
//! three, so that no two readings of the corner can drift apart. Ports
//! `Bevel/Surface/SurfaceMixedCornerSpine.cs`. Built on [`SurfaceLine`] (the bead's axis) and
//! [`SurfaceBandEnd`] (the tube's ring plane); `SurfaceMixedCorner` and its wall, rows and
//! run-out read it.
//!
//! **Two conditions, and they name a curve.** Read the ordinary way, the ball sits inside the
//! material touching the cap face `T` and the bead's void cylinder, so its centre stands a radius
//! INSIDE `T` - a plane - and `2r` from the bead's axis - a cylinder. Plane meets cylinder in an
//! ELLIPSE, and that ellipse is the spine. Turning about the axis is still the honest parameter for
//! it: the angle `phi` out of the axis runs from the way out towards the shared face at zero to the
//! way out towards the wall at the turn's end, exactly as it did, and what the lean of the cap adds
//! is a SLIDE along the axis at each station so that the centre stays in the plane.
//!
//! **And a cap square to the axis is the case that had no slide.** Where the axis stands square to
//! `T` the plane cuts the cylinder square and the ellipse is the circle of radius `2r` the torus
//! was written about: the slide is the same radius at every station, so it cancels into the one
//! constant offset `SurfaceMixedCorner.OuterRailCentre` already carried. That is not an
//! approximation of the old reading, it is the old reading - which is why the square corner's
//! numbers come out as they were at every station and every published face, to the rounding the
//! identity hash is taken at (`SurfaceObliqueMixedCornerReadingTests`). Not to the last BIT: the
//! slide is added before the way out rather than subtracted after it, so the two arithmetics
//! associate differently and may part in the last place of a double.
//!
//! **Both contact curves are planar, whatever the lean.** Write the centre as
//! `E + P cos(phi) + Q sin(phi)` - the slide is itself affine in `cos` and `sin`, because the plane
//! condition is - and the bead's contact, which is the centre pulled halfway back to the axis, has
//! the same shape with `r` where the centre had `2r`. So the tube's contact ring lies in ONE plane,
//! [`SurfaceMixedCornerSpine::tube_ring_plane`], and that plane LEANS from the tube's square
//! section by however much the cap does. A square corner leans it by nothing.
//!
//! **And the two conditions do not say WHOSE ball it is.** A canal between two round strips can be
//! read from either strip, and this type is the reading, not the choice: name the cap, the axis and
//! which side of the cap the centre keeps its radius on, and the rest is the same algebra. The
//! corner of `NoseInNotchTestMeshes` is where that matters. Read as the material ball riding the
//! cap, the canal fits the Onshape control only on the sub-band the two readings happen to share -
//! its cap rail is 0.48 mm out by mid turn. Read as the DUAL - the BEAD's VOID ball, a radius
//! OUTSIDE the wall and `2r` from the CUT's centre line - one canal fits the whole corner to 2.6e-6
//! mm (`SurfaceObliqueMixedCornerWallSpineTests`). So the cap-riding canal is the SQUARE corner's
//! own torus, and a sub-band of an oblique corner's surface rather than the whole of it.
//!
//! **What this type does not decide.** It says where the ball is; it does not say WHICH of the two
//! duals a corner is, how the patch between the two rings is gridded, where the cap's bite is
//! fanned from, or where the run-out picks the spine up. Those are their own rungs - which is why a
//! corner that leans is READ here and refused at the build
//! (`BevelAdvisoryCode::SurfaceObliqueMixedCornerRowsNotBuilt`) rather than published as a blend
//! the right reading and the wrong rows would make.

use vector_math::{math_helper, Vector3};

use super::{SurfaceBandEnd, SurfaceLine};

/// The three arms of the cap-contact curve: `t_contact_at(turn)` is
/// `middle + cosine cos(turn) + sine sin(turn)` (C#'s three `out` parameters).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ContactArms {
    pub middle: Vector3,
    pub cosine: Vector3,
    pub sine: Vector3,
}

/// C# `internal sealed class SurfaceMixedCornerSpine`.
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceMixedCornerSpine {
    axis: SurfaceLine,
    rail_centre: Vector3,
    out_of_the_cap_face: Vector3,
    along_the_axis_out_of_the_cap: f64,
    offset_from_the_cap: f64,
    size: f64,
    towards_the_shared_face: Vector3,
    towards_the_wall_square: Vector3,
    turn_radians: f64,
    tube_ring_plane: SurfaceBandEnd,
}

impl SurfaceMixedCornerSpine {
    /// Reads the spine, or `None` where the corner has none. Every number of it follows from the
    /// frame the caller has already derived; what can fail is the one thing that is not algebra -
    /// see `BevelAdvisoryCode::SurfaceMixedCornerBeadRingCrossesNoRail`.
    ///
    /// - `axis`: the rolling-ball axis - the CYLINDER the ball is pressed against.
    /// - `rail_centre`: where that axis meets the cap face's own plane.
    /// - `out_of_the_cap_face`: the cap's outward normal.
    /// - `along_the_axis_out_of_the_cap`: how square the axis stands to the cap, as the cosine.
    /// - `the_ball_stands_outside_the_cap`: which side of the cap the ball's centre keeps its
    ///   radius on: INSIDE for a ball of material riding a face, OUTSIDE for a void ball riding
    ///   one. Everything else about the two is the same algebra - see the module's notes on the
    ///   dual.
    /// - `towards_the_shared_face`: the way out of the axis at turn zero.
    /// - `towards_the_wall_square`: the quarter turn on from it, on the wall's side.
    /// - `size`: the fillet radius.
    /// - `turn_radians`: how far round the axis the ball rolls.
    /// - `away_along_the_bead`: the way the edge the axis belongs to runs away from the corner,
    ///   which orients the ring plane.
    /// - `square_enough`: how far from parallel to the axis the tube's ring plane must stand, as a
    ///   cosine.
    #[allow(clippy::too_many_arguments)]
    pub fn try_create(
        axis: SurfaceLine,
        rail_centre: Vector3,
        out_of_the_cap_face: Vector3,
        along_the_axis_out_of_the_cap: f64,
        the_ball_stands_outside_the_cap: bool,
        towards_the_shared_face: Vector3,
        towards_the_wall_square: Vector3,
        size: f64,
        turn_radians: f64,
        away_along_the_bead: Vector3,
        square_enough: f64,
    ) -> Option<SurfaceMixedCornerSpine> {
        // An axis lying IN the cap meets the offset plane in two straight lines or in nothing at
        // all, and the slide below would divide by that nothing. The cap-riding reading refuses it
        // before asking, by SurfaceMixedCorner.TryRead's own SquareEnough guard; the DUAL has no
        // production caller yet, so this is the arithmetic's own floor standing under a reading
        // that has nobody to refuse it for it - not a second opinion about the geometry, which is
        // the caller's to name.
        let square = along_the_axis_out_of_the_cap.abs();
        if square.is_nan() || square <= 0.0 {
            return None;
        }

        let offset_from_the_cap = if the_ball_stands_outside_the_cap {
            size
        } else {
            -size
        };

        // P and Q of the TUBE's contact curve, which span the plane that curve lies in.
        let slide_cosine_arm = -2.0 * size * out_of_the_cap_face.dot(towards_the_shared_face)
            / along_the_axis_out_of_the_cap;
        let slide_sine_arm = -2.0 * size * out_of_the_cap_face.dot(towards_the_wall_square)
            / along_the_axis_out_of_the_cap;

        let cosine_arm = (axis.direction * slide_cosine_arm) + (towards_the_shared_face * size);
        let sine_arm = (axis.direction * slide_sine_arm) + (towards_the_wall_square * size);

        // Never zero, and worth writing down rather than branching on: expand the cross product
        // and the two mixed terms are each a vector crossed with something it lies in the span of,
        // so what is left along the axis is r^2 exactly. A degenerate ring plane would need r = 0,
        // which is not a fillet.
        let mut normal = cosine_arm.cross(sine_arm).get_normal();

        if normal.dot(away_along_the_bead) < 0.0 {
            normal = -normal;
        }

        // And THAT is why the one thing that can fail here is a near miss rather than an exact
        // one: the component along the axis is r^2 over the plane's own arm lengths, and those grow
        // as the cap leans, so the ring plane turns to run ALONG the bead only in the sliver of
        // lean just inside the one SurfaceMixedCorner.TryRead already refuses an axis lying in the
        // cap at. Refused rather than cut, because a plane parallel to every rail of the bead names
        // no ring at all.
        let across = normal.dot(axis.direction).abs();
        if across.is_nan() || across <= square_enough {
            return None;
        }

        // The ring stands at turn zero, which is the point the two cylinders touch at. Written out
        // here rather than asked of the spine, so that no spine can exist without its own ring
        // plane: it is BeadContactAt(0) with the turn's cosine and sine substituted, and the two
        // agree to the bit.
        let slide_at_zero = (offset_from_the_cap
            - (2.0 * size * out_of_the_cap_face.dot(towards_the_shared_face)))
            / along_the_axis_out_of_the_cap;
        let ring_stands_at =
            rail_centre + (axis.direction * slide_at_zero) + (towards_the_shared_face * size);

        Some(SurfaceMixedCornerSpine {
            axis,
            rail_centre,
            out_of_the_cap_face,
            along_the_axis_out_of_the_cap,
            offset_from_the_cap,
            size,
            towards_the_shared_face,
            towards_the_wall_square,
            turn_radians,
            tube_ring_plane: SurfaceBandEnd::new(ring_stands_at, normal),
        })
    }

    /// The way out of the bead's axis at turn zero - towards the face the two edges share.
    pub fn towards_the_shared_face(&self) -> Vector3 {
        self.towards_the_shared_face
    }

    /// The quarter turn on from it, on the wall's side, so a direction reads as an angle.
    pub fn towards_the_wall_square(&self) -> Vector3 {
        self.towards_the_wall_square
    }

    /// How far round the axis the ball rolls between the two strips.
    pub fn turn_radians(&self) -> f64 {
        self.turn_radians
    }

    /// The plane the ring the ball traces on the TUBE lies in, as a half space: through that
    /// contact at turn zero - the point the two cylinders touch at - facing the way the tube's own
    /// edge runs away from the corner.
    ///
    /// The contact curve is `E + P cos(phi) + Q sin(phi)`, so its plane is spanned by `P` and `Q`
    /// wherever those two are independent - which they are while the cap is not parallel to the
    /// axis, the one case `SurfaceMixedCorner` refuses before building this. The offset's sign does
    /// not enter either arm: it is a constant of the slide, so the DUAL's ring plane is read by the
    /// same algebra.
    ///
    /// **Which ring it is depends on which reading built the spine.** On the cap-riding one it is
    /// the BEAD's contact ring - which is not what stops the bead's band, since the bead's band
    /// stops on its own square section (`SurfaceMixedCorner.BeadStopsAt`). On the DUAL it is the
    /// CUT's own end ring, the design's `CutRingPlane`: planar, leaning 63.43 degrees across the
    /// cut, and square-ended at both ends of the turn
    /// (`SurfaceObliqueMixedCornerWallSpineTests.TheCutsEndRingIsPlanarAndLeansAcrossTheCut`).
    pub fn tube_ring_plane(&self) -> SurfaceBandEnd {
        self.tube_ring_plane
    }

    /// How far the cap face leans off square to the bead's axis, in degrees.
    pub fn cap_lean_degrees(&self) -> f64 {
        math_helper::radians_to_degrees(
            self.along_the_axis_out_of_the_cap
                .abs()
                .clamp(-1.0, 1.0)
                .acos(),
        )
    }

    /// The way out of the bead's axis at `turn`: towards the shared face at zero and towards the
    /// wall at [`Self::turn_radians`].
    pub fn out_of_the_axis(&self, turn: f64) -> Vector3 {
        (self.towards_the_shared_face * turn.cos()) + (self.towards_the_wall_square * turn.sin())
    }

    /// How far the ball's centre stands along the axis from where the axis meets the cap face, at
    /// `turn` - the slide the lean of the cap asks for.
    ///
    /// From the plane condition alone. The centre is `railCentre + a t + 2r e(phi)` and has to
    /// stand a radius inside the cap, whose plane passes through `railCentre`; take the cap normal
    /// of both sides and solve for `t`. On a square cap `e(phi)` lies IN the cap, so the second
    /// term vanishes identically and `t` is the same radius at every station.
    pub fn slide_at(&self, turn: f64) -> f64 {
        (self.offset_from_the_cap
            - (2.0 * self.size * self.out_of_the_cap_face.dot(self.out_of_the_axis(turn))))
            / self.along_the_axis_out_of_the_cap
    }

    /// Where the rolling ball's centre stands at `turn`: `2r` out of the axis, a radius in from the
    /// cap.
    ///
    /// Written as the axis's own foot in the cap plus the slide plus the way out, in that order,
    /// because on a square cap the slide is exactly the radius under the cap that the torus reading
    /// subtracted and the two arithmetics have to agree to the last bit, not merely to a tolerance.
    pub fn centre_at(&self, turn: f64) -> Vector3 {
        self.rail_centre
            + (self.axis.direction * self.slide_at(turn))
            + (self.out_of_the_axis(turn) * (2.0 * self.size))
    }

    /// Where the ball touches the CAP face at `turn` - the patch's outer rail.
    pub fn t_contact_at(&self, turn: f64) -> Vector3 {
        self.centre_at(turn) - (self.out_of_the_cap_face * self.offset_from_the_cap)
    }

    /// The cap-contact curve written out as its own three arms: [`Self::t_contact_at`] at `turn`
    /// is `middle + cosine cos(turn) + sine sin(turn)`, to the last bits of a double.
    ///
    /// So that a question about WHERE that curve meets a plane is one closed form rather than a
    /// search: substituted into a plane's own equation the three arms give `A cos + B sin = K`,
    /// whose roots are an arc cosine (`SurfaceMixedCornerWall.TryWhereTheContactMeetsTheWall`). On
    /// a square cap the two slide arms are zero and what is left is the circle of radius `2r` about
    /// where the axis meets the cap, which is the curve the run-out was written about.
    pub fn the_contact_arms(&self) -> ContactArms {
        let slide_cosine_arm =
            -2.0 * self.size * self.out_of_the_cap_face.dot(self.towards_the_shared_face)
                / self.along_the_axis_out_of_the_cap;
        let slide_sine_arm =
            -2.0 * self.size * self.out_of_the_cap_face.dot(self.towards_the_wall_square)
                / self.along_the_axis_out_of_the_cap;

        ContactArms {
            middle: self.rail_centre
                + (self.axis.direction
                    * (self.offset_from_the_cap / self.along_the_axis_out_of_the_cap))
                - (self.out_of_the_cap_face * self.offset_from_the_cap),
            cosine: (self.axis.direction * slide_cosine_arm)
                + (self.towards_the_shared_face * (2.0 * self.size)),
            sine: (self.axis.direction * slide_sine_arm)
                + (self.towards_the_wall_square * (2.0 * self.size)),
        }
    }

    /// Where the ball touches the BEAD's cylinder at `turn`: the centre stands `2r` out of the axis
    /// and the cylinder `r`, so the contact is the centre pulled halfway back.
    pub fn bead_contact_at(&self, turn: f64) -> Vector3 {
        self.rail_centre
            + (self.axis.direction * self.slide_at(turn))
            + (self.out_of_the_axis(turn) * self.size)
    }

    /// How far along the bead, away from the corner, the ball stands at `turn` - read from the
    /// axis's foot at `corner` (the vertex the two edges meet at), so that it can be measured
    /// against the bead's own length. `away_along_the_bead` is the way the concave edge runs away
    /// from it.
    pub fn along_the_bead_at(
        &self,
        turn: f64,
        corner: Vector3,
        away_along_the_bead: Vector3,
    ) -> f64 {
        (self.centre_at(turn) - self.axis.closest_point_to(corner)).dot(away_along_the_bead)
    }
}
