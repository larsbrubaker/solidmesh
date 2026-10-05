//! The corner where a chain's convexity TURNS OVER ([`SurfaceMixedCorner`]) - a convex edge's
//! cut running into a concave edge's bead at one vertex - read off the two edge plans that meet
//! there, before anything is cut or stitched. Ports the record of
//! `Bevel/Surface/SurfaceMixedCorner.cs`; its reading and its derived frame (axis, rails, the
//! outward sum, which canal it takes) land with the mixed-corner package's methods.

use std::cell::Cell;
use std::rc::Rc;

use vector_math::Vector3;

use crate::bevel::BevelAdvisory;

use super::{
    SurfaceBandEnd, SurfaceEdgePlan, SurfaceLine, SurfaceMixedCornerDualFrame,
    SurfaceMixedCornerSpine, SurfaceWallSection,
};

/// C# `SurfaceMixedCorner` (a sealed class): the frame of the surface a ball rolling from inside
/// the crease to outside the rim sweeps between a cut and a bead - derived once, here, so that the
/// walkback, the strip ends and the patch all quote the same numbers. Shared by every reading of
/// the corner (the canal, the walls, the run-out, a junction's arms), hence held through `Rc`.
///
/// A mitre cannot serve here, because the two strips are not one surface bent round the corner:
/// one bulges into the void and the other into the material, and no plane through the corner cuts
/// both of them where they belong.
///
/// **How sharp a corner this describes.** The ball's touch point on the cap traces a circle of
/// radius `2r` about the axis, and the axis stands `r / sin(wedge / 2)` from the corner - so the
/// ball reaches the corner only while the void wedge is sixty degrees or more. Under that the cap
/// is left a spike nothing closes, which is a corner of another shape; it is refused by name.
///
/// **Name the three faces.** The two edges MUST share one face region: S, the SHARED face. The
/// convex edge's other region is T, the face the cut also comes down from; the concave edge's
/// other region is W, the wall the bead also stands against. The corner vertex lies on all three.
///
/// **The two cylinders are tangent, at one point.** The cut is one cylinder of radius `r` and the
/// bead another; their axes are skew, and because both stand a radius off S's plane on OPPOSITE
/// sides their common perpendicular is square to S and exactly `2r` long. So they touch at ONE
/// point, [`SurfaceMixedCorner::tangent_point`], the midpoint of that perpendicular. There is no
/// trim curve between the strips; the gap between them is a new surface.
///
/// **And either of the other two faces may be the shared one.** Where S is a chorded wall the bead
/// is tangent to the facet `k` its profile ROLLED to, and the CUT is the chord standing on facet
/// `k` rather than the convex edge arriving at the corner (`SurfaceMixedCornerCutEdge`).
///
/// **Where each strip stops.** The cut stops on the HANDOVER plane - through the bead's axis,
/// spanned by that axis and the way out of it towards S - and the bead stops on the cut's own
/// tangent plane in S. Both pass through the tangent point; stated as [`SurfaceBandEnd`]s they
/// bound the band each edge's walkback takes off as well, so the material removed and the ribbon
/// put back stop on one surface rather than on two that nearly agree.
///
/// **And the blend between them is a canal on an elliptical spine** ([`SurfaceMixedCornerSpine`]):
/// the ball's centre stands a radius inside T and `2r` from the bead's axis. Where the axis LEANS
/// that canal is not the whole corner - measured against the Onshape control on
/// `NoseInNotchTestMeshes` the corner is the DUAL canal (`SurfaceMixedCornerDualFrame`), and which
/// of the two a leaning corner takes is decided by CREASE REACH and never by the lean
/// (`SurfaceMixedCornerWhichCanal`).
#[derive(Debug)]
pub struct SurfaceMixedCorner {
    /// Remembers `TakesTheDualCanal`, which is a walk over the wall's whole rim: asked once per
    /// band end per edge, and nothing it is read from can change after the corner is read. `None`
    /// until first asked (C#'s `bool?`).
    pub takes_the_dual_canal: Cell<Option<bool>>,
    /// Where the rolling ball's centre runs and what it touches as it goes - the ONE source of all
    /// three, which the accessors quote rather than derive a second time.
    pub spine: SurfaceMixedCornerSpine,
    /// Whether the ring is a MITRE ring the chain already had rather than the section through the
    /// point the two cylinders touch at (`SurfaceMixedCornerHandsOverAtTheMitre`). Two things
    /// follow: the cut's band stops on that plane on the face the two edges SHARE as well - the
    /// ordinary corner leaves that side to the other edge's band, but here the ring stands short of
    /// the bead and the blend covers between - and the patch's stations lean
    /// (`SurfaceMixedCornerRingSchedule`).
    pub hands_over_at_a_mitre: bool,
    /// The vertex the two edges meet at, which lies on all three faces of the corner.
    pub corner: Vector3,
    /// The fillet radius the corner was read at.
    pub size: f64,
    /// What names the face region the two edges SHARE - the one S stands for above.
    pub shared_region_key: i32,
    /// The convex edge's plan, whose strip is the cut.
    pub cut: SurfaceEdgePlan,
    /// The concave edge's plan, whose strip is the bead.
    pub bead: SurfaceEdgePlan,
    /// The one point the two cylinders touch at: on the shared face, a radius from both centre
    /// lines, and the corner the shared face's walkback is left with.
    pub tangent_point: Vector3,
    /// The handover plane, as the half space the CUT's band and strip are kept in: through the
    /// tangent point, facing the way the convex edge runs away from the corner.
    pub cut_stops_at: SurfaceBandEnd,
    /// The plane the BEAD's band and end ring both stop on: the bead's own SQUARE section through
    /// the point the two cylinders touch at, facing the way the concave edge runs away from the
    /// corner.
    ///
    /// Square, and not the leaning ring the cap-riding spine traces on the bead
    /// (`SurfaceMixedCornerSpine::tube_ring_plane`), because that spine is not this corner's
    /// surface where the corner leans: the blend is the DUAL canal, whose first ball stands ON the
    /// bead's axis, so the canal leaves the bead's square section tangentially and the two meet G1
    /// (`SurfaceObliqueMixedCornerWallSpineTests.TurnZeroIsABallOfTheBeadItself`). On a square
    /// corner the two planes are the same plane.
    pub bead_stops_at: SurfaceBandEnd,
    /// The centre of the arc the ball's touch point traces on the cap face: where the bead's axis
    /// meets that face's plane.
    pub outer_rail_centre: Vector3,
    /// The cap face's outward normal - the face the outer rail runs across.
    pub out_of_the_cap_face: Vector3,
    /// The void wall's outward normal - the plane the bead's other tangency lies in. On a wall the
    /// bead rolled along, that is the FACET it rolled to and not the chain edge's own facet.
    pub out_of_the_wall: Vector3,
}

impl SurfaceMixedCorner {
    /// How near square two unit directions have to be to count as square - as the cosine between
    /// them, or as the sine for the tests written the other way about. Dimensionless, and so a
    /// FIXED epsilon rather than the part's own ruler: an angle does not grow with the bounding
    /// box. Lengths keep the ruler; angles keep this.
    pub const SQUARE_ENOUGH: f64 = 1e-6;

    /// The bead's rolling-ball axis, the concave edge's own centre line.
    pub fn axis(&self) -> SurfaceLine {
        self.bead.center_line
    }

    /// The cut's centre line, which is the ball's centre at turn zero.
    pub fn cut_center_line(&self) -> SurfaceLine {
        self.cut.center_line
    }

    /// How far the ball rolls round the bead's axis between the two strips, in radians: from the
    /// way out towards the shared face to the way out towards the wall, which is 180 degrees less
    /// the void wedge the bead is drawn in.
    pub fn turn_radians(&self) -> f64 {
        self.spine.turn_radians()
    }

    /// The radius of the arc the ball's touch point traces on the cap face, `2r` - the ball's own
    /// radius out from the axis, twice.
    pub fn outer_rail_radius(&self) -> f64 {
        2.0 * self.size
    }

    /// The unit way out of the bead's axis towards the shared face, which is turn zero.
    pub fn towards_the_shared_face(&self) -> Vector3 {
        self.spine.towards_the_shared_face()
    }

    /// The other half of the cap plane's frame: the way out of the axis square to
    /// [`SurfaceMixedCorner::towards_the_shared_face`], on the wall's side. Turn zero and a quarter
    /// turn, so a direction in the cap reads as an ANGLE against the pair of them.
    pub fn towards_the_wall_square(&self) -> Vector3 {
        self.spine.towards_the_wall_square()
    }

    /// The walk across the face the BEAD was ROLLED along, or `None` where nothing was rolled and
    /// both of the bead's faces are one plane each. It is the face the strip under the blend comes
    /// off, and it is not always the wall nobody picked: see
    /// [`SurfaceMixedCorner::rolled_is_the_shared_face`].
    pub fn rolled_section(&self) -> Option<Rc<SurfaceWallSection>> {
        self.bead.wall_section.clone()
    }

    /// Whether the face the bead rolled along is the face the two edges SHARE rather than the
    /// wall nobody picked. Read off the bead's own plan - the rolled side is the side that carries
    /// a band - so nothing is assumed about which of them curves.
    pub fn rolled_is_the_shared_face(&self) -> bool {
        self.rolled_section().is_some()
            && (if self.bead.left_band.is_some() {
                self.bead.left_region_key()
            } else {
                self.bead.right_region_key()
            }) == self.shared_region_key
    }

    /// The walk across the wall NOBODY PICKED, or `None` where that wall is one plane. `None` as
    /// soon as the bead rolled along the shared face instead, because a bead that rolls has ONE
    /// curved face, so the other one is flat and the single-segment reading is exact for it.
    pub fn unpicked_section(&self) -> Option<Rc<SurfaceWallSection>> {
        if self.rolled_is_the_shared_face() {
            None
        } else {
            self.rolled_section()
        }
    }

    /// The shared face's outward normal.
    pub fn out_of_the_shared_face(&self) -> Vector3 {
        -self.towards_the_shared_face()
    }

    /// A direction every triangle of the patch faces to within a right angle: the sum of the three
    /// faces' outward normals. The patch's own outward turns from the cap's normal at profile zero
    /// to the shared face's at the far end of the profile and to the wall's at the far end of the
    /// turn, so all three are in the sum and nothing on it leaves the hemisphere they name. The
    /// margin closes only for a void wedge of nothing, which the plan has already refused
    /// ([`super::SurfaceBevelPlan::MINIMUM_WEDGE_ANGLE_DEGREES`]).
    pub fn outward_sum(&self) -> Vector3 {
        self.out_of_the_cap_face + self.out_of_the_shared_face() + self.out_of_the_wall
    }

    /// Whether the bead's axis stands off square to the cap - which is what makes the corner's own
    /// surface the DUAL canal rather than the cap-riding one, and so which planes everything about
    /// it is cut at.
    pub fn the_cap_leans(&self) -> bool {
        // C#'s `!(cosine > 1 - SquareEnough)`, which a NaN cosine also passes.
        let cosine = self.out_of_the_cap_face.dot(self.axis().direction).abs();
        cosine <= 1.0 - Self::SQUARE_ENOUGH || cosine.is_nan()
    }

    /// Reads this corner's OTHER canal - the bead's void ball rolling on the wall, pressed against
    /// the cut's tube - and with it the plane the cut's end ring would lie in. On REQUEST: a
    /// two-crease junction asks for it (it IS this canal), and nothing else does, so a corner
    /// nobody asks keeps the cap-riding canal's numbers unchanged to the bit.
    pub fn try_read_the_dual_canal(&self) -> Result<SurfaceMixedCornerDualFrame, BevelAdvisory> {
        SurfaceMixedCornerDualFrame::try_read_the_dual(self, Self::SQUARE_ENOUGH)
    }

    /// The way out of the bead's axis at `turn`: towards the shared face at zero and towards the
    /// wall at [`SurfaceMixedCorner::turn_radians`].
    pub fn out_of_the_axis(&self, turn: f64) -> Vector3 {
        self.spine.out_of_the_axis(turn)
    }

    /// `point` carried `turn` round the bead's axis, the way the turn runs - from the shared face
    /// towards the wall.
    ///
    /// The RIGID turn, which is the ball's own carry only while the cap stands square to the axis:
    /// the patch's rows are laid on the spine instead. What still turns rigidly is everything the
    /// lean gate holds back - the mitre ring's schedule, the run-out's columns and the junction's
    /// patch - so this is the torus's carry, kept under that gate.
    pub fn turned_about(&self, point: Vector3, turn: f64) -> Vector3 {
        // Rodrigues about the axis LINE, so the point is turned about the axis rather than about
        // the origin: take the offset from a point on the line, turn it, put it back.
        let axis = self
            .towards_the_shared_face()
            .cross(self.towards_the_wall_square());
        let from = self.axis().closest_point_to(point);
        let offset = point - from;

        from + (offset * turn.cos())
            + (axis.cross(offset) * turn.sin())
            + (axis * axis.dot(offset) * (1.0 - turn.cos()))
    }

    /// The way one planned edge runs away from `corner`, which is one of its two ends. Shared with
    /// `SurfaceMixedCornerDualFrame`, which orients the dual's own ring plane by the CUT's way out
    /// of the corner exactly as this reading orients the bead's by the bead's.
    pub fn away_from_the_corner(plan: &SurfaceEdgePlan, corner: Vector3) -> Vector3 {
        if (plan.from() - corner).length_squared() <= (plan.to() - corner).length_squared() {
            plan.edge.direction()
        } else {
            -plan.edge.direction()
        }
    }
}
