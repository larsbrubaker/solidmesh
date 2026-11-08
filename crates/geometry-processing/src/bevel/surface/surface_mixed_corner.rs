//! The corner where a chain's convexity TURNS OVER ([`SurfaceMixedCorner`]) - a convex edge's cut
//! running into a concave edge's bead at one vertex - read off the two edge plans that meet there,
//! before anything is cut or stitched. Holds the corner's record with its derived frame (axis,
//! rails, the outward sum), which canal it takes, and the carries into the cap and down to the
//! bead's end. Its reading (`try_read`) is in `surface_mixed_corner_read_off_the_edge_plans`, and
//! its other methods in the neighbouring `surface_mixed_corner_*` files.

use std::cell::Cell;
use std::rc::Rc;

use solidmesh_math::Vector3;

use crate::bevel::BevelAdvisory;
use crate::topology::EdgeConvexity;

use super::surface_mixed_corner_which_canal;
use super::{
    SurfaceBandEnd, SurfaceEdgePlan, SurfaceLine, SurfaceMixedCornerDualFrame,
    SurfaceMixedCornerSpine, SurfaceWallSection,
};

/// The frame of the surface a ball rolling from inside the crease to outside the rim sweeps between
/// a cut and a bead - derived once, here, so that the walkback, the strip ends and the patch all
/// quote the same numbers. Shared by every reading of the corner (the canal, the walls, the
/// run-out, a junction's arms), hence held through `Rc`.
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
    /// Remembers `takes_the_dual_canal`, which is a walk over the wall's whole rim: asked once per
    /// band end per edge, and nothing it is read from can change after the corner is read. `None`
    /// until first asked.
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
    /// (`surface_oblique_mixed_corner_wall_spine_tests::turn_zero_is_a_ball_of_the_bead_itself`).
    /// On a square corner the two planes are the same plane.
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

    /// Builds the corner from its readings; `takes_the_dual_canal` starts unasked.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        corner: Vector3,
        size: f64,
        shared_region_key: i32,
        cut: SurfaceEdgePlan,
        bead: SurfaceEdgePlan,
        tangent_point: Vector3,
        cut_stops_at: SurfaceBandEnd,
        bead_stops_at: SurfaceBandEnd,
        spine: SurfaceMixedCornerSpine,
        outer_rail_centre: Vector3,
        out_of_the_cap_face: Vector3,
        out_of_the_wall: Vector3,
        hands_over_at_a_mitre: bool,
    ) -> Self {
        SurfaceMixedCorner {
            takes_the_dual_canal: Cell::new(None),
            spine,
            hands_over_at_a_mitre,
            corner,
            size,
            shared_region_key,
            cut,
            bead,
            tangent_point,
            cut_stops_at,
            bead_stops_at,
            outer_rail_centre,
            out_of_the_cap_face,
            out_of_the_wall,
        }
    }

    /// Whether BOTH bands are stopped at this corner on the face the two edges SHARE as well as
    /// on the other one - only where the corner hands over on a MITRE ring, where the cut's ring
    /// stands SHORT of the bead's and the blend covers the stretch between.
    pub fn both_bands_stop_on_the_shared_face(&self) -> bool {
        self.hands_over_at_a_mitre
    }

    /// Where the rolling ball touches the cap face at `turn`.
    pub fn outer_rail_point(&self, turn: f64) -> Vector3 {
        self.spine.t_contact_at(turn)
    }

    /// Where the rolling ball's centre stands at `turn`: `2r` out of the axis, a radius in from the
    /// cap.
    pub fn ball_centre_at(&self, turn: f64) -> Vector3 {
        self.spine.centre_at(turn)
    }

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
        // Negated on purpose: a NaN cosine also passes.
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

/// The corner's derived planes and carries: where its ring stands, which canal it takes, and how a
/// point is carried into the cap or down to the bead's end.
impl SurfaceMixedCorner {
    /// Where the patch's own RING stands on the face the two edges share, lifted into the cap's
    /// plane - the far end of the strip under the blend when the bead rolled along THAT face, and
    /// the mirror of [`SurfaceMixedCorner::cap_corner`] at the other end of the turn.
    ///
    /// The cut's tangent line in the shared face, met with the plane the ring is cut at. At an
    /// ordinary corner that is the tangent point itself, read in the cap; at a MITRE handover the
    /// ring stands SHORT of the tangency, so the strip under the blend has to reach the RAIL (on
    /// the test fixture the rail is at 1.5696 where the tangency is at 1.3288).
    pub fn shared_face_foot(&self) -> Vector3 {
        let in_the_shared_face = if self.cut.left_region_key() == self.shared_region_key {
            self.cut.left_tangent_line
        } else {
            self.cut.right_tangent_line
        };
        let across = in_the_shared_face.direction.dot(self.cut_stops_at.inward);

        // The ring's plane parallel to the cut's own tangent line would name no rail at all. It
        // cannot happen at a corner this class read, and the tangency is what the rail is then.
        let rail = if across.abs() > 1e-9 {
            in_the_shared_face.point
                + (in_the_shared_face.direction
                    * ((self.cut_stops_at.point - in_the_shared_face.point)
                        .dot(self.cut_stops_at.inward)
                        / across))
        } else {
            self.tangent_point
        };

        self.up_into_the_cap(rail)
    }

    /// A point of the WALL or of the shared face, carried into the cap face's own plane along a
    /// line both faces hold - the AXIS, measured against the outer rail's centre - so that a
    /// reading taken in the cap's plane is still a reading of that face.
    pub fn up_into_the_cap(&self, at: Vector3) -> Vector3 {
        self.up_into_the_cap_through(at, self.outer_rail_centre)
    }

    /// The same, naming which point of the cap's plane the carry is measured against - one plane,
    /// and so one answer, but the arithmetic of a square corner is kept to the bit wherever it was
    /// written against the corner vertex. Where the cap stands SQUARE the carry is along the cap's
    /// normal; where it LEANS, along the axis.
    pub fn up_into_the_cap_through(&self, at: Vector3, through_the_cap_at: Vector3) -> Vector3 {
        if !self.the_beads_ring_leans() {
            return at
                + (self.out_of_the_cap_face
                    * (through_the_cap_at - at).dot(self.out_of_the_cap_face));
        }

        let along = self.axis().direction;

        at + (along
            * ((through_the_cap_at - at).dot(self.out_of_the_cap_face)
                / along.dot(self.out_of_the_cap_face)))
    }

    /// The corner the cap face, the wall and the meridian plane at the turn all meet at - the point
    /// the planar cap that closes the patch is fanned from: a radius out of the axis along the last
    /// turn's own way out, carried back into the cap along the axis where the cap leans.
    pub fn cap_corner(&self) -> Vector3 {
        self.up_into_the_cap(
            self.outer_rail_centre + (self.out_of_the_axis(self.turn_radians()) * self.size),
        )
    }

    /// The plane the BEAD's band and its end ring are really cut at - its own square section
    /// ordinarily, and the LEANING ring the cap-riding ball traces on the bead where this corner's
    /// canal is the cap-riding one and the cap leans (9.7 um off the square section at 5.625
    /// degrees of lean, at `r = 1`). On the DUAL, and on a square corner, nothing moves.
    pub fn bead_ring_stands_on(&self) -> SurfaceBandEnd {
        if self.the_beads_ring_leans() {
            self.spine.tube_ring_plane()
        } else {
            self.bead_stops_at
        }
    }

    /// Whether that plane is the LEANING one - which says the strip of wall under the blend is no
    /// longer the section extruded a radius down from the cap. Asked as one question everywhere, so
    /// the plane a band was cut at and the face under it agree.
    pub fn the_beads_ring_leans(&self) -> bool {
        self.the_cap_leans() && !self.takes_the_dual_canal()
    }

    /// Where a point of the CAP's plane stands when it is carried down the wall to the plane the
    /// bead's band and end ring stop on - the lower rim of the strip of wall the blend stands over.
    /// Down the AXIS (the wall runs along it); on a SQUARE cap the drop is the radius exactly,
    /// taken as the constant. The divisor cannot vanish: the spine is refused below
    /// `SQUARE_ENOUGH`.
    pub fn down_to_the_beads_end(&self, in_the_cap: Vector3) -> Vector3 {
        if !self.the_beads_ring_leans() {
            return in_the_cap - (self.out_of_the_cap_face * self.size);
        }

        let stops_on = self.bead_ring_stands_on();
        let axis = self.axis().direction;
        let down = axis
            * if axis.dot(self.out_of_the_cap_face) > 0.0 {
                -1.0
            } else {
                1.0
            };

        in_the_cap
            + (down
                * ((stops_on.point - in_the_cap).dot(stops_on.inward) / down.dot(stops_on.inward)))
    }

    /// Which of the two stopping planes bounds `edge` - the handover for the convex edge's cut (or,
    /// on the DUAL, the canal's own leaning cut-ring plane, which on the nose fixture dips 0.89 mm
    /// further back at mid turn), the plane the bead's own ring stands on for the concave edge's
    /// bead.
    pub fn band_end_for(&self, edge: &SurfaceEdgePlan) -> SurfaceBandEnd {
        if edge.convexity() != EdgeConvexity::Convex {
            return self.bead_ring_stands_on();
        }

        if self.takes_the_dual_canal() {
            if let Ok(dual) = self.try_read_the_dual_canal() {
                return dual.cut_ring_plane();
            }
        }

        self.cut_stops_at
    }

    /// Whether this corner's blend is the DUAL canal rather than the cap-riding one, decided by
    /// CREASE REACH and never by the lean. Cached, because the walk is over the wall region's whole
    /// rim and this is asked once per band end per edge; nothing it is read from changes after the
    /// corner is read.
    pub fn takes_the_dual_canal(&self) -> bool {
        if let Some(takes) = self.takes_the_dual_canal.get() {
            return takes;
        }

        let takes = surface_mixed_corner_which_canal::the_corner_takes_the_dual(self);
        self.takes_the_dual_canal.set(Some(takes));
        takes
    }
}

impl SurfaceMixedCorner {
    /// The ordinary corner read off its two edge plans
    /// ([`surface_mixed_corner_read_off_the_edge_plans::try_read`]).
    ///
    /// [`surface_mixed_corner_read_off_the_edge_plans::try_read`]: super::surface_mixed_corner_read_off_the_edge_plans::try_read
    pub fn try_read(
        arriving: &SurfaceEdgePlan,
        leaving: &SurfaceEdgePlan,
        corner: Vector3,
        size: f64,
        near_enough: f64,
    ) -> Result<SurfaceMixedCorner, crate::bevel::BevelAdvisory> {
        super::surface_mixed_corner_read_off_the_edge_plans::try_read(
            arriving,
            leaving,
            corner,
            size,
            near_enough,
        )
    }

    /// A corner whose ring another plan carries and stops.
    #[allow(clippy::too_many_arguments)]
    pub fn try_read_carried(
        ring_carried_by: &SurfaceEdgePlan,
        touching_the_bead: &SurfaceEdgePlan,
        ring_stops_at: Option<SurfaceBandEnd>,
        arriving: &SurfaceEdgePlan,
        leaving: &SurfaceEdgePlan,
        corner: Vector3,
        size: f64,
        near_enough: f64,
    ) -> Result<SurfaceMixedCorner, crate::bevel::BevelAdvisory> {
        super::surface_mixed_corner_read_off_the_edge_plans::try_read_carried(
            ring_carried_by,
            touching_the_bead,
            ring_stops_at,
            arriving,
            leaving,
            corner,
            size,
            near_enough,
        )
    }
}
