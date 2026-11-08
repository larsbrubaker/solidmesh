//! The mixed corner's OTHER canal ([`SurfaceMixedCornerDualFrame`]): the bead's VOID ball, rolling
//! along the wall and pressed against the cut's tube: its record with its reading
//! ([`SurfaceMixedCornerDualFrame::try_read_the_dual`]) and its rail accessors.

use solidmesh_math::Vector3;

use super::{SurfaceBandEnd, SurfaceLine, SurfaceMixedCorner, SurfaceMixedCornerSpine};
use crate::bevel::{BevelAdvisory, BevelAdvisoryArg, BevelAdvisoryCode};

/// The mixed corner's OTHER canal, read - the bead's VOID ball, rolling along the wall and pressed
/// against the cut's tube, where the reading proper carries the material ball riding the cap.
/// Shared by a junction's arm and the rings sewn over it, hence held through `Rc`.
///
/// **The same spine, with its roles swapped.** [`SurfaceMixedCornerSpine`] says where a ball
/// stands that keeps a fixed offset from one plane and a fixed distance from one line; which plane
/// and which line is the caller's to name. This names the WALL as the plane - with the ball a
/// radius OUTSIDE it, since a void ball stands outside the material - and the CUT's own centre
/// line as the axis, and turns from the shared face's outward normal round to the cap's, which is
/// the cut strip's own sweep.
///
/// **Who asks for it, and what is still not known.** Measured against the Onshape control, the
/// oblique corner of `nose_in_notch_test_meshes` IS this canal and not the cap-riding one, to
/// 2.6e-6 mm against 0.48 mm (`SurfaceObliqueMixedCornerWallSpineTests`).
/// `SurfaceTwoCreaseJunction` asks for it by NAME: at a junction of one cut and two creases the
/// blend is this canal by construction. What is NOT known is the rule for a corner where both
/// canals read and only one is right: on a SQUARE corner both span the same two end rings, and no
/// test written on the two faces' PLANES separates them - the one that does is whether the contact
/// rail stays on the face's own EXTENT, which this reading cannot see
/// (`SurfaceMixedCornerDualOnASquareCornerTests`).
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceMixedCornerDualFrame {
    /// The axis the void ball is pressed against: the CUT's own centre line.
    pub axis: SurfaceLine,
    /// Where that centre line crosses the wall's own plane - the dual's "rail centre".
    pub rail_centre: Vector3,
    /// The wall's outward normal, which the ball keeps a radius outside.
    pub out_of_the_wall: Vector3,
    /// How square the cut stands to the wall, as the cosine - what the slide divides by.
    pub along_the_cut_out_of_the_wall: f64,
    /// How far round the cut the ball rolls: from the shared face's outward normal to the cap's,
    /// which is the CUT strip's own sweep and not the bead's.
    pub turn_radians: f64,
    /// The spine itself - where the void ball's centre runs, and what it touches.
    pub spine: SurfaceMixedCornerSpine,
}

impl SurfaceMixedCornerDualFrame {
    /// Builds the frame from its parts; [`Self::try_read_the_dual`] reads one.
    fn new(
        axis: SurfaceLine,
        rail_centre: Vector3,
        out_of_the_wall: Vector3,
        along_the_cut_out_of_the_wall: f64,
        turn_radians: f64,
        spine: SurfaceMixedCornerSpine,
    ) -> Self {
        SurfaceMixedCornerDualFrame {
            axis,
            rail_centre,
            out_of_the_wall,
            along_the_cut_out_of_the_wall,
            turn_radians,
            spine,
        }
    }

    /// Reads the dual of `corner` (already read the ordinary way), or says why it has none.
    /// `square_enough` is how far from lying IN the wall the cut must stand, as a cosine - the
    /// same dimensionless floor the reading proper keeps its own angles at.
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > y)` on purpose: it refuses a NaN as well.
    pub fn try_read_the_dual(
        corner: &SurfaceMixedCorner,
        square_enough: f64,
    ) -> Result<SurfaceMixedCornerDualFrame, BevelAdvisory> {
        let at_the_corner = || vec![BevelAdvisoryArg::Vector3(corner.corner)];
        let axis = corner.cut_center_line();
        let out_of_the_wall = corner.out_of_the_wall;
        let along_the_cut_out_of_the_wall = axis.direction.dot(out_of_the_wall);

        // Asked BEFORE anything divides by it. A cut lying IN the wall meets the plane a radius
        // outside that wall in two straight lines or in nothing at all: there is no ellipse to
        // roll a ball on, and the slide would be an infinity rather than a number. Dimensionless,
        // so the fixed floor and not the part's own ruler.
        if !(along_the_cut_out_of_the_wall.abs() > square_enough) {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceMixedCornerCutLiesInTheWall,
                at_the_corner(),
            ));
        }

        // Where the cut's centre line crosses the wall's own plane, which the corner vertex lies
        // in. The dual's rail centre, in the place the cap-riding reading's outer rail centre
        // stands.
        let rail_centre = axis.point
            + (axis.direction
                * ((corner.corner - axis.point).dot(out_of_the_wall)
                    / along_the_cut_out_of_the_wall));

        // Turn zero is the way out of the cut towards the SHARED face, and the turn ends on the
        // way out towards the cap - so the dual turns the cut strip's own sweep, where the
        // cap-riding reading turns the bead's.
        let towards_the_shared_face = corner.out_of_the_shared_face();
        let towards_the_cap = corner.out_of_the_cap_face;
        let square = towards_the_cap
            - (towards_the_shared_face * towards_the_cap.dot(towards_the_shared_face));

        if !(square.length_squared() > 0.0) {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceCornerNotSupported,
                at_the_corner(),
            ));
        }

        // Math.Clamp passes a NaN through, as f64::clamp does.
        let turn_radians = libm::acos(
            towards_the_shared_face
                .dot(towards_the_cap)
                .clamp(-1.0, 1.0),
        );

        let Some(spine) = SurfaceMixedCornerSpine::try_create(
            axis,
            rail_centre,
            out_of_the_wall,
            along_the_cut_out_of_the_wall,
            true,
            towards_the_shared_face,
            square.get_normal(),
            corner.size,
            turn_radians,
            SurfaceMixedCorner::away_from_the_corner(&corner.cut, corner.corner),
            square_enough,
        ) else {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceMixedCornerBeadRingCrossesNoRail,
                at_the_corner(),
            ));
        };

        Ok(SurfaceMixedCornerDualFrame::new(
            axis,
            rail_centre,
            out_of_the_wall,
            along_the_cut_out_of_the_wall,
            turn_radians,
            spine,
        ))
    }
}

impl SurfaceMixedCornerDualFrame {
    /// The plane the CUT's end ring lies in: through the point the two cylinders touch at,
    /// carrying the whole curve the void ball traces on the cut's tube. Planar because the
    /// contact curve is the spine's own ellipse pulled halfway back to the axis, an affine image of
    /// it; on a square corner it is the cut's own square handover plane.
    pub fn cut_ring_plane(&self) -> SurfaceBandEnd {
        self.spine.tube_ring_plane()
    }

    /// Where the ball touches the WALL at `turn` - the dual's own rail, the mirror of the
    /// cap-riding reading's outer rail.
    pub fn wall_rail_at(&self, turn: f64) -> Vector3 {
        self.spine.t_contact_at(turn)
    }

    /// Where the ball touches the CUT's tube at `turn` - the rail the cut's end ring is cut on.
    pub fn cut_contact_at(&self, turn: f64) -> Vector3 {
        self.spine.bead_contact_at(turn)
    }
}
