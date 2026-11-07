//! The two faces a mixed corner's footprint comes off, told apart ([`SurfaceMixedCornerFaces`]).
//! Its record with its constructor and the strips under the blend
//! ([`SurfaceMixedCornerFaces::strips_under_the_blend`]), and its reading off a corner
//! ([`SurfaceMixedCornerFaces::try_read`]).

use std::rc::Rc;

use vector_math::Vector3;

use super::{SurfaceMixedCorner, SurfaceMixedCornerWall};
use crate::bevel::BevelAdvisory;

/// The face the bead ROLLED along, which the strip under the blend is cut from, and the face NOBODY
/// PICKED, which the runout is trimmed against. Most of the time they are one face and this says
/// so - the two fields are then the same `Rc`.
///
/// **Why they are two questions and not one.** The strip under the blend lies on the face the
/// bead's profile is TANGENT to, which is the face it rolled along. The runout's scallop lies on
/// the face nobody picked, because that is the face the blend finally dies into. On a cube face's
/// rim the bead rolls onto the barrel and the barrel is also the face nobody picked. On the
/// BARREL's own rim picked as a face they come apart - the bead rolls along the barrel, the face
/// the two edges SHARE, while the face nobody picked is the flat side of the cube - and asking the
/// rolled face for the runout's trim would trim the blend against the barrel where the ball
/// actually clears the flat face.
///
/// **And then there are TWO strips.** Where the two faces come apart, both of them have a strip
/// under the blend standing; where they are one face there is one strip, taken once, which is the
/// square an L prism has always given up.
#[derive(Debug)]
pub struct SurfaceMixedCornerFaces {
    /// Whether the rolled face and the unpicked face are one face.
    pub one_face: bool,
    /// The face the bead rolled along, read facet by facet: where the strip under the blend is cut.
    pub rolled_section: Rc<SurfaceMixedCornerWall>,
    /// The face nobody picked: where the runout's trim curve and its scallop are read.
    pub unpicked_face: Rc<SurfaceMixedCornerWall>,
}

impl SurfaceMixedCornerFaces {
    /// The wall the round's section rolls on, the unpicked face's, and whether the two are one
    /// face.
    pub fn new(
        rolled: Rc<SurfaceMixedCornerWall>,
        unpicked: Rc<SurfaceMixedCornerWall>,
        one_face: bool,
    ) -> Self {
        Self {
            one_face,
            rolled_section: rolled,
            unpicked_face: unpicked,
        }
    }

    /// Reads both faces of one mixed corner, or says why it cannot - each of them is a run of
    /// facets in the cap's own plane ([`SurfaceMixedCornerWall`]).
    pub fn try_read(corner: &Rc<SurfaceMixedCorner>) -> Result<Self, BevelAdvisory> {
        let unpicked = Rc::new(SurfaceMixedCornerWall::try_read(corner)?);

        if !corner.rolled_is_the_shared_face() {
            // The bead rolled onto the wall nobody picked - or rolled nowhere at all - so one
            // reading is both faces and every bite comes off it exactly as it always did.
            return Ok(Self::new(Rc::clone(&unpicked), unpicked, true));
        }

        let rolled = Rc::new(SurfaceMixedCornerWall::try_read_section(
            corner,
            corner.rolled_section().as_deref(),
            corner.shared_face_foot(),
        )?);

        Ok(Self::new(rolled, unpicked, false))
    }

    /// Every piece of wall the blend stands over between the bead's stop plane and the cap - the
    /// rolled face's, facet by facet, and the unpicked face's as well where that is a face of its
    /// own.
    pub fn strips_under_the_blend(&self) -> Vec<([Vector3; 4], Vector3)> {
        let mut pieces = self.rolled_section.strip_under_the_blend();

        if !self.one_face {
            pieces.extend(self.unpicked_face.strip_under_the_blend());
        }

        pieces
    }
}
