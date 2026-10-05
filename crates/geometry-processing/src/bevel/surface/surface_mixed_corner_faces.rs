//! The two faces a mixed corner's footprint comes off, told apart ([`SurfaceMixedCornerFaces`]).
//! Ports the record of `Bevel/Surface/SurfaceMixedCornerFaces.cs`; its reading and the strips
//! under the blend land with the mixed-corner package's methods.

use std::rc::Rc;

use super::SurfaceMixedCornerWall;

/// C# `SurfaceMixedCornerFaces` (a sealed class): the face the bead ROLLED along, which the strip
/// under the blend is cut from, and the face NOBODY PICKED, which the runout is trimmed against.
/// Most of the time they are one face and this says so - the two fields are then the same `Rc`.
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
