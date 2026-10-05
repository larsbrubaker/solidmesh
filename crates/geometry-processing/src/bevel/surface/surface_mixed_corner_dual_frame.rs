//! The mixed corner's OTHER canal ([`SurfaceMixedCornerDualFrame`]): the bead's VOID ball,
//! rolling along the wall and pressed against the cut's tube. Ports the record of
//! `Bevel/Surface/SurfaceMixedCornerDualFrame.cs`; its reading and its rail accessors land with
//! the mixed-corner package's methods.

use vector_math::Vector3;

use super::{SurfaceLine, SurfaceMixedCornerSpine};

/// C# `SurfaceMixedCornerDualFrame` (a sealed class): the mixed corner's OTHER canal, read - the
/// bead's VOID ball, rolling along the wall and pressed against the cut's tube, where the reading
/// proper carries the material ball riding the cap. Shared by a junction's arm and the rings sewn
/// over it, hence held through `Rc`.
///
/// **The same spine, with its roles swapped.** [`SurfaceMixedCornerSpine`] says where a ball
/// stands that keeps a fixed offset from one plane and a fixed distance from one line; which plane
/// and which line is the caller's to name. This names the WALL as the plane - with the ball a
/// radius OUTSIDE it, since a void ball stands outside the material - and the CUT's own centre
/// line as the axis, and turns from the shared face's outward normal round to the cap's, which is
/// the cut strip's own sweep.
///
/// **Who asks for it, and what is still not known.** Measured against the Onshape control, the
/// oblique corner of `NoseInNotchTestMeshes` IS this canal and not the cap-riding one, to 2.6e-6 mm
/// against 0.48 mm (`SurfaceObliqueMixedCornerWallSpineTests`). `SurfaceTwoCreaseJunction` asks
/// for it by NAME: at a junction of one cut and two creases the blend is this canal by
/// construction. What is NOT known is the rule for a corner where both canals read and only one is
/// right: on a SQUARE corner both span the same two end rings, and no test written on the two
/// faces' PLANES separates them - the one that does is whether the contact rail stays on the face's
/// own EXTENT, which this reading cannot see (`SurfaceMixedCornerDualOnASquareCornerTests`).
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
