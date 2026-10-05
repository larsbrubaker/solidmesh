//! Closes a JUNCTION - two cuts and a bead at one vertex - with the single torus patch between the
//! two cuts' end rings, and the sector it takes out of the cap face
//! (`Bevel/Surface/SurfaceMixedJunctionPatch.cs`, a static class). Its nested record lands
//! here, one strip standing at the junction ([`SurfaceMixedJunctionPatchArrival`]), with the
//! closing ring's check ([`the_closing_rail_stands`]); the patch's construction lands with a later
//! package.

use std::rc::Rc;

use vector_math::Vector3;

use super::{SurfaceMixedCorner, SurfaceStrip};

/// C# `SurfaceMixedJunctionPatch.Arrival` (a readonly struct): one strip standing at the junction,
/// as the strip ends hand it over.
#[derive(Debug, Clone)]
pub struct SurfaceMixedJunctionPatchArrival {
    /// The strip itself.
    pub strip: Rc<SurfaceStrip>,
    /// Which edge of the whole build it belongs to, which is how its seams are found.
    pub edge_index: i32,
    /// Whether the corner is that strip's FINISH rather than its start.
    pub ends_here: bool,
}

/// Whether the rail the CLOSING ring is reused for (`closing`) really stands where the turn puts
/// it: the `leading` rail carried the row's whole `sweep`, compared in the two cylindrical terms
/// the sweep did not fix - the radius out of the bead's axis and the height along it - against
/// `slack`, the shared faces' own chord sagitta. Returns the verdict and where the turn puts the
/// leading rail, which is where a refusal is named.
///
/// The angle between the two is zero by construction, because `sweep` IS the difference of their
/// angles, so measuring it would only read the same number back.
pub fn the_closing_rail_stands(
    corner: &SurfaceMixedCorner,
    leading: Vector3,
    closing: Vector3,
    sweep: f64,
    slack: f64,
) -> (bool, Vector3) {
    let axis = corner.axis();
    let turned_to = corner.turned_about(leading, sweep);

    let mine_out_of_the_axis = (turned_to - axis.closest_point_to(turned_to)).length();
    let theirs_out_of_the_axis = (closing - axis.closest_point_to(closing)).length();
    let apart_along_the_axis = (closing - turned_to).dot(axis.direction);
    let outwards = theirs_out_of_the_axis - mine_out_of_the_axis;

    let stands =
        ((outwards * outwards) + (apart_along_the_axis * apart_along_the_axis)).sqrt() <= slack;

    (stands, turned_to)
}
