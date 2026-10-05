//! Where a mixed corner's RUN-OUT skirt and another chain's BAND land on one another over the same
//! region, and the two footprints are sewn together rather than refused ([`SurfaceRoomCrossing`]).
//! Ports the record of `Bevel/Surface/SurfaceRoomCrossing.cs`; its derived readings and the test
//! that finds one land with the room package.

use super::SurfaceRoomCut;

/// C# `SurfaceRoomCrossing` (a sealed record): a run-out skirt and a band crossing over one region.
///
/// Every other pair `SurfaceRoom.LargestFitting` solves is two cuts facing each other across a
/// region, each the other's far rim, and the two of them share the gap:
/// `size * (k_a + k_b) <= gap`. A skirt over a band is not that. Both footprints are REMOVALS from
/// one face, so where they overlap the face is simply taken twice and what is left is the lower
/// envelope of the two - the band's strip and the run-out's torus trimmed at the curve where they
/// cross and sewn along that crease. Nothing is shrunk to avoid it, because there is nothing to
/// avoid: a chain edge that builds alone builds in company, which is what "Every Chamfer Works"
/// means.
///
/// Two things have to hold before it can be sewn. The run-out's touch row must really REACH the
/// strip - past the band's foot line and before the face itself ends the run-out - and the crossing
/// must stand inside the band's own WINDOW: the stretch of the band's edge the skirt lies along,
/// widened by the mitre overhang the band's material really carries past each end of its edge
/// ([`SurfaceRoomCut::overhang_before_from`]). This says only that the two meet and how far into
/// each other they reach; the curve itself, and the trimming and sewing along it, is the builder's
/// work.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SurfaceRoomCrossing {
    /// The region both footprints land on (`SurfaceBevelPlan.KeyOf`).
    pub region_key: i32,
    /// The band being crossed: its plan, its edge's span and the side it takes off that edge.
    pub band: SurfaceRoomCut,
    /// The run-out's skirt doing the crossing, from the corner it runs out of.
    pub skirt: SurfaceRoomCut,
    /// How far in front of the band's edge line the corner the skirt runs from stands.
    pub gap: f64,
    /// How far the two reach into each other at this size: `size * (k_band + k_skirt) - gap`, which
    /// is exactly the amount the old pair rule would have refused the size over.
    pub overlap: f64,
    /// Whether the band's own END NOTCH stands under this skirt as well (C# default `false`). One
    /// band's removal is its strip on the face it stands on AND the section its end takes out of
    /// the face beyond (`SurfaceRoomEndCut`), so a run-out crossing that band walks off its strip
    /// and onto that section rather than meeting a second footprint there - and the two are sewn as
    /// ONE crossing.
    pub also_takes_the_cap_notch: bool,
}
