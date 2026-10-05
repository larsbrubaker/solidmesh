//! The bevel surface over ONE chain edge, as the lines it is ruled by ([`SurfaceStrip`]). Ports the
//! fields of `Bevel/Surface/SurfaceStrip.cs`; its construction from a plan and its rail readings
//! (`PointOn`, `ParamOf`, `TryCross`) land with the strip's package as further `impl` blocks.

use vector_math::Vector3;

/// C# `SurfaceStrip`: the bevel surface over ONE chain edge, as the lines it is ruled by - one rail
/// per profile sample, each a line parallel to the edge, the first lying on the left tangent line
/// and the last on the right. A chamfer has two rails and a round has one per arc sample.
///
/// **Rails rather than points.** The surface over a straight edge between two planar faces is
/// ruled: every cross section is the same profile, carried along. Saying so in lines rather than in
/// sampled points is what lets the ends be cut by a PLANE - the mitre with the next edge, or the
/// face beyond the corner - exactly, at whatever parameter each rail meets it, instead of only
/// where a cross section happened to be sampled. `SurfaceBevelBuilder` is what turns rails into
/// triangles.
///
/// **No convexity branch.** The arc is swung about the center line the plan found, from the left
/// foot to the right foot the short way round, and that alone puts it where it belongs: the center
/// stands in the wedge the profile is drawn in - the material wedge on a convex edge, the void
/// wedge on a concave one - so the arc always bulges TOWARD the original corner. On a convex edge
/// that leaves the circular segment under each chord as material, which is why a tessellated cut
/// takes a little more than the closed form; on a concave edge the same construction leaves it as
/// void, and the bead adds a little more. One formula, two readings, and
/// `SurfaceEdgePlan.Convexity` read by nothing here.
///
/// **Which way is out.** The bisector of the two INWARD directions the plan used, read outward
/// ([`SurfaceStrip::outward_reference`]). Every sample's own outward normal lies in the sector
/// between the two tangencies, whose half angle is `(180 - wedge) / 2` and so always under 90
/// degrees, so a triangle wound to face that bisector is wound to face out - on a bead as much as
/// on a cut, because a bead's two faces are the ones that bound the void it fills. Taken from the
/// plan's inward directions rather than from the two FACE normals, which are the same bisector
/// wherever nothing was rolled and not where something was: a profile rolled along a curved wall is
/// tangent to facet k rather than to the chain's own face, so the arc sweeps round by the roll's
/// turn as well, and on a narrow wedge - 40 degrees rolled three 9 degree facets - the last samples
/// leave the hemisphere the face normals' bisector names and the strip is wound inside out.
///
/// Shared by everything that reads a strip's rails (the corner patches, the run-out crossings), so
/// those hold it as `Rc<SurfaceStrip>`.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SurfaceStrip {
    /// The point the edge's parameter is measured from: parameter 0 along every rail.
    pub origin: Vector3,
    /// The unit direction of the edge, which every rail runs along.
    pub along: Vector3,
    /// One point of each rail, rail by rail, at the strip's own origin: the profile sample the rail
    /// carries along the edge.
    pub rail_points: Vec<Vector3>,
    /// The angle between one rail and the next, in degrees - the strip's own sweep divided by the
    /// chord count `BevelArcChording.SegmentsFor` gave it, and zero for a chamfer, whose one chord
    /// sweeps no arc at all.
    ///
    /// Read by the MITRE, which pairs the two strips' rails by their ANGLE from the face they
    /// share: two beads of the same radius tangent to one face carry their rail at angle theta in
    /// the same plane parallel to it, so rail theta of one crosses rail theta of the other on the
    /// mitre plane whatever the two wedges are. Where both arcs took one step that pairing is rail
    /// k against rail k; where they did not, it is the UNION of the two sets of angles
    /// (`SurfaceMitreChordedApart`), and this is the number that says which of the two.
    pub rail_step_degrees: f64,
    /// The direction any triangle of this strip must face, to within a right angle: the bisector of
    /// the two face normals.
    pub outward_reference: Vector3,
}
